"""ZED SDK 4.0 + TensorRT ROS 2 Foxy publisher.

Follows the standard camera/depth message conventions used by Stereolabs'
official zed-ros2-wrapper, but intentionally owns the ZED camera in one
process so YOLO TensorRT and depth share a frame and acquisition timestamp.
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String


def stamp_from_ns(msg_header, timestamp_ns, frame_id):
    msg_header.stamp.sec = int(timestamp_ns // 1_000_000_000)
    msg_header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
    msg_header.frame_id = frame_id


def make_image(array, encoding, timestamp_ns, frame_id):
    array = np.ascontiguousarray(array)
    msg = Image()
    stamp_from_ns(msg.header, timestamp_ns, frame_id)
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = int(array.strides[0])
    msg.data = array.tobytes()
    return msg


def make_camera_info(cam, width, height, timestamp_ns, frame_id,
                     native_width, native_height, right_tx=0.0):
    # ZED camera intrinsics are expressed at the acquisition resolution.
    # Scale K and P for SDK-downscaled rectified images/depth.
    sx = width / native_width
    sy = height / native_height
    fx, fy = float(cam.fx) * sx, float(cam.fy) * sy
    cx, cy = float(cam.cx) * sx, float(cam.cy) * sy
    msg = CameraInfo()
    stamp_from_ns(msg.header, timestamp_ns, frame_id)
    msg.height, msg.width = height, width
    msg.distortion_model = 'plumb_bob'
    msg.d = [float(x) for x in cam.disto]
    msg.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    msg.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    msg.p = [fx, 0., cx, float(right_tx) * sx, 0., fy, cy, 0., 0., 0., 1., 0.]
    return msg


class ZedPerceptionNode(Node):
    def __init__(self):
        super().__init__('zed_box_perception')

        def param(name, value):
            self.declare_parameter(name, value)
            return self.get_parameter(name).value

        self.source = str(param('source', 'live')).lower()
        self.svo_path = str(param('svo_path', ''))
        self.engine_path = str(param('engine_path', '/workspace/data/models/yolo26n-seg.engine'))
        self.enable_yolo = bool(param('enable_yolo', True))
        self.publish_rgb = bool(param('publish_rgb', True))
        self.publish_right = bool(param('publish_right', True))
        self.publish_depth = bool(param('publish_depth', True))
        self.publish_imu = bool(param('publish_imu', True))
        self.camera_fps = int(param('camera_fps', 15))
        self.pub_resolution = str(param('pub_resolution', 'CUSTOM')).upper()
        self.pub_downscale_factor = float(param('pub_downscale_factor', 2.0))
        self.pub_frame_rate = float(param('pub_frame_rate', 0.0))
        self.depth_mode = str(param('depth_mode', 'PERFORMANCE')).upper()
        self.max_depth_m = float(param('max_depth_m', 20.0))
        self.yolo_conf = float(param('yolo_conf', 0.25))
        self.yolo_iou = float(param('yolo_iou', 0.5))
        self.yolo_size = int(param('yolo_size', 640))
        self.left_frame = str(param('left_optical_frame', 'zed_left_camera_optical_frame'))
        self.right_frame = str(param('right_optical_frame', 'zed_right_camera_optical_frame'))
        self.imu_frame = str(param('imu_frame', 'zed_imu_link'))

        qos = qos_profile_sensor_data
        self.pubs = {
            'left': self.create_publisher(Image, '/zed/left/image_rect_color', qos),
            'right': self.create_publisher(Image, '/zed/right/image_rect_color', qos),
            'depth': self.create_publisher(Image, '/zed/depth/depth_registered', qos),
            'left_info': self.create_publisher(CameraInfo, '/zed/left/camera_info', qos),
            'right_info': self.create_publisher(CameraInfo, '/zed/right/camera_info', qos),
            'instances': self.create_publisher(Image, '/zed/perception/instance_ids', qos),
            'detections': self.create_publisher(String, '/zed/perception/detections_json', qos),
            'imu': self.create_publisher(Imu, '/zed/imu/data_raw', qos),
        }

        self.zed = sl.Camera()
        init = sl.InitParameters()
        if self.source == 'svo':
            if not Path(self.svo_path).is_file():
                raise FileNotFoundError(self.svo_path)
            init.set_from_svo_file(self.svo_path)
            init.svo_real_time_mode = False
        elif self.source != 'live':
            raise ValueError('source must be live or svo')
        else:
            init.camera_resolution = sl.RESOLUTION.HD1080
            init.camera_fps = self.camera_fps

        init.coordinate_units = sl.UNIT.METER
        init.depth_minimum_distance = 0.3
        init.depth_maximum_distance = self.max_depth_m
        init.depth_mode = getattr(sl.DEPTH_MODE, self.depth_mode)
        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError('ZED open failed: {}'.format(status))

        self.left = sl.Mat()
        self.left_native = sl.Mat()  # Full-resolution image for local TensorRT inference
        self.right = sl.Mat()
        self.depth = sl.Mat()
        self.runtime = sl.RuntimeParameters()
        self.sensors = sl.SensorsData()
        self.model = None
        if self.enable_yolo:
            from ultralytics import YOLO
            if not Path(self.engine_path).is_file():
                raise FileNotFoundError(self.engine_path)
            self.model = YOLO(self.engine_path, task='segment')

        cfg = self.zed.get_camera_information().camera_configuration
        cal = cfg.calibration_parameters
        self.cal_left = cal.left_cam
        self.cal_right = cal.right_cam
        self.baseline_m = abs(float(cal.stereo_transform.get_translation().get()[0]))
        self.width = int(cfg.resolution.width)
        self.height = int(cfg.resolution.height)
        if self.pub_resolution not in ('NATIVE', 'CUSTOM'):
            raise ValueError("pub_resolution must be NATIVE or CUSTOM")
        if self.pub_downscale_factor < 1.0:
            raise ValueError("pub_downscale_factor must be >= 1.0")
        if self.pub_frame_rate < 0 or (
            self.source == 'live' and self.pub_frame_rate > self.camera_fps
        ):
            raise ValueError("pub_frame_rate must be 0 or between 0 and camera_fps")
        factor = self.pub_downscale_factor if self.pub_resolution == 'CUSTOM' else 1.0
        # Even dimensions match the SDK's usual image output constraints.
        self.pub_width = max(2, int(self.width / factor) // 2 * 2)
        self.pub_height = max(2, int(self.height / factor) // 2 * 2)
        self.pub_size = sl.Resolution(self.pub_width, self.pub_height)
        self.last_visual_publish = None
        self.get_logger().info(
            'ZED opened %dx%d; publishing %dx%d at %s FPS; baseline %.4f m; YOLO=%s' %
            (self.width, self.height, self.pub_width, self.pub_height,
             'grab rate' if self.pub_frame_rate == 0 else str(self.pub_frame_rate),
             self.baseline_m, self.enable_yolo)
        )
        self.nframes = 0
        self.last_report = time.monotonic()
        # Single processing callback: all outputs derive from the same grab().
        self.timer = self.create_timer(0.001, self.process)

    def process(self):
        status = self.zed.grab(self.runtime)
        if status == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            self.get_logger().info('End of SVO')
            self.timer.cancel()
            return
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warning('ZED grab: {}'.format(status))
            return

        timestamp_ns = int(self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
        now = time.monotonic()
        publish_visual = (
            self.pub_frame_rate == 0
            or self.last_visual_publish is None
            or now - self.last_visual_publish >= 1.0 / self.pub_frame_rate
        )
        if publish_visual:
            self.last_visual_publish = now

        # Keep full-resolution imagery local for YOLO. Retrieve the published
        # imagery separately at lower resolution directly through the ZED SDK.
        bgr = None
        if self.model is not None:
            self.zed.retrieve_image(self.left_native, sl.VIEW.LEFT)
            bgr = cv2.cvtColor(
                np.asarray(self.left_native.get_data()), cv2.COLOR_BGRA2BGR
            )

        if publish_visual:
            if self.publish_rgb:
                self.zed.retrieve_image(
                    self.left, sl.VIEW.LEFT, sl.MEM.CPU, self.pub_size
                )
                left_bgr = cv2.cvtColor(
                    np.asarray(self.left.get_data()), cv2.COLOR_BGRA2BGR
                )
                self.pubs['left'].publish(
                    make_image(left_bgr, 'bgr8', timestamp_ns, self.left_frame)
                )
            self.pubs['left_info'].publish(
                make_camera_info(
                    self.cal_left, self.pub_width, self.pub_height, timestamp_ns,
                    self.left_frame, self.width, self.height
                )
            )

            if self.publish_right:
                self.zed.retrieve_image(
                    self.right, sl.VIEW.RIGHT, sl.MEM.CPU, self.pub_size
                )
                right_bgr = cv2.cvtColor(
                    np.asarray(self.right.get_data()), cv2.COLOR_BGRA2BGR
                )
                self.pubs['right'].publish(
                    make_image(right_bgr, 'bgr8', timestamp_ns, self.right_frame)
                )
            # Right projection Tx = -fx * baseline at native resolution,
            # then make_camera_info scales it to the published width.
            self.pubs['right_info'].publish(
                make_camera_info(
                    self.cal_right, self.pub_width, self.pub_height, timestamp_ns,
                    self.right_frame, self.width, self.height,
                    right_tx=-float(self.cal_right.fx) * self.baseline_m
                )
            )

            if self.publish_depth:
                self.zed.retrieve_measure(
                    self.depth, sl.MEASURE.DEPTH, sl.MEM.CPU, self.pub_size
                )
                depths = np.array(
                    self.depth.get_data(), dtype=np.float32, copy=True
                )
                depths[~np.isfinite(depths)] = np.nan
                self.pubs['depth'].publish(
                    make_image(depths, '32FC1', timestamp_ns, self.left_frame)
                )

        if self.model is not None:
            # Infer on native ZED data even if ROS output is downscaled or capped.
            out = self.model.predict(
                source=bgr, imgsz=self.yolo_size, conf=self.yolo_conf,
                iou=self.yolo_iou, classes=[8], retina_masks=False, verbose=False
            )[0]
            if publish_visual:
                instances = np.zeros(
                    (self.pub_height, self.pub_width), dtype=np.uint16
                )
                objects = []
                sx = self.pub_width / self.width
                sy = self.pub_height / self.height
                if out.boxes is not None and out.masks is not None:
                    boxes = out.boxes.xyxy.cpu().numpy()
                    scores = out.boxes.conf.cpu().numpy()
                    classes = out.boxes.cls.cpu().numpy()
                    for idx, (xyxy, score, cls, polygon) in enumerate(
                        zip(boxes, scores, classes, out.masks.xy), start=1
                    ):
                        if idx > 65535:
                            break
                        poly = np.round(
                            np.asarray(polygon) * [sx, sy]
                        ).astype(np.int32)
                        if len(poly) >= 3:
                            cv2.fillPoly(instances, [poly], int(idx))
                        objects.append({
                            'id': idx, 'class_id': int(cls),
                            'confidence': float(score),
                            'bbox_xyxy': [
                                float(xyxy[0] * sx), float(xyxy[1] * sy),
                                float(xyxy[2] * sx), float(xyxy[3] * sy)
                            ]
                        })
                self.pubs['instances'].publish(
                    make_image(instances, '16UC1', timestamp_ns, self.left_frame)
                )
                det = String()
                det.data = json.dumps({
                    'stamp_ns': timestamp_ns, 'frame_id': self.left_frame,
                    'width': self.pub_width, 'height': self.pub_height,
                    'objects': objects
                }, separators=(',', ':'))
                self.pubs['detections'].publish(det)

        if self.publish_imu:
            self.publish_imu_measurement()

        self.nframes += 1
        now = time.monotonic()
        if now - self.last_report >= 10:
            self.get_logger().info('processed %d frames' % self.nframes)
            self.last_report = now

    def publish_imu_measurement(self):
        # Sensor readings have their own timestamps; do NOT stamp them with
        # camera-frame timestamps. Sensor orientation isn't asserted here.
        if self.zed.get_sensors_data(self.sensors, sl.TIME_REFERENCE.CURRENT) != sl.ERROR_CODE.SUCCESS:
            return
        imu_data = self.sensors.get_imu_data()
        ts_ns = int(imu_data.timestamp.get_nanoseconds())
        msg = Imu()
        stamp_from_ns(msg.header, ts_ns, self.imu_frame)
        msg.orientation_covariance[0] = -1.0
        a = imu_data.get_linear_acceleration()
        g = imu_data.get_angular_velocity()
        # ZED SDK linear acceleration m/s^2; angular velocity degrees/s
        # with SDK 4.x defaults. Convert deg/s to rad/s.
        msg.linear_acceleration.x = float(a[0])
        msg.linear_acceleration.y = float(a[1])
        msg.linear_acceleration.z = float(a[2])
        msg.angular_velocity.x = float(np.deg2rad(g[0]))
        msg.angular_velocity.y = float(np.deg2rad(g[1]))
        msg.angular_velocity.z = float(np.deg2rad(g[2]))
        self.pubs['imu'].publish(msg)

    def destroy_node(self):
        self.zed.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ZedPerceptionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
