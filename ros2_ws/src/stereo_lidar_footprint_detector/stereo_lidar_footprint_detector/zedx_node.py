"""Live ZED X ROS2 adapter for the copied footprint pipeline.

Only the ROS input/output adapter is new. Geometry, depth fusion and YOLO mask
processing are copied from Fordypningsprosjekt-ros-kode.
Without an external camera-to-water TF the footprint uses the original
synthetic camera-right/forward plane, NOT a navigational world frame.
"""
import json
import queue
import threading
import time

import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .backend import PipelineBackend
from .images import image_array


class ZedXFootprintNode(Node):
    def __init__(self):
        super().__init__('zedx_footprint_detector')
        defaults = dict(
            left_topic='/zedx/zed_node/left/image_rect_color',
            depth_topic='/zedx/zed_node/depth/depth_registered',
            camera_info_topic='/zedx/zed_node/left/camera_info',
            output_topic='/obstacles/json',
            weights='/workspace/data/models/yolo26n-seg.engine',
            device='',
            baseline_m=0.120001,
            max_stereo_depth_m=10.0,
            sync_slop_s=0.03,
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.backend = PipelineBackend(
            yolo_weights=self.p['weights'],
            device=self.p['device'] or None,
            max_stereo_depth=self.p['max_stereo_depth_m'],
        )
        self.pub = self.create_publisher(String, self.p['output_topic'], 10)
        self.jobs = queue.Queue(maxsize=1)
        self.stopping = threading.Event()
        self.inputs = [
            message_filters.Subscriber(self, kind, self.p[param],
                                       qos_profile=qos_profile_sensor_data)
            for kind, param in [
                (Image, 'left_topic'), (Image, 'depth_topic'),
                (CameraInfo, 'camera_info_topic')
            ]
        ]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            self.inputs, queue_size=8, slop=self.p['sync_slop_s'])
        self.sync.registerCallback(self.receive_frame)
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()
        self.get_logger().info('ZED X stereo-only footprints; camera-local synthetic plane')

    def receive_frame(self, rgb, depth, info):
        try:
            self.jobs.put_nowait((rgb, depth, info))
        except queue.Full:
            try:
                self.jobs.get_nowait()
            except queue.Empty:
                pass
            try:
                self.jobs.put_nowait((rgb, depth, info))
            except queue.Full:
                pass

    def work(self):
        count = 0
        report_start = time.monotonic()
        while not self.stopping.is_set():
            try:
                rgb, depth_msg, info = self.jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            t0 = time.perf_counter()
            try:
                if not (rgb.header.frame_id == depth_msg.header.frame_id == info.header.frame_id):
                    raise ValueError('RGB, depth and CameraInfo frames differ')
                if depth_msg.encoding != '32FC1':
                    raise ValueError('Expected registered 32FC1 depth (meters)')
                rgb_array = image_array(rgb)
                depth_array = image_array(depth_msg)
                if depth_array.shape != rgb_array.shape[:2] or (info.height, info.width) != rgb_array.shape[:2]:
                    raise ValueError('RGB/depth/camera_info dimensions disagree')
                if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
                    raise ValueError('Unexpected ROI or binning')
                k = np.asarray(info.p, dtype=float).reshape(3, 4)[:, :3]
                self.backend.configure(k, self.p['baseline_m'], rgb_array.shape[:2])
                stamp_ns = rgb.header.stamp.sec * 1000000000 + rgb.header.stamp.nanosec
                result = self.backend.run(
                    timestamp=stamp_ns,
                    left_img=rgb_array,
                    disparity_img=None,
                    depth_img=depth_array,
                    lidar_uv=np.empty((0, 2)),
                    lidar_xyz=np.empty((0, 3)),
                    camera_to_water=None,
                )
                frame_id = rgb.header.frame_id + '/right_forward_plane'
                collection = self.backend.outputs.obstacle_collection(
                    result.obstacles, stamp_ns, frame_id)
                self.pub.publish(String(data=json.dumps(collection, allow_nan=False)))
                count += 1
                elapsed = time.monotonic() - report_start
                if elapsed >= 5:
                    self.get_logger().info(
                        f'Obstacles: {count/elapsed:.2f} Hz | last: '
                        f'{len(result.obstacles["features"])} footprints | '
                        f'pipeline {result.runtime_ms:.1f} ms | '
                        f'end-to-end worker {1000*(time.perf_counter()-t0):.1f} ms')
                    count = 0
                    report_start = time.monotonic()
            except Exception as exc:
                self.get_logger().error(f'Footprint frame rejected: {exc}')

    def destroy_node(self):
        self.stopping.set()
        self.worker.join(timeout=3)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ZedXFootprintNode()
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
