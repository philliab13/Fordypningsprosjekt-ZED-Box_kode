"""ROS 2 adapter for original YOLO, stereo/LiDAR fusion, and footprint pipeline.

The original geometry and segmentation algorithms are in detection/.
No MA2-specific hardcoded sensor calibration is used: TF provides all extrinsics.
"""
from collections import deque
import json
import queue
import threading
import time

import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .backend import PipelineBackend
from .images import image_array
from .sensor_helpers import cloud_xyz, nearest_cloud, project_lidar, stamp_ns, transform_matrix


class ZedXFootprintNode(Node):
    def __init__(self):
        super().__init__('zedx_footprint_detector')
        defaults = dict(
            left_topic='/zedx/zed_node/left/image_rect_color',
            depth_topic='/zedx/zed_node/depth/depth_registered',
            camera_info_topic='/zedx/zed_node/left/camera_info',
            lidar_topic='/lidar_aft/points',
            output_topic='/obstacles/json',
            weights='/workspace/data/models/yolo26n-seg.engine',
            device='',
            baseline_m=0.120001,
            max_stereo_depth_m=10.0,
            sync_slop_s=0.03,
            enable_lidar=False,
            require_lidar=False,
            lidar_max_skew_s=0.1,
            fixed_frame='',
            water_frame='',
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.backend = PipelineBackend(
            yolo_weights=self.p['weights'],
            device=self.p['device'] or None,
            max_stereo_depth=self.p['max_stereo_depth_m'],
        )
        self.publisher = self.create_publisher(String, self.p['output_topic'], 10)
        self.jobs = queue.Queue(maxsize=1)
        self.stopping = threading.Event()
        self.clouds = deque(maxlen=100)
        self.cloud_lock = threading.Lock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.p['enable_lidar']:
            self.cloud_sub = self.create_subscription(
                PointCloud2, self.p['lidar_topic'], self.receive_cloud,
                qos_profile_sensor_data)
        self.inputs = [
            message_filters.Subscriber(self, kind, self.p[param],
                                       qos_profile=qos_profile_sensor_data)
            for kind, param in [
                (Image, 'left_topic'), (Image, 'depth_topic'),
                (CameraInfo, 'camera_info_topic')
            ]
        ]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            self.inputs, queue_size=8, slop=float(self.p['sync_slop_s']))
        self.sync.registerCallback(self.receive_frame)
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()
        self.get_logger().info(
            'Footprints: stereo=%s LiDAR=%s water_frame=%r fixed_frame=%r'
            % (True, self.p['enable_lidar'], self.p['water_frame'],
               self.p['fixed_frame']))

    def receive_cloud(self, msg):
        with self.cloud_lock:
            self.clouds.append(msg)

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

    def lookup(self, target, source, stamp):
        return transform_matrix(self.tf_buffer.lookup_transform(
            target, source, Time.from_msg(stamp),
            timeout=Duration(seconds=0.0)).transform)

    def lidar_for_frame(self, rgb, k):
        if not self.p['enable_lidar']:
            return np.empty((0, 2)), np.empty((0, 3))
        with self.cloud_lock:
            cloud = nearest_cloud(tuple(self.clouds), stamp_ns(rgb),
                                  float(self.p['lidar_max_skew_s']))
        if cloud is None:
            if self.p['require_lidar']:
                raise ValueError('No LiDAR scan near camera timestamp')
            return np.empty((0, 2)), np.empty((0, 3))
        if not cloud.header.frame_id:
            raise ValueError('LiDAR cloud missing frame_id')
        camera_stamp = Time.from_msg(rgb.header.stamp)
        lidar_stamp = Time.from_msg(cloud.header.stamp)
        if self.p['fixed_frame']:
            camera_from_lidar = transform_matrix(
                self.tf_buffer.lookup_transform_full(
                    rgb.header.frame_id, camera_stamp,
                    cloud.header.frame_id, lidar_stamp,
                    self.p['fixed_frame'],
                    timeout=Duration(seconds=0.0)).transform)
        else:
            # Without a fixed frame, simultaneous TF is only an approximation
            # for moving platforms; prefer fixed_frame for recorded data.
            camera_from_lidar = self.lookup(
                rgb.header.frame_id, cloud.header.frame_id,
                rgb.header.stamp)
        points = cloud_xyz(cloud)
        return project_lidar(points, camera_from_lidar, k,
                             (rgb.height, rgb.width))

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
                if not (rgb.header.frame_id == depth_msg.header.frame_id ==
                        info.header.frame_id):
                    raise ValueError('RGB/depth/CameraInfo optical frames differ')
                if depth_msg.encoding != '32FC1':
                    raise ValueError('Expected registered 32FC1 depth in metres')
                rgb_array = image_array(rgb)
                depth_array = image_array(depth_msg)
                if (depth_array.shape != rgb_array.shape[:2]
                        or (info.height, info.width) != rgb_array.shape[:2]):
                    raise ValueError('RGB, depth and CameraInfo size mismatch')
                if (info.binning_x > 1 or info.binning_y > 1
                        or info.roi.x_offset or info.roi.y_offset):
                    raise ValueError('Unexpected binning/ROI')
                k = np.asarray(info.p, dtype=float).reshape(3, 4)[:, :3]
                self.backend.configure(k, float(self.p['baseline_m']),
                                       rgb_array.shape[:2])
                uv, xyz = self.lidar_for_frame(rgb, k)
                water_frame = self.p['water_frame']
                transform = (self.lookup(water_frame, rgb.header.frame_id,
                                         rgb.header.stamp)
                             if water_frame else None)
                timestamp = stamp_ns(rgb)
                result = self.backend.run(
                    timestamp=timestamp,
                    left_img=rgb_array,
                    disparity_img=None,
                    depth_img=depth_array,
                    lidar_uv=uv,
                    lidar_xyz=xyz,
                    camera_to_water=transform,
                )
                frame_id = water_frame or rgb.header.frame_id + '/right_forward_plane'
                collection = self.backend.outputs.obstacle_collection(
                    result.obstacles, timestamp, frame_id)
                self.publisher.publish(
                    String(data=json.dumps(collection, allow_nan=False)))
                count += 1
                elapsed = time.monotonic() - report_start
                if elapsed >= 5:
                    self.get_logger().info(
                        f'Obstacles: {count/elapsed:.2f} Hz | '
                        f'last: {len(result.obstacles["features"])} footprints | '
                        f'LiDAR pixels {len(xyz)} | '
                        f'pipeline {result.runtime_ms:.1f} ms | '
                        f'worker {1000*(time.perf_counter()-t0):.1f} ms')
                    count = 0
                    report_start = time.monotonic()
            except (ValueError, TransformException, RuntimeError) as exc:
                self.get_logger().error(f'Footprint frame rejected: {exc}')
            except Exception as exc:
                self.get_logger().error(f'Unexpected footprint error: {exc}')

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
