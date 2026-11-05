from pathlib import Path

from .scenario import load_scenario

import numpy as np
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image


def numpy_to_image_msg(array, encoding, stamp, frame_id):
    array = np.ascontiguousarray(array)

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    msg.height = array.shape[0]
    msg.width = array.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = False

    if array.ndim == 3:
        msg.step = (
            array.shape[1]
            * array.shape[2]
            * array.dtype.itemsize
        )
    else:
        msg.step = (
            array.shape[1]
            * array.dtype.itemsize
        )

    msg.data = array.tobytes()

    return msg


class MA2SVOPlayer(Node):

    def __init__(self):
        super().__init__('ma2_svo_player')

        self.declare_parameter('scenario', 'scen1')
        # One-time startup seek to match the first /clock timestamp.
        # 0 disables seeking and preserves original full-SVO playback.
        self.declare_parameter('start_ns', 0)
        self.declare_parameter(
            'scenario_dir',
            str(Path.home() / 'Documents/Stixel_based_free_space_estimation/'
                'free-space-estimation/configs/scenarios'))
        self.declare_parameter(
            'data_root',
            str(Path.home() / 'Documents/Data_For_Stixel_based_Free_space_estimation/'
                '2023-07-11_Multi_ZED_Summer'))
        config_path, config, self.input_path, self.time_offset_ns = load_scenario(
            self.get_parameter('scenario').value,
            self.get_parameter('scenario_dir').value,
            self.get_parameter('data_root').value)
        self.get_logger().info(
            f"Scenario: {config.get('name', config_path.stem)} ({config_path}) | "
            f"SVO: {self.input_path} | ROS bag: {config.get('rosbag', 'unspecified')}")

        self.get_logger().info(
            f"MA2-SVO time offset: "
            f"{self.time_offset_ns} ns "
            f"({self.time_offset_ns / 1e9:.6f} s)"
        )

        # --------------------------------------------------
        # Publishers
        # --------------------------------------------------

        self.left_pub = self.create_publisher(
            Image,
            '/ma2/zed/left/image',
            10
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/ma2/zed/depth',
            10
        )

        self.disparity_pub = self.create_publisher(
            Image,
            '/ma2/zed/disparity',
            10
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/ma2/zed/camera_info',
            10
        )

        # --------------------------------------------------
        # ZED SVO setup
        # --------------------------------------------------

        self.zed = sl.Camera()

        init_parameters = sl.InitParameters()

        init_parameters.set_from_svo_file(
            self.input_path
        )

        init_parameters.depth_mode = (
            sl.DEPTH_MODE.PERFORMANCE
        )

        init_parameters.coordinate_units = (
            sl.UNIT.METER
        )

        init_parameters.depth_minimum_distance = 0
        init_parameters.depth_maximum_distance = 60

        # Important for offline SVO playback:
        # do not intentionally drop frames to keep real time.
        init_parameters.svo_real_time_mode = False

        err = self.zed.open(init_parameters)

        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(
                f"Could not open SVO: {err}"
            )

        self.runtime_parameters = (
            sl.RuntimeParameters()
        )

        self.left_mat = sl.Mat()
        self.depth_mat = sl.Mat()
        self.disparity_mat = sl.Mat()

        # --------------------------------------------------
        # Camera calibration
        # --------------------------------------------------

        camera_information = (
            self.zed.get_camera_information()
        )

        camera_configuration = (
            camera_information
            .camera_configuration
        )

        calibration = (
            camera_configuration
            .calibration_parameters
            .left_cam
        )

        resolution = (
            camera_configuration.resolution
        )

        self.frame_id = 'ma2_zed_left_optical'

        self.camera_info_msg = CameraInfo()

        self.camera_info_msg.width = (
            resolution.width
        )

        self.camera_info_msg.height = (
            resolution.height
        )

        self.camera_info_msg.distortion_model = (
            'plumb_bob'
        )

        self.camera_info_msg.d = [
            float(x)
            for x in calibration.disto
        ]

        self.camera_info_msg.k = [
            float(calibration.fx), 0.0,
            float(calibration.cx),

            0.0, float(calibration.fy),
            float(calibration.cy),

            0.0, 0.0, 1.0
        ]

        self.camera_info_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        self.camera_info_msg.p = [
            float(calibration.fx), 0.0,
            float(calibration.cx), 0.0,

            0.0, float(calibration.fy),
            float(calibration.cy), 0.0,

            0.0, 0.0, 1.0, 0.0
        ]

        self.get_logger().info(
            f"Opened ZED SVO: "
            f"{resolution.width}x{resolution.height}, "
            f"{camera_configuration.fps:.1f} FPS"
        )

        # --------------------------------------------------
        # Playback timer
        # --------------------------------------------------

        self.next_frame_ready = False
        self.next_left = None
        self.next_depth = None
        self.next_disparity = None
        self.next_stamp_ns = None

        clock_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            clock_qos
        )

        self.load_next_frame()
        start_ns = int(self.get_parameter('start_ns').value)
        if start_ns > 0 and self.next_frame_ready:
            self.seek_to_start(start_ns)

    def seek_to_start(self, start_ns):
        """Seek once by recorded SVO timestamps, not nominal camera FPS.

        The file reports a nominal 15 FPS but may contain substantial
        timestamp gaps. Using fps * duration can therefore overshoot
        the rosbag start by tens of seconds. Binary-search the frames
        using their actual acquisition timestamps, adjusted by claps.
        After searching, decode exactly one candidate frame with the
        existing image/depth retrieval pipeline.
        """
        if self.next_stamp_ns >= start_ns:
            self.get_logger().info(
                'SVO first frame is at/after bag start; no initial seek')
            return
        frame_count = int(self.zed.get_svo_number_of_frames())
        if frame_count < 1:
            raise RuntimeError('SVO contains no frames')

        # Find the LAST SVO frame whose corrected timestamp is <= the
        # requested bag start, so we do not silently skip over the
        # beginning of the requested recording.
        low, high = 0, frame_count - 1
        best_frame = None
        best_stamp = None
        probes = 0
        while low <= high:
            middle = (low + high) // 2
            status = self.zed.set_svo_position(middle)
            if status is not None and status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(
                    f'SVO seek to frame {middle} failed: {status}')
            grab_status = self.zed.grab(self.runtime_parameters)
            if grab_status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(
                    f'SVO timestamp probe failed at frame {middle}: '
                    f'{grab_status}')
            timestamp = int(
                self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).data_ns
            ) + self.time_offset_ns
            probes += 1
            if timestamp <= start_ns:
                best_frame, best_stamp = middle, timestamp
                low = middle + 1
            else:
                high = middle - 1

        target_frame = 0 if best_frame is None else best_frame
        self.get_logger().info(
            f'Initial SVO timestamp search: target frame '
            f'{target_frame}/{frame_count}, probes={probes}, '
            f'candidate_stamp={best_stamp}, bag_start={start_ns}')

        status = self.zed.set_svo_position(target_frame)
        if status is not None and status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(
                f'SVO final seek to frame {target_frame} failed: {status}')
        self.load_next_frame()
        if not self.next_frame_ready:
            raise RuntimeError('No SVO frame after final seek')
        delta_ms = (start_ns - self.next_stamp_ns) / 1e6
        self.get_logger().info(
            f'Initial seek complete: next_svo={self.next_stamp_ns}, '
            f'bag_start={start_ns}, diff_ms={delta_ms:.1f}')
        if abs(delta_ms) > 2000:
            self.get_logger().warning(
                'SVO frame nearest bag start is over 2 seconds away. '
                'Check SVO timestamps or sparse recording sections.')

    def load_next_frame(self):
        while True:
            grab_status = self.zed.grab(
                self.runtime_parameters
            )

            if grab_status == sl.ERROR_CODE.SUCCESS:

                self.zed.retrieve_image(
                    self.left_mat,
                    sl.VIEW.LEFT
                )

                self.zed.retrieve_measure(
                    self.depth_mat,
                    sl.MEASURE.DEPTH
                )

                self.zed.retrieve_measure(
                    self.disparity_mat,
                    sl.MEASURE.DISPARITY
                )

                left = np.ascontiguousarray(
                    self.left_mat.get_data()[:, :, :3]
                )

                depth = (
                    self.depth_mat
                    .get_data()
                    .copy()
                    .astype(np.float32)
                )

                disparity = (
                    -self.disparity_mat
                    .get_data()
                    .copy()
                    .astype(np.float32)
                )

                depth = np.nan_to_num(
                    depth,
                    nan=0.0,
                    posinf=0.0,
                    neginf=0.0
                )

                disparity[
                    ~np.isfinite(disparity)
                ] = 0.0

                svo_timestamp_ns = (
                    self.zed.get_timestamp(
                        sl.TIME_REFERENCE.IMAGE
                    ).data_ns
                )

                synced_timestamp_ns = int(
                    svo_timestamp_ns
                    + self.time_offset_ns
                )

                self.next_left = left
                self.next_depth = depth
                self.next_disparity = disparity
                self.next_stamp_ns = synced_timestamp_ns
                self.next_frame_ready = True

                return

            elif grab_status == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:

                self.get_logger().info(
                    "End of SVO reached"
                )

                self.next_frame_ready = False
                return

            else:

                self.get_logger().warning(
                    f"Skipping bad SVO frame: {grab_status}"
                )


    def clock_callback(self, msg):
        clock_ns = (
            int(msg.clock.sec) * 1_000_000_000
            + int(msg.clock.nanosec)
        )
        if self.next_frame_ready:
            diff_ms = (clock_ns - self.next_stamp_ns) / 1e6

            self.get_logger().info(
                f"clock={clock_ns} | "
                f"next_svo={self.next_stamp_ns} | "
                f"diff={diff_ms:.1f} ms",
                throttle_duration_sec=1.0
            )

        while (
            self.next_frame_ready
            and self.next_stamp_ns <= clock_ns
        ):
            self.publish_loaded_frame()
            self.load_next_frame()

    def publish_loaded_frame(self):
        stamp = Time(
            nanoseconds=self.next_stamp_ns
        ).to_msg()

        left_msg = numpy_to_image_msg(
            self.next_left,
            'bgr8',
            stamp,
            self.frame_id
        )

        depth_msg = numpy_to_image_msg(
            self.next_depth,
            '32FC1',
            stamp,
            self.frame_id
        )

        disparity_msg = numpy_to_image_msg(
            self.next_disparity,
            '32FC1',
            stamp,
            self.frame_id
        )

        self.camera_info_msg.header.stamp = stamp
        self.camera_info_msg.header.frame_id = (
            self.frame_id
        )

        self.left_pub.publish(left_msg)
        self.depth_pub.publish(depth_msg)
        self.disparity_pub.publish(disparity_msg)
        self.camera_info_pub.publish(
            self.camera_info_msg
        )

    def destroy_node(self):
        if hasattr(self, 'zed'):
            self.zed.close()

        return super().destroy_node()



def main(args=None):

    rclpy.init(args=args)

    node = MA2SVOPlayer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()