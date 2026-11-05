"""Publish Foxy /clock using recorded PoseStamped header timestamps.

Foxy rosbag2 play does not implement --clock. Its default playback is paced
by recorded bag arrival times; this relay supplies the corresponding message
header stamps to the existing SVO player. This is approximate simulated time,
not a replacement for a proper timestamp-aware rosbag clock.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


class PoseClockRelay(Node):
    def __init__(self):
        super().__init__('ma2_pose_clock_relay')
        self.declare_parameter('pose_topic', '/navigation/pose')
        topic = self.get_parameter('pose_topic').value
        self.publisher = self.create_publisher(Clock, '/clock', 10)
        self.last_stamp_ns = -1
        self.sub = self.create_subscription(
            PoseStamped, topic, self.on_pose, qos_profile_sensor_data)
        self.get_logger().info(
            f'Publishing /clock from recorded {topic} header timestamps; '
            'start this node before ros2 bag play.')

    def on_pose(self, pose):
        stamp = pose.header.stamp
        ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
        if ns <= 0:
            self.get_logger().warning(
                'Ignoring zero pose timestamp', throttle_duration_sec=5.0)
            return
        if ns < self.last_stamp_ns:
            self.get_logger().warning(
                'Pose timestamp went backwards; did rosbag playback restart?')
        self.last_stamp_ns = ns
        msg = Clock()
        msg.clock = stamp
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseClockRelay()
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
