"""Clock-synchronized, rate-controlled MA2 SQLite rosbag2 playback (ROS2 Foxy).

Reads the decompressed SQLite .db3 file. Uses the same recorded bag timestamps
for /clock and topic scheduling, unlike the old pose-header clock relay.

Limitations: single-process Python publisher, not a deterministic hard realtime
player. This is a development/replay tool, not a production rosbag2 replacement.
"""
import sqlite3
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock

DEFAULT_TOPICS = (
    '/lidar_aft/points',
    '/navigation/pose',
    '/tf',
    '/tf_static',
)


def to_clock_msg(nanoseconds):
    msg = Clock()
    msg.clock.sec = int(nanoseconds // 1_000_000_000)
    msg.clock.nanosec = int(nanoseconds % 1_000_000_000)
    return msg


class MA2BagPlayer(Node):
    def __init__(self):
        super().__init__('ma2_bag_player')
        self.declare_parameter(
            'bag_file',
            '/opt/zed_official_ws/datasets/scen4_2/scen4_2_uncompressed.db3')
        self.declare_parameter('rate', 0.01)
        self.declare_parameter('clock_hz', 20.0)
        self.declare_parameter('start_ns', 0)
        self.declare_parameter('duration_sec', 0.0)
        self.declare_parameter('topics', list(DEFAULT_TOPICS))
        self.declare_parameter('startup_delay_sec', 2.0)

        self.rate = float(self.get_parameter('rate').value)
        self.clock_hz = float(self.get_parameter('clock_hz').value)
        self.start_ns = int(self.get_parameter('start_ns').value)
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.startup_delay_sec = float(
            self.get_parameter('startup_delay_sec').value)
        topics = list(self.get_parameter('topics').value)
        if self.rate <= 0 or self.clock_hz <= 0:
            raise ValueError('rate and clock_hz must be > 0')
        if self.startup_delay_sec < 0 or self.duration_sec < 0:
            raise ValueError('startup_delay_sec and duration_sec must be >= 0')
        if not topics:
            raise ValueError('topics must be nonempty')

        bag = Path(self.get_parameter('bag_file').value)
        if not bag.is_file():
            raise FileNotFoundError(str(bag))
        self.connection = sqlite3.connect(
            'file:' + str(bag) + '?mode=ro', uri=True)
        topic_rows = self.connection.execute(
            'SELECT id, name, type FROM topics').fetchall()
        chosen = {
            int(topic_id): (name, msg_type)
            for topic_id, name, msg_type in topic_rows
            if name in topics
        }
        missing = set(topics) - {name for name, _ in chosen.values()}
        if missing:
            raise ValueError('Bag is missing topics: ' + ', '.join(sorted(missing)))

        self.publishers_by_id = {}
        self.message_types_by_id = {}
        for topic_id, (name, msg_type) in chosen.items():
            try:
                cls = get_message(msg_type)
            except Exception as exc:
                raise RuntimeError(
                    f'Cannot load {msg_type} for {name}: {exc}') from exc
            durability = (DurabilityPolicy.TRANSIENT_LOCAL
                          if name == '/tf_static' else DurabilityPolicy.VOLATILE)
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=durability,
            )
            self.publishers_by_id[topic_id] = self.create_publisher(cls, name, qos)
            self.message_types_by_id[topic_id] = cls

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        placeholders = ','.join('?' for _ in chosen)
        where = 'topic_id IN (' + placeholders + ')'
        minmax = self.connection.execute(
            'SELECT MIN(timestamp), MAX(timestamp) FROM messages WHERE ' + where,
            tuple(chosen)).fetchone()
        if minmax[0] is None:
            raise RuntimeError('No selected-topic messages in the bag')
        self.start_ns = max(self.start_ns, int(minmax[0]))
        end_ns = int(minmax[1])
        if self.duration_sec > 0:
            end_ns = min(
                end_ns,
                self.start_ns + int(self.duration_sec * 1e9))
        if self.start_ns > end_ns:
            raise ValueError('start_ns is later than the selected messages')
        # SQLite timestamps are the rosbag playback timeline, not pose headers.
        self.messages = self.connection.execute(
            'SELECT topic_id, timestamp, data FROM messages '
            'WHERE ' + where + ' AND timestamp >= ? AND timestamp <= ? '
            'ORDER BY timestamp, id',
            (*chosen, self.start_ns, end_ns))
        self.end_ns = end_ns
        self.get_logger().info(
            f'Bag: {bag} | rate={self.rate:g}x, /clock={self.clock_hz:g} Hz, '
            f'first={self.start_ns}, last={self.end_ns}, '
            f'topics={sorted(topics)}')

    def play(self):
        # Start /clock at the selected timestamp; give SVO subscriber time
        # to attach before recorded messages begin moving.
        self.clock_pub.publish(to_clock_msg(self.start_ns))
        self.get_logger().info(
            f'Initial /clock={self.start_ns}; waiting '
            f'{self.startup_delay_sec:g}s before playback')
        time.sleep(self.startup_delay_sec)
        wall_start = time.monotonic()
        period_wall = 1.0 / self.clock_hz
        next_clock_wall = wall_start
        sent = 0
        last_clock_ns = self.start_ns

        for topic_id, timestamp, data in self.messages:
            if not rclpy.ok():
                break
            target_wall = wall_start + (
                (int(timestamp) - self.start_ns) / 1e9 / self.rate)
            while rclpy.ok():
                now = time.monotonic()
                if now >= target_wall:
                    break
                if now >= next_clock_wall:
                    elapsed = max(0.0, (now - wall_start) * self.rate)
                    current_ns = min(
                        int(timestamp),
                        self.start_ns + int(elapsed * 1e9))
                    last_clock_ns = max(last_clock_ns, current_ns)
                    self.clock_pub.publish(to_clock_msg(last_clock_ns))
                    next_clock_wall = now + period_wall
                else:
                    time.sleep(min(
                        0.005, target_wall - now,
                        next_clock_wall - now))
            if not rclpy.ok():
                break
            # Ensure the clock never runs behind the message we are publishing.
            last_clock_ns = max(last_clock_ns, int(timestamp))
            self.clock_pub.publish(to_clock_msg(last_clock_ns))
            cls = self.message_types_by_id[int(topic_id)]
            try:
                msg = deserialize_message(data, cls)
            except Exception as exc:
                raise RuntimeError(
                    f'Unable to deserialize topic id {topic_id}. '
                    'Use the decompressed SQLite bag, not the zstd-compressed '
                    'original. ' + str(exc)) from exc
            self.publishers_by_id[int(topic_id)].publish(msg)
            sent += 1
            if sent % 500 == 0:
                self.get_logger().info(
                    f'Published {sent} bag messages | clock={last_clock_ns} | '
                    f'wall_elapsed={time.monotonic()-wall_start:.1f}s')
        if rclpy.ok():
            self.clock_pub.publish(to_clock_msg(self.end_ns))
        self.get_logger().info(f'Playback finished; messages published: {sent}')

    def destroy_node(self):
        if hasattr(self, 'connection'):
            self.connection.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MA2BagPlayer()
        node.play()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
