"""TF-based LiDAR-to-camera calibration adapted from the original MA1-test branch.

Camera and cloud message frame IDs are the sources of truth. With fixed_frame
empty, scan points are transformed using mounting geometry at image time.
"""
from rclpy.duration import Duration
from rclpy.time import Time
from stixel_free_space.core import transform_matrix


def camera_from_lidar(buffer, image_header, cloud_header, fixed_frame=""):
    if not image_header.frame_id or not cloud_header.frame_id:
        raise ValueError("Image and LiDAR must have nonempty header.frame_id")
    if fixed_frame:
        tf = buffer.lookup_transform_full(
            image_header.frame_id, Time.from_msg(image_header.stamp),
            cloud_header.frame_id, Time.from_msg(cloud_header.stamp),
            fixed_frame, timeout=Duration(seconds=0.0))
    else:
        tf = buffer.lookup_transform(
            image_header.frame_id, cloud_header.frame_id,
            Time.from_msg(image_header.stamp),
            timeout=Duration(seconds=0.0))
    return transform_matrix(tf.transform)
