"""Sensor geometry helpers copied from the established stixel_free_space/core.py.

Source: philliab13/Fordypningsprosjekt-ros-kode
    src/stixel_free_space/stixel_free_space/core.py
These functions preserve the existing ROS/geometry behavior.
"""
import numpy as np
from scipy.spatial.transform import Rotation

def stamp_ns(msg):
    return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

def transform_matrix(transform):
    q, t = transform.rotation, transform.translation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    matrix[:3, 3] = [t.x, t.y, t.z]
    if not np.isfinite(matrix).all():
        raise ValueError('Non-finite TF transform')
    return matrix

def project_lidar(points, camera_from_lidar, k, shape):
    points = np.asarray(points).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    xyz = points @ camera_from_lidar[:3, :3].T + camera_from_lidar[:3, 3]
    xyz = xyz[xyz[:, 2] > 0]
    uvw = xyz @ k.T
    uv = uvw[:, :2] / uvw[:, 2:3]
    h, w = shape
    keep = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    return np.ascontiguousarray(uv[keep]), np.ascontiguousarray(xyz[keep])

def nearest_cloud(clouds, timestamp, max_skew_s):
    if not clouds:
        return None
    cloud = min(clouds, key=lambda msg: abs(stamp_ns(msg) - timestamp))
    return cloud if abs(stamp_ns(cloud) - timestamp) <= max_skew_s * 1e9 else None

def cloud_xyz(msg):
    """Decode XYZ from PointCloud2 on Foxy and newer ROS 2 distributions."""
    from sensor_msgs_py import point_cloud2
    if hasattr(point_cloud2, 'read_points_numpy'):
        return np.asarray(point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)).reshape(-1, 3)
    return np.asarray(list(point_cloud2.read_points(
        msg, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float32).reshape(-1, 3)
