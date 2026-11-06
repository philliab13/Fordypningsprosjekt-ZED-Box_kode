"""Foxy-compatible, vectorized extraction of x/y/z from PointCloud2.

Equivalent input contract to sensor_msgs_py.point_cloud2.read_points_numpy()
for three same-type float fields, including organized row padding.
"""
import numpy as np
from sensor_msgs.msg import PointField


def read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True):
    fields = {field.name: field for field in cloud.fields}
    selected = [fields[name] for name in field_names]
    type_map = {PointField.FLOAT32: "f4", PointField.FLOAT64: "f8"}
    if any(f.datatype not in type_map or f.count != 1 for f in selected):
        raise ValueError("Expected scalar FLOAT32/FLOAT64 point fields")
    endian = ">" if cloud.is_bigendian else "<"
    dtype = np.dtype({
        "names": list(field_names),
        "formats": [endian + type_map[f.datatype] for f in selected],
        "offsets": [f.offset for f in selected],
        "itemsize": cloud.point_step,
    })
    if cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError("Invalid PointCloud2 row_step")
    points = np.ndarray(
        (cloud.height, cloud.width),
        dtype=dtype,
        buffer=cloud.data,
        strides=(cloud.row_step, cloud.point_step),
    )
    xyz = np.column_stack([points[name].reshape(-1) for name in field_names])
    xyz = np.asarray(xyz, dtype=np.float32)
    if skip_nans:
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return np.ascontiguousarray(xyz)
