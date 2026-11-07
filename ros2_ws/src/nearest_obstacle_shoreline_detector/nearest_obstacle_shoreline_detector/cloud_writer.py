"""Minimal ROS 2 Foxy PointCloud2 writer for float32 fields.

Maintains the same PointCloud2 layout as sensor_msgs_py.point_cloud2
create_cloud / create_cloud_xyz32 for this node's packed float32 data.
"""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def create_cloud(header, fields, points):
    fields = list(fields)
    if not fields or any(
        field.datatype != PointField.FLOAT32 or field.count != 1
        or field.offset != 4 * index
        for index, field in enumerate(fields)
    ):
        raise ValueError("Expected contiguous scalar FLOAT32 PointCloud2 fields")
    values = np.asarray(points, dtype="<f4").reshape(-1, len(fields))
    values = np.ascontiguousarray(values)
    point_step = 4 * len(fields)
    return PointCloud2(
        header=header,
        height=1,
        width=len(values),
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * len(values),
        data=values.tobytes(),
        is_dense=bool(np.isfinite(values).all()),
    )


def create_cloud_xyz32(header, points):
    fields = [
        PointField(name=name, offset=4 * index,
                   datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    return create_cloud(header, fields, points)
