"""Minimal PointCloud2 XYZ reader and float32 cloud writer for ROS2 Foxy.

Avoids sensor_msgs_py (missing in some Foxy images). Handles arbitrary point
stride, row padding, field offsets and byte order. Output clouds use FLOAT32.
"""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

_FIELD_FORMATS = {
    PointField.INT8: 'i1',
    PointField.UINT8: 'u1',
    PointField.INT16: 'i2',
    PointField.UINT16: 'u2',
    PointField.INT32: 'i4',
    PointField.UINT32: 'u4',
    PointField.FLOAT32: 'f4',
    PointField.FLOAT64: 'f8',
}


def read_xyz(cloud):
    fields = {f.name: f for f in cloud.fields}
    for name in ('x', 'y', 'z'):
        if name not in fields:
            raise ValueError('PointCloud2 missing ' + name + ' field')
    if cloud.height == 0 or cloud.width == 0:
        return np.empty((0, 3), dtype=np.float32)
    if cloud.point_step <= 0 or cloud.row_step < cloud.point_step * cloud.width:
        raise ValueError('Invalid PointCloud2 point/row step')
    if len(cloud.data) < cloud.row_step * cloud.height:
        raise ValueError('PointCloud2 buffer is shorter than row_step * height')
    endian = '>' if cloud.is_bigendian else '<'
    names, formats, offsets = [], [], []
    for name in ('x', 'y', 'z'):
        f = fields[name]
        if f.datatype not in _FIELD_FORMATS or f.count != 1:
            raise ValueError('Unsupported PointCloud2 field: ' + name)
        dtype = np.dtype(endian + _FIELD_FORMATS[f.datatype])
        if f.offset < 0 or f.offset + dtype.itemsize > cloud.point_step:
            raise ValueError('Invalid PointCloud2 field offset: ' + name)
        names.append(name)
        formats.append(dtype)
        offsets.append(f.offset)
    dtype = np.dtype(dict(names=names, formats=formats,
                          offsets=offsets, itemsize=cloud.point_step))
    records = np.ndarray(
        (cloud.height, cloud.width),
        dtype=dtype,
        buffer=bytes(cloud.data),
        strides=(cloud.row_step, cloud.point_step))
    xyz = np.column_stack([
        records[name].reshape(-1) for name in ('x', 'y', 'z')
    ]).astype(np.float32, copy=False)
    return xyz[np.isfinite(xyz).all(axis=1)]


def create_cloud(header, fields, points):
    """Pack N x F rows of float32 values into a PointCloud2."""
    arr = np.asarray(points, dtype=np.float32)
    if arr.ndim != 2 or arr.shape[1] != len(fields):
        raise ValueError('Expected N x len(fields) float32 point array')
    if any(f.datatype != PointField.FLOAT32 or f.count != 1 or
           f.offset != 4 * i for i, f in enumerate(fields)):
        raise ValueError('Writer requires contiguous FLOAT32 fields')
    arr = np.ascontiguousarray(arr, dtype='<f4')
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(arr)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = arr.shape[1] * 4
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = bool(np.isfinite(arr).all())
    msg.data = arr.tobytes()
    return msg


def create_cloud_xyz32(header, points):
    fields = [
        PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate(('x', 'y', 'z'))
    ]
    arr = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    return create_cloud(header, fields, arr)
