"""Provisional mA1 mounting geometry, independent of the ZED wrapper.

DO NOT run the ZED placeholder publisher until checking the official wrapper's
frame IDs and existing URDF TFs. A TF child must have only one parent.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='ma1_water_plane_tf',
            # Temporary plane follows vessel pitch/roll; Z-up only if
            # vessel_center is Z-up. Not an earth-level plane.
            arguments=['0', '0', '-0.5', '0', '0', '0',
                       'vessel_center', 'ma1_water_plane'],
            output='screen'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='ma1_zedx_mount_placeholder_tf',
            # TEMPORARY ONLY. Replace child with confirmed ZED wrapper
            # root mounting frame, and don't duplicate any wrapper TF.
            arguments=['0', '0', '1.0', '0', '0', '0',
                       'vessel_center', 'zedx_camera_link'],
            output='screen'),
    ])
