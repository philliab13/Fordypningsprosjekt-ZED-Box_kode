"""Mounting TF for the recorded milliAmpere1 ZED 2 and front Ouster.

These mounts are provisional pending header.frame_id and axis verification.
Do not run if any child frame already has a parent on /tf or /tf_static.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def static_tf(name, parent, child, xyz):
    return Node(
        package="tf2_ros", executable="static_transform_publisher",
        name=name, output="screen",
        arguments=[str(v) for v in (*xyz, 0.0, 0.0, 0.0)] + [parent, child])


def generate_launch_description():
    return LaunchDescription([
        # Camera's physical mounting origin; ZED wrapper owns its internal TF.
        static_tf("ma1_camera_mount", "vessel_center", "base_link",
                  (0.90, 0.0, 2.18)),
        # os_lidar is +1.00 m above vessel_center. The existing
        # os_sensor -> os_lidar transform adds +0.078 m vertically,
        # so mount os_sensor at +0.922 m. Preserve the driver's 180 deg yaw.
        static_tf("ma1_front_ouster_mount", "vessel_center", "os_sensor",
                  (2.0, 1.0, 0.922)),
        # Assumes vessel_center has positive Z up. Validate before projection.
        static_tf("ma1_water_plane_mount", "vessel_center", "ma1_water_plane",
                  (0.0, 0.0, -0.30)),
    ])
