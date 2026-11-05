"""Provisional milliAmpere1 ZED X + water-plane TF launch for ROS2 Foxy.

ZED mounting and water height are temporary assumptions for initial testing.
The official ZED wrapper URDF creates the camera TF from cam_pose. Do not
publish a second vessel_center -> zedx_camera_link transform.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    wrapper_share = get_package_share_directory('zed_wrapper')
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            wrapper_share, 'launch', 'include', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': 'zedx',
            'camera_name': 'zedx',
            'base_frame': 'vessel_center',
            # Placeholder! Camera 1 m above vessel reference; no rotation.
            'cam_pose': '[0.0,0.0,1.0,0.0,0.0,0.0]',
            # Vessel navigation already publishes piren->vessel_center.
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_urdf': 'true',
        }.items()
    )
    # Placeholder! Vessel origin 0.50 m above water, assuming Z points up.
    water_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ma1_water_plane_tf',
        arguments=['0', '0', '-0.5', '0', '0', '0',
                   'vessel_center', 'ma1_water_plane'],
        output='screen'
    )
    return LaunchDescription([zed_launch, water_tf])
