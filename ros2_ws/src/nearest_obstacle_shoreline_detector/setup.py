from glob import glob
from setuptools import find_packages, setup

setup(
    name='nearest_obstacle_shoreline_detector',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/nearest_obstacle_shoreline_detector']),
        ('share/nearest_obstacle_shoreline_detector', ['package.xml', 'README.md']),
        ('share/nearest_obstacle_shoreline_detector/config', glob('config/*.yaml')),
        ('share/nearest_obstacle_shoreline_detector/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='phillip-andre',
    maintainer_email='philliab@stud.ntnu.no',
    description='LiDAR-only water-plane projection and nearest obstacle boundary',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'lidar_projection_node = nearest_obstacle_shoreline_detector.lidar_projection_node:main',
    ]},
)
