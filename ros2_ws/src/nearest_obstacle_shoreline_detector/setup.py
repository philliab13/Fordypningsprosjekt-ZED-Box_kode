from glob import glob
from setuptools import find_packages, setup

name = 'nearest_obstacle_shoreline_detector'
setup(
    name=name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        ('share/' + name + '/config', glob('config/*.yaml')),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'], zip_safe=True,
    maintainer='phillip-andre', maintainer_email='philliab@stud.ntnu.no',
    description='Copied LiDAR-only water-plane projection and nearest boundary',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'lidar_projection_node = nearest_obstacle_shoreline_detector.lidar_projection_node:main',
    ]},
)
