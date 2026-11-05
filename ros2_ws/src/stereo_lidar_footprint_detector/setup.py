from setuptools import find_packages, setup
package_name = 'stereo_lidar_footprint_detector'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='phillip-andre', maintainer_email='philliab@stud.ntnu.no',
    description='ZED X ROS2 adapter for existing stereo-depth obstacle footprint pipeline',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'zedx_footprint_node = stereo_lidar_footprint_detector.zedx_node:main',
    ]},
)
