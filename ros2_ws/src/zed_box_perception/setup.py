from setuptools import setup

package_name = 'zed_box_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ZED Box team',
    maintainer_email='maintainer@example.invalid',
    description='ZED 2 + YOLO TensorRT publisher for ROS 2 Foxy',
    license='MIT',
    entry_points={'console_scripts': [
        'zed_perception_node = zed_box_perception.node:main',
    ]},
)
