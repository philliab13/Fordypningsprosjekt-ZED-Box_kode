from setuptools import find_packages, setup

setup(
    name="stereo_lidar_footprint_detector",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/stereo_lidar_footprint_detector"]),
        ("share/stereo_lidar_footprint_detector", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="phillip-andre",
    maintainer_email="philliab@stud.ntnu.no",
    description="Original MA2 stereo-LiDAR footprints, with optional LiDAR for ZED Box Foxy",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "footprint_node = stereo_lidar_footprint_detector.node:main",
    ]},
)
