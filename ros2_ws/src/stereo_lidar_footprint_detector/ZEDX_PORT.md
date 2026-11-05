# ZED X onboard obstacle footprint benchmark

This package ports the existing detection modules from
`philliab13/Fordypningsprosjekt-ros-kode`,
`src/stereo_lidar_footprint_detector/stereo_lidar_footprint_detector/detection/`.
The existing segmentation, stereo-depth estimation, and square-footprint algorithms are reused.
The TensorRT loader uses `task='segment'` for .engine weights.

## Deploy on ZED Box

The official Stereolabs wrapper should be running as `zedx-official`.
It owns the camera and publishes rectified RGB, registered depth, and CameraInfo.

On the ZED Box host, from the new branch:

```bash
mkdir -p ~/zed_official_ws/src
cp -a ~/Fordypningsprosjekt-ZED-Box_kode/ros2_ws/src/stereo_lidar_footprint_detector ~/zed_official_ws/src/
mkdir -p ~/zed_official_ws/models
cp ~/zed_perception/models/yolo26n-seg.engine ~/zed_official_ws/models/
```

From a second shell on the host:

```bash
sudo docker exec -it zedx-official bash
source /opt/ros/foxy/setup.bash
cd /opt/zed_official_ws
colcon build --symlink-install --packages-select stereo_lidar_footprint_detector
source install/setup.bash
ros2 run stereo_lidar_footprint_detector zedx_footprint_node --ros-args \
  -p weights:=/opt/zed_official_ws/models/yolo26n-seg.engine \
  -p baseline_m:=0.120001 -p max_stereo_depth_m:=10.0
```

Check in another sourced shell:

```bash
ros2 topic hz /obstacles/json
ros2 topic echo /obstacles/json
```

## Coordinate safety

The current test has no boat LiDAR and no camera-to-water TF.
It runs the original stereo-only obstacle algorithm using empty LiDAR inputs.
Polygon coordinates are on the original **synthetic camera-right/forward plane**,
not a geographic or vessel-local water plane. Never feed them directly to
the docking planner without transforming into a verified physical frame.

The port has not yet been runtime-tested on the Jetson; check the TensorRT
engine and Python dependencies. Measure final obstacle topic FPS as well as
processing time before drawing performance conclusions.
