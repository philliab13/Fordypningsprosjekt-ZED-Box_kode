# Original footprint detector on ZED Box (Foxy)

Copied from `philliab13/Fordypningsprosjekt-ros-kode`, package
`src/stereo_lidar_footprint_detector` (main, 2026-11-06), with minimal
Foxy/Python 3.8 compatibility edits and one new ROS parameter: `use_lidar`.

## Preserved behavior

- Original `detection/obstacles.py`, `detection/yolo.py`, `backend.py`,
  `images.py`, `visualization.py`, `detection/outputs.py` and the
  original node processing/output logic.
- `std_msgs/msg/String` on `/obstacles/json` containing the original
  FeatureCollection, per-feature properties, `timestamp_ns`, `frame_id`,
  `units`, and polygons. No output schema changes.
- Registered RGB, optical-Z depth and CameraInfo input topics; nearest
  LiDAR cloud with the original MA2 projection and timestamp-skew logic.
- This is the **MA2 calibration** for replay tests. Do not use its hardcoded
  LiDAR-to-camera matrix on MA1 without replacing it with measured MA1 extrinsics.

## Changes for Foxy

- `use_lidar` (default `true`): with `false`, does not subscribe to LiDAR,
  provides empty LiDAR arrays to the **same** depth fusion/footprint geometry,
  and therefore estimates depth from registered stereo depth only.
  `require_lidar` retains its original meaning if `use_lidar=true`.
- `sensor_msgs_py.read_points_numpy` is replaced by local Foxy-compatible
  point-cloud XYZ extraction (handles float32/float64 and row padding).
- Python 3.10 union annotation changed for Python 3.8.
- Original `stixel_free_space.core` and `ma2_calibration` geometry utilities
  are bundled as pure Python to avoid requiring the whole stixel ROS package.
- Only the `footprint_node` entry point is packaged; the separate
  BEV, frame-review, and test nodes are not copied.

## Install in existing zed-official-wrapper container

The **existing container** has ROS Foxy, GPU-enabled PyTorch, and Ultralytics.
The repository is on the host; copy only this package into the container or
mount it into a new test container. Example on the ZED Box host:

```bash
git clone -b feature/footprint-lidar-toggle \
  https://github.com/philliab13/Fordypningsprosjekt-ZED-Box_kode.git \
  ~/Fordypningsprosjekt-ZED-Box_kode
sudo docker cp ~/Fordypningsprosjekt-ZED-Box_kode/ros2_ws/src/stereo_lidar_footprint_detector \
  zed-official-wrapper:/opt/zed_ws/src/
```

Inside the Foxy container, install missing ROS runtime packages if needed
(`ros-foxy-message-filters`, `ros-foxy-tf2-ros`,
`ros-foxy-visualization-msgs`), then:

```bash
source /opt/ros/foxy/setup.bash
cd /opt/zed_ws
colcon build --symlink-install --packages-select stereo_lidar_footprint_detector
source install/setup.bash
```

Pass the actual ZED stereo baseline and existing YOLO model path. With LiDAR:

```bash
ros2 run stereo_lidar_footprint_detector footprint_node --ros-args \
  -p baseline_m:=0.12 \
  -p weights:=/PATH/TO/yolo26n-seg.pt \
  -p use_lidar:=true
```

Stereo-only:

```bash
ros2 run stereo_lidar_footprint_detector footprint_node --ros-args \
  -p baseline_m:=0.12 \
  -p weights:=/PATH/TO/yolo26n-seg.pt \
  -p use_lidar:=false
```

Both modes require matching `/ma2/zed/left/image`,
`/ma2/zed/depth`, `/ma2/zed/camera_info` and a valid TF
from the optical frame to `ma2_water_plane` at the recorded image time.
Alternatively, set `-p water_frame:=''` to use the original synthetic
right/forward plane (not a vessel/world frame).

The default `require_lidar=false` permits stereo fallback if a LiDAR
cloud is not found in a LiDAR-enabled run. For a hard LiDAR requirement,
set `-p require_lidar:=true`.

**Status:** Source port prepared; not tested inside your Orin NX container.
Benchmark FPS and compare JSONs before claiming parity.
