# mA1 recorded-data adaptation (ROS 2 Foxy ZED Box)

This feature branch is based on ZED Box **main**, with the LiDAR-to-camera TF
method adapted from `philliab13/Fordypningsprosjekt-ros-kode` **MA1-test**.
The working MA2 defaults remain available; mA1 is enabled with explicit YAML
parameters. No model/shoreline algorithm or output schema has been changed.

## Recorded topic mapping

| Input | mA1 topic |
| --- | --- |
| Left rectified RGB | `/zed2/zed_node/left/image_rect_color` |
| Registered depth | `/zed2/zed_node/depth/depth_registered` |
| Left camera calibration | `/zed2/zed_node/left/camera_info` |
| Forward Ouster | `/front_lidar/ouster/points` |

The recorded TF graph has these disconnected roots: `vessel_center`
(under `piren`), `os_sensor` (parent of `os_lidar`), and `base_link`
(parent of ZED frames). The TF launch adds ONLY the missing roots:

- `vessel_center -> base_link`: xyz **[0.90, 0.00, 2.18] m**, identity rotation.
- `vessel_center -> os_sensor`: xyz **[2.00, 1.00, 0.00] m**, identity rotation.
- `vessel_center -> ma1_water_plane`: xyz **[0, 0, -0.30] m**, identity rotation.

**Important:** These mounting values assume `vessel_center` has +X forward
and +Z up, and that the physical camera measurement refers to `base_link`.
Confirm the frame conventions and existing TF before publishing. If the vessel
frame is NED/Z-down, the water transform and shoreline height filtering MUST
be adjusted. A missing/incorrect TF chain invalidates the geometry.

## On ZED Box after checking out this branch

Copy changed packages into the existing Docker container (host paths are not
automatically bind-mounted). Avoid overwriting the known-good image; snapshot
or retain it as a rollback point. Inside the container, build:

```bash
cd /opt/zed_ws
colcon build --symlink-install --packages-select \
  stereo_lidar_footprint_detector nearest_obstacle_shoreline_detector
source install/setup.bash
```

For ROS bag replay, ensure `/clock` is present and clock QoS is compatible
before setting `use_sim_time:=true`. Start mount TF (only after checking no
duplicate parents):

```bash
ros2 launch nearest_obstacle_shoreline_detector ma1_mounts.launch.py
```

Run footprint (set actual validated stereo baseline and existing TensorRT engine):

```bash
ros2 run stereo_lidar_footprint_detector footprint_node --ros-args \
  --params-file /opt/zed_ws/src/stereo_lidar_footprint_detector/config/ma1.yaml \
  -p baseline_m:=0.12 \
  -p weights:=/opt/zed_ws/yolo26n-seg.engine -p device:=0
```

The example baseline 0.12 m is the previous setup value, **not a verified
measurement for this recording**. Check left CameraInfo and stereo calibration.
Run shoreline:

```bash
ros2 launch nearest_obstacle_shoreline_detector lidar_projection.launch.py \
  params_file:=/opt/zed_ws/src/nearest_obstacle_shoreline_detector/config/ma1.yaml \
  use_sim_time:=false
```

## Validation before evaluating geometry

1. Check actual `header.frame_id` for RGB/depth/CameraInfo/LiDAR and camera
   registered-depth encodings. Detector requires matching optical frame IDs
   across RGB/depth/info and `32FC1` depth.
2. Check no duplicated TF parent for `base_link`/`os_sensor`.
3. Confirm `tf2_echo ma1_water_plane os_lidar` and
   `tf2_echo zed2_left_camera_optical_frame os_lidar` resolve.
4. Confirm axis handedness, mounting reference origins and 30 cm water level.
5. Measure camera/LiDAR timestamp offsets, output rate and geometry before
   enabling downstream planning.
