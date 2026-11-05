# Stereo + LiDAR footprints and LiDAR-only near-shore boundary

This branch contains two existing **algorithms**, copied from
[`Fordypningsprosjekt-ros-kode`](https://github.com/philliab13/Fordypningsprosjekt-ros-kode),
with small ROS interfaces/config changes for the ZED Box.

1. `stereo_lidar_footprint_detector`: original YOLO26n segmentation, registered
   stereo depth, optional time-matched LiDAR depth fusion, and 2D square footprint
   creation. Output: `/obstacles/json`. The copied
   `detection/obstacles.py` and `detection/pipeline.py` retain their
   geometry and fusion logic.
2. `nearest_obstacle_shoreline_detector`: original *LiDAR-only*
   height filtering, orthogonal water-plane projection and nearest-supported
   angular-sector boundary (`lidar_projection.py`). Outputs:
   `/lidar_water_projection/barrier_json`, `/barrier`,
   `/projected_points`, `/boundary_points`.
   It is **not** semantic shoreline segmentation and does not assume every
   nearest return is quay. This first port does not add RWPS/FastSAM.

No MA2 hard-coded ZED/LiDAR transformation is used in the new footprint
adapter. Camera/LiDAR TF must be present, calibrated and timestamp-compatible.

## Update on the ZED Box HOST

```bash
cd ~/Fordypningsprosjekt-ZED-Box_kode
git fetch origin
git switch feature/zedx-full-pipeline
git reset --hard origin/feature/zedx-full-pipeline
cp -a ros2_ws/src/stereo_lidar_footprint_detector ~/zed_official_ws/src/
cp -a ros2_ws/src/nearest_obstacle_shoreline_detector ~/zed_official_ws/src/
mkdir -p ~/zed_official_ws/models
cp ~/zed_perception/models/yolo26n-seg.engine ~/zed_official_ws/models/
```

The existing `zedx-official` container mounts
`~/zed_official_ws:/opt/zed_official_ws`. Use another terminal:

```bash
sudo docker exec -it zedx-official bash
source /opt/ros/foxy/setup.bash
cd /opt/zed_official_ws
colcon build --symlink-install --packages-select \
  stereo_lidar_footprint_detector nearest_obstacle_shoreline_detector
source install/setup.bash
```

The runtime requires the installed ROS Foxy Python packages
`rclpy`, `message_filters`, `tf2_ros`, `sensor_msgs_py`,
`visualization_msgs`, `geometry_msgs`, plus NumPy, SciPy, OpenCV,
Ultralytics and the already-tested TensorRT engine. The container has NOT
been revalidated with these two packages yet.

## A. Test live ZED X (no LiDAR)

Keep the official ZED X wrapper running in the first terminal.

```bash
ros2 run stereo_lidar_footprint_detector zedx_footprint_node --ros-args \
  --params-file /opt/zed_official_ws/src/stereo_lidar_footprint_detector/config/live_zedx.yaml
```

This is the already-tested stereo-only configuration. Until
`water_frame` is set with valid TF, output geometry is in a **synthetic
camera-right/forward plane**, not vessel/map coordinates.

## B. Test the 2023 MA2 dataset (recorded LiDAR + recorded camera)

**Critical distinction:** The existing MA2 dataset includes an SVO camera
recording **separate** from its rosbag. If the bag does not contain
`/ma2/zed/left/image`, `/ma2/zed/depth`, and
`/ma2/zed/camera_info`, you MUST run the existing SVO player alongside
`ros2 bag play`; the rosbag alone cannot supply stereo imagery.
Use the original old-repo SVO player with its previously established
image timestamp offset. The SVO player and bag must publish the same
recorded-time clock (not live wall-clock data). Do not mix the current
live ZED X images with July 2023 LiDAR scans.

Before running nodes, verify topics and frames on the recorder:

```bash
ros2 bag info /path/to/recording
ros2 topic list
ros2 topic echo /lidar_aft/points --once
ros2 topic echo /ma2/zed/left/camera_info --once
ros2 topic echo /tf_static --once
```

**Foxy note:** Some `ros2 topic echo` variants do not support
`--once`; stop echo manually with Ctrl+C. Start playback with
`ros2 bag play /path/to/bag --clock` where supported by the installed
rosbag version. Ensure `/clock` is active for nodes configured
with `use_sim_time:=true`.

For replay, **stop the live ZED X node** to avoid competing for GPU,
and start the replay source(s). Then, inside the prepared ROS 2 Foxy
container (or another compatible ROS 2 environment on the same network):

```bash
source /opt/ros/foxy/setup.bash
source /opt/zed_official_ws/install/setup.bash
ros2 run stereo_lidar_footprint_detector zedx_footprint_node --ros-args \
  --params-file /opt/zed_official_ws/src/stereo_lidar_footprint_detector/config/ma2_rosbag.yaml
```

The MA2 configuration sets `enable_lidar=true`, `require_lidar=true`,
and uses the bag's `piren` fixed frame and
`ma2_water_plane` water frame. The original SVO player supplies the
three MA2 stereo inputs; the rosbag supplies LiDAR and TF.
These settings only work if the full verified TF tree is available:
`ma2_water_plane <- camera optical`, `piren <- camera optical`,
and `piren <- lidar`. In particular, the old project's static
camera/LiDAR mounts and vessel pose must be published by their existing
launch/calibration setup. This port does not invent them.

If using a **different** rosbag containing all camera and LiDAR topics,
change the three camera input names, LiDAR topic and frames accordingly.

## C. LiDAR-only projection / barrier

This runs **without camera images**, including when only the MA2 LiDAR
rosbag is available:

```bash
ros2 run nearest_obstacle_shoreline_detector lidar_projection_node --ros-args \
  --params-file /opt/zed_official_ws/src/nearest_obstacle_shoreline_detector/config/ma2_lidar_replay.yaml
```

This requires a *Z-up* water-plane TF at each LiDAR timestamp. The old
repository uses `ma2_water_plane_up`. If the existing MA2 setup only
publishes `ma2_water_plane` (Z down), the original project adds a
180-degree X rotation to define `ma2_water_plane_up`:

```bash
ros2 run tf2_ros static_transform_publisher \
  --roll 3.141592653589793 --pitch 0 --yaw 0 \
  --frame-id ma2_water_plane --child-frame-id ma2_water_plane_up
```

**Only use this transform for the original MA2 frame convention;**
do not blindly apply it to a different vessel or Z-up frame.

## D. Measure outputs

```bash
ros2 topic hz /obstacles/json
ros2 topic hz /lidar_water_projection/barrier_json
ros2 topic echo /obstacles/json
ros2 topic echo /lidar_water_projection/barrier_json
```

The footprint node logs `pipeline ... ms`, `worker ... ms`,
`LiDAR pixels N` and output frequency. The shoreline node reports
`status: ok` or explanatory errors. If TF is unavailable, a shoreline
message may contain empty `segments`; do not interpret an empty output
as evidence of no obstacles. An empty `features` array for a frame
without class-8 boat detections is normal.

## Scope / next verification

* Live stereo-only performance was previously observed around 8–11 Hz,
  **without boats**. LiDAR replay throughput and end-to-end fusion have not
  yet been measured on this Jetson.
* Historical MA2 LiDAR and SVO offsets, TF availability and Foxy/Humble
  topic interoperability must be checked on the actual dataset.
* Both pipelines remain independent. Do not concatenate their outputs as
  a planner-ready obstacle set until common spatial frames and timestamps
  are validated. LiDAR nearest boundaries can include boats.
