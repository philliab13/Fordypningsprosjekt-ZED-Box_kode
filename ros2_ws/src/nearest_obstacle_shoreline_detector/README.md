# LiDAR-only nearest-obstacle shoreline projection — ROS 2 Foxy

Port of the LiDAR-only `lidar_projection_node` from
`Fordypningsprosjekt-ros-kode/src/nearest_obstacle_shoreline_detector`.
No FastSAM, RWPS, segmentation models or `shoreline_node` are included.

The filtering and sector-boundary algorithms are copied **unchanged**. The
only runtime code change is using a local Foxy-compatible vectorized
`PointCloud2` XYZ reader instead of `sensor_msgs_py.point_cloud2.read_points_numpy`.
The local reader is copied from the existing Foxy footprint detector.

## Build in zed-official-wrapper

```bash
cd /opt/zed_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select nearest_obstacle_shoreline_detector
source install/setup.bash
```

For rosbag playback (and `/clock`), run:

```bash
ros2 launch nearest_obstacle_shoreline_detector lidar_projection.launch.py use_sim_time:=true
```

For live sensors, omit `use_sim_time:=true`.

The configured water frame is `ma2_water_plane_up`, with Z pointing up.
If you have the `ma2_water_plane` TF but not its positive-up child,
publish this static transform **once**:

```bash
ros2 run tf2_ros static_transform_publisher \
  --roll 3.141592653589793 --pitch 0 --yaw 0 \
  --frame-id ma2_water_plane --child-frame-id ma2_water_plane_up
```

The LiDAR's frame must be connected to the water frame at its measurement
timestamp. If it is not, the detector emits empty outputs and warns.

Published topics:
- `/lidar_water_projection/filtered_points` — 3D height-filtered points
- `/lidar_water_projection/projected_points` — flattened water points
- `/lidar_water_projection/boundary_points` — sector representatives
- `/lidar_water_projection/barrier` — MarkerArray line segments
- `/lidar_water_projection/barrier_json` — unchanged JSON output schema

To overlay on the main PC's BEV viewer:

```bash
ros2 run stereo_lidar_footprint_detector bev_node --ros-args \
  -p shoreline_topic:=/lidar_water_projection/barrier_json
```

This output is the **nearest supported obstacle boundary**, not semantic
shoreline classification. Boats can appear in it.

## Exact deviations from the original package
1. The `read_points_numpy` call and its import were replaced by a local
   Foxy-compatible vectorized reader. No boundary math changed.
2. The ROS package metadata and entry points were reduced to the LiDAR-only
   executable. This avoids unnecessary FastSAM/RWPS dependencies and weights.
3. Copied original LiDAR configuration, launch file, and geometry tests.
   Preserved the original `inputs.py` helper as-is.
4. No change to input/output topics, height thresholds, sectors, JSON
   format, or output coordinate frame.
