## Preferred 2023 MA2 playback: one timeline for rosbag and SVO

Foxy's installed `ros2 bag play` lacks `--clock`; `ma2_pose_clock`
only approximated bag time from navigation pose headers. Use
**`ma2_bag_player` instead**. It reads the **uncompressed** SQLite
recording, and uses the *same bag timestamps* to schedule LiDAR/TF/poses
and publish `/clock`. The SVO player's five-clap time offset is preserved.

Stop previous `ros2 bag play`, `ma2_pose_clock`, SVO player, and official
ZED SVO wrapper before trying this method. Build and source the updated
package. The original compressed `.db3` cannot be used with this player:
it requires `scen4_2_uncompressed.db3`.

For the **first test**, use **0.01x** playback. This is intentionally slow
because the full-HD ZED 2 PERFORMANCE depth player previously published
only ~0.13 frames/s. A future FPS benchmark must separately report the
cost of SVO/depth decoding versus perception node throughput.

**Terminal 1: original SVO player, with one-time startup seek**

```bash
source /opt/ros/foxy/setup.bash
source /opt/zed_official_ws/install/setup.bash
export PYTHONPATH=/opt/zed_official_ws/python:$PYTHONPATH
ros2 run stereo_lidar_footprint_detector ma2_svo_player --ros-args \
  -p scenario:=scen4_2 \
  -p scenario_dir:=/opt/zed_official_ws/src/stereo_lidar_footprint_detector/config \
  -p data_root:=/opt/zed_official_ws/datasets/scene4_2 \
  -p start_ns:=1689070860509022989
```

`start_ns` is the initial recorded **bag** timestamp. The SVO player
calculates a frame-index estimate based on SVO FPS and its first
clap-adjusted acquisition timestamp, seeks **once at initialization**
(approximately 17 seconds), and loads the next frame. It does **not**
skip frames dynamically during playback. To disable the initial seek,
omit `start_ns` (default 0).

**Terminal 2: slow bag replay with built-in clock**

```bash
source /opt/ros/foxy/setup.bash
source /opt/zed_official_ws/install/setup.bash
ros2 run stereo_lidar_footprint_detector ma2_bag_player --ros-args \
  -p bag_file:=/opt/zed_official_ws/datasets/scen4_2/scen4_2_uncompressed.db3 \
  -p start_ns:=1689070860509022989 \
  -p rate:=0.01 \
  -p clock_hz:=20.0 \
  -p duration_sec:=2.0
```

This example runs **two seconds of simulated recording**, taking at
least 200 seconds at 0.01x. Remove `duration_sec` for the full
recording after validating correct playback. Increase `rate` only if
the SVO player can process frames at the required wall-time rate.
The player publishes `/tf_static` as transient-local, and uses reliable
publishers for the selected topics. Custom message types not installed
in Foxy should not be selected.

**Terminal 3: inspect topic rates, stamps, and SVO skew**

```bash
ros2 topic hz /clock
ros2 topic hz /ma2/zed/left/image
ros2 topic hz /lidar_aft/points
```

The `/clock` rate reports real-time wall frequency (nominally 20 Hz);
the sensor rates scale with the playback speed. Do not compare the
slowed-down image publisher's wall-clock Hz to live perception FPS.
If the camera lags, lower `rate` rather than widening LiDAR timestamp
association thresholds. The bag publisher and camera operate in separate
processes; Python timers and SDK processing may still introduce jitter.
Clock-controlled playback does **not** itself guarantee frame-level
acknowledgment or lossless processing when the pipeline cannot keep up.

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

## Foxy rosbag playback: /clock workaround

The installed Foxy `ros2 bag play --help` has **no `--clock` option**.
Instead, this branch includes a small `ma2_pose_clock` ROS node that
publishes the header timestamps from `/navigation/pose` as
`rosgraph_msgs/Clock`. This is sufficient to *attempt* initial synchronized
replay with the original SVO player's `/clock` callback. It is an
approximation: pose messages arrive at ~20 Hz, their header timestamps
may not match bag-record arrival times exactly, and the publisher does
not pace the rosbag. Evaluate timestamp skew and frame skips before
using any geometry for quantitative analysis.

After building and sourcing `stereo_lidar_footprint_detector`, run in
separate terminals, **in this order**:

1. Start `ros2 run stereo_lidar_footprint_detector ma2_pose_clock`
   (**do not** set `use_sim_time=true` on the clock relay).
2. Start `ros2 run stereo_lidar_footprint_detector ma2_svo_player --ros-args
   -p scenario:=scen4_2
   -p scenario_dir:=/opt/zed_official_ws/src/stereo_lidar_footprint_detector/config
   -p data_root:=/opt/zed_official_ws/datasets/scene4_2`
   with accessible `pyzed.sl` Python bindings.
3. Start the rosbag: `ros2 bag play /opt/zed_official_ws/datasets/scen4_2 --topics /lidar_aft/points /navigation/pose /tf /tf_static`.

Because `--topics` is a **greedy** option in Foxy, provide the bag path
before the option, and the selected topics after it. Do not use
`--clock` on this Foxy release.

This test assumes the original SVO's acquisition timestamps plus the
scenario's clap-derived offset correspond to replayed pose and LiDAR
timestamps. If the player reports out-of-sync timestamps, stop and
inspect timing rather than widening fusion skew arbitrarily.

## A. Test live ZED X (no LiDAR)

Keep the official ZED X wrapper running in the first terminal.

```bash
ros2 run stereo_lidar_footprint_detector zedx_footprint_node --ros-args \
  --params-file /opt/zed_official_ws/src/stereo_lidar_footprint_detector/config/live_zedx.yaml
```

This is the already-tested stereo-only configuration. Until
`water_frame` is set with valid TF, output geometry is in a **synthetic
camera-right/forward plane**, not vessel/map coordinates.

## MA2 clock-synchronized SVO publisher (preferred for recorded MA2 test)

The original `ma2_svo_player.py` and `scenario.py` have now been copied
from `Fordypningsprosjekt-ros-kode/src/stixel_free_space` into the
`stereo_lidar_footprint_detector` package, unchanged. The original
`scen4_2` clap timestamps have been added in `config/scen4_2.yaml`;
its SVO path is adapted to the mounted ZED Box dataset directory.
**Do not run the official ZED 2 SVO wrapper at the same time.** That
wrapper replay was observed to stamp images with current wall time, not
the July 2023 acquisition timestamps; the original SVO player applies
the measured offset and publishes on the bag's `/clock` instead.

`pyzed.sl` is mandatory for the original player. It was **missing from
the existing `zedx-official` container** even though the C++ wrapper
worked. First find host-side bindings:

```bash
# ZED Box host, not Docker
find /home/user -path '*/pyzed/sl*.so' -o -path '*/pyzed/sl.py' 2>/dev/null
```

If the bindings are in `/home/user/.local/lib/python3.8/site-packages/pyzed`,
make them visible using the already-mounted workspace, without rebuilding Docker:

```bash
mkdir -p ~/zed_official_ws/python
cp -a /home/user/.local/lib/python3.8/site-packages/pyzed ~/zed_official_ws/python/
```

Then inside `zedx-official`, in the shell that will run the SVO player:

```bash
source /opt/ros/foxy/setup.bash
source /opt/zed_official_ws/install/setup.bash
export PYTHONPATH=/opt/zed_official_ws/python:$PYTHONPATH
python3 -c 'import pyzed.sl as sl; print("pyzed import OK")'
ros2 run stereo_lidar_footprint_detector ma2_svo_player --ros-args \
  -p scenario:=scen4_2 \
  -p scenario_dir:=/opt/zed_official_ws/src/stereo_lidar_footprint_detector/config \
  -p data_root:=/opt/zed_official_ws/datasets/scene4_2
```

If the binding import or SVO open fails, fix that before starting the bag.
The SVO player subscribes to `/clock`, so it should be started BEFORE
`ros2 bag play ... --clock`. It will publish the four original camera topics
(`/ma2/zed/left/image`, `/ma2/zed/depth`,
`/ma2/zed/disparity`, `/ma2/zed/camera_info`) as the bag clock advances.

If no host-side binding exists, the SDK Python package needs to be
installed for the *matching* JetPack/Python/ZED SDK 4.0.6 environment.
Do not assume that the current host Python and container Python share
their search paths.

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
