# mA1 live test setup (ZED Box Orin NX / ROS 2 Foxy)

Based on the `MA1-test` branch of `philliab13/Fordypningsprosjekt-ros-kode`,
particularly `checklist_Ma1.md` and the live node topic defaults.

## Topic mapping

| Stream | Live mA1 / ZED Box topic |
| --- | --- |
| ZED X rectified color | `/zedx/zed_node/left/image_rect_color` |
| ZED X registered depth | `/zedx/zed_node/depth/depth_registered` |
| ZED X calibration | `/zedx/zed_node/left/camera_info` |
| Forward Ouster LiDAR | `/front_lidar/ouster/points` |
| Alternative aft Ouster | `/back_lidar/ouster/points` (verify before use) |
| MA1's existing EO camera | `/camera_eo/image_raw` (not used by this ZED X pipeline) |
| Footprint output | `/obstacles/json` |
| Shoreline boundary output | `/lidar_water_projection/barrier_json` |

The older branch used `/zed/zed_node/rgb/color/rect/image` for a ZED 2
on a different PC. Our *ZED Box* has already verified its own
`/zedx/zed_node/...` topics, which remain unchanged.

## Installation

On ZED Box HOST:

```bash
cd ~/Fordypningsprosjekt-ZED-Box_kode
git fetch origin
git switch mA1
git pull --ff-only origin mA1
cp -a ros2_ws/src/stereo_lidar_footprint_detector ~/zed_official_ws/src/
cp -a ros2_ws/src/nearest_obstacle_shoreline_detector ~/zed_official_ws/src/
```

Inside existing Docker container:

```bash
sudo docker exec -it zedx-official bash
source /opt/ros/foxy/setup.bash
cd /opt/zed_official_ws
colcon build --symlink-install --packages-select \
  stereo_lidar_footprint_detector nearest_obstacle_shoreline_detector
source install/setup.bash
```

## Separate terminals: live nodes

Terminal A, official camera wrapper:

```bash
source /opt/ros/foxy/setup.bash
source /opt/zed_official_ws/install/setup.bash
ros2 launch zed_wrapper zedx.launch.py
```

Terminal B, footprint detector using mA1 front Ouster, but without a
verified water plane or fixed frame **yet**:

```bash
source /opt/ros/foxy/setup.bash
source /opt/zed_official_ws/install/setup.bash
ros2 run stereo_lidar_footprint_detector zedx_footprint_node --ros-args \
  --params-file /opt/zed_official_ws/src/stereo_lidar_footprint_detector/config/ma1_live.yaml
```

The footprint config sets `enable_lidar: true`, `require_lidar: false`
and `water_frame: ''`. Until a calibrated camera-to-LiDAR transform exists,
the node may reject frames when an actual LiDAR scan is selected. The blank
water frame is **only suitable for pipeline startup**; it is not a
calibrated mA1 footprint for a planner. To smoke-test stereo only, override
`-p enable_lidar:=false` after the params file.

Terminal C, shoreline detector:

```bash
source /opt/ros/foxy/setup.bash
source /opt/zed_official_ws/install/setup.bash
ros2 run nearest_obstacle_shoreline_detector lidar_projection_node --ros-args \
  --params-file /opt/zed_official_ws/src/nearest_obstacle_shoreline_detector/config/ma1_live.yaml
```

**Do not interpret shoreline output yet**: `ma1_water_plane` is
currently a *placeholder* target TF frame, not verified as an existing
Z-up water plane. With no matching TF, the detector will publish an
error/no-boundary status. The nearest-obstacle boundary also is not
semantically guaranteed to be shoreline.

## Verify when connected to mA1

```bash
ros2 topic list | grep -Ei 'front_lidar|back_lidar|navigation|camera_eo|tf'
ros2 topic info /front_lidar/ouster/points -v
ros2 topic echo /front_lidar/ouster/points | head -n 8
ros2 topic hz /front_lidar/ouster/points
ros2 topic hz /zedx/zed_node/left/image_rect_color
ros2 topic hz /obstacles/json
ros2 topic hz /lidar_water_projection/barrier_json
```

Check `header.frame_id` of the front cloud, the camera optical frame,
the timestamp base on both devices, and TF connectivity. Do not run the
MA2 `/clock`, bag playback, or SVO player in this live mode.

## Frames remain unresolved until new recording / mA1 inspection

The old `MA1-test` checklist proposed `ma1_water_plane`, `base_link`,
`os_sensor`, and `piren`, and described approximate mounting
relationships. These are **not verified ZED Box/mA1 transforms**.

Determine and validate:
- Exact LiDAR cloud `header.frame_id` (front/aft devices must not
  accidentally share conflicting frame names).
- ZED X `header.frame_id` and the actual parent frame published by the
  official ZED X wrapper.
- Camera and LiDAR mounts relative to the same vessel frame.
- Actual axis directions, water level, and whether `ma1_water_plane`
  should be level in world or vessel-following.
- Whether `piren` exists and is connected to the vessel pose for
  motion compensation, then configure `fixed_frame` if appropriate.

Other computer handles rosbag recording; this branch adds no recorder.
