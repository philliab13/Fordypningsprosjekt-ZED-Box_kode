# ZED Box perception publisher — mA1

ROS 2 **Foxy** package for the ZED 2 / Jetson Orin NX **JetPack 5.1.1**, using the existing **ZED SDK 4.0.x** and the previously working **Ultralytics TensorRT** image. Designed for **one central ROS 2 Humble rosbag recorder** aboard mA1.

This is a focused camera publisher based on the **Stereolabs ZED ROS 2 wrapper's conventions**, not a fork of the full wrapper. See https://github.com/stereolabs/zed-ros2-wrapper and https://docs.stereolabs.com/docs/integrations/ros-2 . The upstream wrapper provides many features not implemented here (tracking, positional tracking, TF URDF, diagnostics). **Do not run the official wrapper and this camera node simultaneously**: both would try to open the same ZED camera.

## Publications

| Topic | Type | Notes |
|---|---|---|
| `/zed/left/image_rect_color` | `sensor_msgs/msg/Image` | `bgr8` left rectified RGB image |
| `/zed/right/image_rect_color` | `sensor_msgs/msg/Image` | `bgr8` right rectified RGB |
| `/zed/depth/depth_registered` | `sensor_msgs/msg/Image` | `32FC1`, metres, NaN for invalid |
| `/zed/left/camera_info` | `sensor_msgs/msg/CameraInfo` | intrinsics from ZED SDK |
| `/zed/right/camera_info` | `sensor_msgs/msg/CameraInfo` | intrinsics and stereo baseline encoded in P |
| `/zed/imu/data_raw` | `sensor_msgs/msg/Imu` | optional; camera IMU timestamp, orientation left unknown |
| `/zed/perception/instance_ids` | `sensor_msgs/msg/Image` | `16UC1`; 0=background, 1..N=YOLO detection IDs |
| `/zed/perception/detections_json` | `std_msgs/msg/String` | JSON contains frame stamp, IDs, class, confidence, bbox; same index as mask |

The same **camera acquisition timestamp** is applied to left, right, depth and detection-mask data derived from each frame. The JSON contains this timestamp because `std_msgs/String` has no header. IMU uses its own hardware timestamp if available. Topics use sensor-data QoS (best effort, keep last 1) for throughput; a central recorder should use compatible QoS overrides.

**Important:** The package does **not** publish camera-to-vessel TF; an accurate measured mA1 extrinsic and a clock synchronization procedure must be established separately. Rosbag recording times do not automatically synchronize separate device clocks.

## Docker (ZED Box)

The Dockerfile extends the **same JetPack 5 Ultralytics image** already used for YOLO and installs ROS 2 Foxy, colcon and the TurboJPEG dependency. This does not upgrade JetPack, CUDA, TensorRT or the host ZED SDK.

```bash
git clone https://github.com/philliab13/Fordypningsprosjekt-ZED-Box_kode.git
cd Fordypningsprosjekt-ZED-Box_kode
docker build -t zed-box-foxy -f docker/Dockerfile .
```

> Build on the Jetson. The base image tag `latest-jetson-jetpack5` is mutable: after a successful validation, pin an image digest for repeatability.

Use the **existing host PyZED binding**, host SDK and TensorRT engine; these are deliberately NOT committed to git. The command assumes the repo lives in `~/Fordypningsprosjekt-ZED-Box_kode` and engine/model files live in `~/zed_perception`.

```bash
sudo docker run --rm -it --runtime=nvidia --network=host --ipc=host --privileged \
  -v /dev:/dev \
  -v /usr/local/zed:/usr/local/zed:ro \
  -v /home/user/.local/lib/python3.8/site-packages/pyzed:/opt/pyzed/pyzed:ro \
  -v ~/zed_perception:/workspace/data:ro \
  -e PYTHONPATH=/opt/pyzed \
  -e LD_LIBRARY_PATH=/usr/local/zed/lib \
  -e ROS_DOMAIN_ID=42 -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  zed-box-foxy bash
```

Inside the container:

```bash
source /opt/ros/foxy/setup.bash
source /opt/zed_ws/install/setup.bash
ros2 run zed_box_perception zed_perception_node --ros-args \
  -p engine_path:=/workspace/data/models/yolo26n-seg.engine \
  -p source:=live \
  -p camera_fps:=15 \
  -p depth_mode:=PERFORMANCE
```

For SVO playback use `-p source:=svo -p svo_path:=/workspace/data/data/YOUR_FILE.svo`. Set `-p enable_yolo:=false` to isolate ZED publishing. Set `-p publish_right:=false`, `-p publish_imu:=false`, or `-p publish_rgb:=false` to reduce throughput.

If a Fast DDS peer profile is needed over campus Wi-Fi, mount the XML and export `FASTRTPS_DEFAULT_PROFILES_FILE` inside the container. For mA1 use a dedicated onboard network and test a **Humble rosbag subscriber**, not merely `ros2 topic list`.

## Central recording

Record on **mA1's Humble recorder**, not necessarily on the Jetson:

```bash
ros2 bag record -o ma1_with_zed \
  /zed/left/image_rect_color /zed/right/image_rect_color \
  /zed/depth/depth_registered \
  /zed/left/camera_info /zed/right/camera_info \
  /zed/imu/data_raw /zed/perception/instance_ids \
  /zed/perception/detections_json \
  /tf /tf_static
```

Append the existing mA1 LiDAR, GNSS and navigation topics to the same command. Inspect `ros2 topic info -v` and confirm QoS compatibility, especially best-effort image data.

## Before deployment

1. Verify PyZED loads in the built container and the TensorRT engine deserializes.
2. Check frame dimensions, encoding, camera `CameraInfo.P`/baseline and frame IDs.
3. Verify every RGB/depth/mask frame has an identical acquisition timestamp.
4. Validate ZED-to-LiDAR extrinsics and synchronize Jetson/mA1 wall clocks (NTP/PTP).
5. Confirm sustained reception and rosbag contents over **onboard Ethernet**. Previous testing showed 52 KB JPEGs succeed Foxy→Humble, whereas uncompressed 1080p RGB was **not** yet received over campus Wi-Fi.
6. Monitor recording throughput and free disk space; uncompressed 1080p stereo+float32 depth is very high bandwidth.

## Package layout

```text
ros2_ws/src/zed_box_perception/
  package.xml
  setup.py
  resource/zed_box_perception
  zed_box_perception/
    __init__.py
    node.py
docker/Dockerfile
```
