

1. Copy pasta

sudo docker start zed-official-wrapper

sudo docker exec -it zed-official-wrapper bash
source /opt/ros/foxy/setup.bash
source /opt/zed_official_ws/install/setup.bash
source /opt/zed_ws/install/setup.bash


export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress></General></Domain></CycloneDDS>"

cd /opt/zed_ws

ros2 run stereo_lidar_footprint_detector footprint_node \
  --ros-args \
  --params-file /opt/zed_ws/src/stereo_lidar_footprint_detector/config/ma1.yaml \
  -p baseline_m:=0.12 \
  -p weights:=/opt/zed_ws/yolo26n-seg.engine \
  -p device:=0 \
  -p use_lidar:=true \
  -p lidar_max_skew_s:=0.5 \
  -p use_sim_time:=false

Lidar detector:

ros2 launch nearest_obstacle_shoreline_detector   lidar_projection.launch.py   params_file:=/opt/zed_ws/src/nearest_obstacle_shoreline_detector/config/ma1.yaml   use_sim_time:=false


Official zed 2 node:

source /opt/zed_official_ws/install/setup.bash

ros2 launch zed_wrapper zed2.launch.py

Tranforms: 
ros2 launch nearest_obstacle_shoreline_detector ma1_mounts.launch.py

dscr