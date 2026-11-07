# JetPack 5-compatible GPU image already validated on the ZED Box.
FROM ultralytics/ultralytics:latest-jetson-jetpack5

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates software-properties-common \
    libturbojpeg libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-foxy-ros-base ros-foxy-sensor-msgs ros-foxy-std-msgs \
       ros-foxy-message-filters ros-foxy-tf2-ros ros-foxy-visualization-msgs \
       python3-scipy \
       python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/zed_ws
COPY ros2_ws/src/zed_box_perception /opt/zed_ws/src/zed_box_perception
COPY ros2_ws/src/stereo_lidar_footprint_detector /opt/zed_ws/src/stereo_lidar_footprint_detector
RUN /bin/bash -c 'source /opt/ros/foxy/setup.bash && colcon build --symlink-install'

CMD ["/bin/bash"]
