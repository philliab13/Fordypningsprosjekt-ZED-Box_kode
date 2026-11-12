FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# CPU-only detector; no ZED SDK, CUDA, or camera drivers are required.
COPY ros2_ws/src/nearest_obstacle_shoreline_detector/package.xml /tmp/detector/package.xml
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-colcon-common-extensions python3-rosdep \
       ros-humble-rmw-fastrtps-cpp ros-humble-rmw-cyclonedds-cpp \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths /tmp/detector --ignore-src --rosdistro humble -r -y \
    && rm -rf /var/lib/apt/lists/* /tmp/detector

WORKDIR /opt/shoreline_ws
COPY ros2_ws/src/nearest_obstacle_shoreline_detector src/nearest_obstacle_shoreline_detector
RUN source /opt/ros/humble/setup.bash \
    && colcon build --packages-select nearest_obstacle_shoreline_detector \
       --event-handlers console_direct+

COPY docker/shoreline/entrypoint.sh /shoreline-entrypoint.sh
COPY docker/shoreline/ma1-live.yaml /config/ma1-live.yaml
RUN chmod +x /shoreline-entrypoint.sh \
    && printf '\nsource /opt/ros/humble/setup.bash\nsource /opt/shoreline_ws/install/setup.bash\n' >> /root/.bashrc

ENV ROS_LOCALHOST_ONLY=0 \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    PARAMS_FILE=/config/ma1-live.yaml \
    USE_SIM_TIME=false

ENTRYPOINT ["/shoreline-entrypoint.sh"]
CMD ["detector"]
