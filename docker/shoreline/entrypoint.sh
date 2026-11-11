#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/shoreline_ws/install/setup.bash

if [[ "${1:-detector}" == "detector" ]]; then
    if [[ "$#" -gt 0 ]]; then shift; fi
    if [[ ! -r "${PARAMS_FILE}" ]]; then
        echo "Cannot read detector parameters: ${PARAMS_FILE}" >&2
        exit 1
    fi
    exec ros2 launch nearest_obstacle_shoreline_detector lidar_projection.launch.py \
        "params_file:=${PARAMS_FILE}" "use_sim_time:=${USE_SIM_TIME}" "$@"
fi

exec "$@"
