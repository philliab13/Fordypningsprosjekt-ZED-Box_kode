# Standalone shoreline detector — Ubuntu 22.04 / ROS 2 Humble

CPU-only container for `nearest_obstacle_shoreline_detector`, intended for the
Ubuntu 22.04 Humble computer running alongside milliAmpere's boat or simulator
containers. No ZED Box, camera SDK, GPU, or Jetson runtime is needed. The existing
`docker/Dockerfile` is the separate historical ZED image.

## Build and start

Run on the Ubuntu computer with Docker Engine and the Docker Compose plugin:

```bash
cd /home/philliab/Documents/mA1/Fordypningsprosjekt-ZED-Box_kode/docker/shoreline
cp .env.example .env
# Edit .env to match the ROS_DOMAIN_ID used by the boat and other PCs.
docker compose config
docker compose up -d --build
docker compose logs -f shoreline
```

The detector starts automatically and restarts unless explicitly stopped.
Start the boat or simulator separately using the existing milliAmpere Compose
file; this project does not start them. On another PC, adjust the checkout path.

```bash
docker compose exec shoreline bash
docker compose down
```

The interactive shell has ROS and the detector workspace sourced. Source code
is built into the image: run `docker compose up -d --build` after code changes.
Parameter files are mounted read-only; run `docker compose restart shoreline`
after editing them. Apply `.env` changes with `docker compose up -d --force-recreate`.

## ROS 2 networking

Compose enables `network_mode: host`, `ipc: host`, `privileged: true`, and full
`/dev` access as requested. Host networking uses the Ubuntu host's interfaces
and IP address, with no Docker port mappings. Shared IPC supports DDS shared
memory with compatible containers on this host. Privileged mode grants broad
host/device access; LAN discovery itself relies on networking and DDS settings.

Every ROS participant must use the same `ROS_DOMAIN_ID` and permit non-local
communication (`ROS_LOCALHOST_ONLY=0`). This project defaults to domain **0**,
since milliAmpere's Compose does not explicitly set a domain. The older ZED
instructions used **42**: check the actual running systems. `.env` here applies
only to this Compose project; configure the boat, simulator, and remote PCs too.

Fast DDS (`rmw_fastrtps_cpp`) is the default. Cyclone DDS is also installed and
can be selected in `.env` if the rest of the deployment uses it. Prefer matching
middleware across participants while diagnosing communication.

On each other computer, set these before launching ROS nodes:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

The network must pass DDS UDP discovery and data between PCs; check host
firewalls, multicast support, Wi-Fi client isolation, and VPN/interface routing
if topics are missing. A shared Wi-Fi name alone does not guarantee DDS access.
For a network that blocks multicast, arrange a DDS discovery server or peer
configuration for all participants; this container cannot override LAN policy.

References: [Docker host networking](https://docs.docker.com/engine/network/drivers/host/),
[ROS 2 environment settings](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).

## Inputs, frames, and simulation

Edit `ma1-live.yaml` to match the actual input topic and TF frames. Defaults:

- LiDAR: `/front_lidar/ouster/points` (`sensor_msgs/msg/PointCloud2`).
- Local water frame: `ma1_water_plane`, with positive Z up.
- Global output frame: `piren`.
- Transform lookup: LiDAR measurement timestamp (`global_tf_use_latest: false`).

The LiDAR frame must connect to the water frame through `/tf` and `/tf_static`.
Global output additionally needs the vessel pose chain into `piren`. Missing
local TF produces empty outputs with warnings; missing global TF skips global
JSON while local output continues. Synchronize clocks across live computers.

The detector does not create mounting transforms automatically. The package's
`ma1_mounts.launch.py` contains provisional mounts: verify geometry and frame
ownership before running it, and do not duplicate existing TF publishers.
If those mounts are verified and absent, run in a separate terminal:

```bash
docker compose run --rm --no-deps shoreline \
  ros2 launch nearest_obstacle_shoreline_detector ma1_mounts.launch.py
```

For simulator input, set `USE_SIM_TIME=true` in `.env` and edit the LiDAR topic
and frames to match the simulator. The simulator must publish `/clock` and the
required TF chain. This detector does not generate simulated sensor data.

For recorded mA1 data, set these in `.env` and recreate the service:

```dotenv
USE_SIM_TIME=true
PARAMS_FILE=/config/package/ma1.yaml
```

Play the bag from the recording computer with `ros2 bag play BAG_PATH --clock`.
The recorded mA1 config uses the latest global transform to accommodate
sensor-relative timestamps. This is approximate when the vessel moves; it does
not synchronize the original scan and pose. Live operation defaults to timestamp
matching instead. `/config/package/lidar_projection.yaml` is also available for
the original mA2 topic/frame settings.

## Verify communication and output

From this directory, while the detector and sensor/simulator publishers run:

```bash
docker compose exec shoreline ros2 topic list
docker compose exec shoreline ros2 topic info /front_lidar/ouster/points --verbose
docker compose exec shoreline ros2 topic echo /front_lidar/ouster/points \
  --field header --once --qos-reliability best_effort
docker compose exec shoreline ros2 run tf2_ros tf2_echo ma1_water_plane os_lidar
docker compose exec shoreline ros2 topic echo /lidar_water_projection/barrier_json --once
docker compose exec shoreline ros2 topic echo /lidar_water_projection/barrier_json_piren --once
```

Replace the input topic and `os_lidar` with the real message's `header.frame_id`.
Run the output echo commands on another PC as well to verify actual LAN data
delivery, rather than relying only on topic discovery. An empty boundary may
also mean no points pass filtering; inspect the JSON `status` and detector logs.

Outputs include filtered/projected/boundary point clouds, a MarkerArray barrier,
local `/lidar_water_projection/barrier_json`, and global
`/lidar_water_projection/barrier_json_piren`. This is a nearest supported obstacle
boundary; boats and other objects can contribute to it.
