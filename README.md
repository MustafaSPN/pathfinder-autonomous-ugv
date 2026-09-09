# Pathfinder — Tracked UGV

Tracked skid-steer platform, ~1.0 × 0.74 m, 0.625 m between track centrelines.

This branch holds the complete ROS 2 source for this vehicle. For the project overview,
photos and demo clips, see the [`main` branch](../../tree/main).

## Platform

- **Drive** — tracked, differential / skid-steer
- **Positioning** — RTK GNSS at 20 Hz, dual-antenna heading
- **IMU** — BNO055 over UART
- **Perception** — Hesai XT16 3D LiDAR, used for the mission console's 2D view only; it is
  **not** wired into the Nav2 costmaps and this vehicle does not do autonomous obstacle
  avoidance
- **Extras on this branch** — `mission_logger.py` (JSONL mission recording),
  `calibrate_cerpm.py` (wheel-odometry scale calibration against RTK), `cyclonedds.xml`
  (DDS tuning for point-cloud transport)

## Stack

- **ROS 2 Humble** on Raspberry Pi 5, in Docker
- **Localization** — dual EKF (`robot_localization`): a local filter publishing
  `odom -> base_link`, `navsat_transform`, and a global filter publishing `map -> odom`
- **Positioning** — RTK GNSS with dual-antenna heading, via `swegeo_driver` (NTRIP client)
- **Navigation** — Nav2: NavFn planner + Regulated Pure Pursuit controller
- **Motor control** — `twist_mux` → `/cmd_vel_out` → ESP32-S3 running micro-ROS

## Layout

```
src/rover_bringup/       launch, URDF, EKF / Nav2 / twist_mux configuration
src/swegeo_driver/       GNSS driver - NTRIP client, position and heading
src/bno055/              vendored BNO055 IMU driver (third party)
apps/pathfinder_webapp/  FastAPI backend and Leaflet mission console
```

## RTK corrections

NTRIP settings come from the environment. No credentials are committed.

```bash
cp ntrip.env.example ntrip.env       # then fill in host, username, password
set -a && . ./ntrip.env && set +a
ros2 launch rover_bringup rover.launch.py
```

`ntrip.env` is git-ignored. `NTRIP_PORT` and `NTRIP_MOUNTPOINT` are pre-set in the
example file to the values this vehicle was tested with.

## Note

The micro-ROS motor-control firmware runs on the ESP32-S3 and is maintained as a
separate project; it is not vendored here. Navigation is outdoor GPS waypoint
following — no SLAM, no simulation, and no obstacle sensor is wired into the costmaps.
