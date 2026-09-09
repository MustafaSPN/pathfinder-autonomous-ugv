# Shared Autonomy Architecture

All three vehicles run the same two ROS 2 packages, the same launch files and the same
localization and navigation topology. What changes per vehicle is configuration:
kinematic limits, EKF inputs, costmap footprint and controller tuning.

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="../media/architecture-detailed-dark.png">
    <img alt="Detailed Pathfinder autonomy architecture showing RTK GNSS, IMU and wheel odometry feeding the local EKF, navsat_transform and global EKF, then Nav2, twist_mux, /cmd_vel_out, the ESP32-S3 micro-ROS bridge and the vehicle motor interface, with the web mission console attached to Nav2." src="../media/architecture-detailed-light.png" width="760">
  </picture>
</p>

<details>
<summary>Diagram source (Mermaid)</summary>

```mermaid
flowchart TD
    GNSS["RTK GNSS<br/>position + dual-antenna heading"]
    IMU["IMU (BNO055)"]
    ODO["Wheel odometry /odom_esp<br/>Tracked and 4x4 only"]

    IMU --> EKFL["Local EKF<br/>publishes odom to base_link"]
    ODO --> EKFL
    EKFL --> NAVSAT["navsat_transform<br/>lat/lon into the map frame"]
    GNSS --> NAVSAT
    NAVSAT --> EKFG["Global EKF<br/>publishes map to odom"]
    GNSS --> EKFG
    IMU --> EKFG
    EKFG --> NAV2["Nav2<br/>NavFn planner + Regulated Pure Pursuit"]
    NAV2 --> MUX["twist_mux<br/>nav2 / joystick / e-stop priority"]
    MUX --> CMD["/cmd_vel_out"]
    CMD --> ESP["ESP32-S3 · micro-ROS"]
    ESP --> MOTOR["Vehicle-specific motor interface"]
    CONSOLE["Web Mission Console<br/>FastAPI · WebSocket · Leaflet"] <--> NAV2
```

Rendered with `mmdc -i architecture-detailed.mmd -o ../media/architecture-detailed-light.png -t default -b "#ffffff" -w 1000 -s 2`
(and `-t dark -b "#0d1117"` for the dark variant). The images are committed because
GitHub renders Mermaid client-side, which does not run in the GitHub mobile app.

</details>

## The abstraction boundary

The boundary between shared autonomy and vehicle-specific hardware is exactly two topics:

| Topic | Direction | Meaning |
|---|---|---|
| `/cmd_vel_out` | autonomy → vehicle | the arbitrated velocity command, after Nav2, the velocity smoother and `twist_mux` |
| `/odom_esp` | vehicle → autonomy | wheel odometry measured by the embedded controller |

Everything above that line — the dual EKF, `navsat_transform`, Nav2, the mission console —
is identical across the fleet. Everything below it is vehicle-specific: tracks, four
wheels, or a steering servo. Porting the stack to a new platform means writing the
firmware behind that boundary and retuning the layer above it, not rewriting the autonomy.

## Where the vehicles genuinely differ

| | Tracked UGV | 4x4 UGV | RC Crawler |
|---|---|---|---|
| `/odom_esp` present | yes | yes | **no — no wheel encoders** |
| Local EKF inputs | wheel odometry + gyro | wheel odometry + gyro | gyro only, accelerometer disabled |
| Absolute position | RTK GNSS | RTK GNSS | RTK GNSS (the only position source) |
| Heading | dual-antenna GNSS (absolute) + gyro (relative) | same | same |

The RC Crawler is the interesting case. With no encoders there is no dead-reckoning
source at all, and integrating the accelerometer twice with no absolute reference would
let position run away. So its local filter is reduced to a heading estimator and every
bit of position information arrives through RTK GNSS via the global filter. The
architecture absorbs that without structural change — one input is simply switched off.

## Frames

```
map  --(global EKF)-->  odom  --(local EKF)-->  base_link
                                                   |
                     base_footprint, gps_link, imu_link,
                     wheel / track links, hesai_lidar (Tracked UGV)
```

`map -> odom` is published by the global EKF, `odom -> base_link` by the local EKF, and
the static vehicle frames come from `robot_state_publisher` reading that vehicle's URDF.

## Scope

Navigation is outdoor GPS waypoint following. There is no SLAM and no map building — the
costmaps run against a blank latched grid. There is no simulation. No obstacle sensor is
wired into the costmaps, so there is no autonomous obstacle avoidance; the Tracked UGV's
LiDAR feeds the mission console's 2D view only.
