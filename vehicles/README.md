# Vehicles

Each vehicle's complete ROS 2 source lives on its own branch. This page is an index;
the case studies with photos and demos are on the [main README](../README.md).

| Vehicle | Branch | Drive | Wheel odometry | Distinguishing work |
|---|---|---|---|---|
| **Tracked UGV** | [`paletli`](../../../tree/paletli) | Tracked skid-steer | Yes, RTK-calibrated scale | Velocity budgeting across two tracks; LiDAR transport tuning |
| **4x4 UGV** | [`rover-4x4`](../../../tree/rover-4x4) | 4-wheel differential | Yes, via dual RoboClaw | Heading and datum correctness; binary GNSS protocol parsing |
| **RC Crawler** | [`rc-crawler`](../../../tree/rc-crawler) | Ackermann steering | **None** | Autonomy with no odometry source; Ackermann-safe Nav2 |

## What is the same on every branch

- `src/rover_bringup` — launch, URDF, EKF / Nav2 / `twist_mux` configuration
- `src/swegeo_driver` — GNSS receiver driver with NTRIP client and dual-antenna heading
- `apps/pathfinder_webapp` — FastAPI + WebSocket + Leaflet mission console
- Dual-EKF localization, NavFn planner, Regulated Pure Pursuit controller
- The `/cmd_vel_out` and `/odom_esp` boundary to the embedded motor controller

## What differs

Kinematic limits, EKF input matrices and sensor rates, costmap footprint and inflation,
controller tuning, URDF geometry, behavior-tree selection, and per-vehicle diagnostic
tooling. See [docs/architecture](../docs/architecture/README.md).

## RTK corrections

Every branch expects NTRIP settings in the environment. Copy that branch's
`ntrip.env.example` to `ntrip.env`, fill in your own caster and account, and source it
before launching. `ntrip.env` is git-ignored and no credentials are committed anywhere in
this repository.
