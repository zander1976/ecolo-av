# aav_kiss_icp

## Overview

**How far did the car move, from the lidar alone?** KISS-ICP (MIT, PRBonn) matches each
scan to a small map of the last few. No saved map, no GPS. A trial: scored beside
`ekf_local`, never TF, never in an EKF by default. The design:
`docs/designs/2026-10-30_kiss-icp.md`.

```
/sensors/lidar/points ─► kiss_icp ─► pipe ─► worker.py (kiss env: KISS-ICP)
                            │
                            ├─► /localization/kiss_icp/odom
                            └─► /localization/kiss_icp/status
```

KISS-ICP comes from its own PyPI wheel (`kiss-icp==1.3.0`) in the Pixi environment
`kiss`: Linux (x86_64, aarch64) and Apple-silicon Macs. Install it once:

```sh
pixi install --locked -e kiss
```

## Interface

### Node `kiss_icp`

| In / out | Topic | Type |
|---|---|---|
| in | `/sensors/lidar/points` | `PointCloud2`, `lidar_link`; reliable, keep last 1 |
| out | `/localization/kiss_icp/odom` | `Odometry`: frame `kiss_odom`, child `base_link`; pose, and twist in `base_link` |
| out | `/localization/kiss_icp/status` | `DiagnosticStatus`: `match_ms`, `points`, `deskew`, `clouds`, `clouds_dropped`, `restarts` |

| Parameter | Default | Means |
|---|---|---|
| `kiss_python` | `''` | the kiss env's Python; empty: `<repo>/.pixi/envs/kiss/bin/python` |
| `min_height` | 0.3 m | above `base_link`: lower is the ground, cut out (a flat ground's rings say "no motion") |
| `min_range`, `max_range` | 0.5, 100 m | from the lidar |
| `voxel_size` | 0 | 0: `max_range / 100` (KISS-ICP's rule: 1 m) |
| `max_points_per_voxel` | 20 | KISS-ICP's default |
| `threads` | 4 | KISS-ICP's threads |
| `deskew` | true | only where the cloud has a `time` field (the VLP-16; Gazebo's has none) |
| `lidar_period` | 0.1 s | for counting clouds dropped |
| `lidar_xyz`, `lidar_rpy`, `body_min`, `body_max` | from `vehicle.yaml` | set by the launch file |

### Launch

| File | Does |
|---|---|
| `kiss_icp.launch.py [kiss_python:=]` | the node; refuses with no kiss env, and says how to install it |
| `sim.launch.py kiss_icp:=true` (aav_bringup) | includes it, in any world with the lidar |

### Modules

| Module | No ROS? | Holds |
|---|---|---|
| `odometry.py` | yes | the cloud's bytes, the mount, the body cut out, deskew times, pose and twist |
| `wire.py` | yes | the pipe's frames, both ways |
| `worker.py` | yes (the kiss env) | KISS-ICP itself |
| `kiss_icp_node.py` | no | the node |

## Invariants

- Never TF; nothing reads its odometry to drive.
- The scan is matched in `base_link` (the mount from `vehicle.yaml`), so the pose is the car's.
- The worker runs with `-I` and a clean environment: two environments' libraries never mix.
- A dead worker is restarted; the odometry carries on from the last pose (`restarts`).
- Covariances are fixed guesses: KISS-ICP gives none.

## Side Effects

- Starts one child process (the kiss env's Python). Reads `vehicle.yaml` at launch.

## Verification

```sh
cd ros/localization/aav_kiss_icp && python3 -m pytest -q   # the maths, the pipe, the worker
pixi run build && colcon test --packages-select aav_kiss_icp && colcon test-result --verbose
pixi run controller-compare --kiss-icp --tours lot --controllers mppi   # Gazebo, scored
```

The worker test runs only where the kiss env is installed.
