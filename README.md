# Safety-First N-Drone Coverage Mission Pipeline

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E)](https://docs.ros.org/en/humble/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-1D3557)](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache--2.0-green)](LICENSE)

A ROS 2 mission-generation and deployment pipeline for **polygon coverage by a configurable ArduPilot fleet**. It preserves disconnected geometry, creates flight-safe connectors around exclusions, exports independent Copter missions, verifies each upload by downloading it again, and retains a reproducible five-drone shared-HOME SITL demonstration.

```bash
coverage-swarm simulate-existing --execute --replace-running --hold-map
```

> **Current scope:** KML/KMZ validation and preparation accept a user-selected positive drone count. The checked-in Noida Stage 21 `simulate-existing` scenario remains an intentionally fixed, reproducible five-drone SITL demo. One-command arbitrary KML-to-flight execution remains under development.

## Why this project exists

Coverage sweeps are only part of a real mission. The fleet also needs safe transitions from HOME, between disconnected components, around no-go zones, and back to HOME. It also needs mission identity, transport precision, upload verification, launch coordination and a terminal action that does not bypass the planner.

## Highlights

- Configurable `N`-drone mission inputs with explicit vehicle identity.
- Polygon/MultiPolygon preservation: connected components are never silently dropped.
- Clearance applied to the global boundary and exclusions, not shared partition borders.
- Deterministic polygon-vertex visibility-graph A* connectors.
- Geometric return to HOME followed by explicit `MAV_CMD_NAV_LAND`; no terminal RTL.
- Transactional MAVLink upload, download and mission fingerprint verification.
- Fail-closed no-arming telemetry preflight.
- Staggered shared-HOME launch gate and live separation monitoring.
- One-terminal SITL supervisor for five vehicles, routers, live KML map and cleanup.

- One-terminal arbitrary-KML mission generation, verified upload, GPS/EKF readiness gating, AUTO execution, LAND and disarm for one ArduCopter SITL vehicle.

## Audited Stage 21 result

| Result | Value |
|---|---:|
| Vehicles completed, landed and disarmed | 5 / 5 |
| Planned unsafe points / segments | 0 / 0 |
| Sampled flown unsafe points / segments | 0 / 0 |
| Worst observed tracking error | 0.919 m |
| Tracking reserve | 2.000 m |
| Minimum observed 3D fleet separation | 24.396 m |
| Runtime separation warnings | 0 |
| Missions containing RTL | 0 |

The flown-trajectory audit sampled telemetry at approximately 0.5 s and tested straight segments between consecutive samples. See [`docs/VALIDATION.md`](docs/VALIDATION.md).

## Architecture

```mermaid
flowchart LR
    A[Boundary + exclusions + partitions] --> B[Authoritative safe space]
    B --> C[Connected planner requests]
    C --> D[polygon_coverage_ros2]
    D --> E[Georeferenced routes]
    E --> F[Vehicle ordering + orientation DP]
    F --> G[Visibility-graph connectors]
    G --> H[N complete routes]
    H --> I[ArduPilot export]
    I --> J[MAVLink upload/readback]
    J --> K[No-arming preflight]
    K --> L[Staggered SITL execution]
    L --> M[LAND at HOME]
```

## Reference environment

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- ArduPilot / ArduCopter SITL, validated with ArduCopter 4.6.2
- MAVProxy with wxPython map support
- Graphical desktop session

Full clean-machine guide: [`docs/REPRODUCE.md`](docs/REPRODUCE.md).

## Quick start

```bash
mkdir -p ~/coverage_ws/src
cd ~/coverage_ws/src

git clone https://github.com/sidkakroo770/polygon_coverage_ros2.git
git clone https://github.com/sidkakroo770/coverage_mission_pipeline.git

cd ~/coverage_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --symlink-install --packages-up-to coverage_mission_pipeline
source install/setup.bash

~/coverage_ws/src/coverage_mission_pipeline/scripts/install_user_launcher.sh
export PATH="$HOME/.local/bin:$PATH"
coverage-swarm-check
```

Safe no-arming integration check:

```bash
coverage-swarm simulate-existing --replace-running
```

Complete demo:

```bash
coverage-swarm simulate-existing \
  --execute \
  --replace-running \
  --hold-map
```

## One-drone arbitrary-KML SITL

The validated one-drone supervisor accepts a valid KML with an explicit HOME,
generates the N=1 boustrophedon route, starts ArduCopter SITL, performs
transactional mission upload/readback, waits for GPS/EKF position readiness,
arms normally, executes AUTO, and verifies LAND plus disarm.

```bash
cd ~/coverage_ws/src/coverage_mission_pipeline

bash scripts/one_drone_sitl/coverage-one-drone-sitl.sh \
  --replace-running \
  ~/Downloads/CSED_mission.kml
```

The runner does not force arm and does not disable arming checks. See
[`docs/ONE_DRONE_SITL_DIAGNOSIS.txt`](docs/ONE_DRONE_SITL_DIAGNOSIS.txt) for
the complete failure analysis and the SITL-versus-real-aircraft boundary.

## Commands

```text
coverage-swarm validate <file.kml|file.kmz> --drones N
coverage-swarm prepare <file.kml|file.kmz> --drones N --output <new-directory>
coverage-swarm simulate-existing [--execute] [--replace-running]
```

For example:

```bash
coverage-swarm prepare mission.kml \
  --drones 3 \
  --output ~/coverage_ws/mission_runs/three_drone_mission
```

`--drone-count N` is accepted as an alias for `--drones N`. If the KML already contains
`PARTITION_1` through `PARTITION_N`, their IDs must match the requested count exactly.
Otherwise, the pipeline creates `N` deterministic equal-route-area partitions.

The tracking reserve defaults to **2.0 m** through `--tracking-margin-m 2.0`.
This is separate from `--clearance-m`, which defines physical centreline clearance
from the mission boundary and exclusions.

`validate` and `prepare` do not connect to or arm vehicles. `simulate-existing`
continues to replay only the audited five-drone Stage 21 bundle.

## Tests

```bash
cd ~/coverage_ws/src/coverage_mission_pipeline
python3 -m pytest -q test
```

## Safety boundary

This is research software, not a certified flight-control system. The demo verifies a specific SITL scenario; it does not prove physical-aircraft airworthiness, collision avoidance or regulatory compliance. Read [`docs/SAFETY.md`](docs/SAFETY.md) before adapting it to hardware.

## Documentation

- [Reproduce the SITL demo](docs/REPRODUCE.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Safety model](docs/SAFETY.md)
- [Validation evidence](docs/VALIDATION.md)
- [Launch-post drafts](docs/LAUNCH_POSTS.md)
- [One-drone SITL diagnosis and verified solution](docs/ONE_DRONE_SITL_DIAGNOSIS.txt)
- [Contributing](CONTRIBUTING.md)

## License

Apache License 2.0. External projects such as ArduPilot, MAVProxy, ROS 2 and `polygon_coverage_ros2` retain their own licenses.
