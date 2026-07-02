# Safety-First Five-Drone Coverage Mission Pipeline

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E)](https://docs.ros.org/en/humble/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-1D3557)](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache--2.0-green)](LICENSE)

A ROS 2 mission-generation and deployment pipeline for **polygon coverage by a five-drone ArduPilot fleet**. It preserves disconnected geometry, creates flight-safe connectors around exclusions, exports independent Copter missions, verifies each upload by downloading it again, and executes a staggered shared-HOME simulation from one terminal.

```bash
coverage-swarm simulate-existing --execute --replace-running --hold-map
```

> **Current scope:** the checked-in Noida Stage 21 scenario is a reproducible five-drone SITL demo. The KML/KMZ front end currently supports validation and preparation; arbitrary KML-to-flight integration remains under development.

## Why this project exists

Coverage sweeps are only part of a real mission. The fleet also needs safe transitions from HOME, between disconnected components, around no-go zones, and back to HOME. It also needs mission identity, transport precision, upload verification, launch coordination and a terminal action that does not bypass the planner.

## Highlights

- Five independent ArduPilot missions with explicit vehicle identity.
- Polygon/MultiPolygon preservation: connected components are never silently dropped.
- Clearance applied to the global boundary and exclusions, not shared partition borders.
- Deterministic polygon-vertex visibility-graph A* connectors.
- Geometric return to HOME followed by explicit `MAV_CMD_NAV_LAND`; no terminal RTL.
- Transactional MAVLink upload, download and mission fingerprint verification.
- Fail-closed no-arming telemetry preflight.
- Staggered shared-HOME launch gate and live separation monitoring.
- One-terminal SITL supervisor for five vehicles, routers, live KML map and cleanup.

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
    G --> H[Five complete routes]
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

## Commands

```text
coverage-swarm validate <file.kml|file.kmz>
coverage-swarm prepare <file.kml|file.kmz> --output <new-directory>
coverage-swarm simulate-existing [--execute] [--replace-running]
```

`validate` and `prepare` do not connect to or arm vehicles.

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
- [Contributing](CONTRIBUTING.md)

## License

Apache License 2.0. External projects such as ArduPilot, MAVProxy, ROS 2 and `polygon_coverage_ros2` retain their own licenses.
