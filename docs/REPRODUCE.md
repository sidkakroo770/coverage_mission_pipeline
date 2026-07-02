# Reproducing the five-drone SITL demo

This guide targets a clean Ubuntu 22.04 graphical installation.

## 1. Install ROS 2 Humble

Follow the official Ubuntu deb guide:

- https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Install the desktop package and development tools. Confirm:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
colcon --help
```

## 2. Install and build ArduPilot SITL

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```

Official references:

- https://ardupilot.org/dev/docs/building-setup-linux.html
- https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
- https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html

Verify graphical MAVProxy dependencies:

```bash
command -v mavproxy.py || command -v mavproxy
python3 - <<'PY'
import wx
from pymavlink import mavutil
print("wxPython and pymavlink are available")
PY
```

## 3. Clone and build the ROS workspace

```bash
mkdir -p ~/coverage_ws/src
cd ~/coverage_ws/src

git clone https://github.com/sidkakroo770/polygon_coverage_ros2.git
git clone https://github.com/sidkakroo770/coverage_mission_pipeline.git

cd ~/coverage_ws
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --base-paths src \
  --symlink-install \
  --packages-up-to coverage_mission_pipeline

source ~/coverage_ws/install/setup.bash
```

Set this only when ArduPilot is not at `~/ardupilot`:

```bash
export ARDUPILOT_HOME=/absolute/path/to/ardupilot
```

## 4. Install the convenient commands

```bash
~/coverage_ws/src/coverage_mission_pipeline/scripts/install_user_launcher.sh
export PATH="$HOME/.local/bin:$PATH"
coverage-swarm-check
```

## 5. Run the no-arming check

```bash
coverage-swarm simulate-existing --replace-running
```

Expected ending:

```text
[5/8] Five mission uploads verified
[6/8] No-arming telemetry preflight passed
[7/8] READY — no vehicle was armed
```

## 6. Execute the complete demo

```bash
coverage-swarm simulate-existing \
  --execute \
  --replace-running \
  --hold-map
```

Expected completion:

```text
[8/8] Simulation completed: all five vehicles landed and disarmed
```

The supervisor creates a run-local ArduPilot location from the checked-in mission HOME. It does not require editing ArduPilot's global `locations.txt`.

## 7. Inspect the evidence

Runs are written under:

```text
~/coverage_ws/mission_runs/noida_stage22/existing-simulation-YYYYMMDD-HHMMSS/
```

Important outputs include:

```text
simulation-upload-report.json
simulation-telemetry-dry-run-report.json
simulation-fleet-execution-report.json
simulation-fleet-checkpoint.json
simulation-fleet-telemetry.csv
simulation-fleet-events.jsonl
logs/map.log
logs/sitl.log
logs/routers/
```

## Troubleshooting

### Map exits immediately

Inspect `logs/map.log`. MAVProxy exits when its command input reaches EOF; the supervisor keeps a private stdin pipe open to prevent that.

### No map window

```bash
echo "$DISPLAY"
python3 -c 'import wx; print(wx.version())'
```

Use a graphical desktop or correctly configured X forwarding.

### Ports already occupied

```bash
coverage-swarm simulate-existing --replace-running
```

### `Input/output error` from unrelated executables

Treat that as an operating-system/filesystem issue, not a ROS error. Reboot first; if it persists, inspect disk space, mount state and kernel storage errors.

## Reproducibility boundary

The demo replays the audited, already-generated Stage 21 missions. It does not invoke the coverage planner during `simulate-existing`. Arbitrary KML-to-flight integration is still in development.
