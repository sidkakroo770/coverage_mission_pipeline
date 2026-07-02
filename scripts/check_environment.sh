#!/usr/bin/env bash
failures=0
pass() { printf 'PASS: %s\n' "$1"; }
fail() { printf 'FAIL: %s\n' "$1"; failures=$((failures + 1)); }
warn() { printf 'WARN: %s\n' "$1"; }

if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  [[ "${VERSION_ID:-}" == "22.04" ]] && pass "Ubuntu 22.04" || warn "reference OS is Ubuntu 22.04; detected ${PRETTY_NAME:-unknown}"
fi
command -v ros2 >/dev/null 2>&1 && pass "ros2" || fail "ros2 not found"
[[ "${ROS_DISTRO:-}" == "humble" ]] && pass "ROS_DISTRO=humble" || fail "expected ROS_DISTRO=humble"
command -v colcon >/dev/null 2>&1 && pass "colcon" || fail "colcon not found"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"
[[ -f "$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py" ]] && pass "ArduPilot SITL" || fail "missing sim_vehicle.py under $ARDUPILOT_HOME"
(command -v mavproxy.py >/dev/null 2>&1 || command -v mavproxy >/dev/null 2>&1) && pass "MAVProxy" || fail "MAVProxy not found"
python3 - <<'PY' >/dev/null 2>&1 && pass "Python dependencies" || fail "wx/pymavlink/Shapely/pyproj/PyYAML import failed"
import wx
from pymavlink import mavutil
import shapely
import pyproj
import yaml
PY
ros2 pkg prefix coverage_mission_pipeline >/dev/null 2>&1 && pass "coverage_mission_pipeline" || fail "package not built/sourced"
[[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] && pass "graphical display" || fail "no display environment"
exit "$failures"
