#!/usr/bin/env bash
set -Eeo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./coverage-one-drone-sitl.sh [options] [mission.kml]

Options:
  --replace-running       Stop stale sim_vehicle, ArduCopter, and MAVProxy processes.
  --speedup N             SITL speedup; default: 2.
  --clearance-m N         Physical clearance; default: 2.
  --tracking-margin-m N   Additional tracking reserve; default: 0.
  --min-component-area N  Minimum component area; default: 250.
  --timeout-s N           Mission execution timeout; default: 1800.
  --keep-run              Keep processes alive after successful completion.
  -h, --help              Show this help.

Default KML:
  ~/Downloads/CSED_mission.kml
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNNER="$SCRIPT_DIR/run_one_drone_sitl.py"

KML="$HOME/Downloads/CSED_mission.kml"
REPLACE_RUNNING=0
KEEP_RUN=0
SPEEDUP="2"
CLEARANCE_M="2"
TRACKING_MARGIN_M="0"
MIN_COMPONENT_AREA_M2="250"
TIMEOUT_S="1800"

while (($#)); do
  case "$1" in
    --replace-running)
      REPLACE_RUNNING=1
      shift
      ;;
    --keep-run)
      KEEP_RUN=1
      shift
      ;;
    --speedup)
      SPEEDUP="${2:?missing value for --speedup}"
      shift 2
      ;;
    --clearance-m)
      CLEARANCE_M="${2:?missing value for --clearance-m}"
      shift 2
      ;;
    --tracking-margin-m)
      TRACKING_MARGIN_M="${2:?missing value for --tracking-margin-m}"
      shift 2
      ;;
    --min-component-area)
      MIN_COMPONENT_AREA_M2="${2:?missing value for --min-component-area}"
      shift 2
      ;;
    --timeout-s)
      TIMEOUT_S="${2:?missing value for --timeout-s}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      echo "ERROR: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      KML="$1"
      shift
      ;;
  esac
done

KML="$(readlink -f "$KML")"
[[ -f "$KML" ]] || { echo "ERROR: KML not found: $KML" >&2; exit 2; }
[[ -f "$RUNNER" ]] || { echo "ERROR: runner missing: $RUNNER" >&2; exit 2; }

ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"
ARDUCOPTER_BIN="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
COPTER_DEFAULTS="$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm"

[[ -x "$ARDUCOPTER_BIN" ]] || {
  echo "ERROR: ArduCopter SITL binary not found or not executable: $ARDUCOPTER_BIN" >&2
  echo "Build it with: cd $ARDUPILOT_HOME && ./waf configure --board sitl && ./waf copter" >&2
  exit 2
}

[[ -f "$COPTER_DEFAULTS" ]] || {
  echo "ERROR: Copter defaults file not found: $COPTER_DEFAULTS" >&2
  exit 2
}

# This launcher is Bash, so Bash setup scripts are correct here.
source /opt/ros/humble/setup.bash
source "$HOME/coverage_ws/install/setup.bash"
set -u

if command -v coverage-swarm >/dev/null 2>&1; then
  PRODUCT_CLI=(coverage-swarm)
else
  PRODUCT_CLI=(ros2 run coverage_mission_pipeline coverage-swarm)
fi

timestamp="$(date +%Y%m%d-%H%M%S)"
RUN_DIR="$HOME/coverage_ws/mission_runs/CSED_1drone_single-$timestamp"
PREPARED="$RUN_DIR/prepared"
PLANNED="$RUN_DIR/planned"
LOGS="$RUN_DIR/logs"
SITL_DIR="$RUN_DIR/sitl"
FLEET_CONFIG="$RUN_DIR/fleet_sitl_1.yaml"
UPLOAD_REPORT="$RUN_DIR/one-drone-upload-report.json"
LOCATION_FILE="$RUN_DIR/ardupilot-locations.txt"

mkdir -p "$PREPARED" "$PLANNED" "$LOGS" "$SITL_DIR"

PIDS=()

start_managed() {
  local name="$1"
  local log="$2"
  shift 2

  setsid "$@" >"$log" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  echo "$pid" >"$RUN_DIR/$name.pid"
  echo "  started $name pid=$pid log=$log"
}

cleanup() {
  local status=$?
  set +e

  if ((KEEP_RUN == 1 && status == 0)); then
    echo "Keeping managed processes alive because --keep-run was set."
    return
  fi

  if ((${#PIDS[@]})); then
    echo
    echo "Stopping managed processes..."
  fi

  local i pid
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    pid="${PIDS[$i]}"
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    fi
  done

  sleep 2

  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    pid="${PIDS[$i]}"
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
  done

  sleep 1

  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    pid="${PIDS[$i]}"
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    fi
  done

  return "$status"
}

show_failure_logs() {
  local status=$?
  if ((status != 0)) && [[ -f "$LOGS/sitl.log" ]]; then
    echo
    echo "===== ArduCopter SITL log tail =====" >&2
    tail -n 120 "$LOGS/sitl.log" >&2 || true
  fi
  return "$status"
}

trap show_failure_logs ERR
trap cleanup EXIT
trap 'exit 130' INT TERM

port_listening() {
  local port="$1"
  ss -H -ltn 2>/dev/null \
    | awk '{print $4}' \
    | grep -Eq "(^|:)${port}$"
}

wait_for_port() {
  local port="$1"
  local timeout_s="$2"
  local deadline=$((SECONDS + timeout_s))

  while ((SECONDS < deadline)); do
    if port_listening "$port"; then
      return 0
    fi
    sleep 0.5
  done

  echo "ERROR: timeout waiting for TCP port $port" >&2
  return 1
}

wait_for_service() {
  local service="$1"
  local timeout_s="$2"
  local deadline=$((SECONDS + timeout_s))

  while ((SECONDS < deadline)); do
    if ros2 service list 2>/dev/null | grep -Fxq "$service"; then
      return 0
    fi
    sleep 0.5
  done

  echo "ERROR: timeout waiting for ROS service $service" >&2
  return 1
}

if ((REPLACE_RUNNING == 1)); then
  echo "[0/10] Stopping stale SITL processes"
  pkill -INT -f sim_vehicle.py 2>/dev/null || true
  pkill -TERM -x arducopter 2>/dev/null || true
  pkill -TERM -f mavproxy.py 2>/dev/null || true
  pkill -TERM -f MAVProxy 2>/dev/null || true
  pkill -TERM -x coverage_planner 2>/dev/null || true
  pkill -TERM -f "ros2 run polygon_coverage_ros2 coverage_planner" 2>/dev/null || true
  sleep 2
elif port_listening 5760; then
  echo "ERROR: TCP port 5760 is already occupied." >&2
  echo "Rerun with --replace-running to stop stale SITL processes." >&2
  exit 2
fi

echo "Run directory: $RUN_DIR"
echo "KML: $KML"

echo "[1/10] Starting coverage planner"
start_managed \
  planner \
  "$LOGS/coverage-planner.log" \
  ros2 run polygon_coverage_ros2 coverage_planner
wait_for_service /plan_coverage 30

echo "[2/10] Validating N=1 KML mission"
"${PRODUCT_CLI[@]}" validate "$KML" \
  --drones 1 \
  --clearance-m "$CLEARANCE_M" \
  --tracking-margin-m "$TRACKING_MARGIN_M" \
  --min-component-area-m2 "$MIN_COMPONENT_AREA_M2"

echo "[3/10] Preparing N=1 mission"
rm -rf "$PREPARED"
"${PRODUCT_CLI[@]}" prepare "$KML" \
  --drones 1 \
  --clearance-m "$CLEARANCE_M" \
  --tracking-margin-m "$TRACKING_MARGIN_M" \
  --min-component-area-m2 "$MIN_COMPONENT_AREA_M2" \
  --output "$PREPARED"

echo "[4/10] Generating boustrophedon route and ArduPilot mission"
rm -rf "$PLANNED"
ros2 run coverage_mission_pipeline run_swarm_mission \
  --mission-json "$PREPARED/mission_output.json" \
  --config "$PREPARED/swarm_mission.yaml" \
  --output "$PLANNED" \
  --service-wait-timeout 30 \
  --request-timeout 180

MISSION_PATH="$(
  find "$PLANNED/ardupilot" \
    -maxdepth 1 \
    -name '*.ardupilot-mission.json' \
    -print
)"

MISSION_COUNT="$(printf '%s\n' "$MISSION_PATH" | sed '/^$/d' | wc -l)"
[[ "$MISSION_COUNT" -eq 1 ]] || {
  echo "ERROR: expected exactly one ArduPilot mission, found $MISSION_COUNT" >&2
  exit 3
}

python3 - "$MISSION_PATH" "$LOCATION_FILE" <<'PY'
import json
import sys
from pathlib import Path

mission_path = Path(sys.argv[1])
location_path = Path(sys.argv[2])
mission = json.loads(mission_path.read_text(encoding="utf-8"))

if mission.get("vehicle_id") != "drone-1":
    raise SystemExit("mission vehicle_id is not drone-1")

items = mission.get("items")
if not isinstance(items, list) or len(items) < 3:
    raise SystemExit("mission does not contain at least three items")

commands = [int(item["command"]) for item in items]
if commands[0] != 22:
    raise SystemExit("first semantic command is not TAKEOFF")
if commands[-1] != 21:
    raise SystemExit("final semantic command is not LAND")
if 20 in commands:
    raise SystemExit("mission contains RTL")

home = items[0]
lat = float(home["latitude_deg"])
lon = float(home["longitude_deg"])
location_path.write_text(
    f"CSEDOneDrone={lat:.9f},{lon:.9f},200.0,0.0\n",
    encoding="utf-8",
)

print(f"PASS: semantic mission verified, {len(items)} items")
print(f"HOME: {lat:.9f}, {lon:.9f}")
PY

echo "[5/10] Starting ArduCopter SITL binary directly"

HOME_SPEC="$(cut -d= -f2- "$LOCATION_FILE")"
[[ -n "$HOME_SPEC" ]] || {
  echo "ERROR: could not derive SITL HOME from $LOCATION_FILE" >&2
  exit 3
}

start_managed \
  sitl \
  "$LOGS/sitl.log" \
  "$ARDUCOPTER_BIN" \
    -w \
    --model + \
    "--speedup=$SPEEDUP" \
    --slave 0 \
    --defaults "$COPTER_DEFAULTS" \
    --sim-address=127.0.0.1 \
    -I0 \
    --home "$HOME_SPEC" \
    --sysid 1

SITL_PID="${PIDS[$((${#PIDS[@]} - 1))]}"
sleep 2

if ! kill -0 "$SITL_PID" 2>/dev/null; then
  echo "ERROR: ArduCopter exited during startup." >&2
  cat "$LOGS/sitl.log" >&2 || true
  exit 4
fi

wait_for_port 5760 60

if ! kill -0 "$SITL_PID" 2>/dev/null; then
  echo "ERROR: ArduCopter exited after opening port 5760." >&2
  cat "$LOGS/sitl.log" >&2 || true
  exit 4
fi

echo "[6/10] Using direct MAVLink connection to SITL on tcp:127.0.0.1:5760"
sleep 3

cat >"$FLEET_CONFIG" <<'YAML'
schema_version: 2
profile: sitl
source_system: 250
source_component: 190
allow_duplicate_missions: false

timeouts:
  heartbeat_s: 15.0
  ready_s: 120.0
  ready_heartbeats: 3
  startup_grace_s: 5.0
  clear_ack_s: 10.0
  item_request_s: 10.0
  upload_ack_s: 15.0
  download_item_s: 10.0
  retries: 3

vehicles:
  - vehicle_id: drone-1
    endpoint: tcp:127.0.0.1:5760
    system_id: 1
    component_id: 1
YAML

[[ -s "$FLEET_CONFIG" ]] || {
  echo "ERROR: failed to create fleet configuration: $FLEET_CONFIG" >&2
  exit 3
}

echo "  Fleet configuration: $FLEET_CONFIG"

echo "[7/10] Uploading and verifying one mission; vehicle remains disarmed"
echo "  Direct TCP upload: waiting for stable STANDBY, uploading the wire mission,"
echo "  downloading it again, and verifying the fingerprint."
rm -f "$UPLOAD_REPORT"
ros2 run coverage_mission_pipeline upload_swarm_missions \
  --missions "$PLANNED" \
  --fleet-config "$FLEET_CONFIG" \
  --report "$UPLOAD_REPORT"

python3 - "$UPLOAD_REPORT" <<'PY'
import json
import sys
from pathlib import Path

report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
assert report["all_verified"] is True
assert report["vehicle_count"] == 1
assert report["vehicles"][0]["vehicle_id"] == "drone-1"
print("PASS: upload/readback report verified")
PY

# Let ArduPilot close the upload TCP client before the executor reconnects.
sleep 5

echo "[8/10] Executing one-drone AUTO mission"
echo "  The runner will wait for GPS/EKF position lock and pre-arm health,"
echo "  provide verified low-throttle SITL RC, then arm normally in LOITER."
python3 "$RUNNER" \
  --endpoint tcp:127.0.0.1:5760 \
  --system-id 1 \
  --component-id 1 \
  --timeout-s "$TIMEOUT_S" \
  --max-final-home-distance-m 10

echo "[9/10] Mission completed, landed, and disarmed"
echo "[10/10] PASS"
echo "Reports, missions, and logs: $RUN_DIR"
