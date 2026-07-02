from __future__ import annotations

import csv
import json
import math
import time
from itertools import combinations
from pathlib import Path
from typing import Any

import yaml
from pymavlink import mavutil


ROOT = Path.home() / "coverage_ws/mission_runs/noida_stage21c"

CONFIG_PATH = (
    Path.home()
    / "coverage_ws/mission_runs/noida_stage19a/fleet_sitl.yaml"
)

UPLOAD_REPORT_PATH = ROOT / "stage21c-upload-report.json"
DRY_RUN_REPORT_PATH = ROOT / "stage21c-telemetry-dry-run-report.json"

STATIC_AUDIT_PATH = (
    Path.home()
    / "coverage_ws/mission_runs/noida_stage21"
    / "stage21b-static-audit-report.json"
)

SOURCE_MISSION_DIR = (
    Path.home()
    / "coverage_ws/mission_runs/noida_stage21/output/ardupilot"
)

REPORT_PATH = ROOT / "stage21c-fleet-execution-report.json"
CHECKPOINT_PATH = ROOT / "stage21c-fleet-checkpoint.json"
TELEMETRY_PATH = ROOT / "stage21c-fleet-telemetry.csv"
EVENTS_PATH = ROOT / "stage21c-fleet-events.jsonl"

SOURCE_SYSTEM = 250
SOURCE_COMPONENT = 197

CONNECT_TIMEOUT_S = 30.0
COMMAND_TIMEOUT_S = 20.0
LAUNCH_GATE_TIMEOUT_S = 600.0
FLEET_TIMEOUT_S = 3600.0

TAKEOFF_WIRE_SEQUENCE = 1

LAUNCH_CLEARANCE_ALTITUDE_M = 12.0
LAUNCH_CLEARANCE_HOME_DISTANCE_M = 30.0
LAUNCH_GATE_STABLE_SAMPLES = 10
MINIMUM_LAUNCH_SPACING_S = 5.0

AIRBORNE_ALTITUDE_M = 3.0
MAXIMUM_PRELAUNCH_HOME_DISTANCE_M = 2.0
MAXIMUM_FINAL_HOME_DISTANCE_M = 10.0

HEARTBEAT_STALE_S = 8.0
POSITION_STALE_S = 5.0

TELEMETRY_SNAPSHOT_INTERVAL_S = 0.5
PROGRESS_INTERVAL_S = 5.0
CHECKPOINT_INTERVAL_S = 5.0

SEPARATION_WARNING_HORIZONTAL_M = 10.0
SEPARATION_WARNING_VERTICAL_M = 5.0

MESSAGE_INTERVALS = {
    "GLOBAL_POSITION_INT": (
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        200_000,
    ),
    "EXTENDED_SYS_STATE": (
        mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
        500_000,
    ),
    "MISSION_CURRENT": (
        mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,
        500_000,
    ),
}


class FleetExecutionError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise FleetExecutionError(message)


def heartbeat_is_armed(message: Any) -> bool:
    return bool(
        int(message.base_mode)
        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )


def haversine_m(
    lat1_deg: float,
    lon1_deg: float,
    lat2_deg: float,
    lon2_deg: float,
) -> float:
    radius_m = 6_371_008.8

    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    value = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1)
        * math.cos(lat2)
        * math.sin(dlon / 2.0) ** 2
    )

    return 2.0 * radius_m * math.asin(math.sqrt(value))


def home_distance_m(state: dict[str, Any]) -> float | None:
    position = state["position"]
    home = state["home"]

    if position is None or home is None:
        return None

    return haversine_m(
        home["latitude_deg"],
        home["longitude_deg"],
        position["latitude_deg"],
        position["longitude_deg"],
    )


def drain_command_acks(connection: Any) -> None:
    while True:
        message = connection.recv_match(
            type="COMMAND_ACK",
            blocking=False,
        )

        if message is None:
            return


def set_message_interval(
    state: dict[str, Any],
    message_name: str,
    message_id: int,
    interval_us: int,
) -> None:
    connection = state["connection"]
    drain_command_acks(connection)

    connection.mav.command_long_send(
        state["system_id"],
        state["component_id"],
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        float(message_id),
        float(interval_us),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        message = connection.recv_match(
            type="COMMAND_ACK",
            blocking=True,
            timeout=1.0,
        )

        if message is None:
            continue

        if message.get_srcSystem() != state["system_id"]:
            continue

        if (
            int(message.command)
            != mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        ):
            continue

        result = int(message.result)

        require(
            result in {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            },
            (
                f"{state['vehicle_id']}: {message_name} interval "
                f"rejected with MAV_RESULT {result}"
            ),
        )

        return

    raise FleetExecutionError(
        f"{state['vehicle_id']}: no interval ACK for {message_name}"
    )


def request_home(state: dict[str, Any]) -> dict[str, float]:
    connection = state["connection"]

    connection.mav.command_long_send(
        state["system_id"],
        state["component_id"],
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        float(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        message = connection.recv_match(
            type="HOME_POSITION",
            blocking=True,
            timeout=1.0,
        )

        if message is None:
            continue

        if message.get_srcSystem() != state["system_id"]:
            continue

        return {
            "latitude_deg": float(message.latitude) / 1e7,
            "longitude_deg": float(message.longitude) / 1e7,
        }

    raise FleetExecutionError(
        f"{state['vehicle_id']}: HOME_POSITION timeout"
    )


def request_mission_count(state: dict[str, Any]) -> int:
    connection = state["connection"]

    try:
        connection.mav.mission_request_list_send(
            state["system_id"],
            state["component_id"],
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        connection.mav.mission_request_list_send(
            state["system_id"],
            state["component_id"],
        )

    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        message = connection.recv_match(
            type="MISSION_COUNT",
            blocking=True,
            timeout=1.0,
        )

        if message is None:
            continue

        if message.get_srcSystem() != state["system_id"]:
            continue

        return int(message.count)

    raise FleetExecutionError(
        f"{state['vehicle_id']}: MISSION_COUNT timeout"
    )


def set_current_mission_item(
    state: dict[str, Any],
    sequence: int,
) -> int:
    connection = state["connection"]

    def send_request() -> None:
        try:
            connection.mav.mission_set_current_send(
                state["system_id"],
                state["component_id"],
                sequence,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            connection.mav.mission_set_current_send(
                state["system_id"],
                state["component_id"],
                sequence,
            )

    # Remove already-queued MISSION_CURRENT telemetry before issuing the
    # command. MAVLink delivery is asynchronous, so these packets describe the
    # state before this request and must not be treated as its acknowledgement.
    while True:
        stale = connection.recv_match(
            type="MISSION_CURRENT",
            blocking=False,
        )

        if stale is None:
            break

        if stale.get_srcSystem() != state["system_id"]:
            continue

        observed = int(stale.seq)
        state["current_sequence"] = observed
        state["highest_sequence"] = max(
            state["highest_sequence"],
            observed,
        )

    deadline = time.monotonic() + COMMAND_TIMEOUT_S
    retry_interval_s = min(
        2.0,
        max(0.25, COMMAND_TIMEOUT_S / 4.0),
    )
    next_retry_s = time.monotonic()

    while time.monotonic() < deadline:
        now_s = time.monotonic()

        if now_s >= next_retry_s:
            send_request()
            next_retry_s = now_s + retry_interval_s

        remaining_s = max(
            0.0,
            deadline - time.monotonic(),
        )

        message = connection.recv_match(
            type="MISSION_CURRENT",
            blocking=True,
            timeout=min(1.0, remaining_s),
        )

        if message is None:
            continue

        if message.get_srcSystem() != state["system_id"]:
            continue

        observed = int(message.seq)

        state["current_sequence"] = observed
        state["highest_sequence"] = max(
            state["highest_sequence"],
            observed,
        )

        # A lower sequence can be telemetry queued before the request or an
        # update emitted while ArduPilot is applying it. Ignore it and continue
        # waiting for the requested item instead of aborting fleet launch.
        if observed < sequence:
            continue

        return observed

    raise FleetExecutionError(
        (
            f"{state['vehicle_id']}: MISSION_CURRENT timeout while "
            f"requesting sequence {sequence}; last observed "
            f"{state['current_sequence']}"
        )
    )


def set_flight_mode(
    state: dict[str, Any],
    mode_name: str,
    expected_armed: bool,
) -> int:
    connection = state["connection"]
    mode_mapping = connection.mode_mapping()

    require(
        mode_mapping is not None and mode_name in mode_mapping,
        f"{state['vehicle_id']}: mode {mode_name} unavailable",
    )

    requested_mode = int(mode_mapping[mode_name])

    connection.mav.set_mode_send(
        state["system_id"],
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        requested_mode,
    )

    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        heartbeat = connection.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1.0,
        )

        if heartbeat is None:
            continue

        if heartbeat.get_srcSystem() != state["system_id"]:
            continue

        armed = heartbeat_is_armed(heartbeat)

        require(
            armed == expected_armed,
            (
                f"{state['vehicle_id']}: armed state changed while "
                f"selecting {mode_name}"
            ),
        )

        state["armed"] = armed
        state["custom_mode"] = int(heartbeat.custom_mode)
        state["last_heartbeat_s"] = time.monotonic()

        if state["custom_mode"] == requested_mode:
            return requested_mode

    raise FleetExecutionError(
        f"{state['vehicle_id']}: timed out entering {mode_name}"
    )


def arm_vehicle(state: dict[str, Any]) -> None:
    connection = state["connection"]
    drain_command_acks(connection)

    connection.mav.command_long_send(
        state["system_id"],
        state["component_id"],
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    deadline = time.monotonic() + COMMAND_TIMEOUT_S
    accepted = False

    while time.monotonic() < deadline:
        message = connection.recv_match(
            blocking=True,
            timeout=1.0,
        )

        if message is None:
            continue

        if message.get_srcSystem() != state["system_id"]:
            continue

        message_type = message.get_type()

        if message_type == "COMMAND_ACK":
            if (
                int(message.command)
                == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
            ):
                result = int(message.result)

                require(
                    result in {
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                    },
                    (
                        f"{state['vehicle_id']}: arming rejected "
                        f"with MAV_RESULT {result}"
                    ),
                )

                accepted = True

        elif message_type == "STATUSTEXT":
            text = str(message.text)
            print(
                f"{state['vehicle_id']} STATUSTEXT: {text}"
            )

        elif message_type == "HEARTBEAT":
            state["last_heartbeat_s"] = time.monotonic()
            state["armed"] = heartbeat_is_armed(message)
            state["custom_mode"] = int(message.custom_mode)

            if state["armed"]:
                require(
                    accepted,
                    (
                        f"{state['vehicle_id']}: armed without "
                        "observing an accepted arm ACK"
                    ),
                )
                return

    raise FleetExecutionError(
        f"{state['vehicle_id']}: arming timeout"
    )


def start_auto_mission(state: dict[str, Any]) -> None:
    connection = state["connection"]
    drain_command_acks(connection)

    connection.mav.command_long_send(
        state["system_id"],
        state["component_id"],
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        message = connection.recv_match(
            blocking=True,
            timeout=1.0,
        )

        if message is None:
            continue

        if message.get_srcSystem() != state["system_id"]:
            continue

        if message.get_type() == "STATUSTEXT":
            print(
                f"{state['vehicle_id']} STATUSTEXT: "
                f"{message.text}"
            )
            continue

        if message.get_type() != "COMMAND_ACK":
            continue

        if (
            int(message.command)
            != mavutil.mavlink.MAV_CMD_MISSION_START
        ):
            continue

        result = int(message.result)

        require(
            result in {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            },
            (
                f"{state['vehicle_id']}: MISSION_START rejected "
                f"with MAV_RESULT {result}"
            ),
        )

        return

    raise FleetExecutionError(
        f"{state['vehicle_id']}: MISSION_START ACK timeout"
    )


def write_event(
    events_handle: Any,
    run_start_s: float,
    state: dict[str, Any],
    event_type: str,
    fields: dict[str, Any],
) -> None:
    record = {
        "time_s": time.monotonic() - run_start_s,
        "vehicle_id": state["vehicle_id"],
        "type": event_type,
        "fields": fields,
    }

    events_handle.write(
        json.dumps(
            record,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    )
    events_handle.flush()


def pump_vehicle(
    state: dict[str, Any],
    events_handle: Any,
    run_start_s: float,
) -> None:
    connection = state["connection"]

    for _ in range(400):
        message = connection.recv_match(blocking=False)

        if message is None:
            break

        if message.get_srcSystem() != state["system_id"]:
            continue

        now = time.monotonic()
        message_type = message.get_type()

        if message_type == "HEARTBEAT":
            state["last_heartbeat_s"] = now
            state["armed"] = heartbeat_is_armed(message)
            state["custom_mode"] = int(message.custom_mode)
            state["system_status"] = int(message.system_status)

        elif message_type == "GLOBAL_POSITION_INT":
            relative_altitude_m = (
                float(message.relative_alt) / 1000.0
            )

            state["position"] = {
                "latitude_deg": float(message.lat) / 1e7,
                "longitude_deg": float(message.lon) / 1e7,
                "relative_altitude_m": relative_altitude_m,
            }
            state["last_position_s"] = now
            state["maximum_relative_altitude_m"] = max(
                state["maximum_relative_altitude_m"],
                relative_altitude_m,
            )

            if relative_altitude_m >= AIRBORNE_ALTITUDE_M:
                state["saw_airborne"] = True

        elif message_type == "EXTENDED_SYS_STATE":
            state["last_landed_state_s"] = now
            state["landed_state"] = int(message.landed_state)

            if state["landed_state"] in {
                mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF,
                mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
                mavutil.mavlink.MAV_LANDED_STATE_LANDING,
            }:
                state["saw_airborne"] = True

        elif message_type == "MISSION_CURRENT":
            sequence = int(message.seq)
            state["last_mission_current_s"] = now
            state["current_sequence"] = sequence
            state["highest_sequence"] = max(
                state["highest_sequence"],
                sequence,
            )

            if sequence >= state["expected_final_sequence"]:
                state["saw_final_sequence"] = True

        elif message_type == "STATUSTEXT":
            text = str(message.text)

            write_event(
                events_handle,
                run_start_s,
                state,
                "STATUSTEXT",
                {
                    "severity": int(message.severity),
                    "text": text,
                },
            )

            important_tokens = (
                "Arming",
                "Disarming",
                "PreArm",
                "Failsafe",
                "RTL",
                "Hit ground",
                "Crash",
                "EKF",
            )

            if any(token in text for token in important_tokens):
                print(
                    f"{state['vehicle_id']} STATUSTEXT: {text}"
                )


def pump_all(
    states: list[dict[str, Any]],
    events_handle: Any,
    run_start_s: float,
) -> None:
    for state in states:
        pump_vehicle(state, events_handle, run_start_s)


def ensure_fresh_airborne_state(
    state: dict[str, Any],
    now: float,
) -> None:
    require(
        state["last_heartbeat_s"] is not None
        and now - state["last_heartbeat_s"]
        <= HEARTBEAT_STALE_S,
        f"{state['vehicle_id']}: heartbeat is stale",
    )

    require(
        state["last_position_s"] is not None
        and now - state["last_position_s"]
        <= POSITION_STALE_S,
        f"{state['vehicle_id']}: position is stale",
    )


def launch_gate_passes(
    previous_states: list[dict[str, Any]],
) -> tuple[bool, str]:
    if not previous_states:
        return True, "first vehicle"

    now = time.monotonic()

    for state in previous_states:
        ensure_fresh_airborne_state(state, now)

        distance = home_distance_m(state)
        altitude = (
            None
            if state["position"] is None
            else state["position"]["relative_altitude_m"]
        )

        if not state["armed"]:
            return (
                False,
                f"{state['vehicle_id']} is no longer armed",
            )

        if altitude is None:
            return False, f"{state['vehicle_id']} has no altitude"

        if altitude < LAUNCH_CLEARANCE_ALTITUDE_M:
            return (
                False,
                (
                    f"{state['vehicle_id']} altitude "
                    f"{altitude:.1f} m"
                ),
            )

        if distance is None:
            return (
                False,
                f"{state['vehicle_id']} has no HOME distance",
            )

        if distance < LAUNCH_CLEARANCE_HOME_DISTANCE_M:
            return (
                False,
                (
                    f"{state['vehicle_id']} only "
                    f"{distance:.1f} m from HOME"
                ),
            )

    return True, "all previous vehicles clear"


def verify_prelaunch_vehicle(state: dict[str, Any]) -> None:
    require(
        not state["armed"],
        f"{state['vehicle_id']}: armed before its launch stage",
    )

    require(
        state["landed_state"]
        == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        (
            f"{state['vehicle_id']}: landed_state is "
            f"{state['landed_state']} before launch"
        ),
    )

    distance = home_distance_m(state)

    require(
        distance is not None
        and distance <= MAXIMUM_PRELAUNCH_HOME_DISTANCE_M,
        (
            f"{state['vehicle_id']}: prelaunch HOME distance is "
            f"{distance}"
        ),
    )


def launch_vehicle(
    state: dict[str, Any],
    run_start_s: float,
    events_handle: Any,
) -> None:
    verify_prelaunch_vehicle(state)

    observed_sequence = set_current_mission_item(
        state,
        TAKEOFF_WIRE_SEQUENCE,
    )

    print(
        f"PASS {state['vehicle_id']}: mission current "
        f"sequence {observed_sequence}"
    )

    loiter_mode = set_flight_mode(
        state,
        "LOITER",
        expected_armed=False,
    )

    print(
        f"PASS {state['vehicle_id']}: LOITER selected "
        f"while disarmed, custom_mode={loiter_mode}"
    )

    arm_vehicle(state)

    print(
        f"PASS {state['vehicle_id']}: armed in LOITER"
    )

    auto_mode = set_flight_mode(
        state,
        "AUTO",
        expected_armed=True,
    )

    print(
        f"PASS {state['vehicle_id']}: AUTO selected "
        f"while armed, custom_mode={auto_mode}"
    )

    start_auto_mission(state)

    state["launched"] = True
    state["launch_time_s"] = time.monotonic() - run_start_s

    write_event(
        events_handle,
        run_start_s,
        state,
        "LAUNCHED",
        {
            "launch_time_s": state["launch_time_s"],
            "expected_final_sequence": (
                state["expected_final_sequence"]
            ),
        },
    )

    print(
        f"LAUNCHED {state['vehicle_id']} at "
        f"t={state['launch_time_s']:.1f} s"
    )


def update_pairwise_separation(
    states: list[dict[str, Any]],
    run_start_s: float,
    separation: dict[str, Any],
) -> None:
    airborne_states = [
        state
        for state in states
        if (
            state["launched"]
            and state["armed"]
            and state["position"] is not None
            and state["position"]["relative_altitude_m"]
            >= AIRBORNE_ALTITUDE_M
        )
    ]

    now_s = time.monotonic() - run_start_s

    for left, right in combinations(airborne_states, 2):
        left_position = left["position"]
        right_position = right["position"]

        horizontal_m = haversine_m(
            left_position["latitude_deg"],
            left_position["longitude_deg"],
            right_position["latitude_deg"],
            right_position["longitude_deg"],
        )

        vertical_m = abs(
            left_position["relative_altitude_m"]
            - right_position["relative_altitude_m"]
        )

        three_dimensional_m = math.hypot(
            horizontal_m,
            vertical_m,
        )

        pair = (
            f"{left['vehicle_id']}|{right['vehicle_id']}"
        )

        current = separation["pairs"].get(pair)

        if (
            current is None
            or three_dimensional_m
            < current["minimum_3d_separation_m"]
        ):
            separation["pairs"][pair] = {
                "minimum_3d_separation_m": three_dimensional_m,
                "horizontal_m_at_minimum": horizontal_m,
                "vertical_m_at_minimum": vertical_m,
                "time_s": now_s,
            }

        overall = separation["overall_minimum"]

        if (
            overall is None
            or three_dimensional_m
            < overall["minimum_3d_separation_m"]
        ):
            separation["overall_minimum"] = {
                "pair": pair,
                "minimum_3d_separation_m": three_dimensional_m,
                "horizontal_m_at_minimum": horizontal_m,
                "vertical_m_at_minimum": vertical_m,
                "time_s": now_s,
            }

        if (
            horizontal_m < SEPARATION_WARNING_HORIZONTAL_M
            and vertical_m < SEPARATION_WARNING_VERTICAL_M
        ):
            warning_key = (
                pair,
                int(now_s // 5.0),
            )

            if warning_key not in separation["printed_warnings"]:
                separation["printed_warnings"].add(warning_key)
                separation["warning_count"] += 1

                print(
                    "SEPARATION WARNING "
                    f"{pair}: horizontal={horizontal_m:.1f} m, "
                    f"vertical={vertical_m:.1f} m"
                )


def snapshot_rows(
    writer: csv.DictWriter,
    states: list[dict[str, Any]],
    run_start_s: float,
) -> None:
    now_s = time.monotonic() - run_start_s

    for state in states:
        position = state["position"]
        distance = home_distance_m(state)

        writer.writerow(
            {
                "time_s": now_s,
                "vehicle_id": state["vehicle_id"],
                "latitude_deg": (
                    None
                    if position is None
                    else position["latitude_deg"]
                ),
                "longitude_deg": (
                    None
                    if position is None
                    else position["longitude_deg"]
                ),
                "relative_altitude_m": (
                    None
                    if position is None
                    else position["relative_altitude_m"]
                ),
                "home_distance_m": distance,
                "armed": state["armed"],
                "landed_state": state["landed_state"],
                "current_sequence": state["current_sequence"],
                "highest_sequence": state["highest_sequence"],
                "custom_mode": state["custom_mode"],
                "launched": state["launched"],
                "completed": state["completed"],
            }
        )


def vehicle_summary(state: dict[str, Any]) -> dict[str, Any]:
    return {
        "vehicle_id": state["vehicle_id"],
        "system_id": state["system_id"],
        "endpoint": state["endpoint"],
        "expected_wire_item_count": (
            state["expected_wire_item_count"]
        ),
        "expected_final_sequence": (
            state["expected_final_sequence"]
        ),
        "launch_time_s": state["launch_time_s"],
        "completion_time_s": state["completion_time_s"],
        "highest_sequence": state["highest_sequence"],
        "maximum_relative_altitude_m": (
            state["maximum_relative_altitude_m"]
        ),
        "final_home_distance_m": home_distance_m(state),
        "saw_airborne": state["saw_airborne"],
        "saw_final_sequence": state["saw_final_sequence"],
        "final_armed": state["armed"],
        "final_landed_state": state["landed_state"],
        "completed": state["completed"],
    }


def write_checkpoint(
    states: list[dict[str, Any]],
    run_start_s: float,
    separation: dict[str, Any],
    status: str,
) -> None:
    output = {
        "schema_version": 1,
        "stage": "21C2",
        "status": status,
        "elapsed_s": time.monotonic() - run_start_s,
        "vehicles": [
            vehicle_summary(state)
            for state in states
        ],
        "separation": {
            "overall_minimum": separation["overall_minimum"],
            "pairs": separation["pairs"],
            "warning_count": separation["warning_count"],
        },
    }

    temporary = CHECKPOINT_PATH.with_suffix(".json.tmp")

    temporary.write_text(
        json.dumps(
            output,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )

    temporary.replace(CHECKPOINT_PATH)


def connect_states(
    config: dict[str, Any],
    upload_report: dict[str, Any],
) -> list[dict[str, Any]]:
    report_by_id = {
        item["vehicle_id"]: item
        for item in upload_report["vehicles"]
    }

    states: list[dict[str, Any]] = []

    for vehicle in config["vehicles"]:
        vehicle_id = vehicle["vehicle_id"]
        endpoint = vehicle["endpoint"]
        system_id = int(vehicle["system_id"])
        component_id = int(vehicle["component_id"])

        print(f"Connecting {vehicle_id} at {endpoint} ...")

        connection = mavutil.mavlink_connection(
            endpoint,
            source_system=SOURCE_SYSTEM,
            source_component=SOURCE_COMPONENT,
            autoreconnect=False,
        )

        heartbeat = connection.wait_heartbeat(
            timeout=CONNECT_TIMEOUT_S,
        )

        require(
            heartbeat is not None,
            f"{vehicle_id}: heartbeat timeout",
        )
        require(
            heartbeat.get_srcSystem() == system_id,
            (
                f"{vehicle_id}: expected sysid {system_id}, "
                f"received {heartbeat.get_srcSystem()}"
            ),
        )
        require(
            heartbeat.get_srcComponent() == component_id,
            (
                f"{vehicle_id}: expected compid {component_id}, "
                f"received {heartbeat.get_srcComponent()}"
            ),
        )
        require(
            not heartbeat_is_armed(heartbeat),
            f"{vehicle_id}: unexpectedly armed",
        )

        frozen = report_by_id[vehicle_id]
        expected_wire_count = int(
            frozen["expected_wire_item_count"]
        )

        state = {
            "vehicle_id": vehicle_id,
            "endpoint": endpoint,
            "system_id": system_id,
            "component_id": component_id,
            "connection": connection,
            "expected_wire_item_count": expected_wire_count,
            "expected_final_sequence": expected_wire_count - 1,
            "home": None,
            "position": None,
            "armed": False,
            "landed_state": None,
            "custom_mode": int(heartbeat.custom_mode),
            "system_status": int(heartbeat.system_status),
            "current_sequence": 0,
            "highest_sequence": 0,
            "last_heartbeat_s": time.monotonic(),
            "last_position_s": None,
            "last_landed_state_s": None,
            "last_mission_current_s": None,
            "maximum_relative_altitude_m": float("-inf"),
            "saw_airborne": False,
            "saw_final_sequence": False,
            "launched": False,
            "launch_time_s": None,
            "completed": False,
            "completion_time_s": None,
        }

        observed_count = request_mission_count(state)

        require(
            observed_count == expected_wire_count,
            (
                f"{vehicle_id}: expected {expected_wire_count} "
                f"mission items, observed {observed_count}"
            ),
        )

        state["home"] = request_home(state)

        for message_name, (
            message_id,
            interval_us,
        ) in MESSAGE_INTERVALS.items():
            set_message_interval(
                state,
                message_name,
                message_id,
                interval_us,
            )

        states.append(state)

        print(
            f"PASS {vehicle_id}: disarmed, mission="
            f"{observed_count}, telemetry configured"
        )

    return states


def main() -> None:
    require(CONFIG_PATH.is_file(), f"missing {CONFIG_PATH}")

    require(
        STATIC_AUDIT_PATH.is_file(),
        f"missing {STATIC_AUDIT_PATH}",
    )

    static_audit = json.loads(
        STATIC_AUDIT_PATH.read_text(encoding="utf-8")
    )

    require(
        static_audit.get("status") == "PASSED",
        "Stage 21B static audit did not pass",
    )
    require(
        static_audit.get("vehicle_count") == 5,
        "Stage 21B static audit is not for five vehicles",
    )
    require(
        static_audit.get("all_return_to_reference") is True,
        "Stage 21B did not verify return-to-reference",
    )
    require(
        static_audit.get("all_end_with_land_at_reference") is True,
        "Stage 21B did not verify LAND at reference",
    )
    require(
        static_audit.get("any_rtl_command") is False,
        "Stage 21B reports an RTL command",
    )

    mission_paths = sorted(
        SOURCE_MISSION_DIR.glob(
            "*.ardupilot-mission.json"
        )
    )

    require(
        len(mission_paths) == 5,
        (
            "expected five Stage 21 ArduPilot missions, "
            f"found {len(mission_paths)}"
        ),
    )

    observed_vehicle_ids = set()

    for mission_path in mission_paths:
        mission = json.loads(
            mission_path.read_text(encoding="utf-8")
        )

        vehicle_id = mission.get("vehicle_id")
        items = mission.get("items")

        require(
            isinstance(vehicle_id, str)
            and vehicle_id,
            f"{mission_path.name}: missing vehicle_id",
        )
        require(
            isinstance(items, list)
            and len(items) >= 3,
            f"{vehicle_id}: invalid semantic mission",
        )

        commands = [
            int(item["command"])
            for item in items
        ]

        require(
            commands[0]
            == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            f"{vehicle_id}: first command is not TAKEOFF",
        )
        require(
            commands[-1]
            == mavutil.mavlink.MAV_CMD_NAV_LAND,
            f"{vehicle_id}: final command is not LAND",
        )
        require(
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
            not in commands,
            f"{vehicle_id}: mission contains RTL",
        )
        require(
            commands[-2]
            == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            (
                f"{vehicle_id}: command before LAND "
                "is not WAYPOINT"
            ),
        )

        takeoff = items[0]
        pre_land = items[-2]
        land = items[-1]

        for label, item in (
            ("pre-LAND waypoint", pre_land),
            ("LAND", land),
        ):
            require(
                math.isclose(
                    float(item["latitude_deg"]),
                    float(takeoff["latitude_deg"]),
                    rel_tol=0.0,
                    abs_tol=2e-7,
                )
                and math.isclose(
                    float(item["longitude_deg"]),
                    float(takeoff["longitude_deg"]),
                    rel_tol=0.0,
                    abs_tol=2e-7,
                ),
                f"{vehicle_id}: {label} is not at HOME",
            )

        require(
            math.isclose(
                float(land["altitude_m"]),
                0.0,
                rel_tol=0.0,
                abs_tol=1e-9,
            ),
            f"{vehicle_id}: LAND altitude is not zero",
        )

        observed_vehicle_ids.add(vehicle_id)

    require(
        observed_vehicle_ids
        == {
            "drone-1",
            "drone-2",
            "drone-3",
            "drone-4",
            "drone-5",
        },
        "Stage 21 source mission vehicle set mismatch",
    )

    print(
        "PASS: five source missions return safely "
        "and terminate with LAND at HOME"
    )
    require(
        UPLOAD_REPORT_PATH.is_file(),
        f"missing {UPLOAD_REPORT_PATH}",
    )
    require(
        DRY_RUN_REPORT_PATH.is_file(),
        f"missing {DRY_RUN_REPORT_PATH}",
    )
    require(
        not REPORT_PATH.exists(),
        f"refusing to overwrite {REPORT_PATH}",
    )

    upload_report = json.loads(
        UPLOAD_REPORT_PATH.read_text(encoding="utf-8")
    )
    dry_run_report = json.loads(
        DRY_RUN_REPORT_PATH.read_text(encoding="utf-8")
    )
    config = yaml.safe_load(
        CONFIG_PATH.read_text(encoding="utf-8")
    )

    require(
        upload_report["all_verified"] is True,
        "Stage 21C upload report is not verified",
    )
    require(
        dry_run_report["status"] == "PASSED",
        "Stage 21C telemetry dry run did not pass",
    )
    require(
        len(config["vehicles"]) == 5,
        "fleet config does not contain five vehicles",
    )

    states: list[dict[str, Any]] = []
    run_start_s = time.monotonic()

    separation: dict[str, Any] = {
        "overall_minimum": None,
        "pairs": {},
        "warning_count": 0,
        "printed_warnings": set(),
    }

    try:
        states = connect_states(config, upload_report)

        with (
            TELEMETRY_PATH.open(
                "w",
                newline="",
                encoding="utf-8",
            ) as telemetry_handle,
            EVENTS_PATH.open(
                "w",
                encoding="utf-8",
            ) as events_handle,
        ):
            writer = csv.DictWriter(
                telemetry_handle,
                fieldnames=[
                    "time_s",
                    "vehicle_id",
                    "latitude_deg",
                    "longitude_deg",
                    "relative_altitude_m",
                    "home_distance_m",
                    "armed",
                    "landed_state",
                    "current_sequence",
                    "highest_sequence",
                    "custom_mode",
                    "launched",
                    "completed",
                ],
            )
            writer.writeheader()

            print()
            print("Collecting initial live telemetry ...")

            initial_deadline = time.monotonic() + 10.0

            while time.monotonic() < initial_deadline:
                pump_all(states, events_handle, run_start_s)

                if all(
                    state["position"] is not None
                    and state["landed_state"] is not None
                    and state["current_sequence"] is not None
                    for state in states
                ):
                    break

                time.sleep(0.02)

            for state in states:
                verify_prelaunch_vehicle(state)

            last_snapshot_s = 0.0
            last_progress_s = 0.0
            last_checkpoint_s = 0.0
            last_launch_wall_s: float | None = None

            for index, state in enumerate(states):
                previous_states = states[:index]

                stable_samples = 0
                gate_deadline = (
                    time.monotonic()
                    + LAUNCH_GATE_TIMEOUT_S
                )

                print()
                print(
                    f"Waiting to launch {state['vehicle_id']} ..."
                )

                while time.monotonic() < gate_deadline:
                    now = time.monotonic()

                    pump_all(
                        states,
                        events_handle,
                        run_start_s,
                    )

                    update_pairwise_separation(
                        states,
                        run_start_s,
                        separation,
                    )

                    for not_yet_launched in states[index:]:
                        require(
                            not not_yet_launched["armed"],
                            (
                                f"{not_yet_launched['vehicle_id']}: "
                                "armed before its scheduled launch"
                            ),
                        )

                    gate_passed, reason = launch_gate_passes(
                        previous_states
                    )

                    spacing_passed = (
                        last_launch_wall_s is None
                        or now - last_launch_wall_s
                        >= MINIMUM_LAUNCH_SPACING_S
                    )

                    if gate_passed and spacing_passed:
                        stable_samples += 1
                    else:
                        stable_samples = 0

                    if (
                        stable_samples
                        >= LAUNCH_GATE_STABLE_SAMPLES
                    ):
                        break

                    elapsed_s = now - run_start_s

                    if elapsed_s - last_snapshot_s >= (
                        TELEMETRY_SNAPSHOT_INTERVAL_S
                    ):
                        snapshot_rows(
                            writer,
                            states,
                            run_start_s,
                        )
                        telemetry_handle.flush()
                        last_snapshot_s = elapsed_s

                    if elapsed_s - last_progress_s >= 5.0:
                        print(
                            f"{state['vehicle_id']} gate: {reason}"
                        )
                        last_progress_s = elapsed_s

                    time.sleep(0.1)

                else:
                    raise FleetExecutionError(
                        f"{state['vehicle_id']}: launch gate timeout"
                    )

                launch_vehicle(
                    state,
                    run_start_s,
                    events_handle,
                )

                last_launch_wall_s = time.monotonic()

            print()
            print(
                "ALL FIVE VEHICLES LAUNCHED — monitoring "
                "until every mission completes"
            )

            fleet_deadline = (
                time.monotonic() + FLEET_TIMEOUT_S
            )

            while time.monotonic() < fleet_deadline:
                now = time.monotonic()
                elapsed_s = now - run_start_s

                pump_all(
                    states,
                    events_handle,
                    run_start_s,
                )

                update_pairwise_separation(
                    states,
                    run_start_s,
                    separation,
                )

                for state in states:
                    if not state["launched"]:
                        continue

                    require(
                        state["last_heartbeat_s"] is not None
                        and now - state["last_heartbeat_s"]
                        <= HEARTBEAT_STALE_S,
                        (
                            f"{state['vehicle_id']}: "
                            "heartbeat became stale"
                        ),
                    )

                    if state["armed"]:
                        require(
                            state["last_position_s"] is not None
                            and now - state["last_position_s"]
                            <= POSITION_STALE_S,
                            (
                                f"{state['vehicle_id']}: "
                                "position became stale"
                            ),
                        )

                    if (
                        not state["armed"]
                        and not state["completed"]
                    ):
                        if not state["saw_airborne"]:
                            raise FleetExecutionError(
                                (
                                    f"{state['vehicle_id']}: "
                                    "disarmed before becoming airborne"
                                )
                            )

                        if not state["saw_final_sequence"]:
                            raise FleetExecutionError(
                                (
                                    f"{state['vehicle_id']}: "
                                    "disarmed before final mission item; "
                                    f"highest_sequence="
                                    f"{state['highest_sequence']}"
                                )
                            )

                        if (
                            state["landed_state"]
                            == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                        ):
                            final_distance = home_distance_m(state)

                            require(
                                final_distance is not None
                                and final_distance
                                <= MAXIMUM_FINAL_HOME_DISTANCE_M,
                                (
                                    f"{state['vehicle_id']}: "
                                    f"landed {final_distance} m "
                                    "from HOME"
                                ),
                            )

                            state["completed"] = True
                            state["completion_time_s"] = elapsed_s

                            write_event(
                                events_handle,
                                run_start_s,
                                state,
                                "COMPLETED",
                                {
                                    "completion_time_s": elapsed_s,
                                    "final_home_distance_m": (
                                        final_distance
                                    ),
                                },
                            )

                            print(
                                f"COMPLETED {state['vehicle_id']}: "
                                f"t={elapsed_s:.1f} s, "
                                f"home_distance="
                                f"{final_distance:.2f} m"
                            )

                if elapsed_s - last_snapshot_s >= (
                    TELEMETRY_SNAPSHOT_INTERVAL_S
                ):
                    snapshot_rows(
                        writer,
                        states,
                        run_start_s,
                    )
                    telemetry_handle.flush()
                    last_snapshot_s = elapsed_s

                if elapsed_s - last_progress_s >= (
                    PROGRESS_INTERVAL_S
                ):
                    print()
                    for state in states:
                        position = state["position"]
                        altitude = (
                            0.0
                            if position is None
                            else position[
                                "relative_altitude_m"
                            ]
                        )

                        print(
                            f"{state['vehicle_id']}: "
                            f"seq={state['current_sequence']}/"
                            f"{state['expected_final_sequence']}, "
                            f"alt={altitude:.1f} m, "
                            f"armed={state['armed']}, "
                            f"completed={state['completed']}"
                        )

                    last_progress_s = elapsed_s

                if elapsed_s - last_checkpoint_s >= (
                    CHECKPOINT_INTERVAL_S
                ):
                    write_checkpoint(
                        states,
                        run_start_s,
                        separation,
                        status="RUNNING",
                    )
                    last_checkpoint_s = elapsed_s

                if all(
                    state["completed"]
                    for state in states
                ):
                    break

                time.sleep(0.02)

            else:
                raise FleetExecutionError(
                    "fleet mission timeout"
                )

            telemetry_handle.flush()

        report = {
            "schema_version": 1,
            "stage": "21C2",
            "status": "PASSED",
            "execution_policy": (
                "staggered_shared_home_clearance"
            ),
            "launch_clearance_altitude_m": (
                LAUNCH_CLEARANCE_ALTITUDE_M
            ),
            "launch_clearance_home_distance_m": (
                LAUNCH_CLEARANCE_HOME_DISTANCE_M
            ),
            "vehicle_count": len(states),
            "all_completed": True,
            "vehicles": [
                vehicle_summary(state)
                for state in states
            ],
            "separation": {
                "overall_minimum": (
                    separation["overall_minimum"]
                ),
                "pairs": separation["pairs"],
                "warning_count": separation["warning_count"],
            },
            "telemetry_path": str(TELEMETRY_PATH),
            "events_path": str(EVENTS_PATH),
        }

        temporary = REPORT_PATH.with_suffix(".json.tmp")

        temporary.write_text(
            json.dumps(
                report,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )

        temporary.replace(REPORT_PATH)

        write_checkpoint(
            states,
            run_start_s,
            separation,
            status="PASSED",
        )

        print()
        print(
            "PASSED: all five staggered missions completed, "
            "RTL-landed and disarmed"
        )
        print(f"Report: {REPORT_PATH}")
        print(f"Telemetry: {TELEMETRY_PATH}")
        print(f"Events: {EVENTS_PATH}")

        if separation["overall_minimum"] is not None:
            minimum = separation["overall_minimum"]

            print(
                "Minimum observed airborne separation: "
                f"{minimum['minimum_3d_separation_m']:.2f} m "
                f"for {minimum['pair']}"
            )

    except Exception:
        if states:
            write_checkpoint(
                states,
                run_start_s,
                separation,
                status="FAILED",
            )
        raise

    finally:
        for state in states:
            try:
                state["connection"].close()
            except Exception:
                pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print(
            "INTERRUPTED: the script did not force-disarm any "
            "airborne vehicle. Inspect the checkpoint and SITL state."
        )
        raise SystemExit(130)
    except Exception as exc:
        print()
        print(f"FAILED: {exc}")
        print(
            "No forced airborne disarm command was sent. "
            f"Checkpoint: {CHECKPOINT_PATH}"
        )
        raise SystemExit(1)
