#!/usr/bin/env python3
"""Execute and monitor an arbitrary-size verified ArduCopter SITL fleet."""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from itertools import combinations
import json
import math
from pathlib import Path
import time
from typing import Any, Mapping, Sequence

import yaml
from pymavlink import mavutil


SOURCE_SYSTEM = 255
SOURCE_COMPONENT = 198
CONNECT_TIMEOUT_S = 30.0
COMMAND_TIMEOUT_S = 20.0
POSITION_READY_TIMEOUT_S = 180.0
PREARM_TIMEOUT_S = 120.0
RC_OVERRIDE_TIMEOUT_S = 15.0
HEARTBEAT_STALE_S = 8.0
POSITION_STALE_S = 5.0
AIRBORNE_ALTITUDE_M = 3.0
RC_THROTTLE_LOW_PWM = 1000
TAKEOFF_WIRE_SEQUENCE = 1


class VariableFleetExecutionError(RuntimeError):
    """Raised when the dynamic fleet cannot proceed safely."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise VariableFleetExecutionError(message)


@dataclass(frozen=True)
class FleetExecutionOptions:
    """Runtime gates for a shared-HOME arbitrary-N SITL fleet."""

    fleet_timeout_s: float = 3600.0
    launch_gate_timeout_s: float = 600.0
    launch_clearance_altitude_m: float = 12.0
    launch_clearance_home_distance_m: float = 30.0
    launch_gate_stable_samples: int = 10
    minimum_launch_spacing_s: float = 5.0
    maximum_prelaunch_home_distance_m: float = 2.0
    maximum_final_home_distance_m: float = 10.0
    separation_warning_horizontal_m: float = 10.0
    separation_warning_vertical_m: float = 5.0
    telemetry_snapshot_interval_s: float = 0.5
    progress_interval_s: float = 5.0
    checkpoint_interval_s: float = 5.0

    def __post_init__(self) -> None:
        positive_values = {
            "fleet_timeout_s": self.fleet_timeout_s,
            "launch_gate_timeout_s": self.launch_gate_timeout_s,
            "launch_clearance_altitude_m": self.launch_clearance_altitude_m,
            "launch_clearance_home_distance_m": self.launch_clearance_home_distance_m,
            "minimum_launch_spacing_s": self.minimum_launch_spacing_s,
            "maximum_prelaunch_home_distance_m": self.maximum_prelaunch_home_distance_m,
            "maximum_final_home_distance_m": self.maximum_final_home_distance_m,
            "separation_warning_horizontal_m": self.separation_warning_horizontal_m,
            "separation_warning_vertical_m": self.separation_warning_vertical_m,
            "telemetry_snapshot_interval_s": self.telemetry_snapshot_interval_s,
            "progress_interval_s": self.progress_interval_s,
            "checkpoint_interval_s": self.checkpoint_interval_s,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")
        if (
            isinstance(self.launch_gate_stable_samples, bool)
            or not isinstance(self.launch_gate_stable_samples, int)
            or self.launch_gate_stable_samples < 1
        ):
            raise ValueError("launch_gate_stable_samples must be a positive integer")


@dataclass
class VehicleState:
    vehicle_id: str
    endpoint: str
    system_id: int
    component_id: int
    expected_wire_item_count: int
    connection: Any
    expected_final_sequence: int = field(init=False)
    home: tuple[float, float] | None = None
    position: tuple[float, float, float] | None = None
    armed: bool = False
    landed_state: int | None = None
    custom_mode: int = 0
    system_status: int = 0
    current_sequence: int = 0
    highest_sequence: int = 0
    last_heartbeat_s: float | None = None
    last_position_s: float | None = None
    maximum_relative_altitude_m: float = float("-inf")
    saw_airborne: bool = False
    saw_final_sequence: bool = False
    launched: bool = False
    launch_time_s: float | None = None
    completed: bool = False
    completion_time_s: float | None = None

    def __post_init__(self) -> None:
        self.expected_final_sequence = self.expected_wire_item_count - 1


def _log(state: VehicleState, message: str) -> None:
    print(f"{state.vehicle_id}: {message}")


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


def home_distance_m(state: VehicleState) -> float | None:
    if state.position is None or state.home is None:
        return None
    return haversine_m(
        state.home[0],
        state.home[1],
        state.position[0],
        state.position[1],
    )


def recv_for_system(
    connection: Any,
    system_id: int,
    *,
    types: str | list[str] | None = None,
    timeout: float = 1.0,
) -> Any | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type=types,
            blocking=True,
            timeout=min(0.5, max(0.0, deadline - time.monotonic())),
        )
        if message is None:
            continue
        if message.get_srcSystem() != system_id:
            continue
        return message
    return None


def drain_command_acks(connection: Any) -> None:
    while connection.recv_match(type="COMMAND_ACK", blocking=False) is not None:
        pass


def wait_heartbeat(state: VehicleState) -> Any:
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline:
        heartbeat = recv_for_system(
            state.connection,
            state.system_id,
            types="HEARTBEAT",
            timeout=1.0,
        )
        if heartbeat is None:
            continue
        require(
            heartbeat.get_srcComponent() == state.component_id,
            (
                f"{state.vehicle_id}: expected component {state.component_id}, "
                f"got {heartbeat.get_srcComponent()}"
            ),
        )
        return heartbeat
    raise VariableFleetExecutionError(
        f"{state.vehicle_id}: heartbeat timeout on {state.endpoint!r}"
    )


def request_mission_count(state: VehicleState) -> int:
    try:
        state.connection.mav.mission_request_list_send(
            state.system_id,
            state.component_id,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        state.connection.mav.mission_request_list_send(
            state.system_id,
            state.component_id,
        )
    message = recv_for_system(
        state.connection,
        state.system_id,
        types="MISSION_COUNT",
        timeout=COMMAND_TIMEOUT_S,
    )
    if message is None:
        raise VariableFleetExecutionError(
            f"{state.vehicle_id}: MISSION_COUNT timeout"
        )
    return int(message.count)


def request_home(state: VehicleState) -> tuple[float, float]:
    state.connection.mav.command_long_send(
        state.system_id,
        state.component_id,
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
    message = recv_for_system(
        state.connection,
        state.system_id,
        types="HOME_POSITION",
        timeout=COMMAND_TIMEOUT_S,
    )
    if message is None:
        raise VariableFleetExecutionError(
            f"{state.vehicle_id}: HOME_POSITION timeout"
        )
    return float(message.latitude) / 1e7, float(message.longitude) / 1e7


def set_message_interval(
    state: VehicleState,
    message_name: str,
    message_id: int,
    interval_us: int,
) -> None:
    drain_command_acks(state.connection)
    state.connection.mav.command_long_send(
        state.system_id,
        state.component_id,
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
        message = recv_for_system(
            state.connection,
            state.system_id,
            types="COMMAND_ACK",
            timeout=1.0,
        )
        if message is None:
            continue
        if int(message.command) != mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            continue
        result = int(message.result)
        require(
            result
            in {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            },
            (
                f"{state.vehicle_id}: {message_name} interval rejected "
                f"with MAV_RESULT {result}"
            ),
        )
        return
    raise VariableFleetExecutionError(
        f"{state.vehicle_id}: no interval ACK for {message_name}"
    )


def set_current_item(state: VehicleState, sequence: int) -> int:
    while state.connection.recv_match(
        type="MISSION_CURRENT",
        blocking=False,
    ) is not None:
        pass

    deadline = time.monotonic() + COMMAND_TIMEOUT_S
    next_send_s = 0.0
    last_seen: int | None = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send_s:
            try:
                state.connection.mav.mission_set_current_send(
                    state.system_id,
                    state.component_id,
                    sequence,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                state.connection.mav.mission_set_current_send(
                    state.system_id,
                    state.component_id,
                    sequence,
                )
            next_send_s = now + 1.0

        message = recv_for_system(
            state.connection,
            state.system_id,
            types="MISSION_CURRENT",
            timeout=0.5,
        )
        if message is None:
            continue
        last_seen = int(message.seq)
        state.current_sequence = last_seen
        state.highest_sequence = max(state.highest_sequence, last_seen)
        if last_seen >= sequence:
            return last_seen

    raise VariableFleetExecutionError(
        (
            f"{state.vehicle_id}: MISSION_CURRENT timeout requesting "
            f"{sequence}; last observed {last_seen}"
        )
    )


def set_mode(
    state: VehicleState,
    mode_name: str,
    *,
    expected_armed: bool,
) -> int:
    mapping = state.connection.mode_mapping()
    require(
        mapping is not None and mode_name in mapping,
        f"{state.vehicle_id}: mode {mode_name} unavailable",
    )
    requested = int(mapping[mode_name])
    state.connection.mav.set_mode_send(
        state.system_id,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        requested,
    )

    deadline = time.monotonic() + COMMAND_TIMEOUT_S
    while time.monotonic() < deadline:
        heartbeat = recv_for_system(
            state.connection,
            state.system_id,
            types="HEARTBEAT",
            timeout=1.0,
        )
        if heartbeat is None:
            continue
        armed = heartbeat_is_armed(heartbeat)
        require(
            armed == expected_armed,
            (
                f"{state.vehicle_id}: armed state changed while selecting "
                f"{mode_name}"
            ),
        )
        state.armed = armed
        state.custom_mode = int(heartbeat.custom_mode)
        state.system_status = int(heartbeat.system_status)
        state.last_heartbeat_s = time.monotonic()
        if state.custom_mode == requested:
            return requested

    raise VariableFleetExecutionError(
        f"{state.vehicle_id}: timed out entering {mode_name}"
    )


def wait_for_position_estimate(
    state: VehicleState,
    *,
    timeout_s: float = POSITION_READY_TIMEOUT_S,
) -> None:
    ekf_message_id = int(
        getattr(mavutil.mavlink, "MAVLINK_MSG_ID_EKF_STATUS_REPORT", 193)
    )
    gps_message_id = int(
        getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GPS_RAW_INT", 24)
    )
    ekf_pos_horiz_abs = int(
        getattr(mavutil.mavlink, "EKF_POS_HORIZ_ABS", 16)
    )
    ekf_pred_pos_horiz_abs = int(
        getattr(mavutil.mavlink, "EKF_PRED_POS_HORIZ_ABS", 512)
    )
    ekf_const_pos_mode = int(
        getattr(mavutil.mavlink, "EKF_CONST_POS_MODE", 128)
    )
    ekf_uninitialized = int(
        getattr(mavutil.mavlink, "EKF_UNINITIALIZED", 1024)
    )

    for name, message_id, interval_us in (
        ("GPS_RAW_INT", gps_message_id, 500_000),
        ("EKF_STATUS_REPORT", ekf_message_id, 500_000),
        (
            "GLOBAL_POSITION_INT",
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            500_000,
        ),
    ):
        set_message_interval(state, name, message_id, interval_us)

    _log(state, "waiting for GPS 3D fix and stable EKF position")
    deadline = time.monotonic() + timeout_s
    last_report_s = 0.0
    gps_fix_type: int | None = None
    satellites: int | None = None
    gps_eph: int | None = None
    ekf_flags: int | None = None
    ready_streak = 0
    last_reason: str | None = None

    while time.monotonic() < deadline:
        message = recv_for_system(
            state.connection,
            state.system_id,
            types=[
                "GPS_RAW_INT",
                "EKF_STATUS_REPORT",
                "GLOBAL_POSITION_INT",
                "STATUSTEXT",
                "HEARTBEAT",
            ],
            timeout=1.0,
        )
        now = time.monotonic()
        if message is None:
            continue

        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            require(
                not heartbeat_is_armed(message),
                f"{state.vehicle_id}: armed during position wait",
            )
        elif message_type == "STATUSTEXT":
            text = str(message.text).strip()
            if text and text != last_reason:
                _log(state, f"STATUSTEXT: {text}")
                last_reason = text
        elif message_type == "GPS_RAW_INT":
            gps_fix_type = int(message.fix_type)
            satellites = int(message.satellites_visible)
            gps_eph = int(message.eph)
        elif message_type == "GLOBAL_POSITION_INT":
            state.position = (
                float(message.lat) / 1e7,
                float(message.lon) / 1e7,
                float(message.relative_alt) / 1000.0,
            )
            state.last_position_s = now
        elif message_type == "EKF_STATUS_REPORT":
            ekf_flags = int(message.flags)
            gps_ready = gps_fix_type is not None and gps_fix_type >= 3
            absolute_ready = bool(
                ekf_flags & (ekf_pos_horiz_abs | ekf_pred_pos_horiz_abs)
            )
            invalid_mode = bool(
                ekf_flags & (ekf_const_pos_mode | ekf_uninitialized)
            )
            if gps_ready and absolute_ready and not invalid_mode:
                ready_streak += 1
            else:
                ready_streak = 0
            if ready_streak >= 3:
                _log(
                    state,
                    (
                        "PASS GPS/EKF position stable "
                        f"(fix={gps_fix_type}, sats={satellites}, "
                        f"flags=0x{ekf_flags:04x})"
                    ),
                )
                return

        if now - last_report_s >= 5.0:
            _log(
                state,
                (
                    "position not ready: "
                    f"fix={gps_fix_type}, sats={satellites}, eph={gps_eph}, "
                    f"flags={None if ekf_flags is None else hex(ekf_flags)}, "
                    f"stable={ready_streak}/3"
                ),
            )
            last_report_s = now

    raise VariableFleetExecutionError(
        (
            f"{state.vehicle_id}: GPS/EKF position did not become ready "
            f"within {timeout_s:.0f}s; fix={gps_fix_type}, sats={satellites}, "
            f"eph={gps_eph}, flags="
            f"{None if ekf_flags is None else hex(ekf_flags)}"
        )
        + ("" if not last_reason else f"; last status: {last_reason}")
    )


def wait_for_prearm_ready(
    state: VehicleState,
    *,
    timeout_s: float = PREARM_TIMEOUT_S,
) -> None:
    prearm_bit = int(
        getattr(mavutil.mavlink, "MAV_SYS_STATUS_PREARM_CHECK", 1 << 28)
    )
    set_message_interval(
        state,
        "SYS_STATUS",
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        500_000,
    )
    _log(state, "waiting for ArduPilot pre-arm health")

    deadline = time.monotonic() + timeout_s
    last_report_s = 0.0
    last_reason: str | None = None
    saw_sys_status = False

    while time.monotonic() < deadline:
        message = recv_for_system(
            state.connection,
            state.system_id,
            timeout=1.0,
        )
        now = time.monotonic()
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            require(
                not heartbeat_is_armed(message),
                f"{state.vehicle_id}: armed during pre-arm wait",
            )
        elif message_type == "STATUSTEXT":
            text = str(message.text).strip()
            if text and text != last_reason:
                _log(state, f"STATUSTEXT: {text}")
                last_reason = text
        elif message_type == "SYS_STATUS":
            saw_sys_status = True
            enabled = int(message.onboard_control_sensors_enabled)
            health = int(message.onboard_control_sensors_health)
            check_enabled = bool(enabled & prearm_bit)
            check_healthy = bool(health & prearm_bit)
            if check_enabled and check_healthy:
                _log(state, "PASS pre-arm checks healthy")
                return
            if now - last_report_s >= 5.0:
                _log(
                    state,
                    (
                        "pre-arm not ready: "
                        f"enabled={check_enabled}, healthy={check_healthy}"
                    ),
                )
                last_report_s = now

    detail = "" if not last_reason else f"; last status: {last_reason}"
    if not saw_sys_status:
        detail += "; no SYS_STATUS received"
    raise VariableFleetExecutionError(
        (
            f"{state.vehicle_id}: pre-arm checks did not become healthy "
            f"within {timeout_s:.0f}s"
        )
        + detail
    )


def send_rc_override(
    state: VehicleState,
    *,
    roll_pwm: int,
    pitch_pwm: int,
    throttle_pwm: int,
    yaw_pwm: int,
) -> None:
    ignore = 65535
    channels_18 = (
        roll_pwm,
        pitch_pwm,
        throttle_pwm,
        yaw_pwm,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
    )
    try:
        state.connection.mav.rc_channels_override_send(
            state.system_id,
            state.component_id,
            *channels_18,
        )
    except TypeError:
        state.connection.mav.rc_channels_override_send(
            state.system_id,
            state.component_id,
            *channels_18[:8],
        )


def release_rc_override(state: VehicleState) -> None:
    release = 0
    ignore = 65535
    channels_18 = (
        release,
        release,
        release,
        release,
        release,
        release,
        release,
        release,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
        ignore,
    )
    try:
        state.connection.mav.rc_channels_override_send(
            state.system_id,
            state.component_id,
            *channels_18,
        )
    except TypeError:
        state.connection.mav.rc_channels_override_send(
            state.system_id,
            state.component_id,
            *channels_18[:8],
        )


def establish_low_throttle_rc(
    state: VehicleState,
    *,
    timeout_s: float = RC_OVERRIDE_TIMEOUT_S,
) -> None:
    set_message_interval(
        state,
        "RC_CHANNELS",
        mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
        200_000,
    )
    _log(state, "providing centered SITL RC with throttle=1000")
    deadline = time.monotonic() + timeout_s
    next_send_s = 0.0
    last_throttle: int | None = None

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send_s:
            send_rc_override(
                state,
                roll_pwm=1500,
                pitch_pwm=1500,
                throttle_pwm=RC_THROTTLE_LOW_PWM,
                yaw_pwm=1500,
            )
            next_send_s = now + 0.5

        message = recv_for_system(
            state.connection,
            state.system_id,
            types=["RC_CHANNELS", "STATUSTEXT", "HEARTBEAT"],
            timeout=0.5,
        )
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            require(
                not heartbeat_is_armed(message),
                f"{state.vehicle_id}: armed while establishing RC input",
            )
        elif message_type == "STATUSTEXT":
            text = str(message.text).strip()
            if text:
                _log(state, f"STATUSTEXT: {text}")
        elif message_type == "RC_CHANNELS":
            last_throttle = int(message.chan3_raw)
            if 950 <= last_throttle <= 1100:
                send_rc_override(
                    state,
                    roll_pwm=1500,
                    pitch_pwm=1500,
                    throttle_pwm=RC_THROTTLE_LOW_PWM,
                    yaw_pwm=1500,
                )
                _log(state, f"PASS low throttle received ({last_throttle} us)")
                return

    raise VariableFleetExecutionError(
        (
            f"{state.vehicle_id}: low-throttle RC override was not accepted"
            + (
                ""
                if last_throttle is None
                else f"; last RC3={last_throttle} us"
            )
            + "; verify SYSID_MYGCS accepts source system 255"
        )
    )


def arm_vehicle(state: VehicleState) -> None:
    drain_command_acks(state.connection)
    state.connection.mav.command_long_send(
        state.system_id,
        state.component_id,
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
    accepted = False
    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        message = recv_for_system(
            state.connection,
            state.system_id,
            timeout=1.0,
        )
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "STATUSTEXT":
            _log(state, f"STATUSTEXT: {message.text}")
        elif message_type == "COMMAND_ACK":
            if int(message.command) != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                continue
            result = int(message.result)
            if result not in {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            }:
                reasons: list[str] = []
                reason_deadline = time.monotonic() + 3.0
                while time.monotonic() < reason_deadline:
                    followup = recv_for_system(
                        state.connection,
                        state.system_id,
                        timeout=0.5,
                    )
                    if followup is None:
                        continue
                    if followup.get_type() == "STATUSTEXT":
                        text = str(followup.text).strip()
                        if text:
                            reasons.append(text)
                            _log(state, f"STATUSTEXT: {text}")
                result_name = mavutil.mavlink.enums["MAV_RESULT"][result].name
                raise VariableFleetExecutionError(
                    (
                        f"{state.vehicle_id}: arming rejected with "
                        f"{result_name} (MAV_RESULT {result})"
                    )
                    + ("" if not reasons else "; " + " | ".join(reasons))
                )
            accepted = True
        elif message_type == "HEARTBEAT" and heartbeat_is_armed(message):
            require(
                accepted,
                f"{state.vehicle_id}: armed without accepted arm ACK",
            )
            state.armed = True
            state.last_heartbeat_s = time.monotonic()
            return

    raise VariableFleetExecutionError(f"{state.vehicle_id}: arming timeout")


def start_auto_mission(state: VehicleState) -> None:
    drain_command_acks(state.connection)
    state.connection.mav.command_long_send(
        state.system_id,
        state.component_id,
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
        message = recv_for_system(
            state.connection,
            state.system_id,
            timeout=1.0,
        )
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            _log(state, f"STATUSTEXT: {message.text}")
            continue
        if message.get_type() != "COMMAND_ACK":
            continue
        if int(message.command) != mavutil.mavlink.MAV_CMD_MISSION_START:
            continue
        result = int(message.result)
        require(
            result
            in {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            },
            (
                f"{state.vehicle_id}: MISSION_START rejected "
                f"with MAV_RESULT {result}"
            ),
        )
        return
    raise VariableFleetExecutionError(
        f"{state.vehicle_id}: MISSION_START ACK timeout"
    )


def request_land(state: VehicleState) -> None:
    try:
        mapping = state.connection.mode_mapping()
        if mapping and "LAND" in mapping:
            state.connection.mav.set_mode_send(
                state.system_id,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                int(mapping["LAND"]),
            )
            _log(state, "emergency LAND requested")
    except Exception as exc:
        _log(state, f"WARNING could not request LAND: {exc}")


def write_event(
    handle: Any,
    run_start_s: float,
    state: VehicleState,
    event_type: str,
    fields: Mapping[str, Any],
) -> None:
    record = {
        "time_s": time.monotonic() - run_start_s,
        "vehicle_id": state.vehicle_id,
        "type": event_type,
        "fields": dict(fields),
    }
    handle.write(
        json.dumps(record, sort_keys=True, allow_nan=False) + "\n"
    )
    handle.flush()


def pump_vehicle(
    state: VehicleState,
    events_handle: Any,
    run_start_s: float,
) -> None:
    for _ in range(400):
        message = state.connection.recv_match(blocking=False)
        if message is None:
            break
        if message.get_srcSystem() != state.system_id:
            continue

        now = time.monotonic()
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            state.last_heartbeat_s = now
            state.armed = heartbeat_is_armed(message)
            state.custom_mode = int(message.custom_mode)
            state.system_status = int(message.system_status)
        elif message_type == "GLOBAL_POSITION_INT":
            altitude_m = float(message.relative_alt) / 1000.0
            state.position = (
                float(message.lat) / 1e7,
                float(message.lon) / 1e7,
                altitude_m,
            )
            state.last_position_s = now
            state.maximum_relative_altitude_m = max(
                state.maximum_relative_altitude_m,
                altitude_m,
            )
            if altitude_m >= AIRBORNE_ALTITUDE_M:
                state.saw_airborne = True
        elif message_type == "EXTENDED_SYS_STATE":
            state.landed_state = int(message.landed_state)
            if state.landed_state in {
                mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF,
                mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
                mavutil.mavlink.MAV_LANDED_STATE_LANDING,
            }:
                state.saw_airborne = True
        elif message_type == "MISSION_CURRENT":
            sequence = int(message.seq)
            state.current_sequence = sequence
            state.highest_sequence = max(
                state.highest_sequence,
                sequence,
            )
            if sequence >= state.expected_final_sequence:
                state.saw_final_sequence = True
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
                _log(state, f"STATUSTEXT: {text}")


def pump_all(
    states: Sequence[VehicleState],
    events_handle: Any,
    run_start_s: float,
) -> None:
    for state in states:
        pump_vehicle(state, events_handle, run_start_s)


def verify_prelaunch_vehicle(
    state: VehicleState,
    options: FleetExecutionOptions,
) -> None:
    require(
        not state.armed,
        f"{state.vehicle_id}: armed before scheduled launch",
    )
    require(
        state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        (
            f"{state.vehicle_id}: landed_state={state.landed_state} "
            "before launch"
        ),
    )
    distance = home_distance_m(state)
    require(
        distance is not None
        and distance <= options.maximum_prelaunch_home_distance_m,
        (
            f"{state.vehicle_id}: prelaunch HOME distance "
            f"is {distance}"
        ),
    )


def launch_gate_status(
    previous_states: Sequence[VehicleState],
    options: FleetExecutionOptions,
    *,
    now_s: float | None = None,
) -> tuple[bool, str]:
    if not previous_states:
        return True, "first vehicle"

    now = time.monotonic() if now_s is None else now_s
    for state in previous_states:
        if state.completed or not state.armed:
            return (
                False,
                (
                    f"{state.vehicle_id} completed/disarmed before all "
                    "shared-HOME launches finished"
                ),
            )
        if (
            state.last_heartbeat_s is None
            or now - state.last_heartbeat_s > HEARTBEAT_STALE_S
        ):
            raise VariableFleetExecutionError(
                f"{state.vehicle_id}: heartbeat is stale during launch gate"
            )
        if (
            state.last_position_s is None
            or now - state.last_position_s > POSITION_STALE_S
        ):
            raise VariableFleetExecutionError(
                f"{state.vehicle_id}: position is stale during launch gate"
            )
        if state.position is None:
            return False, f"{state.vehicle_id} has no position"

        altitude_m = state.position[2]
        distance_m = home_distance_m(state)
        if altitude_m < options.launch_clearance_altitude_m:
            return (
                False,
                f"{state.vehicle_id} altitude {altitude_m:.1f} m",
            )
        if (
            distance_m is None
            or distance_m < options.launch_clearance_home_distance_m
        ):
            return (
                False,
                (
                    f"{state.vehicle_id} only "
                    f"{0.0 if distance_m is None else distance_m:.1f} m "
                    "from HOME"
                ),
            )

    return True, "all previous vehicles clear"


def launch_vehicle(
    state: VehicleState,
    options: FleetExecutionOptions,
    run_start_s: float,
    events_handle: Any,
) -> None:
    verify_prelaunch_vehicle(state, options)
    establish_low_throttle_rc(state)
    arm_vehicle(state)
    _log(state, "PASS armed in LOITER")
    auto_mode = set_mode(state, "AUTO", expected_armed=True)
    _log(state, f"PASS AUTO selected, custom_mode={auto_mode}")
    start_auto_mission(state)
    release_rc_override(state)
    state.launched = True
    state.launch_time_s = time.monotonic() - run_start_s
    write_event(
        events_handle,
        run_start_s,
        state,
        "LAUNCHED",
        {
            "launch_time_s": state.launch_time_s,
            "expected_final_sequence": state.expected_final_sequence,
        },
    )
    _log(state, f"LAUNCHED at t={state.launch_time_s:.1f}s")


def update_pairwise_separation(
    states: Sequence[VehicleState],
    run_start_s: float,
    separation: dict[str, Any],
    options: FleetExecutionOptions,
) -> None:
    airborne = [
        state
        for state in states
        if (
            state.launched
            and state.armed
            and state.position is not None
            and state.position[2] >= AIRBORNE_ALTITUDE_M
        )
    ]
    now_s = time.monotonic() - run_start_s

    for left, right in combinations(airborne, 2):
        assert left.position is not None
        assert right.position is not None
        horizontal_m = haversine_m(
            left.position[0],
            left.position[1],
            right.position[0],
            right.position[1],
        )
        vertical_m = abs(left.position[2] - right.position[2])
        three_d_m = math.hypot(horizontal_m, vertical_m)
        pair = f"{left.vehicle_id}|{right.vehicle_id}"

        current = separation["pairs"].get(pair)
        if (
            current is None
            or three_d_m < current["minimum_3d_separation_m"]
        ):
            separation["pairs"][pair] = {
                "minimum_3d_separation_m": three_d_m,
                "horizontal_m_at_minimum": horizontal_m,
                "vertical_m_at_minimum": vertical_m,
                "time_s": now_s,
            }

        overall = separation["overall_minimum"]
        if (
            overall is None
            or three_d_m < overall["minimum_3d_separation_m"]
        ):
            separation["overall_minimum"] = {
                "pair": pair,
                "minimum_3d_separation_m": three_d_m,
                "horizontal_m_at_minimum": horizontal_m,
                "vertical_m_at_minimum": vertical_m,
                "time_s": now_s,
            }

        if (
            horizontal_m < options.separation_warning_horizontal_m
            and vertical_m < options.separation_warning_vertical_m
        ):
            warning_key = (pair, int(now_s // 5.0))
            if warning_key not in separation["printed_warnings"]:
                separation["printed_warnings"].add(warning_key)
                separation["warning_count"] += 1
                print(
                    "SEPARATION WARNING "
                    f"{pair}: horizontal={horizontal_m:.1f}m, "
                    f"vertical={vertical_m:.1f}m"
                )


def vehicle_summary(state: VehicleState) -> dict[str, Any]:
    return {
        "vehicle_id": state.vehicle_id,
        "system_id": state.system_id,
        "endpoint": state.endpoint,
        "expected_wire_item_count": state.expected_wire_item_count,
        "expected_final_sequence": state.expected_final_sequence,
        "launch_time_s": state.launch_time_s,
        "completion_time_s": state.completion_time_s,
        "highest_sequence": state.highest_sequence,
        "maximum_relative_altitude_m": (
            None
            if state.maximum_relative_altitude_m == float("-inf")
            else state.maximum_relative_altitude_m
        ),
        "final_home_distance_m": home_distance_m(state),
        "saw_airborne": state.saw_airborne,
        "saw_final_sequence": state.saw_final_sequence,
        "final_armed": state.armed,
        "final_landed_state": state.landed_state,
        "completed": state.completed,
    }


def _json_safe_separation(separation: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "overall_minimum": separation["overall_minimum"],
        "pairs": separation["pairs"],
        "warning_count": separation["warning_count"],
    }


def write_checkpoint(
    path: Path,
    states: Sequence[VehicleState],
    run_start_s: float,
    separation: Mapping[str, Any],
    status: str,
) -> None:
    payload = {
        "schema_version": 1,
        "status": status,
        "elapsed_s": time.monotonic() - run_start_s,
        "vehicle_count": len(states),
        "vehicles": [vehicle_summary(state) for state in states],
        "separation": _json_safe_separation(separation),
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def snapshot_rows(
    writer: csv.DictWriter,
    states: Sequence[VehicleState],
    run_start_s: float,
) -> None:
    elapsed_s = time.monotonic() - run_start_s
    for state in states:
        position = state.position
        writer.writerow(
            {
                "time_s": elapsed_s,
                "vehicle_id": state.vehicle_id,
                "latitude_deg": None if position is None else position[0],
                "longitude_deg": None if position is None else position[1],
                "relative_altitude_m": None if position is None else position[2],
                "home_distance_m": home_distance_m(state),
                "armed": state.armed,
                "landed_state": state.landed_state,
                "current_sequence": state.current_sequence,
                "highest_sequence": state.highest_sequence,
                "custom_mode": state.custom_mode,
                "launched": state.launched,
                "completed": state.completed,
            }
        )


def load_execution_inputs(
    fleet_config_path: Path,
    upload_report_path: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    require(fleet_config_path.is_file(), f"missing {fleet_config_path}")
    require(upload_report_path.is_file(), f"missing {upload_report_path}")
    config = yaml.safe_load(fleet_config_path.read_text(encoding="utf-8"))
    report = json.loads(upload_report_path.read_text(encoding="utf-8"))

    require(isinstance(config, dict), "fleet configuration must be an object")
    vehicles = config.get("vehicles")
    require(
        isinstance(vehicles, list) and len(vehicles) >= 1,
        "fleet configuration must contain at least one vehicle",
    )
    require(report.get("all_verified") is True, "upload report is not verified")
    report_vehicles = report.get("vehicles")
    require(
        isinstance(report_vehicles, list),
        "upload report vehicles must be a list",
    )
    config_ids = [str(vehicle["vehicle_id"]) for vehicle in vehicles]
    report_ids = [str(vehicle["vehicle_id"]) for vehicle in report_vehicles]
    require(
        set(config_ids) == set(report_ids),
        "fleet configuration and upload report vehicle sets differ",
    )
    require(
        len(config_ids) == len(set(config_ids)),
        "fleet configuration contains duplicate vehicle IDs",
    )
    endpoints = [str(vehicle["endpoint"]) for vehicle in vehicles]
    system_ids = [int(vehicle["system_id"]) for vehicle in vehicles]
    require(
        len(endpoints) == len(set(endpoints)),
        "fleet configuration contains duplicate endpoints",
    )
    require(
        len(system_ids) == len(set(system_ids)),
        "fleet configuration contains duplicate system IDs",
    )
    return config, report


def connect_states(
    config: Mapping[str, Any],
    upload_report: Mapping[str, Any],
) -> list[VehicleState]:
    report_by_id = {
        str(item["vehicle_id"]): item
        for item in upload_report["vehicles"]
    }
    states: list[VehicleState] = []

    for vehicle in config["vehicles"]:
        vehicle_id = str(vehicle["vehicle_id"])
        endpoint = str(vehicle["endpoint"])
        system_id = int(vehicle["system_id"])
        component_id = int(vehicle["component_id"])
        frozen = report_by_id[vehicle_id]
        expected_wire_count = int(frozen["expected_wire_item_count"])

        print(f"Connecting {vehicle_id} at {endpoint} ...")
        connection = mavutil.mavlink_connection(
            endpoint,
            source_system=SOURCE_SYSTEM,
            source_component=SOURCE_COMPONENT,
            autoreconnect=False,
        )
        state = VehicleState(
            vehicle_id=vehicle_id,
            endpoint=endpoint,
            system_id=system_id,
            component_id=component_id,
            expected_wire_item_count=expected_wire_count,
            connection=connection,
        )
        heartbeat = wait_heartbeat(state)
        require(
            not heartbeat_is_armed(heartbeat),
            f"{vehicle_id}: unexpectedly armed",
        )
        state.custom_mode = int(heartbeat.custom_mode)
        state.system_status = int(heartbeat.system_status)
        state.last_heartbeat_s = time.monotonic()

        observed_count = request_mission_count(state)
        require(
            observed_count == expected_wire_count,
            (
                f"{vehicle_id}: expected {expected_wire_count} mission items, "
                f"observed {observed_count}"
            ),
        )
        state.home = request_home(state)
        for name, message_id, interval_us in (
            (
                "GLOBAL_POSITION_INT",
                mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                200_000,
            ),
            (
                "EXTENDED_SYS_STATE",
                mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
                500_000,
            ),
            (
                "MISSION_CURRENT",
                mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,
                500_000,
            ),
        ):
            set_message_interval(state, name, message_id, interval_us)

        states.append(state)
        _log(
            state,
            (
                f"PASS connected, disarmed, mission={observed_count}, "
                f"HOME={state.home[0]:.9f},{state.home[1]:.9f}"
            ),
        )

    return states


def collect_initial_state(
    states: Sequence[VehicleState],
    events_handle: Any,
    run_start_s: float,
    *,
    timeout_s: float = 15.0,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pump_all(states, events_handle, run_start_s)
        if all(
            state.position is not None
            and state.landed_state is not None
            and state.last_heartbeat_s is not None
            for state in states
        ):
            return
        time.sleep(0.02)

    missing = [
        state.vehicle_id
        for state in states
        if state.position is None or state.landed_state is None
    ]
    raise VariableFleetExecutionError(
        "initial telemetry incomplete for " + ", ".join(missing)
    )


def prepare_all_vehicles(
    states: Sequence[VehicleState],
    events_handle: Any,
    run_start_s: float,
    options: FleetExecutionOptions,
) -> None:
    print()
    print("Preparing every vehicle before the shared-HOME launch sequence ...")
    for state in states:
        observed = set_current_item(state, TAKEOFF_WIRE_SEQUENCE)
        _log(state, f"PASS mission current sequence {observed}")
        wait_for_position_estimate(state)
        mode = set_mode(state, "LOITER", expected_armed=False)
        _log(state, f"PASS LOITER selected, custom_mode={mode}")
        wait_for_prearm_ready(state)

    collect_initial_state(states, events_handle, run_start_s)
    for state in states:
        verify_prelaunch_vehicle(state, options)


def _request_land_for_active(states: Sequence[VehicleState]) -> None:
    for state in states:
        if state.launched and state.armed:
            request_land(state)


def run_variable_fleet(
    *,
    fleet_config_path: Path | str,
    upload_report_path: Path | str,
    report_path: Path | str,
    checkpoint_path: Path | str,
    telemetry_path: Path | str,
    events_path: Path | str,
    options: FleetExecutionOptions | None = None,
) -> dict[str, Any]:
    """Launch N verified SITL missions with shared-HOME staggering."""

    resolved_options = options or FleetExecutionOptions()
    fleet_config = Path(fleet_config_path)
    upload_report = Path(upload_report_path)
    report = Path(report_path)
    checkpoint = Path(checkpoint_path)
    telemetry = Path(telemetry_path)
    events = Path(events_path)

    for output in (report, checkpoint, telemetry, events):
        output.parent.mkdir(parents=True, exist_ok=True)
    require(not report.exists(), f"refusing to overwrite {report}")

    config, upload = load_execution_inputs(fleet_config, upload_report)
    states: list[VehicleState] = []
    run_start_s = time.monotonic()
    separation: dict[str, Any] = {
        "overall_minimum": None,
        "pairs": {},
        "warning_count": 0,
        "printed_warnings": set(),
    }

    try:
        states = connect_states(config, upload)
        with (
            telemetry.open("w", newline="", encoding="utf-8") as telemetry_handle,
            events.open("w", encoding="utf-8") as events_handle,
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
            prepare_all_vehicles(
                states,
                events_handle,
                run_start_s,
                resolved_options,
            )

            last_snapshot_s = 0.0
            last_progress_s = 0.0
            last_checkpoint_s = 0.0
            last_launch_wall_s: float | None = None

            for index, state in enumerate(states):
                previous_states = states[:index]
                stable_samples = 0
                gate_deadline = (
                    time.monotonic()
                    + resolved_options.launch_gate_timeout_s
                )
                print()
                print(f"Waiting to launch {state.vehicle_id} ...")

                while time.monotonic() < gate_deadline:
                    now = time.monotonic()
                    pump_all(states, events_handle, run_start_s)
                    update_pairwise_separation(
                        states,
                        run_start_s,
                        separation,
                        resolved_options,
                    )
                    for pending in states[index:]:
                        require(
                            not pending.armed,
                            (
                                f"{pending.vehicle_id}: armed before its "
                                "scheduled launch"
                            ),
                        )

                    gate_passed, reason = launch_gate_status(
                        previous_states,
                        resolved_options,
                        now_s=now,
                    )
                    spacing_passed = (
                        last_launch_wall_s is None
                        or now - last_launch_wall_s
                        >= resolved_options.minimum_launch_spacing_s
                    )
                    if gate_passed and spacing_passed:
                        stable_samples += 1
                    else:
                        stable_samples = 0

                    if (
                        stable_samples
                        >= resolved_options.launch_gate_stable_samples
                    ):
                        break

                    elapsed_s = now - run_start_s
                    if (
                        elapsed_s - last_snapshot_s
                        >= resolved_options.telemetry_snapshot_interval_s
                    ):
                        snapshot_rows(writer, states, run_start_s)
                        telemetry_handle.flush()
                        last_snapshot_s = elapsed_s
                    if (
                        elapsed_s - last_progress_s
                        >= resolved_options.progress_interval_s
                    ):
                        print(f"{state.vehicle_id} launch gate: {reason}")
                        last_progress_s = elapsed_s
                    time.sleep(0.1)
                else:
                    raise VariableFleetExecutionError(
                        f"{state.vehicle_id}: launch gate timeout"
                    )

                launch_vehicle(
                    state,
                    resolved_options,
                    run_start_s,
                    events_handle,
                )
                last_launch_wall_s = time.monotonic()

            print()
            print(
                f"ALL {len(states)} VEHICLES LAUNCHED — "
                "monitoring until every mission completes"
            )
            fleet_deadline = (
                time.monotonic() + resolved_options.fleet_timeout_s
            )

            while time.monotonic() < fleet_deadline:
                now = time.monotonic()
                elapsed_s = now - run_start_s
                pump_all(states, events_handle, run_start_s)
                update_pairwise_separation(
                    states,
                    run_start_s,
                    separation,
                    resolved_options,
                )

                for state in states:
                    require(
                        state.last_heartbeat_s is not None
                        and now - state.last_heartbeat_s
                        <= HEARTBEAT_STALE_S,
                        f"{state.vehicle_id}: heartbeat became stale",
                    )
                    if state.armed:
                        require(
                            state.last_position_s is not None
                            and now - state.last_position_s
                            <= POSITION_STALE_S,
                            f"{state.vehicle_id}: position became stale",
                        )

                    if not state.armed and not state.completed:
                        require(
                            state.saw_airborne,
                            (
                                f"{state.vehicle_id}: disarmed before "
                                "becoming airborne"
                            ),
                        )
                        require(
                            state.saw_final_sequence,
                            (
                                f"{state.vehicle_id}: disarmed before final "
                                f"mission item; highest={state.highest_sequence}"
                            ),
                        )
                        if (
                            state.landed_state
                            == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                        ):
                            final_distance = home_distance_m(state)
                            require(
                                final_distance is not None
                                and final_distance
                                <= resolved_options.maximum_final_home_distance_m,
                                (
                                    f"{state.vehicle_id}: landed "
                                    f"{final_distance}m from HOME"
                                ),
                            )
                            state.completed = True
                            state.completion_time_s = elapsed_s
                            write_event(
                                events_handle,
                                run_start_s,
                                state,
                                "COMPLETED",
                                {
                                    "completion_time_s": elapsed_s,
                                    "final_home_distance_m": final_distance,
                                },
                            )
                            _log(
                                state,
                                (
                                    f"COMPLETED at t={elapsed_s:.1f}s, "
                                    f"HOME distance={final_distance:.2f}m"
                                ),
                            )

                if (
                    elapsed_s - last_snapshot_s
                    >= resolved_options.telemetry_snapshot_interval_s
                ):
                    snapshot_rows(writer, states, run_start_s)
                    telemetry_handle.flush()
                    last_snapshot_s = elapsed_s

                if (
                    elapsed_s - last_progress_s
                    >= resolved_options.progress_interval_s
                ):
                    print()
                    for state in states:
                        altitude = (
                            0.0
                            if state.position is None
                            else state.position[2]
                        )
                        print(
                            f"{state.vehicle_id}: "
                            f"seq={state.current_sequence}/"
                            f"{state.expected_final_sequence}, "
                            f"alt={altitude:.1f}m, armed={state.armed}, "
                            f"completed={state.completed}"
                        )
                    last_progress_s = elapsed_s

                if (
                    elapsed_s - last_checkpoint_s
                    >= resolved_options.checkpoint_interval_s
                ):
                    write_checkpoint(
                        checkpoint,
                        states,
                        run_start_s,
                        separation,
                        "RUNNING",
                    )
                    last_checkpoint_s = elapsed_s

                if all(state.completed for state in states):
                    break
                time.sleep(0.02)
            else:
                raise VariableFleetExecutionError("fleet mission timeout")

            telemetry_handle.flush()

        result = {
            "schema_version": 1,
            "status": "PASSED",
            "execution_policy": "staggered_shared_home_clearance",
            "vehicle_count": len(states),
            "all_completed": True,
            "launch_clearance_altitude_m": (
                resolved_options.launch_clearance_altitude_m
            ),
            "launch_clearance_home_distance_m": (
                resolved_options.launch_clearance_home_distance_m
            ),
            "vehicles": [vehicle_summary(state) for state in states],
            "separation": _json_safe_separation(separation),
            "telemetry_path": str(telemetry),
            "events_path": str(events),
        }
        temporary = report.with_suffix(report.suffix + ".tmp")
        temporary.write_text(
            json.dumps(result, indent=2, sort_keys=True, allow_nan=False)
            + "\n",
            encoding="utf-8",
        )
        temporary.replace(report)
        write_checkpoint(
            checkpoint,
            states,
            run_start_s,
            separation,
            "PASSED",
        )
        print()
        print(
            f"PASSED: all {len(states)} missions completed, "
            "landed and disarmed"
        )
        if separation["overall_minimum"] is not None:
            minimum = separation["overall_minimum"]
            print(
                "Minimum observed airborne separation: "
                f"{minimum['minimum_3d_separation_m']:.2f}m "
                f"for {minimum['pair']}"
            )
        return result

    except BaseException:
        if states:
            _request_land_for_active(states)
            try:
                write_checkpoint(
                    checkpoint,
                    states,
                    run_start_s,
                    separation,
                    "FAILED",
                )
            except Exception:
                pass
        raise
    finally:
        for state in states:
            try:
                state.connection.close()
            except Exception:
                pass
