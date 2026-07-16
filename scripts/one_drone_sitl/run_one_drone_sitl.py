#!/usr/bin/env python3
"""Execute and monitor one verified ArduCopter AUTO mission in SITL."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any

from pymavlink import mavutil


SOURCE_SYSTEM = 255
SOURCE_COMPONENT = 198
COMMAND_TIMEOUT_S = 20.0
HEARTBEAT_STALE_S = 8.0
POSITION_STALE_S = 5.0
AIRBORNE_ALTITUDE_M = 3.0
PREARM_TIMEOUT_S = 120.0
RC_OVERRIDE_TIMEOUT_S = 15.0
RC_THROTTLE_LOW_PWM = 1000
POSITION_READY_TIMEOUT_S = 180.0
POSITION_READY_STREAK = 3


class RunError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RunError(message)


def is_armed(message: Any) -> bool:
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


def wait_heartbeat(connection: Any, system_id: int, component_id: int) -> Any:
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        heartbeat = recv_for_system(
            connection,
            system_id,
            types="HEARTBEAT",
            timeout=1.0,
        )
        if heartbeat is None:
            continue
        require(
            heartbeat.get_srcComponent() == component_id,
            f"expected component {component_id}, got {heartbeat.get_srcComponent()}",
        )
        return heartbeat
    raise RunError("heartbeat timeout")


def request_mission_count(
    connection: Any,
    system_id: int,
    component_id: int,
) -> int:
    try:
        connection.mav.mission_request_list_send(
            system_id,
            component_id,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        connection.mav.mission_request_list_send(system_id, component_id)

    message = recv_for_system(
        connection,
        system_id,
        types="MISSION_COUNT",
        timeout=COMMAND_TIMEOUT_S,
    )
    if message is None:
        raise RunError("MISSION_COUNT timeout")
    return int(message.count)


def request_home(
    connection: Any,
    system_id: int,
    component_id: int,
) -> tuple[float, float]:
    connection.mav.command_long_send(
        system_id,
        component_id,
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
        connection,
        system_id,
        types="HOME_POSITION",
        timeout=COMMAND_TIMEOUT_S,
    )
    if message is None:
        raise RunError("HOME_POSITION timeout")
    return float(message.latitude) / 1e7, float(message.longitude) / 1e7


def set_message_interval(
    connection: Any,
    system_id: int,
    component_id: int,
    message_id: int,
    interval_us: int,
) -> None:
    drain_command_acks(connection)
    connection.mav.command_long_send(
        system_id,
        component_id,
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
            connection,
            system_id,
            types="COMMAND_ACK",
            timeout=1.0,
        )
        if message is None:
            continue
        if int(message.command) != mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            continue
        require(
            int(message.result)
            in {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            },
            f"SET_MESSAGE_INTERVAL rejected with MAV_RESULT {int(message.result)}",
        )
        return
    raise RunError("SET_MESSAGE_INTERVAL ACK timeout")


def set_current_item(
    connection: Any,
    system_id: int,
    component_id: int,
    sequence: int,
) -> None:
    while connection.recv_match(type="MISSION_CURRENT", blocking=False) is not None:
        pass

    deadline = time.monotonic() + COMMAND_TIMEOUT_S
    next_send = 0.0
    last_seen: int | None = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:
            try:
                connection.mav.mission_set_current_send(
                    system_id,
                    component_id,
                    sequence,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                connection.mav.mission_set_current_send(
                    system_id,
                    component_id,
                    sequence,
                )
            next_send = now + 1.0

        message = recv_for_system(
            connection,
            system_id,
            types="MISSION_CURRENT",
            timeout=0.5,
        )
        if message is None:
            continue
        last_seen = int(message.seq)
        if last_seen >= sequence:
            print(f"PASS: mission current sequence is {last_seen}")
            return

    raise RunError(
        f"MISSION_CURRENT timeout requesting {sequence}; last observed {last_seen}"
    )


def set_mode(
    connection: Any,
    system_id: int,
    mode_name: str,
    *,
    expected_armed: bool,
) -> None:
    mapping = connection.mode_mapping()
    require(mapping is not None and mode_name in mapping, f"mode {mode_name} unavailable")
    requested = int(mapping[mode_name])
    connection.mav.set_mode_send(
        system_id,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        requested,
    )

    deadline = time.monotonic() + COMMAND_TIMEOUT_S
    while time.monotonic() < deadline:
        heartbeat = recv_for_system(
            connection,
            system_id,
            types="HEARTBEAT",
            timeout=1.0,
        )
        if heartbeat is None:
            continue
        require(
            is_armed(heartbeat) == expected_armed,
            f"armed state changed while selecting {mode_name}",
        )
        if int(heartbeat.custom_mode) == requested:
            print(f"PASS: {mode_name} selected")
            return
    raise RunError(f"timed out entering {mode_name}")





def wait_for_position_estimate(
    connection: Any,
    system_id: int,
    component_id: int,
    *,
    timeout_s: float = POSITION_READY_TIMEOUT_S,
) -> None:
    """Wait for a stable GPS-backed absolute horizontal EKF estimate."""

    ekf_message_id = int(
        getattr(
            mavutil.mavlink,
            "MAVLINK_MSG_ID_EKF_STATUS_REPORT",
            193,
        )
    )
    gps_message_id = int(
        getattr(
            mavutil.mavlink,
            "MAVLINK_MSG_ID_GPS_RAW_INT",
            24,
        )
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

    for message_id, interval_us in (
        (gps_message_id, 500_000),
        (ekf_message_id, 500_000),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500_000),
    ):
        set_message_interval(
            connection,
            system_id,
            component_id,
            message_id,
            interval_us,
        )

    print("Waiting for GPS 3D fix and stable EKF horizontal position ...")

    deadline = time.monotonic() + timeout_s
    last_report_s = 0.0
    gps_fix_type: int | None = None
    satellites: int | None = None
    gps_eph: int | None = None
    ekf_flags: int | None = None
    global_position_seen = False
    ready_streak = 0
    last_reason: str | None = None

    while time.monotonic() < deadline:
        message = recv_for_system(
            connection,
            system_id,
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
                not is_armed(message),
                "vehicle armed unexpectedly during position-estimate wait",
            )

        elif message_type == "STATUSTEXT":
            text = str(message.text).strip()
            if text and text != last_reason:
                print(f"STATUSTEXT: {text}")
                last_reason = text

        elif message_type == "GPS_RAW_INT":
            gps_fix_type = int(message.fix_type)
            satellites = int(message.satellites_visible)
            gps_eph = int(message.eph)

        elif message_type == "GLOBAL_POSITION_INT":
            latitude = int(message.lat)
            longitude = int(message.lon)
            global_position_seen = latitude != 0 and longitude != 0

        elif message_type == "EKF_STATUS_REPORT":
            ekf_flags = int(message.flags)

            gps_ready = gps_fix_type is not None and gps_fix_type >= 3
            absolute_ready = bool(
                ekf_flags
                & (ekf_pos_horiz_abs | ekf_pred_pos_horiz_abs)
            )
            invalid_mode = bool(
                ekf_flags & (ekf_const_pos_mode | ekf_uninitialized)
            )

            if gps_ready and absolute_ready and not invalid_mode:
                ready_streak += 1
            else:
                ready_streak = 0

            if ready_streak >= POSITION_READY_STREAK:
                print(
                    "PASS: GPS/EKF position estimate is stable "
                    f"(fix_type={gps_fix_type}, satellites={satellites}, "
                    f"EKF flags=0x{ekf_flags:04x})"
                )
                return

        if now - last_report_s >= 5.0:
            fix_text = "none" if gps_fix_type is None else str(gps_fix_type)
            satellites_text = "none" if satellites is None else str(satellites)
            eph_text = "none" if gps_eph is None else str(gps_eph)
            flags_text = "none" if ekf_flags is None else f"0x{ekf_flags:04x}"
            print(
                "Position not ready: "
                f"GPS fix={fix_text}, sats={satellites_text}, eph={eph_text}, "
                f"EKF flags={flags_text}, global_position={global_position_seen}, "
                f"stable={ready_streak}/{POSITION_READY_STREAK}"
            )
            last_report_s = now

    raise RunError(
        "GPS/EKF position estimate did not become ready within "
        f"{timeout_s:.0f}s; "
        f"fix_type={gps_fix_type}, satellites={satellites}, eph={gps_eph}, "
        f"EKF_flags={None if ekf_flags is None else hex(ekf_flags)}, "
        f"global_position={global_position_seen}"
        + ("" if not last_reason else f"; last status: {last_reason}")
    )


def send_rc_override(
    connection: Any,
    system_id: int,
    component_id: int,
    *,
    roll_pwm: int,
    pitch_pwm: int,
    throttle_pwm: int,
    yaw_pwm: int,
) -> None:
    """Send a MAVLink2 RC override, with an MAVLink1 fallback."""

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
        connection.mav.rc_channels_override_send(
            system_id,
            component_id,
            *channels_18,
        )
    except TypeError:
        connection.mav.rc_channels_override_send(
            system_id,
            component_id,
            *channels_18[:8],
        )


def release_rc_override(
    connection: Any,
    system_id: int,
    component_id: int,
) -> None:
    """Release the first eight overridden RC channels."""

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
        connection.mav.rc_channels_override_send(
            system_id,
            component_id,
            *channels_18,
        )
    except TypeError:
        connection.mav.rc_channels_override_send(
            system_id,
            component_id,
            *channels_18[:8],
        )


def establish_low_throttle_rc(
    connection: Any,
    system_id: int,
    component_id: int,
    *,
    timeout_s: float = RC_OVERRIDE_TIMEOUT_S,
) -> None:
    """Provide valid centered SITL RC input with throttle low and verify it."""

    set_message_interval(
        connection,
        system_id,
        component_id,
        mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
        200_000,
    )

    print(
        "Providing SITL RC input: roll=1500, pitch=1500, "
        f"throttle={RC_THROTTLE_LOW_PWM}, yaw=1500"
    )

    deadline = time.monotonic() + timeout_s
    next_send_s = 0.0
    last_throttle: int | None = None

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send_s:
            send_rc_override(
                connection,
                system_id,
                component_id,
                roll_pwm=1500,
                pitch_pwm=1500,
                throttle_pwm=RC_THROTTLE_LOW_PWM,
                yaw_pwm=1500,
            )
            next_send_s = now + 0.5

        message = recv_for_system(
            connection,
            system_id,
            types=["RC_CHANNELS", "STATUSTEXT", "HEARTBEAT"],
            timeout=0.5,
        )
        if message is None:
            continue

        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            require(
                not is_armed(message),
                "vehicle armed unexpectedly while establishing RC input",
            )
        elif message_type == "STATUSTEXT":
            text = str(message.text).strip()
            if text:
                print(f"STATUSTEXT: {text}")
        elif message_type == "RC_CHANNELS":
            last_throttle = int(message.chan3_raw)
            if 950 <= last_throttle <= 1100:
                print(
                    "PASS: ArduPilot received low throttle "
                    f"on RC channel 3 ({last_throttle} us)"
                )
                # Refresh once immediately before the arm command.
                send_rc_override(
                    connection,
                    system_id,
                    component_id,
                    roll_pwm=1500,
                    pitch_pwm=1500,
                    throttle_pwm=RC_THROTTLE_LOW_PWM,
                    yaw_pwm=1500,
                )
                return

    raise RunError(
        "ArduPilot did not accept the low-throttle RC override"
        + (
            ""
            if last_throttle is None
            else f"; last RC3 value was {last_throttle} us"
        )
        + "; verify SYSID_MYGCS accepts source system 255"
    )


def wait_for_prearm_ready(
    connection: Any,
    system_id: int,
    component_id: int,
    *,
    timeout_s: float = PREARM_TIMEOUT_S,
) -> None:
    """Wait until ArduPilot reports that all enabled pre-arm checks are healthy."""

    prearm_bit = int(
        getattr(
            mavutil.mavlink,
            "MAV_SYS_STATUS_PREARM_CHECK",
            1 << 28,
        )
    )

    # Ask for SYS_STATUS at 2 Hz. STATUSTEXT is received asynchronously and is
    # printed below so any persistent PreArm reason remains visible.
    set_message_interval(
        connection,
        system_id,
        component_id,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        500_000,
    )

    print("Waiting for ArduPilot pre-arm checks to pass ...")

    deadline = time.monotonic() + timeout_s
    last_report_s = 0.0
    last_reason: str | None = None
    saw_sys_status = False

    while time.monotonic() < deadline:
        message = recv_for_system(
            connection,
            system_id,
            timeout=1.0,
        )
        now = time.monotonic()

        if message is None:
            continue

        message_type = message.get_type()

        if message_type == "HEARTBEAT":
            require(
                not is_armed(message),
                "vehicle armed unexpectedly during pre-arm wait",
            )

        elif message_type == "STATUSTEXT":
            text = str(message.text).strip()
            if text:
                if text != last_reason:
                    print(f"STATUSTEXT: {text}")
                    last_reason = text

        elif message_type == "SYS_STATUS":
            saw_sys_status = True
            enabled = int(message.onboard_control_sensors_enabled)
            health = int(message.onboard_control_sensors_health)

            check_enabled = bool(enabled & prearm_bit)
            check_healthy = bool(health & prearm_bit)

            if check_enabled and check_healthy:
                print("PASS: ArduPilot pre-arm checks are healthy")
                return

            if now - last_report_s >= 5.0:
                print(
                    "Pre-arm not ready: "
                    f"enabled={check_enabled}, healthy={check_healthy}"
                )
                last_report_s = now

    detail = (
        f"; last status: {last_reason}"
        if last_reason
        else ""
    )
    if not saw_sys_status:
        detail += "; no SYS_STATUS messages were received"

    raise RunError(
        f"pre-arm checks did not become healthy within {timeout_s:.0f}s"
        + detail
    )


def arm(
    connection: Any,
    system_id: int,
    component_id: int,
) -> None:
    drain_command_acks(connection)
    connection.mav.command_long_send(
        system_id,
        component_id,
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
        message = recv_for_system(connection, system_id, timeout=1.0)
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "STATUSTEXT":
            print(f"STATUSTEXT: {message.text}")
        elif message_type == "COMMAND_ACK":
            if int(message.command) == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                result = int(message.result)
                if result not in {
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                }:
                    reasons: list[str] = []
                    reason_deadline = time.monotonic() + 3.0
                    while time.monotonic() < reason_deadline:
                        followup = recv_for_system(
                            connection,
                            system_id,
                            timeout=0.5,
                        )
                        if followup is None:
                            continue
                        if followup.get_type() == "STATUSTEXT":
                            text = str(followup.text).strip()
                            if text:
                                reasons.append(text)
                                print(f"STATUSTEXT: {text}")
                    result_name = mavutil.mavlink.enums[
                        "MAV_RESULT"
                    ][result].name
                    raise RunError(
                        "arming rejected with "
                        f"{result_name} (MAV_RESULT {result})"
                        + (
                            ""
                            if not reasons
                            else "; " + " | ".join(reasons)
                        )
                    )
                accepted = True
        elif message_type == "HEARTBEAT" and is_armed(message):
            require(accepted, "vehicle armed without an accepted arm ACK")
            print("PASS: vehicle armed")
            return
    raise RunError("arming timeout")


def start_mission(
    connection: Any,
    system_id: int,
    component_id: int,
) -> None:
    drain_command_acks(connection)
    connection.mav.command_long_send(
        system_id,
        component_id,
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
        message = recv_for_system(connection, system_id, timeout=1.0)
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            print(f"STATUSTEXT: {message.text}")
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
            f"MISSION_START rejected with MAV_RESULT {result}",
        )
        print("PASS: mission start accepted")
        return
    raise RunError("MISSION_START ACK timeout")


def request_land(connection: Any, system_id: int) -> None:
    try:
        mapping = connection.mode_mapping()
        if mapping and "LAND" in mapping:
            connection.mav.set_mode_send(
                system_id,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                int(mapping["LAND"]),
            )
            print("Emergency request: LAND mode sent")
    except Exception as exc:
        print(f"WARNING: could not request LAND: {exc}", file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint", default="tcp:127.0.0.1:5760")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=1)
    parser.add_argument("--timeout-s", type=float, default=1800.0)
    parser.add_argument("--max-final-home-distance-m", type=float, default=10.0)
    args = parser.parse_args()

    connection = mavutil.mavlink_connection(
        args.endpoint,
        source_system=SOURCE_SYSTEM,
        source_component=SOURCE_COMPONENT,
        autoreconnect=False,
    )

    launched = False
    try:
        heartbeat = wait_heartbeat(connection, args.system_id, args.component_id)
        require(not is_armed(heartbeat), "vehicle is already armed")
        print(
            f"PASS: connected to sysid={args.system_id}, "
            f"compid={args.component_id}, disarmed"
        )

        mission_count = request_mission_count(
            connection,
            args.system_id,
            args.component_id,
        )
        require(mission_count >= 3, f"uploaded mission is too short: {mission_count}")
        final_sequence = mission_count - 1
        print(f"PASS: uploaded wire mission contains {mission_count} items")

        home_lat, home_lon = request_home(
            connection,
            args.system_id,
            args.component_id,
        )
        print(f"PASS: HOME is {home_lat:.9f}, {home_lon:.9f}")

        for message_id, interval_us in (
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200_000),
            (mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 500_000),
            (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 500_000),
        ):
            set_message_interval(
                connection,
                args.system_id,
                args.component_id,
                message_id,
                interval_us,
            )
        print("PASS: telemetry intervals configured")

        set_current_item(connection, args.system_id, args.component_id, 1)
        wait_for_position_estimate(
            connection,
            args.system_id,
            args.component_id,
        )
        set_mode(
            connection,
            args.system_id,
            "LOITER",
            expected_armed=False,
        )
        wait_for_prearm_ready(
            connection,
            args.system_id,
            args.component_id,
        )
        establish_low_throttle_rc(
            connection,
            args.system_id,
            args.component_id,
        )
        arm(connection, args.system_id, args.component_id)
        launched = True
        set_mode(
            connection,
            args.system_id,
            "AUTO",
            expected_armed=True,
        )
        start_mission(connection, args.system_id, args.component_id)
        release_rc_override(
            connection,
            args.system_id,
            args.component_id,
        )
        print("PASS: SITL RC override released")

        print("MISSION LAUNCHED — monitoring until LAND and disarm")

        start_time = time.monotonic()
        deadline = start_time + args.timeout_s
        last_heartbeat = time.monotonic()
        last_position: float | None = None
        last_progress = 0.0
        armed = True
        landed_state: int | None = None
        current_sequence = 1
        highest_sequence = 1
        position: tuple[float, float, float] | None = None
        saw_airborne = False
        saw_final_sequence = False

        while time.monotonic() < deadline:
            message = recv_for_system(
                connection,
                args.system_id,
                timeout=0.5,
            )
            now = time.monotonic()

            if message is not None:
                message_type = message.get_type()
                if message_type == "HEARTBEAT":
                    last_heartbeat = now
                    armed = is_armed(message)
                elif message_type == "GLOBAL_POSITION_INT":
                    last_position = now
                    position = (
                        float(message.lat) / 1e7,
                        float(message.lon) / 1e7,
                        float(message.relative_alt) / 1000.0,
                    )
                    if position[2] >= AIRBORNE_ALTITUDE_M:
                        saw_airborne = True
                elif message_type == "EXTENDED_SYS_STATE":
                    landed_state = int(message.landed_state)
                    if landed_state in {
                        mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF,
                        mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
                        mavutil.mavlink.MAV_LANDED_STATE_LANDING,
                    }:
                        saw_airborne = True
                elif message_type == "MISSION_CURRENT":
                    current_sequence = int(message.seq)
                    highest_sequence = max(highest_sequence, current_sequence)
                    if current_sequence >= final_sequence:
                        saw_final_sequence = True
                elif message_type == "STATUSTEXT":
                    print(f"STATUSTEXT: {message.text}")

            require(
                now - last_heartbeat <= HEARTBEAT_STALE_S,
                "heartbeat became stale",
            )
            if armed and saw_airborne:
                require(
                    last_position is not None
                    and now - last_position <= POSITION_STALE_S,
                    "position telemetry became stale",
                )

            elapsed = now - start_time
            if elapsed - last_progress >= 5.0:
                if position is None:
                    position_text = "position unavailable"
                else:
                    distance = haversine_m(
                        home_lat,
                        home_lon,
                        position[0],
                        position[1],
                    )
                    position_text = (
                        f"alt={position[2]:.1f} m, "
                        f"home_distance={distance:.1f} m"
                    )
                print(
                    f"t={elapsed:.1f}s seq={current_sequence}/{final_sequence} "
                    f"armed={armed} landed_state={landed_state} {position_text}"
                )
                last_progress = elapsed

            if (
                launched
                and saw_airborne
                and saw_final_sequence
                and not armed
                and landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
            ):
                require(position is not None, "final position unavailable")
                final_distance = haversine_m(
                    home_lat,
                    home_lon,
                    position[0],
                    position[1],
                )
                require(
                    final_distance <= args.max_final_home_distance_m,
                    f"landed {final_distance:.2f} m from HOME",
                )
                print(
                    "PASS: one-drone SITL mission completed, "
                    f"landed and disarmed {final_distance:.2f} m from HOME; "
                    f"highest sequence {highest_sequence}/{final_sequence}"
                )
                return 0

        raise RunError(
            f"mission timeout after {args.timeout_s:.1f}s; "
            f"highest sequence {highest_sequence}/{final_sequence}"
        )

    except KeyboardInterrupt:
        if launched:
            request_land(connection, args.system_id)
        print("\nINTERRUPTED", file=sys.stderr)
        return 130
    except RunError as exc:
        if launched:
            request_land(connection, args.system_id)
        print(f"FAILED: {exc}", file=sys.stderr)
        return 2
    finally:
        try:
            connection.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
