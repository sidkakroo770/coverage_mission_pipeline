from __future__ import annotations

import json
import math
import time
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
OUTPUT_PATH = ROOT / "stage21c-telemetry-dry-run-report.json"

SOURCE_SYSTEM = 250
SOURCE_COMPONENT = 196

CONNECT_TIMEOUT_S = 30.0
COMMAND_TIMEOUT_S = 10.0
COLLECTION_TIME_S = 12.0

MESSAGE_INTERVALS = {
    "GLOBAL_POSITION_INT": (
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        200_000,  # 5 Hz
    ),
    "EXTENDED_SYS_STATE": (
        mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
        500_000,  # 2 Hz
    ),
    "MISSION_CURRENT": (
        mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,
        500_000,  # 2 Hz
    ),
}


class ValidationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def is_armed(heartbeat: Any) -> bool:
    return bool(
        int(heartbeat.base_mode)
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


def drain_command_acks(connection: Any) -> None:
    while True:
        message = connection.recv_match(
            type="COMMAND_ACK",
            blocking=False,
        )

        if message is None:
            return


def set_message_interval(
    connection: Any,
    *,
    vehicle_id: str,
    system_id: int,
    component_id: int,
    message_name: str,
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
        message = connection.recv_match(
            type="COMMAND_ACK",
            blocking=True,
            timeout=1.0,
        )

        if message is None:
            continue

        if message.get_srcSystem() != system_id:
            continue

        if (
            int(message.command)
            != mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        ):
            continue

        result = int(message.result)

        require(
            result
            in {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            },
            (
                f"{vehicle_id}: interval request for "
                f"{message_name} rejected with MAV_RESULT {result}"
            ),
        )

        print(
            f"PASS {vehicle_id}: {message_name} interval accepted"
        )
        return

    raise ValidationError(
        f"{vehicle_id}: no ACK for {message_name} interval request"
    )


def request_home(
    connection: Any,
    *,
    vehicle_id: str,
    system_id: int,
    component_id: int,
) -> dict[str, float]:
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

    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        message = connection.recv_match(
            type="HOME_POSITION",
            blocking=True,
            timeout=1.0,
        )

        if message is None:
            continue

        if message.get_srcSystem() != system_id:
            continue

        return {
            "latitude_deg": float(message.latitude) / 1e7,
            "longitude_deg": float(message.longitude) / 1e7,
        }

    raise ValidationError(
        f"{vehicle_id}: timed out waiting for HOME_POSITION"
    )


def request_mission_count(
    connection: Any,
    *,
    vehicle_id: str,
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
        connection.mav.mission_request_list_send(
            system_id,
            component_id,
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

        if message.get_srcSystem() != system_id:
            continue

        return int(message.count)

    raise ValidationError(
        f"{vehicle_id}: timed out waiting for MISSION_COUNT"
    )


def main() -> None:
    require(CONFIG_PATH.is_file(), f"missing {CONFIG_PATH}")
    require(
        UPLOAD_REPORT_PATH.is_file(),
        f"missing {UPLOAD_REPORT_PATH}",
    )
    require(
        not OUTPUT_PATH.exists(),
        f"refusing to overwrite {OUTPUT_PATH}",
    )

    config = yaml.safe_load(
        CONFIG_PATH.read_text(encoding="utf-8")
    )

    upload_report = json.loads(
        UPLOAD_REPORT_PATH.read_text(encoding="utf-8")
    )

    require(
        upload_report["all_verified"] is True,
        "Stage 21C upload report is not verified",
    )
    require(
        upload_report["vehicle_count"] == 5,
        "Stage 21C upload report does not contain five vehicles",
    )

    report_by_id = {
        item["vehicle_id"]: item
        for item in upload_report["vehicles"]
    }

    connections: dict[str, Any] = {}
    states: dict[str, dict[str, Any]] = {}

    try:
        for vehicle in config["vehicles"]:
            vehicle_id = vehicle["vehicle_id"]
            endpoint = vehicle["endpoint"]
            system_id = int(vehicle["system_id"])
            component_id = int(vehicle["component_id"])

            print()
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
                not is_armed(heartbeat),
                f"{vehicle_id}: unexpectedly armed",
            )

            expected_wire_count = int(
                report_by_id[vehicle_id][
                    "expected_wire_item_count"
                ]
            )

            observed_wire_count = request_mission_count(
                connection,
                vehicle_id=vehicle_id,
                system_id=system_id,
                component_id=component_id,
            )

            require(
                observed_wire_count == expected_wire_count,
                (
                    f"{vehicle_id}: expected "
                    f"{expected_wire_count} mission items, "
                    f"received {observed_wire_count}"
                ),
            )

            home = request_home(
                connection,
                vehicle_id=vehicle_id,
                system_id=system_id,
                component_id=component_id,
            )

            for message_name, (
                message_id,
                interval_us,
            ) in MESSAGE_INTERVALS.items():
                set_message_interval(
                    connection,
                    vehicle_id=vehicle_id,
                    system_id=system_id,
                    component_id=component_id,
                    message_name=message_name,
                    message_id=message_id,
                    interval_us=interval_us,
                )

            connections[vehicle_id] = connection

            states[vehicle_id] = {
                "vehicle_id": vehicle_id,
                "endpoint": endpoint,
                "system_id": system_id,
                "component_id": component_id,
                "expected_wire_item_count": expected_wire_count,
                "observed_wire_item_count": observed_wire_count,
                "home": home,
                "counts": {
                    "HEARTBEAT": 0,
                    "GLOBAL_POSITION_INT": 0,
                    "EXTENDED_SYS_STATE": 0,
                    "MISSION_CURRENT": 0,
                },
                "latest_position": None,
                "latest_landed_state": None,
                "latest_mission_sequence": None,
                "latest_armed": False,
            }

        print()
        print(
            f"Collecting telemetry for {COLLECTION_TIME_S:.0f} s ..."
        )

        deadline = time.monotonic() + COLLECTION_TIME_S

        while time.monotonic() < deadline:
            for vehicle_id, connection in connections.items():
                state = states[vehicle_id]

                for _ in range(100):
                    message = connection.recv_match(
                        blocking=False,
                    )

                    if message is None:
                        break

                    if (
                        message.get_srcSystem()
                        != state["system_id"]
                    ):
                        continue

                    message_type = message.get_type()

                    if message_type not in state["counts"]:
                        continue

                    state["counts"][message_type] += 1

                    if message_type == "HEARTBEAT":
                        state["latest_armed"] = is_armed(message)

                    elif message_type == "GLOBAL_POSITION_INT":
                        state["latest_position"] = {
                            "latitude_deg": (
                                float(message.lat) / 1e7
                            ),
                            "longitude_deg": (
                                float(message.lon) / 1e7
                            ),
                            "relative_altitude_m": (
                                float(message.relative_alt)
                                / 1000.0
                            ),
                        }

                    elif message_type == "EXTENDED_SYS_STATE":
                        state["latest_landed_state"] = int(
                            message.landed_state
                        )

                    elif message_type == "MISSION_CURRENT":
                        state["latest_mission_sequence"] = int(
                            message.seq
                        )

            time.sleep(0.01)

        results = []

        for vehicle_id in sorted(states):
            state = states[vehicle_id]
            counts = state["counts"]

            require(
                counts["HEARTBEAT"] >= 3,
                f"{vehicle_id}: insufficient HEARTBEAT telemetry",
            )
            require(
                counts["GLOBAL_POSITION_INT"] >= 20,
                (
                    f"{vehicle_id}: only "
                    f"{counts['GLOBAL_POSITION_INT']} "
                    "GLOBAL_POSITION_INT messages"
                ),
            )
            require(
                counts["EXTENDED_SYS_STATE"] >= 8,
                (
                    f"{vehicle_id}: only "
                    f"{counts['EXTENDED_SYS_STATE']} "
                    "EXTENDED_SYS_STATE messages"
                ),
            )
            require(
                counts["MISSION_CURRENT"] >= 8,
                (
                    f"{vehicle_id}: only "
                    f"{counts['MISSION_CURRENT']} "
                    "MISSION_CURRENT messages"
                ),
            )
            require(
                state["latest_armed"] is False,
                f"{vehicle_id}: became armed during dry run",
            )
            require(
                state["latest_landed_state"]
                == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                (
                    f"{vehicle_id}: landed_state is "
                    f"{state['latest_landed_state']}"
                ),
            )
            require(
                state["latest_position"] is not None,
                f"{vehicle_id}: no position telemetry",
            )
            require(
                state["latest_mission_sequence"] is not None,
                f"{vehicle_id}: no mission sequence telemetry",
            )

            position = state["latest_position"]
            home = state["home"]

            distance_from_home_m = haversine_m(
                home["latitude_deg"],
                home["longitude_deg"],
                position["latitude_deg"],
                position["longitude_deg"],
            )

            require(
                distance_from_home_m <= 2.0,
                (
                    f"{vehicle_id}: {distance_from_home_m:.2f} m "
                    "from HOME before launch"
                ),
            )
            require(
                abs(position["relative_altitude_m"]) <= 2.0,
                (
                    f"{vehicle_id}: relative altitude is "
                    f"{position['relative_altitude_m']:.2f} m"
                ),
            )

            result = {
                **state,
                "distance_from_home_m": distance_from_home_m,
                "passed": True,
            }

            results.append(result)

            print(
                f"PASS {vehicle_id}: "
                f"position={counts['GLOBAL_POSITION_INT']}, "
                f"landed={counts['EXTENDED_SYS_STATE']}, "
                f"mission={counts['MISSION_CURRENT']}, "
                f"seq={state['latest_mission_sequence']}, "
                f"home_distance={distance_from_home_m:.3f} m"
            )

        output = {
            "schema_version": 1,
            "stage": "21C1",
            "status": "PASSED",
            "armed_any_vehicle": False,
            "changed_any_mode": False,
            "vehicle_count": len(results),
            "vehicles": results,
        }

        temporary = OUTPUT_PATH.with_suffix(".json.tmp")

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

        temporary.replace(OUTPUT_PATH)

        print()
        print(
            "PASSED: Stage 21C telemetry dry run for five vehicles"
        )
        print("No vehicle was armed and no mode was changed.")
        print(f"Report: {OUTPUT_PATH}")

    finally:
        for connection in connections.values():
            try:
                connection.close()
            except Exception:
                pass


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print()
        print(f"FAILED: {exc}")
        print("No vehicle was intentionally armed or mode-changed.")
        raise SystemExit(1)
