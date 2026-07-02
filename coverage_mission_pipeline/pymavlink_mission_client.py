#!/usr/bin/env python3
"""Direct MAVLink mission upload/readback client backed by pymavlink."""

from __future__ import annotations

import time
from typing import Any

from .fleet_config import FleetUploadConfig, FleetVehicleConfig
from .mission_fingerprint import MissionItemRecord, wire_int_frame
from .mission_protocol import MissionUploadError, VehicleIdentity


class PymavlinkMissionClient:
    """One explicit endpoint, one expected vehicle, no global vehicle selector."""

    def __init__(
        self,
        vehicle: FleetVehicleConfig,
        fleet: FleetUploadConfig,
    ) -> None:
        self._vehicle = vehicle
        self._fleet = fleet
        self._connection: Any = None
        self._mavutil: Any = None
        self._identity: VehicleIdentity | None = None

    def _import_pymavlink(self) -> Any:
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise MissionUploadError(
                "pymavlink is unavailable to this Python interpreter; install it "
                "or run from the environment that provides MAVProxy"
            ) from exc
        return mavutil

    @property
    def _mav(self) -> Any:
        if self._connection is None:
            raise MissionUploadError("MAVLink connection is not open")
        return self._connection.mav

    def connect(self) -> VehicleIdentity:
        self._mavutil = self._import_pymavlink()
        try:
            self._connection = self._mavutil.mavlink_connection(
                self._vehicle.endpoint,
                source_system=self._fleet.source_system,
                source_component=self._fleet.source_component,
                autoreconnect=False,
            )
            heartbeat = self._connection.wait_heartbeat(
                timeout=self._fleet.timeouts.heartbeat_s
            )
        except Exception as exc:
            self.close()
            raise MissionUploadError(
                f"{self._vehicle.vehicle_id}: could not connect to "
                f"{self._vehicle.endpoint!r}: {exc}"
            ) from exc
        if heartbeat is None:
            self.close()
            raise MissionUploadError(
                f"{self._vehicle.vehicle_id}: heartbeat timeout on "
                f"{self._vehicle.endpoint!r}"
            )
        identity = VehicleIdentity(
            system_id=int(heartbeat.get_srcSystem()),
            component_id=int(heartbeat.get_srcComponent()),
            autopilot=int(getattr(heartbeat, "autopilot", 0)),
            vehicle_type=int(getattr(heartbeat, "type", 0)),
        )
        self._identity = identity
        try:
            self._wait_until_ready(heartbeat)
        except Exception:
            self.close()
            raise
        return identity

    def _ensure_identity(self) -> VehicleIdentity:
        if self._identity is None:
            raise MissionUploadError("vehicle identity is not established")
        return self._identity

    def _target(self) -> tuple[int, int]:
        identity = self._ensure_identity()
        return identity.system_id, identity.component_id

    def _drain(self) -> None:
        if self._connection is None:
            return
        while self._connection.recv_match(blocking=False) is not None:
            pass

    def _recv(self, types: tuple[str, ...], timeout_s: float) -> Any:
        identity = self._ensure_identity()
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            message = self._connection.recv_match(
                type=list(types),
                blocking=True,
                timeout=remaining,
            )
            if message is None:
                return None
            if int(message.get_srcSystem()) != identity.system_id:
                continue
            # MISSION messages are normally emitted by component 1. Do not
            # accept another component on the same endpoint silently.
            if int(message.get_srcComponent()) != identity.component_id:
                continue
            return message

    def _mission_result_name(self, result: int) -> str:
        try:
            enum = self._mavutil.mavlink.enums["MAV_MISSION_RESULT"][int(result)]
            return str(enum.name)
        except Exception:
            return str(result)

    def _mission_type(self) -> int:
        return int(self._mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def _state_name(self, state: int) -> str:
        try:
            enum = self._mavutil.mavlink.enums["MAV_STATE"][int(state)]
            return str(enum.name)
        except Exception:
            return str(state)

    def _heartbeat_is_armed(self, heartbeat: Any) -> bool:
        flag = int(self._mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return bool(int(heartbeat.base_mode) & flag)

    def _wait_until_ready(self, first_heartbeat: Any) -> None:
        """Wait for completed autopilot startup, not merely the first heartbeat.

        ArduPilot SITL may accept a TCP client and emit an early heartbeat while
        vehicle initialisation is still in progress. During that interval the
        mission object exists but reports zero capacity. Requiring a stable
        disarmed STANDBY state keeps mission upload behind completed startup.
        """

        standby = int(self._mavutil.mavlink.MAV_STATE_STANDBY)
        required = self._fleet.timeouts.ready_heartbeats
        deadline = time.monotonic() + self._fleet.timeouts.ready_s
        streak = 0
        last_state: int | None = None
        heartbeat: Any | None = first_heartbeat

        while True:
            if heartbeat is not None:
                state = int(getattr(heartbeat, "system_status", -1))
                last_state = state
                if self._heartbeat_is_armed(heartbeat):
                    raise MissionUploadError(
                        f"{self._vehicle.vehicle_id}: vehicle became armed while "
                        "waiting for startup readiness"
                    )
                if state == standby:
                    streak += 1
                else:
                    streak = 0
                if streak >= required:
                    break

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                state_text = (
                    "none" if last_state is None else self._state_name(last_state)
                )
                raise MissionUploadError(
                    f"{self._vehicle.vehicle_id}: vehicle did not reach stable "
                    f"disarmed MAV_STATE_STANDBY within "
                    f"{self._fleet.timeouts.ready_s:.1f}s; "
                    f"last_state={state_text}, ready_heartbeats={streak}/{required}"
                )
            heartbeat = self._recv(
                ("HEARTBEAT",),
                min(self._fleet.timeouts.heartbeat_s, remaining),
            )

        grace_deadline = time.monotonic() + self._fleet.timeouts.startup_grace_s
        while True:
            remaining = grace_deadline - time.monotonic()
            if remaining <= 0.0:
                return
            heartbeat = self._recv(
                ("HEARTBEAT",),
                min(self._fleet.timeouts.heartbeat_s, remaining),
            )
            if heartbeat is None:
                continue
            if self._heartbeat_is_armed(heartbeat):
                raise MissionUploadError(
                    f"{self._vehicle.vehicle_id}: vehicle became armed during "
                    "the startup grace period"
                )
            state = int(getattr(heartbeat, "system_status", -1))
            if state != standby:
                raise MissionUploadError(
                    f"{self._vehicle.vehicle_id}: vehicle left MAV_STATE_STANDBY "
                    f"during startup grace period; state={self._state_name(state)}"
                )

    def _send_clear_all(self) -> None:
        target_system, target_component = self._target()
        try:
            self._mav.mission_clear_all_send(
                target_system,
                target_component,
                self._mission_type(),
            )
        except TypeError:
            self._mav.mission_clear_all_send(target_system, target_component)

    def _send_count(self, count: int) -> None:
        target_system, target_component = self._target()
        try:
            self._mav.mission_count_send(
                target_system,
                target_component,
                count,
                self._mission_type(),
            )
        except TypeError:
            self._mav.mission_count_send(target_system, target_component, count)

    def _send_request_list(self) -> None:
        target_system, target_component = self._target()
        try:
            self._mav.mission_request_list_send(
                target_system,
                target_component,
                self._mission_type(),
            )
        except TypeError:
            self._mav.mission_request_list_send(target_system, target_component)

    def _send_request_int(self, seq: int) -> None:
        target_system, target_component = self._target()
        try:
            self._mav.mission_request_int_send(
                target_system,
                target_component,
                seq,
                self._mission_type(),
            )
        except TypeError:
            self._mav.mission_request_int_send(
                target_system,
                target_component,
                seq,
            )

    def _send_ack(self, result: int) -> None:
        target_system, target_component = self._target()
        try:
            self._mav.mission_ack_send(
                target_system,
                target_component,
                result,
                self._mission_type(),
            )
        except TypeError:
            self._mav.mission_ack_send(target_system, target_component, result)

    def _send_item_int(self, record: MissionItemRecord) -> None:
        target_system, target_component = self._target()
        try:
            self._mav.mission_item_int_send(
                target_system,
                target_component,
                record.seq,
                wire_int_frame(record.frame),
                record.command,
                1 if record.seq == 0 else 0,
                record.autocontinue,
                record.param1,
                record.param2,
                record.param3,
                record.param4,
                record.x,
                record.y,
                record.z,
                self._mission_type(),
            )
        except TypeError:
            self._mav.mission_item_int_send(
                target_system,
                target_component,
                record.seq,
                wire_int_frame(record.frame),
                record.command,
                1 if record.seq == 0 else 0,
                record.autocontinue,
                record.param1,
                record.param2,
                record.param3,
                record.param4,
                record.x,
                record.y,
                record.z,
            )

    def _send_item_float(self, record: MissionItemRecord) -> None:
        target_system, target_component = self._target()
        latitude = record.x / 10_000_000.0
        longitude = record.y / 10_000_000.0
        try:
            self._mav.mission_item_send(
                target_system,
                target_component,
                record.seq,
                record.frame,
                record.command,
                1 if record.seq == 0 else 0,
                record.autocontinue,
                record.param1,
                record.param2,
                record.param3,
                record.param4,
                latitude,
                longitude,
                record.z,
                self._mission_type(),
            )
        except TypeError:
            self._mav.mission_item_send(
                target_system,
                target_component,
                record.seq,
                record.frame,
                record.command,
                1 if record.seq == 0 else 0,
                record.autocontinue,
                record.param1,
                record.param2,
                record.param3,
                record.param4,
                latitude,
                longitude,
                record.z,
            )

    def is_armed(self) -> bool:
        identity = self._ensure_identity()
        message = self._recv(("HEARTBEAT",), self._fleet.timeouts.heartbeat_s)
        if message is None:
            raise MissionUploadError(
                f"{self._vehicle.vehicle_id}: heartbeat timeout while checking armed state"
            )
        if int(message.get_srcSystem()) != identity.system_id:
            raise MissionUploadError("heartbeat identity changed unexpectedly")
        return self._heartbeat_is_armed(message)

    def clear_mission(self) -> None:
        self._drain()
        for attempt in range(1, self._fleet.timeouts.retries + 1):
            self._send_clear_all()
            ack = self._recv(("MISSION_ACK",), self._fleet.timeouts.clear_ack_s)
            if ack is None:
                continue
            result = int(ack.type)
            if result == int(self._mavutil.mavlink.MAV_MISSION_ACCEPTED):
                return
            raise MissionUploadError(
                f"{self._vehicle.vehicle_id}: mission clear rejected: "
                f"{self._mission_result_name(result)}"
            )
        raise MissionUploadError(
            f"{self._vehicle.vehicle_id}: mission clear timed out after "
            f"{self._fleet.timeouts.retries} attempt(s)"
        )

    def upload_mission(self, records: tuple[MissionItemRecord, ...]) -> None:
        if not records:
            raise MissionUploadError("cannot upload an empty mission")
        self._drain()
        self._send_count(len(records))
        deadline = time.monotonic() + (
            self._fleet.timeouts.item_request_s * len(records)
            + self._fleet.timeouts.upload_ack_s
        )
        sent: set[int] = set()
        consecutive_timeouts = 0

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise MissionUploadError(
                    f"{self._vehicle.vehicle_id}: mission upload exceeded its overall timeout"
                )
            message = self._recv(
                ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"),
                min(self._fleet.timeouts.item_request_s, remaining),
            )
            if message is None:
                consecutive_timeouts += 1
                if consecutive_timeouts >= self._fleet.timeouts.retries:
                    raise MissionUploadError(
                        f"{self._vehicle.vehicle_id}: mission upload request timeout"
                    )
                self._send_count(len(records))
                continue
            consecutive_timeouts = 0
            message_type = message.get_type()
            if message_type == "MISSION_ACK":
                result = int(message.type)
                if result != int(self._mavutil.mavlink.MAV_MISSION_ACCEPTED):
                    result_name = self._mission_result_name(result)
                    detail = ""
                    if result_name == "MAV_MISSION_NO_SPACE":
                        detail = (
                            f" for {len(records)} item(s) after startup readiness; "
                            "the autopilot mission capacity may be smaller than "
                            "the generated mission"
                        )
                    raise MissionUploadError(
                        f"{self._vehicle.vehicle_id}: mission upload rejected: "
                        f"{result_name}{detail}"
                    )
                if len(sent) != len(records):
                    missing = sorted(set(range(len(records))) - sent)
                    raise MissionUploadError(
                        f"{self._vehicle.vehicle_id}: autopilot acknowledged upload "
                        f"before requesting every item; missing {missing}"
                    )
                return

            seq = int(message.seq)
            if seq < 0 or seq >= len(records):
                raise MissionUploadError(
                    f"{self._vehicle.vehicle_id}: autopilot requested invalid mission "
                    f"sequence {seq}"
                )
            # ArduPilot can emit legacy MISSION_REQUEST messages between
            # MISSION_REQUEST_INT messages even on a MAVLink2 link.  Replying
            # with MISSION_ITEM would quantize latitude/longitude to float32.
            # Its receive state machine accepts MISSION_ITEM_INT for either
            # request type, so always preserve integer coordinate precision.
            self._send_item_int(records[seq])
            sent.add(seq)

    def _record_from_message(self, message: Any) -> MissionItemRecord:
        message_type = message.get_type()
        if message_type == "MISSION_ITEM_INT":
            x = int(message.x)
            y = int(message.y)
        elif message_type == "MISSION_ITEM":
            x = int(round(float(message.x) * 10_000_000.0))
            y = int(round(float(message.y) * 10_000_000.0))
        else:
            raise MissionUploadError(f"unexpected mission item type {message_type}")
        return MissionItemRecord(
            seq=int(message.seq),
            frame=int(message.frame),
            command=int(message.command),
            autocontinue=int(message.autocontinue),
            param1=float(message.param1),
            param2=float(message.param2),
            param3=float(message.param3),
            param4=float(message.param4),
            x=x,
            y=y,
            z=float(message.z),
        )

    def download_mission(self) -> tuple[MissionItemRecord, ...]:
        self._drain()
        count_message = None
        for _attempt in range(self._fleet.timeouts.retries):
            self._send_request_list()
            count_message = self._recv(
                ("MISSION_COUNT",),
                self._fleet.timeouts.download_item_s,
            )
            if count_message is not None:
                break
        if count_message is None:
            raise MissionUploadError(
                f"{self._vehicle.vehicle_id}: mission readback count timeout"
            )
        count = int(count_message.count)
        if count <= 0:
            raise MissionUploadError(
                f"{self._vehicle.vehicle_id}: autopilot readback contains no mission items"
            )

        records: list[MissionItemRecord] = []
        accepted = int(self._mavutil.mavlink.MAV_MISSION_ACCEPTED)
        try:
            for seq in range(count):
                item_message = None
                for _attempt in range(self._fleet.timeouts.retries):
                    self._send_request_int(seq)
                    deadline = time.monotonic() + self._fleet.timeouts.download_item_s
                    while True:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            break
                        candidate = self._recv(
                            ("MISSION_ITEM_INT", "MISSION_ITEM"),
                            remaining,
                        )
                        if candidate is None:
                            break
                        if int(candidate.seq) != seq:
                            continue
                        item_message = candidate
                        break
                    if item_message is not None:
                        break
                if item_message is None:
                    raise MissionUploadError(
                        f"{self._vehicle.vehicle_id}: mission readback timeout at item {seq}"
                    )
                records.append(self._record_from_message(item_message))
            self._send_ack(accepted)
        except Exception:
            try:
                self._send_ack(
                    int(self._mavutil.mavlink.MAV_MISSION_ERROR)
                )
            except Exception:
                pass
            raise
        return tuple(records)

    def close(self) -> None:
        connection = self._connection
        self._connection = None
        self._identity = None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass


def pymavlink_client_factory(
    vehicle: FleetVehicleConfig,
    config: FleetUploadConfig,
) -> PymavlinkMissionClient:
    return PymavlinkMissionClient(vehicle, config)
