#!/usr/bin/env python3
"""Fail-closed, transactional fleet mission upload orchestration."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Callable, Mapping, Protocol

from .ardupilot_mission import (
    ArduPilotMission,
    ArduPilotMissionError,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,
)
from .fleet_config import FleetUploadConfig, FleetVehicleConfig
from .mission_fingerprint import (
    MissionItemRecord,
    compare_mission_records,
    mission_fingerprint,
    records_from_ardupilot_mission,
    semantic_records_from_wire_readback,
    wire_records_from_semantic,
)

FLEET_UPLOAD_REPORT_SCHEMA_VERSION = 2


class MissionUploadError(RuntimeError):
    """Raised when upload or readback verification cannot be proven."""


@dataclass(frozen=True)
class VehicleIdentity:
    system_id: int
    component_id: int
    autopilot: int | None = None
    vehicle_type: int | None = None


class MissionVehicleClient(Protocol):
    """Explicit one-vehicle connection used by the fleet coordinator."""

    def connect(self) -> VehicleIdentity:
        ...

    def is_armed(self) -> bool:
        ...

    def clear_mission(self) -> None:
        ...

    def upload_mission(self, records: tuple[MissionItemRecord, ...]) -> None:
        ...

    def download_mission(self) -> tuple[MissionItemRecord, ...]:
        ...

    def close(self) -> None:
        ...


ClientFactory = Callable[[FleetVehicleConfig, FleetUploadConfig], MissionVehicleClient]


@dataclass(frozen=True)
class LoadedVehicleMission:
    vehicle: FleetVehicleConfig
    path: Path
    sha256: str
    mission: ArduPilotMission
    records: tuple[MissionItemRecord, ...]
    wire_records: tuple[MissionItemRecord, ...]
    fingerprint: str


@dataclass(frozen=True)
class VehicleUploadResult:
    vehicle_id: str
    endpoint: str
    expected_system_id: int
    observed_system_id: int
    observed_component_id: int
    mission_path: str
    mission_sha256: str
    expected_item_count: int
    readback_item_count: int
    expected_wire_item_count: int
    readback_wire_item_count: int
    expected_fingerprint: str
    readback_fingerprint: str
    verified: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "vehicle_id": self.vehicle_id,
            "endpoint": self.endpoint,
            "expected_system_id": self.expected_system_id,
            "observed_system_id": self.observed_system_id,
            "observed_component_id": self.observed_component_id,
            "mission_path": self.mission_path,
            "mission_sha256": self.mission_sha256,
            "expected_item_count": self.expected_item_count,
            "readback_item_count": self.readback_item_count,
            "expected_wire_item_count": self.expected_wire_item_count,
            "readback_wire_item_count": self.readback_wire_item_count,
            "expected_fingerprint": self.expected_fingerprint,
            "readback_fingerprint": self.readback_fingerprint,
            "verified": self.verified,
        }


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise MissionUploadError(f"could not hash {path}: {exc}") from exc
    return digest.hexdigest()


def _mission_path(root: Path, vehicle_id: str) -> Path:
    filename = f"{vehicle_id}.ardupilot-mission.json"
    candidates = (root / "ardupilot" / filename, root / filename)
    existing = [candidate for candidate in candidates if candidate.is_file()]
    if not existing:
        raise MissionUploadError(
            f"mission JSON for {vehicle_id!r} not found; expected "
            f"{candidates[0]} or {candidates[1]}"
        )
    if len(existing) > 1:
        raise MissionUploadError(
            f"mission JSON for {vehicle_id!r} is ambiguous: "
            + ", ".join(str(path) for path in existing)
        )
    return existing[0]


def _same_e7_position(
    left: Any,
    right: Any,
) -> bool:
    """Compare mission coordinates at MAVLink MISSION_ITEM_INT precision."""

    return (
        int(round(float(left.latitude_deg) * 10_000_000.0))
        == int(round(float(right.latitude_deg) * 10_000_000.0))
        and int(round(float(left.longitude_deg) * 10_000_000.0))
        == int(round(float(right.longitude_deg) * 10_000_000.0))
    )


def _validate_terminal_semantics(
    mission: ArduPilotMission,
    vehicle_id: str,
) -> None:
    """Require a safe generated mission before opening any vehicle connection."""

    items = mission.items

    if items[0].command != MAV_CMD_NAV_TAKEOFF:
        raise MissionUploadError(
            f"{vehicle_id}: first semantic mission item must be TAKEOFF"
        )

    if len(items) < 3:
        raise MissionUploadError(
            f"{vehicle_id}: semantic mission is too short to prove "
            "a safe terminal route"
        )

    terminal = items[-1]
    preterminal = items[-2]
    reference = items[0]

    if terminal.command not in {
        MAV_CMD_NAV_RETURN_TO_LAUNCH,
        MAV_CMD_NAV_LAND,
    }:
        raise MissionUploadError(
            f"{vehicle_id}: final semantic mission item must be "
            "RTL or LAND at reference"
        )

    if preterminal.command != MAV_CMD_NAV_WAYPOINT:
        raise MissionUploadError(
            f"{vehicle_id}: item before the terminal action must be "
            "a waypoint at the reference"
        )

    if not _same_e7_position(preterminal, reference):
        raise MissionUploadError(
            f"{vehicle_id}: terminal waypoint must match the "
            "TAKEOFF reference"
        )

    if terminal.command == MAV_CMD_NAV_LAND:
        if not _same_e7_position(terminal, reference):
            raise MissionUploadError(
                f"{vehicle_id}: final LAND coordinates must match "
                "the TAKEOFF reference"
            )

        if terminal.altitude_m != 0.0:
            raise MissionUploadError(
                f"{vehicle_id}: final LAND altitude must be zero"
            )


def load_vehicle_missions(
    missions_directory: Path | str,
    config: FleetUploadConfig,
) -> tuple[LoadedVehicleMission, ...]:
    root = Path(missions_directory)
    if not root.exists() or not root.is_dir():
        raise MissionUploadError(f"missions directory does not exist: {root}")

    loaded: list[LoadedVehicleMission] = []
    for vehicle in config.vehicles:
        path = _mission_path(root, vehicle.vehicle_id)
        try:
            mission = ArduPilotMission.read_json(path)
        except ArduPilotMissionError as exc:
            raise MissionUploadError(
                f"invalid mission JSON for {vehicle.vehicle_id}: {exc}"
            ) from exc
        if mission.vehicle_id != vehicle.vehicle_id:
            raise MissionUploadError(
                f"mission identity mismatch for {vehicle.vehicle_id}: "
                f"file declares {mission.vehicle_id!r}"
            )
        _validate_terminal_semantics(
            mission,
            vehicle.vehicle_id,
        )
        records = records_from_ardupilot_mission(mission)
        wire_records = wire_records_from_semantic(records)
        loaded.append(
            LoadedVehicleMission(
                vehicle=vehicle,
                path=path,
                sha256=_sha256_file(path),
                mission=mission,
                records=records,
                wire_records=wire_records,
                fingerprint=mission_fingerprint(records),
            )
        )

    if not config.allow_duplicate_missions:
        by_fingerprint: dict[str, list[str]] = {}
        for item in loaded:
            by_fingerprint.setdefault(item.fingerprint, []).append(
                item.vehicle.vehicle_id
            )
        duplicates = {
            fingerprint: vehicle_ids
            for fingerprint, vehicle_ids in by_fingerprint.items()
            if len(vehicle_ids) > 1
        }
        if duplicates:
            details = "; ".join(
                f"{fingerprint[:12]}... assigned to {', '.join(vehicle_ids)}"
                for fingerprint, vehicle_ids in sorted(duplicates.items())
            )
            raise MissionUploadError(
                "duplicate mission assignments are forbidden: " + details
            )

    return tuple(loaded)


def upload_and_verify_vehicle(
    loaded: LoadedVehicleMission,
    config: FleetUploadConfig,
    client_factory: ClientFactory,
) -> VehicleUploadResult:
    client = client_factory(loaded.vehicle, config)
    identity: VehicleIdentity | None = None
    try:
        identity = client.connect()
        if identity.system_id != loaded.vehicle.system_id:
            raise MissionUploadError(
                f"{loaded.vehicle.vehicle_id}: endpoint {loaded.vehicle.endpoint!r} "
                f"reported system ID {identity.system_id}, expected "
                f"{loaded.vehicle.system_id}"
            )
        if identity.component_id != loaded.vehicle.component_id:
            raise MissionUploadError(
                f"{loaded.vehicle.vehicle_id}: endpoint {loaded.vehicle.endpoint!r} "
                f"reported component ID {identity.component_id}, expected "
                f"{loaded.vehicle.component_id}"
            )
        if client.is_armed():
            raise MissionUploadError(
                f"{loaded.vehicle.vehicle_id}: vehicle is armed; mission upload refused"
            )

        client.clear_mission()
        client.upload_mission(loaded.wire_records)
        wire_readback = client.download_mission()
        try:
            readback = semantic_records_from_wire_readback(wire_readback)
        except Exception as exc:
            raise MissionUploadError(
                f"{loaded.vehicle.vehicle_id}: invalid ArduPilot wire readback: {exc}"
            ) from exc
        differences = compare_mission_records(loaded.records, readback)
        readback_fingerprint = mission_fingerprint(readback)
        if differences or readback_fingerprint != loaded.fingerprint:
            preview = "; ".join(differences[:8])
            if len(differences) > 8:
                preview += f"; and {len(differences) - 8} more difference(s)"
            raise MissionUploadError(
                f"{loaded.vehicle.vehicle_id}: mission readback verification failed; "
                f"expected fingerprint {loaded.fingerprint}, read back "
                f"{readback_fingerprint}"
                + (f"; {preview}" if preview else "")
            )

        return VehicleUploadResult(
            vehicle_id=loaded.vehicle.vehicle_id,
            endpoint=loaded.vehicle.endpoint,
            expected_system_id=loaded.vehicle.system_id,
            observed_system_id=identity.system_id,
            observed_component_id=identity.component_id,
            mission_path=str(loaded.path),
            mission_sha256=loaded.sha256,
            expected_item_count=len(loaded.records),
            readback_item_count=len(readback),
            expected_wire_item_count=len(loaded.wire_records),
            readback_wire_item_count=len(wire_readback),
            expected_fingerprint=loaded.fingerprint,
            readback_fingerprint=readback_fingerprint,
            verified=True,
        )
    finally:
        try:
            client.close()
        except Exception:
            pass


def _atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    if path.exists():
        raise MissionUploadError(f"report path already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(
        payload,
        indent=2,
        sort_keys=True,
        allow_nan=False,
    ) + "\n"
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=str(path.parent),
        text=True,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def upload_and_verify_fleet(
    missions_directory: Path | str,
    config: FleetUploadConfig,
    report_path: Path | str,
    client_factory: ClientFactory,
) -> dict[str, Any]:
    """Upload sequentially and write a report only after every readback passes.

    Loading and duplicate checks happen before any network connection is opened.
    Each vehicle then receives a dedicated client instance and is fully verified
    before the next vehicle is touched.
    """

    loaded = load_vehicle_missions(missions_directory, config)
    results: list[VehicleUploadResult] = []
    for item in loaded:
        results.append(upload_and_verify_vehicle(item, config, client_factory))

    report = {
        "schema_version": FLEET_UPLOAD_REPORT_SCHEMA_VERSION,
        "status": "VERIFIED",
        "profile": config.profile,
        "vehicle_count": len(results),
        "all_verified": all(result.verified for result in results),
        "vehicles": [result.to_dict() for result in results],
    }
    _atomic_json(Path(report_path), report)
    return report
