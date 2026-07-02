#!/usr/bin/env python3
"""Transport-normalized mission records, comparison and SHA-256 fingerprints."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import struct
from typing import Any, Iterable

from .ardupilot_mission import ArduPilotMission
from .ardupilot_mission import MAV_CMD_NAV_LAND
from coverage_mission_pipeline.ardupilot_mission import MAV_CMD_NAV_RETURN_TO_LAUNCH

# MAV_FRAME equivalents used by MISSION_ITEM and MISSION_ITEM_INT.
_GLOBAL_FRAME_NORMALIZATION = {
    0: 0,   # MAV_FRAME_GLOBAL
    3: 3,   # MAV_FRAME_GLOBAL_RELATIVE_ALT
    5: 0,   # MAV_FRAME_GLOBAL_INT
    6: 3,   # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    10: 10, # MAV_FRAME_GLOBAL_TERRAIN_ALT
    11: 10, # MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
}

# ArduPilot reserves mission sequence zero for HOME.  The generated mission
# JSON is semantic and deliberately starts with TAKEOFF at sequence zero, so
# direct MAVLink transfer requires an explicit wire-only HOME slot.
MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_CMD_NAV_WAYPOINT = 16


class MissionFingerprintError(ValueError):
    """Raised when a mission cannot be represented safely on MAVLink."""


def float32(value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise MissionFingerprintError("mission values must be finite")
    return struct.unpack("<f", struct.pack("<f", result))[0]


def float32_bits(value: float) -> str:
    return struct.pack("<f", float32(value)).hex()


def degrees_e7(value: float) -> int:
    result = float(value)
    if not math.isfinite(result):
        raise MissionFingerprintError("mission coordinates must be finite")
    scaled = int(round(result * 10_000_000.0))
    if scaled < -2_147_483_648 or scaled > 2_147_483_647:
        raise MissionFingerprintError("mission coordinate exceeds int32 range")
    return scaled


def normalize_global_frame(frame: int) -> int:
    if isinstance(frame, bool) or not isinstance(frame, int):
        raise MissionFingerprintError("frame must be an integer")
    return _GLOBAL_FRAME_NORMALIZATION.get(frame, frame)



def canonical_mission_frame(command: int, frame: int) -> int:
    """Return the frame used for semantic mission comparison.

    ArduPilot may store MAV_CMD_NAV_RETURN_TO_LAUNCH using
    MAV_FRAME_GLOBAL even when the generated semantic mission uses
    MAV_FRAME_GLOBAL_RELATIVE_ALT. RTL carries no positional coordinate
    whose interpretation depends on that frame distinction.

    This affects comparison and fingerprinting only. It does not alter
    the stored record or uploaded wire representation.
    """

    normalized = normalize_global_frame(frame)
    if command == MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return MAV_FRAME_GLOBAL
    return normalized


def canonical_mission_param4(
    command: int,
    frame: int,
    param4: float,
) -> float:
    """Return command-aware param4 for semantic comparison.

    ArduPilot Copter stores location mission items in a compact
    internal representation. For MAV_CMD_NAV_LAND in
    MAV_FRAME_GLOBAL_RELATIVE_ALT, the tested ArduPilot build
    reads an uploaded default param4 of 0.0 back as 1.0.

    Only the exact default pair 0.0/1.0 is canonicalized.
    Non-default LAND yaw values and every other command's
    param4 remain fingerprint-significant.
    """

    value = float32(param4)
    normalized_frame = normalize_global_frame(frame)

    if (
        command == MAV_CMD_NAV_LAND
        and normalized_frame
        == MAV_FRAME_GLOBAL_RELATIVE_ALT
        and value in {0.0, 1.0}
    ):
        return float32(1.0)

    return value


def wire_int_frame(frame: int) -> int:
    normalized = normalize_global_frame(frame)
    return {0: 5, 3: 6, 10: 11}.get(normalized, normalized)


@dataclass(frozen=True)
class MissionItemRecord:
    """One mission item represented at MAVLink transport precision.

    ``current`` is intentionally excluded. ArduPilot treats that field as upload
    protocol state rather than durable mission content and may not echo it during
    readback. Every safety-relevant command, frame, parameter and coordinate is
    included.
    """

    seq: int
    frame: int
    command: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float

    def __post_init__(self) -> None:
        for name in ("seq", "frame", "command", "autocontinue", "x", "y"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise MissionFingerprintError(f"{name} must be an integer")
        if self.seq < 0:
            raise MissionFingerprintError("seq must be non-negative")
        if self.autocontinue not in {0, 1}:
            raise MissionFingerprintError("autocontinue must be 0 or 1")
        object.__setattr__(self, "frame", normalize_global_frame(self.frame))
        for name in ("param1", "param2", "param3", "param4", "z"):
            object.__setattr__(self, name, float32(getattr(self, name)))
        for name in ("x", "y"):
            value = getattr(self, name)
            if value < -2_147_483_648 or value > 2_147_483_647:
                raise MissionFingerprintError(f"{name} exceeds int32 range")

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "seq": self.seq,
            "frame": canonical_mission_frame(self.command, self.frame),
            "command": self.command,
            "autocontinue": self.autocontinue,
            "param1_f32": float32_bits(self.param1),
            "param2_f32": float32_bits(self.param2),
            "param3_f32": float32_bits(self.param3),
            "param4_f32": float32_bits(
                canonical_mission_param4(
                    self.command,
                    self.frame,
                    self.param4,
                )
            ),
            "x_e7": self.x,
            "y_e7": self.y,
            "z_f32": float32_bits(self.z),
        }


def records_from_ardupilot_mission(
    mission: ArduPilotMission,
) -> tuple[MissionItemRecord, ...]:
    if not isinstance(mission, ArduPilotMission):
        raise MissionFingerprintError("mission must be an ArduPilotMission")
    records = tuple(
        MissionItemRecord(
            seq=item.seq,
            frame=item.frame,
            command=item.command,
            autocontinue=item.autocontinue,
            param1=item.param1,
            param2=item.param2,
            param3=item.param3,
            param4=item.param4,
            x=degrees_e7(item.latitude_deg),
            y=degrees_e7(item.longitude_deg),
            z=item.altitude_m,
        )
        for item in mission.items
    )
    validate_record_sequence(records)
    return records


def validate_record_sequence(records: Iterable[MissionItemRecord]) -> tuple[MissionItemRecord, ...]:
    result = tuple(records)
    if not result:
        raise MissionFingerprintError("mission records must not be empty")
    if any(not isinstance(item, MissionItemRecord) for item in result):
        raise MissionFingerprintError("records contains an invalid item")
    for expected_seq, item in enumerate(result):
        if item.seq != expected_seq:
            raise MissionFingerprintError(
                "mission record sequence numbers must be consecutive from zero"
            )
    return result


def wire_records_from_semantic(
    semantic_records: Iterable[MissionItemRecord],
) -> tuple[MissionItemRecord, ...]:
    """Create ArduPilot's on-wire mission: HOME at zero, semantic items after it.

    ArduPilot owns sequence zero and may replace its coordinates with the
    vehicle's current HOME during storage/readback.  The placeholder therefore
    exists only to preserve the semantic TAKEOFF as the first executable item.
    """

    semantic = validate_record_sequence(semantic_records)
    first = semantic[0]
    home = MissionItemRecord(
        seq=0,
        frame=MAV_FRAME_GLOBAL,
        command=MAV_CMD_NAV_WAYPOINT,
        autocontinue=1,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        x=first.x,
        y=first.y,
        z=0.0,
    )
    shifted = tuple(
        MissionItemRecord(
            seq=item.seq + 1,
            frame=item.frame,
            command=item.command,
            autocontinue=item.autocontinue,
            param1=item.param1,
            param2=item.param2,
            param3=item.param3,
            param4=item.param4,
            x=item.x,
            y=item.y,
            z=item.z,
        )
        for item in semantic
    )
    return validate_record_sequence((home, *shifted))


def semantic_records_from_wire_readback(
    wire_records: Iterable[MissionItemRecord],
) -> tuple[MissionItemRecord, ...]:
    """Validate and remove ArduPilot's dynamic HOME item from readback."""

    wire = validate_record_sequence(wire_records)
    if len(wire) < 2:
        raise MissionFingerprintError(
            "ArduPilot readback must contain HOME plus at least one mission item"
        )
    home = wire[0]
    if home.frame != MAV_FRAME_GLOBAL:
        raise MissionFingerprintError(
            f"ArduPilot HOME item must use global frame, got {home.frame}"
        )
    if home.command != MAV_CMD_NAV_WAYPOINT:
        raise MissionFingerprintError(
            f"ArduPilot HOME item must be MAV_CMD_NAV_WAYPOINT, got {home.command}"
        )

    semantic = tuple(
        MissionItemRecord(
            seq=item.seq - 1,
            frame=item.frame,
            command=item.command,
            autocontinue=item.autocontinue,
            param1=item.param1,
            param2=item.param2,
            param3=item.param3,
            param4=item.param4,
            x=item.x,
            y=item.y,
            z=item.z,
        )
        for item in wire[1:]
    )
    return validate_record_sequence(semantic)


def mission_fingerprint(records: Iterable[MissionItemRecord]) -> str:
    normalized = validate_record_sequence(records)
    payload = json.dumps(
        [item.canonical_dict() for item in normalized],
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def compare_mission_records(
    expected: Iterable[MissionItemRecord],
    actual: Iterable[MissionItemRecord],
) -> tuple[str, ...]:
    left = validate_record_sequence(expected)
    right = validate_record_sequence(actual)
    differences: list[str] = []
    if len(left) != len(right):
        differences.append(
            f"item count differs: expected {len(left)}, read back {len(right)}"
        )
    for index in range(min(len(left), len(right))):
        expected_dict = left[index].canonical_dict()
        actual_dict = right[index].canonical_dict()
        for field in expected_dict:
            if expected_dict[field] != actual_dict[field]:
                differences.append(
                    f"item {index} field {field} differs: "
                    f"expected {expected_dict[field]!r}, "
                    f"read back {actual_dict[field]!r}"
                )
    return tuple(differences)
