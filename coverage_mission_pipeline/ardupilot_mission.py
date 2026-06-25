#!/usr/bin/env python3
"""Strict ArduPilot Copter mission model and QGC WPL 110 exporter.

The builder consumes one CompleteVehicleRouteRecord and converts its authoritative
local cruise path to WGS84 mission items.  Every positional item uses
MAV_FRAME_GLOBAL_RELATIVE_ALT, so route waypoint ``z_m`` values are interpreted
explicitly as metres above the ArduPilot home altitude.

This module does not upload missions, arm a vehicle, change flight modes, infer a
home location, or parse mission-specific input JSON.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
from typing import Any, Iterable, Mapping, Optional

from .complete_vehicle_route_record import (
    CompleteVehicleRouteRecord,
    CompleteVehicleRouteRecordError,
)

ARDUPILOT_MISSION_SCHEMA_VERSION = 1
QGC_WPL_110_HEADER = "QGC WPL 110"

MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22

END_ACTION_RTL = "rtl"
END_ACTION_LAND_AT_REFERENCE = "land_at_reference"
END_ACTION_NONE = "none"

_SUPPORTED_COMMANDS = frozenset(
    {
        MAV_CMD_NAV_WAYPOINT,
        MAV_CMD_NAV_RETURN_TO_LAUNCH,
        MAV_CMD_NAV_LAND,
        MAV_CMD_NAV_TAKEOFF,
    }
)
_LOCATION_COMMANDS = frozenset(
    {MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_LAND, MAV_CMD_NAV_TAKEOFF}
)
_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_ALTITUDE_TOLERANCE_M = 1.0e-6
_POSITION_TOLERANCE_DEG = 1.0e-9
_ZERO_TOLERANCE = 1.0e-12


class ArduPilotMissionError(ValueError):
    """Raised when an ArduPilot mission model or export is unsafe or malformed."""


def _identifier(value: Any, path: str) -> str:
    if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
        raise ArduPilotMissionError(f"{path} must match {_ID_PATTERN.pattern!r}")
    return value


def _strict_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ArduPilotMissionError(f"{path} must be an object")
    return value


def _strict_keys(value: Mapping[str, Any], required: set[str], path: str) -> None:
    actual = set(value.keys())
    missing = sorted(required - actual)
    unknown = sorted(actual - required)
    if missing:
        raise ArduPilotMissionError(
            f"{path} is missing required field(s): {', '.join(missing)}"
        )
    if unknown:
        raise ArduPilotMissionError(
            f"{path} contains unknown field(s): {', '.join(unknown)}"
        )


def _finite_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ArduPilotMissionError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ArduPilotMissionError(f"{path} must be finite")
    return result


def _finite_nonnegative(value: Any, path: str) -> float:
    result = _finite_number(value, path)
    if result < 0.0:
        raise ArduPilotMissionError(f"{path} must be non-negative")
    return result


def _binary_integer(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value not in {0, 1}:
        raise ArduPilotMissionError(f"{path} must be the integer 0 or 1")
    return value


def _nonnegative_integer(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ArduPilotMissionError(f"{path} must be a non-negative integer")
    return value


def _is_zero(value: float) -> bool:
    return math.isclose(value, 0.0, rel_tol=0.0, abs_tol=_ZERO_TOLERANCE)


def _format_float(value: float) -> str:
    """Return deterministic decimal text accepted by QGC WPL 110 readers."""
    if not math.isfinite(value):
        raise ArduPilotMissionError("cannot export a non-finite mission value")
    if _is_zero(value):
        value = 0.0
    return f"{value:.10f}"


@dataclass(frozen=True)
class ArduPilotMissionItem:
    """One validated QGC WPL 110 / MAVLink mission item."""

    seq: int
    current: int
    frame: int
    command: int
    param1: float
    param2: float
    param3: float
    param4: float
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    autocontinue: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "seq", _nonnegative_integer(self.seq, "item.seq"))
        object.__setattr__(self, "current", _binary_integer(self.current, "item.current"))
        object.__setattr__(
            self,
            "autocontinue",
            _binary_integer(self.autocontinue, "item.autocontinue"),
        )
        if isinstance(self.frame, bool) or not isinstance(self.frame, int):
            raise ArduPilotMissionError("item.frame must be integer MAV_FRAME_GLOBAL_RELATIVE_ALT (3)")
        if self.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT:
            raise ArduPilotMissionError(
                "item.frame must be MAV_FRAME_GLOBAL_RELATIVE_ALT (3)"
            )
        if isinstance(self.command, bool) or not isinstance(self.command, int):
            raise ArduPilotMissionError("item.command must be an integer")
        if self.command not in _SUPPORTED_COMMANDS:
            raise ArduPilotMissionError("item.command is not supported by this exporter")

        for name in (
            "param1",
            "param2",
            "param3",
            "param4",
            "latitude_deg",
            "longitude_deg",
            "altitude_m",
        ):
            object.__setattr__(self, name, _finite_number(getattr(self, name), f"item.{name}"))

        if self.command in _LOCATION_COMMANDS:
            if not -90.0 <= self.latitude_deg <= 90.0:
                raise ArduPilotMissionError(
                    "item.latitude_deg must be in the range [-90, 90]"
                )
            if not -180.0 <= self.longitude_deg <= 180.0:
                raise ArduPilotMissionError(
                    "item.longitude_deg must be in the range [-180, 180]"
                )
        else:
            values = (
                self.param1,
                self.param2,
                self.param3,
                self.param4,
                self.latitude_deg,
                self.longitude_deg,
                self.altitude_m,
            )
            if any(not _is_zero(value) for value in values):
                raise ArduPilotMissionError(
                    "MAV_CMD_NAV_RETURN_TO_LAUNCH must use zero parameters and coordinates"
                )

        if self.command == MAV_CMD_NAV_WAYPOINT:
            if self.param1 < 0.0:
                raise ArduPilotMissionError("waypoint hold time must be non-negative")
            if any(
                not _is_zero(value)
                for value in (self.param2, self.param3, self.param4)
            ):
                raise ArduPilotMissionError(
                    "this exporter requires waypoint param2, param3 and param4 to be zero"
                )
            if self.altitude_m <= 0.0:
                raise ArduPilotMissionError(
                    "waypoint relative altitude must be greater than zero"
                )
        elif self.command == MAV_CMD_NAV_TAKEOFF:
            if any(
                not _is_zero(value)
                for value in (self.param1, self.param2, self.param3, self.param4)
            ):
                raise ArduPilotMissionError(
                    "this exporter requires takeoff parameters to be zero"
                )
            if self.altitude_m <= 0.0:
                raise ArduPilotMissionError(
                    "takeoff relative altitude must be greater than zero"
                )
        elif self.command == MAV_CMD_NAV_LAND:
            if any(
                not _is_zero(value)
                for value in (self.param1, self.param2, self.param3, self.param4)
            ):
                raise ArduPilotMissionError(
                    "this exporter requires land parameters to be zero"
                )
            if not _is_zero(self.altitude_m):
                raise ArduPilotMissionError(
                    "land-at-reference altitude must be zero in the relative-altitude frame"
                )

    @property
    def command_name(self) -> str:
        return {
            MAV_CMD_NAV_WAYPOINT: "MAV_CMD_NAV_WAYPOINT",
            MAV_CMD_NAV_RETURN_TO_LAUNCH: "MAV_CMD_NAV_RETURN_TO_LAUNCH",
            MAV_CMD_NAV_LAND: "MAV_CMD_NAV_LAND",
            MAV_CMD_NAV_TAKEOFF: "MAV_CMD_NAV_TAKEOFF",
        }[self.command]

    def to_dict(self) -> dict[str, Any]:
        return {
            "seq": self.seq,
            "current": self.current,
            "frame": self.frame,
            "command": self.command,
            "param1": self.param1,
            "param2": self.param2,
            "param3": self.param3,
            "param4": self.param4,
            "latitude_deg": self.latitude_deg,
            "longitude_deg": self.longitude_deg,
            "altitude_m": self.altitude_m,
            "autocontinue": self.autocontinue,
        }

    @classmethod
    def from_dict(cls, value: Any, path: str = "item") -> "ArduPilotMissionItem":
        mapping = _strict_mapping(value, path)
        _strict_keys(
            mapping,
            {
                "seq",
                "current",
                "frame",
                "command",
                "param1",
                "param2",
                "param3",
                "param4",
                "latitude_deg",
                "longitude_deg",
                "altitude_m",
                "autocontinue",
            },
            path,
        )
        try:
            return cls(**mapping)
        except ArduPilotMissionError as exc:
            raise ArduPilotMissionError(f"{path} is invalid: {exc}") from exc

    def to_wpl_line(self) -> str:
        fields = (
            str(self.seq),
            str(self.current),
            str(self.frame),
            str(self.command),
            _format_float(self.param1),
            _format_float(self.param2),
            _format_float(self.param3),
            _format_float(self.param4),
            _format_float(self.latitude_deg),
            _format_float(self.longitude_deg),
            _format_float(self.altitude_m),
            str(self.autocontinue),
        )
        return "\t".join(fields)


@dataclass(frozen=True)
class ArduPilotMissionBuildConfig:
    """Explicit conversion policy from a complete cruise route to Copter items."""

    end_action: str = END_ACTION_RTL
    waypoint_hold_s: float = 0.0
    include_takeoff: bool = True
    skip_initial_reference_waypoint: bool = True
    minimum_relative_altitude_m: float = 1.0

    def __post_init__(self) -> None:
        if self.end_action not in {
            END_ACTION_RTL,
            END_ACTION_LAND_AT_REFERENCE,
            END_ACTION_NONE,
        }:
            raise ArduPilotMissionError(
                "end_action must be 'rtl', 'land_at_reference' or 'none'"
            )
        object.__setattr__(
            self,
            "waypoint_hold_s",
            _finite_nonnegative(self.waypoint_hold_s, "waypoint_hold_s"),
        )
        if not isinstance(self.include_takeoff, bool):
            raise ArduPilotMissionError("include_takeoff must be a bool")
        if not isinstance(self.skip_initial_reference_waypoint, bool):
            raise ArduPilotMissionError(
                "skip_initial_reference_waypoint must be a bool"
            )
        minimum = _finite_number(
            self.minimum_relative_altitude_m,
            "minimum_relative_altitude_m",
        )
        if minimum <= 0.0:
            raise ArduPilotMissionError(
                "minimum_relative_altitude_m must be greater than zero"
            )
        object.__setattr__(self, "minimum_relative_altitude_m", minimum)
        if self.skip_initial_reference_waypoint and not self.include_takeoff:
            raise ArduPilotMissionError(
                "skip_initial_reference_waypoint requires include_takeoff=true"
            )


@dataclass(frozen=True)
class ArduPilotMission:
    """Strict deterministic mission model for one vehicle."""

    vehicle_id: str
    items: tuple[ArduPilotMissionItem, ...]
    altitude_reference: str = "home_relative"

    def __post_init__(self) -> None:
        object.__setattr__(self, "vehicle_id", _identifier(self.vehicle_id, "vehicle_id"))
        if self.altitude_reference != "home_relative":
            raise ArduPilotMissionError(
                "altitude_reference must be 'home_relative'"
            )
        try:
            items = tuple(self.items)
        except TypeError as exc:
            raise ArduPilotMissionError("items must be iterable") from exc
        if not items:
            raise ArduPilotMissionError("items must not be empty")
        if any(not isinstance(item, ArduPilotMissionItem) for item in items):
            raise ArduPilotMissionError(
                "items must contain only ArduPilotMissionItem objects"
            )
        object.__setattr__(self, "items", items)

        for expected_seq, item in enumerate(items):
            if item.seq != expected_seq:
                raise ArduPilotMissionError(
                    "mission item sequence numbers must be consecutive from zero"
                )
            expected_current = 1 if expected_seq == 0 else 0
            if item.current != expected_current:
                raise ArduPilotMissionError(
                    "only the first mission item may have current=1"
                )
            if item.autocontinue != 1:
                raise ArduPilotMissionError(
                    "every mission item must use autocontinue=1"
                )

        takeoff_indexes = [
            index for index, item in enumerate(items) if item.command == MAV_CMD_NAV_TAKEOFF
        ]
        if takeoff_indexes and takeoff_indexes != [0]:
            raise ArduPilotMissionError(
                "MAV_CMD_NAV_TAKEOFF may appear only as the first mission item"
            )
        terminal_indexes = [
            index
            for index, item in enumerate(items)
            if item.command in {MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_CMD_NAV_LAND}
        ]
        if terminal_indexes and terminal_indexes != [len(items) - 1]:
            raise ArduPilotMissionError(
                "RTL or LAND may appear only once as the final mission item"
            )
        waypoint_count = sum(
            item.command == MAV_CMD_NAV_WAYPOINT for item in items
        )
        if waypoint_count < 1:
            raise ArduPilotMissionError(
                "a generated mission requires at least one navigation waypoint"
            )

    @property
    def end_action(self) -> str:
        command = self.items[-1].command
        if command == MAV_CMD_NAV_RETURN_TO_LAUNCH:
            return END_ACTION_RTL
        if command == MAV_CMD_NAV_LAND:
            return END_ACTION_LAND_AT_REFERENCE
        return END_ACTION_NONE

    @property
    def waypoint_items(self) -> tuple[ArduPilotMissionItem, ...]:
        return tuple(
            item for item in self.items if item.command == MAV_CMD_NAV_WAYPOINT
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": ARDUPILOT_MISSION_SCHEMA_VERSION,
            "vehicle_id": self.vehicle_id,
            "altitude_reference": self.altitude_reference,
            "items": [item.to_dict() for item in self.items],
        }

    @classmethod
    def from_dict(cls, value: Any) -> "ArduPilotMission":
        mapping = _strict_mapping(value, "root")
        _strict_keys(
            mapping,
            {"schema_version", "vehicle_id", "altitude_reference", "items"},
            "root",
        )
        if mapping["schema_version"] != ARDUPILOT_MISSION_SCHEMA_VERSION:
            raise ArduPilotMissionError(
                f"unsupported schema_version: {mapping['schema_version']!r}"
            )
        raw_items = mapping["items"]
        if not isinstance(raw_items, list):
            raise ArduPilotMissionError("items must be an array")
        return cls(
            vehicle_id=mapping["vehicle_id"],
            altitude_reference=mapping["altitude_reference"],
            items=tuple(
                ArduPilotMissionItem.from_dict(item, f"items[{index}]")
                for index, item in enumerate(raw_items)
            ),
        )

    def to_json(self) -> str:
        return json.dumps(
            self.to_dict(),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        ) + "\n"

    @classmethod
    def from_json(cls, text: str) -> "ArduPilotMission":
        if not isinstance(text, str):
            raise ArduPilotMissionError("JSON input must be text")
        try:
            value = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ArduPilotMissionError(
                f"invalid JSON at line {exc.lineno}, column {exc.colno}: {exc.msg}"
            ) from exc
        return cls.from_dict(value)

    def to_qgc_wpl_110(self) -> str:
        return QGC_WPL_110_HEADER + "\n" + "\n".join(
            item.to_wpl_line() for item in self.items
        ) + "\n"

    @classmethod
    def from_qgc_wpl_110(
        cls,
        text: str,
        *,
        vehicle_id: str,
    ) -> "ArduPilotMission":
        if not isinstance(text, str):
            raise ArduPilotMissionError("QGC WPL input must be text")
        lines = text.splitlines()
        if not lines or lines[0].strip() != QGC_WPL_110_HEADER:
            raise ArduPilotMissionError(
                f"QGC WPL input must begin with {QGC_WPL_110_HEADER!r}"
            )
        items: list[ArduPilotMissionItem] = []
        for line_number, raw_line in enumerate(lines[1:], start=2):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) != 12:
                raise ArduPilotMissionError(
                    f"QGC WPL line {line_number} must contain exactly 12 fields"
                )
            try:
                item = ArduPilotMissionItem(
                    seq=int(fields[0]),
                    current=int(fields[1]),
                    frame=int(fields[2]),
                    command=int(fields[3]),
                    param1=float(fields[4]),
                    param2=float(fields[5]),
                    param3=float(fields[6]),
                    param4=float(fields[7]),
                    latitude_deg=float(fields[8]),
                    longitude_deg=float(fields[9]),
                    altitude_m=float(fields[10]),
                    autocontinue=int(fields[11]),
                )
            except (ValueError, ArduPilotMissionError) as exc:
                raise ArduPilotMissionError(
                    f"QGC WPL line {line_number} is invalid: {exc}"
                ) from exc
            items.append(item)
        return cls(vehicle_id=vehicle_id, items=tuple(items))

    @property
    def json_filename(self) -> str:
        return f"{self.vehicle_id}.ardupilot-mission.json"

    @property
    def waypoint_filename(self) -> str:
        return f"{self.vehicle_id}.waypoints"

    def write_json(self, path: Path | str) -> Path:
        return self._atomic_write(path, self.json_filename, self.to_json())

    def write_qgc_wpl_110(self, path: Path | str) -> Path:
        return self._atomic_write(
            path,
            self.waypoint_filename,
            self.to_qgc_wpl_110(),
        )

    @staticmethod
    def _atomic_write(path: Path | str, filename: str, text: str) -> Path:
        destination = Path(path)
        if destination.exists() and destination.is_dir():
            destination = destination / filename
        elif not destination.suffix:
            destination = destination / filename
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(
            f".{destination.name}.{os.getpid()}.tmp"
        )
        try:
            temporary.write_text(text, encoding="utf-8")
            os.replace(temporary, destination)
        finally:
            if temporary.exists():
                temporary.unlink()
        return destination

    @classmethod
    def read_json(cls, path: Path | str) -> "ArduPilotMission":
        try:
            text = Path(path).read_text(encoding="utf-8")
        except OSError as exc:
            raise ArduPilotMissionError(f"could not read mission JSON: {exc}") from exc
        return cls.from_json(text)

    @classmethod
    def read_qgc_wpl_110(
        cls,
        path: Path | str,
        *,
        vehicle_id: str,
    ) -> "ArduPilotMission":
        try:
            text = Path(path).read_text(encoding="utf-8")
        except OSError as exc:
            raise ArduPilotMissionError(f"could not read waypoint file: {exc}") from exc
        return cls.from_qgc_wpl_110(text, vehicle_id=vehicle_id)


def _item(
    *,
    seq: int,
    command: int,
    latitude_deg: float = 0.0,
    longitude_deg: float = 0.0,
    altitude_m: float = 0.0,
    param1: float = 0.0,
) -> ArduPilotMissionItem:
    return ArduPilotMissionItem(
        seq=seq,
        current=1 if seq == 0 else 0,
        frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=command,
        param1=param1,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        autocontinue=1,
    )


def build_ardupilot_mission(
    route: CompleteVehicleRouteRecord,
    *,
    config: Optional[ArduPilotMissionBuildConfig] = None,
) -> ArduPilotMission:
    """Convert one complete route record into a Copter QGC WPL 110 mission.

    Route ``z_m`` values are interpreted as home-relative altitude because all
    generated positional items use MAV_FRAME_GLOBAL_RELATIVE_ALT.
    """
    if not isinstance(route, CompleteVehicleRouteRecord):
        raise ArduPilotMissionError(
            "route must be a CompleteVehicleRouteRecord"
        )
    if config is not None and not isinstance(config, ArduPilotMissionBuildConfig):
        raise ArduPilotMissionError(
            "config must be an ArduPilotMissionBuildConfig"
        )
    policy = config or ArduPilotMissionBuildConfig()
    if route.is_idle:
        raise ArduPilotMissionError(
            f"vehicle {route.vehicle_id!r} has no route and cannot produce a mission"
        )

    try:
        geographic = route.geographic_waypoints()
    except CompleteVehicleRouteRecordError as exc:
        raise ArduPilotMissionError(
            f"could not convert route waypoints to WGS84: {exc}"
        ) from exc
    if not geographic:
        raise ArduPilotMissionError("route contains no waypoints")

    cruise_altitude = geographic[0].altitude_m
    if cruise_altitude < policy.minimum_relative_altitude_m:
        raise ArduPilotMissionError(
            "route cruise altitude is below minimum_relative_altitude_m"
        )
    for index, waypoint in enumerate(geographic[1:], start=1):
        if not math.isclose(
            waypoint.altitude_m,
            cruise_altitude,
            rel_tol=0.0,
            abs_tol=_ALTITUDE_TOLERANCE_M,
        ):
            raise ArduPilotMissionError(
                f"route waypoint {index} has inconsistent cruise altitude"
            )

    items: list[ArduPilotMissionItem] = []
    first = geographic[0]
    if policy.include_takeoff:
        items.append(
            _item(
                seq=0,
                command=MAV_CMD_NAV_TAKEOFF,
                latitude_deg=first.latitude_deg,
                longitude_deg=first.longitude_deg,
                altitude_m=cruise_altitude,
            )
        )

    start_index = 1 if policy.skip_initial_reference_waypoint else 0
    route_points = geographic[start_index:]
    if not route_points:
        # Preserve a usable navigation item even for a one-point route after the
        # explicit takeoff duplicate has been removed.
        route_points = geographic[-1:]

    for waypoint in route_points:
        items.append(
            _item(
                seq=len(items),
                command=MAV_CMD_NAV_WAYPOINT,
                latitude_deg=waypoint.latitude_deg,
                longitude_deg=waypoint.longitude_deg,
                altitude_m=waypoint.altitude_m,
                param1=policy.waypoint_hold_s,
            )
        )

    if policy.end_action == END_ACTION_RTL:
        items.append(
            _item(
                seq=len(items),
                command=MAV_CMD_NAV_RETURN_TO_LAUNCH,
            )
        )
    elif policy.end_action == END_ACTION_LAND_AT_REFERENCE:
        if not route.return_to_reference:
            raise ArduPilotMissionError(
                "land_at_reference requires route.return_to_reference=true"
            )
        final = geographic[-1]
        if not (
            math.isclose(
                final.latitude_deg,
                first.latitude_deg,
                rel_tol=0.0,
                abs_tol=_POSITION_TOLERANCE_DEG,
            )
            and math.isclose(
                final.longitude_deg,
                first.longitude_deg,
                rel_tol=0.0,
                abs_tol=_POSITION_TOLERANCE_DEG,
            )
        ):
            raise ArduPilotMissionError(
                "land_at_reference requires the final route point at the reference"
            )
        items.append(
            _item(
                seq=len(items),
                command=MAV_CMD_NAV_LAND,
                latitude_deg=first.latitude_deg,
                longitude_deg=first.longitude_deg,
                altitude_m=0.0,
            )
        )

    return ArduPilotMission(vehicle_id=route.vehicle_id, items=tuple(items))


def build_ardupilot_missions(
    routes: Iterable[CompleteVehicleRouteRecord],
    *,
    config: Optional[ArduPilotMissionBuildConfig] = None,
) -> tuple[ArduPilotMission, ...]:
    """Build one mission per non-idle route, sorted by vehicle ID.

    Idle routes are rejected rather than silently omitted.
    """
    try:
        values = tuple(routes)
    except TypeError as exc:
        raise ArduPilotMissionError("routes must be iterable") from exc
    if not values:
        raise ArduPilotMissionError("routes must not be empty")
    if any(not isinstance(route, CompleteVehicleRouteRecord) for route in values):
        raise ArduPilotMissionError(
            "routes must contain only CompleteVehicleRouteRecord objects"
        )
    vehicle_ids = [route.vehicle_id for route in values]
    if len(vehicle_ids) != len(set(vehicle_ids)):
        raise ArduPilotMissionError("route vehicle IDs must be unique")
    return tuple(
        build_ardupilot_mission(route, config=config)
        for route in sorted(values, key=lambda item: item.vehicle_id)
    )
