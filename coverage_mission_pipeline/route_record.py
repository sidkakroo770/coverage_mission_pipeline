#!/usr/bin/env python3
"""Stable route records and local/projected/geographic coordinate conversion.

A route record stores only the planner's local Cartesian waypoints plus the
LocalCartesianFrame needed to recover projected or geographic coordinates.
Projected and longitude/latitude coordinates are derived on demand so the
serialized contract has one authoritative coordinate representation.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
from typing import Any, Iterable, Mapping, Optional

from pyproj import CRS, Transformer

from .planning_result import (
    CoveragePlanningResult,
    CoverageWaypoint,
    PlanningResultError,
)
from .prepared_component import LocalCartesianFrame, PreparedComponentError

ROUTE_SCHEMA_VERSION = 1
_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class RouteRecordError(ValueError):
    """Raised when a route record or coordinate conversion is unsafe."""


def _strict_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise RouteRecordError(f"{path} must be an object")
    return value


def _strict_keys(
    value: Mapping[str, Any],
    required: set[str],
    path: str,
) -> None:
    actual = set(value.keys())
    missing = sorted(required - actual)
    unknown = sorted(actual - required)
    if missing:
        raise RouteRecordError(
            f"{path} is missing required field(s): {', '.join(missing)}"
        )
    if unknown:
        raise RouteRecordError(
            f"{path} contains unknown field(s): {', '.join(unknown)}"
        )


def _identifier(value: Any, path: str) -> str:
    if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
        raise RouteRecordError(f"{path} must match {_ID_PATTERN.pattern!r}")
    return value


def _optional_identifier(value: Any, path: str) -> Optional[str]:
    if value is None:
        return None
    return _identifier(value, path)


def _finite_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RouteRecordError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise RouteRecordError(f"{path} must be finite")
    return result


def _validated_frame(frame: Any) -> LocalCartesianFrame:
    if not isinstance(frame, LocalCartesianFrame):
        raise RouteRecordError("frame must be a LocalCartesianFrame")
    return frame


@dataclass(frozen=True)
class ProjectedWaypoint:
    """One finite waypoint expressed in the frame's projected CRS."""

    easting_m: float
    northing_m: float
    altitude_m: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "easting_m",
            _finite_number(self.easting_m, "projected.easting_m"),
        )
        object.__setattr__(
            self,
            "northing_m",
            _finite_number(self.northing_m, "projected.northing_m"),
        )
        object.__setattr__(
            self,
            "altitude_m",
            _finite_number(self.altitude_m, "projected.altitude_m"),
        )

    def to_dict(self) -> dict[str, float]:
        return {
            "easting_m": self.easting_m,
            "northing_m": self.northing_m,
            "altitude_m": self.altitude_m,
        }


@dataclass(frozen=True)
class GeographicWaypoint:
    """One finite WGS84 longitude/latitude waypoint with unchanged altitude."""

    longitude_deg: float
    latitude_deg: float
    altitude_m: float

    def __post_init__(self) -> None:
        longitude = _finite_number(self.longitude_deg, "geographic.longitude_deg")
        latitude = _finite_number(self.latitude_deg, "geographic.latitude_deg")
        altitude = _finite_number(self.altitude_m, "geographic.altitude_m")
        if longitude < -180.0 or longitude > 180.0:
            raise RouteRecordError(
                "geographic.longitude_deg must be in the range [-180, 180]"
            )
        if latitude < -90.0 or latitude > 90.0:
            raise RouteRecordError(
                "geographic.latitude_deg must be in the range [-90, 90]"
            )
        object.__setattr__(self, "longitude_deg", longitude)
        object.__setattr__(self, "latitude_deg", latitude)
        object.__setattr__(self, "altitude_m", altitude)

    def to_dict(self) -> dict[str, float]:
        return {
            "longitude_deg": self.longitude_deg,
            "latitude_deg": self.latitude_deg,
            "altitude_m": self.altitude_m,
        }


def local_to_projected(
    waypoint: CoverageWaypoint,
    frame: LocalCartesianFrame,
) -> ProjectedWaypoint:
    """Translate one local waypoint into the frame's projected coordinates."""
    if not isinstance(waypoint, CoverageWaypoint):
        raise RouteRecordError("waypoint must be a CoverageWaypoint")
    checked_frame = _validated_frame(frame)
    return ProjectedWaypoint(
        easting_m=checked_frame.origin_easting_m + waypoint.x_m,
        northing_m=checked_frame.origin_northing_m + waypoint.y_m,
        altitude_m=waypoint.z_m,
    )


def projected_to_local(
    waypoint: ProjectedWaypoint,
    frame: LocalCartesianFrame,
) -> CoverageWaypoint:
    """Translate one projected waypoint into the frame's local coordinates."""
    if not isinstance(waypoint, ProjectedWaypoint):
        raise RouteRecordError("waypoint must be a ProjectedWaypoint")
    checked_frame = _validated_frame(frame)
    try:
        return CoverageWaypoint(
            waypoint.easting_m - checked_frame.origin_easting_m,
            waypoint.northing_m - checked_frame.origin_northing_m,
            waypoint.altitude_m,
        )
    except PlanningResultError as exc:
        raise RouteRecordError(str(exc)) from exc


def _transformer(source: Any, target: Any) -> Transformer:
    try:
        source_crs = CRS.from_user_input(source)
        target_crs = CRS.from_user_input(target)
        return Transformer.from_crs(source_crs, target_crs, always_xy=True)
    except Exception as exc:
        raise RouteRecordError("could not construct coordinate transformer") from exc


def projected_to_geographic(
    waypoint: ProjectedWaypoint,
    frame: LocalCartesianFrame,
) -> GeographicWaypoint:
    """Convert one projected waypoint to WGS84 longitude/latitude."""
    if not isinstance(waypoint, ProjectedWaypoint):
        raise RouteRecordError("waypoint must be a ProjectedWaypoint")
    checked_frame = _validated_frame(frame)
    transformer = _transformer(checked_frame.projected_crs, "EPSG:4326")
    try:
        longitude, latitude = transformer.transform(
            waypoint.easting_m,
            waypoint.northing_m,
            errcheck=True,
        )
        return GeographicWaypoint(longitude, latitude, waypoint.altitude_m)
    except RouteRecordError:
        raise
    except Exception as exc:
        raise RouteRecordError(
            "projected waypoint could not be transformed to WGS84"
        ) from exc


def geographic_to_projected(
    waypoint: GeographicWaypoint,
    frame: LocalCartesianFrame,
) -> ProjectedWaypoint:
    """Convert one WGS84 longitude/latitude waypoint to the projected CRS."""
    if not isinstance(waypoint, GeographicWaypoint):
        raise RouteRecordError("waypoint must be a GeographicWaypoint")
    checked_frame = _validated_frame(frame)
    transformer = _transformer("EPSG:4326", checked_frame.projected_crs)
    try:
        easting, northing = transformer.transform(
            waypoint.longitude_deg,
            waypoint.latitude_deg,
            errcheck=True,
        )
        return ProjectedWaypoint(easting, northing, waypoint.altitude_m)
    except RouteRecordError:
        raise
    except Exception as exc:
        raise RouteRecordError(
            "geographic waypoint could not be transformed to projected CRS"
        ) from exc


def local_to_geographic(
    waypoint: CoverageWaypoint,
    frame: LocalCartesianFrame,
) -> GeographicWaypoint:
    """Convert one local waypoint directly to WGS84 longitude/latitude."""
    return projected_to_geographic(local_to_projected(waypoint, frame), frame)


def geographic_to_local(
    waypoint: GeographicWaypoint,
    frame: LocalCartesianFrame,
) -> CoverageWaypoint:
    """Convert one WGS84 waypoint directly to the frame's local coordinates."""
    return projected_to_local(geographic_to_projected(waypoint, frame), frame)


@dataclass(frozen=True)
class CoverageRouteRecord:
    """Stable georeferenced record of one successful component route."""

    request_id: str
    component_id: str
    source_region_id: str
    assigned_vehicle_id: Optional[str]
    frame: LocalCartesianFrame
    response_message: str
    waypoints: tuple[CoverageWaypoint, ...]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "request_id",
            _identifier(self.request_id, "request_id"),
        )
        object.__setattr__(
            self,
            "component_id",
            _identifier(self.component_id, "component_id"),
        )
        object.__setattr__(
            self,
            "source_region_id",
            _identifier(self.source_region_id, "source_region_id"),
        )
        object.__setattr__(
            self,
            "assigned_vehicle_id",
            _optional_identifier(
                self.assigned_vehicle_id,
                "assigned_vehicle_id",
            ),
        )
        _validated_frame(self.frame)
        if not isinstance(self.response_message, str):
            raise RouteRecordError("response_message must be a string")
        try:
            points = tuple(self.waypoints)
        except TypeError as exc:
            raise RouteRecordError("waypoints must be iterable") from exc
        if not points:
            raise RouteRecordError("waypoints must not be empty")
        if any(not isinstance(point, CoverageWaypoint) for point in points):
            raise RouteRecordError(
                "waypoints must contain only CoverageWaypoint objects"
            )
        object.__setattr__(self, "waypoints", points)

    @classmethod
    def from_result(
        cls,
        result: CoveragePlanningResult,
        frame: LocalCartesianFrame,
    ) -> "CoverageRouteRecord":
        """Attach complete frame metadata to one validated planner result."""
        if not isinstance(result, CoveragePlanningResult):
            raise RouteRecordError("result must be a CoveragePlanningResult")
        checked_frame = _validated_frame(frame)
        if result.frame_id != checked_frame.frame_id:
            raise RouteRecordError(
                "result frame mismatch: "
                f"expected {checked_frame.frame_id!r}, got {result.frame_id!r}"
            )
        return cls(
            request_id=result.request_id,
            component_id=result.component_id,
            source_region_id=result.source_region_id,
            assigned_vehicle_id=result.assigned_vehicle_id,
            frame=checked_frame,
            response_message=result.response_message,
            waypoints=result.waypoints,
        )

    def to_result(self) -> CoveragePlanningResult:
        """Recover the validated ROS-independent planner result."""
        try:
            return CoveragePlanningResult(
                request_id=self.request_id,
                component_id=self.component_id,
                source_region_id=self.source_region_id,
                assigned_vehicle_id=self.assigned_vehicle_id,
                frame_id=self.frame.frame_id,
                response_message=self.response_message,
                waypoints=self.waypoints,
            )
        except PlanningResultError as exc:
            raise RouteRecordError(str(exc)) from exc

    def projected_waypoints(self) -> tuple[ProjectedWaypoint, ...]:
        """Return every waypoint in the frame's projected CRS, preserving order."""
        return tuple(local_to_projected(point, self.frame) for point in self.waypoints)

    def geographic_waypoints(self) -> tuple[GeographicWaypoint, ...]:
        """Return every waypoint as WGS84 longitude/latitude, preserving order."""
        return tuple(local_to_geographic(point, self.frame) for point in self.waypoints)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": ROUTE_SCHEMA_VERSION,
            "request_id": self.request_id,
            "component_id": self.component_id,
            "source_region_id": self.source_region_id,
            "assigned_vehicle_id": self.assigned_vehicle_id,
            "frame": self.frame.to_dict(),
            "response_message": self.response_message,
            "waypoints_local_m": [point.to_dict() for point in self.waypoints],
        }

    @classmethod
    def from_dict(cls, value: Any) -> "CoverageRouteRecord":
        mapping = _strict_mapping(value, "root")
        _strict_keys(
            mapping,
            {
                "schema_version",
                "request_id",
                "component_id",
                "source_region_id",
                "assigned_vehicle_id",
                "frame",
                "response_message",
                "waypoints_local_m",
            },
            "root",
        )
        if mapping["schema_version"] != ROUTE_SCHEMA_VERSION:
            raise RouteRecordError(
                f"unsupported schema_version: {mapping['schema_version']!r}"
            )
        if not isinstance(mapping["response_message"], str):
            raise RouteRecordError("response_message must be a string")
        raw_waypoints = mapping["waypoints_local_m"]
        if not isinstance(raw_waypoints, list):
            raise RouteRecordError("waypoints_local_m must be an array")
        waypoints: list[CoverageWaypoint] = []
        for index, raw in enumerate(raw_waypoints):
            point = _strict_mapping(raw, f"waypoints_local_m[{index}]")
            _strict_keys(
                point,
                {"x_m", "y_m", "z_m"},
                f"waypoints_local_m[{index}]",
            )
            try:
                waypoints.append(
                    CoverageWaypoint(point["x_m"], point["y_m"], point["z_m"])
                )
            except PlanningResultError as exc:
                raise RouteRecordError(
                    f"waypoints_local_m[{index}] is invalid: {exc}"
                ) from exc
        try:
            frame = LocalCartesianFrame.from_dict(mapping["frame"])
        except PreparedComponentError as exc:
            raise RouteRecordError(f"frame is invalid: {exc}") from exc
        return cls(
            request_id=mapping["request_id"],
            component_id=mapping["component_id"],
            source_region_id=mapping["source_region_id"],
            assigned_vehicle_id=mapping["assigned_vehicle_id"],
            frame=frame,
            response_message=mapping["response_message"],
            waypoints=tuple(waypoints),
        )

    def to_json(self) -> str:
        """Return canonical deterministic UTF-8 JSON text."""
        return json.dumps(
            self.to_dict(),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        ) + "\n"

    @classmethod
    def from_json(cls, text: str) -> "CoverageRouteRecord":
        if not isinstance(text, str):
            raise RouteRecordError("JSON input must be text")
        try:
            value = json.loads(text)
        except json.JSONDecodeError as exc:
            raise RouteRecordError(
                f"invalid JSON at line {exc.lineno}, "
                f"column {exc.colno}: {exc.msg}"
            ) from exc
        return cls.from_dict(value)

    @property
    def filename(self) -> str:
        return f"{self.request_id}.route.json"

    def write(self, path: Path | str) -> Path:
        """Atomically write this route record and return the destination path."""
        destination = Path(path)
        if destination.exists() and destination.is_dir():
            destination = destination / self.filename
        elif not destination.suffix:
            destination = destination / self.filename
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(
            f".{destination.name}.{os.getpid()}.tmp"
        )
        try:
            temporary.write_text(self.to_json(), encoding="utf-8")
            os.replace(temporary, destination)
        finally:
            if temporary.exists():
                temporary.unlink()
        return destination

    @classmethod
    def read(cls, path: Path | str) -> "CoverageRouteRecord":
        source = Path(path)
        try:
            text = source.read_text(encoding="utf-8")
        except OSError as exc:
            raise RouteRecordError(f"could not read route record: {exc}") from exc
        return cls.from_json(text)


def make_route_records(
    results: Iterable[CoveragePlanningResult],
    frames_by_component_id: Mapping[str, LocalCartesianFrame],
) -> tuple[CoverageRouteRecord, ...]:
    """Attach frames to every result without silently dropping any result."""
    if not isinstance(frames_by_component_id, Mapping):
        raise RouteRecordError("frames_by_component_id must be a mapping")
    try:
        result_list = tuple(results)
    except TypeError as exc:
        raise RouteRecordError("results must be iterable") from exc
    if not result_list:
        raise RouteRecordError("results must not be empty")
    records: list[CoverageRouteRecord] = []
    seen_components: set[str] = set()
    for index, result in enumerate(result_list):
        if not isinstance(result, CoveragePlanningResult):
            raise RouteRecordError(
                f"results[{index}] must be a CoveragePlanningResult"
            )
        if result.component_id in seen_components:
            raise RouteRecordError(
                f"duplicate component_id in results: {result.component_id!r}"
            )
        seen_components.add(result.component_id)
        if result.component_id not in frames_by_component_id:
            raise RouteRecordError(
                f"missing frame for component {result.component_id!r}"
            )
        records.append(
            CoverageRouteRecord.from_result(
                result,
                frames_by_component_id[result.component_id],
            )
        )
    return tuple(records)
