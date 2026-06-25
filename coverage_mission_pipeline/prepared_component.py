#!/usr/bin/env python3
"""Stable, ROS-independent representation of one prepared planner component.

The objects in this module form the contract between input-specific mission
adapters and the future ROS planner client.  Every record contains exactly one
connected Shapely Polygon in a local Cartesian metric frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
from typing import Any, Iterable, Mapping, Optional, Sequence, Tuple

from pyproj import CRS
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.geometry.polygon import orient
from shapely.validation import explain_validity

SCHEMA_VERSION = 1
_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class PreparedComponentError(ValueError):
    """Raised when a prepared-component record is unsafe or malformed."""


def _strict_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PreparedComponentError(f"{path} must be an object")
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
        raise PreparedComponentError(
            f"{path} is missing required field(s): {', '.join(missing)}"
        )
    if unknown:
        raise PreparedComponentError(
            f"{path} contains unknown field(s): {', '.join(unknown)}"
        )


def _identifier(value: Any, path: str) -> str:
    if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
        raise PreparedComponentError(
            f"{path} must match {_ID_PATTERN.pattern!r}"
        )
    return value


def _optional_identifier(value: Any, path: str) -> Optional[str]:
    if value is None:
        return None
    return _identifier(value, path)


def _finite_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PreparedComponentError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise PreparedComponentError(f"{path} must be finite")
    return result


def _positive_integer(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise PreparedComponentError(f"{path} must be a positive integer")
    return value


def _canonical_projected_crs(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise PreparedComponentError(f"{path} must be a non-empty CRS string")
    try:
        crs = CRS.from_user_input(value.strip())
    except Exception as exc:
        raise PreparedComponentError(f"{path} is not a valid CRS") from exc
    if not crs.is_projected:
        raise PreparedComponentError(f"{path} must be a projected CRS")
    return crs.to_string()


def _canonical_polygon(geometry: BaseGeometry, path: str) -> Polygon:
    if not isinstance(geometry, Polygon):
        geom_type = getattr(geometry, "geom_type", type(geometry).__name__)
        raise PreparedComponentError(
            f"{path} must be exactly one Polygon, got {geom_type}"
        )
    if geometry.is_empty:
        raise PreparedComponentError(f"{path} must not be empty")
    if not geometry.is_valid:
        raise PreparedComponentError(
            f"{path} is invalid: {explain_validity(geometry)}"
        )
    if geometry.area <= 0.0:
        raise PreparedComponentError(f"{path} must have positive area")

    for ring_name, coordinates in [
        ("hull", geometry.exterior.coords),
        *[
            (f"holes[{index}]", ring.coords)
            for index, ring in enumerate(geometry.interiors)
        ],
    ]:
        for point_index, coordinate in enumerate(coordinates):
            x, y = coordinate[:2]
            if not math.isfinite(float(x)) or not math.isfinite(float(y)):
                raise PreparedComponentError(
                    f"{path}.{ring_name}[{point_index}] must be finite"
                )

    return orient(geometry, sign=1.0)


def _ring_to_json(coordinates: Iterable[Sequence[float]]) -> list[list[float]]:
    points = [(float(point[0]), float(point[1])) for point in coordinates]
    if len(points) >= 2 and points[0] == points[-1]:
        points.pop()
    return [[x, y] for x, y in points]


def _ring_from_json(value: Any, path: str) -> list[Tuple[float, float]]:
    if not isinstance(value, list):
        raise PreparedComponentError(f"{path} must be an array")

    points: list[Tuple[float, float]] = []
    for index, coordinate in enumerate(value):
        if (
            not isinstance(coordinate, (list, tuple))
            or len(coordinate) != 2
        ):
            raise PreparedComponentError(
                f"{path}[{index}] must be [x, y]"
            )
        points.append(
            (
                _finite_number(coordinate[0], f"{path}[{index}][0]"),
                _finite_number(coordinate[1], f"{path}[{index}][1]"),
            )
        )

    if len(points) >= 2 and points[0] == points[-1]:
        points.pop()
    if len(set(points)) < 3:
        raise PreparedComponentError(
            f"{path} must contain at least three distinct points"
        )
    return points


@dataclass(frozen=True)
class LocalCartesianFrame:
    """Metadata needed to map local planner points back to a projected CRS."""

    frame_id: str
    projected_crs: str
    origin_easting_m: float
    origin_northing_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise PreparedComponentError("frame.frame_id must not be empty")
        object.__setattr__(self, "frame_id", self.frame_id.strip())
        object.__setattr__(
            self,
            "projected_crs",
            _canonical_projected_crs(
                self.projected_crs,
                "frame.projected_crs",
            ),
        )
        object.__setattr__(
            self,
            "origin_easting_m",
            _finite_number(
                self.origin_easting_m,
                "frame.origin_projected_m.easting",
            ),
        )
        object.__setattr__(
            self,
            "origin_northing_m",
            _finite_number(
                self.origin_northing_m,
                "frame.origin_projected_m.northing",
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "coordinate_system": "local_cartesian",
            "units": "metres",
            "axis_order": ["x_east", "y_north"],
            "projected_crs": self.projected_crs,
            "origin_projected_m": {
                "easting": self.origin_easting_m,
                "northing": self.origin_northing_m,
            },
        }

    @classmethod
    def from_dict(cls, value: Any) -> "LocalCartesianFrame":
        mapping = _strict_mapping(value, "frame")
        _strict_keys(
            mapping,
            {
                "frame_id",
                "coordinate_system",
                "units",
                "axis_order",
                "projected_crs",
                "origin_projected_m",
            },
            "frame",
        )
        if mapping["coordinate_system"] != "local_cartesian":
            raise PreparedComponentError(
                "frame.coordinate_system must be 'local_cartesian'"
            )
        if mapping["units"] != "metres":
            raise PreparedComponentError("frame.units must be 'metres'")
        if mapping["axis_order"] != ["x_east", "y_north"]:
            raise PreparedComponentError(
                "frame.axis_order must be ['x_east', 'y_north']"
            )
        origin = _strict_mapping(
            mapping["origin_projected_m"],
            "frame.origin_projected_m",
        )
        _strict_keys(
            origin,
            {"easting", "northing"},
            "frame.origin_projected_m",
        )
        return cls(
            frame_id=mapping["frame_id"],
            projected_crs=mapping["projected_crs"],
            origin_easting_m=origin["easting"],
            origin_northing_m=origin["northing"],
        )


@dataclass(frozen=True)
class PreparedComponent:
    """One connected polygon ready for a future planner request."""

    component_id: str
    source_region_id: str
    component_index: int
    frame: LocalCartesianFrame
    polygon: Polygon
    assigned_vehicle_id: Optional[str] = None

    def __post_init__(self) -> None:
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
            "component_index",
            _positive_integer(self.component_index, "component_index"),
        )
        if not isinstance(self.frame, LocalCartesianFrame):
            raise PreparedComponentError(
                "frame must be a LocalCartesianFrame"
            )
        object.__setattr__(
            self,
            "assigned_vehicle_id",
            _optional_identifier(
                self.assigned_vehicle_id,
                "assigned_vehicle_id",
            ),
        )
        object.__setattr__(
            self,
            "polygon",
            _canonical_polygon(self.polygon, "polygon"),
        )

    def to_dict(self) -> dict[str, Any]:
        polygon = orient(self.polygon, sign=1.0)
        return {
            "schema_version": SCHEMA_VERSION,
            "component_id": self.component_id,
            "source_region_id": self.source_region_id,
            "component_index": self.component_index,
            "assigned_vehicle_id": self.assigned_vehicle_id,
            "frame": self.frame.to_dict(),
            "polygon": {
                "hull": _ring_to_json(polygon.exterior.coords),
                "holes": [
                    _ring_to_json(interior.coords)
                    for interior in polygon.interiors
                ],
            },
        }

    @classmethod
    def from_dict(cls, value: Any) -> "PreparedComponent":
        mapping = _strict_mapping(value, "root")
        _strict_keys(
            mapping,
            {
                "schema_version",
                "component_id",
                "source_region_id",
                "component_index",
                "assigned_vehicle_id",
                "frame",
                "polygon",
            },
            "root",
        )
        if mapping["schema_version"] != SCHEMA_VERSION:
            raise PreparedComponentError(
                f"unsupported schema_version: {mapping['schema_version']!r}"
            )

        polygon_mapping = _strict_mapping(mapping["polygon"], "polygon")
        _strict_keys(polygon_mapping, {"hull", "holes"}, "polygon")
        hull = _ring_from_json(polygon_mapping["hull"], "polygon.hull")
        holes_raw = polygon_mapping["holes"]
        if not isinstance(holes_raw, list):
            raise PreparedComponentError("polygon.holes must be an array")
        holes = [
            _ring_from_json(hole, f"polygon.holes[{index}]")
            for index, hole in enumerate(holes_raw)
        ]

        return cls(
            component_id=mapping["component_id"],
            source_region_id=mapping["source_region_id"],
            component_index=mapping["component_index"],
            assigned_vehicle_id=mapping["assigned_vehicle_id"],
            frame=LocalCartesianFrame.from_dict(mapping["frame"]),
            polygon=Polygon(hull, holes),
        )

    def to_json(self) -> str:
        """Return canonical, deterministic UTF-8 JSON text."""
        return json.dumps(
            self.to_dict(),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        ) + "\n"

    @classmethod
    def from_json(cls, text: str) -> "PreparedComponent":
        if not isinstance(text, str):
            raise PreparedComponentError("JSON input must be text")
        try:
            value = json.loads(text)
        except json.JSONDecodeError as exc:
            raise PreparedComponentError(
                f"invalid JSON at line {exc.lineno}, "
                f"column {exc.colno}: {exc.msg}"
            ) from exc
        return cls.from_dict(value)

    @property
    def filename(self) -> str:
        return f"{self.component_id}.json"

    def write(self, path: Path | str) -> Path:
        """Atomically write this record and return the destination path."""
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(f".{destination.name}.tmp")
        temporary.write_text(self.to_json(), encoding="utf-8")
        os.replace(temporary, destination)
        return destination

    @classmethod
    def read(cls, path: Path | str) -> "PreparedComponent":
        source = Path(path)
        try:
            text = source.read_text(encoding="utf-8")
        except FileNotFoundError as exc:
            raise PreparedComponentError(
                f"prepared component does not exist: {source}"
            ) from exc
        return cls.from_json(text)


def make_prepared_components(
    polygons: Iterable[BaseGeometry],
    *,
    source_region_id: str,
    frame: LocalCartesianFrame,
    assigned_vehicle_id: Optional[str] = None,
) -> list[PreparedComponent]:
    """Convert an ordered component sequence into deterministic records."""
    region_id = _identifier(source_region_id, "source_region_id")
    vehicle_id = _optional_identifier(
        assigned_vehicle_id,
        "assigned_vehicle_id",
    )
    if not isinstance(frame, LocalCartesianFrame):
        raise PreparedComponentError("frame must be a LocalCartesianFrame")

    result: list[PreparedComponent] = []
    for index, polygon in enumerate(polygons, start=1):
        result.append(
            PreparedComponent(
                component_id=f"{region_id}_component_{index}",
                source_region_id=region_id,
                component_index=index,
                assigned_vehicle_id=vehicle_id,
                frame=frame,
                polygon=polygon,
            )
        )
    return result
