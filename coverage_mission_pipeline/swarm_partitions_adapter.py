#!/usr/bin/env python3
"""Adapter for the JSON contract exported by ``atissss/Swarm-Partitions``.

The upstream exporter writes boundary, partition, and no-go geometry as WGS84
longitude/latitude rings while recording the projected planning CRS separately.
This module validates that contract, reconstructs the polygonal geometry in the
projected CRS, applies the coverage pipeline's global clearance rule, preserves
every connected partition component, and produces a ``GenericMissionDefinition``.

The exporter intentionally does not define vehicle homes or coverage-planner
parameters.  Those operational values are supplied explicitly through
``SwarmPartitionsAdapterConfig`` rather than inferred from geometry.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from types import MappingProxyType
from typing import Any, Iterable, Mapping, Optional, Union

from pyproj import CRS, Transformer
from shapely import affinity
from shapely.geometry import MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union
from shapely.validation import explain_validity

from .generic_mission_pipeline import (
    ComponentPlanningSpec,
    GenericMissionDefinition,
    GenericMissionPipelineConfig,
    GenericMissionPipelineResult,
    PlanningBatchRunner,
    run_generic_mission_pipeline,
)
from .mission_geometry_core import (
    clip_partition_to_safe_area,
    create_safe_area,
)
from .planning_request import LocalPoint2D
from .prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
    make_prepared_components,
)
from .start_goal_policy import StartGoalPolicyConfig
from .vehicle_component_ordering import VehicleReference


SWARM_PARTITIONS_ADAPTER_ALGORITHM = "swarm_partitions_json_adapter_v1"
_COORDINATE_CRS = "EPSG:4326"
_AXIS_ORDER = ("longitude", "latitude")
_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_REFERENCE_TYPES = frozenset({"home", "launch", "current_position", "custom"})
_NUMERICAL_GEOMETRY_TOLERANCE_M = 1.0e-7


class SwarmPartitionsAdapterError(ValueError):
    """Raised when the exported JSON or explicit adapter configuration is unsafe."""


def _strict_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SwarmPartitionsAdapterError(f"{path} must be an object")
    return value


def _strict_keys(
    value: Mapping[str, Any],
    required: set[str],
    path: str,
    *,
    optional: set[str] | None = None,
) -> None:
    allowed = required | (optional or set())
    actual = set(value)
    missing = sorted(required - actual)
    unknown = sorted(actual - allowed)
    if missing:
        raise SwarmPartitionsAdapterError(
            f"{path} is missing required field(s): {', '.join(missing)}"
        )
    if unknown:
        raise SwarmPartitionsAdapterError(
            f"{path} contains unknown field(s): {', '.join(unknown)}"
        )


def _identifier(value: Any, path: str) -> str:
    if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
        raise SwarmPartitionsAdapterError(
            f"{path} must match {_ID_PATTERN.pattern!r}"
        )
    return value


def _finite_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SwarmPartitionsAdapterError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise SwarmPartitionsAdapterError(f"{path} must be finite")
    return result


def _finite_nonnegative(value: Any, path: str) -> float:
    result = _finite_number(value, path)
    if result < 0.0:
        raise SwarmPartitionsAdapterError(f"{path} must be non-negative")
    return result


def _positive_integer(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise SwarmPartitionsAdapterError(f"{path} must be a positive integer")
    return value


def _canonical_crs(value: Any, path: str) -> CRS:
    if not isinstance(value, str) or not value.strip():
        raise SwarmPartitionsAdapterError(f"{path} must be a non-empty CRS string")
    try:
        return CRS.from_user_input(value.strip())
    except Exception as exc:
        raise SwarmPartitionsAdapterError(f"{path} is not a valid CRS") from exc


def _coordinate_pair(value: Any, path: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise SwarmPartitionsAdapterError(f"{path} must be [longitude, latitude]")
    longitude = _finite_number(value[0], f"{path}[0]")
    latitude = _finite_number(value[1], f"{path}[1]")
    if longitude < -180.0 or longitude > 180.0:
        raise SwarmPartitionsAdapterError(f"{path}[0] must be in [-180, 180]")
    if latitude < -90.0 or latitude > 90.0:
        raise SwarmPartitionsAdapterError(f"{path}[1] must be in [-90, 90]")
    return longitude, latitude


def _ring_coordinates(value: Any, path: str) -> list[tuple[float, float]]:
    if not isinstance(value, list):
        raise SwarmPartitionsAdapterError(f"{path} must be an array")
    coordinates = [
        _coordinate_pair(item, f"{path}[{index}]")
        for index, item in enumerate(value)
    ]
    if len(coordinates) >= 2 and coordinates[0] == coordinates[-1]:
        coordinates.pop()
    if len(set(coordinates)) < 3:
        raise SwarmPartitionsAdapterError(
            f"{path} must contain at least three distinct points"
        )
    return coordinates


def _transform_ring(
    coordinates: Iterable[tuple[float, float]],
    transformer: Transformer,
    path: str,
) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    for index, (longitude, latitude) in enumerate(coordinates):
        try:
            x, y = transformer.transform(longitude, latitude, errcheck=True)
        except Exception as exc:
            raise SwarmPartitionsAdapterError(
                f"{path}[{index}] could not be transformed to the planning CRS"
            ) from exc
        if not math.isfinite(float(x)) or not math.isfinite(float(y)):
            raise SwarmPartitionsAdapterError(
                f"{path}[{index}] transformed to non-finite coordinates"
            )
        result.append((float(x), float(y)))
    return result


def _ring_set_to_polygon(
    value: Any,
    transformer: Transformer,
    path: str,
) -> Polygon:
    mapping = _strict_mapping(value, path)
    _strict_keys(mapping, {"exterior", "holes"}, path)
    exterior_lonlat = _ring_coordinates(mapping["exterior"], f"{path}.exterior")
    holes_raw = mapping["holes"]
    if not isinstance(holes_raw, list):
        raise SwarmPartitionsAdapterError(f"{path}.holes must be an array")
    hole_lonlat = [
        _ring_coordinates(hole, f"{path}.holes[{index}]")
        for index, hole in enumerate(holes_raw)
    ]
    exterior = _transform_ring(exterior_lonlat, transformer, f"{path}.exterior")
    holes = [
        _transform_ring(hole, transformer, f"{path}.holes[{index}]")
        for index, hole in enumerate(hole_lonlat)
    ]
    polygon = Polygon(exterior, holes)
    if polygon.is_empty:
        raise SwarmPartitionsAdapterError(f"{path} must not be empty")
    if not polygon.is_valid:
        raise SwarmPartitionsAdapterError(
            f"{path} is invalid after projection: {explain_validity(polygon)}"
        )
    if polygon.area <= 0.0:
        raise SwarmPartitionsAdapterError(f"{path} must have positive area")
    return polygon


def _geometry_list(
    value: Any,
    transformer: Transformer,
    path: str,
    *,
    require_nonempty: bool = True,
) -> BaseGeometry:
    if not isinstance(value, list):
        raise SwarmPartitionsAdapterError(f"{path} must be an array")
    if require_nonempty and not value:
        raise SwarmPartitionsAdapterError(f"{path} must not be empty")
    polygons = [
        _ring_set_to_polygon(item, transformer, f"{path}[{index}]")
        for index, item in enumerate(value)
    ]
    if not polygons:
        return Polygon()
    geometry: BaseGeometry = polygons[0] if len(polygons) == 1 else MultiPolygon(polygons)
    if not geometry.is_valid:
        raise SwarmPartitionsAdapterError(
            f"{path} is invalid: {explain_validity(geometry)}"
        )
    return geometry


def _translate_to_local(geometry: BaseGeometry, frame: LocalCartesianFrame) -> BaseGeometry:
    return affinity.translate(
        geometry,
        xoff=-frame.origin_easting_m,
        yoff=-frame.origin_northing_m,
    )


@dataclass(frozen=True)
class SwarmPartitionAssignment:
    """Explicit assignment of one exported partition ID to a vehicle ID."""

    partition_id: int
    vehicle_id: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "partition_id",
            _positive_integer(self.partition_id, "partition_id"),
        )
        object.__setattr__(
            self,
            "vehicle_id",
            _identifier(self.vehicle_id, "vehicle_id"),
        )


@dataclass(frozen=True)
class SwarmVehicleMissionProfile:
    """Explicit home/reference and coverage parameters for one vehicle."""

    vehicle_id: str
    reference_longitude_deg: float
    reference_latitude_deg: float
    altitude_m: float
    lateral_footprint_m: float
    lateral_overlap: float
    reference_type: str = "home"
    start_goal_boundary_clearance_m: float = 0.0
    minimum_start_goal_separation_m: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "vehicle_id", _identifier(self.vehicle_id, "vehicle_id"))
        longitude = _finite_number(
            self.reference_longitude_deg,
            "reference_longitude_deg",
        )
        latitude = _finite_number(
            self.reference_latitude_deg,
            "reference_latitude_deg",
        )
        if longitude < -180.0 or longitude > 180.0:
            raise SwarmPartitionsAdapterError(
                "reference_longitude_deg must be in [-180, 180]"
            )
        if latitude < -90.0 or latitude > 90.0:
            raise SwarmPartitionsAdapterError(
                "reference_latitude_deg must be in [-90, 90]"
            )
        altitude = _finite_number(self.altitude_m, "altitude_m")
        footprint = _finite_number(self.lateral_footprint_m, "lateral_footprint_m")
        overlap = _finite_number(self.lateral_overlap, "lateral_overlap")
        if footprint <= 0.0:
            raise SwarmPartitionsAdapterError(
                "lateral_footprint_m must be greater than zero"
            )
        if overlap < 0.0 or overlap >= 1.0:
            raise SwarmPartitionsAdapterError(
                "lateral_overlap must be in the range [0, 1)"
            )
        if self.reference_type not in _REFERENCE_TYPES:
            raise SwarmPartitionsAdapterError(
                "reference_type must be one of: " + ", ".join(sorted(_REFERENCE_TYPES))
            )
        object.__setattr__(self, "reference_longitude_deg", longitude)
        object.__setattr__(self, "reference_latitude_deg", latitude)
        object.__setattr__(self, "altitude_m", altitude)
        object.__setattr__(self, "lateral_footprint_m", footprint)
        object.__setattr__(self, "lateral_overlap", overlap)
        object.__setattr__(
            self,
            "start_goal_boundary_clearance_m",
            _finite_nonnegative(
                self.start_goal_boundary_clearance_m,
                "start_goal_boundary_clearance_m",
            ),
        )
        object.__setattr__(
            self,
            "minimum_start_goal_separation_m",
            _finite_nonnegative(
                self.minimum_start_goal_separation_m,
                "minimum_start_goal_separation_m",
            ),
        )


@dataclass(frozen=True)
class SwarmPartitionsAdapterConfig:
    """Operational inputs missing from the geometry-only exporter contract."""

    assignments: tuple[SwarmPartitionAssignment, ...]
    vehicles: tuple[SwarmVehicleMissionProfile, ...]
    clearance_m: float = 0.0
    min_component_area_m2: float = 0.0
    coverage_gap_tolerance_m2: float = 1.0e-4
    coverage_gap_relative_tolerance: float = 1.0e-9
    partition_overlap_tolerance_m2: float = 1.0e-6
    frame_id: str = "map"

    def __post_init__(self) -> None:
        try:
            assignments = tuple(self.assignments)
            vehicles = tuple(self.vehicles)
        except TypeError as exc:
            raise SwarmPartitionsAdapterError(
                "assignments and vehicles must be iterable"
            ) from exc
        if not assignments:
            raise SwarmPartitionsAdapterError("assignments must not be empty")
        if not vehicles:
            raise SwarmPartitionsAdapterError("vehicles must not be empty")
        if any(not isinstance(item, SwarmPartitionAssignment) for item in assignments):
            raise SwarmPartitionsAdapterError(
                "assignments must contain only SwarmPartitionAssignment objects"
            )
        if any(not isinstance(item, SwarmVehicleMissionProfile) for item in vehicles):
            raise SwarmPartitionsAdapterError(
                "vehicles must contain only SwarmVehicleMissionProfile objects"
            )
        partition_ids = [item.partition_id for item in assignments]
        vehicle_ids = [item.vehicle_id for item in vehicles]
        if len(partition_ids) != len(set(partition_ids)):
            raise SwarmPartitionsAdapterError("assignment partition IDs must be unique")
        if len(vehicle_ids) != len(set(vehicle_ids)):
            raise SwarmPartitionsAdapterError("vehicle profile IDs must be unique")
        known_vehicles = set(vehicle_ids)
        unknown = sorted(
            {item.vehicle_id for item in assignments} - known_vehicles
        )
        if unknown:
            raise SwarmPartitionsAdapterError(
                "assignments reference unknown vehicle(s): " + ", ".join(unknown)
            )
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise SwarmPartitionsAdapterError("frame_id must not be empty")
        object.__setattr__(self, "assignments", assignments)
        object.__setattr__(self, "vehicles", vehicles)
        object.__setattr__(self, "frame_id", self.frame_id.strip())
        for name in (
            "clearance_m",
            "min_component_area_m2",
            "coverage_gap_tolerance_m2",
            "coverage_gap_relative_tolerance",
            "partition_overlap_tolerance_m2",
        ):
            object.__setattr__(
                self,
                name,
                _finite_nonnegative(getattr(self, name), name),
            )


@dataclass(frozen=True)
class SwarmPartitionsAdapterResult:
    """Validated geometry and generic mission inputs reconstructed from the JSON."""

    definition: GenericMissionDefinition
    frame: LocalCartesianFrame
    coordinate_crs: str
    planning_crs: str
    random_seed: int
    boundary_projected: BaseGeometry
    exclusions_projected: tuple[BaseGeometry, ...]
    partitions_projected_by_id: Mapping[int, BaseGeometry]
    safe_area_projected: BaseGeometry
    safe_area_local: BaseGeometry
    component_ids_by_partition_id: Mapping[int, tuple[str, ...]]
    dynamic_exclusion_count: int = 0
    algorithm: str = SWARM_PARTITIONS_ADAPTER_ALGORITHM

    def __post_init__(self) -> None:
        if not isinstance(self.definition, GenericMissionDefinition):
            raise SwarmPartitionsAdapterError(
                "definition must be a GenericMissionDefinition"
            )
        if not isinstance(self.frame, LocalCartesianFrame):
            raise SwarmPartitionsAdapterError("frame must be a LocalCartesianFrame")
        if self.algorithm != SWARM_PARTITIONS_ADAPTER_ALGORITHM:
            raise SwarmPartitionsAdapterError(
                f"algorithm must be {SWARM_PARTITIONS_ADAPTER_ALGORITHM!r}"
            )
        if isinstance(self.random_seed, bool) or not isinstance(self.random_seed, int):
            raise SwarmPartitionsAdapterError("random_seed must be an integer")
        if (
            isinstance(self.dynamic_exclusion_count, bool)
            or not isinstance(self.dynamic_exclusion_count, int)
            or self.dynamic_exclusion_count < 0
        ):
            raise SwarmPartitionsAdapterError(
                "dynamic_exclusion_count must be a non-negative integer"
            )
        for name in ("boundary_projected", "safe_area_projected", "safe_area_local"):
            value = getattr(self, name)
            if not isinstance(value, BaseGeometry) or value.is_empty or not value.is_valid:
                raise SwarmPartitionsAdapterError(f"{name} must be valid non-empty geometry")
        object.__setattr__(self, "exclusions_projected", tuple(self.exclusions_projected))
        object.__setattr__(
            self,
            "partitions_projected_by_id",
            MappingProxyType(dict(self.partitions_projected_by_id)),
        )
        object.__setattr__(
            self,
            "component_ids_by_partition_id",
            MappingProxyType(
                {
                    int(key): tuple(value)
                    for key, value in self.component_ids_by_partition_id.items()
                }
            ),
        )

    @property
    def component_count(self) -> int:
        return len(self.definition.components)

    @property
    def partition_count(self) -> int:
        return len(self.partitions_projected_by_id)

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "algorithm": self.algorithm,
            "coordinate_crs": self.coordinate_crs,
            "planning_crs": self.planning_crs,
            "random_seed": self.random_seed,
            "partition_count": self.partition_count,
            "component_count": self.component_count,
            "vehicle_ids": list(self.definition.vehicle_ids),
            "dynamic_exclusion_count": self.dynamic_exclusion_count,
            "component_ids_by_partition_id": {
                str(key): list(value)
                for key, value in sorted(self.component_ids_by_partition_id.items())
            },
            "safe_area_m2": float(self.safe_area_projected.area),
            "frame": self.frame.to_dict(),
        }


@dataclass(frozen=True)
class SwarmPartitionsPipelineResult:
    """Adapter diagnostics plus the complete Stage 13 pipeline result."""

    adapter: SwarmPartitionsAdapterResult
    mission: GenericMissionPipelineResult

    def __post_init__(self) -> None:
        if not isinstance(self.adapter, SwarmPartitionsAdapterResult):
            raise SwarmPartitionsAdapterError(
                "adapter must be a SwarmPartitionsAdapterResult"
            )
        if not isinstance(self.mission, GenericMissionPipelineResult):
            raise SwarmPartitionsAdapterError(
                "mission must be a GenericMissionPipelineResult"
            )
        if self.mission.active_vehicle_ids != tuple(
            vehicle_id
            for vehicle_id in self.adapter.definition.vehicle_ids
            if any(
                component.assigned_vehicle_id == vehicle_id
                for component in self.adapter.definition.components
            )
        ):
            raise SwarmPartitionsAdapterError(
                "pipeline active vehicles do not match adapted component assignments"
            )

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "adapter": self.adapter.to_summary_dict(),
            "mission": self.mission.to_summary_dict(),
        }


def _parse_payload_geometry(
    payload: Mapping[str, Any],
    config: SwarmPartitionsAdapterConfig,
) -> tuple[
    CRS,
    int,
    BaseGeometry,
    tuple[BaseGeometry, ...],
    dict[int, BaseGeometry],
    int,
]:
    _strict_keys(
        payload,
        {"metadata", "boundary", "partitions", "no_go_zones"},
        "root",
    )
    metadata = _strict_mapping(payload["metadata"], "metadata")
    _strict_keys(metadata, {"crs", "n_partitions", "generation"}, "metadata")
    crs_mapping = _strict_mapping(metadata["crs"], "metadata.crs")
    _strict_keys(
        crs_mapping,
        {"coordinates", "axis_order", "planning"},
        "metadata.crs",
    )
    coordinate_crs = _canonical_crs(
        crs_mapping["coordinates"],
        "metadata.crs.coordinates",
    )
    if coordinate_crs.to_epsg() != 4326:
        raise SwarmPartitionsAdapterError(
            "metadata.crs.coordinates must resolve to EPSG:4326"
        )
    axis_order = crs_mapping["axis_order"]
    if axis_order != list(_AXIS_ORDER):
        raise SwarmPartitionsAdapterError(
            "metadata.crs.axis_order must be ['longitude', 'latitude']"
        )
    planning_crs = _canonical_crs(
        crs_mapping["planning"],
        "metadata.crs.planning",
    )
    if not planning_crs.is_projected:
        raise SwarmPartitionsAdapterError(
            "metadata.crs.planning must be a projected CRS"
        )
    n_partitions = _positive_integer(
        metadata["n_partitions"],
        "metadata.n_partitions",
    )
    generation = _strict_mapping(metadata["generation"], "metadata.generation")
    _strict_keys(generation, {"random_seed"}, "metadata.generation")
    random_seed = generation["random_seed"]
    if isinstance(random_seed, bool) or not isinstance(random_seed, int):
        raise SwarmPartitionsAdapterError(
            "metadata.generation.random_seed must be an integer"
        )

    transformer = Transformer.from_crs(
        coordinate_crs,
        planning_crs,
        always_xy=True,
    )
    boundary_entries = payload["boundary"]
    if not isinstance(boundary_entries, list) or len(boundary_entries) != 1:
        raise SwarmPartitionsAdapterError(
            "boundary must contain exactly one polygon ring set"
        )
    boundary = _geometry_list(
        boundary_entries,
        transformer,
        "boundary",
    )
    if not isinstance(boundary, Polygon):
        raise SwarmPartitionsAdapterError("boundary must resolve to one Polygon")

    partitions_raw = payload["partitions"]
    if not isinstance(partitions_raw, list):
        raise SwarmPartitionsAdapterError("partitions must be an array")
    if len(partitions_raw) != n_partitions:
        raise SwarmPartitionsAdapterError(
            "metadata.n_partitions does not match partitions length"
        )
    partitions: dict[int, BaseGeometry] = {}
    for index, raw in enumerate(partitions_raw):
        item = _strict_mapping(raw, f"partitions[{index}]")
        _strict_keys(item, {"id", "geometry"}, f"partitions[{index}]")
        partition_id = _positive_integer(item["id"], f"partitions[{index}].id")
        if partition_id in partitions:
            raise SwarmPartitionsAdapterError(
                f"duplicate partition ID: {partition_id}"
            )
        partitions[partition_id] = _geometry_list(
            item["geometry"],
            transformer,
            f"partitions[{index}].geometry",
        )
    expected_ids = list(range(1, n_partitions + 1))
    if sorted(partitions) != expected_ids:
        raise SwarmPartitionsAdapterError(
            "partition IDs must be sequential from 1 through metadata.n_partitions"
        )

    no_go = _strict_mapping(payload["no_go_zones"], "no_go_zones")
    _strict_keys(
        no_go,
        {"predetermined"},
        "no_go_zones",
        optional={"dynamic"},
    )
    predetermined_raw = no_go["predetermined"]
    if not isinstance(predetermined_raw, list):
        raise SwarmPartitionsAdapterError(
            "no_go_zones.predetermined must be an array"
        )
    exclusions: list[BaseGeometry] = []
    names: set[str] = set()
    for index, raw in enumerate(predetermined_raw):
        item = _strict_mapping(raw, f"no_go_zones.predetermined[{index}]")
        _strict_keys(
            item,
            {"name", "geometry"},
            f"no_go_zones.predetermined[{index}]",
        )
        name = item["name"]
        if not isinstance(name, str) or not name.strip():
            raise SwarmPartitionsAdapterError(
                f"no_go_zones.predetermined[{index}].name must not be empty"
            )
        if name in names:
            raise SwarmPartitionsAdapterError(
                f"duplicate predetermined no-go name: {name!r}"
            )
        names.add(name)
        exclusions.append(
            _geometry_list(
                item["geometry"],
                transformer,
                f"no_go_zones.predetermined[{index}].geometry",
            )
        )

    dynamic_raw = no_go.get("dynamic", [])
    if not isinstance(dynamic_raw, list):
        raise SwarmPartitionsAdapterError("no_go_zones.dynamic must be an array")
    dynamic_ids: set[int] = set()
    for index, raw in enumerate(dynamic_raw):
        item = _strict_mapping(raw, f"no_go_zones.dynamic[{index}]")
        _strict_keys(item, {"id", "geometry"}, f"no_go_zones.dynamic[{index}]")
        dynamic_id = _positive_integer(
            item["id"],
            f"no_go_zones.dynamic[{index}].id",
        )
        if dynamic_id in dynamic_ids:
            raise SwarmPartitionsAdapterError(
                f"duplicate dynamic no-go ID: {dynamic_id}"
            )
        dynamic_ids.add(dynamic_id)
        exclusions.append(
            _geometry_list(
                item["geometry"],
                transformer,
                f"no_go_zones.dynamic[{index}].geometry",
            )
        )
    if dynamic_ids and sorted(dynamic_ids) != list(range(1, len(dynamic_ids) + 1)):
        raise SwarmPartitionsAdapterError(
            "dynamic no-go IDs must be sequential from 1"
        )

    assignment_ids = sorted(item.partition_id for item in config.assignments)
    if assignment_ids != expected_ids:
        missing = sorted(set(expected_ids) - set(assignment_ids))
        extra = sorted(set(assignment_ids) - set(expected_ids))
        details: list[str] = []
        if missing:
            details.append("missing: " + ", ".join(map(str, missing)))
        if extra:
            details.append("unexpected: " + ", ".join(map(str, extra)))
        suffix = "" if not details else " (" + "; ".join(details) + ")"
        raise SwarmPartitionsAdapterError(
            "assignments must contain exactly one entry per exported partition" + suffix
        )

    tolerance = config.partition_overlap_tolerance_m2
    ordered_partition_ids = sorted(partitions)
    for left_index, left_id in enumerate(ordered_partition_ids):
        left = partitions[left_id]
        outside_area = left.difference(boundary).area
        if outside_area > tolerance:
            raise SwarmPartitionsAdapterError(
                f"partition {left_id} extends outside the mission boundary by "
                f"{outside_area:.9f} m^2"
            )
        for right_id in ordered_partition_ids[left_index + 1 :]:
            overlap = left.intersection(partitions[right_id]).area
            if overlap > tolerance:
                raise SwarmPartitionsAdapterError(
                    f"partitions {left_id} and {right_id} overlap by "
                    f"{overlap:.9f} m^2"
                )

    return (
        planning_crs,
        random_seed,
        boundary,
        tuple(exclusions),
        partitions,
        len(dynamic_raw),
    )


def adapt_swarm_partitions_payload(
    payload: Mapping[str, Any],
    config: SwarmPartitionsAdapterConfig,
) -> SwarmPartitionsAdapterResult:
    """Validate one exporter payload and create a complete generic definition."""
    if not isinstance(payload, Mapping):
        raise SwarmPartitionsAdapterError("payload must be an object")
    if not isinstance(config, SwarmPartitionsAdapterConfig):
        raise SwarmPartitionsAdapterError(
            "config must be a SwarmPartitionsAdapterConfig"
        )

    (
        planning_crs,
        random_seed,
        boundary_projected,
        exclusions_projected,
        partitions_projected,
        dynamic_count,
    ) = _parse_payload_geometry(payload, config)

    safe_area_projected = create_safe_area(
        boundary_projected,
        exclusions_projected,
        config.clearance_m,
    )
    minx, miny, _, _ = boundary_projected.bounds
    frame = LocalCartesianFrame(
        frame_id=config.frame_id,
        projected_crs=planning_crs.to_string(),
        origin_easting_m=float(minx),
        origin_northing_m=float(miny),
    )
    safe_area_local = _translate_to_local(safe_area_projected, frame)

    assignment_by_partition = {
        item.partition_id: item.vehicle_id for item in config.assignments
    }
    prepared_components: list[PreparedComponent] = []
    component_ids_by_partition: dict[int, tuple[str, ...]] = {}
    clipped_projected_components: list[Polygon] = []
    local_component_geometries: list[BaseGeometry] = []

    for partition_id in sorted(partitions_projected):
        projected_components = clip_partition_to_safe_area(
            partitions_projected[partition_id],
            safe_area_projected,
            min_component_area_m2=config.min_component_area_m2,
        )
        if not projected_components:
            raise SwarmPartitionsAdapterError(
                f"partition {partition_id} has no plannable component after clearance"
            )
        clipped_projected_components.extend(projected_components)
        local_components = [
            _translate_to_local(component, frame)
            for component in projected_components
        ]
        local_component_geometries.extend(local_components)
        records = make_prepared_components(
            local_components,
            source_region_id=f"partition_{partition_id}",
            frame=frame,
            assigned_vehicle_id=assignment_by_partition[partition_id],
        )
        prepared_components.extend(records)
        component_ids_by_partition[partition_id] = tuple(
            record.component_id for record in records
        )

    # Projection round trips can leave nanometre-scale boundary slivers even
    # after an exact intersection.  Unioning the translated clipped components
    # back into the translated safe area preserves the authoritative geometry
    # while making the strict downstream ``covers`` invariant numerically stable.
    safe_area_local = unary_union([safe_area_local, *local_component_geometries]).buffer(
        _NUMERICAL_GEOMETRY_TOLERANCE_M,
        join_style=2,
    )
    if safe_area_local.is_empty or not safe_area_local.is_valid:
        raise SwarmPartitionsAdapterError(
            "local safe-area normalization produced invalid geometry"
        )

    covered = unary_union(clipped_projected_components)
    missing_area = float(safe_area_projected.difference(covered).area)
    extra_area = float(covered.difference(safe_area_projected).area)
    allowed_gap = max(
        config.coverage_gap_tolerance_m2,
        config.coverage_gap_relative_tolerance * float(safe_area_projected.area),
    )
    if missing_area > allowed_gap:
        raise SwarmPartitionsAdapterError(
            "exported partitions do not cover the complete safe area: "
            f"missing {missing_area:.9f} m^2, allowed {allowed_gap:.9f} m^2"
        )
    if extra_area > allowed_gap:
        raise SwarmPartitionsAdapterError(
            "clipped partition components extend beyond the safe area: "
            f"extra {extra_area:.9f} m^2"
        )

    try:
        to_projected = Transformer.from_crs(
            _COORDINATE_CRS,
            planning_crs,
            always_xy=True,
        )
    except Exception as exc:  # pragma: no cover - planning CRS already validated
        raise SwarmPartitionsAdapterError(
            "could not construct vehicle-reference transformer"
        ) from exc

    references: list[VehicleReference] = []
    profiles_by_vehicle = {item.vehicle_id: item for item in config.vehicles}
    local_reference_by_vehicle: dict[str, LocalPoint2D] = {}
    for profile in sorted(config.vehicles, key=lambda item: item.vehicle_id):
        try:
            easting, northing = to_projected.transform(
                profile.reference_longitude_deg,
                profile.reference_latitude_deg,
                errcheck=True,
            )
        except Exception as exc:
            raise SwarmPartitionsAdapterError(
                f"vehicle {profile.vehicle_id!r} reference could not be transformed"
            ) from exc
        local = LocalPoint2D(
            float(easting) - frame.origin_easting_m,
            float(northing) - frame.origin_northing_m,
        )
        if not safe_area_local.covers(Point(local.x_m, local.y_m)):
            raise SwarmPartitionsAdapterError(
                f"vehicle {profile.vehicle_id!r} reference is outside the safe area"
            )
        local_reference_by_vehicle[profile.vehicle_id] = local
        references.append(
            VehicleReference(
                vehicle_id=profile.vehicle_id,
                frame=frame,
                position=local,
                reference_type=profile.reference_type,
            )
        )

    planning_specs: list[ComponentPlanningSpec] = []
    for component in prepared_components:
        vehicle_id = component.assigned_vehicle_id
        assert vehicle_id is not None
        profile = profiles_by_vehicle[vehicle_id]
        anchor = local_reference_by_vehicle[vehicle_id]
        planning_specs.append(
            ComponentPlanningSpec(
                component_id=component.component_id,
                request_id=component.component_id,
                start_anchor=anchor,
                goal_anchor=anchor,
                altitude_m=profile.altitude_m,
                lateral_footprint_m=profile.lateral_footprint_m,
                lateral_overlap=profile.lateral_overlap,
                start_goal_policy=StartGoalPolicyConfig(
                    boundary_clearance_m=(
                        profile.start_goal_boundary_clearance_m
                    ),
                    minimum_start_goal_separation_m=(
                        profile.minimum_start_goal_separation_m
                    ),
                ),
            )
        )

    definition = GenericMissionDefinition(
        components=tuple(prepared_components),
        vehicle_references=tuple(references),
        planning_specs=tuple(planning_specs),
        free_space_by_vehicle_id={
            profile.vehicle_id: safe_area_local for profile in config.vehicles
        },
    )
    return SwarmPartitionsAdapterResult(
        definition=definition,
        frame=frame,
        coordinate_crs=_COORDINATE_CRS,
        planning_crs=planning_crs.to_string(),
        random_seed=random_seed,
        boundary_projected=boundary_projected,
        exclusions_projected=exclusions_projected,
        partitions_projected_by_id=partitions_projected,
        safe_area_projected=safe_area_projected,
        safe_area_local=safe_area_local,
        component_ids_by_partition_id=component_ids_by_partition,
        dynamic_exclusion_count=dynamic_count,
    )


def load_swarm_partitions_json(
    path: Path | str,
    config: SwarmPartitionsAdapterConfig,
) -> SwarmPartitionsAdapterResult:
    """Read and adapt one JSON file produced by the coworker's exporter."""
    source = Path(path)
    try:
        text = source.read_text(encoding="utf-8")
    except OSError as exc:
        raise SwarmPartitionsAdapterError(
            f"could not read Swarm-Partitions JSON: {exc}"
        ) from exc
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise SwarmPartitionsAdapterError(
            f"invalid JSON at line {exc.lineno}, column {exc.colno}: {exc.msg}"
        ) from exc
    return adapt_swarm_partitions_payload(payload, config)


def run_swarm_partitions_mission_pipeline(
    source: Union[Path, str, Mapping[str, Any]],
    adapter_config: SwarmPartitionsAdapterConfig,
    planner_runner: PlanningBatchRunner,
    *,
    pipeline_config: Optional[GenericMissionPipelineConfig] = None,
) -> SwarmPartitionsPipelineResult:
    """Adapt the exporter output and run the complete Stage 13 pipeline."""
    if isinstance(source, Mapping):
        adapter = adapt_swarm_partitions_payload(source, adapter_config)
    else:
        adapter = load_swarm_partitions_json(source, adapter_config)
    mission = run_generic_mission_pipeline(
        adapter.definition,
        planner_runner,
        config=pipeline_config,
    )
    return SwarmPartitionsPipelineResult(adapter=adapter, mission=mission)
