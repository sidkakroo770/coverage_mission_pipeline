#!/usr/bin/env python3
"""Vehicle references and deterministic ordering of assigned components.

The ordering policy is intentionally geometry-only and route-independent.  It
starts at an explicit vehicle reference point, chooses the component with the
smallest straight-line distance lower bound, then repeats from the geometry of
the selected component.  This produces a deterministic greedy visit sequence
before route connectors are constructed.

The recorded transition points are geometry proxies, not flight connectors.
A later connector layer must still test straight-line visibility and route
around exclusions when a direct connection is blocked.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
from typing import Any, Iterable, Mapping, Optional

from shapely.geometry import Point
from shapely.ops import nearest_points

from .planning_request import LocalPoint2D, PlanningRequestError
from .prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
    PreparedComponentError,
)
from .route_record import (
    GeographicWaypoint,
    ProjectedWaypoint,
    RouteRecordError,
    geographic_to_local,
    projected_to_local,
)

VEHICLE_REFERENCE_SCHEMA_VERSION = 1
COMPONENT_ORDERING_ALGORITHM = "greedy_nearest_component_v1"
_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_REFERENCE_TYPES = frozenset({"home", "launch", "current_position", "custom"})
_DISTANCE_ROUND_DIGITS = 12
_OVERLAP_AREA_TOLERANCE_M2 = 1.0e-9


class VehicleOrderingError(ValueError):
    """Raised when vehicle references or component ordering inputs are unsafe."""


def _identifier(value: Any, path: str) -> str:
    if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
        raise VehicleOrderingError(f"{path} must match {_ID_PATTERN.pattern!r}")
    return value


def _strict_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise VehicleOrderingError(f"{path} must be an object")
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
        raise VehicleOrderingError(
            f"{path} is missing required field(s): {', '.join(missing)}"
        )
    if unknown:
        raise VehicleOrderingError(
            f"{path} contains unknown field(s): {', '.join(unknown)}"
        )


def _finite_nonnegative(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise VehicleOrderingError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise VehicleOrderingError(f"{path} must be finite and non-negative")
    return result


def _frames_match(left: LocalCartesianFrame, right: LocalCartesianFrame) -> bool:
    return left == right


def _local_point_from_projected(
    frame: LocalCartesianFrame,
    easting_m: Any,
    northing_m: Any,
) -> LocalPoint2D:
    try:
        projected = ProjectedWaypoint(easting_m, northing_m, 0.0)
        local = projected_to_local(projected, frame)
        return LocalPoint2D(local.x_m, local.y_m)
    except (RouteRecordError, PlanningRequestError) as exc:
        raise VehicleOrderingError(str(exc)) from exc


def _local_point_from_geographic(
    frame: LocalCartesianFrame,
    longitude_deg: Any,
    latitude_deg: Any,
) -> LocalPoint2D:
    try:
        geographic = GeographicWaypoint(longitude_deg, latitude_deg, 0.0)
        local = geographic_to_local(geographic, frame)
        return LocalPoint2D(local.x_m, local.y_m)
    except (RouteRecordError, PlanningRequestError) as exc:
        raise VehicleOrderingError(str(exc)) from exc


@dataclass(frozen=True)
class VehicleReference:
    """One explicit vehicle anchor expressed in a local Cartesian frame."""

    vehicle_id: str
    frame: LocalCartesianFrame
    position: LocalPoint2D
    reference_type: str = "home"

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "vehicle_id",
            _identifier(self.vehicle_id, "vehicle_id"),
        )
        if not isinstance(self.frame, LocalCartesianFrame):
            raise VehicleOrderingError("frame must be a LocalCartesianFrame")
        if not isinstance(self.position, LocalPoint2D):
            raise VehicleOrderingError("position must be a LocalPoint2D")
        if self.reference_type not in _REFERENCE_TYPES:
            raise VehicleOrderingError(
                "reference_type must be one of: "
                + ", ".join(sorted(_REFERENCE_TYPES))
            )

    @classmethod
    def from_projected(
        cls,
        vehicle_id: str,
        frame: LocalCartesianFrame,
        *,
        easting_m: Any,
        northing_m: Any,
        reference_type: str = "home",
    ) -> "VehicleReference":
        """Create a reference from coordinates in the frame's projected CRS."""
        if not isinstance(frame, LocalCartesianFrame):
            raise VehicleOrderingError("frame must be a LocalCartesianFrame")
        return cls(
            vehicle_id=vehicle_id,
            frame=frame,
            position=_local_point_from_projected(frame, easting_m, northing_m),
            reference_type=reference_type,
        )

    @classmethod
    def from_geographic(
        cls,
        vehicle_id: str,
        frame: LocalCartesianFrame,
        *,
        longitude_deg: Any,
        latitude_deg: Any,
        reference_type: str = "home",
    ) -> "VehicleReference":
        """Create a reference from WGS84 longitude/latitude coordinates."""
        if not isinstance(frame, LocalCartesianFrame):
            raise VehicleOrderingError("frame must be a LocalCartesianFrame")
        return cls(
            vehicle_id=vehicle_id,
            frame=frame,
            position=_local_point_from_geographic(
                frame,
                longitude_deg,
                latitude_deg,
            ),
            reference_type=reference_type,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": VEHICLE_REFERENCE_SCHEMA_VERSION,
            "vehicle_id": self.vehicle_id,
            "reference_type": self.reference_type,
            "frame": self.frame.to_dict(),
            "position_local_m": self.position.to_dict(),
        }

    @classmethod
    def from_dict(cls, value: Any) -> "VehicleReference":
        mapping = _strict_mapping(value, "root")
        _strict_keys(
            mapping,
            {
                "schema_version",
                "vehicle_id",
                "reference_type",
                "frame",
                "position_local_m",
            },
            "root",
        )
        if mapping["schema_version"] != VEHICLE_REFERENCE_SCHEMA_VERSION:
            raise VehicleOrderingError(
                f"unsupported schema_version: {mapping['schema_version']!r}"
            )
        position = _strict_mapping(mapping["position_local_m"], "position_local_m")
        _strict_keys(position, {"x_m", "y_m"}, "position_local_m")
        try:
            frame = LocalCartesianFrame.from_dict(mapping["frame"])
            local = LocalPoint2D(position["x_m"], position["y_m"])
        except (PreparedComponentError, PlanningRequestError) as exc:
            raise VehicleOrderingError(str(exc)) from exc
        return cls(
            vehicle_id=mapping["vehicle_id"],
            frame=frame,
            position=local,
            reference_type=mapping["reference_type"],
        )

    def to_json(self) -> str:
        return json.dumps(
            self.to_dict(),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        ) + "\n"

    @classmethod
    def from_json(cls, text: str) -> "VehicleReference":
        if not isinstance(text, str):
            raise VehicleOrderingError("JSON input must be text")
        try:
            value = json.loads(text)
        except json.JSONDecodeError as exc:
            raise VehicleOrderingError(
                f"invalid JSON at line {exc.lineno}, "
                f"column {exc.colno}: {exc.msg}"
            ) from exc
        return cls.from_dict(value)

    @property
    def filename(self) -> str:
        return f"{self.vehicle_id}.reference.json"

    def write(self, path: Path | str) -> Path:
        """Atomically write this reference and return the destination path."""
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
    def read(cls, path: Path | str) -> "VehicleReference":
        try:
            text = Path(path).read_text(encoding="utf-8")
        except OSError as exc:
            raise VehicleOrderingError(
                f"could not read vehicle reference: {exc}"
            ) from exc
        return cls.from_json(text)


@dataclass(frozen=True)
class ComponentVisit:
    """One component in a vehicle's deterministic greedy visit sequence."""

    visit_index: int
    component: PreparedComponent
    predecessor_component_id: Optional[str]
    transition_start: LocalPoint2D
    transition_end: LocalPoint2D
    straight_line_lower_bound_m: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.visit_index, bool)
            or not isinstance(self.visit_index, int)
            or self.visit_index < 1
        ):
            raise VehicleOrderingError("visit_index must be a positive integer")
        if not isinstance(self.component, PreparedComponent):
            raise VehicleOrderingError("component must be a PreparedComponent")
        if self.predecessor_component_id is not None:
            object.__setattr__(
                self,
                "predecessor_component_id",
                _identifier(
                    self.predecessor_component_id,
                    "predecessor_component_id",
                ),
            )
        if not isinstance(self.transition_start, LocalPoint2D):
            raise VehicleOrderingError(
                "transition_start must be a LocalPoint2D"
            )
        if not isinstance(self.transition_end, LocalPoint2D):
            raise VehicleOrderingError("transition_end must be a LocalPoint2D")
        distance = _finite_nonnegative(
            self.straight_line_lower_bound_m,
            "straight_line_lower_bound_m",
        )
        actual = math.hypot(
            self.transition_end.x_m - self.transition_start.x_m,
            self.transition_end.y_m - self.transition_start.y_m,
        )
        if not math.isclose(distance, actual, rel_tol=0.0, abs_tol=1.0e-8):
            raise VehicleOrderingError(
                "straight_line_lower_bound_m does not match transition points"
            )
        object.__setattr__(self, "straight_line_lower_bound_m", distance)
        if not self.component.polygon.covers(
            Point(self.transition_end.x_m, self.transition_end.y_m)
        ):
            raise VehicleOrderingError(
                "transition_end must lie on or inside the visited component"
            )

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "visit_index": self.visit_index,
            "component_id": self.component.component_id,
            "source_region_id": self.component.source_region_id,
            "predecessor_component_id": self.predecessor_component_id,
            "transition_start": self.transition_start.to_dict(),
            "transition_end": self.transition_end.to_dict(),
            "straight_line_lower_bound_m": self.straight_line_lower_bound_m,
        }


@dataclass(frozen=True)
class VehicleComponentPlan:
    """An ordered, complete set of component visits for one vehicle."""

    reference: VehicleReference
    visits: tuple[ComponentVisit, ...]
    algorithm: str = COMPONENT_ORDERING_ALGORITHM

    def __post_init__(self) -> None:
        if not isinstance(self.reference, VehicleReference):
            raise VehicleOrderingError("reference must be a VehicleReference")
        if self.algorithm != COMPONENT_ORDERING_ALGORITHM:
            raise VehicleOrderingError(
                f"algorithm must be {COMPONENT_ORDERING_ALGORITHM!r}"
            )
        try:
            visits = tuple(self.visits)
        except TypeError as exc:
            raise VehicleOrderingError("visits must be iterable") from exc
        if any(not isinstance(visit, ComponentVisit) for visit in visits):
            raise VehicleOrderingError(
                "visits must contain only ComponentVisit objects"
            )
        object.__setattr__(self, "visits", visits)

        ids = [visit.component.component_id for visit in visits]
        if len(ids) != len(set(ids)):
            raise VehicleOrderingError("component IDs must be unique in a plan")

        previous: Optional[ComponentVisit] = None
        for expected_index, visit in enumerate(visits, start=1):
            component = visit.component
            if visit.visit_index != expected_index:
                raise VehicleOrderingError(
                    "visit indices must be consecutive and one-based"
                )
            if component.assigned_vehicle_id != self.reference.vehicle_id:
                raise VehicleOrderingError(
                    f"component {component.component_id!r} is not assigned to "
                    f"vehicle {self.reference.vehicle_id!r}"
                )
            if not _frames_match(component.frame, self.reference.frame):
                raise VehicleOrderingError(
                    f"component {component.component_id!r} frame does not match "
                    "the vehicle reference frame"
                )
            start_point = Point(
                visit.transition_start.x_m,
                visit.transition_start.y_m,
            )
            if previous is None:
                if visit.predecessor_component_id is not None:
                    raise VehicleOrderingError(
                        "the first visit must not have a predecessor component"
                    )
                if visit.transition_start != self.reference.position:
                    raise VehicleOrderingError(
                        "the first transition must start at the vehicle reference"
                    )
            else:
                if (
                    visit.predecessor_component_id
                    != previous.component.component_id
                ):
                    raise VehicleOrderingError(
                        "visit predecessor chain is inconsistent"
                    )
                if not previous.component.polygon.covers(start_point):
                    raise VehicleOrderingError(
                        "transition_start must lie on or inside the predecessor "
                        "component"
                    )
            previous = visit

    @property
    def vehicle_id(self) -> str:
        return self.reference.vehicle_id

    @property
    def ordered_components(self) -> tuple[PreparedComponent, ...]:
        return tuple(visit.component for visit in self.visits)

    @property
    def component_ids(self) -> tuple[str, ...]:
        return tuple(component.component_id for component in self.ordered_components)

    @property
    def total_straight_line_lower_bound_m(self) -> float:
        return sum(visit.straight_line_lower_bound_m for visit in self.visits)

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "vehicle_id": self.vehicle_id,
            "reference_type": self.reference.reference_type,
            "algorithm": self.algorithm,
            "component_count": len(self.visits),
            "component_ids": list(self.component_ids),
            "total_straight_line_lower_bound_m": (
                self.total_straight_line_lower_bound_m
            ),
            "visits": [visit.to_summary_dict() for visit in self.visits],
        }


def _validate_component_collection(
    components: Iterable[PreparedComponent],
    *,
    context: str,
) -> tuple[PreparedComponent, ...]:
    try:
        values = tuple(components)
    except TypeError as exc:
        raise VehicleOrderingError(f"{context} must be iterable") from exc
    for index, component in enumerate(values):
        if not isinstance(component, PreparedComponent):
            raise VehicleOrderingError(
                f"{context}[{index}] must be a PreparedComponent"
            )
    ids = [component.component_id for component in values]
    if len(ids) != len(set(ids)):
        raise VehicleOrderingError("component IDs must be globally unique")

    for left_index, left in enumerate(values):
        for right in values[left_index + 1 :]:
            try:
                overlap_area = left.polygon.intersection(right.polygon).area
            except Exception as exc:
                raise VehicleOrderingError(
                    "could not validate component overlap"
                ) from exc
            if overlap_area > _OVERLAP_AREA_TOLERANCE_M2:
                raise VehicleOrderingError(
                    f"components {left.component_id!r} and "
                    f"{right.component_id!r} overlap by "
                    f"{overlap_area:.9f} m^2"
                )
    return values


def _transition_between(source: Any, target: PreparedComponent) -> tuple[
    LocalPoint2D,
    LocalPoint2D,
    float,
]:
    try:
        source_point, target_point = nearest_points(source, target.polygon)
        start = LocalPoint2D(float(source_point.x), float(source_point.y))
        end = LocalPoint2D(float(target_point.x), float(target_point.y))
    except Exception as exc:
        raise VehicleOrderingError(
            f"could not compute transition to component "
            f"{target.component_id!r}"
        ) from exc
    distance = math.hypot(end.x_m - start.x_m, end.y_m - start.y_m)
    if not math.isfinite(distance):
        raise VehicleOrderingError("transition distance is not finite")
    return start, end, distance


def order_components_for_vehicle(
    reference: VehicleReference,
    components: Iterable[PreparedComponent],
) -> VehicleComponentPlan:
    """Greedily order every assigned component without dropping any component.

    Candidate ranking uses the exact planar distance from the vehicle reference
    point for the first visit and polygon-to-polygon distance thereafter.  Equal
    distances are resolved by component ID, making the result independent of
    input iteration order.
    """
    if not isinstance(reference, VehicleReference):
        raise VehicleOrderingError("reference must be a VehicleReference")
    values = _validate_component_collection(components, context="components")
    for component in values:
        if component.assigned_vehicle_id is None:
            raise VehicleOrderingError(
                f"component {component.component_id!r} has no vehicle assignment"
            )
        if component.assigned_vehicle_id != reference.vehicle_id:
            raise VehicleOrderingError(
                f"component {component.component_id!r} is assigned to "
                f"{component.assigned_vehicle_id!r}, not {reference.vehicle_id!r}"
            )
        if not _frames_match(component.frame, reference.frame):
            raise VehicleOrderingError(
                f"component {component.component_id!r} frame does not match "
                "the vehicle reference frame"
            )

    remaining = list(values)
    visits: list[ComponentVisit] = []
    source_geometry: Any = Point(reference.position.x_m, reference.position.y_m)
    predecessor_id: Optional[str] = None

    while remaining:
        ranked: list[tuple[tuple[Any, ...], PreparedComponent]] = []
        for candidate in remaining:
            try:
                distance = float(source_geometry.distance(candidate.polygon))
            except Exception as exc:
                raise VehicleOrderingError(
                    f"could not rank component {candidate.component_id!r}"
                ) from exc
            if not math.isfinite(distance) or distance < 0.0:
                raise VehicleOrderingError(
                    f"distance to component {candidate.component_id!r} is invalid"
                )
            key = (
                round(distance, _DISTANCE_ROUND_DIGITS),
                candidate.component_id,
                candidate.source_region_id,
                candidate.component_index,
            )
            ranked.append((key, candidate))

        _, selected = min(ranked, key=lambda item: item[0])
        transition_start, transition_end, distance = _transition_between(
            source_geometry,
            selected,
        )
        if predecessor_id is None:
            # nearest_points(Point, polygon) returns the original point when it
            # lies inside the polygon, but explicitly preserve the reference in
            # all first-visit cases for traceability.
            transition_start = reference.position
            distance = math.hypot(
                transition_end.x_m - transition_start.x_m,
                transition_end.y_m - transition_start.y_m,
            )

        visits.append(
            ComponentVisit(
                visit_index=len(visits) + 1,
                component=selected,
                predecessor_component_id=predecessor_id,
                transition_start=transition_start,
                transition_end=transition_end,
                straight_line_lower_bound_m=distance,
            )
        )
        remaining.remove(selected)
        source_geometry = selected.polygon
        predecessor_id = selected.component_id

    plan = VehicleComponentPlan(reference=reference, visits=tuple(visits))
    if set(plan.component_ids) != {component.component_id for component in values}:
        raise VehicleOrderingError("component ordering did not preserve every input")
    return plan


def order_components_by_vehicle(
    components: Iterable[PreparedComponent],
    references: Iterable[VehicleReference],
) -> tuple[VehicleComponentPlan, ...]:
    """Group and order all components, returning one plan for every reference.

    References with no assigned components produce an empty plan.  Every input
    component must have an assignment and a matching reference; duplicate IDs,
    missing references and positive-area component overlaps are rejected.
    """
    values = _validate_component_collection(components, context="components")
    try:
        reference_values = tuple(references)
    except TypeError as exc:
        raise VehicleOrderingError("references must be iterable") from exc
    for index, reference in enumerate(reference_values):
        if not isinstance(reference, VehicleReference):
            raise VehicleOrderingError(
                f"references[{index}] must be a VehicleReference"
            )
    vehicle_ids = [reference.vehicle_id for reference in reference_values]
    if len(vehicle_ids) != len(set(vehicle_ids)):
        raise VehicleOrderingError("vehicle reference IDs must be unique")

    references_by_id = {
        reference.vehicle_id: reference for reference in reference_values
    }
    grouped: dict[str, list[PreparedComponent]] = {
        vehicle_id: [] for vehicle_id in references_by_id
    }
    for component in values:
        vehicle_id = component.assigned_vehicle_id
        if vehicle_id is None:
            raise VehicleOrderingError(
                f"component {component.component_id!r} has no vehicle assignment"
            )
        if vehicle_id not in references_by_id:
            raise VehicleOrderingError(
                f"component {component.component_id!r} references unknown vehicle "
                f"{vehicle_id!r}"
            )
        grouped[vehicle_id].append(component)

    plans = tuple(
        order_components_for_vehicle(
            references_by_id[vehicle_id],
            grouped[vehicle_id],
        )
        for vehicle_id in sorted(references_by_id)
    )
    planned_ids = [
        component_id
        for plan in plans
        for component_id in plan.component_ids
    ]
    input_ids = [component.component_id for component in values]
    if len(planned_ids) != len(input_ids) or set(planned_ids) != set(input_ids):
        raise VehicleOrderingError(
            "vehicle grouping did not preserve every input component exactly once"
        )
    return plans
