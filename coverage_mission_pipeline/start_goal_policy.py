#!/usr/bin/env python3
"""Deterministic start/goal selection from explicit local reference anchors.

This module does not infer launch sites, vehicle homes, component ordering, or
route connectors.  A caller supplies one start reference anchor and one goal
reference anchor in the same local Cartesian frame as a PreparedComponent.
The policy keeps an anchor unchanged when it is already feasible; otherwise it
projects it to the nearest feasible point.  An optional boundary-clearance
constraint can keep selected points away from the component hull and holes.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable

from shapely.geometry import GeometryCollection, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points, unary_union

from .planning_request import CoveragePlanningRequest, LocalPoint2D
from .prepared_component import PreparedComponent


class StartGoalPolicyError(ValueError):
    """Raised when start/goal policy inputs or results are unsafe."""


def _finite_nonnegative(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise StartGoalPolicyError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise StartGoalPolicyError(f"{name} must be finite and non-negative")
    return result


def _polygon_parts(geometry: BaseGeometry) -> tuple[Polygon, ...]:
    """Extract every positive-area Polygon from a polygonal result."""
    if geometry.is_empty:
        return ()
    if isinstance(geometry, Polygon):
        return (geometry,) if geometry.area > 0.0 else ()
    if isinstance(geometry, MultiPolygon):
        parts = [part for part in geometry.geoms if part.area > 0.0]
    elif isinstance(geometry, GeometryCollection):
        parts = []
        for child in geometry.geoms:
            parts.extend(_polygon_parts(child))
    else:
        return ()

    parts.sort(
        key=lambda part: (
            round(part.bounds[0], 12),
            round(part.bounds[1], 12),
            round(part.bounds[2], 12),
            round(part.bounds[3], 12),
            round(part.area, 12),
        )
    )
    return tuple(parts)


@dataclass(frozen=True)
class StartGoalPolicyConfig:
    """Explicit constraints for nearest-feasible-point selection."""

    boundary_clearance_m: float = 0.0
    minimum_start_goal_separation_m: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "boundary_clearance_m",
            _finite_nonnegative(
                self.boundary_clearance_m,
                "boundary_clearance_m",
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
class StartGoalSelection:
    """Traceable result of projecting explicit anchors into component space."""

    component_id: str
    start_anchor: LocalPoint2D
    goal_anchor: LocalPoint2D
    start: LocalPoint2D
    goal: LocalPoint2D
    boundary_clearance_m: float
    start_projection_distance_m: float
    goal_projection_distance_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.component_id, str) or not self.component_id:
            raise StartGoalPolicyError("component_id must not be empty")
        for name in ("start_anchor", "goal_anchor", "start", "goal"):
            if not isinstance(getattr(self, name), LocalPoint2D):
                raise StartGoalPolicyError(f"{name} must be a LocalPoint2D")
        object.__setattr__(
            self,
            "boundary_clearance_m",
            _finite_nonnegative(
                self.boundary_clearance_m,
                "boundary_clearance_m",
            ),
        )
        object.__setattr__(
            self,
            "start_projection_distance_m",
            _finite_nonnegative(
                self.start_projection_distance_m,
                "start_projection_distance_m",
            ),
        )
        object.__setattr__(
            self,
            "goal_projection_distance_m",
            _finite_nonnegative(
                self.goal_projection_distance_m,
                "goal_projection_distance_m",
            ),
        )

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "component_id": self.component_id,
            "boundary_clearance_m": self.boundary_clearance_m,
            "start_anchor": self.start_anchor.to_dict(),
            "goal_anchor": self.goal_anchor.to_dict(),
            "start": self.start.to_dict(),
            "goal": self.goal.to_dict(),
            "start_projection_distance_m": self.start_projection_distance_m,
            "goal_projection_distance_m": self.goal_projection_distance_m,
        }


def _feasible_geometry(
    component: PreparedComponent,
    clearance_m: float,
) -> BaseGeometry:
    if clearance_m == 0.0:
        geometry: BaseGeometry = component.polygon
    else:
        geometry = component.polygon.buffer(
            -clearance_m,
            join_style=2,
            mitre_limit=5.0,
        )

    parts = _polygon_parts(geometry)
    if not parts:
        raise StartGoalPolicyError(
            f"component {component.component_id!r} has no positive-area free "
            f"space after applying {clearance_m:.6f} m boundary clearance"
        )
    feasible = parts[0] if len(parts) == 1 else unary_union(parts)
    if feasible.is_empty or not feasible.is_valid:
        raise StartGoalPolicyError(
            f"component {component.component_id!r} produced invalid feasible "
            "start/goal geometry"
        )
    return feasible


def _nearest_feasible_point(
    feasible: BaseGeometry,
    anchor: LocalPoint2D,
) -> tuple[LocalPoint2D, float]:
    anchor_geometry = Point(anchor.x_m, anchor.y_m)
    if feasible.covers(anchor_geometry):
        return anchor, 0.0

    candidates: list[tuple[float, float, float]] = []
    for part in _polygon_parts(feasible):
        _, nearest = nearest_points(anchor_geometry, part)
        x = float(nearest.x)
        y = float(nearest.y)
        distance = math.hypot(x - anchor.x_m, y - anchor.y_m)
        if not all(math.isfinite(value) for value in (x, y, distance)):
            raise StartGoalPolicyError(
                "nearest-point projection produced non-finite coordinates"
            )
        candidates.append((distance, x, y))

    if not candidates:
        raise StartGoalPolicyError("no polygonal feasible point was available")

    distance, x, y = min(
        candidates,
        key=lambda item: (
            round(item[0], 12),
            round(item[1], 12),
            round(item[2], 12),
        ),
    )
    selected = LocalPoint2D(x, y)
    if not feasible.covers(Point(selected.x_m, selected.y_m)):
        raise StartGoalPolicyError(
            "nearest-point projection did not produce a feasible point"
        )
    return selected, distance


def select_start_goal(
    component: PreparedComponent,
    *,
    start_anchor: LocalPoint2D,
    goal_anchor: LocalPoint2D,
    config: StartGoalPolicyConfig | None = None,
) -> StartGoalSelection:
    """Select deterministic feasible points nearest to explicit anchors."""
    if not isinstance(component, PreparedComponent):
        raise StartGoalPolicyError("component must be a PreparedComponent")
    if not isinstance(start_anchor, LocalPoint2D):
        raise StartGoalPolicyError("start_anchor must be a LocalPoint2D")
    if not isinstance(goal_anchor, LocalPoint2D):
        raise StartGoalPolicyError("goal_anchor must be a LocalPoint2D")
    if config is not None and not isinstance(config, StartGoalPolicyConfig):
        raise StartGoalPolicyError("config must be a StartGoalPolicyConfig")

    policy = config or StartGoalPolicyConfig()
    feasible = _feasible_geometry(component, policy.boundary_clearance_m)
    start, start_distance = _nearest_feasible_point(feasible, start_anchor)
    goal, goal_distance = _nearest_feasible_point(feasible, goal_anchor)

    separation = math.hypot(start.x_m - goal.x_m, start.y_m - goal.y_m)
    if separation + 1.0e-9 < policy.minimum_start_goal_separation_m:
        raise StartGoalPolicyError(
            "selected start and goal separation is "
            f"{separation:.6f} m, below required minimum "
            f"{policy.minimum_start_goal_separation_m:.6f} m"
        )

    return StartGoalSelection(
        component_id=component.component_id,
        start_anchor=start_anchor,
        goal_anchor=goal_anchor,
        start=start,
        goal=goal,
        boundary_clearance_m=policy.boundary_clearance_m,
        start_projection_distance_m=start_distance,
        goal_projection_distance_m=goal_distance,
    )


def planning_request_from_anchors(
    component: PreparedComponent,
    *,
    start_anchor: LocalPoint2D,
    goal_anchor: LocalPoint2D,
    altitude_m: float,
    lateral_footprint_m: float,
    lateral_overlap: float,
    request_id: str | None = None,
    policy_config: StartGoalPolicyConfig | None = None,
) -> tuple[CoveragePlanningRequest, StartGoalSelection]:
    """Build one validated request while retaining selection diagnostics."""
    selection = select_start_goal(
        component,
        start_anchor=start_anchor,
        goal_anchor=goal_anchor,
        config=policy_config,
    )
    request = CoveragePlanningRequest.for_component(
        component,
        start=selection.start,
        goal=selection.goal,
        altitude_m=altitude_m,
        lateral_footprint_m=lateral_footprint_m,
        lateral_overlap=lateral_overlap,
        request_id=request_id,
    )
    return request, selection
