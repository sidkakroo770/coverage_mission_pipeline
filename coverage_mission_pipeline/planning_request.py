#!/usr/bin/env python3
"""Validated, ROS-independent inputs for one coverage-planning request.

This module does not choose start or goal points.  A caller must provide them
explicitly in the same local Cartesian frame as the prepared component.  The
model rejects points outside the connected free-space polygon, including points
inside holes, before any ROS service request can be constructed.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Any

from shapely.geometry import Point

from .prepared_component import PreparedComponent

_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class PlanningRequestError(ValueError):
    """Raised when planner inputs are malformed or geometrically invalid."""


def _identifier(value: Any, path: str) -> str:
    if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
        raise PlanningRequestError(
            f"{path} must match {_ID_PATTERN.pattern!r}"
        )
    return value


def _finite_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlanningRequestError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise PlanningRequestError(f"{path} must be finite")
    return result


@dataclass(frozen=True)
class LocalPoint2D:
    """A finite point in a prepared component's local metric frame."""

    x_m: float
    y_m: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x_m", _finite_number(self.x_m, "point.x_m"))
        object.__setattr__(self, "y_m", _finite_number(self.y_m, "point.y_m"))

    def to_dict(self) -> dict[str, float]:
        return {"x_m": self.x_m, "y_m": self.y_m}


@dataclass(frozen=True)
class CoveragePlanningRequest:
    """Complete validated inputs for one future PlanCoverage service call."""

    request_id: str
    component: PreparedComponent
    start: LocalPoint2D
    goal: LocalPoint2D
    altitude_m: float
    lateral_footprint_m: float
    lateral_overlap: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "request_id",
            _identifier(self.request_id, "request_id"),
        )
        if not isinstance(self.component, PreparedComponent):
            raise PlanningRequestError(
                "component must be a PreparedComponent"
            )
        if not isinstance(self.start, LocalPoint2D):
            raise PlanningRequestError("start must be a LocalPoint2D")
        if not isinstance(self.goal, LocalPoint2D):
            raise PlanningRequestError("goal must be a LocalPoint2D")

        object.__setattr__(
            self,
            "altitude_m",
            _finite_number(self.altitude_m, "altitude_m"),
        )
        footprint = _finite_number(
            self.lateral_footprint_m,
            "lateral_footprint_m",
        )
        if footprint <= 0.0:
            raise PlanningRequestError(
                "lateral_footprint_m must be greater than zero"
            )
        object.__setattr__(self, "lateral_footprint_m", footprint)

        overlap = _finite_number(self.lateral_overlap, "lateral_overlap")
        if overlap < 0.0 or overlap >= 1.0:
            raise PlanningRequestError(
                "lateral_overlap must be in the range [0, 1)"
            )
        object.__setattr__(self, "lateral_overlap", overlap)

        self._validate_point_in_free_space(self.start, "start")
        self._validate_point_in_free_space(self.goal, "goal")

    def _validate_point_in_free_space(
        self,
        point: LocalPoint2D,
        path: str,
    ) -> None:
        geometry_point = Point(point.x_m, point.y_m)
        if not self.component.polygon.covers(geometry_point):
            raise PlanningRequestError(
                f"{path} must lie inside or on the boundary of the "
                "prepared component free space"
            )

    @classmethod
    def for_component(
        cls,
        component: PreparedComponent,
        *,
        start: LocalPoint2D,
        goal: LocalPoint2D,
        altitude_m: float,
        lateral_footprint_m: float,
        lateral_overlap: float,
        request_id: str | None = None,
    ) -> "CoveragePlanningRequest":
        """Create a request using the component ID unless one is supplied."""
        if not isinstance(component, PreparedComponent):
            raise PlanningRequestError(
                "component must be a PreparedComponent"
            )
        return cls(
            request_id=(
                component.component_id if request_id is None else request_id
            ),
            component=component,
            start=start,
            goal=goal,
            altitude_m=altitude_m,
            lateral_footprint_m=lateral_footprint_m,
            lateral_overlap=lateral_overlap,
        )

    def to_summary_dict(self) -> dict[str, Any]:
        """Return deterministic request metadata without duplicating geometry."""
        return {
            "request_id": self.request_id,
            "component_id": self.component.component_id,
            "source_region_id": self.component.source_region_id,
            "assigned_vehicle_id": self.component.assigned_vehicle_id,
            "frame_id": self.component.frame.frame_id,
            "start": self.start.to_dict(),
            "goal": self.goal.to_dict(),
            "altitude_m": self.altitude_m,
            "lateral_footprint_m": self.lateral_footprint_m,
            "lateral_overlap": self.lateral_overlap,
        }
