#!/usr/bin/env python3
"""Validated ROS-independent results from one coverage-planning request."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Optional

from .planning_request import CoveragePlanningRequest


class PlanningResultError(ValueError):
    """Raised when a planner response is malformed or reports failure."""


class PlanningServiceRejectedError(PlanningResultError):
    """Raised when /plan_coverage explicitly returns success=false."""


@dataclass(frozen=True)
class CoverageWaypoint:
    """One finite route waypoint in the component's local metric frame."""

    x_m: float
    y_m: float
    z_m: float

    def __post_init__(self) -> None:
        for name in ("x_m", "y_m", "z_m"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise PlanningResultError(f"waypoint.{name} must be a number")
            converted = float(value)
            if not math.isfinite(converted):
                raise PlanningResultError(f"waypoint.{name} must be finite")
            object.__setattr__(self, name, converted)

    def to_dict(self) -> dict[str, float]:
        return {"x_m": self.x_m, "y_m": self.y_m, "z_m": self.z_m}


@dataclass(frozen=True)
class CoveragePlanningResult:
    """Successful ordered route returned for one validated request."""

    request_id: str
    component_id: str
    source_region_id: str
    assigned_vehicle_id: Optional[str]
    frame_id: str
    response_message: str
    waypoints: tuple[CoverageWaypoint, ...]

    def __post_init__(self) -> None:
        text_fields = {
            "request_id": self.request_id,
            "component_id": self.component_id,
            "source_region_id": self.source_region_id,
            "frame_id": self.frame_id,
            "response_message": self.response_message,
        }
        for name, value in text_fields.items():
            if not isinstance(value, str):
                raise PlanningResultError(f"{name} must be a string")
            if name != "response_message" and not value:
                raise PlanningResultError(f"{name} must not be empty")

        if self.assigned_vehicle_id is not None and not isinstance(
            self.assigned_vehicle_id, str
        ):
            raise PlanningResultError(
                "assigned_vehicle_id must be a string or None"
            )

        if not isinstance(self.waypoints, tuple):
            object.__setattr__(self, "waypoints", tuple(self.waypoints))
        if not self.waypoints:
            raise PlanningResultError("waypoints must not be empty")
        if any(not isinstance(point, CoverageWaypoint) for point in self.waypoints):
            raise PlanningResultError(
                "waypoints must contain only CoverageWaypoint objects"
            )

    @classmethod
    def from_request(
        cls,
        request: CoveragePlanningRequest,
        *,
        response_message: str,
        waypoints: Iterable[CoverageWaypoint],
    ) -> "CoveragePlanningResult":
        if not isinstance(request, CoveragePlanningRequest):
            raise PlanningResultError(
                "request must be a CoveragePlanningRequest"
            )
        return cls(
            request_id=request.request_id,
            component_id=request.component.component_id,
            source_region_id=request.component.source_region_id,
            assigned_vehicle_id=request.component.assigned_vehicle_id,
            frame_id=request.component.frame.frame_id,
            response_message=response_message,
            waypoints=tuple(waypoints),
        )

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "request_id": self.request_id,
            "component_id": self.component_id,
            "source_region_id": self.source_region_id,
            "assigned_vehicle_id": self.assigned_vehicle_id,
            "frame_id": self.frame_id,
            "response_message": self.response_message,
            "waypoint_count": len(self.waypoints),
            "first_waypoint": self.waypoints[0].to_dict(),
            "last_waypoint": self.waypoints[-1].to_dict(),
        }


def plan_coverage_response_to_result(
    request: CoveragePlanningRequest,
    response: Any,
) -> CoveragePlanningResult:
    """Validate a generated PlanCoverage response and detach its route data."""
    if not isinstance(request, CoveragePlanningRequest):
        raise PlanningResultError(
            "request must be a CoveragePlanningRequest"
        )

    required = ("success", "message", "waypoints")
    if any(not hasattr(response, field) for field in required):
        raise PlanningResultError(
            "response does not match the PlanCoverage response contract"
        )

    if not isinstance(response.success, bool):
        raise PlanningResultError("response.success must be a bool")
    if not isinstance(response.message, str):
        raise PlanningResultError("response.message must be a string")
    if not response.success:
        diagnostic = response.message or "planner returned no diagnostic"
        raise PlanningServiceRejectedError(diagnostic)

    try:
        frame_id = response.waypoints.header.frame_id
        poses = response.waypoints.poses
    except AttributeError as exc:
        raise PlanningResultError(
            "response.waypoints does not match geometry_msgs/PoseArray"
        ) from exc

    expected_frame = request.component.frame.frame_id
    if frame_id != expected_frame:
        raise PlanningResultError(
            "response waypoint frame mismatch: "
            f"expected {expected_frame!r}, got {frame_id!r}"
        )

    try:
        pose_list = list(poses)
    except TypeError as exc:
        raise PlanningResultError("response waypoints.poses must be iterable") from exc
    if not pose_list:
        raise PlanningResultError(
            "planner reported success but returned no waypoints"
        )

    detached: list[CoverageWaypoint] = []
    for index, pose in enumerate(pose_list):
        try:
            position = pose.position
            point = CoverageWaypoint(position.x, position.y, position.z)
        except AttributeError as exc:
            raise PlanningResultError(
                f"response waypoint {index} has no valid position"
            ) from exc
        except PlanningResultError as exc:
            raise PlanningResultError(
                f"response waypoint {index} is invalid: {exc}"
            ) from exc
        detached.append(point)

    first_z = detached[0].z_m
    for index, point in enumerate(detached[1:], start=1):
        if not math.isclose(point.z_m, first_z, rel_tol=0.0, abs_tol=1.0e-6):
            raise PlanningResultError(
                f"response waypoint {index} has inconsistent altitude"
            )

    return CoveragePlanningResult.from_request(
        request,
        response_message=response.message,
        waypoints=detached,
    )
