#!/usr/bin/env python3
"""Build a PlanCoverage request message without creating a ROS client.

This is a deterministic message-conversion layer only.  It does not initialize
rclpy, wait for a service, send a request, choose start/goal points, or process
a response.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Type

from .planning_request import CoveragePlanningRequest
from .ros_polygon_conversion import (
    RosPolygonConversionError,
    RosPolygonMessageTypes,
    component_to_polygon_with_holes_stamped,
)


class RosPlanCoverageConversionError(ValueError):
    """Raised when a validated request cannot be represented in ROS."""


@dataclass(frozen=True)
class RosPlanCoverageMessageTypes:
    """Injectable generated message classes used by the request converter."""

    point32: Type[Any]
    polygon: Type[Any]
    polygon_with_holes_stamped: Type[Any]
    pose_stamped: Type[Any]
    plan_coverage_request: Type[Any]


def load_ros_plan_coverage_message_types() -> RosPlanCoverageMessageTypes:
    """Load the generated ROS classes from a sourced ROS 2 workspace."""
    try:
        from geometry_msgs.msg import Point32, Polygon, PoseStamped
        from polygon_coverage_msgs.msg import PolygonWithHolesStamped
        from polygon_coverage_msgs.srv import PlanCoverage
    except ImportError as exc:
        raise RosPlanCoverageConversionError(
            "PlanCoverage ROS interfaces are unavailable; source the ROS 2 "
            "and coverage workspace setup files before using this converter"
        ) from exc

    return RosPlanCoverageMessageTypes(
        point32=Point32,
        polygon=Polygon,
        polygon_with_holes_stamped=PolygonWithHolesStamped,
        pose_stamped=PoseStamped,
        plan_coverage_request=PlanCoverage.Request,
    )


def _pose_message(
    request: CoveragePlanningRequest,
    point: Any,
    altitude_m: float,
    polygon_message: Any,
    message_types: RosPlanCoverageMessageTypes,
) -> Any:
    pose = message_types.pose_stamped()
    pose.header.frame_id = request.component.frame.frame_id
    pose.header.stamp.sec = polygon_message.header.stamp.sec
    pose.header.stamp.nanosec = polygon_message.header.stamp.nanosec

    pose.pose.position.x = point.x_m
    pose.pose.position.y = point.y_m
    pose.pose.position.z = altitude_m
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose


def planning_request_to_ros(
    request: CoveragePlanningRequest,
    *,
    stamp: Optional[Any] = None,
    message_types: Optional[RosPlanCoverageMessageTypes] = None,
) -> Any:
    """Convert one validated model into a PlanCoverage.Request message."""
    if not isinstance(request, CoveragePlanningRequest):
        raise RosPlanCoverageConversionError(
            "request must be a CoveragePlanningRequest"
        )

    types = message_types or load_ros_plan_coverage_message_types()
    polygon_types = RosPolygonMessageTypes(
        point32=types.point32,
        polygon=types.polygon,
        polygon_with_holes_stamped=types.polygon_with_holes_stamped,
    )

    try:
        polygon_message = component_to_polygon_with_holes_stamped(
            request.component,
            altitude_m=request.altitude_m,
            stamp=stamp,
            message_types=polygon_types,
        )
        # Polygon points are Point32, so use the represented altitude for poses
        # as well.  This keeps all request z values exactly consistent.
        represented_altitude = polygon_message.polygon.hull.points[0].z

        ros_request = types.plan_coverage_request()
        required_fields = (
            "polygon",
            "start_pose",
            "goal_pose",
            "lateral_footprint",
            "lateral_overlap",
        )
        if any(not hasattr(ros_request, field) for field in required_fields):
            raise RosPlanCoverageConversionError(
                "the supplied ROS classes do not match the PlanCoverage "
                "request contract"
            )

        ros_request.polygon = polygon_message
        ros_request.start_pose = _pose_message(
            request,
            request.start,
            represented_altitude,
            polygon_message,
            types,
        )
        ros_request.goal_pose = _pose_message(
            request,
            request.goal,
            represented_altitude,
            polygon_message,
            types,
        )
        ros_request.lateral_footprint = request.lateral_footprint_m
        ros_request.lateral_overlap = request.lateral_overlap
    except RosPolygonConversionError as exc:
        raise RosPlanCoverageConversionError(str(exc)) from exc
    except RosPlanCoverageConversionError:
        raise
    except (AttributeError, TypeError, ValueError, OverflowError, IndexError) as exc:
        raise RosPlanCoverageConversionError(
            "the supplied ROS classes do not match the PlanCoverage request "
            "contract"
        ) from exc

    return ros_request
