#!/usr/bin/env python3
"""Convert prepared components into ROS polygon messages without planning.

This module intentionally does not create a ROS node, call a service, choose
start/goal poses, or import rclpy.  It performs only the deterministic boundary
conversion required before constructing a future PlanCoverage request.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Any, Optional, Type

from .prepared_component import PreparedComponent


class RosPolygonConversionError(ValueError):
    """Raised when a prepared component cannot be represented safely in ROS."""


@dataclass(frozen=True)
class RosPolygonMessageTypes:
    """Injectable ROS message classes used by the converter.

    Dependency injection keeps the geometry conversion unit-testable without a
    running ROS graph.  Production callers normally use load_ros_message_types().
    """

    point32: Type[Any]
    polygon: Type[Any]
    polygon_with_holes_stamped: Type[Any]


def load_ros_message_types() -> RosPolygonMessageTypes:
    """Load generated ROS 2 message classes from the sourced workspace."""
    try:
        from geometry_msgs.msg import Point32, Polygon
        from polygon_coverage_msgs.msg import PolygonWithHolesStamped
    except ImportError as exc:
        raise RosPolygonConversionError(
            "ROS polygon message packages are unavailable; source the ROS 2 "
            "and coverage workspace setup files before using this converter"
        ) from exc

    return RosPolygonMessageTypes(
        point32=Point32,
        polygon=Polygon,
        polygon_with_holes_stamped=PolygonWithHolesStamped,
    )


def _float32(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RosPolygonConversionError(f"{path} must be a number")

    numeric = float(value)
    if not math.isfinite(numeric):
        raise RosPolygonConversionError(f"{path} must be finite")

    try:
        encoded = struct.pack("!f", numeric)
    except (OverflowError, struct.error) as exc:
        raise RosPolygonConversionError(
            f"{path} is outside the geometry_msgs/Point32 range"
        ) from exc

    converted = struct.unpack("!f", encoded)[0]
    if not math.isfinite(converted):
        raise RosPolygonConversionError(
            f"{path} is outside the geometry_msgs/Point32 range"
        )
    return converted


def _copy_stamp(message: Any, stamp: Any) -> None:
    if stamp is None:
        return

    if not hasattr(stamp, "sec") or not hasattr(stamp, "nanosec"):
        raise RosPolygonConversionError(
            "stamp must provide integer sec and nanosec fields"
        )

    sec = stamp.sec
    nanosec = stamp.nanosec
    if isinstance(sec, bool) or not isinstance(sec, int):
        raise RosPolygonConversionError("stamp.sec must be an integer")
    if isinstance(nanosec, bool) or not isinstance(nanosec, int):
        raise RosPolygonConversionError("stamp.nanosec must be an integer")
    if nanosec < 0 or nanosec >= 1_000_000_000:
        raise RosPolygonConversionError(
            "stamp.nanosec must be in [0, 1000000000)"
        )

    try:
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = nanosec
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise RosPolygonConversionError(
            "stamp cannot be represented by the ROS header"
        ) from exc


def _point_message(
    x: float,
    y: float,
    altitude_m: float,
    message_types: RosPolygonMessageTypes,
    path: str,
) -> Any:
    point = message_types.point32()
    point.x = _float32(x, f"{path}.x")
    point.y = _float32(y, f"{path}.y")
    point.z = altitude_m
    return point


def component_to_polygon_with_holes_stamped(
    component: PreparedComponent,
    *,
    altitude_m: float = 0.0,
    stamp: Optional[Any] = None,
    message_types: Optional[RosPolygonMessageTypes] = None,
) -> Any:
    """Build one PolygonWithHolesStamped from one prepared component.

    The output contains the component's local metric x/y coordinates and the
    caller-supplied altitude.  Rings remain open (the first vertex is not
    duplicated), the hull is counterclockwise, and holes are clockwise because
    PreparedComponent canonicalizes its polygon before this conversion.
    """
    if not isinstance(component, PreparedComponent):
        raise RosPolygonConversionError(
            "component must be a PreparedComponent"
        )

    altitude = _float32(altitude_m, "altitude_m")
    types = message_types or load_ros_message_types()

    try:
        message = types.polygon_with_holes_stamped()
        message.header.frame_id = component.frame.frame_id
        _copy_stamp(message, stamp)

        hull_points = []
        for index, coordinate in enumerate(
            list(component.polygon.exterior.coords)[:-1]
        ):
            hull_points.append(
                _point_message(
                    coordinate[0],
                    coordinate[1],
                    altitude,
                    types,
                    f"polygon.hull[{index}]",
                )
            )
        message.polygon.hull.points = hull_points

        holes = []
        for hole_index, interior in enumerate(component.polygon.interiors):
            hole = types.polygon()
            hole.points = [
                _point_message(
                    coordinate[0],
                    coordinate[1],
                    altitude,
                    types,
                    f"polygon.holes[{hole_index}][{point_index}]",
                )
                for point_index, coordinate in enumerate(
                    list(interior.coords)[:-1]
                )
            ]
            holes.append(hole)
        message.polygon.holes = holes
    except RosPolygonConversionError:
        raise
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise RosPolygonConversionError(
            "the supplied ROS message classes do not match the expected "
            "PolygonWithHolesStamped contract"
        ) from exc

    return message
