#!/usr/bin/env python3
"""Tests for prepared-component to ROS polygon conversion."""

from dataclasses import dataclass
import math

import pytest
from shapely.geometry import Polygon, box

from coverage_mission_pipeline.prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
)
from coverage_mission_pipeline.ros_polygon_conversion import (
    RosPolygonConversionError,
    RosPolygonMessageTypes,
    component_to_polygon_with_holes_stamped,
    load_ros_message_types,
)


class FakeTime:
    def __init__(self) -> None:
        self.sec = 0
        self.nanosec = 0


class FakeHeader:
    def __init__(self) -> None:
        self.frame_id = ""
        self.stamp = FakeTime()


class FakePoint32:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class FakePolygon:
    def __init__(self) -> None:
        self.points = []


class FakePolygonWithHoles:
    def __init__(self) -> None:
        self.hull = FakePolygon()
        self.holes = []


class FakePolygonWithHolesStamped:
    def __init__(self) -> None:
        self.header = FakeHeader()
        self.polygon = FakePolygonWithHoles()


@dataclass
class InputStamp:
    sec: int
    nanosec: int


@pytest.fixture
def message_types() -> RosPolygonMessageTypes:
    return RosPolygonMessageTypes(
        point32=FakePoint32,
        polygon=FakePolygon,
        polygon_with_holes_stamped=FakePolygonWithHolesStamped,
    )


@pytest.fixture
def frame() -> LocalCartesianFrame:
    return LocalCartesianFrame(
        frame_id="map",
        projected_crs="EPSG:32643",
        origin_easting_m=631285.61,
        origin_northing_m=3358862.37,
    )


def component(frame: LocalCartesianFrame, polygon=None) -> PreparedComponent:
    return PreparedComponent(
        component_id="partition-1_component_1",
        source_region_id="partition-1",
        component_index=1,
        assigned_vehicle_id="drone-1",
        frame=frame,
        polygon=polygon or box(10.0, 20.0, 50.0, 80.0),
    )


def signed_area(points) -> float:
    return 0.5 * sum(
        left.x * right.y - right.x * left.y
        for left, right in zip(points, points[1:] + points[:1])
    )


def test_converts_hull_frame_and_altitude(frame, message_types) -> None:
    message = component_to_polygon_with_holes_stamped(
        component(frame),
        altitude_m=42.5,
        message_types=message_types,
    )

    assert message.header.frame_id == "map"
    assert len(message.polygon.hull.points) == 4
    assert [(point.x, point.y) for point in message.polygon.hull.points] == [
        (50.0, 20.0),
        (50.0, 80.0),
        (10.0, 80.0),
        (10.0, 20.0),
    ]
    assert all(point.z == 42.5 for point in message.polygon.hull.points)


def test_does_not_duplicate_closing_vertex(frame, message_types) -> None:
    message = component_to_polygon_with_holes_stamped(
        component(frame),
        message_types=message_types,
    )
    points = message.polygon.hull.points
    assert len(points) == 4
    assert (points[0].x, points[0].y) != (points[-1].x, points[-1].y)


def test_preserves_holes_and_ring_orientation(frame, message_types) -> None:
    polygon = Polygon(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)],
        [[(5.0, 5.0), (5.0, 10.0), (10.0, 10.0), (10.0, 5.0)]],
    )
    message = component_to_polygon_with_holes_stamped(
        component(frame, polygon),
        altitude_m=7.0,
        message_types=message_types,
    )

    assert len(message.polygon.holes) == 1
    assert len(message.polygon.holes[0].points) == 4
    assert signed_area(message.polygon.hull.points) > 0.0
    assert signed_area(message.polygon.holes[0].points) < 0.0
    assert all(point.z == 7.0 for point in message.polygon.holes[0].points)


def test_copies_stamp_fields(frame, message_types) -> None:
    message = component_to_polygon_with_holes_stamped(
        component(frame),
        stamp=InputStamp(sec=123, nanosec=456),
        message_types=message_types,
    )
    assert message.header.stamp.sec == 123
    assert message.header.stamp.nanosec == 456


@pytest.mark.parametrize("altitude", [float("nan"), float("inf"), True, "10"])
def test_rejects_invalid_altitude(frame, message_types, altitude) -> None:
    with pytest.raises(RosPolygonConversionError, match="altitude_m"):
        component_to_polygon_with_holes_stamped(
            component(frame),
            altitude_m=altitude,
            message_types=message_types,
        )


def test_rejects_point_outside_float32_range(frame, message_types) -> None:
    huge = Polygon(
        [(0.0, 0.0), (1.0e40, 0.0), (1.0e40, 1.0), (0.0, 1.0)]
    )
    with pytest.raises(RosPolygonConversionError, match="Point32 range"):
        component_to_polygon_with_holes_stamped(
            component(frame, huge),
            message_types=message_types,
        )


def test_rejects_wrong_component_type(message_types) -> None:
    with pytest.raises(RosPolygonConversionError, match="PreparedComponent"):
        component_to_polygon_with_holes_stamped(
            object(),
            message_types=message_types,
        )


@pytest.mark.parametrize(
    "stamp",
    [object(), InputStamp(1, -1), InputStamp(1, 1_000_000_000)],
)
def test_rejects_invalid_stamp(frame, message_types, stamp) -> None:
    with pytest.raises(RosPolygonConversionError, match="stamp"):
        component_to_polygon_with_holes_stamped(
            component(frame),
            stamp=stamp,
            message_types=message_types,
        )


def test_float32_conversion_remains_finite(frame, message_types) -> None:
    polygon = Polygon(
        [(0.1, 0.2), (2.3, 0.2), (2.3, 3.4), (0.1, 3.4)]
    )
    message = component_to_polygon_with_holes_stamped(
        component(frame, polygon),
        message_types=message_types,
    )
    assert all(
        math.isfinite(value)
        for point in message.polygon.hull.points
        for value in (point.x, point.y, point.z)
    )


def test_actual_ros_message_integration(frame) -> None:
    pytest.importorskip("geometry_msgs.msg")
    pytest.importorskip("polygon_coverage_msgs.msg")

    types = load_ros_message_types()
    message = component_to_polygon_with_holes_stamped(
        component(frame),
        altitude_m=12.0,
        message_types=types,
    )

    assert message.header.frame_id == "map"
    assert len(message.polygon.hull.points) == 4
    assert message.polygon.hull.points[0].z == pytest.approx(12.0)
