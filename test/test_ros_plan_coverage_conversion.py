#!/usr/bin/env python3
"""Tests for validated model to PlanCoverage.Request conversion."""

from dataclasses import dataclass

import pytest
from shapely.geometry import Polygon

from coverage_mission_pipeline.planning_request import (
    CoveragePlanningRequest,
    LocalPoint2D,
)
from coverage_mission_pipeline.prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
)
from coverage_mission_pipeline.ros_plan_coverage_conversion import (
    RosPlanCoverageConversionError,
    RosPlanCoverageMessageTypes,
    load_ros_plan_coverage_message_types,
    planning_request_to_ros,
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


class FakePoint:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class FakeQuaternion:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0


class FakePose:
    def __init__(self) -> None:
        self.position = FakePoint()
        self.orientation = FakeQuaternion()


class FakePoseStamped:
    def __init__(self) -> None:
        self.header = FakeHeader()
        self.pose = FakePose()


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


class FakePlanCoverageRequest:
    def __init__(self) -> None:
        self.polygon = None
        self.start_pose = None
        self.goal_pose = None
        self.lateral_footprint = 0.0
        self.lateral_overlap = 0.0


@dataclass
class InputStamp:
    sec: int
    nanosec: int


@pytest.fixture
def message_types() -> RosPlanCoverageMessageTypes:
    return RosPlanCoverageMessageTypes(
        point32=FakePoint32,
        polygon=FakePolygon,
        polygon_with_holes_stamped=FakePolygonWithHolesStamped,
        pose_stamped=FakePoseStamped,
        plan_coverage_request=FakePlanCoverageRequest,
    )


@pytest.fixture
def model() -> CoveragePlanningRequest:
    polygon = Polygon(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)],
        [[(8.0, 8.0), (8.0, 12.0), (12.0, 12.0), (12.0, 8.0)]],
    )
    component = PreparedComponent(
        "partition-1_component_1",
        "partition-1",
        1,
        LocalCartesianFrame("map", "EPSG:32643", 1.0, 2.0),
        polygon,
        "drone-1",
    )
    return CoveragePlanningRequest(
        "request-1",
        component,
        LocalPoint2D(1.25, 2.5),
        LocalPoint2D(18.0, 17.0),
        32.25,
        2.5,
        0.15,
    )


def test_builds_complete_request(model, message_types) -> None:
    message = planning_request_to_ros(model, message_types=message_types)

    assert message.polygon.header.frame_id == "map"
    assert len(message.polygon.polygon.hull.points) == 4
    assert len(message.polygon.polygon.holes) == 1
    assert message.start_pose.pose.position.x == 1.25
    assert message.start_pose.pose.position.y == 2.5
    assert message.goal_pose.pose.position.x == 18.0
    assert message.goal_pose.pose.position.y == 17.0
    assert message.lateral_footprint == 2.5
    assert message.lateral_overlap == 0.15


def test_uses_same_frame_for_polygon_start_and_goal(model, message_types) -> None:
    message = planning_request_to_ros(model, message_types=message_types)
    assert message.polygon.header.frame_id == "map"
    assert message.start_pose.header.frame_id == "map"
    assert message.goal_pose.header.frame_id == "map"


def test_copies_stamp_to_all_headers(model, message_types) -> None:
    message = planning_request_to_ros(
        model,
        stamp=InputStamp(123, 456),
        message_types=message_types,
    )
    headers = [
        message.polygon.header,
        message.start_pose.header,
        message.goal_pose.header,
    ]
    assert all(header.stamp.sec == 123 for header in headers)
    assert all(header.stamp.nanosec == 456 for header in headers)


def test_sets_identity_orientations(model, message_types) -> None:
    message = planning_request_to_ros(model, message_types=message_types)
    for pose in [message.start_pose.pose, message.goal_pose.pose]:
        assert pose.orientation.x == 0.0
        assert pose.orientation.y == 0.0
        assert pose.orientation.z == 0.0
        assert pose.orientation.w == 1.0


def test_uses_identical_represented_altitude(model, message_types) -> None:
    message = planning_request_to_ros(model, message_types=message_types)
    polygon_z = message.polygon.polygon.hull.points[0].z
    assert message.start_pose.pose.position.z == polygon_z
    assert message.goal_pose.pose.position.z == polygon_z


def test_rejects_wrong_model_type(message_types) -> None:
    with pytest.raises(RosPlanCoverageConversionError, match="CoveragePlanningRequest"):
        planning_request_to_ros(object(), message_types=message_types)


def test_rejects_altitude_outside_point32_range(model, message_types) -> None:
    invalid = CoveragePlanningRequest(
        model.request_id,
        model.component,
        model.start,
        model.goal,
        1.0e40,
        model.lateral_footprint_m,
        model.lateral_overlap,
    )
    with pytest.raises(RosPlanCoverageConversionError, match="Point32 range"):
        planning_request_to_ros(invalid, message_types=message_types)


def test_rejects_malformed_ros_classes(model, message_types) -> None:
    class BrokenRequest:
        pass

    broken = RosPlanCoverageMessageTypes(
        point32=message_types.point32,
        polygon=message_types.polygon,
        polygon_with_holes_stamped=message_types.polygon_with_holes_stamped,
        pose_stamped=message_types.pose_stamped,
        plan_coverage_request=BrokenRequest,
    )
    with pytest.raises(RosPlanCoverageConversionError, match="contract"):
        planning_request_to_ros(model, message_types=broken)


def test_actual_ros_request_integration(model) -> None:
    pytest.importorskip("geometry_msgs.msg")
    pytest.importorskip("polygon_coverage_msgs.srv")

    types = load_ros_plan_coverage_message_types()
    message = planning_request_to_ros(model, message_types=types)

    assert message.polygon.header.frame_id == "map"
    assert message.start_pose.pose.orientation.w == pytest.approx(1.0)
    assert message.goal_pose.pose.orientation.w == pytest.approx(1.0)
    assert message.lateral_footprint == pytest.approx(2.5)
    assert message.lateral_overlap == pytest.approx(0.15)
