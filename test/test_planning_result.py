#!/usr/bin/env python3
"""Tests for detached, validated PlanCoverage results."""

import math

import pytest
from shapely.geometry import Polygon

from coverage_mission_pipeline.planning_request import (
    CoveragePlanningRequest,
    LocalPoint2D,
)
from coverage_mission_pipeline.planning_result import (
    CoveragePlanningResult,
    CoverageWaypoint,
    PlanningResultError,
    PlanningServiceRejectedError,
    plan_coverage_response_to_result,
)
from coverage_mission_pipeline.prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
)


class FakeHeader:
    def __init__(self, frame_id="map"):
        self.frame_id = frame_id


class FakePosition:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class FakePose:
    def __init__(self, x, y, z):
        self.position = FakePosition(x, y, z)


class FakePoseArray:
    def __init__(self, poses, frame_id="map"):
        self.header = FakeHeader(frame_id)
        self.poses = poses


class FakeResponse:
    def __init__(self, success=True, message="planned 2 waypoints", poses=None):
        self.success = success
        self.message = message
        self.waypoints = FakePoseArray(
            [FakePose(1.0, 2.0, 5.0), FakePose(3.0, 4.0, 5.0)]
            if poses is None
            else poses
        )


@pytest.fixture
def planning_request():
    component = PreparedComponent(
        "component-1",
        "region-1",
        1,
        LocalCartesianFrame("map", "EPSG:32643", 10.0, 20.0),
        Polygon([(0, 0), (10, 0), (10, 10), (0, 10)]),
        "drone-1",
    )
    return CoveragePlanningRequest(
        "request-1",
        component,
        LocalPoint2D(1, 1),
        LocalPoint2D(9, 1),
        5.0,
        1.0,
        0.1,
    )


def test_waypoint_accepts_finite_numbers():
    point = CoverageWaypoint(1, 2.5, -3)
    assert point == CoverageWaypoint(1.0, 2.5, -3.0)


@pytest.mark.parametrize("value", [math.inf, -math.inf, math.nan, True, "1"])
def test_waypoint_rejects_invalid_coordinates(value):
    with pytest.raises(PlanningResultError):
        CoverageWaypoint(value, 1.0, 2.0)


def test_result_requires_nonempty_waypoints(planning_request):
    with pytest.raises(PlanningResultError, match="must not be empty"):
        CoveragePlanningResult.from_request(
            planning_request,
            response_message="ok",
            waypoints=[],
        )


def test_converts_successful_response(planning_request):
    result = plan_coverage_response_to_result(planning_request, FakeResponse())
    assert result.request_id == "request-1"
    assert result.component_id == "component-1"
    assert result.source_region_id == "region-1"
    assert result.assigned_vehicle_id == "drone-1"
    assert result.frame_id == "map"
    assert len(result.waypoints) == 2
    assert result.waypoints[1] == CoverageWaypoint(3.0, 4.0, 5.0)


def test_summary_contains_route_endpoints(planning_request):
    result = plan_coverage_response_to_result(planning_request, FakeResponse())
    summary = result.to_summary_dict()
    assert summary["waypoint_count"] == 2
    assert summary["first_waypoint"] == {"x_m": 1.0, "y_m": 2.0, "z_m": 5.0}
    assert summary["last_waypoint"] == {"x_m": 3.0, "y_m": 4.0, "z_m": 5.0}


def test_rejects_wrong_request_type():
    with pytest.raises(PlanningResultError, match="CoveragePlanningRequest"):
        plan_coverage_response_to_result(object(), FakeResponse())


def test_rejects_explicit_planner_failure(planning_request):
    with pytest.raises(PlanningServiceRejectedError, match="no route"):
        plan_coverage_response_to_result(
            planning_request,
            FakeResponse(False, "no route"),
        )


def test_rejects_failure_without_diagnostic(planning_request):
    with pytest.raises(PlanningServiceRejectedError, match="no diagnostic"):
        plan_coverage_response_to_result(planning_request, FakeResponse(False, ""))


def test_rejects_missing_response_contract(planning_request):
    with pytest.raises(PlanningResultError, match="contract"):
        plan_coverage_response_to_result(planning_request, object())


def test_rejects_non_bool_success(planning_request):
    response = FakeResponse()
    response.success = 1
    with pytest.raises(PlanningResultError, match="bool"):
        plan_coverage_response_to_result(planning_request, response)


def test_rejects_non_string_message(planning_request):
    response = FakeResponse()
    response.message = None
    with pytest.raises(PlanningResultError, match="string"):
        plan_coverage_response_to_result(planning_request, response)


def test_rejects_wrong_frame(planning_request):
    response = FakeResponse()
    response.waypoints.header.frame_id = "odom"
    with pytest.raises(PlanningResultError, match="frame mismatch"):
        plan_coverage_response_to_result(planning_request, response)


def test_rejects_empty_waypoint_response(planning_request):
    with pytest.raises(PlanningResultError, match="no waypoints"):
        plan_coverage_response_to_result(planning_request, FakeResponse(poses=[]))


def test_rejects_non_iterable_poses(planning_request):
    response = FakeResponse()
    response.waypoints.poses = None
    with pytest.raises(PlanningResultError, match="iterable"):
        plan_coverage_response_to_result(planning_request, response)


def test_rejects_pose_without_position(planning_request):
    with pytest.raises(PlanningResultError, match="waypoint 0"):
        plan_coverage_response_to_result(planning_request, FakeResponse(poses=[object()]))


@pytest.mark.parametrize("coordinate", [math.nan, math.inf, -math.inf])
def test_rejects_nonfinite_response_coordinate(planning_request, coordinate):
    poses = [FakePose(coordinate, 2.0, 5.0)]
    with pytest.raises(PlanningResultError, match="waypoint 0"):
        plan_coverage_response_to_result(planning_request, FakeResponse(poses=poses))


def test_rejects_inconsistent_altitudes(planning_request):
    poses = [FakePose(1, 1, 5.0), FakePose(2, 2, 5.01)]
    with pytest.raises(PlanningResultError, match="inconsistent altitude"):
        plan_coverage_response_to_result(planning_request, FakeResponse(poses=poses))


def test_accepts_tiny_altitude_rounding_difference(planning_request):
    poses = [FakePose(1, 1, 5.0), FakePose(2, 2, 5.0000005)]
    result = plan_coverage_response_to_result(planning_request, FakeResponse(poses=poses))
    assert len(result.waypoints) == 2
