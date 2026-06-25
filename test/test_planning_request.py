#!/usr/bin/env python3
"""Tests for strict, ROS-independent coverage-planning inputs."""

import pytest
from shapely.geometry import Polygon, box

from coverage_mission_pipeline.planning_request import (
    CoveragePlanningRequest,
    LocalPoint2D,
    PlanningRequestError,
)
from coverage_mission_pipeline.prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
)


@pytest.fixture
def component() -> PreparedComponent:
    polygon = Polygon(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)],
        [[(8.0, 8.0), (8.0, 12.0), (12.0, 12.0), (12.0, 8.0)]],
    )
    return PreparedComponent(
        component_id="partition-1_component_1",
        source_region_id="partition-1",
        component_index=1,
        assigned_vehicle_id="drone-1",
        frame=LocalCartesianFrame("map", "EPSG:32643", 1.0, 2.0),
        polygon=polygon,
    )


def valid_request(component: PreparedComponent) -> CoveragePlanningRequest:
    return CoveragePlanningRequest(
        request_id="request-1",
        component=component,
        start=LocalPoint2D(1.0, 1.0),
        goal=LocalPoint2D(19.0, 19.0),
        altitude_m=30.0,
        lateral_footprint_m=2.5,
        lateral_overlap=0.2,
    )


def test_accepts_valid_request(component) -> None:
    request = valid_request(component)
    assert request.request_id == "request-1"
    assert request.lateral_footprint_m == 2.5
    assert request.lateral_overlap == 0.2


def test_factory_defaults_request_id_to_component_id(component) -> None:
    request = CoveragePlanningRequest.for_component(
        component,
        start=LocalPoint2D(1.0, 1.0),
        goal=LocalPoint2D(2.0, 2.0),
        altitude_m=10.0,
        lateral_footprint_m=1.0,
        lateral_overlap=0.0,
    )
    assert request.request_id == component.component_id


def test_factory_accepts_explicit_request_id(component) -> None:
    request = CoveragePlanningRequest.for_component(
        component,
        request_id="mission42-request7",
        start=LocalPoint2D(1.0, 1.0),
        goal=LocalPoint2D(2.0, 2.0),
        altitude_m=10.0,
        lateral_footprint_m=1.0,
        lateral_overlap=0.0,
    )
    assert request.request_id == "mission42-request7"


def test_factory_rejects_explicit_empty_request_id(component) -> None:
    with pytest.raises(PlanningRequestError, match="request_id"):
        CoveragePlanningRequest.for_component(
            component,
            request_id="",
            start=LocalPoint2D(1.0, 1.0),
            goal=LocalPoint2D(2.0, 2.0),
            altitude_m=10.0,
            lateral_footprint_m=1.0,
            lateral_overlap=0.0,
        )


def test_boundary_points_are_allowed(component) -> None:
    request = CoveragePlanningRequest(
        "request",
        component,
        LocalPoint2D(0.0, 10.0),
        LocalPoint2D(20.0, 20.0),
        10.0,
        1.0,
        0.0,
    )
    assert request.start == LocalPoint2D(0.0, 10.0)


def test_same_start_and_goal_are_allowed(component) -> None:
    point = LocalPoint2D(1.0, 1.0)
    request = CoveragePlanningRequest(
        "request", component, point, point, 10.0, 1.0, 0.0
    )
    assert request.start == request.goal


@pytest.mark.parametrize(
    "point,path",
    [
        (LocalPoint2D(-0.1, 1.0), "start"),
        (LocalPoint2D(20.1, 1.0), "start"),
        (LocalPoint2D(10.0, 10.0), "start"),
    ],
)
def test_rejects_start_outside_free_space(component, point, path) -> None:
    with pytest.raises(PlanningRequestError, match=path):
        CoveragePlanningRequest(
            "request",
            component,
            point,
            LocalPoint2D(1.0, 1.0),
            10.0,
            1.0,
            0.0,
        )


def test_rejects_goal_inside_hole(component) -> None:
    with pytest.raises(PlanningRequestError, match="goal"):
        CoveragePlanningRequest(
            "request",
            component,
            LocalPoint2D(1.0, 1.0),
            LocalPoint2D(10.0, 10.0),
            10.0,
            1.0,
            0.0,
        )


@pytest.mark.parametrize("value", [0.0, -1.0, float("nan"), float("inf"), True, "1"])
def test_rejects_invalid_footprint(component, value) -> None:
    with pytest.raises(PlanningRequestError, match="lateral_footprint_m"):
        CoveragePlanningRequest(
            "request",
            component,
            LocalPoint2D(1.0, 1.0),
            LocalPoint2D(2.0, 2.0),
            10.0,
            value,
            0.0,
        )


@pytest.mark.parametrize(
    "value", [-0.01, 1.0, 1.5, float("nan"), float("inf"), True, "0.1"]
)
def test_rejects_invalid_overlap(component, value) -> None:
    with pytest.raises(PlanningRequestError, match="lateral_overlap"):
        CoveragePlanningRequest(
            "request",
            component,
            LocalPoint2D(1.0, 1.0),
            LocalPoint2D(2.0, 2.0),
            10.0,
            1.0,
            value,
        )


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True, "10"])
def test_rejects_invalid_altitude(component, value) -> None:
    with pytest.raises(PlanningRequestError, match="altitude_m"):
        CoveragePlanningRequest(
            "request",
            component,
            LocalPoint2D(1.0, 1.0),
            LocalPoint2D(2.0, 2.0),
            value,
            1.0,
            0.0,
        )


@pytest.mark.parametrize("coordinate", [float("nan"), float("inf"), True, "1"])
def test_rejects_invalid_local_point(coordinate) -> None:
    with pytest.raises(PlanningRequestError, match="point.x_m"):
        LocalPoint2D(coordinate, 0.0)


def test_rejects_unsafe_request_id(component) -> None:
    with pytest.raises(PlanningRequestError, match="request_id"):
        CoveragePlanningRequest(
            "../request",
            component,
            LocalPoint2D(1.0, 1.0),
            LocalPoint2D(2.0, 2.0),
            10.0,
            1.0,
            0.0,
        )


def test_rejects_wrong_component_type() -> None:
    with pytest.raises(PlanningRequestError, match="PreparedComponent"):
        CoveragePlanningRequest(
            "request",
            object(),
            LocalPoint2D(1.0, 1.0),
            LocalPoint2D(2.0, 2.0),
            10.0,
            1.0,
            0.0,
        )


def test_rejects_wrong_start_type(component) -> None:
    with pytest.raises(PlanningRequestError, match="start"):
        CoveragePlanningRequest(
            "request", component, (1.0, 1.0), LocalPoint2D(2.0, 2.0),
            10.0, 1.0, 0.0,
        )


def test_summary_does_not_duplicate_polygon(component) -> None:
    summary = valid_request(component).to_summary_dict()
    assert summary["component_id"] == component.component_id
    assert summary["assigned_vehicle_id"] == "drone-1"
    assert summary["start"] == {"x_m": 1.0, "y_m": 1.0}
    assert "polygon" not in summary
