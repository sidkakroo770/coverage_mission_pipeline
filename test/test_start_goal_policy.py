#!/usr/bin/env python3
"""Tests for deterministic start/goal selection from explicit anchors."""

import math

import pytest
from shapely.geometry import Point, Polygon

from coverage_mission_pipeline.planning_request import (
    LocalPoint2D,
    PlanningRequestError,
)
from coverage_mission_pipeline.prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
)
from coverage_mission_pipeline.start_goal_policy import (
    StartGoalPolicyConfig,
    StartGoalPolicyError,
    StartGoalSelection,
    planning_request_from_anchors,
    select_start_goal,
)


@pytest.fixture
def component() -> PreparedComponent:
    polygon = Polygon(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)],
        [[(8.0, 8.0), (8.0, 12.0), (12.0, 12.0), (12.0, 8.0)]],
    )
    return PreparedComponent(
        "partition-1_component_1",
        "partition-1",
        1,
        LocalCartesianFrame("map", "EPSG:32643", 100.0, 200.0),
        polygon,
        "drone-1",
    )


def test_keeps_feasible_anchors_unchanged(component) -> None:
    start = LocalPoint2D(2.0, 3.0)
    goal = LocalPoint2D(18.0, 17.0)
    result = select_start_goal(
        component,
        start_anchor=start,
        goal_anchor=goal,
    )
    assert result.start is start
    assert result.goal is goal
    assert result.start_projection_distance_m == 0.0
    assert result.goal_projection_distance_m == 0.0


def test_projects_external_start_to_nearest_hull_boundary(component) -> None:
    result = select_start_goal(
        component,
        start_anchor=LocalPoint2D(-5.0, 7.0),
        goal_anchor=LocalPoint2D(10.0, 18.0),
    )
    assert result.start.x_m == pytest.approx(0.0)
    assert result.start.y_m == pytest.approx(7.0)
    assert result.start_projection_distance_m == pytest.approx(5.0)


def test_projects_external_goal_to_nearest_hull_boundary(component) -> None:
    result = select_start_goal(
        component,
        start_anchor=LocalPoint2D(1.0, 1.0),
        goal_anchor=LocalPoint2D(25.0, 6.0),
    )
    assert result.goal.x_m == pytest.approx(20.0)
    assert result.goal.y_m == pytest.approx(6.0)
    assert result.goal_projection_distance_m == pytest.approx(5.0)


def test_anchor_inside_hole_projects_to_hole_boundary(component) -> None:
    result = select_start_goal(
        component,
        start_anchor=LocalPoint2D(10.0, 10.0),
        goal_anchor=LocalPoint2D(2.0, 2.0),
    )
    assert component.polygon.covers(Point(result.start.x_m, result.start.y_m))
    assert result.start_projection_distance_m == pytest.approx(2.0)
    assert (
        result.start.x_m == pytest.approx(8.0)
        or result.start.y_m == pytest.approx(8.0)
        or result.start.x_m == pytest.approx(12.0)
        or result.start.y_m == pytest.approx(12.0)
    )


def test_zero_clearance_allows_original_boundary(component) -> None:
    result = select_start_goal(
        component,
        start_anchor=LocalPoint2D(0.0, 5.0),
        goal_anchor=LocalPoint2D(20.0, 5.0),
    )
    assert result.start == LocalPoint2D(0.0, 5.0)
    assert result.goal == LocalPoint2D(20.0, 5.0)


def test_positive_clearance_moves_hull_points_inward(component) -> None:
    result = select_start_goal(
        component,
        start_anchor=LocalPoint2D(-5.0, 5.0),
        goal_anchor=LocalPoint2D(25.0, 5.0),
        config=StartGoalPolicyConfig(boundary_clearance_m=2.0),
    )
    assert result.start == LocalPoint2D(2.0, 5.0)
    assert result.goal == LocalPoint2D(18.0, 5.0)


def test_positive_clearance_expands_hole_avoidance(component) -> None:
    result = select_start_goal(
        component,
        start_anchor=LocalPoint2D(10.0, 10.0),
        goal_anchor=LocalPoint2D(2.0, 2.0),
        config=StartGoalPolicyConfig(boundary_clearance_m=1.0),
    )
    assert result.start_projection_distance_m == pytest.approx(3.0)


def test_anchor_inside_clearanced_space_remains_unchanged(component) -> None:
    anchor = LocalPoint2D(3.0, 3.0)
    result = select_start_goal(
        component,
        start_anchor=anchor,
        goal_anchor=LocalPoint2D(17.0, 17.0),
        config=StartGoalPolicyConfig(boundary_clearance_m=2.0),
    )
    assert result.start is anchor


def test_clearance_that_removes_all_space_fails(component) -> None:
    with pytest.raises(StartGoalPolicyError, match="no positive-area"):
        select_start_goal(
            component,
            start_anchor=LocalPoint2D(1.0, 1.0),
            goal_anchor=LocalPoint2D(2.0, 2.0),
            config=StartGoalPolicyConfig(boundary_clearance_m=20.0),
        )


def test_clearance_can_split_space_and_nearest_piece_is_used() -> None:
    polygon = Polygon(
        [
            (0.0, 0.0),
            (8.0, 0.0),
            (8.0, 4.0),
            (12.0, 4.0),
            (12.0, 0.0),
            (20.0, 0.0),
            (20.0, 10.0),
            (12.0, 10.0),
            (12.0, 6.0),
            (8.0, 6.0),
            (8.0, 10.0),
            (0.0, 10.0),
        ]
    )
    prepared = PreparedComponent(
        "dumbbell_component_1",
        "dumbbell",
        1,
        LocalCartesianFrame("map", "EPSG:32643", 0.0, 0.0),
        polygon,
    )
    result = select_start_goal(
        prepared,
        start_anchor=LocalPoint2D(-1.0, 5.0),
        goal_anchor=LocalPoint2D(21.0, 5.0),
        config=StartGoalPolicyConfig(boundary_clearance_m=1.1),
    )
    assert result.start.x_m < 8.0
    assert result.goal.x_m > 12.0


def test_selected_points_are_valid_for_original_component(component) -> None:
    result = select_start_goal(
        component,
        start_anchor=LocalPoint2D(-100.0, -100.0),
        goal_anchor=LocalPoint2D(100.0, 100.0),
        config=StartGoalPolicyConfig(boundary_clearance_m=1.0),
    )
    assert component.polygon.covers(Point(result.start.x_m, result.start.y_m))
    assert component.polygon.covers(Point(result.goal.x_m, result.goal.y_m))


def test_same_selected_start_and_goal_allowed_by_default(component) -> None:
    result = select_start_goal(
        component,
        start_anchor=LocalPoint2D(-1.0, 10.0),
        goal_anchor=LocalPoint2D(-1.0, 10.0),
    )
    assert result.start == result.goal


def test_minimum_start_goal_separation_is_enforced(component) -> None:
    with pytest.raises(StartGoalPolicyError, match="below required minimum"):
        select_start_goal(
            component,
            start_anchor=LocalPoint2D(1.0, 1.0),
            goal_anchor=LocalPoint2D(1.5, 1.5),
            config=StartGoalPolicyConfig(
                minimum_start_goal_separation_m=2.0,
            ),
        )


def test_minimum_start_goal_separation_accepts_exact_threshold(component) -> None:
    result = select_start_goal(
        component,
        start_anchor=LocalPoint2D(1.0, 1.0),
        goal_anchor=LocalPoint2D(4.0, 5.0),
        config=StartGoalPolicyConfig(
            minimum_start_goal_separation_m=5.0,
        ),
    )
    assert math.hypot(
        result.start.x_m - result.goal.x_m,
        result.start.y_m - result.goal.y_m,
    ) == pytest.approx(5.0)


@pytest.mark.parametrize("value", [-1.0, float("inf"), float("nan"), True, "1"])
def test_invalid_boundary_clearance_rejected(value) -> None:
    with pytest.raises(StartGoalPolicyError):
        StartGoalPolicyConfig(boundary_clearance_m=value)


@pytest.mark.parametrize("value", [-1.0, float("inf"), float("nan"), True, "1"])
def test_invalid_minimum_separation_rejected(value) -> None:
    with pytest.raises(StartGoalPolicyError):
        StartGoalPolicyConfig(minimum_start_goal_separation_m=value)


def test_rejects_wrong_component_type(component) -> None:
    with pytest.raises(StartGoalPolicyError, match="PreparedComponent"):
        select_start_goal(
            object(),
            start_anchor=LocalPoint2D(1.0, 1.0),
            goal_anchor=LocalPoint2D(2.0, 2.0),
        )


def test_rejects_wrong_start_anchor_type(component) -> None:
    with pytest.raises(StartGoalPolicyError, match="start_anchor"):
        select_start_goal(
            component,
            start_anchor=(1.0, 1.0),
            goal_anchor=LocalPoint2D(2.0, 2.0),
        )


def test_rejects_wrong_goal_anchor_type(component) -> None:
    with pytest.raises(StartGoalPolicyError, match="goal_anchor"):
        select_start_goal(
            component,
            start_anchor=LocalPoint2D(1.0, 1.0),
            goal_anchor=(2.0, 2.0),
        )


def test_rejects_wrong_config_type(component) -> None:
    with pytest.raises(StartGoalPolicyError, match="StartGoalPolicyConfig"):
        select_start_goal(
            component,
            start_anchor=LocalPoint2D(1.0, 1.0),
            goal_anchor=LocalPoint2D(2.0, 2.0),
            config=object(),
        )


def test_selection_summary_is_deterministic(component) -> None:
    selection = select_start_goal(
        component,
        start_anchor=LocalPoint2D(-1.0, 4.0),
        goal_anchor=LocalPoint2D(21.0, 6.0),
    )
    assert selection.to_summary_dict() == {
        "component_id": "partition-1_component_1",
        "boundary_clearance_m": 0.0,
        "start_anchor": {"x_m": -1.0, "y_m": 4.0},
        "goal_anchor": {"x_m": 21.0, "y_m": 6.0},
        "start": {"x_m": 0.0, "y_m": 4.0},
        "goal": {"x_m": 20.0, "y_m": 6.0},
        "start_projection_distance_m": 1.0,
        "goal_projection_distance_m": 1.0,
    }


def test_selection_model_validates_types() -> None:
    point = LocalPoint2D(0.0, 0.0)
    with pytest.raises(StartGoalPolicyError, match="start must"):
        StartGoalSelection(
            "component-1",
            point,
            point,
            object(),
            point,
            0.0,
            0.0,
            0.0,
        )


def test_planning_request_from_anchors_builds_valid_request(component) -> None:
    request, selection = planning_request_from_anchors(
        component,
        start_anchor=LocalPoint2D(-2.0, 5.0),
        goal_anchor=LocalPoint2D(22.0, 5.0),
        altitude_m=30.0,
        lateral_footprint_m=2.5,
        lateral_overlap=0.15,
        request_id="request-1",
        policy_config=StartGoalPolicyConfig(boundary_clearance_m=1.0),
    )
    assert request.request_id == "request-1"
    assert request.start == selection.start == LocalPoint2D(1.0, 5.0)
    assert request.goal == selection.goal == LocalPoint2D(19.0, 5.0)
    assert request.altitude_m == 30.0
    assert request.lateral_footprint_m == 2.5
    assert request.lateral_overlap == 0.15


def test_planning_request_from_anchors_uses_component_id_by_default(component) -> None:
    request, _ = planning_request_from_anchors(
        component,
        start_anchor=LocalPoint2D(1.0, 1.0),
        goal_anchor=LocalPoint2D(19.0, 19.0),
        altitude_m=10.0,
        lateral_footprint_m=1.0,
        lateral_overlap=0.1,
    )
    assert request.request_id == component.component_id


def test_invalid_planning_parameters_still_fail_closed(component) -> None:
    with pytest.raises(PlanningRequestError, match="lateral_footprint"):
        planning_request_from_anchors(
            component,
            start_anchor=LocalPoint2D(1.0, 1.0),
            goal_anchor=LocalPoint2D(19.0, 19.0),
            altitude_m=10.0,
            lateral_footprint_m=0.0,
            lateral_overlap=0.1,
        )
