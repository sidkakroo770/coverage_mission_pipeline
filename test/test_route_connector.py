#!/usr/bin/env python3
"""Tests for safe visibility-graph route connectors."""

from dataclasses import replace
import math

import pytest
from shapely.geometry import GeometryCollection, LineString, MultiPolygon, Point, Polygon

from coverage_mission_pipeline.planning_request import LocalPoint2D
from coverage_mission_pipeline.planning_result import CoverageWaypoint
from coverage_mission_pipeline.prepared_component import LocalCartesianFrame
from coverage_mission_pipeline.route_connector import (
    DIRECT_CONNECTOR_ALGORITHM,
    TRIVIAL_CONNECTOR_ALGORITHM,
    VISIBILITY_ASTAR_ALGORITHM,
    ConnectedRouteSequence,
    ConnectorPath,
    ConnectorPlannerConfig,
    ConnectorPlanningError,
    RouteConnector,
    connect_ordered_route_records,
    plan_connector,
)
from coverage_mission_pipeline.route_record import CoverageRouteRecord


@pytest.fixture
def simple_free_space() -> Polygon:
    return Polygon([(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)])


@pytest.fixture
def hole_free_space() -> Polygon:
    return Polygon(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)],
        [[(8.0, 1.0), (12.0, 1.0), (12.0, 7.0), (8.0, 7.0)]],
    )


@pytest.fixture
def frame() -> LocalCartesianFrame:
    return LocalCartesianFrame("map", "EPSG:32643", 631285.61, 3358862.37)


def make_route(
    frame: LocalCartesianFrame,
    request_id: str,
    component_id: str,
    points: list[tuple[float, float, float]],
    *,
    vehicle_id: str | None = "drone-1",
) -> CoverageRouteRecord:
    return CoverageRouteRecord(
        request_id=request_id,
        component_id=component_id,
        source_region_id=f"region-{component_id}",
        assigned_vehicle_id=vehicle_id,
        frame=frame,
        response_message="ok",
        waypoints=tuple(CoverageWaypoint(*point) for point in points),
    )


def assert_path_is_covered(free_space: Polygon, path: ConnectorPath) -> None:
    for left, right in zip(path.waypoints, path.waypoints[1:]):
        assert free_space.covers(
            LineString([(left.x_m, left.y_m), (right.x_m, right.y_m)])
        )


def test_direct_segment_fast_path(simple_free_space) -> None:
    path = plan_connector(
        simple_free_space,
        LocalPoint2D(1.0, 2.0),
        LocalPoint2D(19.0, 8.0),
    )
    assert path.algorithm == DIRECT_CONNECTOR_ALGORITHM
    assert path.is_direct
    assert path.waypoints == (LocalPoint2D(1.0, 2.0), LocalPoint2D(19.0, 8.0))
    assert path.length_m == pytest.approx(math.hypot(18.0, 6.0))


def test_same_point_connector(simple_free_space) -> None:
    point = LocalPoint2D(3.0, 4.0)
    path = plan_connector(simple_free_space, point, point)
    assert path.algorithm == TRIVIAL_CONNECTOR_ALGORITHM
    assert path.is_direct
    assert path.waypoints == (point,)
    assert path.length_m == 0.0


def test_direct_segment_on_outer_boundary_is_allowed(simple_free_space) -> None:
    path = plan_connector(
        simple_free_space,
        LocalPoint2D(0.0, 0.0),
        LocalPoint2D(20.0, 0.0),
    )
    assert path.algorithm == DIRECT_CONNECTOR_ALGORITHM


def test_direct_segment_on_hole_boundary_is_allowed(hole_free_space) -> None:
    path = plan_connector(
        hole_free_space,
        LocalPoint2D(8.0, 1.0),
        LocalPoint2D(12.0, 1.0),
    )
    assert path.algorithm == DIRECT_CONNECTOR_ALGORITHM


def test_blocked_segment_uses_visibility_astar(hole_free_space) -> None:
    path = plan_connector(
        hole_free_space,
        LocalPoint2D(2.0, 5.0),
        LocalPoint2D(18.0, 5.0),
    )
    assert path.algorithm == VISIBILITY_ASTAR_ALGORITHM
    assert not path.is_direct
    assert len(path.waypoints) == 4
    assert_path_is_covered(hole_free_space, path)


def test_shorter_side_of_single_exclusion_is_selected(hole_free_space) -> None:
    path = plan_connector(
        hole_free_space,
        LocalPoint2D(2.0, 5.0),
        LocalPoint2D(18.0, 5.0),
    )
    intermediate = path.waypoints[1:-1]
    assert tuple((point.x_m, point.y_m) for point in intermediate) == (
        (8.0, 7.0),
        (12.0, 7.0),
    )
    expected = 2.0 * math.hypot(6.0, 2.0) + 4.0
    assert path.length_m == pytest.approx(expected)


def test_equal_sides_have_deterministic_lexicographic_tie_break() -> None:
    free_space = Polygon(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)],
        [[(8.0, 3.0), (12.0, 3.0), (12.0, 7.0), (8.0, 7.0)]],
    )
    expected = None
    for _ in range(5):
        path = plan_connector(
            free_space,
            LocalPoint2D(2.0, 5.0),
            LocalPoint2D(18.0, 5.0),
        )
        coordinates = tuple((point.x_m, point.y_m) for point in path.waypoints)
        expected = coordinates if expected is None else expected
        assert coordinates == expected
    assert expected == ((2.0, 5.0), (8.0, 3.0), (12.0, 3.0), (18.0, 5.0))


def test_multiple_exclusions_are_handled_globally() -> None:
    free_space = Polygon(
        [(0.0, 0.0), (30.0, 0.0), (30.0, 12.0), (0.0, 12.0)],
        [
            [(7.0, 2.0), (11.0, 2.0), (11.0, 9.0), (7.0, 9.0)],
            [(18.0, 3.0), (22.0, 3.0), (22.0, 10.0), (18.0, 10.0)],
        ],
    )
    path = plan_connector(
        free_space,
        LocalPoint2D(2.0, 6.0),
        LocalPoint2D(28.0, 6.0),
    )
    assert path.algorithm == VISIBILITY_ASTAR_ALGORITHM
    assert len(path.waypoints) >= 4
    assert_path_is_covered(free_space, path)


def test_concave_outer_boundary_is_respected() -> None:
    free_space = Polygon(
        [
            (0.0, 0.0),
            (12.0, 0.0),
            (12.0, 12.0),
            (8.0, 12.0),
            (8.0, 4.0),
            (4.0, 4.0),
            (4.0, 12.0),
            (0.0, 12.0),
        ]
    )
    path = plan_connector(
        free_space,
        LocalPoint2D(2.0, 10.0),
        LocalPoint2D(10.0, 10.0),
    )
    assert path.algorithm == VISIBILITY_ASTAR_ALGORITHM
    assert_path_is_covered(free_space, path)
    assert any(point.y_m == 4.0 for point in path.waypoints)


def test_collinear_boundary_vertices_are_removed() -> None:
    free_space = Polygon(
        [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    )
    path = plan_connector(
        free_space,
        LocalPoint2D(0.0, 0.0),
        LocalPoint2D(10.0, 0.0),
    )
    assert path.waypoints == (LocalPoint2D(0.0, 0.0), LocalPoint2D(10.0, 0.0))


def test_start_outside_free_space_is_rejected(simple_free_space) -> None:
    with pytest.raises(ConnectorPlanningError, match="start is outside"):
        plan_connector(
            simple_free_space,
            LocalPoint2D(-1.0, 5.0),
            LocalPoint2D(10.0, 5.0),
        )


def test_goal_outside_free_space_is_rejected(simple_free_space) -> None:
    with pytest.raises(ConnectorPlanningError, match="goal is outside"):
        plan_connector(
            simple_free_space,
            LocalPoint2D(1.0, 5.0),
            LocalPoint2D(21.0, 5.0),
        )


def test_point_inside_hole_is_outside_free_space(hole_free_space) -> None:
    with pytest.raises(ConnectorPlanningError, match="goal is outside"):
        plan_connector(
            hole_free_space,
            LocalPoint2D(1.0, 1.0),
            LocalPoint2D(10.0, 4.0),
        )


def test_disconnected_multipolygon_is_rejected() -> None:
    free_space = MultiPolygon(
        [
            Polygon([(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]),
            Polygon([(10.0, 0.0), (14.0, 0.0), (14.0, 4.0), (10.0, 4.0)]),
        ]
    )
    with pytest.raises(ConnectorPlanningError, match="same connected"):
        plan_connector(
            free_space,
            LocalPoint2D(1.0, 1.0),
            LocalPoint2D(11.0, 1.0),
        )


def test_same_multipolygon_component_is_supported() -> None:
    free_space = MultiPolygon(
        [
            Polygon([(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]),
            Polygon([(10.0, 0.0), (14.0, 0.0), (14.0, 4.0), (10.0, 4.0)]),
        ]
    )
    path = plan_connector(
        free_space,
        LocalPoint2D(1.0, 1.0),
        LocalPoint2D(3.0, 3.0),
    )
    assert path.algorithm == DIRECT_CONNECTOR_ALGORITHM


def test_geometry_collection_preserves_polygonal_parts() -> None:
    free_space = GeometryCollection(
        [
            Polygon([(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]),
            LineString([(20.0, 20.0), (21.0, 21.0)]),
        ]
    )
    path = plan_connector(
        free_space,
        LocalPoint2D(1.0, 1.0),
        LocalPoint2D(4.0, 4.0),
    )
    assert path.algorithm == DIRECT_CONNECTOR_ALGORITHM


def test_empty_free_space_is_rejected() -> None:
    with pytest.raises(ConnectorPlanningError, match="must not be empty"):
        plan_connector(Polygon(), LocalPoint2D(0.0, 0.0), LocalPoint2D(0.0, 0.0))


def test_non_polygonal_free_space_is_rejected() -> None:
    with pytest.raises(ConnectorPlanningError, match="positive-area polygon"):
        plan_connector(
            LineString([(0.0, 0.0), (1.0, 1.0)]),
            LocalPoint2D(0.0, 0.0),
            LocalPoint2D(1.0, 1.0),
        )


def test_invalid_free_space_is_rejected() -> None:
    bowtie = Polygon([(0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)])
    with pytest.raises(ConnectorPlanningError, match="invalid"):
        plan_connector(bowtie, LocalPoint2D(0.1, 0.1), LocalPoint2D(1.9, 0.1))


def test_wrong_start_type_is_rejected(simple_free_space) -> None:
    with pytest.raises(ConnectorPlanningError, match="start"):
        plan_connector(simple_free_space, object(), LocalPoint2D(1.0, 1.0))


def test_wrong_goal_type_is_rejected(simple_free_space) -> None:
    with pytest.raises(ConnectorPlanningError, match="goal"):
        plan_connector(simple_free_space, LocalPoint2D(1.0, 1.0), object())


def test_wrong_config_type_is_rejected(simple_free_space) -> None:
    with pytest.raises(ConnectorPlanningError, match="config"):
        plan_connector(
            simple_free_space,
            LocalPoint2D(1.0, 1.0),
            LocalPoint2D(2.0, 2.0),
            config=object(),
        )


@pytest.mark.parametrize("value", [True, 0, 1, -3, 1.5, "10"])
def test_invalid_max_visibility_nodes(value) -> None:
    with pytest.raises(ConnectorPlanningError, match="max_visibility_nodes"):
        ConnectorPlannerConfig(value)


def test_visibility_node_limit_fails_closed(hole_free_space) -> None:
    with pytest.raises(ConnectorPlanningError, match="exceeding configured maximum"):
        plan_connector(
            hole_free_space,
            LocalPoint2D(2.0, 5.0),
            LocalPoint2D(18.0, 5.0),
            config=ConnectorPlannerConfig(max_visibility_nodes=4),
        )


def test_connector_path_rejects_bad_length() -> None:
    with pytest.raises(ConnectorPlanningError, match="does not match"):
        ConnectorPath(
            LocalPoint2D(0.0, 0.0),
            LocalPoint2D(1.0, 0.0),
            (LocalPoint2D(0.0, 0.0), LocalPoint2D(1.0, 0.0)),
            2.0,
            DIRECT_CONNECTOR_ALGORITHM,
        )


def test_connector_path_rejects_wrong_endpoints() -> None:
    with pytest.raises(ConnectorPlanningError, match="begin at start"):
        ConnectorPath(
            LocalPoint2D(0.0, 0.0),
            LocalPoint2D(1.0, 0.0),
            (LocalPoint2D(0.5, 0.0), LocalPoint2D(1.0, 0.0)),
            0.5,
            DIRECT_CONNECTOR_ALGORITHM,
        )


def test_connector_path_summary(simple_free_space) -> None:
    path = plan_connector(
        simple_free_space,
        LocalPoint2D(1.0, 1.0),
        LocalPoint2D(2.0, 1.0),
    )
    summary = path.to_summary_dict()
    assert summary["algorithm"] == DIRECT_CONNECTOR_ALGORITHM
    assert summary["waypoint_count"] == 2
    assert summary["waypoints"][0] == {"x_m": 1.0, "y_m": 1.0}


def test_route_connector_converts_to_constant_altitude(simple_free_space) -> None:
    path = plan_connector(
        simple_free_space,
        LocalPoint2D(1.0, 1.0),
        LocalPoint2D(3.0, 1.0),
    )
    connector = RouteConnector("a", "b", path, 30.0)
    assert connector.waypoints == (
        CoverageWaypoint(1.0, 1.0, 30.0),
        CoverageWaypoint(3.0, 1.0, 30.0),
    )


def test_route_connector_rejects_same_request_id(simple_free_space) -> None:
    path = plan_connector(
        simple_free_space,
        LocalPoint2D(1.0, 1.0),
        LocalPoint2D(3.0, 1.0),
    )
    with pytest.raises(ConnectorPlanningError, match="must differ"):
        RouteConnector("a", "a", path, 30.0)


def test_connect_single_route_has_no_connectors(frame, simple_free_space) -> None:
    route = make_route(frame, "request-a", "component-a", [(1, 1, 20), (4, 1, 20)])
    result = connect_ordered_route_records([route], simple_free_space)
    assert result.vehicle_id == "drone-1"
    assert result.connectors == ()
    assert result.waypoints == route.waypoints
    assert result.total_connector_length_m == 0.0


def test_connect_two_routes_with_direct_segment(frame, simple_free_space) -> None:
    first = make_route(frame, "request-a", "component-a", [(1, 1, 20), (4, 1, 20)])
    second = make_route(frame, "request-b", "component-b", [(10, 1, 20), (12, 1, 20)])
    result = connect_ordered_route_records([first, second], simple_free_space)
    assert len(result.connectors) == 1
    assert result.connectors[0].path.algorithm == DIRECT_CONNECTOR_ALGORITHM
    assert result.waypoints == (
        CoverageWaypoint(1, 1, 20),
        CoverageWaypoint(4, 1, 20),
        CoverageWaypoint(10, 1, 20),
        CoverageWaypoint(12, 1, 20),
    )


def test_connect_two_routes_around_exclusion(frame, hole_free_space) -> None:
    first = make_route(frame, "request-a", "component-a", [(2, 5, 20), (4, 5, 20)])
    second = make_route(frame, "request-b", "component-b", [(16, 5, 20), (18, 5, 20)])
    result = connect_ordered_route_records([first, second], hole_free_space)
    connector = result.connectors[0]
    assert connector.path.algorithm == VISIBILITY_ASTAR_ALGORITHM
    assert connector.path.waypoints[1:-1] == (
        LocalPoint2D(8.0, 7.0),
        LocalPoint2D(12.0, 7.0),
    )
    assert result.waypoints[0] == first.waypoints[0]
    assert result.waypoints[-1] == second.waypoints[-1]


def test_connect_three_routes_preserves_order(frame, simple_free_space) -> None:
    routes = [
        make_route(frame, "r-a", "c-a", [(1, 1, 10), (2, 1, 10)]),
        make_route(frame, "r-b", "c-b", [(5, 1, 10), (6, 1, 10)]),
        make_route(frame, "r-c", "c-c", [(9, 1, 10), (10, 1, 10)]),
    ]
    result = connect_ordered_route_records(routes, simple_free_space)
    assert [route.request_id for route in result.routes] == ["r-a", "r-b", "r-c"]
    assert [(c.from_request_id, c.to_request_id) for c in result.connectors] == [
        ("r-a", "r-b"),
        ("r-b", "r-c"),
    ]


def test_touching_route_endpoints_do_not_duplicate_waypoint(frame, simple_free_space) -> None:
    first = make_route(frame, "r-a", "c-a", [(1, 1, 10), (5, 1, 10)])
    second = make_route(frame, "r-b", "c-b", [(5, 1, 10), (9, 1, 10)])
    result = connect_ordered_route_records([first, second], simple_free_space)
    assert result.connectors[0].path.algorithm == TRIVIAL_CONNECTOR_ALGORITHM
    assert result.waypoints == (
        CoverageWaypoint(1, 1, 10),
        CoverageWaypoint(5, 1, 10),
        CoverageWaypoint(9, 1, 10),
    )


def test_route_waypoint_outside_free_space_is_rejected(frame, simple_free_space) -> None:
    route = make_route(frame, "r-a", "c-a", [(1, 1, 10), (25, 1, 10)])
    with pytest.raises(ConnectorPlanningError, match="waypoint 1"):
        connect_ordered_route_records([route], simple_free_space)


def test_route_segment_crossing_hole_is_rejected(frame, hole_free_space) -> None:
    route = make_route(frame, "r-a", "c-a", [(2, 5, 10), (18, 5, 10)])
    with pytest.raises(ConnectorPlanningError, match="segment 0"):
        connect_ordered_route_records([route], hole_free_space)


def test_route_altitude_mismatch_is_rejected(frame, simple_free_space) -> None:
    first = make_route(frame, "r-a", "c-a", [(1, 1, 10), (2, 1, 10)])
    second = make_route(frame, "r-b", "c-b", [(5, 1, 11), (6, 1, 11)])
    with pytest.raises(ConnectorPlanningError, match="common altitude"):
        connect_ordered_route_records([first, second], simple_free_space)


def test_intra_route_altitude_mismatch_is_rejected(frame, simple_free_space) -> None:
    route = make_route(frame, "r-a", "c-a", [(1, 1, 10), (2, 1, 11)])
    with pytest.raises(ConnectorPlanningError, match="common altitude"):
        connect_ordered_route_records([route], simple_free_space)


def test_frame_mismatch_is_rejected(frame, simple_free_space) -> None:
    other_frame = LocalCartesianFrame("map", "EPSG:32643", 1.0, 2.0)
    first = make_route(frame, "r-a", "c-a", [(1, 1, 10)])
    second = make_route(other_frame, "r-b", "c-b", [(2, 2, 10)])
    with pytest.raises(ConnectorPlanningError, match="same frame"):
        connect_ordered_route_records([first, second], simple_free_space)


def test_vehicle_mismatch_is_rejected(frame, simple_free_space) -> None:
    first = make_route(frame, "r-a", "c-a", [(1, 1, 10)], vehicle_id="drone-1")
    second = make_route(frame, "r-b", "c-b", [(2, 2, 10)], vehicle_id="drone-2")
    with pytest.raises(ConnectorPlanningError, match="same vehicle"):
        connect_ordered_route_records([first, second], simple_free_space)


def test_missing_vehicle_assignment_is_rejected(frame, simple_free_space) -> None:
    route = make_route(frame, "r-a", "c-a", [(1, 1, 10)], vehicle_id=None)
    with pytest.raises(ConnectorPlanningError, match="assigned vehicle"):
        connect_ordered_route_records([route], simple_free_space)


def test_duplicate_request_id_is_rejected(frame, simple_free_space) -> None:
    first = make_route(frame, "r-a", "c-a", [(1, 1, 10)])
    second = make_route(frame, "r-a", "c-b", [(2, 2, 10)])
    with pytest.raises(ConnectorPlanningError, match="request IDs"):
        connect_ordered_route_records([first, second], simple_free_space)


def test_duplicate_component_id_is_rejected(frame, simple_free_space) -> None:
    first = make_route(frame, "r-a", "c-a", [(1, 1, 10)])
    second = make_route(frame, "r-b", "c-a", [(2, 2, 10)])
    with pytest.raises(ConnectorPlanningError, match="component IDs"):
        connect_ordered_route_records([first, second], simple_free_space)


def test_empty_route_batch_is_rejected(simple_free_space) -> None:
    with pytest.raises(ConnectorPlanningError, match="must not be empty"):
        connect_ordered_route_records([], simple_free_space)


def test_non_iterable_route_batch_is_rejected(simple_free_space) -> None:
    with pytest.raises(ConnectorPlanningError, match="iterable"):
        connect_ordered_route_records(123, simple_free_space)


def test_wrong_route_item_is_rejected(simple_free_space) -> None:
    with pytest.raises(ConnectorPlanningError, match="CoverageRouteRecord"):
        connect_ordered_route_records([object()], simple_free_space)


def test_route_assembly_passes_config_to_connector(frame, hole_free_space) -> None:
    first = make_route(frame, "r-a", "c-a", [(2, 5, 10), (4, 5, 10)])
    second = make_route(frame, "r-b", "c-b", [(16, 5, 10), (18, 5, 10)])
    with pytest.raises(ConnectorPlanningError, match="exceeding configured maximum"):
        connect_ordered_route_records(
            [first, second],
            hole_free_space,
            config=ConnectorPlannerConfig(max_visibility_nodes=4),
        )


def test_connected_sequence_summary(frame, simple_free_space) -> None:
    first = make_route(frame, "r-a", "c-a", [(1, 1, 10), (2, 1, 10)])
    second = make_route(frame, "r-b", "c-b", [(5, 1, 10), (6, 1, 10)])
    result = connect_ordered_route_records([first, second], simple_free_space)
    summary = result.to_summary_dict()
    assert summary["vehicle_id"] == "drone-1"
    assert summary["route_request_ids"] == ["r-a", "r-b"]
    assert summary["connector_count"] == 1
    assert summary["total_connector_length_m"] == pytest.approx(3.0)


def test_connected_sequence_rejects_wrong_connector_count(frame, simple_free_space) -> None:
    route = make_route(frame, "r-a", "c-a", [(1, 1, 10)])
    with pytest.raises(ConnectorPlanningError, match="connector count"):
        ConnectedRouteSequence(
            vehicle_id="drone-1",
            frame=frame,
            routes=(route,),
            connectors=(
                RouteConnector(
                    "x",
                    "y",
                    plan_connector(
                        simple_free_space,
                        LocalPoint2D(1, 1),
                        LocalPoint2D(2, 1),
                    ),
                    10,
                ),
            ),
            waypoints=route.waypoints,
        )
