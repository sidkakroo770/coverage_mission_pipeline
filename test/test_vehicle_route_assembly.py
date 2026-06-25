#!/usr/bin/env python3
"""Tests for complete per-vehicle route direction optimization and assembly."""

import math

import pytest
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points

from coverage_mission_pipeline.planning_request import LocalPoint2D
from coverage_mission_pipeline.planning_result import CoverageWaypoint
from coverage_mission_pipeline.prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
)
from coverage_mission_pipeline.route_connector import (
    ConnectorPlannerConfig,
    DIRECT_CONNECTOR_ALGORITHM,
    TRIVIAL_CONNECTOR_ALGORITHM,
    VISIBILITY_ASTAR_ALGORITHM,
)
from coverage_mission_pipeline.route_record import CoverageRouteRecord
from coverage_mission_pipeline.vehicle_component_ordering import (
    ComponentVisit,
    VehicleComponentPlan,
    VehicleReference,
    order_components_for_vehicle,
)
from coverage_mission_pipeline.vehicle_route_assembly import (
    FORWARD_ROUTE_DIRECTION,
    REVERSED_ROUTE_DIRECTION,
    ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM,
    CompleteVehicleRoute,
    OrientedRoute,
    VehicleRouteAssemblyConfig,
    VehicleRouteAssemblyError,
    assemble_vehicle_route,
)


@pytest.fixture
def frame() -> LocalCartesianFrame:
    return LocalCartesianFrame("map", "EPSG:32643", 631000.0, 3358000.0)


@pytest.fixture
def open_space() -> Polygon:
    return Polygon([(-20.0, -20.0), (120.0, -20.0), (120.0, 40.0), (-20.0, 40.0)])


def component(
    frame: LocalCartesianFrame,
    component_id: str,
    bounds: tuple[float, float, float, float],
    *,
    vehicle_id: str = "drone-1",
) -> PreparedComponent:
    min_x, min_y, max_x, max_y = bounds
    return PreparedComponent(
        component_id=component_id,
        source_region_id=f"region-{component_id}",
        component_index=1,
        frame=frame,
        polygon=Polygon(
            [
                (min_x, min_y),
                (max_x, min_y),
                (max_x, max_y),
                (min_x, max_y),
            ]
        ),
        assigned_vehicle_id=vehicle_id,
    )


def route(
    component_value: PreparedComponent,
    points: list[tuple[float, float]] | tuple[tuple[float, float], ...],
    *,
    request_id: str | None = None,
    altitude_m: float = 30.0,
) -> CoverageRouteRecord:
    return CoverageRouteRecord(
        request_id=request_id or f"request-{component_value.component_id}",
        component_id=component_value.component_id,
        source_region_id=component_value.source_region_id,
        assigned_vehicle_id=component_value.assigned_vehicle_id,
        frame=component_value.frame,
        response_message="planned",
        waypoints=tuple(
            CoverageWaypoint(x, y, altitude_m) for x, y in points
        ),
    )


def manual_plan(
    reference: VehicleReference,
    components: tuple[PreparedComponent, ...] | list[PreparedComponent],
) -> VehicleComponentPlan:
    visits = []
    source = Point(reference.position.x_m, reference.position.y_m)
    predecessor = None
    for index, value in enumerate(components, start=1):
        source_point, target_point = nearest_points(source, value.polygon)
        start = (
            reference.position
            if index == 1
            else LocalPoint2D(float(source_point.x), float(source_point.y))
        )
        end = LocalPoint2D(float(target_point.x), float(target_point.y))
        visits.append(
            ComponentVisit(
                visit_index=index,
                component=value,
                predecessor_component_id=predecessor,
                transition_start=start,
                transition_end=end,
                straight_line_lower_bound_m=math.hypot(
                    end.x_m - start.x_m,
                    end.y_m - start.y_m,
                ),
            )
        )
        source = value.polygon
        predecessor = value.component_id
    return VehicleComponentPlan(reference, tuple(visits))


def basic_single(
    frame: LocalCartesianFrame,
    *,
    home: tuple[float, float] = (0.0, 0.0),
    points: tuple[tuple[float, float], ...] = ((5.0, 0.0), (10.0, 0.0)),
):
    comp = component(frame, "component-a", (4.0, -1.0, 11.0, 1.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(*home), "home")
    plan = manual_plan(reference, [comp])
    return reference, comp, plan, route(comp, points)


def test_default_config() -> None:
    config = VehicleRouteAssemblyConfig()
    assert config.return_to_reference is False
    assert config.connector_config == ConnectorPlannerConfig()


def test_config_accepts_return_to_reference() -> None:
    config = VehicleRouteAssemblyConfig(return_to_reference=True)
    assert config.return_to_reference is True


@pytest.mark.parametrize("value", [1, 0, "false", None])
def test_config_rejects_non_bool_return(value) -> None:
    with pytest.raises(VehicleRouteAssemblyError, match="bool"):
        VehicleRouteAssemblyConfig(return_to_reference=value)


@pytest.mark.parametrize("value", [None, object(), 512])
def test_config_rejects_invalid_connector_config(value) -> None:
    with pytest.raises(VehicleRouteAssemblyError, match="ConnectorPlannerConfig"):
        VehicleRouteAssemblyConfig(connector_config=value)


def test_oriented_route_forward_preserves_waypoint_order(frame) -> None:
    _, _, _, source = basic_single(frame)
    oriented = OrientedRoute(source, FORWARD_ROUTE_DIRECTION)
    assert oriented.waypoints == source.waypoints


def test_oriented_route_reverse_reverses_waypoint_order(frame) -> None:
    _, _, _, source = basic_single(frame)
    oriented = OrientedRoute(source, REVERSED_ROUTE_DIRECTION)
    assert oriented.waypoints == tuple(reversed(source.waypoints))


def test_oriented_route_reverse_single_waypoint_is_stable(frame) -> None:
    _, comp, _, _ = basic_single(frame)
    source = route(comp, [(5.0, 0.0)])
    oriented = OrientedRoute(source, REVERSED_ROUTE_DIRECTION)
    assert oriented.waypoints == source.waypoints


def test_oriented_route_start_and_goal(frame) -> None:
    _, _, _, source = basic_single(frame)
    oriented = OrientedRoute(source, REVERSED_ROUTE_DIRECTION)
    assert oriented.start == LocalPoint2D(10.0, 0.0)
    assert oriented.goal == LocalPoint2D(5.0, 0.0)


def test_oriented_route_length_is_direction_invariant(frame) -> None:
    _, comp, _, _ = basic_single(frame)
    source = route(comp, [(5.0, 0.0), (7.0, 3.0), (10.0, 0.0)])
    forward = OrientedRoute(source, FORWARD_ROUTE_DIRECTION)
    reverse = OrientedRoute(source, REVERSED_ROUTE_DIRECTION)
    assert forward.route_length_m == pytest.approx(reverse.route_length_m)


def test_oriented_route_to_record_preserves_metadata(frame) -> None:
    _, _, _, source = basic_single(frame)
    record = OrientedRoute(source, REVERSED_ROUTE_DIRECTION).to_route_record()
    assert record.request_id == source.request_id
    assert record.component_id == source.component_id
    assert record.frame == source.frame
    assert record.waypoints == tuple(reversed(source.waypoints))


def test_oriented_route_summary(frame) -> None:
    _, _, _, source = basic_single(frame)
    summary = OrientedRoute(source, FORWARD_ROUTE_DIRECTION).to_summary_dict()
    assert summary["direction"] == "forward"
    assert summary["waypoint_count"] == 2
    assert summary["start"] == {"x_m": 5.0, "y_m": 0.0}


def test_oriented_route_rejects_wrong_source_type() -> None:
    with pytest.raises(VehicleRouteAssemblyError, match="CoverageRouteRecord"):
        OrientedRoute(object(), FORWARD_ROUTE_DIRECTION)


@pytest.mark.parametrize("direction", ["", "backward", "FORWARD", None])
def test_oriented_route_rejects_bad_direction(frame, direction) -> None:
    _, _, _, source = basic_single(frame)
    with pytest.raises(VehicleRouteAssemblyError, match="direction"):
        OrientedRoute(source, direction)


def test_idle_plan_produces_idle_complete_route(frame, open_space) -> None:
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.0, 0.0))
    plan = VehicleComponentPlan(reference, ())
    result = assemble_vehicle_route(plan, (), open_space)
    assert result.is_idle
    assert result.waypoints == ()
    assert result.reference_connector is None
    assert result.route_directions == ()


def test_idle_plan_rejects_supplied_route(frame, open_space) -> None:
    _, _, _, source = basic_single(frame)
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.0, 0.0))
    plan = VehicleComponentPlan(reference, ())
    with pytest.raises(VehicleRouteAssemblyError, match="exactly one"):
        assemble_vehicle_route(plan, (source,), open_space)


def test_one_route_selects_forward_when_forward_start_is_nearer(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    result = assemble_vehicle_route(plan, [source], open_space)
    assert result.route_directions == (FORWARD_ROUTE_DIRECTION,)


def test_one_route_selects_reverse_when_reverse_start_is_nearer(frame, open_space) -> None:
    _, _, plan, source = basic_single(
        frame,
        points=((10.0, 0.0), (5.0, 0.0)),
    )
    result = assemble_vehicle_route(plan, [source], open_space)
    assert result.route_directions == (REVERSED_ROUTE_DIRECTION,)
    assert result.oriented_routes[0].start == LocalPoint2D(5.0, 0.0)


def test_equal_cost_tie_prefers_forward(frame, open_space) -> None:
    _, comp, plan, _ = basic_single(
        frame,
        home=(7.5, 5.0),
        points=((5.0, 0.0), (10.0, 0.0)),
    )
    source = route(comp, [(5.0, 0.0), (10.0, 0.0)])
    result = assemble_vehicle_route(plan, [source], open_space)
    assert result.route_directions == (FORWARD_ROUTE_DIRECTION,)


def test_input_route_order_is_reordered_to_component_plan(frame, open_space) -> None:
    first = component(frame, "component-a", (5.0, 0.0, 7.0, 2.0))
    second = component(frame, "component-b", (15.0, 0.0, 17.0, 2.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.0, 1.0))
    plan = manual_plan(reference, [first, second])
    first_route = route(first, [(5.0, 1.0), (7.0, 1.0)])
    second_route = route(second, [(15.0, 1.0), (17.0, 1.0)])
    result = assemble_vehicle_route(plan, [second_route, first_route], open_space)
    assert tuple(item.source_route.component_id for item in result.oriented_routes) == (
        "component-a",
        "component-b",
    )


def test_reference_connector_direct(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    result = assemble_vehicle_route(plan, [source], open_space)
    assert result.reference_connector.path.algorithm == DIRECT_CONNECTOR_ALGORITHM
    assert result.reference_connector.path.start == LocalPoint2D(0.0, 0.0)


def test_reference_connector_trivial_when_home_equals_route_start(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame, home=(5.0, 0.0))
    result = assemble_vehicle_route(plan, [source], open_space)
    assert result.reference_connector.path.algorithm == TRIVIAL_CONNECTOR_ALGORITHM
    assert result.reference_connector.path.length_m == 0.0


def test_reference_connector_uses_astar_around_hole(frame) -> None:
    free_space = Polygon(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)],
        [[(8.0, 1.0), (12.0, 1.0), (12.0, 7.0), (8.0, 7.0)]],
    )
    comp = component(frame, "component-a", (16.0, 4.0, 19.0, 6.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(2.0, 5.0))
    plan = manual_plan(reference, [comp])
    source = route(comp, [(18.0, 5.0), (17.0, 5.0)])
    result = assemble_vehicle_route(plan, [source], free_space)
    assert result.reference_connector.path.algorithm == VISIBILITY_ASTAR_ALGORITHM


def test_two_routes_create_one_inter_route_connector(frame, open_space) -> None:
    first = component(frame, "component-a", (5.0, 0.0, 7.0, 2.0))
    second = component(frame, "component-b", (15.0, 0.0, 17.0, 2.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.0, 1.0))
    plan = manual_plan(reference, [first, second])
    result = assemble_vehicle_route(
        plan,
        [
            route(first, [(5.0, 1.0), (7.0, 1.0)]),
            route(second, [(15.0, 1.0), (17.0, 1.0)]),
        ],
        open_space,
    )
    assert len(result.inter_route_connectors) == 1
    assert result.inter_route_connectors[0].path.start == result.oriented_routes[0].goal
    assert result.inter_route_connectors[0].path.goal == result.oriented_routes[1].start


def test_assembled_waypoints_do_not_duplicate_joins(frame, open_space) -> None:
    first = component(frame, "component-a", (5.0, 0.0, 7.0, 2.0))
    second = component(frame, "component-b", (15.0, 0.0, 17.0, 2.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.0, 1.0))
    plan = manual_plan(reference, [first, second])
    result = assemble_vehicle_route(
        plan,
        [
            route(first, [(5.0, 1.0), (7.0, 1.0)]),
            route(second, [(15.0, 1.0), (17.0, 1.0)]),
        ],
        open_space,
    )
    pairs = list(zip(result.waypoints, result.waypoints[1:]))
    assert all(left != right for left, right in pairs)


def test_optional_return_connector(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    result = assemble_vehicle_route(
        plan,
        [source],
        open_space,
        config=VehicleRouteAssemblyConfig(return_to_reference=True),
    )
    assert result.return_connector is not None
    assert result.waypoints[-1].x_m == pytest.approx(0.0)
    assert result.waypoints[-1].y_m == pytest.approx(0.0)


def test_default_has_no_return_connector(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    result = assemble_vehicle_route(plan, [source], open_space)
    assert result.return_connector is None
    assert result.return_to_reference is False


def test_return_to_reference_single_route_is_symmetric_tie_forward(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    result = assemble_vehicle_route(
        plan,
        [source],
        open_space,
        config=VehicleRouteAssemblyConfig(return_to_reference=True),
    )
    assert result.route_directions == (FORWARD_ROUTE_DIRECTION,)


def test_global_dp_can_reverse_first_route_even_when_forward_start_is_nearer(frame, open_space) -> None:
    first = component(frame, "component-a", (0.0, -1.0, 10.0, 1.0))
    second = component(frame, "component-b", (-2.0, -1.0, -1.0, 1.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.1, 0.0))
    plan = manual_plan(reference, [first, second])
    first_route = route(first, [(0.0, 0.0), (10.0, 0.0)])
    second_route = route(second, [(-1.0, 0.0), (-2.0, 0.0)])
    result = assemble_vehicle_route(plan, [first_route, second_route], open_space)
    assert result.route_directions[0] == REVERSED_ROUTE_DIRECTION


def test_return_option_can_change_final_route_direction(frame, open_space) -> None:
    first = component(frame, "component-a", (-1.0, 9.0, 1.0, 11.0))
    second = component(frame, "component-b", (-1.1, -0.5, 1.1, 0.5))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(-1.0, 0.0))
    plan = manual_plan(reference, [first, second])
    first_route = route(first, [(0.0, 10.0), (-0.1, 10.0)])
    second_route = route(second, [(-1.0, 0.0), (1.0, 0.0)])

    no_return = assemble_vehicle_route(plan, [first_route, second_route], open_space)
    with_return = assemble_vehicle_route(
        plan,
        [first_route, second_route],
        open_space,
        config=VehicleRouteAssemblyConfig(return_to_reference=True),
    )
    assert no_return.route_directions[-1] == FORWARD_ROUTE_DIRECTION
    assert with_return.route_directions[-1] == REVERSED_ROUTE_DIRECTION


def test_three_route_orientation_signature_is_deterministic(frame, open_space) -> None:
    components = [
        component(frame, "component-a", (5.0, 0.0, 7.0, 2.0)),
        component(frame, "component-b", (15.0, 0.0, 17.0, 2.0)),
        component(frame, "component-c", (25.0, 0.0, 27.0, 2.0)),
    ]
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.0, 1.0))
    plan = manual_plan(reference, components)
    routes = [
        route(components[0], [(5.0, 1.0), (7.0, 1.0)]),
        route(components[1], [(15.0, 1.0), (17.0, 1.0)]),
        route(components[2], [(25.0, 1.0), (27.0, 1.0)]),
    ]
    first = assemble_vehicle_route(plan, routes, open_space)
    second = assemble_vehicle_route(plan, reversed(routes), open_space)
    assert first.route_directions == second.route_directions
    assert first.waypoints == second.waypoints


def test_total_route_length(frame, open_space) -> None:
    _, comp, plan, _ = basic_single(frame)
    source = route(comp, [(5.0, 0.0), (8.0, 4.0), (10.0, 4.0)])
    result = assemble_vehicle_route(plan, [source], open_space)
    assert result.total_route_length_m == pytest.approx(7.0)


def test_total_connector_length(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    result = assemble_vehicle_route(plan, [source], open_space)
    assert result.total_connector_length_m == pytest.approx(5.0)


def test_total_path_length_equals_route_plus_connectors(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    result = assemble_vehicle_route(plan, [source], open_space)
    assert result.total_path_length_m == pytest.approx(
        result.total_route_length_m + result.total_connector_length_m
    )


def test_summary_contains_algorithm_and_directions(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    result = assemble_vehicle_route(plan, [source], open_space)
    summary = result.to_summary_dict()
    assert summary["algorithm"] == ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM
    assert summary["route_directions"] == ["forward"]
    assert summary["route_count"] == 1


def test_route_records_property_uses_selected_orientation(frame, open_space) -> None:
    _, _, plan, source = basic_single(
        frame,
        points=((10.0, 0.0), (5.0, 0.0)),
    )
    result = assemble_vehicle_route(plan, [source], open_space)
    assert result.route_records[0].waypoints == tuple(reversed(source.waypoints))


def test_rejects_wrong_component_plan_type(open_space) -> None:
    with pytest.raises(VehicleRouteAssemblyError, match="VehicleComponentPlan"):
        assemble_vehicle_route(object(), (), open_space)


def test_rejects_wrong_config_type(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    with pytest.raises(VehicleRouteAssemblyError, match="VehicleRouteAssemblyConfig"):
        assemble_vehicle_route(plan, [source], open_space, config=object())


def test_rejects_non_iterable_routes(frame, open_space) -> None:
    _, _, plan, _ = basic_single(frame)
    with pytest.raises(VehicleRouteAssemblyError, match="iterable"):
        assemble_vehicle_route(plan, None, open_space)


def test_rejects_wrong_route_item_type(frame, open_space) -> None:
    _, _, plan, _ = basic_single(frame)
    with pytest.raises(VehicleRouteAssemblyError, match=r"routes\[0\]"):
        assemble_vehicle_route(plan, [object()], open_space)


def test_rejects_duplicate_route_component_ids(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    duplicate = CoverageRouteRecord(
        request_id="different-request",
        component_id=source.component_id,
        source_region_id=source.source_region_id,
        assigned_vehicle_id=source.assigned_vehicle_id,
        frame=source.frame,
        response_message=source.response_message,
        waypoints=source.waypoints,
    )
    with pytest.raises(VehicleRouteAssemblyError, match="component IDs"):
        assemble_vehicle_route(plan, [source, duplicate], open_space)


def test_rejects_duplicate_request_ids(frame, open_space) -> None:
    first = component(frame, "component-a", (5.0, 0.0, 7.0, 2.0))
    second = component(frame, "component-b", (15.0, 0.0, 17.0, 2.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.0, 1.0))
    plan = manual_plan(reference, [first, second])
    routes = [
        route(first, [(5.0, 1.0), (7.0, 1.0)], request_id="same"),
        route(second, [(15.0, 1.0), (17.0, 1.0)], request_id="same"),
    ]
    with pytest.raises(VehicleRouteAssemblyError, match="request IDs"):
        assemble_vehicle_route(plan, routes, open_space)


def test_rejects_missing_component_route(frame, open_space) -> None:
    first = component(frame, "component-a", (5.0, 0.0, 7.0, 2.0))
    second = component(frame, "component-b", (15.0, 0.0, 17.0, 2.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.0, 1.0))
    plan = manual_plan(reference, [first, second])
    with pytest.raises(VehicleRouteAssemblyError, match="missing"):
        assemble_vehicle_route(plan, [route(first, [(5.0, 1.0)])], open_space)


def test_rejects_unexpected_component_route(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    extra_component = component(frame, "component-extra", (20.0, 0.0, 22.0, 2.0))
    extra = route(extra_component, [(20.0, 1.0)])
    with pytest.raises(VehicleRouteAssemblyError, match="unexpected"):
        assemble_vehicle_route(plan, [source, extra], open_space)


def test_rejects_route_assigned_to_wrong_vehicle(frame, open_space) -> None:
    _, comp, plan, source = basic_single(frame)
    wrong = CoverageRouteRecord(
        source.request_id,
        source.component_id,
        source.source_region_id,
        "drone-2",
        source.frame,
        source.response_message,
        source.waypoints,
    )
    with pytest.raises(VehicleRouteAssemblyError, match="not assigned"):
        assemble_vehicle_route(plan, [wrong], open_space)


def test_rejects_route_frame_mismatch(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    other_frame = LocalCartesianFrame("other", "EPSG:32643", 631000.0, 3358000.0)
    wrong = CoverageRouteRecord(
        source.request_id,
        source.component_id,
        source.source_region_id,
        source.assigned_vehicle_id,
        other_frame,
        source.response_message,
        source.waypoints,
    )
    with pytest.raises(VehicleRouteAssemblyError, match="frame"):
        assemble_vehicle_route(plan, [wrong], open_space)


def test_rejects_route_waypoint_outside_free_space(frame) -> None:
    _, comp, plan, _ = basic_single(frame)
    source = route(comp, [(5.0, 0.0), (200.0, 0.0)])
    free_space = Polygon([(-20.0, -20.0), (120.0, -20.0), (120.0, 40.0), (-20.0, 40.0)])
    with pytest.raises(VehicleRouteAssemblyError, match="outside free_space"):
        assemble_vehicle_route(plan, [source], free_space)


def test_rejects_route_segment_crossing_hole(frame) -> None:
    free_space = Polygon(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)],
        [[(8.0, 1.0), (12.0, 1.0), (12.0, 7.0), (8.0, 7.0)]],
    )
    comp = component(frame, "component-a", (1.0, 4.0, 19.0, 6.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(1.0, 5.0))
    plan = manual_plan(reference, [comp])
    source = route(comp, [(2.0, 5.0), (18.0, 5.0)])
    with pytest.raises(VehicleRouteAssemblyError, match="segment"):
        assemble_vehicle_route(plan, [source], free_space)


def test_rejects_inconsistent_altitude_within_route(frame, open_space) -> None:
    _, comp, plan, _ = basic_single(frame)
    source = CoverageRouteRecord(
        "request-a",
        comp.component_id,
        comp.source_region_id,
        comp.assigned_vehicle_id,
        comp.frame,
        "planned",
        (CoverageWaypoint(5.0, 0.0, 30.0), CoverageWaypoint(10.0, 0.0, 31.0)),
    )
    with pytest.raises(VehicleRouteAssemblyError, match="common altitude"):
        assemble_vehicle_route(plan, [source], open_space)


def test_rejects_different_altitude_between_routes(frame, open_space) -> None:
    first = component(frame, "component-a", (5.0, 0.0, 7.0, 2.0))
    second = component(frame, "component-b", (15.0, 0.0, 17.0, 2.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.0, 1.0))
    plan = manual_plan(reference, [first, second])
    routes = [
        route(first, [(5.0, 1.0), (7.0, 1.0)], altitude_m=30.0),
        route(second, [(15.0, 1.0), (17.0, 1.0)], altitude_m=40.0),
    ]
    with pytest.raises(VehicleRouteAssemblyError, match="common altitude"):
        assemble_vehicle_route(plan, routes, open_space)


def test_rejects_reference_outside_free_space(frame) -> None:
    _, _, plan, source = basic_single(frame, home=(-100.0, 0.0))
    free_space = Polygon([(-20.0, -20.0), (120.0, -20.0), (120.0, 40.0), (-20.0, 40.0)])
    with pytest.raises(VehicleRouteAssemblyError, match="cannot reach"):
        assemble_vehicle_route(plan, [source], free_space)


def test_rejects_routes_in_disconnected_free_space_components(frame) -> None:
    first = component(frame, "component-a", (1.0, 0.0, 3.0, 2.0))
    second = component(frame, "component-b", (21.0, 0.0, 23.0, 2.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.0, 1.0))
    plan = manual_plan(reference, [first, second])
    free_space = Polygon([(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]).union(
        Polygon([(20.0, 0.0), (25.0, 0.0), (25.0, 5.0), (20.0, 5.0)])
    )
    routes = [
        route(first, [(1.0, 1.0), (3.0, 1.0)]),
        route(second, [(21.0, 1.0), (23.0, 1.0)]),
    ]
    with pytest.raises(VehicleRouteAssemblyError, match="planned route index 2"):
        assemble_vehicle_route(plan, routes, free_space)


def test_connector_complexity_limit_is_propagated(frame) -> None:
    # Eight boundary vertices plus start/goal exceed the configured maximum of 2.
    free_space = Polygon(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)],
        [[(8.0, 1.0), (12.0, 1.0), (12.0, 7.0), (8.0, 7.0)]],
    )
    comp = component(frame, "component-a", (16.0, 4.0, 19.0, 6.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(2.0, 5.0))
    plan = manual_plan(reference, [comp])
    source = route(comp, [(18.0, 5.0), (17.0, 5.0)])
    config = VehicleRouteAssemblyConfig(
        connector_config=ConnectorPlannerConfig(max_visibility_nodes=2)
    )
    with pytest.raises(VehicleRouteAssemblyError, match="cannot reach"):
        assemble_vehicle_route(plan, [source], free_space, config=config)


def test_complete_vehicle_route_rejects_wrong_algorithm(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    valid = assemble_vehicle_route(plan, [source], open_space)
    with pytest.raises(VehicleRouteAssemblyError, match="algorithm"):
        CompleteVehicleRoute(
            valid.component_plan,
            valid.oriented_routes,
            valid.reference_connector,
            valid.inter_route_connectors,
            valid.return_connector,
            valid.waypoints,
            valid.return_to_reference,
            algorithm="wrong",
        )


def test_complete_vehicle_route_rejects_tampered_waypoints(frame, open_space) -> None:
    _, _, plan, source = basic_single(frame)
    valid = assemble_vehicle_route(plan, [source], open_space)
    with pytest.raises(VehicleRouteAssemblyError, match="assembled waypoints"):
        CompleteVehicleRoute(
            valid.component_plan,
            valid.oriented_routes,
            valid.reference_connector,
            valid.inter_route_connectors,
            valid.return_connector,
            valid.waypoints[:-1],
            valid.return_to_reference,
        )
