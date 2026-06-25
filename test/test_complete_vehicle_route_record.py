#!/usr/bin/env python3
"""Tests for complete per-vehicle route serialization and waypoint spans."""

from dataclasses import replace
import json
import math

import pytest
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points

from coverage_mission_pipeline.complete_vehicle_route_record import (
    COMPLETE_VEHICLE_ROUTE_SCHEMA_VERSION,
    CONNECTOR_SEGMENT_KIND,
    COVERAGE_ROUTE_SEGMENT_KIND,
    INTER_ROUTE_CONNECTOR_ROLE,
    REFERENCE_CONNECTOR_ROLE,
    RETURN_CONNECTOR_ROLE,
    CompleteVehicleRouteRecord,
    CompleteVehicleRouteRecordError,
    ConnectorSpanRecord,
    CoverageRouteSpanRecord,
    WaypointSpan,
    make_complete_vehicle_route_records,
)
from coverage_mission_pipeline.planning_request import LocalPoint2D
from coverage_mission_pipeline.planning_result import CoverageWaypoint
from coverage_mission_pipeline.prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
)
from coverage_mission_pipeline.route_connector import (
    DIRECT_CONNECTOR_ALGORITHM,
    VISIBILITY_ASTAR_ALGORITHM,
)
from coverage_mission_pipeline.route_record import CoverageRouteRecord
from coverage_mission_pipeline.vehicle_component_ordering import (
    ComponentVisit,
    VehicleComponentPlan,
    VehicleReference,
)
from coverage_mission_pipeline.vehicle_route_assembly import (
    FORWARD_ROUTE_DIRECTION,
    REVERSED_ROUTE_DIRECTION,
    ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM,
    VehicleRouteAssemblyConfig,
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
    value: PreparedComponent,
    points: tuple[tuple[float, float], ...],
    *,
    request_id: str,
    altitude_m: float = 30.0,
) -> CoverageRouteRecord:
    return CoverageRouteRecord(
        request_id=request_id,
        component_id=value.component_id,
        source_region_id=value.source_region_id,
        assigned_vehicle_id=value.assigned_vehicle_id,
        frame=value.frame,
        response_message=f"planned {request_id}",
        waypoints=tuple(CoverageWaypoint(x, y, altitude_m) for x, y in points),
    )


def manual_plan(
    reference: VehicleReference,
    components: tuple[PreparedComponent, ...],
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


def assembled(
    frame: LocalCartesianFrame,
    open_space: Polygon,
    *,
    vehicle_id: str = "drone-1",
    return_to_reference: bool = False,
):
    first = component(frame, f"{vehicle_id}-a", (4.0, -1.0, 11.0, 1.0), vehicle_id=vehicle_id)
    second = component(frame, f"{vehicle_id}-b", (18.0, -1.0, 25.0, 1.0), vehicle_id=vehicle_id)
    reference = VehicleReference(vehicle_id, frame, LocalPoint2D(0.0, 0.0), "home")
    plan = manual_plan(reference, (first, second))
    routes = (
        route(first, ((5.0, 0.0), (10.0, 0.0)), request_id=f"{vehicle_id}-request-a"),
        route(second, ((19.0, 0.0), (24.0, 0.0)), request_id=f"{vehicle_id}-request-b"),
    )
    return assemble_vehicle_route(
        plan,
        routes,
        open_space,
        config=VehicleRouteAssemblyConfig(return_to_reference=return_to_reference),
    )


def record(frame: LocalCartesianFrame, open_space: Polygon, *, return_home=False):
    return CompleteVehicleRouteRecord.from_complete_route(
        assembled(frame, open_space, return_to_reference=return_home)
    )


def test_waypoint_span_properties() -> None:
    span = WaypointSpan(3, 7)
    assert span.waypoint_count == 5
    assert span.to_dict() == {"start_index": 3, "end_index": 7}


@pytest.mark.parametrize("start,end", [(-1, 0), (0, -1), (True, 1), (0, 1.5)])
def test_waypoint_span_rejects_bad_indexes(start, end) -> None:
    with pytest.raises(CompleteVehicleRouteRecordError):
        WaypointSpan(start, end)


def test_waypoint_span_rejects_reversed_range() -> None:
    with pytest.raises(CompleteVehicleRouteRecordError, match="greater"):
        WaypointSpan(3, 2)


def test_waypoint_span_strict_dict() -> None:
    with pytest.raises(CompleteVehicleRouteRecordError, match="unknown"):
        WaypointSpan.from_dict({"start_index": 0, "end_index": 1, "extra": 2})


def test_route_span_round_trip() -> None:
    value = CoverageRouteSpanRecord(
        WaypointSpan(1, 3),
        "request-a",
        "component-a",
        "region-a",
        FORWARD_ROUTE_DIRECTION,
        "planned",
        4.0,
    )
    assert CoverageRouteSpanRecord.from_dict(value.to_dict(), "segment") == value
    assert value.kind == COVERAGE_ROUTE_SEGMENT_KIND


@pytest.mark.parametrize("direction", ["", "backward", None])
def test_route_span_rejects_bad_direction(direction) -> None:
    with pytest.raises(CompleteVehicleRouteRecordError, match="direction"):
        CoverageRouteSpanRecord(
            WaypointSpan(0, 1),
            "request-a",
            "component-a",
            "region-a",
            direction,
            "planned",
            1.0,
        )


def test_route_span_rejects_bad_identifier() -> None:
    with pytest.raises(CompleteVehicleRouteRecordError, match="request_id"):
        CoverageRouteSpanRecord(
            WaypointSpan(0, 1),
            "bad id",
            "component-a",
            "region-a",
            FORWARD_ROUTE_DIRECTION,
            "planned",
            1.0,
        )


def test_connector_span_round_trip() -> None:
    value = ConnectorSpanRecord(
        WaypointSpan(0, 2),
        REFERENCE_CONNECTOR_ROLE,
        "vehicle-reference:drone-1",
        "request-a",
        DIRECT_CONNECTOR_ALGORITHM,
        3.0,
    )
    assert ConnectorSpanRecord.from_dict(value.to_dict(), "segment") == value
    assert value.kind == CONNECTOR_SEGMENT_KIND


@pytest.mark.parametrize("role", ["", "home", None])
def test_connector_span_rejects_bad_role(role) -> None:
    with pytest.raises(CompleteVehicleRouteRecordError, match="role"):
        ConnectorSpanRecord(
            WaypointSpan(0, 1),
            role,
            "a",
            "b",
            DIRECT_CONNECTOR_ALGORITHM,
            1.0,
        )


def test_connector_span_rejects_same_ids() -> None:
    with pytest.raises(CompleteVehicleRouteRecordError, match="differ"):
        ConnectorSpanRecord(
            WaypointSpan(0, 1),
            INTER_ROUTE_CONNECTOR_ROLE,
            "same",
            "same",
            DIRECT_CONNECTOR_ALGORITHM,
            1.0,
        )


def test_connector_span_rejects_bad_algorithm() -> None:
    with pytest.raises(CompleteVehicleRouteRecordError, match="algorithm"):
        ConnectorSpanRecord(
            WaypointSpan(0, 1),
            INTER_ROUTE_CONNECTOR_ROLE,
            "a",
            "b",
            "grid_astar",
            1.0,
        )


def test_from_complete_route_preserves_summary(frame, open_space) -> None:
    source = assembled(frame, open_space)
    result = CompleteVehicleRouteRecord.from_complete_route(source)
    assert result.vehicle_id == source.vehicle_id
    assert result.frame == source.frame
    assert result.reference_type == "home"
    assert result.reference_position == LocalPoint2D(0.0, 0.0)
    assert result.algorithm == ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM
    assert result.return_to_reference is False
    assert result.waypoints == source.waypoints
    assert result.route_directions == source.route_directions
    assert result.total_path_length_m == pytest.approx(source.total_path_length_m)


def test_segments_alternate_without_return(frame, open_space) -> None:
    value = record(frame, open_space)
    assert [segment.kind for segment in value.segments] == [
        CONNECTOR_SEGMENT_KIND,
        COVERAGE_ROUTE_SEGMENT_KIND,
        CONNECTOR_SEGMENT_KIND,
        COVERAGE_ROUTE_SEGMENT_KIND,
    ]
    assert [segment.role for segment in value.connector_segments] == [
        REFERENCE_CONNECTOR_ROLE,
        INTER_ROUTE_CONNECTOR_ROLE,
    ]


def test_segments_include_return_connector(frame, open_space) -> None:
    value = record(frame, open_space, return_home=True)
    assert value.return_to_reference
    assert value.connector_segments[-1].role == RETURN_CONNECTOR_ROLE
    assert value.waypoints[-1].x_m == pytest.approx(value.reference_position.x_m)
    assert value.waypoints[-1].y_m == pytest.approx(value.reference_position.y_m)


def test_segment_spans_cover_single_array_without_gaps(frame, open_space) -> None:
    value = record(frame, open_space, return_home=True)
    assert value.segments[0].span.start_index == 0
    for previous, following in zip(value.segments, value.segments[1:]):
        assert previous.span.end_index == following.span.start_index
    assert value.segments[-1].span.end_index == len(value.waypoints) - 1


def test_segment_waypoints_match_source_paths(frame, open_space) -> None:
    source = assembled(frame, open_space, return_to_reference=True)
    value = CompleteVehicleRouteRecord.from_complete_route(source)
    assert value.segment_waypoints(value.connector_segments[0]) == source.reference_connector.waypoints
    assert value.segment_waypoints(value.route_segments[0]) == source.oriented_routes[0].waypoints
    assert value.segment_waypoints(value.connector_segments[-1]) == source.return_connector.waypoints


def test_record_stores_one_waypoint_array(frame, open_space) -> None:
    payload = record(frame, open_space).to_dict()
    assert "waypoints_local_m" in payload
    assert "projected_waypoints" not in payload
    assert "geographic_waypoints" not in payload
    assert all("waypoints" not in segment for segment in payload["segments"])


def test_derived_lengths_match_source(frame, open_space) -> None:
    source = assembled(frame, open_space, return_to_reference=True)
    value = CompleteVehicleRouteRecord.from_complete_route(source)
    assert value.total_route_length_m == pytest.approx(source.total_route_length_m)
    assert value.total_connector_length_m == pytest.approx(source.total_connector_length_m)
    assert value.total_path_length_m == pytest.approx(source.total_path_length_m)


def test_json_is_deterministic(frame, open_space) -> None:
    value = record(frame, open_space)
    assert value.to_json() == value.to_json()
    assert value.to_json().endswith("\n")
    assert json.loads(value.to_json()) == value.to_dict()


def test_json_round_trip(frame, open_space) -> None:
    value = record(frame, open_space, return_home=True)
    restored = CompleteVehicleRouteRecord.from_json(value.to_json())
    assert restored == value
    assert restored.to_json() == value.to_json()


def test_write_and_read_round_trip(frame, open_space, tmp_path) -> None:
    value = record(frame, open_space)
    destination = value.write(tmp_path)
    assert destination.name == "drone-1.complete-route.json"
    assert CompleteVehicleRouteRecord.read(destination) == value
    assert not list(tmp_path.glob("*.tmp"))


def test_write_explicit_filename(frame, open_space, tmp_path) -> None:
    value = record(frame, open_space)
    destination = value.write(tmp_path / "custom.json")
    assert destination == tmp_path / "custom.json"


def test_projected_waypoints(frame, open_space) -> None:
    value = record(frame, open_space)
    points = value.projected_waypoints()
    assert points[0].easting_m == pytest.approx(frame.origin_easting_m)
    assert points[0].northing_m == pytest.approx(frame.origin_northing_m)
    assert points[0].altitude_m == pytest.approx(30.0)


def test_geographic_waypoints_preserve_count_and_altitude(frame, open_space) -> None:
    value = record(frame, open_space)
    points = value.geographic_waypoints()
    assert len(points) == len(value.waypoints)
    assert all(point.altitude_m == pytest.approx(30.0) for point in points)


def test_summary(frame, open_space) -> None:
    value = record(frame, open_space, return_home=True)
    summary = value.to_summary_dict()
    assert summary["vehicle_id"] == "drone-1"
    assert summary["route_count"] == 2
    assert summary["connector_count"] == 3
    assert summary["return_to_reference"] is True
    assert summary["component_ids"] == ["drone-1-a", "drone-1-b"]


def test_idle_complete_route_round_trip(frame, open_space) -> None:
    reference = VehicleReference("idle-drone", frame, LocalPoint2D(3.0, 4.0), "launch")
    source = assemble_vehicle_route(VehicleComponentPlan(reference, ()), (), open_space)
    value = CompleteVehicleRouteRecord.from_complete_route(source)
    assert value.is_idle
    assert value.segments == ()
    assert value.waypoints == ()
    assert CompleteVehicleRouteRecord.from_json(value.to_json()) == value


def test_reversed_direction_is_preserved(frame, open_space) -> None:
    value = record(frame, open_space)
    assert set(value.route_directions).issubset(
        {FORWARD_ROUTE_DIRECTION, REVERSED_ROUTE_DIRECTION}
    )
    payload = value.to_dict()
    restored = CompleteVehicleRouteRecord.from_dict(payload)
    assert restored.route_directions == value.route_directions


def test_visibility_algorithm_is_serialized(frame) -> None:
    free_space = Polygon(
        [(0.0, 0.0), (30.0, 0.0), (30.0, 20.0), (0.0, 20.0)],
        holes=[[(8.0, 2.0), (12.0, 2.0), (12.0, 12.0), (8.0, 12.0)]],
    )
    comp = component(frame, "component-a", (16.0, 4.0, 22.0, 8.0))
    reference = VehicleReference("drone-1", frame, LocalPoint2D(2.0, 6.0), "home")
    plan = manual_plan(reference, (comp,))
    source = assemble_vehicle_route(
        plan,
        (route(comp, ((17.0, 6.0), (21.0, 6.0)), request_id="request-a"),),
        free_space,
    )
    value = CompleteVehicleRouteRecord.from_complete_route(source)
    assert value.connector_segments[0].algorithm == VISIBILITY_ASTAR_ALGORITHM


def mutate(value: CompleteVehicleRouteRecord, function):
    payload = value.to_dict()
    function(payload)
    return payload


@pytest.mark.parametrize(
    "field",
    [
        "vehicle_id",
        "frame",
        "reference",
        "algorithm",
        "return_to_reference",
        "segments",
        "waypoints_local_m",
    ],
)
def test_rejects_missing_root_fields(frame, open_space, field) -> None:
    payload = record(frame, open_space).to_dict()
    del payload[field]
    with pytest.raises(CompleteVehicleRouteRecordError, match="missing"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_unknown_root_field(frame, open_space) -> None:
    payload = record(frame, open_space).to_dict()
    payload["extra"] = 1
    with pytest.raises(CompleteVehicleRouteRecordError, match="unknown"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_wrong_schema_version(frame, open_space) -> None:
    payload = record(frame, open_space).to_dict()
    payload["schema_version"] = COMPLETE_VEHICLE_ROUTE_SCHEMA_VERSION + 1
    with pytest.raises(CompleteVehicleRouteRecordError, match="unsupported"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_unknown_segment_kind(frame, open_space) -> None:
    payload = record(frame, open_space).to_dict()
    payload["segments"][0]["kind"] = "teleport"
    with pytest.raises(CompleteVehicleRouteRecordError, match="kind"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_segment_span_out_of_bounds(frame, open_space) -> None:
    payload = record(frame, open_space).to_dict()
    payload["segments"][-1]["span"]["end_index"] = 999
    with pytest.raises(CompleteVehicleRouteRecordError, match="exceeds"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_segment_gap(frame, open_space) -> None:
    payload = record(frame, open_space).to_dict()
    payload["segments"][1]["span"]["start_index"] += 1
    with pytest.raises(CompleteVehicleRouteRecordError, match="boundary"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_wrong_segment_length(frame, open_space) -> None:
    payload = record(frame, open_space).to_dict()
    payload["segments"][0]["length_m"] += 1.0
    with pytest.raises(CompleteVehicleRouteRecordError, match="length_m"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_wrong_first_segment_role(frame, open_space) -> None:
    payload = record(frame, open_space).to_dict()
    payload["segments"][0]["role"] = INTER_ROUTE_CONNECTOR_ROLE
    with pytest.raises(CompleteVehicleRouteRecordError, match="wrong role"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_broken_connector_chain(frame, open_space) -> None:
    payload = record(frame, open_space).to_dict()
    payload["segments"][2]["to_request_id"] = "wrong-request"
    with pytest.raises(CompleteVehicleRouteRecordError, match="endpoint chain"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_wrong_segment_alternation(frame, open_space) -> None:
    value = record(frame, open_space)
    with pytest.raises(CompleteVehicleRouteRecordError):
        replace(
            value,
            segments=(value.segments[0], value.segments[2], *value.segments[2:]),
        )


def test_rejects_first_waypoint_not_at_reference(frame, open_space) -> None:
    payload = record(frame, open_space).to_dict()
    payload["waypoints_local_m"][0]["x_m"] += 0.5
    with pytest.raises(CompleteVehicleRouteRecordError, match="first waypoint"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_inconsistent_altitude(frame, open_space) -> None:
    payload = record(frame, open_space).to_dict()
    payload["waypoints_local_m"][2]["z_m"] += 1.0
    with pytest.raises(CompleteVehicleRouteRecordError, match="altitude"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_missing_return_connector(frame, open_space) -> None:
    value = record(frame, open_space, return_home=True)
    payload = value.to_dict()
    payload["segments"].pop()
    with pytest.raises(CompleteVehicleRouteRecordError):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_final_return_point_not_at_reference(frame, open_space) -> None:
    payload = record(frame, open_space, return_home=True).to_dict()
    payload["waypoints_local_m"][-1]["x_m"] += 0.25
    with pytest.raises(CompleteVehicleRouteRecordError, match="final waypoint"):
        CompleteVehicleRouteRecord.from_dict(payload)


def test_rejects_idle_record_with_waypoint(frame) -> None:
    with pytest.raises(CompleteVehicleRouteRecordError, match="idle"):
        CompleteVehicleRouteRecord(
            "drone-1",
            frame,
            "home",
            LocalPoint2D(0.0, 0.0),
            ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM,
            False,
            (),
            (CoverageWaypoint(0.0, 0.0, 30.0),),
        )


def test_rejects_nonidle_record_without_waypoints(frame) -> None:
    segment = ConnectorSpanRecord(
        WaypointSpan(0, 0),
        REFERENCE_CONNECTOR_ROLE,
        "vehicle-reference:drone-1",
        "request-a",
        DIRECT_CONNECTOR_ALGORITHM,
        0.0,
    )
    with pytest.raises(CompleteVehicleRouteRecordError, match="non-idle"):
        CompleteVehicleRouteRecord(
            "drone-1",
            frame,
            "home",
            LocalPoint2D(0.0, 0.0),
            ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM,
            False,
            (segment,),
            (),
        )


def test_segment_waypoints_rejects_foreign_segment(frame, open_space) -> None:
    value = record(frame, open_space)
    foreign = ConnectorSpanRecord(
        WaypointSpan(0, 0),
        REFERENCE_CONNECTOR_ROLE,
        "vehicle-reference:other",
        "request-x",
        DIRECT_CONNECTOR_ALGORITHM,
        0.0,
    )
    with pytest.raises(CompleteVehicleRouteRecordError, match="does not belong"):
        value.segment_waypoints(foreign)


def test_from_complete_route_rejects_wrong_type() -> None:
    with pytest.raises(CompleteVehicleRouteRecordError, match="CompleteVehicleRoute"):
        CompleteVehicleRouteRecord.from_complete_route(object())


def test_batch_records_are_sorted_by_vehicle(frame, open_space) -> None:
    drone_b = assembled(frame, open_space, vehicle_id="drone-b")
    drone_a = assembled(frame, open_space, vehicle_id="drone-a")
    values = make_complete_vehicle_route_records((drone_b, drone_a))
    assert [value.vehicle_id for value in values] == ["drone-a", "drone-b"]


def test_batch_rejects_duplicate_vehicle_ids(frame, open_space) -> None:
    first = assembled(frame, open_space)
    second = assembled(frame, open_space)
    with pytest.raises(CompleteVehicleRouteRecordError, match="unique"):
        make_complete_vehicle_route_records((first, second))


def test_batch_rejects_empty() -> None:
    with pytest.raises(CompleteVehicleRouteRecordError, match="must not be empty"):
        make_complete_vehicle_route_records(())


def test_batch_rejects_wrong_item() -> None:
    with pytest.raises(CompleteVehicleRouteRecordError, match=r"routes\[0\]"):
        make_complete_vehicle_route_records((object(),))
