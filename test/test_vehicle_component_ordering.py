#!/usr/bin/env python3
"""Tests for vehicle references and deterministic component ordering."""

from dataclasses import replace
import json
import math

import pytest
from shapely.geometry import Point, Polygon

from coverage_mission_pipeline.planning_request import LocalPoint2D
from coverage_mission_pipeline.planning_result import CoverageWaypoint
from coverage_mission_pipeline.prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
)
from coverage_mission_pipeline.route_record import local_to_geographic
from coverage_mission_pipeline.vehicle_component_ordering import (
    COMPONENT_ORDERING_ALGORITHM,
    VEHICLE_REFERENCE_SCHEMA_VERSION,
    ComponentVisit,
    VehicleComponentPlan,
    VehicleOrderingError,
    VehicleReference,
    order_components_by_vehicle,
    order_components_for_vehicle,
)


@pytest.fixture
def frame() -> LocalCartesianFrame:
    return LocalCartesianFrame("map", "EPSG:32643", 631285.61, 3358862.37)


@pytest.fixture
def reference(frame) -> VehicleReference:
    return VehicleReference("drone-1", frame, LocalPoint2D(0.0, 0.0))


def component(
    frame: LocalCartesianFrame,
    component_id: str,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    *,
    vehicle_id: str | None = "drone-1",
    source_region_id: str | None = None,
    component_index: int = 1,
    holes=(),
) -> PreparedComponent:
    return PreparedComponent(
        component_id=component_id,
        source_region_id=source_region_id or component_id.replace("_component", ""),
        component_index=component_index,
        frame=frame,
        polygon=Polygon(
            [(x0, y0), (x1, y0), (x1, y1), (x0, y1)],
            holes,
        ),
        assigned_vehicle_id=vehicle_id,
    )


def test_vehicle_reference_accepts_local_position(frame) -> None:
    value = VehicleReference(
        "drone-7",
        frame,
        LocalPoint2D(12.5, -4.25),
        "launch",
    )
    assert value.vehicle_id == "drone-7"
    assert value.position == LocalPoint2D(12.5, -4.25)
    assert value.reference_type == "launch"


@pytest.mark.parametrize(
    "reference_type",
    ["home", "launch", "current_position", "custom"],
)
def test_vehicle_reference_accepts_supported_reference_types(
    frame,
    reference_type,
) -> None:
    value = VehicleReference(
        "drone-1",
        frame,
        LocalPoint2D(0.0, 0.0),
        reference_type,
    )
    assert value.reference_type == reference_type


@pytest.mark.parametrize(
    "vehicle_id",
    ["", " bad", "bad id", "bad/id", ".bad", "a" * 129],
)
def test_vehicle_reference_rejects_unsafe_vehicle_id(frame, vehicle_id) -> None:
    with pytest.raises(VehicleOrderingError, match="vehicle_id"):
        VehicleReference(vehicle_id, frame, LocalPoint2D(0.0, 0.0))


def test_vehicle_reference_rejects_unknown_reference_type(frame) -> None:
    with pytest.raises(VehicleOrderingError, match="reference_type"):
        VehicleReference(
            "drone-1",
            frame,
            LocalPoint2D(0.0, 0.0),
            "garage",
        )


def test_vehicle_reference_rejects_wrong_frame_type() -> None:
    with pytest.raises(VehicleOrderingError, match="LocalCartesianFrame"):
        VehicleReference("drone-1", object(), LocalPoint2D(0.0, 0.0))


def test_vehicle_reference_rejects_wrong_position_type(frame) -> None:
    with pytest.raises(VehicleOrderingError, match="LocalPoint2D"):
        VehicleReference("drone-1", frame, (0.0, 0.0))


def test_vehicle_reference_from_projected(frame) -> None:
    value = VehicleReference.from_projected(
        "drone-1",
        frame,
        easting_m=frame.origin_easting_m + 12.0,
        northing_m=frame.origin_northing_m - 3.5,
    )
    assert value.position.x_m == pytest.approx(12.0)
    assert value.position.y_m == pytest.approx(-3.5)


def test_vehicle_reference_from_geographic_round_trip(frame) -> None:
    expected = LocalPoint2D(17.25, -9.5)
    geographic = local_to_geographic(
        CoverageWaypoint(expected.x_m, expected.y_m, 0.0),
        frame,
    )
    value = VehicleReference.from_geographic(
        "drone-1",
        frame,
        longitude_deg=geographic.longitude_deg,
        latitude_deg=geographic.latitude_deg,
    )
    assert value.position.x_m == pytest.approx(expected.x_m, abs=1.0e-6)
    assert value.position.y_m == pytest.approx(expected.y_m, abs=1.0e-6)


@pytest.mark.parametrize(
    "longitude, latitude",
    [(181.0, 0.0), (0.0, 91.0), (math.inf, 0.0), (0.0, math.nan)],
)
def test_vehicle_reference_from_geographic_rejects_invalid_coordinates(
    frame,
    longitude,
    latitude,
) -> None:
    with pytest.raises(VehicleOrderingError):
        VehicleReference.from_geographic(
            "drone-1",
            frame,
            longitude_deg=longitude,
            latitude_deg=latitude,
        )


def test_vehicle_reference_dict_contract(reference) -> None:
    value = reference.to_dict()
    assert value["schema_version"] == VEHICLE_REFERENCE_SCHEMA_VERSION
    assert value["vehicle_id"] == "drone-1"
    assert value["position_local_m"] == {"x_m": 0.0, "y_m": 0.0}
    assert value["frame"]["projected_crs"] == "EPSG:32643"


def test_vehicle_reference_json_is_deterministic(reference) -> None:
    first = reference.to_json()
    second = reference.to_json()
    assert first == second
    assert first.endswith("\n")
    assert json.loads(first) == reference.to_dict()


def test_vehicle_reference_json_round_trip(reference) -> None:
    recovered = VehicleReference.from_json(reference.to_json())
    assert recovered == reference


def test_vehicle_reference_strict_unknown_field(reference) -> None:
    value = reference.to_dict()
    value["unexpected"] = True
    with pytest.raises(VehicleOrderingError, match="unknown field"):
        VehicleReference.from_dict(value)


def test_vehicle_reference_strict_missing_field(reference) -> None:
    value = reference.to_dict()
    del value["position_local_m"]
    with pytest.raises(VehicleOrderingError, match="missing required"):
        VehicleReference.from_dict(value)


def test_vehicle_reference_rejects_wrong_schema_version(reference) -> None:
    value = reference.to_dict()
    value["schema_version"] = 999
    with pytest.raises(VehicleOrderingError, match="unsupported schema_version"):
        VehicleReference.from_dict(value)


def test_vehicle_reference_rejects_malformed_json() -> None:
    with pytest.raises(VehicleOrderingError, match="invalid JSON"):
        VehicleReference.from_json("{")


def test_vehicle_reference_atomic_write_and_read(reference, tmp_path) -> None:
    destination = reference.write(tmp_path)
    assert destination.name == "drone-1.reference.json"
    assert VehicleReference.read(destination) == reference
    assert not list(tmp_path.glob("*.tmp"))


def test_single_component_order(reference, frame) -> None:
    item = component(frame, "component-a", 10.0, 0.0, 20.0, 10.0)
    plan = order_components_for_vehicle(reference, [item])
    assert plan.component_ids == ("component-a",)
    assert plan.visits[0].transition_start == reference.position
    assert plan.visits[0].transition_end == LocalPoint2D(10.0, 0.0)
    assert plan.visits[0].straight_line_lower_bound_m == pytest.approx(10.0)


def test_first_component_is_nearest_to_vehicle_reference(reference, frame) -> None:
    far = component(frame, "far", 30.0, 0.0, 40.0, 10.0)
    near = component(frame, "near", 5.0, 0.0, 10.0, 10.0)
    plan = order_components_for_vehicle(reference, [far, near])
    assert plan.component_ids == ("near", "far")


def test_subsequent_component_uses_polygon_to_polygon_distance(reference, frame) -> None:
    first = component(frame, "first", 2.0, 0.0, 4.0, 2.0)
    north = component(frame, "north", 2.0, 5.0, 4.0, 7.0)
    east = component(frame, "east", 20.0, 0.0, 22.0, 2.0)
    plan = order_components_for_vehicle(reference, [east, north, first])
    assert plan.component_ids == ("first", "north", "east")
    assert plan.visits[1].straight_line_lower_bound_m == pytest.approx(3.0)


def test_order_is_independent_of_input_iteration_order(reference, frame) -> None:
    values = [
        component(frame, "c", 30.0, 0.0, 31.0, 1.0),
        component(frame, "a", 10.0, 0.0, 11.0, 1.0),
        component(frame, "b", 20.0, 0.0, 21.0, 1.0),
    ]
    forward = order_components_for_vehicle(reference, values)
    reverse = order_components_for_vehicle(reference, reversed(values))
    assert forward.component_ids == reverse.component_ids == ("a", "b", "c")


def test_equal_distance_tie_breaks_by_component_id(reference, frame) -> None:
    beta = component(frame, "beta", -2.0, 5.0, 0.0, 7.0)
    alpha = component(frame, "alpha", 0.0, 5.0, 2.0, 7.0)
    plan = order_components_for_vehicle(reference, [beta, alpha])
    assert plan.component_ids[0] == "alpha"


def test_reference_inside_component_produces_zero_first_distance(frame) -> None:
    reference = VehicleReference("drone-1", frame, LocalPoint2D(5.0, 5.0))
    item = component(frame, "inside", 0.0, 0.0, 10.0, 10.0)
    plan = order_components_for_vehicle(reference, [item])
    visit = plan.visits[0]
    assert visit.transition_start == reference.position
    assert visit.transition_end == reference.position
    assert visit.straight_line_lower_bound_m == 0.0


def test_reference_in_component_hole_projects_to_hole_boundary(frame) -> None:
    reference = VehicleReference("drone-1", frame, LocalPoint2D(5.0, 5.0))
    item = component(
        frame,
        "with-hole",
        0.0,
        0.0,
        10.0,
        10.0,
        holes=[[(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)]],
    )
    plan = order_components_for_vehicle(reference, [item])
    visit = plan.visits[0]
    assert visit.straight_line_lower_bound_m == pytest.approx(1.0)
    assert item.polygon.covers(Point(visit.transition_end.x_m, visit.transition_end.y_m))


def test_touching_components_have_zero_transition_distance(reference, frame) -> None:
    left = component(frame, "left", 1.0, 0.0, 3.0, 2.0)
    right = component(frame, "right", 3.0, 0.0, 5.0, 2.0)
    plan = order_components_for_vehicle(reference, [right, left])
    assert plan.component_ids == ("left", "right")
    assert plan.visits[1].straight_line_lower_bound_m == 0.0


def test_every_input_component_is_preserved_once(reference, frame) -> None:
    values = [
        component(frame, f"component-{index}", index * 3.0, 0.0, index * 3.0 + 1.0, 1.0)
        for index in range(1, 8)
    ]
    plan = order_components_for_vehicle(reference, values)
    assert len(plan.visits) == len(values)
    assert set(plan.component_ids) == {value.component_id for value in values}
    assert len(set(plan.component_ids)) == len(values)


def test_empty_component_set_produces_idle_plan(reference) -> None:
    plan = order_components_for_vehicle(reference, [])
    assert plan.visits == ()
    assert plan.component_ids == ()
    assert plan.total_straight_line_lower_bound_m == 0.0


def test_plan_total_is_sum_of_visit_lower_bounds(reference, frame) -> None:
    values = [
        component(frame, "a", 3.0, 0.0, 4.0, 1.0),
        component(frame, "b", 8.0, 0.0, 9.0, 1.0),
        component(frame, "c", 15.0, 0.0, 16.0, 1.0),
    ]
    plan = order_components_for_vehicle(reference, values)
    assert plan.total_straight_line_lower_bound_m == pytest.approx(
        sum(visit.straight_line_lower_bound_m for visit in plan.visits)
    )


def test_plan_summary_is_traceable(reference, frame) -> None:
    item = component(frame, "component-a", 2.0, 0.0, 4.0, 2.0)
    summary = order_components_for_vehicle(reference, [item]).to_summary_dict()
    assert summary["vehicle_id"] == "drone-1"
    assert summary["algorithm"] == COMPONENT_ORDERING_ALGORITHM
    assert summary["component_ids"] == ["component-a"]
    assert summary["visits"][0]["predecessor_component_id"] is None


def test_rejects_unassigned_component(reference, frame) -> None:
    item = component(
        frame,
        "unassigned",
        0.0,
        0.0,
        1.0,
        1.0,
        vehicle_id=None,
    )
    with pytest.raises(VehicleOrderingError, match="no vehicle assignment"):
        order_components_for_vehicle(reference, [item])


def test_rejects_component_assigned_to_other_vehicle(reference, frame) -> None:
    item = component(
        frame,
        "wrong",
        0.0,
        0.0,
        1.0,
        1.0,
        vehicle_id="drone-2",
    )
    with pytest.raises(VehicleOrderingError, match="not 'drone-1'"):
        order_components_for_vehicle(reference, [item])


def test_rejects_component_frame_mismatch(reference, frame) -> None:
    other_frame = LocalCartesianFrame("map", "EPSG:32643", 0.0, 0.0)
    item = component(other_frame, "wrong-frame", 0.0, 0.0, 1.0, 1.0)
    with pytest.raises(VehicleOrderingError, match="frame does not match"):
        order_components_for_vehicle(reference, [item])


def test_rejects_duplicate_component_ids(reference, frame) -> None:
    first = component(frame, "duplicate", 0.0, 0.0, 1.0, 1.0)
    second = component(frame, "duplicate", 3.0, 0.0, 4.0, 1.0)
    with pytest.raises(VehicleOrderingError, match="globally unique"):
        order_components_for_vehicle(reference, [first, second])


def test_rejects_positive_area_component_overlap(reference, frame) -> None:
    first = component(frame, "first", 0.0, 0.0, 5.0, 5.0)
    second = component(frame, "second", 4.0, 0.0, 9.0, 5.0)
    with pytest.raises(VehicleOrderingError, match="overlap"):
        order_components_for_vehicle(reference, [first, second])


def test_batch_groups_components_by_vehicle(frame) -> None:
    references = [
        VehicleReference("drone-2", frame, LocalPoint2D(100.0, 0.0)),
        VehicleReference("drone-1", frame, LocalPoint2D(0.0, 0.0)),
    ]
    values = [
        component(frame, "d2-a", 90.0, 0.0, 91.0, 1.0, vehicle_id="drone-2"),
        component(frame, "d1-a", 2.0, 0.0, 3.0, 1.0, vehicle_id="drone-1"),
        component(frame, "d2-b", 110.0, 0.0, 111.0, 1.0, vehicle_id="drone-2"),
    ]
    plans = order_components_by_vehicle(values, references)
    assert tuple(plan.vehicle_id for plan in plans) == ("drone-1", "drone-2")
    assert plans[0].component_ids == ("d1-a",)
    assert set(plans[1].component_ids) == {"d2-a", "d2-b"}


def test_batch_returns_idle_plan_for_reference_without_components(frame) -> None:
    references = [
        VehicleReference("drone-1", frame, LocalPoint2D(0.0, 0.0)),
        VehicleReference("drone-2", frame, LocalPoint2D(100.0, 0.0)),
    ]
    values = [
        component(frame, "only", 2.0, 0.0, 3.0, 1.0, vehicle_id="drone-1")
    ]
    plans = order_components_by_vehicle(values, references)
    assert plans[0].component_ids == ("only",)
    assert plans[1].vehicle_id == "drone-2"
    assert plans[1].component_ids == ()


def test_batch_rejects_unknown_vehicle_assignment(frame) -> None:
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0.0, 0.0))
    item = component(
        frame,
        "unknown",
        0.0,
        0.0,
        1.0,
        1.0,
        vehicle_id="drone-9",
    )
    with pytest.raises(VehicleOrderingError, match="unknown vehicle"):
        order_components_by_vehicle([item], [reference])


def test_batch_rejects_duplicate_vehicle_references(frame) -> None:
    references = [
        VehicleReference("drone-1", frame, LocalPoint2D(0.0, 0.0)),
        VehicleReference("drone-1", frame, LocalPoint2D(10.0, 0.0)),
    ]
    with pytest.raises(VehicleOrderingError, match="reference IDs must be unique"):
        order_components_by_vehicle([], references)


def test_batch_preserves_all_components_across_vehicles(frame) -> None:
    references = [
        VehicleReference("drone-1", frame, LocalPoint2D(0.0, 0.0)),
        VehicleReference("drone-2", frame, LocalPoint2D(100.0, 0.0)),
    ]
    values = [
        component(frame, "a", 1.0, 0.0, 2.0, 1.0, vehicle_id="drone-1"),
        component(frame, "b", 5.0, 0.0, 6.0, 1.0, vehicle_id="drone-1"),
        component(frame, "c", 101.0, 0.0, 102.0, 1.0, vehicle_id="drone-2"),
        component(frame, "d", 105.0, 0.0, 106.0, 1.0, vehicle_id="drone-2"),
    ]
    plans = order_components_by_vehicle(values, references)
    output = [component_id for plan in plans for component_id in plan.component_ids]
    assert len(output) == len(values)
    assert set(output) == {value.component_id for value in values}


def test_batch_rejects_overlaps_even_across_vehicles(frame) -> None:
    references = [
        VehicleReference("drone-1", frame, LocalPoint2D(0.0, 0.0)),
        VehicleReference("drone-2", frame, LocalPoint2D(10.0, 0.0)),
    ]
    values = [
        component(frame, "a", 0.0, 0.0, 5.0, 5.0, vehicle_id="drone-1"),
        component(frame, "b", 4.0, 0.0, 9.0, 5.0, vehicle_id="drone-2"),
    ]
    with pytest.raises(VehicleOrderingError, match="overlap"):
        order_components_by_vehicle(values, references)


def test_component_visit_rejects_inconsistent_distance(frame) -> None:
    item = component(frame, "a", 0.0, 0.0, 2.0, 2.0)
    with pytest.raises(VehicleOrderingError, match="does not match"):
        ComponentVisit(
            1,
            item,
            None,
            LocalPoint2D(-1.0, 0.0),
            LocalPoint2D(0.0, 0.0),
            99.0,
        )


def test_vehicle_component_plan_rejects_broken_predecessor_chain(reference, frame) -> None:
    first = component(frame, "a", 1.0, 0.0, 2.0, 1.0)
    second = component(frame, "b", 4.0, 0.0, 5.0, 1.0)
    visit_a = ComponentVisit(
        1,
        first,
        None,
        reference.position,
        LocalPoint2D(1.0, 0.0),
        1.0,
    )
    visit_b = ComponentVisit(
        2,
        second,
        "wrong",
        LocalPoint2D(2.0, 0.0),
        LocalPoint2D(4.0, 0.0),
        2.0,
    )
    with pytest.raises(VehicleOrderingError, match="predecessor chain"):
        VehicleComponentPlan(reference, (visit_a, visit_b))


def test_vehicle_component_plan_rejects_unknown_algorithm(reference) -> None:
    with pytest.raises(VehicleOrderingError, match="algorithm"):
        VehicleComponentPlan(reference, (), "other")
