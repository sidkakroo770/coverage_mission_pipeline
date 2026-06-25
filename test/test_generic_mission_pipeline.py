#!/usr/bin/env python3
"""Tests for the schema-independent end-to-end mission pipeline."""

from __future__ import annotations

from dataclasses import replace
import json
import math
from pathlib import Path

import pytest
from shapely.geometry import GeometryCollection, Polygon

from coverage_mission_pipeline.ardupilot_mission import (
    ArduPilotMission,
    ArduPilotMissionBuildConfig,
    END_ACTION_LAND_AT_REFERENCE,
    END_ACTION_NONE,
    END_ACTION_RTL,
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
)
from coverage_mission_pipeline.complete_vehicle_route_record import (
    CompleteVehicleRouteRecord,
)
from coverage_mission_pipeline.generic_mission_pipeline import (
    GENERIC_MISSION_MANIFEST_SCHEMA_VERSION,
    GENERIC_MISSION_PIPELINE_ALGORITHM,
    ComponentPlanningSpec,
    GenericMissionDefinition,
    GenericMissionPipelineConfig,
    GenericMissionPipelineError,
    GenericMissionPipelineResult,
    run_generic_mission_pipeline,
)
from coverage_mission_pipeline.planning_request import (
    CoveragePlanningRequest,
    LocalPoint2D,
)
from coverage_mission_pipeline.planning_result import (
    CoveragePlanningResult,
    CoverageWaypoint,
)
from coverage_mission_pipeline.prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
)
from coverage_mission_pipeline.route_connector import VISIBILITY_ASTAR_ALGORITHM
from coverage_mission_pipeline.route_record import CoverageRouteRecord
from coverage_mission_pipeline.start_goal_policy import (
    StartGoalPolicyConfig,
    StartGoalSelection,
)
from coverage_mission_pipeline.vehicle_component_ordering import VehicleReference
from coverage_mission_pipeline.vehicle_route_assembly import (
    VehicleRouteAssemblyConfig,
)


class FakePlannerRunner:
    def __init__(self) -> None:
        self.calls: list[tuple[CoveragePlanningRequest, ...]] = []

    def run(self, requests):
        values = tuple(requests)
        self.calls.append(values)
        return tuple(
            CoveragePlanningResult.from_request(
                request,
                response_message=f"planned {request.request_id}",
                waypoints=(
                    CoverageWaypoint(
                        request.start.x_m,
                        request.start.y_m,
                        request.altitude_m,
                    ),
                    CoverageWaypoint(
                        request.goal.x_m,
                        request.goal.y_m,
                        request.altitude_m,
                    ),
                ),
            )
            for request in values
        )


class ReversedResultRunner(FakePlannerRunner):
    def run(self, requests):
        return tuple(reversed(super().run(requests)))


class WrongTypeRunner:
    def run(self, requests):
        return (object(),)


class NonIterableRunner:
    def run(self, requests):
        return 7


class RaisingRunner:
    def run(self, requests):
        raise RuntimeError("planner exploded")


@pytest.fixture
def frame() -> LocalCartesianFrame:
    return LocalCartesianFrame("map", "EPSG:32643", 631000.0, 3358000.0)


@pytest.fixture
def free_space() -> Polygon:
    return Polygon([(-20.0, -20.0), (100.0, -20.0), (100.0, 50.0), (-20.0, 50.0)])


def component(
    frame: LocalCartesianFrame,
    component_id: str,
    vehicle_id: str,
    x0: float,
    *,
    y0: float = 0.0,
) -> PreparedComponent:
    return PreparedComponent(
        component_id=component_id,
        source_region_id=f"region-{component_id}",
        component_index=1,
        frame=frame,
        polygon=Polygon(
            [
                (x0, y0),
                (x0 + 8.0, y0),
                (x0 + 8.0, y0 + 8.0),
                (x0, y0 + 8.0),
            ]
        ),
        assigned_vehicle_id=vehicle_id,
    )


def spec(
    item: PreparedComponent,
    *,
    request_id: str | None = None,
    altitude_m: float = 30.0,
    start_anchor: LocalPoint2D | None = None,
    goal_anchor: LocalPoint2D | None = None,
    policy: StartGoalPolicyConfig | None = None,
) -> ComponentPlanningSpec:
    minx, miny, maxx, maxy = item.polygon.bounds
    return ComponentPlanningSpec(
        component_id=item.component_id,
        start_anchor=start_anchor or LocalPoint2D(minx + 1.0, miny + 1.0),
        goal_anchor=goal_anchor or LocalPoint2D(maxx - 1.0, maxy - 1.0),
        altitude_m=altitude_m,
        lateral_footprint_m=2.0,
        lateral_overlap=0.1,
        request_id=request_id,
        start_goal_policy=policy or StartGoalPolicyConfig(),
    )


def one_vehicle_definition(frame, free_space) -> GenericMissionDefinition:
    first = component(frame, "component-a", "drone-1", 10.0)
    second = component(frame, "component-b", "drone-1", 40.0)
    return GenericMissionDefinition(
        components=(second, first),
        vehicle_references=(
            VehicleReference("drone-1", frame, LocalPoint2D(0.0, 0.0), "home"),
        ),
        planning_specs=(
            spec(second, request_id="request-b"),
            spec(first, request_id="request-a"),
        ),
        free_space_by_vehicle_id={"drone-1": free_space},
    )


def two_vehicle_definition(frame, free_space) -> GenericMissionDefinition:
    a = component(frame, "component-a", "drone-b", 30.0)
    b = component(frame, "component-b", "drone-a", 10.0)
    return GenericMissionDefinition(
        components=(a, b),
        vehicle_references=(
            VehicleReference("drone-b", frame, LocalPoint2D(25.0, 0.0), "launch"),
            VehicleReference("drone-a", frame, LocalPoint2D(0.0, 0.0), "home"),
        ),
        planning_specs=(spec(a), spec(b)),
        free_space_by_vehicle_id={
            "drone-b": free_space,
            "drone-a": free_space,
        },
    )


# ---------------------------------------------------------------------------
# ComponentPlanningSpec validation
# ---------------------------------------------------------------------------


def test_spec_defaults_request_id_to_component(frame) -> None:
    item = component(frame, "component-a", "drone-1", 0.0)
    assert spec(item).resolved_request_id == "component-a"


def test_spec_preserves_explicit_request_id(frame) -> None:
    item = component(frame, "component-a", "drone-1", 0.0)
    assert spec(item, request_id="request-a").resolved_request_id == "request-a"


def test_spec_summary_is_complete(frame) -> None:
    item = component(frame, "component-a", "drone-1", 0.0)
    summary = spec(item, request_id="request-a").to_summary_dict()
    assert summary["component_id"] == "component-a"
    assert summary["request_id"] == "request-a"
    assert summary["altitude_m"] == 30.0
    assert summary["start_goal_policy"]["boundary_clearance_m"] == 0.0


@pytest.mark.parametrize("value", ["", " bad", "a/b", "x" * 129, 1, None])
def test_spec_rejects_invalid_component_id(value) -> None:
    with pytest.raises(ValueError, match="component_id"):
        ComponentPlanningSpec(
            value,
            LocalPoint2D(0, 0),
            LocalPoint2D(1, 1),
            30,
            2,
            0.1,
        )


@pytest.mark.parametrize("value", ["", "bad/id", "x" * 129, 4])
def test_spec_rejects_invalid_request_id(value) -> None:
    with pytest.raises(ValueError, match="request_id"):
        ComponentPlanningSpec(
            "component-a",
            LocalPoint2D(0, 0),
            LocalPoint2D(1, 1),
            30,
            2,
            0.1,
            request_id=value,
        )


@pytest.mark.parametrize("field_name", ["start_anchor", "goal_anchor"])
def test_spec_rejects_wrong_anchor_type(field_name) -> None:
    kwargs = dict(
        component_id="component-a",
        start_anchor=LocalPoint2D(0, 0),
        goal_anchor=LocalPoint2D(1, 1),
        altitude_m=30,
        lateral_footprint_m=2,
        lateral_overlap=0.1,
    )
    kwargs[field_name] = object()
    with pytest.raises(ValueError, match=field_name):
        ComponentPlanningSpec(**kwargs)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, "30", True])
def test_spec_rejects_invalid_altitude(value) -> None:
    with pytest.raises(ValueError, match="altitude_m"):
        ComponentPlanningSpec(
            "component-a",
            LocalPoint2D(0, 0),
            LocalPoint2D(1, 1),
            value,
            2,
            0.1,
        )


@pytest.mark.parametrize("value", [0, -1, math.nan, math.inf, "2", False])
def test_spec_rejects_invalid_footprint(value) -> None:
    with pytest.raises(ValueError, match="lateral_footprint_m"):
        ComponentPlanningSpec(
            "component-a",
            LocalPoint2D(0, 0),
            LocalPoint2D(1, 1),
            30,
            value,
            0.1,
        )


@pytest.mark.parametrize("value", [-0.1, 1.0, 1.1, math.nan, "0.1", True])
def test_spec_rejects_invalid_overlap(value) -> None:
    with pytest.raises(ValueError, match="lateral_overlap"):
        ComponentPlanningSpec(
            "component-a",
            LocalPoint2D(0, 0),
            LocalPoint2D(1, 1),
            30,
            2,
            value,
        )


def test_spec_rejects_wrong_policy_type() -> None:
    with pytest.raises(ValueError, match="StartGoalPolicyConfig"):
        ComponentPlanningSpec(
            "component-a",
            LocalPoint2D(0, 0),
            LocalPoint2D(1, 1),
            30,
            2,
            0.1,
            start_goal_policy=object(),
        )


# ---------------------------------------------------------------------------
# GenericMissionDefinition validation
# ---------------------------------------------------------------------------


def test_definition_normalizes_iterables_and_mapping(frame, free_space) -> None:
    item = component(frame, "component-a", "drone-1", 0.0)
    definition = GenericMissionDefinition(
        components=[item],
        vehicle_references=[
            VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
        ],
        planning_specs=[spec(item)],
        free_space_by_vehicle_id={"drone-1": free_space},
    )
    assert isinstance(definition.components, tuple)
    assert isinstance(definition.vehicle_references, tuple)
    assert definition.component_ids == ("component-a",)
    assert definition.vehicle_ids == ("drone-1",)
    with pytest.raises(TypeError):
        definition.free_space_by_vehicle_id["drone-2"] = free_space


def test_definition_allows_all_idle_vehicles(frame, free_space) -> None:
    definition = GenericMissionDefinition(
        components=(),
        vehicle_references=(
            VehicleReference("drone-1", frame, LocalPoint2D(0, 0)),
        ),
        planning_specs=(),
        free_space_by_vehicle_id={"drone-1": free_space},
    )
    assert definition.components == ()


def test_definition_rejects_no_references(free_space) -> None:
    with pytest.raises(ValueError, match="must not be empty"):
        GenericMissionDefinition((), (), (), {})


def test_definition_rejects_wrong_component_type(frame, free_space) -> None:
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="PreparedComponent"):
        GenericMissionDefinition(
            (object(),),
            (reference,),
            (),
            {"drone-1": free_space},
        )


def test_definition_rejects_wrong_reference_type(frame, free_space) -> None:
    with pytest.raises(ValueError, match="VehicleReference"):
        GenericMissionDefinition((), (object(),), (), {"x": free_space})


def test_definition_rejects_wrong_spec_type(frame, free_space) -> None:
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="ComponentPlanningSpec"):
        GenericMissionDefinition(
            (),
            (reference,),
            (object(),),
            {"drone-1": free_space},
        )


def test_definition_rejects_non_mapping_free_space(frame) -> None:
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="mapping"):
        GenericMissionDefinition((), (reference,), (), [])


def test_definition_rejects_duplicate_component_ids(frame, free_space) -> None:
    a = component(frame, "same", "drone-1", 0)
    b = component(frame, "same", "drone-1", 20)
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="component IDs"):
        GenericMissionDefinition(
            (a, b),
            (reference,),
            (spec(a), spec(b)),
            {"drone-1": free_space},
        )


def test_definition_rejects_duplicate_vehicle_ids(frame, free_space) -> None:
    refs = (
        VehicleReference("drone-1", frame, LocalPoint2D(0, 0)),
        VehicleReference("drone-1", frame, LocalPoint2D(1, 0)),
    )
    with pytest.raises(ValueError, match="vehicle reference IDs"):
        GenericMissionDefinition((), refs, (), {"drone-1": free_space})


def test_definition_rejects_duplicate_spec_component_ids(frame, free_space) -> None:
    item = component(frame, "component-a", "drone-1", 0)
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="spec component IDs"):
        GenericMissionDefinition(
            (item,),
            (reference,),
            (spec(item), spec(item, request_id="request-2")),
            {"drone-1": free_space},
        )


def test_definition_rejects_duplicate_request_ids(frame, free_space) -> None:
    a = component(frame, "component-a", "drone-1", 0)
    b = component(frame, "component-b", "drone-1", 20)
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="request IDs"):
        GenericMissionDefinition(
            (a, b),
            (reference,),
            (spec(a, request_id="same"), spec(b, request_id="same")),
            {"drone-1": free_space},
        )


def test_definition_rejects_missing_spec(frame, free_space) -> None:
    item = component(frame, "component-a", "drone-1", 0)
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="missing: component-a"):
        GenericMissionDefinition(
            (item,),
            (reference,),
            (),
            {"drone-1": free_space},
        )


def test_definition_rejects_unexpected_spec(frame, free_space) -> None:
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    ghost = ComponentPlanningSpec(
        "ghost", LocalPoint2D(0, 0), LocalPoint2D(1, 1), 30, 2, 0.1
    )
    with pytest.raises(ValueError, match="unexpected: ghost"):
        GenericMissionDefinition(
            (),
            (reference,),
            (ghost,),
            {"drone-1": free_space},
        )


def test_definition_rejects_missing_free_space(frame) -> None:
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="missing: drone-1"):
        GenericMissionDefinition((), (reference,), (), {})


def test_definition_rejects_extra_free_space(frame, free_space) -> None:
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="unexpected: drone-2"):
        GenericMissionDefinition(
            (),
            (reference,),
            (),
            {"drone-1": free_space, "drone-2": free_space},
        )


@pytest.mark.parametrize(
    "geometry, message",
    [
        (object(), "Shapely"),
        (GeometryCollection(), "must not be empty"),
        (Polygon([(0, 0), (1, 1), (1, 0), (0, 1)]), "must be valid"),
    ],
)
def test_definition_rejects_invalid_free_space(frame, geometry, message) -> None:
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match=message):
        GenericMissionDefinition(
            (),
            (reference,),
            (),
            {"drone-1": geometry},
        )


def test_definition_rejects_unassigned_component(frame, free_space) -> None:
    item = PreparedComponent(
        "component-a",
        "region-a",
        1,
        frame,
        Polygon([(0, 0), (5, 0), (5, 5), (0, 5)]),
        None,
    )
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="no vehicle assignment"):
        GenericMissionDefinition(
            (item,),
            (reference,),
            (spec(item),),
            {"drone-1": free_space},
        )


def test_definition_rejects_unknown_vehicle_assignment(frame, free_space) -> None:
    item = component(frame, "component-a", "ghost", 0)
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="unknown vehicle"):
        GenericMissionDefinition(
            (item,),
            (reference,),
            (spec(item),),
            {"drone-1": free_space},
        )


def test_definition_rejects_component_frame_mismatch(frame, free_space) -> None:
    other = LocalCartesianFrame("other", "EPSG:32643", 631000, 3358000)
    item = component(other, "component-a", "drone-1", 0)
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    with pytest.raises(ValueError, match="frame does not match"):
        GenericMissionDefinition(
            (item,),
            (reference,),
            (spec(item),),
            {"drone-1": free_space},
        )


def test_definition_rejects_component_outside_free_space(frame) -> None:
    item = component(frame, "component-a", "drone-1", 100)
    reference = VehicleReference("drone-1", frame, LocalPoint2D(0, 0))
    small = Polygon([(-10, -10), (20, -10), (20, 20), (-10, 20)])
    with pytest.raises(ValueError, match="not completely covered"):
        GenericMissionDefinition(
            (item,),
            (reference,),
            (spec(item),),
            {"drone-1": small},
        )


# ---------------------------------------------------------------------------
# Pipeline execution and failure boundaries
# ---------------------------------------------------------------------------


def test_pipeline_runs_every_layer(frame, free_space) -> None:
    definition = one_vehicle_definition(frame, free_space)
    runner = FakePlannerRunner()
    result = run_generic_mission_pipeline(definition, runner)

    assert result.algorithm == GENERIC_MISSION_PIPELINE_ALGORITHM
    assert len(runner.calls) == 1
    assert [request.request_id for request in runner.calls[0]] == [
        "request-a",
        "request-b",
    ]
    assert len(result.component_plans) == 1
    assert result.component_plans[0].component_ids == (
        "component-a",
        "component-b",
    )
    assert len(result.planning_results) == 2
    assert len(result.component_route_records) == 2
    assert len(result.complete_vehicle_routes) == 1
    assert len(result.complete_route_records) == 1
    assert len(result.ardupilot_missions) == 1
    assert result.active_vehicle_ids == ("drone-1",)
    assert result.idle_vehicle_ids == ()


def test_pipeline_is_independent_of_component_and_spec_input_order(frame, free_space) -> None:
    definition = one_vehicle_definition(frame, free_space)
    reversed_definition = GenericMissionDefinition(
        components=tuple(reversed(definition.components)),
        vehicle_references=definition.vehicle_references,
        planning_specs=tuple(reversed(definition.planning_specs)),
        free_space_by_vehicle_id=definition.free_space_by_vehicle_id,
    )
    first = run_generic_mission_pipeline(definition, FakePlannerRunner())
    second = run_generic_mission_pipeline(reversed_definition, FakePlannerRunner())
    assert first.to_summary_dict() == second.to_summary_dict()


def test_pipeline_sorts_vehicles(frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        two_vehicle_definition(frame, free_space),
        FakePlannerRunner(),
    )
    assert [plan.vehicle_id for plan in result.component_plans] == [
        "drone-a",
        "drone-b",
    ]
    assert [mission.vehicle_id for mission in result.ardupilot_missions] == [
        "drone-a",
        "drone-b",
    ]


def test_pipeline_projects_explicit_anchors(frame, free_space) -> None:
    item = component(frame, "component-a", "drone-1", 10)
    definition = GenericMissionDefinition(
        components=(item,),
        vehicle_references=(
            VehicleReference("drone-1", frame, LocalPoint2D(0, 0)),
        ),
        planning_specs=(
            spec(
                item,
                start_anchor=LocalPoint2D(0, 1),
                goal_anchor=LocalPoint2D(30, 7),
                policy=StartGoalPolicyConfig(boundary_clearance_m=1.0),
            ),
        ),
        free_space_by_vehicle_id={"drone-1": free_space},
    )
    result = run_generic_mission_pipeline(definition, FakePlannerRunner())
    selection = result.start_goal_selections[0]
    assert selection.start == LocalPoint2D(11.0, 1.0)
    assert selection.goal == LocalPoint2D(17.0, 7.0)
    assert result.planning_requests[0].start == selection.start
    assert result.planning_requests[0].goal == selection.goal


def test_pipeline_uses_requested_altitude_and_overlap(frame, free_space) -> None:
    item = component(frame, "component-a", "drone-1", 10)
    custom = ComponentPlanningSpec(
        item.component_id,
        LocalPoint2D(11, 1),
        LocalPoint2D(17, 7),
        45.0,
        3.5,
        0.25,
    )
    definition = GenericMissionDefinition(
        (item,),
        (VehicleReference("drone-1", frame, LocalPoint2D(0, 0)),),
        (custom,),
        {"drone-1": free_space},
    )
    result = run_generic_mission_pipeline(definition, FakePlannerRunner())
    request = result.planning_requests[0]
    assert request.altitude_m == 45.0
    assert request.lateral_footprint_m == 3.5
    assert request.lateral_overlap == 0.25
    assert all(point.z_m == 45.0 for point in result.complete_route_records[0].waypoints)


def test_pipeline_uses_astar_for_blocked_transition(frame) -> None:
    free = Polygon(
        [(0, 0), (40, 0), (40, 20), (0, 20)],
        holes=[[(17, 2), (23, 2), (23, 18), (17, 18)]],
    )
    left = component(frame, "left", "drone-1", 4, y0=6)
    right = component(frame, "right", "drone-1", 28, y0=6)
    definition = GenericMissionDefinition(
        (left, right),
        (VehicleReference("drone-1", frame, LocalPoint2D(1, 10)),),
        (
            spec(left, start_anchor=LocalPoint2D(5, 10), goal_anchor=LocalPoint2D(12, 10)),
            spec(right, start_anchor=LocalPoint2D(29, 10), goal_anchor=LocalPoint2D(35, 10)),
        ),
        {"drone-1": free},
    )
    result = run_generic_mission_pipeline(definition, FakePlannerRunner())
    algorithms = [
        segment.algorithm
        for segment in result.complete_route_records[0].connector_segments
    ]
    assert VISIBILITY_ASTAR_ALGORITHM in algorithms


def test_pipeline_supports_return_to_reference_and_land(frame, free_space) -> None:
    config = GenericMissionPipelineConfig(
        vehicle_route=VehicleRouteAssemblyConfig(return_to_reference=True),
        ardupilot=ArduPilotMissionBuildConfig(
            end_action=END_ACTION_LAND_AT_REFERENCE
        ),
    )
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space),
        FakePlannerRunner(),
        config=config,
    )
    assert result.complete_route_records[0].return_to_reference is True
    assert result.ardupilot_missions[0].end_action == END_ACTION_LAND_AT_REFERENCE


@pytest.mark.parametrize("end_action", [END_ACTION_RTL, END_ACTION_NONE])
def test_pipeline_supports_non_landing_end_actions(frame, free_space, end_action) -> None:
    config = GenericMissionPipelineConfig(
        ardupilot=ArduPilotMissionBuildConfig(end_action=end_action)
    )
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space),
        FakePlannerRunner(),
        config=config,
    )
    assert result.ardupilot_missions[0].end_action == end_action


def test_pipeline_tracks_idle_vehicle_without_mission(frame, free_space) -> None:
    active = component(frame, "component-a", "drone-a", 10)
    definition = GenericMissionDefinition(
        components=(active,),
        vehicle_references=(
            VehicleReference("drone-b", frame, LocalPoint2D(60, 0)),
            VehicleReference("drone-a", frame, LocalPoint2D(0, 0)),
        ),
        planning_specs=(spec(active),),
        free_space_by_vehicle_id={
            "drone-a": free_space,
            "drone-b": free_space,
        },
    )
    result = run_generic_mission_pipeline(definition, FakePlannerRunner())
    assert result.active_vehicle_ids == ("drone-a",)
    assert result.idle_vehicle_ids == ("drone-b",)
    assert [mission.vehicle_id for mission in result.ardupilot_missions] == [
        "drone-a"
    ]


def test_pipeline_supports_all_idle_vehicles_without_calling_planner(frame, free_space) -> None:
    definition = GenericMissionDefinition(
        (),
        (
            VehicleReference("drone-b", frame, LocalPoint2D(1, 0)),
            VehicleReference("drone-a", frame, LocalPoint2D(0, 0)),
        ),
        (),
        {"drone-a": free_space, "drone-b": free_space},
    )
    runner = FakePlannerRunner()
    result = run_generic_mission_pipeline(definition, runner)
    assert runner.calls == []
    assert result.planning_requests == ()
    assert result.ardupilot_missions == ()
    assert result.idle_vehicle_ids == ("drone-a", "drone-b")


def test_pipeline_can_reject_idle_vehicles(frame, free_space) -> None:
    definition = GenericMissionDefinition(
        (),
        (VehicleReference("drone-1", frame, LocalPoint2D(0, 0)),),
        (),
        {"drone-1": free_space},
    )
    with pytest.raises(GenericMissionPipelineError) as error:
        run_generic_mission_pipeline(
            definition,
            FakePlannerRunner(),
            config=GenericMissionPipelineConfig(allow_idle_vehicles=False),
        )
    assert error.value.stage == "ardupilot_export"
    assert "drone-1" in str(error.value)


def test_pipeline_rejects_wrong_definition_type() -> None:
    with pytest.raises(GenericMissionPipelineError) as error:
        run_generic_mission_pipeline(object(), FakePlannerRunner())
    assert error.value.stage == "input_validation"


@pytest.mark.parametrize("runner", [None, object()])
def test_pipeline_rejects_runner_without_run(runner, frame, free_space) -> None:
    with pytest.raises(GenericMissionPipelineError) as error:
        run_generic_mission_pipeline(one_vehicle_definition(frame, free_space), runner)
    assert error.value.stage == "input_validation"


def test_pipeline_rejects_wrong_config_type(frame, free_space) -> None:
    with pytest.raises(GenericMissionPipelineError) as error:
        run_generic_mission_pipeline(
            one_vehicle_definition(frame, free_space),
            FakePlannerRunner(),
            config=object(),
        )
    assert error.value.stage == "input_validation"


def test_pipeline_wraps_request_building_failure(frame, free_space) -> None:
    item = component(frame, "component-a", "drone-1", 10)
    definition = GenericMissionDefinition(
        (item,),
        (VehicleReference("drone-1", frame, LocalPoint2D(0, 0)),),
        (
            spec(
                item,
                policy=StartGoalPolicyConfig(boundary_clearance_m=100.0),
            ),
        ),
        {"drone-1": free_space},
    )
    with pytest.raises(GenericMissionPipelineError) as error:
        run_generic_mission_pipeline(definition, FakePlannerRunner())
    assert error.value.stage == "request_building"


def test_pipeline_wraps_planner_exception(frame, free_space) -> None:
    with pytest.raises(GenericMissionPipelineError) as error:
        run_generic_mission_pipeline(
            one_vehicle_definition(frame, free_space),
            RaisingRunner(),
        )
    assert error.value.stage == "coverage_planning"
    assert "planner exploded" in str(error.value)


def test_pipeline_rejects_non_iterable_planner_result(frame, free_space) -> None:
    with pytest.raises(GenericMissionPipelineError) as error:
        run_generic_mission_pipeline(
            one_vehicle_definition(frame, free_space),
            NonIterableRunner(),
        )
    assert error.value.stage == "coverage_planning"
    assert "iterable" in str(error.value)


def test_pipeline_rejects_wrong_planner_result_type(frame, free_space) -> None:
    with pytest.raises(GenericMissionPipelineError) as error:
        run_generic_mission_pipeline(
            one_vehicle_definition(frame, free_space),
            WrongTypeRunner(),
        )
    assert error.value.stage == "coverage_planning"
    assert "CoveragePlanningResult" in str(error.value)


def test_pipeline_rejects_reordered_planner_results(frame, free_space) -> None:
    with pytest.raises(GenericMissionPipelineError) as error:
        run_generic_mission_pipeline(
            one_vehicle_definition(frame, free_space),
            ReversedResultRunner(),
        )
    assert error.value.stage == "coverage_planning"
    assert "match request order" in str(error.value)


def test_pipeline_wraps_ardupilot_minimum_altitude_failure(frame, free_space) -> None:
    definition = one_vehicle_definition(frame, free_space)
    config = GenericMissionPipelineConfig(
        ardupilot=ArduPilotMissionBuildConfig(
            minimum_relative_altitude_m=50.0
        )
    )
    with pytest.raises(GenericMissionPipelineError) as error:
        run_generic_mission_pipeline(
            definition,
            FakePlannerRunner(),
            config=config,
        )
    assert error.value.stage == "ardupilot_export"


# ---------------------------------------------------------------------------
# Result validation, summaries and artifact publication
# ---------------------------------------------------------------------------


def test_result_summary_contains_all_output_layers(frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space),
        FakePlannerRunner(),
    )
    summary = result.to_summary_dict()
    assert summary["algorithm"] == GENERIC_MISSION_PIPELINE_ALGORITHM
    assert summary["component_count"] == 2
    assert summary["vehicle_count"] == 1
    assert summary["planning_request_ids"] == ["request-a", "request-b"]
    assert len(summary["component_routes"]) == 2
    assert len(summary["complete_vehicle_routes"]) == 1
    assert summary["ardupilot_missions"][0]["end_action"] == END_ACTION_RTL


def test_manifest_is_deterministic(frame, free_space) -> None:
    first = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    second = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    assert first.manifest_json() == second.manifest_json()
    assert first.manifest_json().endswith("\n")


def test_manifest_lists_relative_artifact_paths(frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    manifest = result.manifest_dict()
    assert manifest["schema_version"] == GENERIC_MISSION_MANIFEST_SCHEMA_VERSION
    assert manifest["files"]["component_routes"]["request-a"].startswith(
        "component_routes/"
    )
    assert manifest["files"]["complete_routes"]["drone-1"].startswith(
        "complete_routes/"
    )
    assert manifest["files"]["qgc_wpl_110"]["drone-1"].endswith(
        ".waypoints"
    )


def test_write_artifacts_publishes_complete_bundle(tmp_path, frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    destination = tmp_path / "mission-output"
    returned = result.write_artifacts(destination)
    assert returned == destination
    assert destination.is_dir()
    assert (destination / "generic-mission-manifest.json").is_file()

    manifest = json.loads(
        (destination / "generic-mission-manifest.json").read_text()
    )
    for category in manifest["files"].values():
        for relative_path in category.values():
            assert (destination / relative_path).is_file()


def test_written_component_routes_round_trip(tmp_path, frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    destination = result.write_artifacts(tmp_path / "bundle")
    for source in result.component_route_records:
        restored = CoverageRouteRecord.read(
            destination / "component_routes" / source.filename
        )
        assert restored == source


def test_written_complete_routes_round_trip(tmp_path, frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    destination = result.write_artifacts(tmp_path / "bundle")
    for source in result.complete_route_records:
        restored = CompleteVehicleRouteRecord.read(
            destination / "complete_routes" / source.filename
        )
        assert restored == source


def test_written_missions_round_trip_both_formats(tmp_path, frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    destination = result.write_artifacts(tmp_path / "bundle")
    source = result.ardupilot_missions[0]
    restored_json = ArduPilotMission.read_json(
        destination / "ardupilot" / source.json_filename
    )
    restored_wpl = ArduPilotMission.read_qgc_wpl_110(
        destination / "ardupilot" / source.waypoint_filename,
        vehicle_id=source.vehicle_id,
    )
    assert restored_json == source
    assert restored_wpl.to_qgc_wpl_110() == source.to_qgc_wpl_110()


def test_write_artifacts_rejects_existing_destination(tmp_path, frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    destination = tmp_path / "existing"
    destination.mkdir()
    with pytest.raises(GenericMissionPipelineError) as error:
        result.write_artifacts(destination)
    assert error.value.stage == "artifact_write"


def test_write_artifacts_leaves_no_temporary_directory(tmp_path, frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    destination = tmp_path / "bundle"
    result.write_artifacts(destination)
    assert not [path for path in tmp_path.iterdir() if path.name.startswith(".bundle.")]


def test_idle_bundle_contains_route_record_but_no_mission_files(tmp_path, frame, free_space) -> None:
    definition = GenericMissionDefinition(
        (),
        (VehicleReference("drone-1", frame, LocalPoint2D(0, 0)),),
        (),
        {"drone-1": free_space},
    )
    result = run_generic_mission_pipeline(definition, FakePlannerRunner())
    destination = result.write_artifacts(tmp_path / "bundle")
    assert (destination / "complete_routes" / "drone-1.complete-route.json").is_file()
    assert list((destination / "ardupilot").iterdir()) == []
    manifest = json.loads((destination / "generic-mission-manifest.json").read_text())
    assert manifest["summary"]["idle_vehicle_ids"] == ["drone-1"]
    assert manifest["files"]["ardupilot_mission_json"] == {}


def test_result_rejects_wrong_algorithm(frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    with pytest.raises(ValueError, match="algorithm"):
        replace(result, algorithm="wrong")


def test_result_rejects_reordered_results(frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    with pytest.raises(ValueError, match="planning_results"):
        replace(result, planning_results=tuple(reversed(result.planning_results)))


def test_result_rejects_reordered_complete_records(frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        two_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    with pytest.raises(ValueError, match="complete_route_records"):
        replace(
            result,
            complete_route_records=tuple(reversed(result.complete_route_records)),
        )


def test_result_rejects_missing_active_mission(frame, free_space) -> None:
    result = run_generic_mission_pipeline(
        one_vehicle_definition(frame, free_space), FakePlannerRunner()
    )
    with pytest.raises(ValueError, match="ardupilot_missions"):
        replace(result, ardupilot_missions=())


def test_pipeline_does_not_mutate_definition_mapping(frame, free_space) -> None:
    mapping = {"drone-1": free_space}
    item = component(frame, "component-a", "drone-1", 10)
    definition = GenericMissionDefinition(
        (item,),
        (VehicleReference("drone-1", frame, LocalPoint2D(0, 0)),),
        (spec(item),),
        mapping,
    )
    mapping.clear()
    result = run_generic_mission_pipeline(definition, FakePlannerRunner())
    assert result.active_vehicle_ids == ("drone-1",)
