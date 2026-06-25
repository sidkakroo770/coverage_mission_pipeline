#!/usr/bin/env python3
"""Generic end-to-end orchestration above prepared mission geometry.

This module deliberately does not parse any mission-specific input schema.  A
caller supplies validated prepared components, explicit vehicle references,
explicit per-component planning specifications, and authoritative per-vehicle
free-space geometry.  The pipeline then executes the existing layers in one
fail-closed deterministic flow:

    component ordering -> start/goal selection -> ROS planner batch
    -> route records -> vehicle route assembly -> complete route records
    -> ArduPilot missions

The planner is injected through a tiny ``run(requests)`` protocol.  The real
``SequentialPlanCoverageRunner`` satisfies that protocol, while tests can use a
pure-Python fake runner without a ROS graph.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
from types import MappingProxyType
from typing import Any, Iterable, Mapping, Optional, Protocol, Sequence

from shapely.geometry.base import BaseGeometry

from .ardupilot_mission import (
    ArduPilotMission,
    ArduPilotMissionBuildConfig,
    build_ardupilot_mission,
)
from .complete_vehicle_route_record import (
    CompleteVehicleRouteRecord,
    make_complete_vehicle_route_records,
)
from .planning_request import CoveragePlanningRequest, LocalPoint2D
from .planning_result import CoveragePlanningResult
from .prepared_component import PreparedComponent
from .route_record import CoverageRouteRecord, make_route_records
from .start_goal_policy import (
    StartGoalPolicyConfig,
    StartGoalSelection,
    planning_request_from_anchors,
)
from .vehicle_component_ordering import (
    VehicleComponentPlan,
    VehicleReference,
    order_components_by_vehicle,
)
from .vehicle_route_assembly import (
    CompleteVehicleRoute,
    VehicleRouteAssemblyConfig,
    assemble_vehicle_route,
)

GENERIC_MISSION_PIPELINE_ALGORITHM = "generic_end_to_end_pipeline_v1"
GENERIC_MISSION_MANIFEST_SCHEMA_VERSION = 1
_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class GenericMissionPipelineError(RuntimeError):
    """Fail-closed pipeline error annotated with the failed stage."""

    def __init__(self, stage: str, message: str) -> None:
        if not isinstance(stage, str) or not stage:
            raise ValueError("stage must be a non-empty string")
        if not isinstance(message, str) or not message:
            raise ValueError("message must be a non-empty string")
        super().__init__(f"{stage}: {message}")
        self.stage = stage
        self.detail = message


class PlanningBatchRunner(Protocol):
    """Minimal planner interface required by the generic pipeline."""

    def run(
        self,
        requests: Iterable[CoveragePlanningRequest],
    ) -> Sequence[CoveragePlanningResult]:
        ...


def _identifier(value: Any, path: str) -> str:
    if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
        raise ValueError(f"{path} must match {_ID_PATTERN.pattern!r}")
    return value


def _finite_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{path} must be finite")
    return result


def _run_stage(stage: str, operation: Any) -> Any:
    try:
        return operation()
    except GenericMissionPipelineError:
        raise
    except Exception as exc:
        raise GenericMissionPipelineError(stage, str(exc)) from exc


@dataclass(frozen=True)
class ComponentPlanningSpec:
    """Explicit planner parameters and reference anchors for one component."""

    component_id: str
    start_anchor: LocalPoint2D
    goal_anchor: LocalPoint2D
    altitude_m: float
    lateral_footprint_m: float
    lateral_overlap: float
    request_id: Optional[str] = None
    start_goal_policy: StartGoalPolicyConfig = field(
        default_factory=StartGoalPolicyConfig
    )

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "component_id",
            _identifier(self.component_id, "component_id"),
        )
        if not isinstance(self.start_anchor, LocalPoint2D):
            raise ValueError("start_anchor must be a LocalPoint2D")
        if not isinstance(self.goal_anchor, LocalPoint2D):
            raise ValueError("goal_anchor must be a LocalPoint2D")

        altitude = _finite_number(self.altitude_m, "altitude_m")
        footprint = _finite_number(
            self.lateral_footprint_m,
            "lateral_footprint_m",
        )
        overlap = _finite_number(self.lateral_overlap, "lateral_overlap")
        if footprint <= 0.0:
            raise ValueError("lateral_footprint_m must be greater than zero")
        if overlap < 0.0 or overlap >= 1.0:
            raise ValueError("lateral_overlap must be in the range [0, 1)")
        object.__setattr__(self, "altitude_m", altitude)
        object.__setattr__(self, "lateral_footprint_m", footprint)
        object.__setattr__(self, "lateral_overlap", overlap)

        if self.request_id is not None:
            object.__setattr__(
                self,
                "request_id",
                _identifier(self.request_id, "request_id"),
            )
        if not isinstance(self.start_goal_policy, StartGoalPolicyConfig):
            raise ValueError(
                "start_goal_policy must be a StartGoalPolicyConfig"
            )

    @property
    def resolved_request_id(self) -> str:
        return self.component_id if self.request_id is None else self.request_id

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "component_id": self.component_id,
            "request_id": self.resolved_request_id,
            "start_anchor": self.start_anchor.to_dict(),
            "goal_anchor": self.goal_anchor.to_dict(),
            "altitude_m": self.altitude_m,
            "lateral_footprint_m": self.lateral_footprint_m,
            "lateral_overlap": self.lateral_overlap,
            "start_goal_policy": {
                "boundary_clearance_m": (
                    self.start_goal_policy.boundary_clearance_m
                ),
                "minimum_start_goal_separation_m": (
                    self.start_goal_policy.minimum_start_goal_separation_m
                ),
            },
        }


@dataclass(frozen=True)
class GenericMissionDefinition:
    """Schema-independent, fully explicit inputs for one pipeline run."""

    components: tuple[PreparedComponent, ...]
    vehicle_references: tuple[VehicleReference, ...]
    planning_specs: tuple[ComponentPlanningSpec, ...]
    free_space_by_vehicle_id: Mapping[str, BaseGeometry]

    def __post_init__(self) -> None:
        try:
            components = tuple(self.components)
        except TypeError as exc:
            raise ValueError("components must be iterable") from exc
        try:
            references = tuple(self.vehicle_references)
        except TypeError as exc:
            raise ValueError("vehicle_references must be iterable") from exc
        try:
            specs = tuple(self.planning_specs)
        except TypeError as exc:
            raise ValueError("planning_specs must be iterable") from exc
        if not isinstance(self.free_space_by_vehicle_id, Mapping):
            raise ValueError("free_space_by_vehicle_id must be a mapping")
        free_spaces = dict(self.free_space_by_vehicle_id)

        if not references:
            raise ValueError("vehicle_references must not be empty")
        if any(not isinstance(item, PreparedComponent) for item in components):
            raise ValueError(
                "components must contain only PreparedComponent objects"
            )
        if any(not isinstance(item, VehicleReference) for item in references):
            raise ValueError(
                "vehicle_references must contain only VehicleReference objects"
            )
        if any(not isinstance(item, ComponentPlanningSpec) for item in specs):
            raise ValueError(
                "planning_specs must contain only ComponentPlanningSpec objects"
            )

        component_ids = [item.component_id for item in components]
        vehicle_ids = [item.vehicle_id for item in references]
        spec_component_ids = [item.component_id for item in specs]
        request_ids = [item.resolved_request_id for item in specs]
        if len(component_ids) != len(set(component_ids)):
            raise ValueError("component IDs must be unique")
        if len(vehicle_ids) != len(set(vehicle_ids)):
            raise ValueError("vehicle reference IDs must be unique")
        if len(spec_component_ids) != len(set(spec_component_ids)):
            raise ValueError("planning spec component IDs must be unique")
        if len(request_ids) != len(set(request_ids)):
            raise ValueError("planning request IDs must be unique")

        if set(spec_component_ids) != set(component_ids) or len(specs) != len(
            components
        ):
            missing = sorted(set(component_ids) - set(spec_component_ids))
            extra = sorted(set(spec_component_ids) - set(component_ids))
            details: list[str] = []
            if missing:
                details.append("missing: " + ", ".join(missing))
            if extra:
                details.append("unexpected: " + ", ".join(extra))
            suffix = "" if not details else " (" + "; ".join(details) + ")"
            raise ValueError(
                "planning_specs must contain exactly one spec per component"
                + suffix
            )

        if set(free_spaces) != set(vehicle_ids):
            missing = sorted(set(vehicle_ids) - set(free_spaces))
            extra = sorted(set(free_spaces) - set(vehicle_ids))
            details = []
            if missing:
                details.append("missing: " + ", ".join(missing))
            if extra:
                details.append("unexpected: " + ", ".join(extra))
            suffix = "" if not details else " (" + "; ".join(details) + ")"
            raise ValueError(
                "free_space_by_vehicle_id must contain exactly one entry per "
                "vehicle reference" + suffix
            )

        references_by_id = {item.vehicle_id: item for item in references}
        for vehicle_id, geometry in free_spaces.items():
            _identifier(vehicle_id, "free_space vehicle ID")
            if not isinstance(geometry, BaseGeometry):
                raise ValueError(
                    f"free space for vehicle {vehicle_id!r} must be a Shapely geometry"
                )
            if geometry.is_empty:
                raise ValueError(
                    f"free space for vehicle {vehicle_id!r} must not be empty"
                )
            if not geometry.is_valid:
                raise ValueError(
                    f"free space for vehicle {vehicle_id!r} must be valid"
                )
            if geometry.area <= 0.0:
                raise ValueError(
                    f"free space for vehicle {vehicle_id!r} must have positive area"
                )

        for component in components:
            vehicle_id = component.assigned_vehicle_id
            if vehicle_id is None:
                raise ValueError(
                    f"component {component.component_id!r} has no vehicle assignment"
                )
            if vehicle_id not in references_by_id:
                raise ValueError(
                    f"component {component.component_id!r} references unknown "
                    f"vehicle {vehicle_id!r}"
                )
            reference = references_by_id[vehicle_id]
            if component.frame != reference.frame:
                raise ValueError(
                    f"component {component.component_id!r} frame does not match "
                    "its vehicle reference"
                )
            if not free_spaces[vehicle_id].covers(component.polygon):
                raise ValueError(
                    f"component {component.component_id!r} is not completely "
                    f"covered by vehicle {vehicle_id!r} free space"
                )

        object.__setattr__(self, "components", components)
        object.__setattr__(self, "vehicle_references", references)
        object.__setattr__(self, "planning_specs", specs)
        object.__setattr__(
            self,
            "free_space_by_vehicle_id",
            MappingProxyType(free_spaces),
        )

    @property
    def component_ids(self) -> tuple[str, ...]:
        return tuple(component.component_id for component in self.components)

    @property
    def vehicle_ids(self) -> tuple[str, ...]:
        return tuple(
            sorted(reference.vehicle_id for reference in self.vehicle_references)
        )


@dataclass(frozen=True)
class GenericMissionPipelineConfig:
    """Shared policies for route assembly and ArduPilot export."""

    vehicle_route: VehicleRouteAssemblyConfig = field(
        default_factory=VehicleRouteAssemblyConfig
    )
    ardupilot: ArduPilotMissionBuildConfig = field(
        default_factory=ArduPilotMissionBuildConfig
    )
    allow_idle_vehicles: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.vehicle_route, VehicleRouteAssemblyConfig):
            raise ValueError(
                "vehicle_route must be a VehicleRouteAssemblyConfig"
            )
        if not isinstance(self.ardupilot, ArduPilotMissionBuildConfig):
            raise ValueError("ardupilot must be an ArduPilotMissionBuildConfig")
        if not isinstance(self.allow_idle_vehicles, bool):
            raise ValueError("allow_idle_vehicles must be a bool")


@dataclass(frozen=True)
class GenericMissionPipelineResult:
    """Complete immutable outputs from one successful generic pipeline run."""

    component_plans: tuple[VehicleComponentPlan, ...]
    planning_requests: tuple[CoveragePlanningRequest, ...]
    start_goal_selections: tuple[StartGoalSelection, ...]
    planning_results: tuple[CoveragePlanningResult, ...]
    component_route_records: tuple[CoverageRouteRecord, ...]
    complete_vehicle_routes: tuple[CompleteVehicleRoute, ...]
    complete_route_records: tuple[CompleteVehicleRouteRecord, ...]
    ardupilot_missions: tuple[ArduPilotMission, ...]
    algorithm: str = GENERIC_MISSION_PIPELINE_ALGORITHM

    def __post_init__(self) -> None:
        if self.algorithm != GENERIC_MISSION_PIPELINE_ALGORITHM:
            raise ValueError(
                f"algorithm must be {GENERIC_MISSION_PIPELINE_ALGORITHM!r}"
            )
        fields_and_types = (
            ("component_plans", VehicleComponentPlan),
            ("planning_requests", CoveragePlanningRequest),
            ("start_goal_selections", StartGoalSelection),
            ("planning_results", CoveragePlanningResult),
            ("component_route_records", CoverageRouteRecord),
            ("complete_vehicle_routes", CompleteVehicleRoute),
            ("complete_route_records", CompleteVehicleRouteRecord),
            ("ardupilot_missions", ArduPilotMission),
        )
        for name, expected_type in fields_and_types:
            try:
                values = tuple(getattr(self, name))
            except TypeError as exc:
                raise ValueError(f"{name} must be iterable") from exc
            if any(not isinstance(item, expected_type) for item in values):
                raise ValueError(
                    f"{name} must contain only {expected_type.__name__} objects"
                )
            object.__setattr__(self, name, values)

        request_keys = tuple(
            (item.request_id, item.component.component_id)
            for item in self.planning_requests
        )
        selection_components = tuple(
            item.component_id for item in self.start_goal_selections
        )
        result_keys = tuple(
            (item.request_id, item.component_id) for item in self.planning_results
        )
        record_keys = tuple(
            (item.request_id, item.component_id)
            for item in self.component_route_records
        )
        if len(request_keys) != len(set(request_keys)):
            raise ValueError("planning request identities must be unique")
        if selection_components != tuple(key[1] for key in request_keys):
            raise ValueError(
                "start_goal_selections must match planning request order"
            )
        if result_keys != request_keys:
            raise ValueError("planning_results must match planning request order")
        if record_keys != request_keys:
            raise ValueError(
                "component_route_records must match planning request order"
            )

        plan_vehicle_ids = tuple(plan.vehicle_id for plan in self.component_plans)
        complete_vehicle_ids = tuple(
            route.vehicle_id for route in self.complete_vehicle_routes
        )
        record_vehicle_ids = tuple(
            record.vehicle_id for record in self.complete_route_records
        )
        if plan_vehicle_ids != tuple(sorted(plan_vehicle_ids)):
            raise ValueError("component plans must be sorted by vehicle ID")
        if complete_vehicle_ids != plan_vehicle_ids:
            raise ValueError(
                "complete_vehicle_routes must match component plan vehicle order"
            )
        if record_vehicle_ids != plan_vehicle_ids:
            raise ValueError(
                "complete_route_records must match component plan vehicle order"
            )

        active_vehicle_ids = tuple(
            record.vehicle_id
            for record in self.complete_route_records
            if not record.is_idle
        )
        mission_vehicle_ids = tuple(
            mission.vehicle_id for mission in self.ardupilot_missions
        )
        if mission_vehicle_ids != active_vehicle_ids:
            raise ValueError(
                "ardupilot_missions must contain every active vehicle exactly once"
            )

    @property
    def active_vehicle_ids(self) -> tuple[str, ...]:
        return tuple(
            record.vehicle_id
            for record in self.complete_route_records
            if not record.is_idle
        )

    @property
    def idle_vehicle_ids(self) -> tuple[str, ...]:
        return tuple(
            record.vehicle_id
            for record in self.complete_route_records
            if record.is_idle
        )

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "algorithm": self.algorithm,
            "component_count": len(self.planning_requests),
            "vehicle_count": len(self.component_plans),
            "active_vehicle_ids": list(self.active_vehicle_ids),
            "idle_vehicle_ids": list(self.idle_vehicle_ids),
            "planning_request_ids": [
                request.request_id for request in self.planning_requests
            ],
            "component_routes": [
                {
                    "request_id": record.request_id,
                    "component_id": record.component_id,
                    "source_region_id": record.source_region_id,
                    "assigned_vehicle_id": record.assigned_vehicle_id,
                    "frame_id": record.frame.frame_id,
                    "response_message": record.response_message,
                    "waypoint_count": len(record.waypoints),
                    "first_waypoint": record.waypoints[0].to_dict(),
                    "last_waypoint": record.waypoints[-1].to_dict(),
                }
                for record in self.component_route_records
            ],
            "complete_vehicle_routes": [
                record.to_summary_dict() for record in self.complete_route_records
            ],
            "ardupilot_missions": [
                {
                    "vehicle_id": mission.vehicle_id,
                    "item_count": len(mission.items),
                    "waypoint_count": len(mission.waypoint_items),
                    "end_action": mission.end_action,
                }
                for mission in self.ardupilot_missions
            ],
        }

    def manifest_dict(self) -> dict[str, Any]:
        component_files = {
            record.request_id: f"component_routes/{record.filename}"
            for record in self.component_route_records
        }
        complete_files = {
            record.vehicle_id: f"complete_routes/{record.filename}"
            for record in self.complete_route_records
        }
        mission_json_files = {
            mission.vehicle_id: f"ardupilot/{mission.json_filename}"
            for mission in self.ardupilot_missions
        }
        waypoint_files = {
            mission.vehicle_id: f"ardupilot/{mission.waypoint_filename}"
            for mission in self.ardupilot_missions
        }
        return {
            "schema_version": GENERIC_MISSION_MANIFEST_SCHEMA_VERSION,
            "algorithm": self.algorithm,
            "summary": self.to_summary_dict(),
            "files": {
                "component_routes": component_files,
                "complete_routes": complete_files,
                "ardupilot_mission_json": mission_json_files,
                "qgc_wpl_110": waypoint_files,
            },
        }

    def manifest_json(self) -> str:
        return json.dumps(
            self.manifest_dict(),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        ) + "\n"

    def write_artifacts(self, destination: Path | str) -> Path:
        """Atomically publish one complete output directory.

        The destination must not already exist.  All files are first written to
        a sibling temporary directory and the directory is renamed into place
        only after every artifact and the manifest have been created.
        """
        target = Path(destination)
        if target.exists():
            raise GenericMissionPipelineError(
                "artifact_write",
                f"destination already exists: {target}",
            )
        parent = target.parent
        parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(
            tempfile.mkdtemp(prefix=f".{target.name}.", dir=str(parent))
        )
        try:
            component_dir = temporary / "component_routes"
            complete_dir = temporary / "complete_routes"
            ardupilot_dir = temporary / "ardupilot"
            component_dir.mkdir()
            complete_dir.mkdir()
            ardupilot_dir.mkdir()

            for record in self.component_route_records:
                record.write(component_dir)
            for record in self.complete_route_records:
                record.write(complete_dir)
            for mission in self.ardupilot_missions:
                mission.write_json(ardupilot_dir)
                mission.write_qgc_wpl_110(ardupilot_dir)

            (temporary / "generic-mission-manifest.json").write_text(
                self.manifest_json(),
                encoding="utf-8",
            )
            os.replace(temporary, target)
        except Exception as exc:
            shutil.rmtree(temporary, ignore_errors=True)
            if isinstance(exc, GenericMissionPipelineError):
                raise
            raise GenericMissionPipelineError("artifact_write", str(exc)) from exc
        return target


def run_generic_mission_pipeline(
    definition: GenericMissionDefinition,
    planner_runner: PlanningBatchRunner,
    *,
    config: Optional[GenericMissionPipelineConfig] = None,
) -> GenericMissionPipelineResult:
    """Execute every generic mission layer in one deterministic fail-closed run."""
    if not isinstance(definition, GenericMissionDefinition):
        raise GenericMissionPipelineError(
            "input_validation",
            "definition must be a GenericMissionDefinition",
        )
    if planner_runner is None or not callable(getattr(planner_runner, "run", None)):
        raise GenericMissionPipelineError(
            "input_validation",
            "planner_runner must expose a callable run(requests) method",
        )
    if config is not None and not isinstance(config, GenericMissionPipelineConfig):
        raise GenericMissionPipelineError(
            "input_validation",
            "config must be a GenericMissionPipelineConfig",
        )
    policy = config or GenericMissionPipelineConfig()

    component_plans = _run_stage(
        "component_ordering",
        lambda: order_components_by_vehicle(
            definition.components,
            definition.vehicle_references,
        ),
    )
    specs_by_component = {
        spec.component_id: spec for spec in definition.planning_specs
    }

    requests: list[CoveragePlanningRequest] = []
    selections: list[StartGoalSelection] = []

    def build_requests() -> None:
        for plan in component_plans:
            for component in plan.ordered_components:
                spec = specs_by_component[component.component_id]
                request, selection = planning_request_from_anchors(
                    component,
                    start_anchor=spec.start_anchor,
                    goal_anchor=spec.goal_anchor,
                    altitude_m=spec.altitude_m,
                    lateral_footprint_m=spec.lateral_footprint_m,
                    lateral_overlap=spec.lateral_overlap,
                    request_id=spec.resolved_request_id,
                    policy_config=spec.start_goal_policy,
                )
                requests.append(request)
                selections.append(selection)

    _run_stage("request_building", build_requests)
    request_tuple = tuple(requests)
    selection_tuple = tuple(selections)

    if request_tuple:
        raw_results = _run_stage(
            "coverage_planning",
            lambda: planner_runner.run(request_tuple),
        )
        try:
            planning_results = tuple(raw_results)
        except TypeError as exc:
            raise GenericMissionPipelineError(
                "coverage_planning",
                "planner result must be iterable",
            ) from exc
        if any(
            not isinstance(item, CoveragePlanningResult)
            for item in planning_results
        ):
            raise GenericMissionPipelineError(
                "coverage_planning",
                "planner result must contain only CoveragePlanningResult objects",
            )
        expected = tuple(
            (request.request_id, request.component.component_id)
            for request in request_tuple
        )
        actual = tuple(
            (result.request_id, result.component_id)
            for result in planning_results
        )
        if actual != expected:
            raise GenericMissionPipelineError(
                "coverage_planning",
                "planner results must match request order and identity exactly",
            )
    else:
        planning_results = ()

    frames_by_component = {
        component.component_id: component.frame
        for component in definition.components
    }
    if planning_results:
        component_route_records = _run_stage(
            "component_route_recording",
            lambda: make_route_records(
                planning_results,
                frames_by_component,
            ),
        )
    else:
        component_route_records = ()

    records_by_component = {
        record.component_id: record for record in component_route_records
    }
    complete_routes: list[CompleteVehicleRoute] = []

    def assemble_routes() -> None:
        for plan in component_plans:
            ordered_records = tuple(
                records_by_component[component_id]
                for component_id in plan.component_ids
            )
            complete_routes.append(
                assemble_vehicle_route(
                    plan,
                    ordered_records,
                    definition.free_space_by_vehicle_id[plan.vehicle_id],
                    config=policy.vehicle_route,
                )
            )

    _run_stage("vehicle_route_assembly", assemble_routes)
    complete_route_tuple = tuple(complete_routes)
    complete_route_records = _run_stage(
        "complete_route_recording",
        lambda: make_complete_vehicle_route_records(complete_route_tuple),
    )

    idle_vehicle_ids = tuple(
        record.vehicle_id for record in complete_route_records if record.is_idle
    )
    if idle_vehicle_ids and not policy.allow_idle_vehicles:
        raise GenericMissionPipelineError(
            "ardupilot_export",
            "idle vehicles are not allowed: " + ", ".join(idle_vehicle_ids),
        )

    missions: list[ArduPilotMission] = []

    def build_missions() -> None:
        for record in complete_route_records:
            if record.is_idle:
                continue
            missions.append(
                build_ardupilot_mission(
                    record,
                    config=policy.ardupilot,
                )
            )

    _run_stage("ardupilot_export", build_missions)

    return _run_stage(
        "result_validation",
        lambda: GenericMissionPipelineResult(
            component_plans=component_plans,
            planning_requests=request_tuple,
            start_goal_selections=selection_tuple,
            planning_results=planning_results,
            component_route_records=component_route_records,
            complete_vehicle_routes=complete_route_tuple,
            complete_route_records=complete_route_records,
            ardupilot_missions=tuple(missions),
        ),
    )
