#!/usr/bin/env python3
"""Strict operational configuration for Swarm-Partitions mission runs.

The geometry exporter intentionally contains no vehicle homes or flight-planning
policy.  This module stores those operational values in a standalone JSON/YAML
contract and converts them into ``SwarmPartitionsAdapterConfig`` plus
``GenericMissionPipelineConfig``.  It performs no geometry parsing and no ROS
calls, so the same configuration can be validated before a planner is started.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
from typing import Any, Mapping

import yaml

from .ardupilot_mission import (
    END_ACTION_LAND_AT_REFERENCE,
    ArduPilotMissionBuildConfig,
    ArduPilotMissionError,
)
from .generic_mission_pipeline import GenericMissionPipelineConfig
from .route_connector import ConnectorPlannerConfig, ConnectorPlanningError
from .swarm_partitions_adapter import (
    SwarmPartitionAssignment,
    SwarmPartitionsAdapterConfig,
    SwarmPartitionsAdapterError,
    SwarmVehicleMissionProfile,
)
from .vehicle_route_assembly import (
    VehicleRouteAssemblyConfig,
    VehicleRouteAssemblyError,
)


SWARM_MISSION_CONFIG_SCHEMA_VERSION = 1
_SUPPORTED_SUFFIXES = frozenset({".json", ".yaml", ".yml"})


class SwarmMissionConfigError(ValueError):
    """Raised when an operational mission configuration is malformed or unsafe."""


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SwarmMissionConfigError(f"{path} must be an object")
    return value


def _strict_keys(value: Mapping[str, Any], required: set[str], path: str) -> None:
    actual = set(value.keys())
    missing = sorted(required - actual)
    unknown = sorted(actual - required)
    if missing:
        raise SwarmMissionConfigError(
            f"{path} is missing required field(s): {', '.join(missing)}"
        )
    if unknown:
        raise SwarmMissionConfigError(
            f"{path} contains unknown field(s): {', '.join(unknown)}"
        )


def _list(value: Any, path: str) -> list[Any]:
    if not isinstance(value, list):
        raise SwarmMissionConfigError(f"{path} must be an array")
    return value


def _integer(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise SwarmMissionConfigError(f"{path} must be an integer")
    return value


def _bool(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise SwarmMissionConfigError(f"{path} must be a bool")
    return value


def _construct(path: str, constructor: Any) -> Any:
    try:
        return constructor()
    except SwarmMissionConfigError:
        raise
    except (
        SwarmPartitionsAdapterError,
        ArduPilotMissionError,
        ConnectorPlanningError,
        VehicleRouteAssemblyError,
        ValueError,
        TypeError,
    ) as exc:
        raise SwarmMissionConfigError(f"{path} is invalid: {exc}") from exc


def _assignment_from_dict(value: Any, path: str) -> SwarmPartitionAssignment:
    item = _mapping(value, path)
    _strict_keys(item, {"partition_id", "vehicle_id"}, path)
    return _construct(
        path,
        lambda: SwarmPartitionAssignment(
            partition_id=item["partition_id"],
            vehicle_id=item["vehicle_id"],
        ),
    )


def _vehicle_from_dict(value: Any, path: str) -> SwarmVehicleMissionProfile:
    item = _mapping(value, path)
    _strict_keys(item, {"vehicle_id", "reference", "coverage"}, path)

    reference = _mapping(item["reference"], f"{path}.reference")
    _strict_keys(
        reference,
        {"type", "longitude_deg", "latitude_deg"},
        f"{path}.reference",
    )

    coverage = _mapping(item["coverage"], f"{path}.coverage")
    _strict_keys(
        coverage,
        {
            "altitude_m",
            "lateral_footprint_m",
            "lateral_overlap",
            "start_goal_boundary_clearance_m",
            "minimum_start_goal_separation_m",
        },
        f"{path}.coverage",
    )

    return _construct(
        path,
        lambda: SwarmVehicleMissionProfile(
            vehicle_id=item["vehicle_id"],
            reference_longitude_deg=reference["longitude_deg"],
            reference_latitude_deg=reference["latitude_deg"],
            reference_type=reference["type"],
            altitude_m=coverage["altitude_m"],
            lateral_footprint_m=coverage["lateral_footprint_m"],
            lateral_overlap=coverage["lateral_overlap"],
            start_goal_boundary_clearance_m=coverage[
                "start_goal_boundary_clearance_m"
            ],
            minimum_start_goal_separation_m=coverage[
                "minimum_start_goal_separation_m"
            ],
        ),
    )


def _adapter_from_dict(
    adapter_value: Any,
    assignments_value: Any,
    vehicles_value: Any,
) -> SwarmPartitionsAdapterConfig:
    adapter = _mapping(adapter_value, "adapter")
    _strict_keys(
        adapter,
        {
            "frame_id",
            "clearance_m",
            "min_component_area_m2",
            "coverage_gap_tolerance_m2",
            "coverage_gap_relative_tolerance",
            "partition_overlap_tolerance_m2",
        },
        "adapter",
    )

    assignments_raw = _list(assignments_value, "assignments")
    vehicles_raw = _list(vehicles_value, "vehicles")
    assignments = tuple(
        sorted(
            (
                _assignment_from_dict(item, f"assignments[{index}]")
                for index, item in enumerate(assignments_raw)
            ),
            key=lambda item: item.partition_id,
        )
    )
    vehicles = tuple(
        sorted(
            (
                _vehicle_from_dict(item, f"vehicles[{index}]")
                for index, item in enumerate(vehicles_raw)
            ),
            key=lambda item: item.vehicle_id,
        )
    )

    return _construct(
        "adapter",
        lambda: SwarmPartitionsAdapterConfig(
            assignments=assignments,
            vehicles=vehicles,
            frame_id=adapter["frame_id"],
            clearance_m=adapter["clearance_m"],
            min_component_area_m2=adapter["min_component_area_m2"],
            coverage_gap_tolerance_m2=adapter["coverage_gap_tolerance_m2"],
            coverage_gap_relative_tolerance=adapter[
                "coverage_gap_relative_tolerance"
            ],
            partition_overlap_tolerance_m2=adapter[
                "partition_overlap_tolerance_m2"
            ],
        ),
    )


def _pipeline_from_dict(value: Any) -> GenericMissionPipelineConfig:
    pipeline = _mapping(value, "pipeline")
    _strict_keys(pipeline, {"allow_idle_vehicles", "route", "ardupilot"}, "pipeline")

    route = _mapping(pipeline["route"], "pipeline.route")
    _strict_keys(route, {"return_to_reference", "connector"}, "pipeline.route")
    connector = _mapping(route["connector"], "pipeline.route.connector")
    _strict_keys(
        connector,
        {"max_visibility_nodes"},
        "pipeline.route.connector",
    )

    ardupilot = _mapping(pipeline["ardupilot"], "pipeline.ardupilot")
    _strict_keys(
        ardupilot,
        {
            "end_action",
            "waypoint_hold_s",
            "include_takeoff",
            "skip_initial_reference_waypoint",
            "minimum_relative_altitude_m",
        },
        "pipeline.ardupilot",
    )

    connector_config = _construct(
        "pipeline.route.connector",
        lambda: ConnectorPlannerConfig(
            max_visibility_nodes=connector["max_visibility_nodes"]
        ),
    )
    route_config = _construct(
        "pipeline.route",
        lambda: VehicleRouteAssemblyConfig(
            return_to_reference=_bool(
                route["return_to_reference"],
                "pipeline.route.return_to_reference",
            ),
            connector_config=connector_config,
        ),
    )
    ardupilot_config = _construct(
        "pipeline.ardupilot",
        lambda: ArduPilotMissionBuildConfig(
            end_action=ardupilot["end_action"],
            waypoint_hold_s=ardupilot["waypoint_hold_s"],
            include_takeoff=_bool(
                ardupilot["include_takeoff"],
                "pipeline.ardupilot.include_takeoff",
            ),
            skip_initial_reference_waypoint=_bool(
                ardupilot["skip_initial_reference_waypoint"],
                "pipeline.ardupilot.skip_initial_reference_waypoint",
            ),
            minimum_relative_altitude_m=ardupilot[
                "minimum_relative_altitude_m"
            ],
        ),
    )
    return _construct(
        "pipeline",
        lambda: GenericMissionPipelineConfig(
            vehicle_route=route_config,
            ardupilot=ardupilot_config,
            allow_idle_vehicles=_bool(
                pipeline["allow_idle_vehicles"],
                "pipeline.allow_idle_vehicles",
            ),
        ),
    )


@dataclass(frozen=True)
class SwarmMissionOperationalConfig:
    """Validated adapter and pipeline policies for one mission run."""

    adapter: SwarmPartitionsAdapterConfig
    pipeline: GenericMissionPipelineConfig

    def __post_init__(self) -> None:
        if not isinstance(self.adapter, SwarmPartitionsAdapterConfig):
            raise SwarmMissionConfigError(
                "adapter must be a SwarmPartitionsAdapterConfig"
            )
        if not isinstance(self.pipeline, GenericMissionPipelineConfig):
            raise SwarmMissionConfigError(
                "pipeline must be a GenericMissionPipelineConfig"
            )

        if (
            self.pipeline.ardupilot.end_action == END_ACTION_LAND_AT_REFERENCE
            and not self.pipeline.vehicle_route.return_to_reference
        ):
            raise SwarmMissionConfigError(
                "land_at_reference requires pipeline.route.return_to_reference=true"
            )

        minimum = self.pipeline.ardupilot.minimum_relative_altitude_m
        below = sorted(
            profile.vehicle_id
            for profile in self.adapter.vehicles
            if profile.altitude_m < minimum
        )
        if below:
            raise SwarmMissionConfigError(
                "vehicle altitude_m is below pipeline.ardupilot."
                "minimum_relative_altitude_m for: "
                + ", ".join(below)
            )

    @classmethod
    def from_dict(cls, value: Any) -> "SwarmMissionOperationalConfig":
        root = _mapping(value, "root")
        _strict_keys(
            root,
            {"schema_version", "adapter", "assignments", "vehicles", "pipeline"},
            "root",
        )
        version = _integer(root["schema_version"], "schema_version")
        if version != SWARM_MISSION_CONFIG_SCHEMA_VERSION:
            raise SwarmMissionConfigError(
                f"unsupported schema_version: {version!r}"
            )
        adapter = _adapter_from_dict(
            root["adapter"],
            root["assignments"],
            root["vehicles"],
        )
        pipeline = _pipeline_from_dict(root["pipeline"])
        return cls(adapter=adapter, pipeline=pipeline)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": SWARM_MISSION_CONFIG_SCHEMA_VERSION,
            "adapter": {
                "frame_id": self.adapter.frame_id,
                "clearance_m": self.adapter.clearance_m,
                "min_component_area_m2": self.adapter.min_component_area_m2,
                "coverage_gap_tolerance_m2": (
                    self.adapter.coverage_gap_tolerance_m2
                ),
                "coverage_gap_relative_tolerance": (
                    self.adapter.coverage_gap_relative_tolerance
                ),
                "partition_overlap_tolerance_m2": (
                    self.adapter.partition_overlap_tolerance_m2
                ),
            },
            "assignments": [
                {
                    "partition_id": item.partition_id,
                    "vehicle_id": item.vehicle_id,
                }
                for item in sorted(
                    self.adapter.assignments,
                    key=lambda item: item.partition_id,
                )
            ],
            "vehicles": [
                {
                    "vehicle_id": profile.vehicle_id,
                    "reference": {
                        "type": profile.reference_type,
                        "longitude_deg": profile.reference_longitude_deg,
                        "latitude_deg": profile.reference_latitude_deg,
                    },
                    "coverage": {
                        "altitude_m": profile.altitude_m,
                        "lateral_footprint_m": profile.lateral_footprint_m,
                        "lateral_overlap": profile.lateral_overlap,
                        "start_goal_boundary_clearance_m": (
                            profile.start_goal_boundary_clearance_m
                        ),
                        "minimum_start_goal_separation_m": (
                            profile.minimum_start_goal_separation_m
                        ),
                    },
                }
                for profile in sorted(
                    self.adapter.vehicles,
                    key=lambda item: item.vehicle_id,
                )
            ],
            "pipeline": {
                "allow_idle_vehicles": self.pipeline.allow_idle_vehicles,
                "route": {
                    "return_to_reference": (
                        self.pipeline.vehicle_route.return_to_reference
                    ),
                    "connector": {
                        "max_visibility_nodes": (
                            self.pipeline.vehicle_route.connector_config.
                            max_visibility_nodes
                        ),
                    },
                },
                "ardupilot": {
                    "end_action": self.pipeline.ardupilot.end_action,
                    "waypoint_hold_s": self.pipeline.ardupilot.waypoint_hold_s,
                    "include_takeoff": self.pipeline.ardupilot.include_takeoff,
                    "skip_initial_reference_waypoint": (
                        self.pipeline.ardupilot.skip_initial_reference_waypoint
                    ),
                    "minimum_relative_altitude_m": (
                        self.pipeline.ardupilot.minimum_relative_altitude_m
                    ),
                },
            },
        }

    @classmethod
    def from_json(cls, text: str) -> "SwarmMissionOperationalConfig":
        if not isinstance(text, str):
            raise SwarmMissionConfigError("JSON input must be text")
        try:
            value = json.loads(text)
        except json.JSONDecodeError as exc:
            raise SwarmMissionConfigError(
                f"invalid JSON at line {exc.lineno}, column {exc.colno}: {exc.msg}"
            ) from exc
        return cls.from_dict(value)

    def to_json(self) -> str:
        return json.dumps(
            self.to_dict(),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        ) + "\n"

    @classmethod
    def from_yaml(cls, text: str) -> "SwarmMissionOperationalConfig":
        if not isinstance(text, str):
            raise SwarmMissionConfigError("YAML input must be text")
        try:
            value = yaml.safe_load(text)
        except yaml.YAMLError as exc:
            raise SwarmMissionConfigError(f"invalid YAML: {exc}") from exc
        if value is None:
            raise SwarmMissionConfigError("YAML input must not be empty")
        return cls.from_dict(value)

    def to_yaml(self) -> str:
        return yaml.safe_dump(
            self.to_dict(),
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )

    @classmethod
    def read(cls, path: Path | str) -> "SwarmMissionOperationalConfig":
        source = Path(path)
        suffix = source.suffix.lower()
        if suffix not in _SUPPORTED_SUFFIXES:
            raise SwarmMissionConfigError(
                "configuration path must end in .json, .yaml or .yml"
            )
        try:
            text = source.read_text(encoding="utf-8")
        except OSError as exc:
            raise SwarmMissionConfigError(
                f"could not read operational configuration: {exc}"
            ) from exc
        if suffix == ".json":
            return cls.from_json(text)
        return cls.from_yaml(text)

    def write(self, path: Path | str) -> Path:
        destination = Path(path)
        suffix = destination.suffix.lower()
        if suffix not in _SUPPORTED_SUFFIXES:
            raise SwarmMissionConfigError(
                "configuration path must end in .json, .yaml or .yml"
            )
        destination.parent.mkdir(parents=True, exist_ok=True)
        text = self.to_json() if suffix == ".json" else self.to_yaml()
        temporary = destination.with_name(
            f".{destination.name}.{os.getpid()}.tmp"
        )
        try:
            temporary.write_text(text, encoding="utf-8")
            os.replace(temporary, destination)
        finally:
            if temporary.exists():
                temporary.unlink()
        return destination


def load_swarm_mission_operational_config(
    path: Path | str,
) -> SwarmMissionOperationalConfig:
    """Convenience wrapper used by the future ROS command-line entry point."""
    return SwarmMissionOperationalConfig.read(path)
