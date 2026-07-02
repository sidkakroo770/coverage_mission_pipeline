#!/usr/bin/env python3
"""Build strict Stage-21-compatible product inputs from KML/KMZ."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from pyproj import CRS, Transformer
import yaml
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.geometry.polygon import orient
from shapely.ops import transform

from .automatic_partitioning import (
    AutomaticPartitioningResult,
    create_equal_route_area_partitions,
)
from .kml_input import KmlMissionInput, load_kml_mission_input
from .mission_geometry_core import extract_polygon_components
from .swarm_mission_config import SwarmMissionOperationalConfig
from .swarm_partitions_adapter import adapt_swarm_partitions_payload


@dataclass(frozen=True)
class KmlProductArtifacts:
    mission_input: KmlMissionInput
    mission_output: dict[str, Any]
    operational_config: dict[str, Any]
    automatic_partitioning: AutomaticPartitioningResult | None


def _ring_coordinates(ring: Any) -> list[list[float]]:
    return [[float(x), float(y)] for x, y in ring.coords]


def _polygon_record(polygon: Polygon) -> dict[str, Any]:
    value = orient(polygon, sign=1.0)
    return {
        "exterior": _ring_coordinates(value.exterior),
        "holes": [_ring_coordinates(interior) for interior in value.interiors],
    }


def _geometry_records(geometry: BaseGeometry) -> list[dict[str, Any]]:
    return [_polygon_record(polygon) for polygon in extract_polygon_components(geometry)]


def _to_wgs84_geometry(
    geometry: BaseGeometry,
    planning_crs: str,
) -> BaseGeometry:
    transformer = Transformer.from_crs(planning_crs, "EPSG:4326", always_xy=True)
    return transform(transformer.transform, geometry)


def _operational_config_dict(
    mission: KmlMissionInput,
    *,
    altitude_m: float,
    lateral_footprint_m: float,
    lateral_overlap: float,
    clearance_m: float,
    tracking_margin_m: float,
    min_component_area_m2: float,
) -> dict[str, Any]:
    vehicles = []
    assignments = []
    for index in range(1, 6):
        vehicle_id = f"drone-{index}"
        assignments.append({"partition_id": index, "vehicle_id": vehicle_id})
        vehicles.append(
            {
                "vehicle_id": vehicle_id,
                "reference": {
                    "type": "home",
                    "longitude_deg": mission.home_longitude_deg,
                    "latitude_deg": mission.home_latitude_deg,
                },
                "coverage": {
                    "altitude_m": altitude_m,
                    "lateral_footprint_m": lateral_footprint_m,
                    "lateral_overlap": lateral_overlap,
                    "start_goal_boundary_clearance_m": 0.0,
                    "minimum_start_goal_separation_m": 0.0,
                },
            }
        )
    return {
        "schema_version": 2,
        "adapter": {
            "frame_id": "map",
            "clearance_m": clearance_m,
            "tracking_margin_m": tracking_margin_m,
            "min_component_area_m2": min_component_area_m2,
            "coverage_gap_tolerance_m2": max(min_component_area_m2, 1.0e-4),
            "coverage_gap_relative_tolerance": 1.0e-9,
            "partition_overlap_tolerance_m2": 1.0e-6,
        },
        "assignments": assignments,
        "vehicles": vehicles,
        "pipeline": {
            "allow_idle_vehicles": False,
            "route": {
                "return_to_reference": True,
                "connector": {"max_visibility_nodes": 512},
            },
            "ardupilot": {
                "end_action": "land_at_reference",
                "waypoint_hold_s": 0.0,
                "include_takeoff": True,
                "skip_initial_reference_waypoint": True,
                "minimum_relative_altitude_m": 1.0,
            },
        },
    }


def build_kml_product_artifacts(
    path: Path | str,
    *,
    clearance_m: float = 10.0,
    tracking_margin_m: float = 2.0,
    altitude_m: float = 20.0,
    lateral_footprint_m: float = 14.9553871794,
    lateral_overlap: float = 0.2,
    min_component_area_m2: float = 250.0,
    random_seed: int = 42,
) -> KmlProductArtifacts:
    mission = load_kml_mission_input(
        path,
        clearance_m=clearance_m,
        tracking_margin_m=tracking_margin_m,
        expected_partition_count=5,
    )

    automatic: AutomaticPartitioningResult | None = None
    if mission.supplied_partitions_wgs84:
        partition_geometries_wgs84 = dict(mission.supplied_partitions_wgs84)
    else:
        automatic = create_equal_route_area_partitions(
            mission.boundary_projected,
            mission.route_space_projected,
            partition_count=5,
        )
        partition_geometries_wgs84 = {
            index: _to_wgs84_geometry(geometry, mission.planning_crs)
            for index, geometry in enumerate(automatic.partitions_projected, start=1)
        }

    payload = {
        "metadata": {
            "crs": {
                "coordinates": "EPSG:4326",
                "axis_order": ["longitude", "latitude"],
                "planning": CRS.from_user_input(mission.planning_crs).to_string(),
            },
            "n_partitions": 5,
            "generation": {"random_seed": random_seed},
        },
        "boundary": [_polygon_record(mission.boundary_wgs84)],
        "partitions": [
            {
                "id": partition_id,
                "geometry": _geometry_records(partition_geometries_wgs84[partition_id]),
            }
            for partition_id in range(1, 6)
        ],
        "no_go_zones": {
            "predetermined": [
                {"name": name, "geometry": _geometry_records(geometry)}
                for name, geometry in mission.exclusions_wgs84
            ]
        },
    }

    config = _operational_config_dict(
        mission,
        altitude_m=altitude_m,
        lateral_footprint_m=lateral_footprint_m,
        lateral_overlap=lateral_overlap,
        clearance_m=clearance_m,
        tracking_margin_m=tracking_margin_m,
        min_component_area_m2=min_component_area_m2,
    )

    validated_config = SwarmMissionOperationalConfig.from_dict(config)
    adapt_swarm_partitions_payload(payload, validated_config.adapter)

    return KmlProductArtifacts(
        mission_input=mission,
        mission_output=payload,
        operational_config=validated_config.to_dict(),
        automatic_partitioning=automatic,
    )


def write_kml_product_artifacts(
    artifacts: KmlProductArtifacts,
    output_directory: Path | str,
) -> Path:
    destination = Path(output_directory).expanduser().resolve()
    destination.mkdir(parents=True, exist_ok=True)

    (destination / "mission_output.json").write_text(
        json.dumps(artifacts.mission_output, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (destination / "swarm_mission.yaml").write_text(
        yaml.safe_dump(
            artifacts.operational_config,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        ),
        encoding="utf-8",
    )

    summary = {
        "source": str(artifacts.mission_input.source_path),
        "planning_crs": artifacts.mission_input.planning_crs,
        "home": {
            "longitude_deg": artifacts.mission_input.home_longitude_deg,
            "latitude_deg": artifacts.mission_input.home_latitude_deg,
            "explicit": artifacts.mission_input.home_was_explicit,
        },
        "boundary_area_m2": artifacts.mission_input.boundary_projected.area,
        "safe_area_m2": artifacts.mission_input.safe_area_projected.area,
        "route_space_m2": artifacts.mission_input.route_space_projected.area,
        "exclusion_count": artifacts.mission_input.exclusion_count,
        "partitions_supplied": artifacts.mission_input.supplied_partition_count == 5,
        "automatic_partition_rotation_degrees": (
            None
            if artifacts.automatic_partitioning is None
            else artifacts.automatic_partitioning.rotation_degrees
        ),
        "route_area_by_partition_m2": (
            None
            if artifacts.automatic_partitioning is None
            else list(artifacts.automatic_partitioning.route_area_by_partition_m2)
        ),
    }
    (destination / "input-summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return destination
