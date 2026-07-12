#!/usr/bin/env python3
"""
Generate QGroundControl/ArduPilot waypoint missions for N drones from one KML/KMZ.

The existing ROS 2 coverage pipeline performs the actual coverage planning,
safe connector construction, return-to-HOME routing, and ArduPilot export.

KML/KMZ naming:
  BOUNDARY             one connected mission polygon
  HOME                 one launch/home Point
  NO-GO ...            exclusion polygons
  PARTITION_1 ... N    optional pre-made partitions; either supply all 1..N or none

When partitions are not supplied:
  - N=1 uses the complete boundary.
  - N>1 creates deterministic principal-axis slabs balanced by operational
    route-space area.

Every final QGC WPL 110 file is normalized and verified as:
  row 0  synthetic HOME metadata
  row 1  MAV_CMD_NAV_TAKEOFF
  rows 2+ coverage/connector waypoints
  final  MAV_CMD_NAV_LAND at HOME

Examples:
  # Interactive inbox mode: place one KML/KMZ in ~/coverage_ws/kml_inbox
  python3 generate_waypoints.py --drones 5 --altitude 20 \
      --output-dir generated_waypoints

  # Direct path mode remains available
  python3 generate_waypoints.py mission.kml --drones 1 --altitude 2 \
      --output drone-1.waypoints
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time
from typing import Any

from pyproj import CRS, Transformer
import yaml
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.geometry.polygon import orient
from shapely.ops import transform


SCRIPT_VERSION = "2026-07-12.3"

QGC_HEADER = "QGC WPL 110"

MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22


class GenerationError(RuntimeError):
    """User-facing mission generation failure."""


def positive_int(value: str) -> int:
    try:
        number = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if number < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return number


def positive_float(value: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return number


def nonnegative_float(value: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(number) or number < 0.0:
        raise argparse.ArgumentTypeError("must be finite and non-negative")
    return number


def overlap_float(value: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(number) or not 0.0 <= number < 1.0:
        raise argparse.ArgumentTypeError("must be in the range [0, 1)")
    return number


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate one validated QGC WPL 110 .waypoints mission per drone "
            "from a KML/KMZ mission area."
        )
    )
    parser.add_argument(
        "kml",
        nargs="?",
        type=Path,
        help=(
            "Optional input .kml/.kmz path. When omitted, the script creates "
            "~/coverage_ws/kml_inbox and interactively reads a file from it."
        ),
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("~/coverage_ws/kml_inbox"),
        help=(
            "Interactive KML/KMZ inbox used when no positional file is supplied "
            "(default: ~/coverage_ws/kml_inbox)"
        ),
    )
    parser.add_argument(
        "-n",
        "--drones",
        type=positive_int,
        default=1,
        help="Number of drones/partitions (default: 1)",
    )
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path("~/coverage_ws"),
        help="ROS 2 workspace (default: ~/coverage_ws)",
    )
    output_group = parser.add_mutually_exclusive_group()
    output_group.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Single output file; valid only with --drones 1",
    )
    output_group.add_argument(
        "--output-dir",
        type=Path,
        help=(
            "Directory for drone-1.waypoints ... drone-N.waypoints "
            "(default: <KML stem>_waypoints)"
        ),
    )
    parser.add_argument(
        "--altitude",
        type=positive_float,
        default=2.0,
        help="HOME-relative flight/takeoff altitude in metres (default: 2.0)",
    )
    parser.add_argument(
        "--clearance",
        type=nonnegative_float,
        default=1.0,
        help="Boundary/exclusion clearance in metres (default: 1.0)",
    )
    parser.add_argument(
        "--tracking-margin",
        type=nonnegative_float,
        default=0.5,
        help="Additional operational tracking margin in metres (default: 0.5)",
    )
    parser.add_argument(
        "--footprint",
        type=positive_float,
        default=2.0,
        help="Lateral coverage footprint in metres (default: 2.0)",
    )
    parser.add_argument(
        "--overlap",
        type=overlap_float,
        default=0.1,
        help="Lateral overlap ratio (default: 0.1)",
    )
    parser.add_argument(
        "--min-component-area",
        type=nonnegative_float,
        default=0.0,
        help="Discardable component threshold in square metres (default: 0)",
    )
    parser.add_argument(
        "--max-visibility-nodes",
        type=positive_int,
        default=512,
        help="Connector visibility-graph node limit (default: 512)",
    )
    parser.add_argument(
        "--planner-timeout",
        type=positive_float,
        default=30.0,
        help="Seconds to wait for /plan_coverage (default: 30)",
    )
    parser.add_argument(
        "--keep-intermediates",
        action="store_true",
        help="Keep generated JSON, YAML, routes, reports, and raw exports",
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {SCRIPT_VERSION}",
    )
    return parser


def _add_pipeline_to_python_path(workspace: Path) -> None:
    """Make the source or symlink-installed package importable without shell setup."""
    candidates: list[Path] = [
        workspace / "src" / "coverage_mission_pipeline",
    ]
    candidates.extend(
        sorted(
            (
                workspace
                / "install"
                / "coverage_mission_pipeline"
                / "lib"
            ).glob("python*/site-packages")
        )
    )

    inserted = False
    for candidate in candidates:
        if (
            candidate.is_dir()
            and (candidate / "coverage_mission_pipeline").is_dir()
        ):
            value = str(candidate.resolve())
            if value not in sys.path:
                sys.path.insert(0, value)
            inserted = True

    if not inserted:
        raise GenerationError(
            "Could not find the coverage_mission_pipeline Python package under "
            f"{workspace}. Build the workspace first."
        )


def _load_pipeline_api(workspace: Path) -> dict[str, Any]:
    _add_pipeline_to_python_path(workspace)
    try:
        automatic = importlib.import_module(
            "coverage_mission_pipeline.automatic_partitioning"
        )
        kml_input = importlib.import_module(
            "coverage_mission_pipeline.kml_input"
        )
        geometry_core = importlib.import_module(
            "coverage_mission_pipeline.mission_geometry_core"
        )
        config_module = importlib.import_module(
            "coverage_mission_pipeline.swarm_mission_config"
        )
        adapter_module = importlib.import_module(
            "coverage_mission_pipeline.swarm_partitions_adapter"
        )
    except (ImportError, ModuleNotFoundError) as exc:
        raise GenerationError(
            "Could not import the built coverage_mission_pipeline package: "
            f"{exc}"
        ) from exc

    return {
        "create_equal_route_area_partitions":
            automatic.create_equal_route_area_partitions,
        "load_kml_mission_input": kml_input.load_kml_mission_input,
        "extract_polygon_components":
            geometry_core.extract_polygon_components,
        "SwarmMissionOperationalConfig":
            config_module.SwarmMissionOperationalConfig,
        "adapt_swarm_partitions_payload":
            adapter_module.adapt_swarm_partitions_payload,
    }


def _ring_coordinates(ring: Any) -> list[list[float]]:
    return [[float(x), float(y)] for x, y in ring.coords]


def _polygon_record(polygon: Polygon) -> dict[str, Any]:
    value = orient(polygon, sign=1.0)
    return {
        "exterior": _ring_coordinates(value.exterior),
        "holes": [
            _ring_coordinates(interior)
            for interior in value.interiors
        ],
    }


def _geometry_records(
    geometry: BaseGeometry,
    extract_polygon_components: Any,
) -> list[dict[str, Any]]:
    components = extract_polygon_components(geometry)
    if not components:
        raise GenerationError("Geometry contains no polygon components")
    return [_polygon_record(polygon) for polygon in components]


def _to_wgs84_geometry(
    geometry: BaseGeometry,
    planning_crs: str,
) -> BaseGeometry:
    transformer = Transformer.from_crs(
        planning_crs,
        "EPSG:4326",
        always_xy=True,
    )
    result = transform(transformer.transform, geometry)
    if result.is_empty or not result.is_valid:
        raise GenerationError(
            "Partition geometry became invalid during WGS84 conversion"
        )
    return result


def _build_operational_config(
    mission: Any,
    *,
    drone_count: int,
    altitude_m: float,
    lateral_footprint_m: float,
    lateral_overlap: float,
    clearance_m: float,
    tracking_margin_m: float,
    min_component_area_m2: float,
    max_visibility_nodes: int,
) -> dict[str, Any]:
    assignments: list[dict[str, Any]] = []
    vehicles: list[dict[str, Any]] = []

    for index in range(1, drone_count + 1):
        vehicle_id = f"drone-{index}"
        assignments.append(
            {
                "partition_id": index,
                "vehicle_id": vehicle_id,
            }
        )
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
            "coverage_gap_tolerance_m2": max(
                min_component_area_m2,
                1.0e-4,
            ),
            "coverage_gap_relative_tolerance": 1.0e-9,
            "partition_overlap_tolerance_m2": 1.0e-6,
        },
        "assignments": assignments,
        "vehicles": vehicles,
        "pipeline": {
            "allow_idle_vehicles": False,
            "route": {
                "return_to_reference": True,
                "connector": {
                    "max_visibility_nodes": max_visibility_nodes,
                },
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


def build_inputs(
    *,
    api: dict[str, Any],
    source: Path,
    drone_count: int,
    clearance_m: float,
    tracking_margin_m: float,
    altitude_m: float,
    lateral_footprint_m: float,
    lateral_overlap: float,
    min_component_area_m2: float,
    max_visibility_nodes: int,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """Create validated mission JSON and YAML payloads for arbitrary N."""
    load_kml_mission_input = api["load_kml_mission_input"]
    create_equal_route_area_partitions = (
        api["create_equal_route_area_partitions"]
    )
    extract_polygon_components = api["extract_polygon_components"]

    try:
        mission = load_kml_mission_input(
            source,
            clearance_m=clearance_m,
            tracking_margin_m=tracking_margin_m,
            expected_partition_count=drone_count,
        )
    except Exception as exc:
        raise GenerationError(f"KML/KMZ validation failed: {exc}") from exc

    automatic = None
    if mission.supplied_partitions_wgs84:
        supplied = dict(mission.supplied_partitions_wgs84)
        expected_ids = set(range(1, drone_count + 1))
        if set(supplied) != expected_ids:
            raise GenerationError(
                "Supplied partition IDs must be exactly "
                f"1..{drone_count}; found {sorted(supplied)}"
            )
        partition_geometries_wgs84 = supplied
        partition_mode = "supplied"
    elif drone_count == 1:
        partition_geometries_wgs84 = {
            1: mission.boundary_wgs84,
        }
        partition_mode = "single-boundary"
    else:
        try:
            automatic = create_equal_route_area_partitions(
                mission.boundary_projected,
                mission.route_space_projected,
                partition_count=drone_count,
            )
        except Exception as exc:
            raise GenerationError(
                f"Automatic {drone_count}-way partitioning failed: {exc}"
            ) from exc

        partition_geometries_wgs84 = {
            index: _to_wgs84_geometry(
                geometry,
                mission.planning_crs,
            )
            for index, geometry in enumerate(
                automatic.partitions_projected,
                start=1,
            )
        }
        partition_mode = "automatic-equal-route-area"

    mission_output = {
        "metadata": {
            "crs": {
                "coordinates": "EPSG:4326",
                "axis_order": ["longitude", "latitude"],
                "planning": CRS.from_user_input(
                    mission.planning_crs
                ).to_string(),
            },
            "n_partitions": drone_count,
            "generation": {
                "random_seed": 42,
            },
        },
        "boundary": [
            _polygon_record(mission.boundary_wgs84)
        ],
        "partitions": [
            {
                "id": partition_id,
                "geometry": _geometry_records(
                    partition_geometries_wgs84[partition_id],
                    extract_polygon_components,
                ),
            }
            for partition_id in range(1, drone_count + 1)
        ],
        "no_go_zones": {
            "predetermined": [
                {
                    "name": name,
                    "geometry": _geometry_records(
                        geometry,
                        extract_polygon_components,
                    ),
                }
                for name, geometry in mission.exclusions_wgs84
            ]
        },
    }

    config = _build_operational_config(
        mission,
        drone_count=drone_count,
        altitude_m=altitude_m,
        lateral_footprint_m=lateral_footprint_m,
        lateral_overlap=lateral_overlap,
        clearance_m=clearance_m,
        tracking_margin_m=tracking_margin_m,
        min_component_area_m2=min_component_area_m2,
        max_visibility_nodes=max_visibility_nodes,
    )

    try:
        validated_config = api[
            "SwarmMissionOperationalConfig"
        ].from_dict(config)
        api["adapt_swarm_partitions_payload"](
            mission_output,
            validated_config.adapter,
        )
    except Exception as exc:
        raise GenerationError(
            f"Generated N-drone input validation failed: {exc}"
        ) from exc

    summary = {
        "source": str(mission.source_path),
        "drone_count": drone_count,
        "partition_mode": partition_mode,
        "planning_crs": mission.planning_crs,
        "home": {
            "longitude_deg": mission.home_longitude_deg,
            "latitude_deg": mission.home_latitude_deg,
            "explicit": mission.home_was_explicit,
        },
        "boundary_area_m2": mission.boundary_projected.area,
        "safe_area_m2": mission.safe_area_projected.area,
        "route_space_m2": mission.route_space_projected.area,
        "exclusion_count": mission.exclusion_count,
        "automatic_partition_rotation_degrees": (
            None
            if automatic is None
            else automatic.rotation_degrees
        ),
        "route_area_by_partition_m2": (
            None
            if automatic is None
            else list(automatic.route_area_by_partition_m2)
        ),
    }

    return mission_output, validated_config.to_dict(), summary


def write_inputs(
    directory: Path,
    *,
    mission_output: dict[str, Any],
    config: dict[str, Any],
    summary: dict[str, Any],
) -> tuple[Path, Path]:
    directory.mkdir(parents=True, exist_ok=False)

    mission_json = directory / "mission_output.json"
    config_yaml = directory / "swarm_mission.yaml"

    mission_json.write_text(
        json.dumps(
            mission_output,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    config_yaml.write_text(
        yaml.safe_dump(
            config,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        ),
        encoding="utf-8",
    )
    (directory / "input-summary.json").write_text(
        json.dumps(
            summary,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )

    return mission_json, config_yaml


def _ros_shell(workspace: Path, command: str) -> str:
    humble = Path("/opt/ros/humble/setup.bash")
    overlay = workspace / "install" / "setup.bash"

    if not humble.is_file():
        raise GenerationError(f"ROS setup file not found: {humble}")
    if not overlay.is_file():
        raise GenerationError(
            f"Workspace overlay not found: {overlay}. "
            "Run colcon build first."
        )

    # Do not enable Bash nounset (-u): ROS/ament setup scripts legitimately
    # inspect variables which may not yet exist.
    return (
        "set -eo pipefail; "
        f"source {shlex.quote(str(humble))}; "
        f"source {shlex.quote(str(overlay))}; "
        f"{command}"
    )


def _service_ready(workspace: Path) -> bool:
    result = subprocess.run(
        [
            "bash",
            "-lc",
            _ros_shell(
                workspace,
                "ros2 service list 2>/dev/null "
                "| grep -Fx '/plan_coverage'",
            ),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def _stop_process_group(
    process: subprocess.Popen[Any] | None,
) -> None:
    if process is None or process.poll() is not None:
        return

    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass

    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=3)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass

    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def start_planner_if_needed(
    workspace: Path,
    timeout_s: float,
) -> tuple[subprocess.Popen[Any] | None, bool]:
    if _service_ready(workspace):
        print("Using existing /plan_coverage service.")
        return None, False

    print("Starting polygon_coverage_ros2 coverage_planner...")
    process = subprocess.Popen(
        [
            "bash",
            "-lc",
            _ros_shell(
                workspace,
                "exec ros2 run polygon_coverage_ros2 "
                "coverage_planner",
            ),
        ],
        start_new_session=True,
    )

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise GenerationError(
                "coverage_planner exited early with code "
                f"{process.returncode}"
            )
        if _service_ready(workspace):
            print("/plan_coverage is ready.")
            return process, True
        time.sleep(0.4)

    _stop_process_group(process)
    raise GenerationError(
        f"Timed out after {timeout_s:.1f}s waiting for /plan_coverage"
    )


def run_pipeline(
    workspace: Path,
    *,
    mission_json: Path,
    config_yaml: Path,
    run_directory: Path,
) -> None:
    command = " ".join(
        [
            "ros2 run coverage_mission_pipeline run_swarm_mission",
            "--mission-json",
            shlex.quote(str(mission_json)),
            "--config",
            shlex.quote(str(config_yaml)),
            "--output",
            shlex.quote(str(run_directory)),
        ]
    )

    result = subprocess.run(
        [
            "bash",
            "-lc",
            _ros_shell(workspace, command),
        ],
        check=False,
    )
    if result.returncode != 0:
        raise GenerationError(
            "coverage_mission_pipeline failed with exit code "
            f"{result.returncode}"
        )


def _parse_wpl_rows(text: str, source: Path) -> list[list[str]]:
    lines = [
        line.strip()
        for line in text.splitlines()
        if line.strip()
    ]
    if not lines or lines[0] != QGC_HEADER:
        raise GenerationError(
            f"{source} must begin with {QGC_HEADER!r}"
        )

    rows: list[list[str]] = []
    for line_number, line in enumerate(lines[1:], start=2):
        fields = line.split()
        if len(fields) != 12:
            raise GenerationError(
                f"{source}:{line_number} has {len(fields)} fields; "
                "expected 12"
            )
        try:
            int(fields[0])
            int(fields[1])
            int(fields[2])
            int(fields[3])
            for index in range(4, 11):
                float(fields[index])
            int(fields[11])
        except ValueError as exc:
            raise GenerationError(
                f"{source}:{line_number} contains invalid numeric data"
            ) from exc
        rows.append(fields)

    if not rows:
        raise GenerationError(f"{source} contains no waypoint rows")
    return rows


def normalize_qgc_waypoints(
    source: Path,
    destination: Path,
    *,
    home_longitude_deg: float,
    home_latitude_deg: float,
    altitude_m: float,
) -> tuple[int, int]:
    """Make HOME row 0 and TAKEOFF row 1, then fail-closed validate."""
    rows = _parse_wpl_rows(
        source.read_text(encoding="utf-8"),
        source,
    )

    first_is_home_metadata = (
        rows[0][0] == "0"
        and rows[0][1] == "1"
        and int(rows[0][3]) == MAV_CMD_NAV_WAYPOINT
    )
    semantic_rows = rows[1:] if first_is_home_metadata else rows[:]

    # Idempotently remove all prior TAKEOFF entries, then insert one canonical
    # command as the first real ArduPilot mission item.
    semantic_rows = [
        row
        for row in semantic_rows
        if int(row[3]) != MAV_CMD_NAV_TAKEOFF
    ]

    home_row = [
        "0",
        "1",
        str(MAV_FRAME_GLOBAL),
        str(MAV_CMD_NAV_WAYPOINT),
        "0.0000000000",
        "0.0000000000",
        "0.0000000000",
        "0.0000000000",
        f"{home_latitude_deg:.10f}",
        f"{home_longitude_deg:.10f}",
        "0.0000000000",
        "1",
    ]
    takeoff_row = [
        "1",
        "0",
        str(MAV_FRAME_GLOBAL_RELATIVE_ALT),
        str(MAV_CMD_NAV_TAKEOFF),
        "0.0000000000",
        "0.0000000000",
        "0.0000000000",
        "0.0000000000",
        f"{home_latitude_deg:.10f}",
        f"{home_longitude_deg:.10f}",
        f"{altitude_m:.10f}",
        "1",
    ]

    final_rows = [home_row, takeoff_row, *semantic_rows]
    for index, row in enumerate(final_rows):
        row[0] = str(index)
        row[1] = "1" if index == 0 else "0"
        row[11] = "1"

    commands = [int(row[3]) for row in final_rows]
    takeoff_indexes = [
        index
        for index, command in enumerate(commands)
        if command == MAV_CMD_NAV_TAKEOFF
    ]
    land_indexes = [
        index
        for index, command in enumerate(commands)
        if command == MAV_CMD_NAV_LAND
    ]
    rtl_indexes = [
        index
        for index, command in enumerate(commands)
        if command == MAV_CMD_NAV_RETURN_TO_LAUNCH
    ]
    navigation_count = sum(
        command == MAV_CMD_NAV_WAYPOINT
        for command in commands[2:]
    )

    if [int(value) for value in final_rows[0][:4]] != [
        0,
        1,
        MAV_FRAME_GLOBAL,
        MAV_CMD_NAV_WAYPOINT,
    ]:
        raise GenerationError(
            f"{source}: synthetic HOME row validation failed"
        )
    if takeoff_indexes != [1]:
        raise GenerationError(
            f"{source}: TAKEOFF must appear exactly at row 1"
        )
    if float(final_rows[1][10]) <= 0.0:
        raise GenerationError(
            f"{source}: TAKEOFF altitude must be positive"
        )
    if navigation_count < 1:
        raise GenerationError(
            f"{source}: no navigation waypoints were produced"
        )
    if rtl_indexes:
        raise GenerationError(
            f"{source}: unexpected RTL command found at {rtl_indexes}"
        )
    if land_indexes != [len(final_rows) - 1]:
        raise GenerationError(
            f"{source}: exactly one final LAND command is required"
        )
    if int(final_rows[-1][2]) != MAV_FRAME_GLOBAL_RELATIVE_ALT:
        raise GenerationError(
            f"{source}: LAND must use relative-altitude frame"
        )
    if not math.isclose(
        float(final_rows[-1][8]),
        home_latitude_deg,
        rel_tol=0.0,
        abs_tol=1.0e-8,
    ) or not math.isclose(
        float(final_rows[-1][9]),
        home_longitude_deg,
        rel_tol=0.0,
        abs_tol=1.0e-8,
    ):
        raise GenerationError(
            f"{source}: final LAND coordinates do not match HOME"
        )

    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        QGC_HEADER
        + "\n"
        + "\n".join(
            "\t".join(row)
            for row in final_rows
        )
        + "\n",
        encoding="utf-8",
    )
    return len(final_rows), navigation_count



def choose_kml_from_inbox(input_directory: Path) -> Path:
    """Interactively select one KML/KMZ from a dedicated inbox directory."""
    inbox = input_directory.expanduser().resolve()
    try:
        inbox.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise GenerationError(
            f"Could not create KML inbox {inbox}: {exc}"
        ) from exc

    print()
    print("KML input required")
    print(f"Put your .kml or .kmz file in:")
    print(f"  {inbox}")
    print()
    print("The original file is only read; it is not modified or deleted.")

    while True:
        try:
            candidates = sorted(
                (
                    path.resolve()
                    for path in inbox.iterdir()
                    if path.is_file()
                    and path.suffix.casefold() in {".kml", ".kmz"}
                ),
                key=lambda path: path.name.casefold(),
            )
        except OSError as exc:
            raise GenerationError(
                f"Could not read KML inbox {inbox}: {exc}"
            ) from exc

        if not candidates:
            answer = input(
                "\nNo KML/KMZ file found. Copy one into the directory, "
                "then press Enter to scan again (or type q to quit): "
            ).strip().casefold()
            if answer in {"q", "quit", "exit"}:
                raise GenerationError("Cancelled before selecting a KML/KMZ file")
            continue

        if len(candidates) == 1:
            selected = candidates[0]
            print(f"\nUsing input file: {selected}")
            return selected

        print("\nMultiple KML/KMZ files found:")
        for index, path in enumerate(candidates, start=1):
            print(f"  {index}. {path.name}")

        answer = input(
            "Enter the file number, press Enter to rescan, or type q to quit: "
        ).strip()

        if not answer:
            continue
        if answer.casefold() in {"q", "quit", "exit"}:
            raise GenerationError("Cancelled before selecting a KML/KMZ file")

        try:
            selected_index = int(answer)
        except ValueError:
            print("Please enter a valid file number.")
            continue

        if not 1 <= selected_index <= len(candidates):
            print(
                f"Please enter a number from 1 to {len(candidates)}."
            )
            continue

        selected = candidates[selected_index - 1]
        print(f"\nUsing input file: {selected}")
        return selected

def resolve_outputs(
    *,
    kml: Path,
    drone_count: int,
    output: Path | None,
    output_dir: Path | None,
) -> tuple[Path, dict[int, Path]]:
    if output is not None and drone_count != 1:
        raise GenerationError(
            "--output is valid only with --drones 1; "
            "use --output-dir for N-drone generation"
        )

    if output is not None:
        target = output.expanduser().resolve()
        return target.parent, {1: target}

    destination = (
        output_dir.expanduser().resolve()
        if output_dir is not None
        else (Path.cwd() / f"{kml.stem}_waypoints").resolve()
    )
    return destination, {
        index: destination / f"drone-{index}.waypoints"
        for index in range(1, drone_count + 1)
    }


def main() -> int:
    args = build_argument_parser().parse_args()

    workspace = args.workspace.expanduser().resolve()
    source = (
        args.kml.expanduser().resolve()
        if args.kml is not None
        else choose_kml_from_inbox(args.input_dir)
    )

    if not source.is_file():
        raise GenerationError(f"Input file does not exist: {source}")
    if source.suffix.casefold() not in {".kml", ".kmz"}:
        raise GenerationError("Input file must end in .kml or .kmz")
    if args.altitude < 1.0:
        raise GenerationError(
            "--altitude must be at least 1.0 m because the generated "
            "ArduPilot configuration enforces that minimum"
        )

    output_root, output_paths = resolve_outputs(
        kml=source,
        drone_count=args.drones,
        output=args.output,
        output_dir=args.output_dir,
    )

    api = _load_pipeline_api(workspace)

    mission_output, config, summary = build_inputs(
        api=api,
        source=source,
        drone_count=args.drones,
        clearance_m=args.clearance,
        tracking_margin_m=args.tracking_margin,
        altitude_m=args.altitude,
        lateral_footprint_m=args.footprint,
        lateral_overlap=args.overlap,
        min_component_area_m2=args.min_component_area,
        max_visibility_nodes=args.max_visibility_nodes,
    )

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    working_root = (
        workspace
        / "mission_runs"
        / f"generate_waypoints_{source.stem}_n{args.drones}_{timestamp}"
    )
    input_directory = working_root / "inputs"
    pipeline_directory = working_root / "pipeline"

    mission_json, config_yaml = write_inputs(
        input_directory,
        mission_output=mission_output,
        config=config,
        summary=summary,
    )

    planner_process: subprocess.Popen[Any] | None = None
    started_by_script = False
    try:
        planner_process, started_by_script = start_planner_if_needed(
            workspace,
            args.planner_timeout,
        )
        run_pipeline(
            workspace,
            mission_json=mission_json,
            config_yaml=config_yaml,
            run_directory=pipeline_directory,
        )
    finally:
        if started_by_script:
            _stop_process_group(planner_process)

    home = summary["home"]
    produced: dict[int, tuple[int, int]] = {}
    for index in range(1, args.drones + 1):
        raw = (
            pipeline_directory
            / "ardupilot"
            / f"drone-{index}.waypoints"
        )
        if not raw.is_file():
            raise GenerationError(
                "Pipeline did not produce the expected mission file: "
                f"{raw}"
            )

        destination = output_paths[index]
        produced[index] = normalize_qgc_waypoints(
            raw,
            destination,
            home_longitude_deg=home["longitude_deg"],
            home_latitude_deg=home["latitude_deg"],
            altitude_m=args.altitude,
        )

    unexpected = sorted(
        path.name
        for path in (
            pipeline_directory / "ardupilot"
        ).glob("drone-*.waypoints")
        if path.name
        not in {
            f"drone-{index}.waypoints"
            for index in range(1, args.drones + 1)
        }
    )
    if unexpected:
        raise GenerationError(
            "Pipeline produced unexpected additional waypoint files: "
            + ", ".join(unexpected)
        )

    print()
    print("PASS: N-drone waypoint missions generated")
    print(f"Drones: {args.drones}")
    print(f"Partition mode: {summary['partition_mode']}")
    print(
        "HOME: "
        f"{home['latitude_deg']:.10f}, "
        f"{home['longitude_deg']:.10f}"
    )
    print(
        f"Altitude: {args.altitude:.2f} m relative to HOME"
    )
    for index in range(1, args.drones + 1):
        rows, navigation_count = produced[index]
        print(
            f"  drone-{index}: {output_paths[index]} "
            f"({rows} rows, {navigation_count} navigation waypoints)"
        )
    print(
        "Every file: row 0 HOME, row 1 TAKEOFF, final row LAND"
    )

    if args.keep_intermediates:
        print(f"Intermediates kept at: {working_root}")
    else:
        shutil.rmtree(working_root, ignore_errors=True)

    # Ensure directory exists even if the N=1 --output path was selected.
    output_root.mkdir(parents=True, exist_ok=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nCancelled.", file=sys.stderr)
        raise SystemExit(130)
    except GenerationError as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        raise SystemExit(1)
