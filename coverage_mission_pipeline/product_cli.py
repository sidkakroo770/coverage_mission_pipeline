#!/usr/bin/env python3
"""User-facing coverage mission product CLI."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .kml_input import KmlInputError
from .kml_product import build_kml_product_artifacts, write_kml_product_artifacts
from .map_overlay import write_input_overlay


def _positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return parsed


def _add_geometry_arguments(
    parser: argparse.ArgumentParser,
    *,
    default_drone_count: int,
    default_clearance_m: float,
    default_tracking_margin_m: float,
) -> None:
    parser.add_argument(
        "--drones",
        "--drone-count",
        dest="drone_count",
        type=_positive_int,
        default=default_drone_count,
        metavar="N",
        help=(
            "Number of drones/partitions to generate "
            f"(default: {default_drone_count})."
        ),
    )
    parser.add_argument(
        "--clearance-m",
        type=float,
        default=default_clearance_m,
    )
    parser.add_argument(
        "--tracking-margin-m",
        type=float,
        default=default_tracking_margin_m,
    )
    parser.add_argument("--altitude-m", type=float, default=20.0)
    parser.add_argument(
        "--lateral-footprint-m",
        type=float,
        default=14.9553871794,
    )
    parser.add_argument("--lateral-overlap", type=float, default=0.2)
    parser.add_argument("--min-component-area-m2", type=float, default=250.0)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="coverage-swarm")
    subparsers = parser.add_subparsers(dest="command", required=True)

    for command in ("validate", "prepare"):
        subparser = subparsers.add_parser(command)
        subparser.add_argument("kml", type=Path)
        _add_geometry_arguments(
            subparser,
            default_drone_count=5,
            default_clearance_m=10.0,
            default_tracking_margin_m=2.0,
        )

    prepare = subparsers.choices["prepare"]
    prepare.add_argument("--output", type=Path, required=True)

    simulate = subparsers.add_parser(
        "simulate",
        help=(
            "Generate and supervise an arbitrary-N KML mission in "
            "ArduCopter SITL from one terminal."
        ),
    )
    simulate.add_argument("kml", type=Path)
    _add_geometry_arguments(
        simulate,
        default_drone_count=1,
        default_clearance_m=2.0,
        default_tracking_margin_m=0.0,
    )
    simulate.add_argument(
        "--execute",
        action="store_true",
        help=(
            "Arm and execute after generation and verified upload. "
            "Without this flag, no vehicle is armed."
        ),
    )
    simulate.add_argument(
        "--replace-running",
        action="store_true",
        help="Stop stale SITL, MAVProxy, and coverage-planner processes first.",
    )
    simulate.add_argument("--speedup", type=float, default=2.0)
    simulate.add_argument("--run-directory", type=Path)
    simulate.add_argument(
        "--keep-run",
        action="store_true",
        help="Keep managed SITL/planner processes alive after success.",
    )
    simulate.add_argument("--fleet-timeout-s", type=float, default=3600.0)
    simulate.add_argument(
        "--launch-gate-timeout-s",
        type=float,
        default=600.0,
    )
    simulate.add_argument(
        "--launch-clearance-altitude-m",
        type=float,
        default=12.0,
    )
    simulate.add_argument(
        "--launch-clearance-home-distance-m",
        type=float,
        default=30.0,
    )
    simulate.add_argument(
        "--minimum-launch-spacing-s",
        type=float,
        default=5.0,
    )
    simulate.add_argument(
        "--maximum-final-home-distance-m",
        type=float,
        default=10.0,
    )
    simulate.add_argument(
        "--separation-warning-horizontal-m",
        type=float,
        default=10.0,
    )
    simulate.add_argument(
        "--separation-warning-vertical-m",
        type=float,
        default=5.0,
    )

    existing = subparsers.add_parser(
        "simulate-existing",
        help=(
            "Launch the already-verified Stage 21 five-drone SITL mission "
            "and its working KML map from one terminal."
        ),
    )
    existing.add_argument(
        "--execute",
        action="store_true",
        help="Arm and execute after upload and no-arming preflight pass.",
    )
    existing.add_argument(
        "--replace-running",
        action="store_true",
        help="Stop stale sim_vehicle, ArduCopter, and MAVProxy processes first.",
    )
    existing.add_argument("--speedup", type=float, default=2.0)
    existing.add_argument("--run-directory", type=Path)
    existing.add_argument(
        "--hold-map",
        action="store_true",
        help="Keep the map open after successful landing until Enter is pressed.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)

    if arguments.command == "simulate-existing":
        from .existing_simulation import run_existing_simulation

        try:
            return run_existing_simulation(
                execute=arguments.execute,
                replace_running=arguments.replace_running,
                speedup=arguments.speedup,
                run_directory=arguments.run_directory,
                hold_map=arguments.hold_map,
            )
        except (RuntimeError, OSError, ValueError) as exc:
            print(f"FAILED [simulation]: {exc}", file=sys.stderr)
            return 4

    if arguments.command == "simulate":
        from .variable_fleet_runner import FleetExecutionOptions
        from .variable_simulation import run_variable_simulation

        try:
            options = FleetExecutionOptions(
                fleet_timeout_s=arguments.fleet_timeout_s,
                launch_gate_timeout_s=arguments.launch_gate_timeout_s,
                launch_clearance_altitude_m=(
                    arguments.launch_clearance_altitude_m
                ),
                launch_clearance_home_distance_m=(
                    arguments.launch_clearance_home_distance_m
                ),
                minimum_launch_spacing_s=(
                    arguments.minimum_launch_spacing_s
                ),
                maximum_final_home_distance_m=(
                    arguments.maximum_final_home_distance_m
                ),
                separation_warning_horizontal_m=(
                    arguments.separation_warning_horizontal_m
                ),
                separation_warning_vertical_m=(
                    arguments.separation_warning_vertical_m
                ),
            )
            return run_variable_simulation(
                kml=arguments.kml,
                drone_count=arguments.drone_count,
                execute=arguments.execute,
                replace_running=arguments.replace_running,
                speedup=arguments.speedup,
                clearance_m=arguments.clearance_m,
                tracking_margin_m=arguments.tracking_margin_m,
                altitude_m=arguments.altitude_m,
                lateral_footprint_m=arguments.lateral_footprint_m,
                lateral_overlap=arguments.lateral_overlap,
                min_component_area_m2=arguments.min_component_area_m2,
                run_directory=arguments.run_directory,
                keep_run=arguments.keep_run,
                fleet_options=options,
            )
        except (RuntimeError, OSError, ValueError) as exc:
            print(f"FAILED [simulation]: {exc}", file=sys.stderr)
            return 4

    try:
        artifacts = build_kml_product_artifacts(
            arguments.kml,
            clearance_m=arguments.clearance_m,
            tracking_margin_m=arguments.tracking_margin_m,
            altitude_m=arguments.altitude_m,
            lateral_footprint_m=arguments.lateral_footprint_m,
            lateral_overlap=arguments.lateral_overlap,
            min_component_area_m2=arguments.min_component_area_m2,
            drone_count=arguments.drone_count,
        )
        mission = artifacts.mission_input
        print("PASS: KML mission input is valid")
        print(f"Planning CRS: {mission.planning_crs}")
        print(f"Boundary area: {mission.boundary_projected.area:.3f} m^2")
        print(f"Safe area: {mission.safe_area_projected.area:.3f} m^2")
        print(f"Route space: {mission.route_space_projected.area:.3f} m^2")
        print(f"No-go zones: {mission.exclusion_count}")
        print(f"Drones: {arguments.drone_count}")
        print(f"Tracking reserve: {arguments.tracking_margin_m:.3f} m")
        print(
            "HOME: "
            f"{mission.home_latitude_deg:.9f}, "
            f"{mission.home_longitude_deg:.9f} "
            f"({'explicit' if mission.home_was_explicit else 'automatic'})"
        )
        print(
            "Partitions: "
            + (
                "supplied KML partitions"
                if mission.supplied_partition_count
                else "automatic equal-route-area"
            )
        )

        if arguments.command == "prepare":
            output = write_kml_product_artifacts(
                artifacts,
                arguments.output,
            )
            overlay = write_input_overlay(
                artifacts.mission_output,
                artifacts.operational_config,
                output / "map-input-overlay.kml",
            )
            print(f"Prepared output: {output}")
            print(f"Map overlay: {overlay}")
        return 0
    except (KmlInputError, ValueError, OSError) as exc:
        print(f"FAILED [kml_input]: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
