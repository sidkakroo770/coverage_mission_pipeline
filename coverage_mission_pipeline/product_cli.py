#!/usr/bin/env python3
"""User-facing Stage 22 product CLI (validate and prepare phases)."""

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


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="coverage-swarm")
    subparsers = parser.add_subparsers(dest="command", required=True)

    for command in ("validate", "prepare"):
        subparser = subparsers.add_parser(command)
        subparser.add_argument("kml", type=Path)
        subparser.add_argument(
            "--drones",
            "--drone-count",
            dest="drone_count",
            type=_positive_int,
            default=5,
            metavar="N",
            help="Number of drones/partitions to generate (default: 5).",
        )
        subparser.add_argument("--clearance-m", type=float, default=10.0)
        subparser.add_argument("--tracking-margin-m", type=float, default=2.0)
        subparser.add_argument("--altitude-m", type=float, default=20.0)
        subparser.add_argument("--lateral-footprint-m", type=float, default=14.9553871794)
        subparser.add_argument("--lateral-overlap", type=float, default=0.2)
        subparser.add_argument("--min-component-area-m2", type=float, default=250.0)

    prepare = subparsers.choices["prepare"]
    prepare.add_argument("--output", type=Path, required=True)

    simulate = subparsers.add_parser(
        "simulate-existing",
        help=(
            "Launch the already-verified Stage 21 five-drone SITL mission "
            "and its working KML map from one terminal."
        ),
    )
    simulate.add_argument(
        "--execute",
        action="store_true",
        help="Arm and execute after upload and no-arming preflight pass.",
    )
    simulate.add_argument(
        "--replace-running",
        action="store_true",
        help="Stop stale sim_vehicle, ArduCopter, and MAVProxy processes first.",
    )
    simulate.add_argument("--speedup", type=float, default=2.0)
    simulate.add_argument("--run-directory", type=Path)
    simulate.add_argument(
        "--hold-map",
        action="store_true",
        help="Keep the map open after successful landing until Enter is pressed.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)

    if arguments.command == "simulate-existing":
        from .existing_simulation import (
            ExistingSimulationError,
            run_existing_simulation,
        )

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
            f"{mission.home_latitude_deg:.9f}, {mission.home_longitude_deg:.9f} "
            f"({'explicit' if mission.home_was_explicit else 'automatic'})"
        )
        print(
            "Partitions: "
            + ("supplied KML partitions" if mission.supplied_partition_count else "automatic equal-route-area")
        )

        if arguments.command == "prepare":
            output = write_kml_product_artifacts(artifacts, arguments.output)
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
