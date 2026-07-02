#!/usr/bin/env python3
"""Command line entry point for transactional fleet mission upload."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Sequence

from .fleet_config import FleetConfigError, load_fleet_upload_config
from .mission_protocol import MissionUploadError, upload_and_verify_fleet
from .pymavlink_mission_client import pymavlink_client_factory

EXIT_INPUT_ERROR = 2
EXIT_UPLOAD_ERROR = 3


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="upload_swarm_missions",
        description=(
            "Upload each generated ArduPilot mission over its own MAVLink "
            "connection, download it again, and refuse success unless every "
            "readback fingerprint matches. This command never arms vehicles."
        ),
    )
    parser.add_argument(
        "--missions",
        required=True,
        type=Path,
        help=(
            "Generated mission bundle directory, either containing the "
            "ardupilot/ subdirectory or the mission JSON files directly."
        ),
    )
    parser.add_argument(
        "--fleet-config",
        required=True,
        type=Path,
        help="Strict fleet .yaml/.yml/.json configuration.",
    )
    parser.add_argument(
        "--report",
        required=True,
        type=Path,
        help="New JSON report path. Existing files are never overwritten.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        config = load_fleet_upload_config(args.fleet_config)
        report = upload_and_verify_fleet(
            args.missions,
            config,
            args.report,
            pymavlink_client_factory,
        )
    except FleetConfigError as exc:
        print(f"FAILED [fleet_config]: {exc}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    except MissionUploadError as exc:
        print(f"FAILED [mission_upload]: {exc}", file=sys.stderr)
        return EXIT_UPLOAD_ERROR
    except Exception as exc:
        print(f"FAILED [unexpected]: {exc}", file=sys.stderr)
        return EXIT_UPLOAD_ERROR

    print(
        "VERIFIED fleet mission upload: "
        f"{report['vehicle_count']} vehicle(s), report={args.report}"
    )
    for vehicle in report["vehicles"]:
        print(
            f"  {vehicle['vehicle_id']}: sysid={vehicle['observed_system_id']} "
            f"items={vehicle['readback_item_count']} "
            f"wire_items={vehicle['readback_wire_item_count']} "
            f"fingerprint={vehicle['readback_fingerprint'][:16]}..."
        )
    print("No vehicle was armed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
