#!/usr/bin/env python3
"""Production ROS 2 command-line entry point for Swarm-Partitions missions.

The command consumes three explicit paths:

* a JSON geometry export produced by ``atissss/Swarm-Partitions``;
* a strict operational JSON/YAML configuration;
* a new destination directory for all generated artifacts.

Input geometry and configuration are validated before ROS is initialized.  The
coverage service is then called sequentially and fail-closed.  A complete output
bundle is published atomically only after planning, route assembly, serialization
and ArduPilot export all succeed.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Any, Callable, Optional, Protocol, Sequence

from .generic_mission_pipeline import (
    GenericMissionPipelineError,
    run_generic_mission_pipeline,
)
from .sequential_plan_coverage_client import (
    PlanCoverageClientError,
    RclpyPlanCoverageTransport,
    SequentialClientConfig,
    SequentialPlanCoverageRunner,
)
from .swarm_mission_config import (
    SwarmMissionConfigError,
    SwarmMissionOperationalConfig,
    load_swarm_mission_operational_config,
)
from .swarm_partitions_adapter import (
    SwarmPartitionsAdapterError,
    SwarmPartitionsAdapterResult,
    SwarmPartitionsPipelineResult,
    load_swarm_partitions_json,
)

PRODUCTION_RUN_SCHEMA_VERSION = 1
PRODUCTION_RUN_ALGORITHM = "swarm_partitions_ros_cli_v1"
PRODUCTION_RUN_FILENAME = "production-run.json"
NORMALIZED_CONFIG_FILENAME = "operational-config.normalized.json"

EXIT_INPUT_ERROR = 2
EXIT_ROS_ERROR = 3
EXIT_PLANNING_ERROR = 4
EXIT_ARTIFACT_ERROR = 5


class SwarmMissionCliError(RuntimeError):
    """Fail-closed CLI error with a stable stage and process exit code."""

    def __init__(self, stage: str, message: str, exit_code: int) -> None:
        if not isinstance(stage, str) or not stage:
            raise ValueError("stage must be a non-empty string")
        if not isinstance(message, str) or not message:
            raise ValueError("message must be a non-empty string")
        if isinstance(exit_code, bool) or not isinstance(exit_code, int) or exit_code <= 0:
            raise ValueError("exit_code must be a positive integer")
        super().__init__(f"{stage}: {message}")
        self.stage = stage
        self.detail = message
        self.exit_code = exit_code


class RosRuntime(Protocol):
    """Minimal ROS lifecycle used by the production command and unit tests."""

    def init(self) -> None:
        ...

    def create_node(self, node_name: str) -> Any:
        ...

    def shutdown(self) -> None:
        ...


class _DefaultRosRuntime:
    """Lazy rclpy runtime so importing this module does not require ROS."""

    def __init__(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
        except ImportError as exc:
            raise SwarmMissionCliError(
                "ros_initialization",
                "rclpy is unavailable; source /opt/ros/humble/setup.zsh and the workspace setup.zsh",
                EXIT_ROS_ERROR,
            ) from exc
        self._rclpy = rclpy
        self._node_type = Node
        self._initialized = False

    def init(self) -> None:
        try:
            self._rclpy.init(args=None)
        except Exception as exc:
            raise SwarmMissionCliError(
                "ros_initialization",
                f"rclpy.init failed: {exc}",
                EXIT_ROS_ERROR,
            ) from exc
        self._initialized = True

    def create_node(self, node_name: str) -> Any:
        try:
            return self._node_type(node_name)
        except Exception as exc:
            raise SwarmMissionCliError(
                "ros_initialization",
                f"could not create ROS node {node_name!r}: {exc}",
                EXIT_ROS_ERROR,
            ) from exc

    def shutdown(self) -> None:
        try:
            if self._initialized and self._rclpy.ok():
                self._rclpy.shutdown()
        finally:
            self._initialized = False


@dataclass(frozen=True)
class ProductionMissionRunResult:
    """Successful production execution plus the atomically published directory."""

    pipeline: SwarmPartitionsPipelineResult
    output_directory: Path
    production_record: dict[str, Any]

    def __post_init__(self) -> None:
        if not isinstance(self.pipeline, SwarmPartitionsPipelineResult):
            raise ValueError("pipeline must be a SwarmPartitionsPipelineResult")
        output = Path(self.output_directory)
        if not output.exists() or not output.is_dir():
            raise ValueError("output_directory must be an existing directory")
        if not isinstance(self.production_record, dict):
            raise ValueError("production_record must be a dict")
        object.__setattr__(self, "output_directory", output)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise SwarmMissionCliError(
            "input_validation",
            f"could not hash input file {path}: {exc}",
            EXIT_INPUT_ERROR,
        ) from exc
    return digest.hexdigest()


def _validate_input_file(path: Path | str, label: str) -> Path:
    source = Path(path)
    if not source.exists():
        raise SwarmMissionCliError(
            "input_validation",
            f"{label} does not exist: {source}",
            EXIT_INPUT_ERROR,
        )
    if not source.is_file():
        raise SwarmMissionCliError(
            "input_validation",
            f"{label} must be a regular file: {source}",
            EXIT_INPUT_ERROR,
        )
    return source


def _validate_output_path(path: Path | str) -> Path:
    target = Path(path)
    if target.exists():
        raise SwarmMissionCliError(
            "input_validation",
            f"output directory already exists: {target}",
            EXIT_INPUT_ERROR,
        )
    ancestor = target.parent
    while not ancestor.exists() and ancestor != ancestor.parent:
        ancestor = ancestor.parent
    if ancestor.exists() and not ancestor.is_dir():
        raise SwarmMissionCliError(
            "input_validation",
            f"output parent is not a directory: {ancestor}",
            EXIT_INPUT_ERROR,
        )
    return target


def _build_ros_runner(node: Any, config: SequentialClientConfig) -> Any:
    transport = RclpyPlanCoverageTransport(
        node,
        service_name=config.service_name,
    )
    return SequentialPlanCoverageRunner(transport, config)


def _production_record(
    *,
    mission_json_path: Path,
    config_path: Path,
    operational: SwarmMissionOperationalConfig,
    adapter: SwarmPartitionsAdapterResult,
    mission_result: Any,
    client_config: SequentialClientConfig,
) -> dict[str, Any]:
    return {
        "schema_version": PRODUCTION_RUN_SCHEMA_VERSION,
        "algorithm": PRODUCTION_RUN_ALGORITHM,
        "inputs": {
            "mission_json": {
                "filename": mission_json_path.name,
                "sha256": _sha256(mission_json_path),
            },
            "operational_config": {
                "filename": config_path.name,
                "sha256": _sha256(config_path),
            },
        },
        "ros": {
            "service_name": client_config.service_name,
            "service_wait_timeout_s": client_config.service_wait_timeout_s,
            "request_timeout_s": client_config.request_timeout_s,
            "node_name": client_config.node_name,
        },
        "adapter": adapter.to_summary_dict(),
        "mission": mission_result.to_summary_dict(),
        "files": {
            "generic_manifest": "generic-mission-manifest.json",
            "normalized_operational_config": NORMALIZED_CONFIG_FILENAME,
        },
        "operational_config_schema_version": operational.to_dict()["schema_version"],
    }


def _publish_bundle(
    *,
    result: SwarmPartitionsPipelineResult,
    operational: SwarmMissionOperationalConfig,
    production_record: dict[str, Any],
    destination: Path,
) -> Path:
    if destination.exists():
        raise SwarmMissionCliError(
            "artifact_write",
            f"destination already exists: {destination}",
            EXIT_ARTIFACT_ERROR,
        )
    try:
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary_root = Path(
            tempfile.mkdtemp(
                prefix=f".{destination.name}.production.",
                dir=str(destination.parent),
            )
        )
    except OSError as exc:
        raise SwarmMissionCliError(
            "artifact_write",
            f"could not create temporary output directory: {exc}",
            EXIT_ARTIFACT_ERROR,
        ) from exc

    bundle = temporary_root / "bundle"
    try:
        result.mission.write_artifacts(bundle)
        (bundle / NORMALIZED_CONFIG_FILENAME).write_text(
            operational.to_json(),
            encoding="utf-8",
        )
        (bundle / PRODUCTION_RUN_FILENAME).write_text(
            json.dumps(
                production_record,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )
        os.replace(bundle, destination)
    except Exception as exc:
        shutil.rmtree(temporary_root, ignore_errors=True)
        if isinstance(exc, SwarmMissionCliError):
            raise
        raise SwarmMissionCliError(
            "artifact_write",
            str(exc),
            EXIT_ARTIFACT_ERROR,
        ) from exc
    finally:
        if temporary_root.exists():
            shutil.rmtree(temporary_root, ignore_errors=True)
    return destination


def run_production_swarm_mission(
    mission_json_path: Path | str,
    operational_config_path: Path | str,
    output_directory: Path | str,
    *,
    client_config: Optional[SequentialClientConfig] = None,
    runtime_factory: Callable[[], RosRuntime] = _DefaultRosRuntime,
    runner_builder: Callable[[Any, SequentialClientConfig], Any] = _build_ros_runner,
) -> ProductionMissionRunResult:
    """Validate inputs, call the real planner, and atomically publish all outputs."""
    mission_path = _validate_input_file(mission_json_path, "mission JSON")
    config_path = _validate_input_file(
        operational_config_path,
        "operational configuration",
    )
    destination = _validate_output_path(output_directory)

    try:
        operational = load_swarm_mission_operational_config(config_path)
    except SwarmMissionConfigError as exc:
        raise SwarmMissionCliError(
            "input_validation",
            f"operational configuration is invalid: {exc}",
            EXIT_INPUT_ERROR,
        ) from exc

    try:
        adapter = load_swarm_partitions_json(mission_path, operational.adapter)
    except SwarmPartitionsAdapterError as exc:
        raise SwarmMissionCliError(
            "input_validation",
            f"mission JSON is invalid: {exc}",
            EXIT_INPUT_ERROR,
        ) from exc

    try:
        service_config = client_config or SequentialClientConfig()
    except PlanCoverageClientError as exc:
        raise SwarmMissionCliError(
            "input_validation",
            f"ROS client configuration is invalid: {exc}",
            EXIT_INPUT_ERROR,
        ) from exc
    if not isinstance(service_config, SequentialClientConfig):
        raise SwarmMissionCliError(
            "input_validation",
            "client_config must be a SequentialClientConfig",
            EXIT_INPUT_ERROR,
        )

    runtime: Optional[RosRuntime] = None
    node: Any = None
    try:
        try:
            runtime = runtime_factory()
            runtime.init()
            node = runtime.create_node(service_config.node_name)
            runner = runner_builder(node, service_config)
        except SwarmMissionCliError:
            raise
        except Exception as exc:
            raise SwarmMissionCliError(
                "ros_initialization",
                str(exc),
                EXIT_ROS_ERROR,
            ) from exc

        try:
            mission_result = run_generic_mission_pipeline(
                adapter.definition,
                runner,
                config=operational.pipeline,
            )
            combined = SwarmPartitionsPipelineResult(
                adapter=adapter,
                mission=mission_result,
            )
        except GenericMissionPipelineError as exc:
            raise SwarmMissionCliError(
                "mission_pipeline",
                str(exc),
                EXIT_PLANNING_ERROR,
            ) from exc
        except Exception as exc:
            raise SwarmMissionCliError(
                "mission_pipeline",
                str(exc),
                EXIT_PLANNING_ERROR,
            ) from exc
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if runtime is not None:
            try:
                runtime.shutdown()
            except Exception:
                pass

    record = _production_record(
        mission_json_path=mission_path,
        config_path=config_path,
        operational=operational,
        adapter=adapter,
        mission_result=combined.mission,
        client_config=service_config,
    )
    published = _publish_bundle(
        result=combined,
        operational=operational,
        production_record=record,
        destination=destination,
    )
    return ProductionMissionRunResult(
        pipeline=combined,
        output_directory=published,
        production_record=record,
    )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="run_swarm_mission",
        description=(
            "Run the Swarm-Partitions mission pipeline through the real ROS 2 "
            "/plan_coverage service and atomically export ArduPilot missions."
        ),
    )
    parser.add_argument(
        "--mission-json",
        required=True,
        type=Path,
        help="Swarm-Partitions mission_output.json path.",
    )
    parser.add_argument(
        "--config",
        required=True,
        type=Path,
        help="Operational .json/.yaml/.yml configuration path.",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="New output directory; existing paths are never overwritten.",
    )
    parser.add_argument("--service-name", default="/plan_coverage")
    parser.add_argument("--service-wait-timeout", type=float, default=10.0)
    parser.add_argument("--request-timeout", type=float, default=30.0)
    parser.add_argument(
        "--node-name",
        default="coverage_mission_pipeline",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        client_config = SequentialClientConfig(
            service_name=args.service_name,
            service_wait_timeout_s=args.service_wait_timeout,
            request_timeout_s=args.request_timeout,
            node_name=args.node_name,
        )
        result = run_production_swarm_mission(
            args.mission_json,
            args.config,
            args.output,
            client_config=client_config,
        )
    except PlanCoverageClientError as exc:
        print(f"FAILED [input_validation]: {exc}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    except SwarmMissionCliError as exc:
        print(f"FAILED [{exc.stage}]: {exc.detail}", file=sys.stderr)
        return exc.exit_code
    except Exception as exc:  # defensive process boundary
        print(f"FAILED [unexpected]: {exc}", file=sys.stderr)
        return EXIT_PLANNING_ERROR

    summary = result.pipeline.to_summary_dict()
    print(
        "PASS production swarm mission: "
        f"{summary['adapter']['component_count']} component(s), "
        f"{len(summary['mission']['active_vehicle_ids'])} active vehicle(s), "
        f"output={result.output_directory}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
