#!/usr/bin/env python3
"""One-terminal arbitrary-N KML-to-ArduCopter SITL supervisor."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from typing import IO, Any, Mapping, Sequence

import yaml

from .ardupilot_mission import (
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,
)
from .fleet_upload_cli import main as upload_fleet_main
from .kml_input import KmlInputError
from .kml_product import (
    build_kml_product_artifacts,
    write_kml_product_artifacts,
)
from .map_overlay import write_input_overlay
from .sequential_plan_coverage_client import SequentialClientConfig
from .swarm_mission_cli import (
    SwarmMissionCliError,
    run_production_swarm_mission,
)
from .variable_fleet_runner import (
    FleetExecutionOptions,
    VariableFleetExecutionError,
    run_variable_fleet,
)


class VariableSimulationError(RuntimeError):
    """Raised when arbitrary-N SITL supervision cannot proceed safely."""


@dataclass(frozen=True)
class MissionBundleSummary:
    vehicle_ids: tuple[str, ...]
    home_latitude_deg: float
    home_longitude_deg: float
    semantic_item_counts: Mapping[str, int]


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen[Any]
    log_handle: IO[str]


@dataclass
class ProcessSupervisor:
    """Own every child process group created by one simulation run."""

    processes: list[ManagedProcess] = field(default_factory=list)

    def start(
        self,
        name: str,
        command: Sequence[str],
        *,
        cwd: Path,
        log_path: Path,
        env: Mapping[str, str] | None = None,
        stdin: int | IO[Any] | None = subprocess.DEVNULL,
    ) -> subprocess.Popen[Any]:
        cwd.mkdir(parents=True, exist_ok=True)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        handle = log_path.open("w", encoding="utf-8", buffering=1)
        process = subprocess.Popen(
            list(command),
            cwd=str(cwd),
            stdin=stdin,
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
            env=None if env is None else dict(env),
        )
        self.processes.append(ManagedProcess(name, process, handle))
        return process

    def stop_all(self) -> None:
        for managed in reversed(self.processes):
            if managed.process.poll() is None:
                try:
                    os.killpg(managed.process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if all(item.process.poll() is not None for item in self.processes):
                break
            time.sleep(0.2)

        for managed in reversed(self.processes):
            if managed.process.poll() is None:
                try:
                    os.killpg(managed.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if all(item.process.poll() is not None for item in self.processes):
                break
            time.sleep(0.2)

        for managed in reversed(self.processes):
            if managed.process.poll() is None:
                try:
                    os.killpg(managed.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if managed.process.stdin is not None:
                try:
                    managed.process.stdin.close()
                except OSError:
                    pass
            managed.log_handle.close()


def sitl_tcp_port(vehicle_number: int) -> int:
    if (
        isinstance(vehicle_number, bool)
        or not isinstance(vehicle_number, int)
        or vehicle_number < 1
    ):
        raise ValueError("vehicle_number must be a positive integer")
    return 5760 + 10 * (vehicle_number - 1)


def sitl_map_tcp_port(vehicle_number: int) -> int:
    """Return a secondary SITL TCP telemetry port that does not steal SERIAL0."""

    if (
        isinstance(vehicle_number, bool)
        or not isinstance(vehicle_number, int)
        or vehicle_number < 1
    ):
        raise ValueError("vehicle_number must be a positive integer")
    return 5762 + 10 * (vehicle_number - 1)


def find_mavproxy() -> str:
    for executable in ("mavproxy.py", "mavproxy"):
        found = shutil.which(executable)
        if found:
            return found
    raise VariableSimulationError(
        "MAVProxy executable not found; install MAVProxy to use --map"
    )


def start_live_map(
    supervisor: ProcessSupervisor,
    *,
    drone_count: int,
    overlay_path: Path,
    run_directory: Path,
    logs: Path,
) -> subprocess.Popen[Any]:
    """Start a MAVProxy map on secondary SITL ports, leaving SERIAL0 free."""

    mavproxy = find_mavproxy()
    masters: list[str] = []
    for index in range(1, drone_count + 1):
        masters.append(f"--master=tcp:127.0.0.1:{sitl_map_tcp_port(index)}")

    map_commands = (
        "module load kmlread; "
        f"kml load {overlay_path}; "
        "map set showdirection 1"
    )
    return supervisor.start(
        "live-map",
        (
            mavproxy,
            *masters,
            "--map",
            "--console",
            f"--aircraft=variable-n-live-map-{drone_count}",
            f"--cmd={map_commands}",
        ),
        cwd=run_directory,
        log_path=logs / "live-map.log",
        # MAVProxy can exit when stdin is already EOF. Keep a private pipe open.
        stdin=subprocess.PIPE,
    )


def _parse_listening_ports(output: str) -> set[int]:
    ports: set[int] = set()
    for line in output.splitlines():
        fields = line.split()
        if len(fields) < 4:
            continue
        local_address = fields[3]
        try:
            port_text = local_address.rsplit(":", 1)[1]
            ports.add(int(port_text))
        except (IndexError, ValueError):
            continue
    return ports


def listening_tcp_ports() -> set[int]:
    try:
        completed = subprocess.run(
            ("ss", "-H", "-ltn"),
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise VariableSimulationError(
            "could not inspect passive TCP listeners with 'ss -H -ltn'"
        ) from exc
    return _parse_listening_ports(completed.stdout)


def wait_for_ports(ports: Sequence[int], timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    pending = set(ports)
    while time.monotonic() < deadline:
        listening = listening_tcp_ports()
        pending = {port for port in pending if port not in listening}
        if not pending:
            return
        time.sleep(0.5)
    raise VariableSimulationError(
        "SITL did not open required TCP ports: "
        + ", ".join(str(port) for port in sorted(pending))
    )


def wait_for_service(service_name: str, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        completed = subprocess.run(
            ("ros2", "service", "list"),
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode == 0:
            services = {line.strip() for line in completed.stdout.splitlines()}
            if service_name in services:
                return
        time.sleep(0.5)
    raise VariableSimulationError(
        f"timeout waiting for ROS service {service_name}"
    )


def wait_for_process_alive(
    process: subprocess.Popen[Any],
    *,
    name: str,
    startup_s: float = 2.0,
) -> None:
    deadline = time.monotonic() + startup_s
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise VariableSimulationError(
                f"{name} exited during startup with code {return_code}"
            )
        time.sleep(0.1)
    return


def replace_stale_processes() -> None:
    commands = (
        ("pkill", "-INT", "-f", "sim_vehicle.py"),
        ("pkill", "-TERM", "-x", "arducopter"),
        ("pkill", "-TERM", "-f", "mavproxy.py"),
        ("pkill", "-TERM", "-f", "MAVProxy"),
        ("pkill", "-TERM", "-x", "coverage_planner"),
        (
            "pkill",
            "-TERM",
            "-f",
            "ros2 run polygon_coverage_ros2 coverage_planner",
        ),
    )
    for command in commands:
        subprocess.run(
            command,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    time.sleep(2.0)


def _same_e7(
    left_lat: float,
    left_lon: float,
    right_lat: float,
    right_lon: float,
) -> bool:
    return (
        int(round(left_lat * 10_000_000.0))
        == int(round(right_lat * 10_000_000.0))
        and int(round(left_lon * 10_000_000.0))
        == int(round(right_lon * 10_000_000.0))
    )


def validate_mission_bundle(
    planned_directory: Path | str,
    drone_count: int,
) -> MissionBundleSummary:
    root = Path(planned_directory)
    mission_directory = root / "ardupilot"
    mission_paths = sorted(
        mission_directory.glob("*.ardupilot-mission.json")
    )
    if len(mission_paths) != drone_count:
        raise VariableSimulationError(
            f"expected {drone_count} ArduPilot missions, "
            f"found {len(mission_paths)}"
        )

    expected_ids = {f"drone-{index}" for index in range(1, drone_count + 1)}
    observed_ids: set[str] = set()
    shared_home: tuple[float, float] | None = None
    item_counts: dict[str, int] = {}

    for path in mission_paths:
        payload = json.loads(path.read_text(encoding="utf-8"))
        vehicle_id = payload.get("vehicle_id")
        items = payload.get("items")
        if not isinstance(vehicle_id, str) or not vehicle_id:
            raise VariableSimulationError(
                f"{path.name}: missing vehicle_id"
            )
        if vehicle_id in observed_ids:
            raise VariableSimulationError(
                f"duplicate mission vehicle_id: {vehicle_id}"
            )
        if not isinstance(items, list) or len(items) < 3:
            raise VariableSimulationError(
                f"{vehicle_id}: mission must contain at least three items"
            )

        commands = [int(item["command"]) for item in items]
        if commands[0] != MAV_CMD_NAV_TAKEOFF:
            raise VariableSimulationError(
                f"{vehicle_id}: first semantic command is not TAKEOFF"
            )
        if commands[-1] != MAV_CMD_NAV_LAND:
            raise VariableSimulationError(
                f"{vehicle_id}: final semantic command is not LAND"
            )
        if MAV_CMD_NAV_RETURN_TO_LAUNCH in commands:
            raise VariableSimulationError(
                f"{vehicle_id}: mission contains RTL"
            )
        if commands[-2] != MAV_CMD_NAV_WAYPOINT:
            raise VariableSimulationError(
                f"{vehicle_id}: command before LAND is not WAYPOINT"
            )

        takeoff = items[0]
        pre_land = items[-2]
        land = items[-1]
        home = (
            float(takeoff["latitude_deg"]),
            float(takeoff["longitude_deg"]),
        )
        for label, item in (
            ("pre-LAND waypoint", pre_land),
            ("LAND", land),
        ):
            if not _same_e7(
                float(item["latitude_deg"]),
                float(item["longitude_deg"]),
                home[0],
                home[1],
            ):
                raise VariableSimulationError(
                    f"{vehicle_id}: {label} is not at HOME"
                )
        if not math.isclose(
            float(land["altitude_m"]),
            0.0,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise VariableSimulationError(
                f"{vehicle_id}: LAND altitude is not zero"
            )

        if shared_home is None:
            shared_home = home
        elif not _same_e7(
            home[0],
            home[1],
            shared_home[0],
            shared_home[1],
        ):
            raise VariableSimulationError(
                f"{vehicle_id}: HOME differs from fleet HOME"
            )

        observed_ids.add(vehicle_id)
        item_counts[vehicle_id] = len(items)

    if observed_ids != expected_ids:
        raise VariableSimulationError(
            "mission vehicle set mismatch; expected "
            + ", ".join(sorted(expected_ids))
            + ", observed "
            + ", ".join(sorted(observed_ids))
        )
    if shared_home is None:
        raise VariableSimulationError("could not derive shared HOME")

    return MissionBundleSummary(
        vehicle_ids=tuple(
            f"drone-{index}" for index in range(1, drone_count + 1)
        ),
        home_latitude_deg=shared_home[0],
        home_longitude_deg=shared_home[1],
        semantic_item_counts=item_counts,
    )


def write_direct_fleet_config(
    destination: Path | str,
    drone_count: int,
) -> Path:
    if (
        isinstance(drone_count, bool)
        or not isinstance(drone_count, int)
        or drone_count < 1
        or drone_count > 254
    ):
        raise ValueError("drone_count must be between 1 and 254 for SITL")
    path = Path(destination)
    payload = {
        "schema_version": 2,
        "profile": "sitl",
        "source_system": 250,
        "source_component": 190,
        "allow_duplicate_missions": False,
        "timeouts": {
            "heartbeat_s": 15.0,
            "ready_s": 120.0,
            "ready_heartbeats": 3,
            "startup_grace_s": 5.0,
            "clear_ack_s": 10.0,
            "item_request_s": 10.0,
            "upload_ack_s": 15.0,
            "download_item_s": 10.0,
            "retries": 3,
        },
        "vehicles": [
            {
                "vehicle_id": f"drone-{index}",
                "endpoint": (
                    f"tcp:127.0.0.1:{sitl_tcp_port(index)}"
                ),
                "system_id": index,
                "component_id": 1,
            }
            for index in range(1, drone_count + 1)
        ],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return path


def verify_upload_report(path: Path | str, drone_count: int) -> None:
    report = json.loads(Path(path).read_text(encoding="utf-8"))
    if report.get("all_verified") is not True:
        raise VariableSimulationError("fleet upload report is not verified")
    if int(report.get("vehicle_count", -1)) != drone_count:
        raise VariableSimulationError(
            "fleet upload report vehicle count mismatch"
        )
    vehicles = report.get("vehicles")
    if not isinstance(vehicles, list):
        raise VariableSimulationError(
            "fleet upload report vehicles must be a list"
        )
    expected = {f"drone-{index}" for index in range(1, drone_count + 1)}
    observed = {str(vehicle.get("vehicle_id")) for vehicle in vehicles}
    if observed != expected:
        raise VariableSimulationError(
            "fleet upload report vehicle set mismatch"
        )
    if not all(vehicle.get("verified") is True for vehicle in vehicles):
        raise VariableSimulationError(
            "one or more fleet mission uploads were not verified"
        )


def write_run_state(
    path: Path,
    *,
    status: str,
    kml: Path,
    drone_count: int,
    execute: bool,
    process_ids: Mapping[str, Any],
) -> None:
    payload = {
        "schema_version": 1,
        "status": status,
        "execute": execute,
        "drone_count": drone_count,
        "kml": str(kml),
        "process_ids": dict(process_ids),
        "updated_utc": (
            datetime.now(timezone.utc).isoformat(timespec="seconds")
        ),
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _tail_process_log(log_path: Path, *, lines: int = 120) -> str:
    if not log_path.is_file():
        return f"log does not exist: {log_path}"
    content = log_path.read_text(encoding="utf-8", errors="replace")
    return "\n".join(content.splitlines()[-lines:])


def run_variable_simulation(
    *,
    kml: Path | str,
    drone_count: int,
    execute: bool,
    replace_running: bool,
    speedup: float,
    clearance_m: float,
    tracking_margin_m: float,
    altitude_m: float,
    lateral_footprint_m: float,
    lateral_overlap: float,
    min_component_area_m2: float,
    run_directory: Path | None = None,
    keep_run: bool = False,
    show_map: bool = False,
    hold_map: bool = False,
    fleet_options: FleetExecutionOptions | None = None,
) -> int:
    """Generate, upload, and optionally execute an arbitrary-N SITL mission."""

    if (
        isinstance(drone_count, bool)
        or not isinstance(drone_count, int)
        or drone_count < 1
    ):
        raise VariableSimulationError(
            "drone_count must be a positive integer"
        )
    if drone_count > 254:
        raise VariableSimulationError(
            "SITL execution supports at most 254 vehicles because "
            "MAVLink system ID 255 is reserved for the GCS"
        )
    if not math.isfinite(speedup) or speedup <= 0.0:
        raise VariableSimulationError(
            "speedup must be finite and greater than zero"
        )
    if hold_map and not show_map:
        raise VariableSimulationError("--hold-map requires --map")
    if show_map:
        find_mavproxy()

    source_kml = Path(kml).expanduser().resolve()
    if not source_kml.is_file():
        raise VariableSimulationError(f"KML not found: {source_kml}")

    ardupilot_home = Path(
        os.environ.get("ARDUPILOT_HOME", str(Path.home() / "ardupilot"))
    ).expanduser().resolve()
    arducopter = ardupilot_home / "build/sitl/bin/arducopter"
    copter_defaults = (
        ardupilot_home
        / "Tools/autotest/default_params/copter.parm"
    )
    if not arducopter.is_file() or not os.access(arducopter, os.X_OK):
        raise VariableSimulationError(
            f"ArduCopter SITL binary not executable: {arducopter}"
        )
    if not copter_defaults.is_file():
        raise VariableSimulationError(
            f"Copter defaults file missing: {copter_defaults}"
        )

    primary_ports = [
        sitl_tcp_port(index)
        for index in range(1, drone_count + 1)
    ]
    map_ports = [
        sitl_map_tcp_port(index)
        for index in range(1, drone_count + 1)
    ] if show_map else []
    required_ports = [*primary_ports, *map_ports]
    if replace_running:
        print("[0/10] Stopping stale SITL/planner processes")
        replace_stale_processes()

    occupied = sorted(
        port
        for port in required_ports
        if port in listening_tcp_ports()
    )
    if occupied:
        raise VariableSimulationError(
            "SITL TCP ports already occupied: "
            + ", ".join(str(port) for port in occupied)
            + "; rerun with --replace-running"
        )

    if run_directory is None:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        run_directory = (
            Path.home()
            / "coverage_ws/mission_runs"
            / f"{source_kml.stem}_{drone_count}drone-{timestamp}"
        )
    run_directory = run_directory.expanduser().resolve()
    if run_directory.exists() and any(run_directory.iterdir()):
        raise VariableSimulationError(
            f"run directory already exists and is not empty: {run_directory}"
        )

    prepared = run_directory / "prepared"
    planned = run_directory / "planned"
    logs = run_directory / "logs"
    sitl_root = run_directory / "sitl"
    copied_kml = run_directory / source_kml.name
    fleet_config = run_directory / "fleet_sitl_direct.yaml"
    upload_report = run_directory / "fleet-upload-report.json"
    execution_report = run_directory / "fleet-execution-report.json"
    checkpoint = run_directory / "fleet-checkpoint.json"
    telemetry = run_directory / "fleet-telemetry.csv"
    events = run_directory / "fleet-events.jsonl"
    state_path = run_directory / "run-state.json"

    run_directory.mkdir(parents=True, exist_ok=True)
    logs.mkdir(parents=True, exist_ok=True)
    sitl_root.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_kml, copied_kml)

    supervisor = ProcessSupervisor()
    process_ids: dict[str, Any] = {}
    success = False
    step_total = 11 if show_map else 10

    try:
        print(f"Run directory: {run_directory}")
        print(f"KML: {source_kml}")
        print(f"Drones: {drone_count}")

        print(f"[1/{step_total}] Validating and preparing arbitrary-N KML mission")
        artifacts = build_kml_product_artifacts(
            source_kml,
            clearance_m=clearance_m,
            tracking_margin_m=tracking_margin_m,
            altitude_m=altitude_m,
            lateral_footprint_m=lateral_footprint_m,
            lateral_overlap=lateral_overlap,
            min_component_area_m2=min_component_area_m2,
            drone_count=drone_count,
        )
        write_kml_product_artifacts(artifacts, prepared)
        write_input_overlay(
            artifacts.mission_output,
            artifacts.operational_config,
            prepared / "map-input-overlay.kml",
        )
        if not artifacts.mission_input.home_was_explicit:
            raise VariableSimulationError(
                "simulation requires an explicit HOME placemark in the KML"
            )
        print(
            "PASS: KML prepared; "
            f"safe_area={artifacts.mission_input.safe_area_projected.area:.3f}m², "
            f"route_space={artifacts.mission_input.route_space_projected.area:.3f}m²"
        )

        print(f"[2/{step_total}] Starting coverage planner")
        planner = supervisor.start(
            "planner",
            (
                "ros2",
                "run",
                "polygon_coverage_ros2",
                "coverage_planner",
            ),
            cwd=run_directory,
            log_path=logs / "coverage-planner.log",
        )
        process_ids["planner"] = planner.pid
        wait_for_process_alive(planner, name="coverage planner")
        wait_for_service("/plan_coverage", 30.0)

        print(f"[3/{step_total}] Generating N boustrophedon routes and missions")
        run_production_swarm_mission(
            prepared / "mission_output.json",
            prepared / "swarm_mission.yaml",
            planned,
            client_config=SequentialClientConfig(
                service_name="/plan_coverage",
                service_wait_timeout_s=30.0,
                request_timeout_s=180.0,
                node_name="coverage_variable_n_simulation",
            ),
        )

        print(f"[4/{step_total}] Verifying every semantic mission")
        summary = validate_mission_bundle(planned, drone_count)
        if not _same_e7(
            summary.home_latitude_deg,
            summary.home_longitude_deg,
            artifacts.mission_input.home_latitude_deg,
            artifacts.mission_input.home_longitude_deg,
        ):
            raise VariableSimulationError(
                "generated mission HOME differs from the explicit KML HOME"
            )
        print(
            "PASS: verified "
            f"{len(summary.vehicle_ids)} mission(s), shared HOME "
            f"{summary.home_latitude_deg:.9f},"
            f"{summary.home_longitude_deg:.9f}"
        )

        print(f"[5/{step_total}] Starting {drone_count} direct ArduCopter SITL instance(s)")
        sitl_pids: list[int] = []
        home_spec = (
            f"{summary.home_latitude_deg:.9f},"
            f"{summary.home_longitude_deg:.9f},200.0,0.0"
        )
        for index in range(1, drone_count + 1):
            instance = index - 1
            vehicle_id = f"drone-{index}"
            vehicle_directory = sitl_root / vehicle_id
            process = supervisor.start(
                f"sitl-{vehicle_id}",
                (
                    str(arducopter),
                    "-w",
                    "--model",
                    "+",
                    f"--speedup={speedup:g}",
                    "--slave",
                    "0",
                    "--defaults",
                    str(copter_defaults),
                    "--sim-address=127.0.0.1",
                    f"-I{instance}",
                    "--home",
                    home_spec,
                    "--sysid",
                    str(index),
                ),
                cwd=vehicle_directory,
                log_path=logs / "sitl" / f"{vehicle_id}.log",
            )
            sitl_pids.append(process.pid)
            wait_for_process_alive(
                process,
                name=f"ArduCopter {vehicle_id}",
            )
        process_ids["sitl"] = sitl_pids
        wait_for_ports(required_ports, timeout_s=120.0)
        for index, managed in enumerate(
            supervisor.processes[-drone_count:],
            start=1,
        ):
            if managed.process.poll() is not None:
                log_path = logs / "sitl" / f"drone-{index}.log"
                raise VariableSimulationError(
                    f"drone-{index} exited after opening its port:\n"
                    + _tail_process_log(log_path)
                )
        print(
            "PASS: SITL ports ready: "
            + ", ".join(str(port) for port in primary_ports)
        )

        if show_map:
            print(f"[6/{step_total}] Starting live MAVProxy map")
            live_map = start_live_map(
                supervisor,
                drone_count=drone_count,
                overlay_path=prepared / "map-input-overlay.kml",
                run_directory=run_directory,
                logs=logs,
            )
            process_ids["live_map"] = live_map.pid
            wait_for_process_alive(
                live_map,
                name="live MAVProxy map",
                startup_s=3.0,
            )
            time.sleep(3.0)
            if live_map.poll() is not None:
                raise VariableSimulationError(
                    "live MAVProxy map exited during startup:\n"
                    + _tail_process_log(logs / "live-map.log")
                )
            print("PASS: live MAVProxy map opened with mission overlay")

        write_direct_fleet_config(fleet_config, drone_count)

        print(f"[{7 if show_map else 6}/{step_total}] Uploading and readback-verifying every mission")
        upload_status = upload_fleet_main(
            (
                "--missions",
                str(planned),
                "--fleet-config",
                str(fleet_config),
                "--report",
                str(upload_report),
            )
        )
        if upload_status != 0:
            raise VariableSimulationError(
                f"mission upload/readback failed with code {upload_status}"
            )
        verify_upload_report(upload_report, drone_count)
        print(f"PASS: {drone_count} mission upload(s) verified")

        write_run_state(
            state_path,
            status="READY",
            kml=copied_kml,
            drone_count=drone_count,
            execute=execute,
            process_ids=process_ids,
        )

        if not execute:
            print(f"[{8 if show_map else 7}/{step_total}] READY — no vehicle was armed")
            print("Add --execute to run the staggered fleet mission.")
            print(f"Reports and missions: {run_directory}")
            success = True
            if show_map and hold_map and sys.stdin.isatty():
                input("Press Enter to close the live map and SITL...")
            return 0

        # Allow every SERIAL0 upload connection to close before the fleet
        # executor opens its long-lived direct TCP clients.
        time.sleep(5.0)

        print(f"[{8 if show_map else 7}/{step_total}] Executing staggered arbitrary-N AUTO fleet")
        write_run_state(
            state_path,
            status="EXECUTING",
            kml=copied_kml,
            drone_count=drone_count,
            execute=True,
            process_ids=process_ids,
        )
        run_variable_fleet(
            fleet_config_path=fleet_config,
            upload_report_path=upload_report,
            report_path=execution_report,
            checkpoint_path=checkpoint,
            telemetry_path=telemetry,
            events_path=events,
            options=fleet_options,
        )

        print(f"[{9 if show_map else 8}/{step_total}] Every mission completed")
        write_run_state(
            state_path,
            status="PASSED",
            kml=copied_kml,
            drone_count=drone_count,
            execute=True,
            process_ids=process_ids,
        )
        print(f"[{10 if show_map else 9}/{step_total}] Every vehicle landed and disarmed")
        print(f"[{11 if show_map else 10}/{step_total}] PASS")
        print(f"Reports, missions, and logs: {run_directory}")
        success = True
        if show_map and hold_map and sys.stdin.isatty():
            input("Press Enter to close the live map and SITL...")
        return 0

    except KeyboardInterrupt:
        write_run_state(
            state_path,
            status="INTERRUPTED",
            kml=copied_kml,
            drone_count=drone_count,
            execute=execute,
            process_ids=process_ids,
        )
        print("\nINTERRUPTED: stopping processes created by this run")
        return 130
    except (
        KmlInputError,
        SwarmMissionCliError,
        VariableFleetExecutionError,
        VariableSimulationError,
        OSError,
        ValueError,
        RuntimeError,
    ):
        write_run_state(
            state_path,
            status="FAILED",
            kml=copied_kml,
            drone_count=drone_count,
            execute=execute,
            process_ids=process_ids,
        )
        raise
    finally:
        if not (success and keep_run):
            supervisor.stop_all()
        elif supervisor.processes:
            print("Keeping managed processes alive because --keep-run was set.")
