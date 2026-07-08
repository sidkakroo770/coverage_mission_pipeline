#!/usr/bin/env python3
"""Single-terminal supervisor for the already-verified Stage 21 SITL mission."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from types import ModuleType
from typing import IO, Any, Mapping, Sequence

import yaml

from .fleet_upload_cli import main as upload_fleet_main


class ExistingSimulationError(RuntimeError):
    """Raised when the supervised Stage 21 simulation cannot proceed safely."""


def _resolve_demo_root() -> Path:
    """Locate the checked-in demo from a source tree or ROS installation."""
    source_candidate = Path(__file__).resolve().parents[1] / "demo/noida_stage21"
    if source_candidate.is_dir():
        return source_candidate
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as exc:
        raise ExistingSimulationError(
            "could not locate packaged demo assets; build and source the ROS workspace"
        ) from exc
    installed_candidate = (
        Path(get_package_share_directory("coverage_mission_pipeline"))
        / "demo/noida_stage21"
    )
    if installed_candidate.is_dir():
        return installed_candidate
    raise ExistingSimulationError(
        f"packaged demo assets are missing: {installed_candidate}"
    )


@dataclass(frozen=True)
class ExistingSimulationPaths:
    """All immutable inputs required by the verified Stage 21 replay."""

    home: Path
    ardupilot: Path
    kml: Path
    missions: Path
    static_audit: Path
    source_fleet_config: Path
    preflight_script: Path
    runner_script: Path

    @classmethod
    def defaults(cls, home: Path | None = None) -> "ExistingSimulationPaths":
        root = (home or Path.home()).expanduser().resolve()
        demo_root = _resolve_demo_root()
        ardupilot = Path(
            os.environ.get("ARDUPILOT_HOME", str(root / "ardupilot"))
        ).expanduser().resolve()
        return cls(
            home=root,
            ardupilot=ardupilot,
            kml=demo_root / "map/stage21_routes_and_zones.kml",
            missions=demo_root / "output",
            static_audit=demo_root / "reports/stage21b-static-audit-report.json",
            source_fleet_config=demo_root / "config/fleet_sitl.yaml",
            preflight_script=demo_root / "scripts/stage21c_telemetry_dry_run.py",
            runner_script=demo_root / "scripts/stage21c_run_fleet.py",
        )

    def validate(self) -> None:
        required_files = {
            "working KML overlay": self.kml,
            "Stage 21 static audit": self.static_audit,
            "source fleet configuration": self.source_fleet_config,
            "verified telemetry preflight": self.preflight_script,
            "verified fleet runner": self.runner_script,
        }
        for label, path in required_files.items():
            if not path.is_file():
                raise ExistingSimulationError(f"missing {label}: {path}")

        if not self.missions.is_dir():
            raise ExistingSimulationError(
                f"missing Stage 21 mission output directory: {self.missions}"
            )
        mission_files = sorted(
            (self.missions / "ardupilot").glob("*.ardupilot-mission.json")
        )
        if len(mission_files) != 5:
            raise ExistingSimulationError(
                "expected five Stage 21 ArduPilot missions, "
                f"found {len(mission_files)}"
            )

        simulator = self.ardupilot / "Tools/autotest/sim_vehicle.py"
        if not simulator.is_file():
            raise ExistingSimulationError(f"missing sim_vehicle.py: {simulator}")

        audit = json.loads(self.static_audit.read_text(encoding="utf-8"))
        if audit.get("status") != "PASSED":
            raise ExistingSimulationError("Stage 21 static audit did not pass")
        if audit.get("vehicle_count") != 5:
            raise ExistingSimulationError("Stage 21 static audit is not for five vehicles")
        if audit.get("all_return_to_reference") is not True:
            raise ExistingSimulationError("return-to-reference was not verified")
        if audit.get("all_end_with_land_at_reference") is not True:
            raise ExistingSimulationError("LAND-at-reference was not verified")
        if audit.get("any_rtl_command") is not False:
            raise ExistingSimulationError("Stage 21 audit reports an RTL command")


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen[Any]
    log_handle: IO[str] | None = None


@dataclass
class ProcessSupervisor:
    """Own child process groups and stop only the children created by this run."""

    processes: list[ManagedProcess] = field(default_factory=list)

    def start(
        self,
        name: str,
        command: Sequence[str],
        *,
        cwd: Path,
        log_path: Path,
        stdin: int | IO[Any] | None = subprocess.DEVNULL,
        env: Mapping[str, str] | None = None,
    ) -> subprocess.Popen[Any]:
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
            process = managed.process
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if all(item.process.poll() is not None for item in self.processes):
                break
            time.sleep(0.2)

        for managed in reversed(self.processes):
            process = managed.process
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if all(item.process.poll() is not None for item in self.processes):
                break
            time.sleep(0.2)

        for managed in reversed(self.processes):
            process = managed.process
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if managed.log_handle is not None:
                managed.log_handle.close()


def _find_mavproxy() -> str:
    for executable in ("mavproxy.py", "mavproxy"):
        found = shutil.which(executable)
        if found:
            return found
    raise ExistingSimulationError(
        "MAVProxy executable not found; expected mavproxy.py or mavproxy on PATH"
    )


def _port_open(port: int) -> bool:
    # Inspect the listening socket without becoming a MAVLink TCP client.
    try:
        completed = subprocess.run(
            ("ss", "-H", "-ltn"),
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise ExistingSimulationError(
            "could not inspect passive TCP listeners with 'ss -H -ltn'"
        ) from exc

    for line in completed.stdout.splitlines():
        fields = line.split()
        if len(fields) < 4:
            continue
        local_address = fields[3]
        try:
            observed = int(local_address.rsplit(":", 1)[1])
        except (IndexError, ValueError):
            continue
        if observed == port:
            return True
    return False


def _wait_for_ports(ports: Sequence[int], timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    pending = set(ports)
    while time.monotonic() < deadline:
        pending = {port for port in pending if not _port_open(port)}
        if not pending:
            return
        time.sleep(0.5)
    raise ExistingSimulationError(
        "SITL did not open required TCP port(s): "
        + ", ".join(str(port) for port in sorted(pending))
    )


def _wait_for_process_alive(
    process: subprocess.Popen[Any],
    *,
    name: str,
    startup_s: float = 2.0,
) -> None:
    deadline = time.monotonic() + startup_s
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise ExistingSimulationError(
                f"{name} exited during startup with code {return_code}"
            )
        time.sleep(0.1)


def _replace_stale_stage_processes() -> None:
    commands = (
        ("pkill", "-INT", "-f", "sim_vehicle.py"),
        ("pkill", "-TERM", "-x", "arducopter"),
        ("pkill", "-TERM", "-f", "mavproxy.py"),
        ("pkill", "-TERM", "-f", "MAVProxy"),
    )
    for command in commands:
        subprocess.run(command, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def write_router_fleet_config(source: Path, destination: Path) -> Path:
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    vehicles = payload.get("vehicles") if isinstance(payload, dict) else None
    if not isinstance(vehicles, list) or len(vehicles) != 5:
        raise ExistingSimulationError(
            "source fleet configuration must contain exactly five vehicles"
        )
    for index, vehicle in enumerate(vehicles):
        vehicle["endpoint"] = f"udpin:127.0.0.1:{14601 + index}"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return destination


def _read_shared_home(missions: Path) -> tuple[float, float]:
    mission_files = sorted((missions / "ardupilot").glob("*.ardupilot-mission.json"))
    if len(mission_files) != 5:
        raise ExistingSimulationError(
            f"expected five demo missions, found {len(mission_files)}"
        )
    shared_home: tuple[float, float] | None = None
    for mission_path in mission_files:
        payload = json.loads(mission_path.read_text(encoding="utf-8"))
        items = payload.get("items")
        if not isinstance(items, list) or not items:
            raise ExistingSimulationError(f"{mission_path.name}: mission has no items")
        first = items[0]
        home = (float(first["latitude_deg"]), float(first["longitude_deg"]))
        if shared_home is None:
            shared_home = home
        elif abs(home[0] - shared_home[0]) > 2e-7 or abs(home[1] - shared_home[1]) > 2e-7:
            raise ExistingSimulationError(
                f"{mission_path.name}: HOME differs from the fleet reference"
            )
    if shared_home is None:
        raise ExistingSimulationError("could not derive shared HOME")
    return shared_home


def _write_ardupilot_locations(missions: Path, destination: Path) -> str:
    """Create a run-local location file without editing user configuration."""
    latitude_deg, longitude_deg = _read_shared_home(missions)
    location_name = "CoverageSwarmDemo"
    destination.write_text(
        f"{location_name}={latitude_deg:.9f},{longitude_deg:.9f},200.0,0.0\n",
        encoding="utf-8",
    )
    return location_name


def _load_verified_script(path: Path, module_name: str) -> ModuleType:
    specification = importlib.util.spec_from_file_location(module_name, path)
    if specification is None or specification.loader is None:
        raise ExistingSimulationError(f"could not load verified script: {path}")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _run_preflight(
    script: Path,
    *,
    run_directory: Path,
    fleet_config: Path,
    upload_report: Path,
) -> None:
    module = _load_verified_script(script, "stage22_existing_preflight")
    module.ROOT = run_directory
    module.CONFIG_PATH = fleet_config
    module.UPLOAD_REPORT_PATH = upload_report
    module.OUTPUT_PATH = run_directory / "simulation-telemetry-dry-run-report.json"
    module.main()


def _run_fleet(
    script: Path,
    *,
    run_directory: Path,
    fleet_config: Path,
    upload_report: Path,
    static_audit: Path,
    missions: Path,
) -> None:
    module = _load_verified_script(script, "stage22_existing_runner")
    module.ROOT = run_directory
    module.CONFIG_PATH = fleet_config
    module.UPLOAD_REPORT_PATH = upload_report
    module.DRY_RUN_REPORT_PATH = (
        run_directory / "simulation-telemetry-dry-run-report.json"
    )
    module.STATIC_AUDIT_PATH = static_audit
    module.SOURCE_MISSION_DIR = missions / "ardupilot"
    module.REPORT_PATH = run_directory / "simulation-fleet-execution-report.json"
    module.CHECKPOINT_PATH = run_directory / "simulation-fleet-checkpoint.json"
    module.TELEMETRY_PATH = run_directory / "simulation-fleet-telemetry.csv"
    module.EVENTS_PATH = run_directory / "simulation-fleet-events.jsonl"
    module.main()


def _write_run_state(
    path: Path,
    *,
    status: str,
    kml: Path,
    execute: bool,
    process_ids: dict[str, int],
) -> None:
    payload = {
        "schema_version": 1,
        "status": status,
        "execute": execute,
        "kml": str(kml),
        "process_ids": process_ids,
        "updated_utc": datetime.utcnow().isoformat(timespec="seconds") + "Z",
    }
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def run_existing_simulation(
    *,
    execute: bool,
    replace_running: bool,
    speedup: float,
    run_directory: Path | None = None,
    hold_map: bool = False,
    paths: ExistingSimulationPaths | None = None,
) -> int:
    """Run the existing audited mission and live map from one terminal."""

    if speedup <= 0.0:
        raise ExistingSimulationError("speedup must be greater than zero")

    resolved = paths or ExistingSimulationPaths.defaults()
    resolved.validate()
    mavproxy = _find_mavproxy()

    if replace_running:
        print("[0/8] Stopping stale SITL/MAVProxy processes")
        _replace_stale_stage_processes()

    required_ports = (5760, 5770, 5780, 5790, 5800)
    occupied = [port for port in required_ports if _port_open(port)]
    if occupied:
        raise ExistingSimulationError(
            "SITL port(s) already occupied: "
            + ", ".join(str(port) for port in occupied)
            + "; rerun with --replace-running"
        )

    if run_directory is None:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        run_directory = (
            resolved.home
            / "coverage_ws/mission_runs/noida_stage22"
            / f"existing-simulation-{timestamp}"
        )
    run_directory = run_directory.expanduser().resolve()
    if run_directory.exists() and any(run_directory.iterdir()):
        raise ExistingSimulationError(
            f"run directory already exists and is not empty: {run_directory}"
        )

    logs = run_directory / "logs"
    sitl_directory = run_directory / "sitl"
    logs.mkdir(parents=True, exist_ok=True)
    sitl_directory.mkdir(parents=True, exist_ok=True)

    fleet_config = write_router_fleet_config(
        resolved.source_fleet_config,
        run_directory / "fleet_sitl_router.yaml",
    )
    copied_kml = run_directory / resolved.kml.name
    shutil.copy2(resolved.kml, copied_kml)

    state_path = run_directory / "run-state.json"
    supervisor = ProcessSupervisor()
    process_ids: dict[str, int] = {}

    try:
        print(f"Run directory: {run_directory}")
        print("[1/8] Starting five ArduCopter SITL vehicles")
        simulator = resolved.ardupilot / "Tools/autotest/sim_vehicle.py"
        locations_path = run_directory / "ardupilot-locations.txt"
        location_name = _write_ardupilot_locations(resolved.missions, locations_path)
        sitl_environment = os.environ.copy()
        sitl_environment["ARDUPILOT_LOCATIONS"] = str(locations_path)
        sitl = supervisor.start(
            "sitl",
            (
                str(simulator),
                "-v", "ArduCopter",
                "-f", "quad",
                "-L", location_name,
                "-n", "5",
                "--auto-sysid",
                f"--speedup={speedup:g}",
                "--no-mavproxy",
                "--use-dir", str(sitl_directory),
                "--no-rebuild",
                "-w",
            ),
            cwd=sitl_directory,
            log_path=logs / "sitl.log",
            env=sitl_environment,
        )
        process_ids["sitl"] = sitl.pid
        _wait_for_process_alive(sitl, name="SITL")
        _wait_for_ports(required_ports, timeout_s=120.0)
        print("[2/8] SITL fleet ready")

        router_pids: list[int] = []
        for index, tcp_port in enumerate(required_ports):
            drone_number = index + 1
            runner_port = 14601 + index
            router = supervisor.start(
                f"router-{drone_number}",
                (
                    mavproxy,
                    f"--master=tcp:127.0.0.1:{tcp_port}",
                    f"--out=udp:127.0.0.1:{runner_port}",
                    "--out=udp:127.0.0.1:14550",
                    f"--aircraft=stage22-router-{drone_number}",
                    "--non-interactive",
                ),
                cwd=run_directory,
                log_path=logs / "routers" / f"drone-{drone_number}.log",
            )
            _wait_for_process_alive(router, name=f"router-{drone_number}")
            router_pids.append(router.pid)
        process_ids["routers"] = router_pids  # type: ignore[assignment]
        time.sleep(3.0)
        print("[3/8] Five MAVLink routers ready")

        map_commands = (
            "module load kmlread; "
            f"kml load {copied_kml}; "
            "map set showdirection 1"
        )
        map_process = supervisor.start(
            "map",
            (
                mavproxy,
                "--master=udpin:127.0.0.1:14550",
                "--map",
                "--console",
                "--aircraft=stage22-five-drone-map",
                f"--cmd={map_commands}",
            ),
            cwd=run_directory,
            log_path=logs / "map.log",
            # MAVProxy exits on stdin EOF; keep a private pipe open.
            stdin=subprocess.PIPE,
        )
        process_ids["map"] = map_process.pid
        _wait_for_process_alive(map_process, name="MAVProxy map", startup_s=3.0)
        print("[4/8] Live map opened and KML overlay loaded")

        upload_report = run_directory / "simulation-upload-report.json"
        upload_status = upload_fleet_main(
            (
                "--missions", str(resolved.missions),
                "--fleet-config", str(fleet_config),
                "--report", str(upload_report),
            )
        )
        if upload_status != 0:
            raise ExistingSimulationError(
                f"mission upload/readback failed with code {upload_status}"
            )
        print("[5/8] Five mission uploads verified")

        _run_preflight(
            resolved.preflight_script,
            run_directory=run_directory,
            fleet_config=fleet_config,
            upload_report=upload_report,
        )
        print("[6/8] No-arming telemetry preflight passed")

        _write_run_state(
            state_path,
            status="READY",
            kml=copied_kml,
            execute=execute,
            process_ids=process_ids,
        )

        if not execute:
            print("[7/8] READY — no vehicle was armed")
            print("Rerun with --execute to fly the existing Stage 21 mission.")
            print(f"Reports: {run_directory}")
            return 0

        print("[7/8] Executing verified staggered five-drone mission")
        _write_run_state(
            state_path,
            status="EXECUTING",
            kml=copied_kml,
            execute=True,
            process_ids=process_ids,
        )
        _run_fleet(
            resolved.runner_script,
            run_directory=run_directory,
            fleet_config=fleet_config,
            upload_report=upload_report,
            static_audit=resolved.static_audit,
            missions=resolved.missions,
        )
        _write_run_state(
            state_path,
            status="PASSED",
            kml=copied_kml,
            execute=True,
            process_ids=process_ids,
        )
        print("[8/8] Simulation completed: all five vehicles landed and disarmed")
        print(f"Reports and logs: {run_directory}")

        if hold_map and sys.stdin.isatty():
            input("Press Enter to close the map and SITL... ")
        return 0

    except KeyboardInterrupt:
        _write_run_state(
            state_path,
            status="INTERRUPTED",
            kml=copied_kml,
            execute=execute,
            process_ids=process_ids,
        )
        print("\nINTERRUPTED: stopping only processes started by this run")
        return 130
    except Exception:
        _write_run_state(
            state_path,
            status="FAILED",
            kml=copied_kml,
            execute=execute,
            process_ids=process_ids,
        )
        raise
    finally:
        supervisor.stop_all()
