from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from coverage_mission_pipeline.product_cli import _parser
from coverage_mission_pipeline.variable_simulation import (
    VariableSimulationError,
    _parse_listening_ports,
    sitl_map_tcp_port,
    sitl_tcp_port,
    validate_mission_bundle,
    write_direct_fleet_config,
)


def _write_mission(
    directory: Path,
    vehicle_id: str,
    *,
    latitude_deg: float = 30.0,
    longitude_deg: float = 76.0,
    include_rtl: bool = False,
) -> None:
    commands = [22, 16, 16, 21]
    if include_rtl:
        commands.insert(-1, 20)
    items = []
    for index, command in enumerate(commands):
        is_terminal_home = index == 0 or index >= len(commands) - 2
        items.append(
            {
                "sequence": index,
                "command": command,
                "frame": 3,
                "latitude_deg": (
                    latitude_deg
                    if is_terminal_home
                    else latitude_deg + 0.0001
                ),
                "longitude_deg": (
                    longitude_deg
                    if is_terminal_home
                    else longitude_deg + 0.0001
                ),
                "altitude_m": (
                    0.0 if command == 21 else 20.0
                ),
                "hold_time_s": 0.0,
                "acceptance_radius_m": 0.0,
                "pass_radius_m": 0.0,
                "yaw_angle_deg": 0.0,
                "autocontinue": True,
            }
        )
    payload = {
        "schema_version": 1,
        "vehicle_id": vehicle_id,
        "items": items,
    }
    output = directory / "ardupilot"
    output.mkdir(parents=True, exist_ok=True)
    (output / f"{vehicle_id}.ardupilot-mission.json").write_text(
        json.dumps(payload),
        encoding="utf-8",
    )


def test_sitl_tcp_port_scales_by_instance() -> None:
    assert sitl_tcp_port(1) == 5760
    assert sitl_tcp_port(2) == 5770
    assert sitl_tcp_port(8) == 5830
    with pytest.raises(ValueError):
        sitl_tcp_port(0)


def test_sitl_map_tcp_port_uses_secondary_serial_port() -> None:
    assert sitl_map_tcp_port(1) == 5762
    assert sitl_map_tcp_port(2) == 5772
    assert sitl_map_tcp_port(8) == 5832
    with pytest.raises(ValueError):
        sitl_map_tcp_port(0)


def test_parse_listening_ports_is_passive() -> None:
    output = (
        "LISTEN 0 5 127.0.0.1:5760 0.0.0.0:*\n"
        "LISTEN 0 5 [::]:5770 [::]:*\n"
    )
    assert _parse_listening_ports(output) == {5760, 5770}


def test_write_direct_fleet_config_supports_arbitrary_n(tmp_path: Path) -> None:
    path = write_direct_fleet_config(tmp_path / "fleet.yaml", 3)
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert [vehicle["vehicle_id"] for vehicle in payload["vehicles"]] == [
        "drone-1",
        "drone-2",
        "drone-3",
    ]
    assert [vehicle["endpoint"] for vehicle in payload["vehicles"]] == [
        "tcp:127.0.0.1:5760",
        "tcp:127.0.0.1:5770",
        "tcp:127.0.0.1:5780",
    ]
    assert [vehicle["system_id"] for vehicle in payload["vehicles"]] == [
        1,
        2,
        3,
    ]



def test_direct_fleet_config_rejects_reserved_gcs_system_id(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="between 1 and 254"):
        write_direct_fleet_config(tmp_path / "fleet.yaml", 255)


def test_validate_mission_bundle_accepts_arbitrary_n(tmp_path: Path) -> None:
    for index in range(1, 4):
        _write_mission(tmp_path, f"drone-{index}")
    summary = validate_mission_bundle(tmp_path, 3)
    assert summary.vehicle_ids == ("drone-1", "drone-2", "drone-3")
    assert summary.home_latitude_deg == 30.0
    assert summary.semantic_item_counts == {
        "drone-1": 4,
        "drone-2": 4,
        "drone-3": 4,
    }


def test_validate_mission_bundle_rejects_rtl(tmp_path: Path) -> None:
    _write_mission(tmp_path, "drone-1", include_rtl=True)
    with pytest.raises(VariableSimulationError, match="contains RTL"):
        validate_mission_bundle(tmp_path, 1)


def test_simulate_cli_accepts_drone_count() -> None:
    args = _parser().parse_args(
        [
            "simulate",
            "mission.kml",
            "--drones",
            "7",
            "--execute",
        ]
    )
    assert args.command == "simulate"
    assert args.drone_count == 7
    assert args.execute is True


def test_simulate_cli_accepts_live_map_flags() -> None:
    args = _parser().parse_args(
        [
            "simulate",
            "mission.kml",
            "--drones",
            "5",
            "--execute",
            "--map",
            "--hold-map",
        ]
    )
    assert args.command == "simulate"
    assert args.drone_count == 5
    assert args.map is True
    assert args.hold_map is True
