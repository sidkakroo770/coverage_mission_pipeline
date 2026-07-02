from pathlib import Path

import pytest
import yaml

from coverage_mission_pipeline.existing_simulation import (
    ExistingSimulationError,
    write_router_fleet_config,
)


def _fleet_payload(vehicle_count: int = 5):
    return {
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
                "endpoint": f"tcp:127.0.0.1:{5750 + index * 10}",
                "system_id": index,
                "component_id": 1,
            }
            for index in range(1, vehicle_count + 1)
        ],
    }


def test_write_router_fleet_config_assigns_independent_udp_ports(tmp_path: Path):
    source = tmp_path / "source.yaml"
    destination = tmp_path / "generated.yaml"
    source.write_text(yaml.safe_dump(_fleet_payload()), encoding="utf-8")

    result = write_router_fleet_config(source, destination)
    payload = yaml.safe_load(result.read_text(encoding="utf-8"))

    assert result == destination
    assert [vehicle["endpoint"] for vehicle in payload["vehicles"]] == [
        "udpin:127.0.0.1:14601",
        "udpin:127.0.0.1:14602",
        "udpin:127.0.0.1:14603",
        "udpin:127.0.0.1:14604",
        "udpin:127.0.0.1:14605",
    ]
    assert [vehicle["system_id"] for vehicle in payload["vehicles"]] == [1, 2, 3, 4, 5]


def test_write_router_fleet_config_rejects_non_five_vehicle_input(tmp_path: Path):
    source = tmp_path / "source.yaml"
    source.write_text(yaml.safe_dump(_fleet_payload(4)), encoding="utf-8")

    with pytest.raises(ExistingSimulationError, match="exactly five"):
        write_router_fleet_config(source, tmp_path / "generated.yaml")
