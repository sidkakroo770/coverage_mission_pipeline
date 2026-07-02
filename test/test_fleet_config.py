from pathlib import Path

import pytest

from coverage_mission_pipeline.fleet_config import (
    FleetConfigError,
    FleetUploadConfig,
    load_fleet_upload_config,
)


def valid_dict():
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
                "vehicle_id": "drone-1",
                "endpoint": "tcp:127.0.0.1:5760",
                "system_id": 1,
                "component_id": 1,
            },
            {
                "vehicle_id": "drone-2",
                "endpoint": "tcp:127.0.0.1:5770",
                "system_id": 2,
                "component_id": 1,
            },
        ],
    }


def test_valid_config_round_trip():
    config = FleetUploadConfig.from_dict(valid_dict())
    assert config.profile == "sitl"
    assert [v.system_id for v in config.vehicles] == [1, 2]
    assert config.to_dict() == valid_dict()


@pytest.mark.parametrize("field", ["vehicle_id", "endpoint", "system_id"])
def test_duplicate_vehicle_identity_fields_are_rejected(field):
    value = valid_dict()
    value["vehicles"][1][field] = value["vehicles"][0][field]
    with pytest.raises(FleetConfigError, match=f"duplicate {field}"):
        FleetUploadConfig.from_dict(value)


def test_unknown_key_is_rejected():
    value = valid_dict()
    value["unsafe_magic"] = True
    with pytest.raises(FleetConfigError, match="unknown field"):
        FleetUploadConfig.from_dict(value)


def test_yaml_loader(tmp_path: Path):
    path = tmp_path / "fleet.yaml"
    path.write_text(
        """
schema_version: 2
profile: sitl
source_system: 250
source_component: 190
allow_duplicate_missions: false
timeouts:
  heartbeat_s: 15
  ready_s: 120
  ready_heartbeats: 3
  startup_grace_s: 5
  clear_ack_s: 10
  item_request_s: 10
  upload_ack_s: 15
  download_item_s: 10
  retries: 3
vehicles:
  - vehicle_id: drone-1
    endpoint: tcp:127.0.0.1:5760
    system_id: 1
""".lstrip(),
        encoding="utf-8",
    )
    config = load_fleet_upload_config(path)
    assert config.vehicles[0].component_id == 1
