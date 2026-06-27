import copy
import json
from pathlib import Path

import pytest
import yaml

from coverage_mission_pipeline import (
    END_ACTION_LAND_AT_REFERENCE,
    END_ACTION_NONE,
    END_ACTION_RTL,
    GenericMissionPipelineConfig,
    SWARM_MISSION_CONFIG_SCHEMA_VERSION,
    SwarmMissionConfigError,
    SwarmMissionOperationalConfig,
    SwarmPartitionsAdapterConfig,
    load_swarm_mission_operational_config,
)


def valid_payload() -> dict:
    return {
        "schema_version": 1,
        "adapter": {
            "frame_id": "map",
            "clearance_m": 5.0,
            "min_component_area_m2": 0.0,
            "coverage_gap_tolerance_m2": 0.0001,
            "coverage_gap_relative_tolerance": 1.0e-9,
            "partition_overlap_tolerance_m2": 1.0e-6,
        },
        "assignments": [
            {"partition_id": 2, "vehicle_id": "drone-2"},
            {"partition_id": 1, "vehicle_id": "drone-1"},
        ],
        "vehicles": [
            {
                "vehicle_id": "drone-2",
                "reference": {
                    "type": "launch",
                    "longitude_deg": 76.367,
                    "latitude_deg": 30.355,
                },
                "coverage": {
                    "altitude_m": 35.0,
                    "lateral_footprint_m": 2.5,
                    "lateral_overlap": 0.15,
                    "start_goal_boundary_clearance_m": 1.0,
                    "minimum_start_goal_separation_m": 2.0,
                },
            },
            {
                "vehicle_id": "drone-1",
                "reference": {
                    "type": "home",
                    "longitude_deg": 76.366,
                    "latitude_deg": 30.354,
                },
                "coverage": {
                    "altitude_m": 30.0,
                    "lateral_footprint_m": 2.0,
                    "lateral_overlap": 0.10,
                    "start_goal_boundary_clearance_m": 0.5,
                    "minimum_start_goal_separation_m": 1.0,
                },
            },
        ],
        "pipeline": {
            "allow_idle_vehicles": True,
            "route": {
                "return_to_reference": False,
                "connector": {"max_visibility_nodes": 512},
            },
            "ardupilot": {
                "end_action": "rtl",
                "waypoint_hold_s": 0.0,
                "include_takeoff": True,
                "skip_initial_reference_waypoint": True,
                "minimum_relative_altitude_m": 1.0,
            },
        },
    }


def parse(payload=None):
    return SwarmMissionOperationalConfig.from_dict(
        valid_payload() if payload is None else payload
    )


def mutate(path, value):
    payload = valid_payload()
    target = payload
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    return payload


class TestHappyPath:
    def test_schema_version_constant(self):
        assert SWARM_MISSION_CONFIG_SCHEMA_VERSION == 1

    def test_parse_returns_operational_config(self):
        assert isinstance(parse(), SwarmMissionOperationalConfig)

    def test_adapter_type(self):
        assert isinstance(parse().adapter, SwarmPartitionsAdapterConfig)

    def test_pipeline_type(self):
        assert isinstance(parse().pipeline, GenericMissionPipelineConfig)

    def test_assignments_are_canonicalized_by_partition(self):
        assert [x.partition_id for x in parse().adapter.assignments] == [1, 2]

    def test_vehicles_are_canonicalized_by_id(self):
        assert [x.vehicle_id for x in parse().adapter.vehicles] == [
            "drone-1",
            "drone-2",
        ]

    def test_adapter_scalars_preserved(self):
        config = parse().adapter
        assert config.frame_id == "map"
        assert config.clearance_m == 5.0
        assert config.min_component_area_m2 == 0.0
        assert config.coverage_gap_tolerance_m2 == 0.0001
        assert config.coverage_gap_relative_tolerance == 1.0e-9
        assert config.partition_overlap_tolerance_m2 == 1.0e-6

    def test_vehicle_profile_preserved(self):
        profile = parse().adapter.vehicles[0]
        assert profile.vehicle_id == "drone-1"
        assert profile.reference_type == "home"
        assert profile.reference_longitude_deg == 76.366
        assert profile.reference_latitude_deg == 30.354
        assert profile.altitude_m == 30.0
        assert profile.lateral_footprint_m == 2.0
        assert profile.lateral_overlap == 0.10
        assert profile.start_goal_boundary_clearance_m == 0.5
        assert profile.minimum_start_goal_separation_m == 1.0

    def test_route_policy_preserved(self):
        route = parse().pipeline.vehicle_route
        assert route.return_to_reference is False
        assert route.connector_config.max_visibility_nodes == 512

    def test_ardupilot_policy_preserved(self):
        policy = parse().pipeline.ardupilot
        assert policy.end_action == END_ACTION_RTL
        assert policy.waypoint_hold_s == 0.0
        assert policy.include_takeoff is True
        assert policy.skip_initial_reference_waypoint is True
        assert policy.minimum_relative_altitude_m == 1.0

    def test_allow_idle_preserved(self):
        assert parse().pipeline.allow_idle_vehicles is True

    def test_to_dict_round_trip(self):
        config = parse()
        assert SwarmMissionOperationalConfig.from_dict(config.to_dict()) == config

    def test_json_round_trip(self):
        config = parse()
        assert SwarmMissionOperationalConfig.from_json(config.to_json()) == config

    def test_yaml_round_trip(self):
        config = parse()
        assert SwarmMissionOperationalConfig.from_yaml(config.to_yaml()) == config

    def test_json_is_deterministic(self):
        assert parse().to_json() == parse().to_json()

    def test_yaml_is_deterministic(self):
        assert parse().to_yaml() == parse().to_yaml()

    def test_json_has_trailing_newline(self):
        assert parse().to_json().endswith("\n")

    def test_yaml_has_schema_first(self):
        assert parse().to_yaml().splitlines()[0] == "schema_version: 1"

    def test_to_dict_uses_canonical_assignment_order(self):
        data = parse().to_dict()
        assert [item["partition_id"] for item in data["assignments"]] == [1, 2]

    def test_to_dict_uses_canonical_vehicle_order(self):
        data = parse().to_dict()
        assert [item["vehicle_id"] for item in data["vehicles"]] == [
            "drone-1",
            "drone-2",
        ]


@pytest.mark.parametrize(
    "missing",
    ["schema_version", "adapter", "assignments", "vehicles", "pipeline"],
)
def test_missing_top_level_fields(missing):
    payload = valid_payload()
    del payload[missing]
    with pytest.raises(SwarmMissionConfigError, match="missing required"):
        parse(payload)


def test_unknown_top_level_field():
    payload = valid_payload()
    payload["mystery"] = 1
    with pytest.raises(SwarmMissionConfigError, match="unknown field"):
        parse(payload)


@pytest.mark.parametrize("version", [0, 2, -1])
def test_unsupported_schema_version(version):
    with pytest.raises(SwarmMissionConfigError, match="unsupported schema_version"):
        parse(mutate(["schema_version"], version))


@pytest.mark.parametrize("version", [True, 1.0, "1", None])
def test_schema_version_must_be_integer(version):
    with pytest.raises(SwarmMissionConfigError, match="must be an integer"):
        parse(mutate(["schema_version"], version))


@pytest.mark.parametrize(
    "field",
    [
        "frame_id",
        "clearance_m",
        "min_component_area_m2",
        "coverage_gap_tolerance_m2",
        "coverage_gap_relative_tolerance",
        "partition_overlap_tolerance_m2",
    ],
)
def test_missing_adapter_fields(field):
    payload = valid_payload()
    del payload["adapter"][field]
    with pytest.raises(SwarmMissionConfigError, match="missing required"):
        parse(payload)


def test_unknown_adapter_field():
    payload = valid_payload()
    payload["adapter"]["unknown"] = 1
    with pytest.raises(SwarmMissionConfigError, match="unknown field"):
        parse(payload)


@pytest.mark.parametrize("field", ["partition_id", "vehicle_id"])
def test_missing_assignment_fields(field):
    payload = valid_payload()
    del payload["assignments"][0][field]
    with pytest.raises(SwarmMissionConfigError, match="missing required"):
        parse(payload)


def test_assignment_unknown_field():
    payload = valid_payload()
    payload["assignments"][0]["unknown"] = 1
    with pytest.raises(SwarmMissionConfigError, match="unknown field"):
        parse(payload)


def test_assignments_must_be_array():
    with pytest.raises(SwarmMissionConfigError, match="assignments must be an array"):
        parse(mutate(["assignments"], {}))


def test_empty_assignments_rejected_by_adapter():
    with pytest.raises(SwarmMissionConfigError, match="assignments must not be empty"):
        parse(mutate(["assignments"], []))


def test_duplicate_partition_assignment_rejected():
    payload = valid_payload()
    payload["assignments"][1]["partition_id"] = 2
    with pytest.raises(SwarmMissionConfigError, match="partition IDs must be unique"):
        parse(payload)


def test_unknown_assignment_vehicle_rejected():
    payload = valid_payload()
    payload["assignments"][0]["vehicle_id"] = "ghost"
    with pytest.raises(SwarmMissionConfigError, match="unknown vehicle"):
        parse(payload)


@pytest.mark.parametrize("field", ["vehicle_id", "reference", "coverage"])
def test_missing_vehicle_fields(field):
    payload = valid_payload()
    del payload["vehicles"][0][field]
    with pytest.raises(SwarmMissionConfigError, match="missing required"):
        parse(payload)


def test_vehicles_must_be_array():
    with pytest.raises(SwarmMissionConfigError, match="vehicles must be an array"):
        parse(mutate(["vehicles"], {}))


def test_empty_vehicles_rejected():
    with pytest.raises(SwarmMissionConfigError, match="vehicles must not be empty"):
        parse(mutate(["vehicles"], []))


def test_duplicate_vehicle_rejected():
    payload = valid_payload()
    payload["vehicles"][1]["vehicle_id"] = "drone-2"
    with pytest.raises(SwarmMissionConfigError, match="profile IDs must be unique"):
        parse(payload)


@pytest.mark.parametrize("field", ["type", "longitude_deg", "latitude_deg"])
def test_missing_reference_fields(field):
    payload = valid_payload()
    del payload["vehicles"][0]["reference"][field]
    with pytest.raises(SwarmMissionConfigError, match="missing required"):
        parse(payload)


@pytest.mark.parametrize(
    "field",
    [
        "altitude_m",
        "lateral_footprint_m",
        "lateral_overlap",
        "start_goal_boundary_clearance_m",
        "minimum_start_goal_separation_m",
    ],
)
def test_missing_coverage_fields(field):
    payload = valid_payload()
    del payload["vehicles"][0]["coverage"][field]
    with pytest.raises(SwarmMissionConfigError, match="missing required"):
        parse(payload)


@pytest.mark.parametrize("longitude", [-181, 181])
def test_invalid_longitude(longitude):
    payload = valid_payload()
    payload["vehicles"][0]["reference"]["longitude_deg"] = longitude
    with pytest.raises(SwarmMissionConfigError, match="longitude"):
        parse(payload)


@pytest.mark.parametrize("latitude", [-91, 91])
def test_invalid_latitude(latitude):
    payload = valid_payload()
    payload["vehicles"][0]["reference"]["latitude_deg"] = latitude
    with pytest.raises(SwarmMissionConfigError, match="latitude"):
        parse(payload)


@pytest.mark.parametrize("reference_type", ["", "base", None, 1])
def test_invalid_reference_type(reference_type):
    payload = valid_payload()
    payload["vehicles"][0]["reference"]["type"] = reference_type
    with pytest.raises(SwarmMissionConfigError, match="reference_type"):
        parse(payload)


@pytest.mark.parametrize("altitude", [0.5, -1.0])
def test_altitude_below_ardupilot_minimum_rejected(altitude):
    payload = valid_payload()
    payload["vehicles"][0]["coverage"]["altitude_m"] = altitude
    with pytest.raises(SwarmMissionConfigError, match="below"):
        parse(payload)


def test_equal_to_minimum_altitude_allowed():
    payload = valid_payload()
    payload["vehicles"][0]["coverage"]["altitude_m"] = 1.0
    assert parse(payload).adapter.vehicles[1].altitude_m == 1.0


@pytest.mark.parametrize("overlap", [-0.1, 1.0, 1.1])
def test_invalid_overlap(overlap):
    payload = valid_payload()
    payload["vehicles"][0]["coverage"]["lateral_overlap"] = overlap
    with pytest.raises(SwarmMissionConfigError, match="lateral_overlap"):
        parse(payload)


@pytest.mark.parametrize("field", ["allow_idle_vehicles", "route", "ardupilot"])
def test_missing_pipeline_fields(field):
    payload = valid_payload()
    del payload["pipeline"][field]
    with pytest.raises(SwarmMissionConfigError, match="missing required"):
        parse(payload)


@pytest.mark.parametrize("value", [1, 0, "true", None])
def test_allow_idle_must_be_bool(value):
    payload = valid_payload()
    payload["pipeline"]["allow_idle_vehicles"] = value
    with pytest.raises(SwarmMissionConfigError, match="must be a bool"):
        parse(payload)


@pytest.mark.parametrize("value", [1, 0, "false", None])
def test_return_to_reference_must_be_bool(value):
    payload = valid_payload()
    payload["pipeline"]["route"]["return_to_reference"] = value
    with pytest.raises(SwarmMissionConfigError, match="must be a bool"):
        parse(payload)


@pytest.mark.parametrize("value", [True, 1.0, 1, 0, -1])
def test_invalid_visibility_node_limit(value):
    payload = valid_payload()
    payload["pipeline"]["route"]["connector"]["max_visibility_nodes"] = value
    with pytest.raises(SwarmMissionConfigError, match="max_visibility_nodes"):
        parse(payload)


@pytest.mark.parametrize("end_action", [END_ACTION_RTL, END_ACTION_NONE])
def test_supported_non_landing_end_actions(end_action):
    payload = valid_payload()
    payload["pipeline"]["ardupilot"]["end_action"] = end_action
    assert parse(payload).pipeline.ardupilot.end_action == end_action


def test_land_at_reference_requires_return():
    payload = valid_payload()
    payload["pipeline"]["ardupilot"]["end_action"] = END_ACTION_LAND_AT_REFERENCE
    with pytest.raises(SwarmMissionConfigError, match="requires"):
        parse(payload)


def test_land_at_reference_allowed_with_return():
    payload = valid_payload()
    payload["pipeline"]["ardupilot"]["end_action"] = END_ACTION_LAND_AT_REFERENCE
    payload["pipeline"]["route"]["return_to_reference"] = True
    assert parse(payload).pipeline.ardupilot.end_action == END_ACTION_LAND_AT_REFERENCE


@pytest.mark.parametrize("value", [1, 0, "true", None])
def test_include_takeoff_must_be_bool(value):
    payload = valid_payload()
    payload["pipeline"]["ardupilot"]["include_takeoff"] = value
    with pytest.raises(SwarmMissionConfigError, match="must be a bool"):
        parse(payload)


@pytest.mark.parametrize("value", [1, 0, "true", None])
def test_skip_initial_reference_must_be_bool(value):
    payload = valid_payload()
    payload["pipeline"]["ardupilot"]["skip_initial_reference_waypoint"] = value
    with pytest.raises(SwarmMissionConfigError, match="must be a bool"):
        parse(payload)


def test_skip_initial_requires_takeoff():
    payload = valid_payload()
    payload["pipeline"]["ardupilot"]["include_takeoff"] = False
    with pytest.raises(SwarmMissionConfigError, match="requires include_takeoff"):
        parse(payload)


@pytest.mark.parametrize("text", ["", "{", "[]", "null"])
def test_invalid_json(text):
    with pytest.raises(SwarmMissionConfigError):
        SwarmMissionOperationalConfig.from_json(text)


@pytest.mark.parametrize("text", ["", "- item", ":::", "[unterminated"])
def test_invalid_yaml_or_wrong_root(text):
    with pytest.raises(SwarmMissionConfigError):
        SwarmMissionOperationalConfig.from_yaml(text)


def test_yaml_safe_loader_rejects_python_object_tag():
    text = "!!python/object/apply:os.system ['echo unsafe']"
    with pytest.raises(SwarmMissionConfigError, match="invalid YAML"):
        SwarmMissionOperationalConfig.from_yaml(text)


@pytest.mark.parametrize("suffix", [".json", ".yaml", ".yml"])
def test_write_and_read_round_trip(tmp_path: Path, suffix):
    config = parse()
    path = tmp_path / f"mission{suffix}"
    assert config.write(path) == path
    assert SwarmMissionOperationalConfig.read(path) == config
    assert load_swarm_mission_operational_config(path) == config


def test_json_write_is_parseable_by_stdlib(tmp_path: Path):
    path = parse().write(tmp_path / "mission.json")
    assert json.loads(path.read_text())["schema_version"] == 1


def test_yaml_write_is_parseable_by_pyyaml(tmp_path: Path):
    path = parse().write(tmp_path / "mission.yaml")
    assert yaml.safe_load(path.read_text())["schema_version"] == 1


def test_write_creates_parent_directory(tmp_path: Path):
    path = tmp_path / "nested" / "mission.yaml"
    parse().write(path)
    assert path.exists()


def test_write_replaces_existing_file_atomically(tmp_path: Path):
    path = tmp_path / "mission.json"
    path.write_text("old")
    parse().write(path)
    assert SwarmMissionOperationalConfig.read(path) == parse()


@pytest.mark.parametrize("suffix", ["", ".txt", ".toml"])
def test_unsupported_write_suffix(tmp_path: Path, suffix):
    with pytest.raises(SwarmMissionConfigError, match="must end"):
        parse().write(tmp_path / f"mission{suffix}")


@pytest.mark.parametrize("suffix", ["", ".txt", ".toml"])
def test_unsupported_read_suffix(tmp_path: Path, suffix):
    path = tmp_path / f"mission{suffix}"
    path.write_text("{}")
    with pytest.raises(SwarmMissionConfigError, match="must end"):
        SwarmMissionOperationalConfig.read(path)


def test_missing_file_reports_read_error(tmp_path: Path):
    with pytest.raises(SwarmMissionConfigError, match="could not read"):
        SwarmMissionOperationalConfig.read(tmp_path / "missing.yaml")


def test_constructor_rejects_wrong_adapter_type():
    with pytest.raises(SwarmMissionConfigError, match="adapter must"):
        SwarmMissionOperationalConfig(adapter=object(), pipeline=parse().pipeline)


def test_constructor_rejects_wrong_pipeline_type():
    with pytest.raises(SwarmMissionConfigError, match="pipeline must"):
        SwarmMissionOperationalConfig(adapter=parse().adapter, pipeline=object())


def test_from_dict_does_not_mutate_input():
    payload = valid_payload()
    original = copy.deepcopy(payload)
    parse(payload)
    assert payload == original
