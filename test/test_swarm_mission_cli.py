from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from pyproj import Transformer
from shapely.geometry import MultiPolygon, Polygon

import coverage_mission_pipeline.swarm_mission_cli as cli
from coverage_mission_pipeline import (
    ComponentPlanningSpec,
    ConnectorPlannerConfig,
    CoveragePlanningResult,
    CoverageWaypoint,
    GenericMissionPipelineConfig,
    LocalPoint2D,
    SequentialClientConfig,
    StartGoalPolicyConfig,
    SwarmMissionOperationalConfig,
    SwarmPartitionAssignment,
    SwarmPartitionsAdapterConfig,
    SwarmVehicleMissionProfile,
    VehicleRouteAssemblyConfig,
)
from coverage_mission_pipeline.ardupilot_mission import ArduPilotMissionBuildConfig


PROJECTED_CRS = "EPSG:32643"
TO_WGS84 = Transformer.from_crs(PROJECTED_CRS, "EPSG:4326", always_xy=True)
BOUNDARY = Polygon(
    [
        (300000.0, 3200000.0),
        (301000.0, 3200000.0),
        (301000.0, 3201000.0),
        (300000.0, 3201000.0),
    ]
)
PARTITION_1 = Polygon(
    [
        (300000.0, 3200000.0),
        (300500.0, 3200000.0),
        (300500.0, 3201000.0),
        (300000.0, 3201000.0),
    ]
)
PARTITION_2 = Polygon(
    [
        (300500.0, 3200000.0),
        (301000.0, 3200000.0),
        (301000.0, 3201000.0),
        (300500.0, 3201000.0),
    ]
)


def _ring(ring) -> list[list[float]]:
    return [list(TO_WGS84.transform(x, y)) for x, y in ring.coords]


def _geometry(geometry) -> list[dict]:
    polygons = list(geometry.geoms) if isinstance(geometry, MultiPolygon) else [geometry]
    return [
        {
            "exterior": _ring(polygon.exterior),
            "holes": [_ring(interior) for interior in polygon.interiors],
        }
        for polygon in polygons
    ]


def _payload() -> dict:
    return {
        "metadata": {
            "crs": {
                "coordinates": "EPSG:4326",
                "axis_order": ["longitude", "latitude"],
                "planning": PROJECTED_CRS,
            },
            "n_partitions": 2,
            "generation": {"random_seed": 42},
        },
        "boundary": _geometry(BOUNDARY),
        "partitions": [
            {"id": 1, "geometry": _geometry(PARTITION_1)},
            {"id": 2, "geometry": _geometry(PARTITION_2)},
        ],
        "no_go_zones": {"predetermined": []},
    }


def _profile(vehicle_id: str, x: float, y: float) -> SwarmVehicleMissionProfile:
    lon, lat = TO_WGS84.transform(x, y)
    return SwarmVehicleMissionProfile(
        vehicle_id=vehicle_id,
        reference_longitude_deg=lon,
        reference_latitude_deg=lat,
        altitude_m=30.0,
        lateral_footprint_m=2.0,
        lateral_overlap=0.10,
    )


def _operational() -> SwarmMissionOperationalConfig:
    adapter = SwarmPartitionsAdapterConfig(
        assignments=(
            SwarmPartitionAssignment(1, "drone-1"),
            SwarmPartitionAssignment(2, "drone-2"),
        ),
        vehicles=(
            _profile("drone-1", 300100.0, 3200500.0),
            _profile("drone-2", 300900.0, 3200500.0),
        ),
    )
    pipeline = GenericMissionPipelineConfig(
        vehicle_route=VehicleRouteAssemblyConfig(
            return_to_reference=False,
            connector_config=ConnectorPlannerConfig(max_visibility_nodes=512),
        ),
        ardupilot=ArduPilotMissionBuildConfig(),
        allow_idle_vehicles=True,
    )
    return SwarmMissionOperationalConfig(adapter=adapter, pipeline=pipeline)


def _write_inputs(tmp_path: Path) -> tuple[Path, Path]:
    mission = tmp_path / "mission_output.json"
    config = tmp_path / "swarm_mission.yaml"
    mission.write_text(json.dumps(_payload()), encoding="utf-8")
    _operational().write(config)
    return mission, config


class FakePlanner:
    def __init__(self, *, fail: bool = False):
        self.fail = fail
        self.requests = ()

    def run(self, requests):
        self.requests = tuple(requests)
        if self.fail:
            raise RuntimeError("planner exploded")
        results = []
        for request in self.requests:
            representative = request.component.polygon.representative_point()
            results.append(
                CoveragePlanningResult.from_request(
                    request,
                    response_message="fake coverage",
                    waypoints=(
                        CoverageWaypoint(
                            request.start.x_m,
                            request.start.y_m,
                            request.altitude_m,
                        ),
                        CoverageWaypoint(
                            representative.x,
                            representative.y,
                            request.altitude_m,
                        ),
                    ),
                )
            )
        return tuple(results)


class FakeNode:
    def __init__(self, name: str):
        self.name = name
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True


class FakeRuntime:
    def __init__(self, *, fail_init=False, fail_create=False, fail_shutdown=False):
        self.fail_init = fail_init
        self.fail_create = fail_create
        self.fail_shutdown = fail_shutdown
        self.initialized = False
        self.shutdown_called = False
        self.node = None

    def init(self):
        if self.fail_init:
            raise RuntimeError("init failed")
        self.initialized = True

    def create_node(self, node_name):
        if self.fail_create:
            raise RuntimeError("create failed")
        self.node = FakeNode(node_name)
        return self.node

    def shutdown(self):
        self.shutdown_called = True
        if self.fail_shutdown:
            raise RuntimeError("shutdown failed")


def _run(tmp_path: Path, **overrides):
    mission, config = _write_inputs(tmp_path)
    output = tmp_path / "output"
    runtime = overrides.pop("runtime", FakeRuntime())
    planner = overrides.pop("planner", FakePlanner())
    captured = {}

    def runtime_factory():
        return runtime

    def runner_builder(node, client_config):
        captured["node"] = node
        captured["client_config"] = client_config
        return planner

    result = cli.run_production_swarm_mission(
        mission,
        config,
        output,
        runtime_factory=overrides.pop("runtime_factory", runtime_factory),
        runner_builder=overrides.pop("runner_builder", runner_builder),
        **overrides,
    )
    return result, runtime, planner, captured, mission, config, output


def test_successful_run_publishes_output_bundle(tmp_path):
    result, _, _, _, _, _, output = _run(tmp_path)
    assert result.output_directory == output
    assert output.is_dir()


@pytest.mark.parametrize(
    "relative_path",
    [
        "generic-mission-manifest.json",
        "operational-config.normalized.json",
        "production-run.json",
        "component_routes/partition_1_component_1.route.json",
        "component_routes/partition_2_component_1.route.json",
        "complete_routes/drone-1.complete-route.json",
        "complete_routes/drone-2.complete-route.json",
        "ardupilot/drone-1.ardupilot-mission.json",
        "ardupilot/drone-2.ardupilot-mission.json",
        "ardupilot/drone-1.waypoints",
        "ardupilot/drone-2.waypoints",
    ],
)
def test_successful_bundle_contains_expected_files(tmp_path, relative_path):
    result, _, _, _, _, _, _ = _run(tmp_path)
    assert (result.output_directory / relative_path).is_file()


def test_runtime_is_initialized_and_shutdown(tmp_path):
    _, runtime, _, _, _, _, _ = _run(tmp_path)
    assert runtime.initialized
    assert runtime.shutdown_called


def test_node_is_destroyed(tmp_path):
    _, runtime, _, _, _, _, _ = _run(tmp_path)
    assert runtime.node.destroyed


def test_node_name_reaches_runtime(tmp_path):
    client = SequentialClientConfig(node_name="custom_pipeline_node")
    _, runtime, _, _, _, _, _ = _run(tmp_path, client_config=client)
    assert runtime.node.name == "custom_pipeline_node"


@pytest.mark.parametrize(
    "field,value",
    [
        ("service_name", "/custom_plan"),
        ("service_wait_timeout_s", 17.0),
        ("request_timeout_s", 41.0),
        ("node_name", "custom_node"),
    ],
)
def test_client_config_reaches_runner_builder(tmp_path, field, value):
    kwargs = {field: value}
    client = SequentialClientConfig(**kwargs)
    _, _, _, captured, _, _, _ = _run(tmp_path, client_config=client)
    assert getattr(captured["client_config"], field) == value


def test_planner_receives_every_component_once(tmp_path):
    _, _, planner, _, _, _, _ = _run(tmp_path)
    assert tuple(request.component.component_id for request in planner.requests) == (
        "partition_1_component_1",
        "partition_2_component_1",
    )


def test_production_record_is_written_deterministically(tmp_path):
    result, _, _, _, _, _, output = _run(tmp_path)
    on_disk = json.loads((output / cli.PRODUCTION_RUN_FILENAME).read_text())
    assert on_disk == result.production_record


def test_production_record_has_schema_and_algorithm(tmp_path):
    result, *_ = _run(tmp_path)
    assert result.production_record["schema_version"] == 1
    assert result.production_record["algorithm"] == cli.PRODUCTION_RUN_ALGORITHM


def test_production_record_contains_input_hashes(tmp_path):
    result, _, _, _, mission, config, _ = _run(tmp_path)
    expected_mission = hashlib.sha256(mission.read_bytes()).hexdigest()
    expected_config = hashlib.sha256(config.read_bytes()).hexdigest()
    assert result.production_record["inputs"]["mission_json"]["sha256"] == expected_mission
    assert result.production_record["inputs"]["operational_config"]["sha256"] == expected_config


def test_production_record_does_not_store_absolute_input_paths(tmp_path):
    result, _, _, _, mission, config, _ = _run(tmp_path)
    encoded = json.dumps(result.production_record)
    assert str(mission.resolve()) not in encoded
    assert str(config.resolve()) not in encoded


def test_normalized_config_round_trips(tmp_path):
    result, *_ = _run(tmp_path)
    loaded = SwarmMissionOperationalConfig.read(
        result.output_directory / cli.NORMALIZED_CONFIG_FILENAME
    )
    assert loaded.to_dict() == _operational().to_dict()


def test_qgc_files_have_expected_header(tmp_path):
    result, *_ = _run(tmp_path)
    for vehicle_id in ("drone-1", "drone-2"):
        text = (result.output_directory / "ardupilot" / f"{vehicle_id}.waypoints").read_text()
        assert text.startswith("QGC WPL 110\n")


def test_existing_output_rejected_before_runtime_creation(tmp_path):
    mission, config = _write_inputs(tmp_path)
    output = tmp_path / "output"
    output.mkdir()
    called = False

    def factory():
        nonlocal called
        called = True
        return FakeRuntime()

    with pytest.raises(cli.SwarmMissionCliError) as exc_info:
        cli.run_production_swarm_mission(
            mission,
            config,
            output,
            runtime_factory=factory,
        )
    assert exc_info.value.stage == "input_validation"
    assert exc_info.value.exit_code == cli.EXIT_INPUT_ERROR
    assert not called


@pytest.mark.parametrize("missing", ["mission", "config"])
def test_missing_input_rejected_before_runtime_creation(tmp_path, missing):
    mission, config = _write_inputs(tmp_path)
    if missing == "mission":
        mission.unlink()
    else:
        config.unlink()
    called = False

    def factory():
        nonlocal called
        called = True
        return FakeRuntime()

    with pytest.raises(cli.SwarmMissionCliError) as exc_info:
        cli.run_production_swarm_mission(
            mission,
            config,
            tmp_path / "output",
            runtime_factory=factory,
        )
    assert exc_info.value.stage == "input_validation"
    assert not called


@pytest.mark.parametrize("directory_input", ["mission", "config"])
def test_directory_input_is_rejected(tmp_path, directory_input):
    mission, config = _write_inputs(tmp_path)
    selected = mission if directory_input == "mission" else config
    selected.unlink()
    selected.mkdir()
    with pytest.raises(cli.SwarmMissionCliError, match="regular file"):
        cli.run_production_swarm_mission(
            mission,
            config,
            tmp_path / "output",
            runtime_factory=FakeRuntime,
        )


def test_invalid_yaml_rejected_before_ros(tmp_path):
    mission, config = _write_inputs(tmp_path)
    config.write_text("!!python/object/apply:os.system ['echo unsafe']", encoding="utf-8")
    called = False

    def factory():
        nonlocal called
        called = True
        return FakeRuntime()

    with pytest.raises(cli.SwarmMissionCliError) as exc_info:
        cli.run_production_swarm_mission(
            mission,
            config,
            tmp_path / "output",
            runtime_factory=factory,
        )
    assert exc_info.value.stage == "input_validation"
    assert "operational configuration" in exc_info.value.detail
    assert not called


def test_invalid_mission_json_rejected_before_ros(tmp_path):
    mission, config = _write_inputs(tmp_path)
    mission.write_text("{not-json", encoding="utf-8")
    called = False

    def factory():
        nonlocal called
        called = True
        return FakeRuntime()

    with pytest.raises(cli.SwarmMissionCliError) as exc_info:
        cli.run_production_swarm_mission(
            mission,
            config,
            tmp_path / "output",
            runtime_factory=factory,
        )
    assert exc_info.value.stage == "input_validation"
    assert "mission JSON" in exc_info.value.detail
    assert not called


def test_runtime_factory_failure_is_ros_error(tmp_path):
    mission, config = _write_inputs(tmp_path)

    def factory():
        raise RuntimeError("runtime unavailable")

    with pytest.raises(cli.SwarmMissionCliError) as exc_info:
        cli.run_production_swarm_mission(
            mission,
            config,
            tmp_path / "output",
            runtime_factory=factory,
        )
    assert exc_info.value.stage == "ros_initialization"
    assert exc_info.value.exit_code == cli.EXIT_ROS_ERROR


@pytest.mark.parametrize("failure", ["init", "create"])
def test_runtime_lifecycle_failure_is_ros_error(tmp_path, failure):
    mission, config = _write_inputs(tmp_path)
    runtime = FakeRuntime(
        fail_init=failure == "init",
        fail_create=failure == "create",
    )
    with pytest.raises(cli.SwarmMissionCliError) as exc_info:
        cli.run_production_swarm_mission(
            mission,
            config,
            tmp_path / "output",
            runtime_factory=lambda: runtime,
        )
    assert exc_info.value.stage == "ros_initialization"
    assert exc_info.value.exit_code == cli.EXIT_ROS_ERROR
    assert runtime.shutdown_called


def test_runner_builder_failure_is_ros_error_and_cleans_up(tmp_path):
    mission, config = _write_inputs(tmp_path)
    runtime = FakeRuntime()

    def builder(node, client_config):
        raise RuntimeError("client construction failed")

    with pytest.raises(cli.SwarmMissionCliError) as exc_info:
        cli.run_production_swarm_mission(
            mission,
            config,
            tmp_path / "output",
            runtime_factory=lambda: runtime,
            runner_builder=builder,
        )
    assert exc_info.value.stage == "ros_initialization"
    assert runtime.node.destroyed
    assert runtime.shutdown_called


def test_planner_failure_is_mission_pipeline_error(tmp_path):
    mission, config = _write_inputs(tmp_path)
    runtime = FakeRuntime()
    planner = FakePlanner(fail=True)
    with pytest.raises(cli.SwarmMissionCliError) as exc_info:
        cli.run_production_swarm_mission(
            mission,
            config,
            tmp_path / "output",
            runtime_factory=lambda: runtime,
            runner_builder=lambda node, client_config: planner,
        )
    assert exc_info.value.stage == "mission_pipeline"
    assert exc_info.value.exit_code == cli.EXIT_PLANNING_ERROR
    assert not (tmp_path / "output").exists()
    assert runtime.node.destroyed
    assert runtime.shutdown_called


def test_shutdown_failure_does_not_discard_success(tmp_path):
    runtime = FakeRuntime(fail_shutdown=True)
    result, *_ = _run(tmp_path, runtime=runtime)
    assert result.output_directory.exists()


def test_destroy_node_failure_does_not_discard_success(tmp_path):
    class BadNode(FakeNode):
        def destroy_node(self):
            self.destroyed = True
            raise RuntimeError("destroy failed")

    class Runtime(FakeRuntime):
        def create_node(self, node_name):
            self.node = BadNode(node_name)
            return self.node

    result, *_ = _run(tmp_path, runtime=Runtime())
    assert result.output_directory.exists()


def test_invalid_client_config_type_is_input_error(tmp_path):
    mission, config = _write_inputs(tmp_path)
    with pytest.raises(cli.SwarmMissionCliError) as exc_info:
        cli.run_production_swarm_mission(
            mission,
            config,
            tmp_path / "output",
            client_config=object(),
            runtime_factory=FakeRuntime,
        )
    assert exc_info.value.stage == "input_validation"


@pytest.mark.parametrize(
    "argv,attribute,expected",
    [
        (
            ["--mission-json", "m.json", "--config", "c.yaml", "--output", "out"],
            "service_name",
            "/plan_coverage",
        ),
        (
            ["--mission-json", "m.json", "--config", "c.yaml", "--output", "out"],
            "service_wait_timeout",
            10.0,
        ),
        (
            ["--mission-json", "m.json", "--config", "c.yaml", "--output", "out"],
            "request_timeout",
            30.0,
        ),
        (
            ["--mission-json", "m.json", "--config", "c.yaml", "--output", "out"],
            "node_name",
            "coverage_mission_pipeline",
        ),
        (
            [
                "--mission-json",
                "m.json",
                "--config",
                "c.yaml",
                "--output",
                "out",
                "--service-name",
                "/other",
            ],
            "service_name",
            "/other",
        ),
        (
            [
                "--mission-json",
                "m.json",
                "--config",
                "c.yaml",
                "--output",
                "out",
                "--service-wait-timeout",
                "12.5",
            ],
            "service_wait_timeout",
            12.5,
        ),
        (
            [
                "--mission-json",
                "m.json",
                "--config",
                "c.yaml",
                "--output",
                "out",
                "--request-timeout",
                "55",
            ],
            "request_timeout",
            55.0,
        ),
        (
            [
                "--mission-json",
                "m.json",
                "--config",
                "c.yaml",
                "--output",
                "out",
                "--node-name",
                "mission_node",
            ],
            "node_name",
            "mission_node",
        ),
    ],
)
def test_argument_parser_values(argv, attribute, expected):
    args = cli.build_argument_parser().parse_args(argv)
    assert getattr(args, attribute) == expected


@pytest.mark.parametrize("missing_flag", ["--mission-json", "--config", "--output"])
def test_argument_parser_requires_all_three_paths(missing_flag):
    values = {
        "--mission-json": "m.json",
        "--config": "c.yaml",
        "--output": "out",
    }
    argv = []
    for flag, value in values.items():
        if flag != missing_flag:
            argv.extend([flag, value])
    with pytest.raises(SystemExit) as exc_info:
        cli.build_argument_parser().parse_args(argv)
    assert exc_info.value.code == 2


def test_main_success_prints_pass(monkeypatch, capsys, tmp_path):
    output = tmp_path / "output"
    output.mkdir()
    fake = SimpleNamespace(
        pipeline=SimpleNamespace(
            to_summary_dict=lambda: {
                "adapter": {"component_count": 3},
                "mission": {"active_vehicle_ids": ["drone-1", "drone-2"]},
            }
        ),
        output_directory=output,
    )
    monkeypatch.setattr(cli, "run_production_swarm_mission", lambda *args, **kwargs: fake)
    code = cli.main(
        [
            "--mission-json",
            "m.json",
            "--config",
            "c.yaml",
            "--output",
            str(output),
        ]
    )
    captured = capsys.readouterr()
    assert code == 0
    assert "PASS production swarm mission" in captured.out
    assert "3 component(s)" in captured.out
    assert "2 active vehicle(s)" in captured.out


@pytest.mark.parametrize(
    "stage,exit_code",
    [
        ("input_validation", cli.EXIT_INPUT_ERROR),
        ("ros_initialization", cli.EXIT_ROS_ERROR),
        ("mission_pipeline", cli.EXIT_PLANNING_ERROR),
        ("artifact_write", cli.EXIT_ARTIFACT_ERROR),
    ],
)
def test_main_returns_stable_cli_error_code(monkeypatch, capsys, stage, exit_code):
    def fail(*args, **kwargs):
        raise cli.SwarmMissionCliError(stage, "boom", exit_code)

    monkeypatch.setattr(cli, "run_production_swarm_mission", fail)
    code = cli.main(
        [
            "--mission-json",
            "m.json",
            "--config",
            "c.yaml",
            "--output",
            "out",
        ]
    )
    captured = capsys.readouterr()
    assert code == exit_code
    assert f"FAILED [{stage}]: boom" in captured.err


def test_main_invalid_client_options_return_input_error(capsys):
    code = cli.main(
        [
            "--mission-json",
            "m.json",
            "--config",
            "c.yaml",
            "--output",
            "out",
            "--request-timeout",
            "0",
        ]
    )
    assert code == cli.EXIT_INPUT_ERROR
    assert "FAILED [input_validation]" in capsys.readouterr().err


def test_main_unexpected_error_returns_planning_error(monkeypatch, capsys):
    monkeypatch.setattr(
        cli,
        "run_production_swarm_mission",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("unexpected")),
    )
    code = cli.main(
        [
            "--mission-json",
            "m.json",
            "--config",
            "c.yaml",
            "--output",
            "out",
        ]
    )
    assert code == cli.EXIT_PLANNING_ERROR
    assert "FAILED [unexpected]: unexpected" in capsys.readouterr().err


def test_cli_error_validates_constructor():
    with pytest.raises(ValueError):
        cli.SwarmMissionCliError("", "message", 1)
    with pytest.raises(ValueError):
        cli.SwarmMissionCliError("stage", "", 1)
    with pytest.raises(ValueError):
        cli.SwarmMissionCliError("stage", "message", 0)
    with pytest.raises(ValueError):
        cli.SwarmMissionCliError("stage", "message", True)


def test_production_result_validates_types(tmp_path):
    output = tmp_path / "out"
    output.mkdir()
    with pytest.raises(ValueError):
        cli.ProductionMissionRunResult(object(), output, {})
    with pytest.raises(ValueError):
        cli.ProductionMissionRunResult(
            SimpleNamespace(),
            tmp_path / "missing",
            {},
        )
