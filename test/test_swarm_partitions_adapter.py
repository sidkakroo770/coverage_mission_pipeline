from __future__ import annotations

import json
from pathlib import Path

import pytest
from pyproj import Transformer
from shapely.geometry import MultiPolygon, Polygon

from coverage_mission_pipeline.generic_mission_pipeline import (
    GenericMissionPipelineError,
)
from coverage_mission_pipeline.planning_result import (
    CoveragePlanningResult,
    CoverageWaypoint,
)
from coverage_mission_pipeline.swarm_partitions_adapter import (
    SWARM_PARTITIONS_ADAPTER_ALGORITHM,
    SwarmPartitionAssignment,
    SwarmPartitionsAdapterConfig,
    SwarmPartitionsAdapterError,
    SwarmPartitionsPipelineResult,
    SwarmVehicleMissionProfile,
    adapt_swarm_partitions_payload,
    load_swarm_partitions_json,
    run_swarm_partitions_mission_pipeline,
)


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


def _payload(
    *,
    boundary=BOUNDARY,
    partitions=(PARTITION_1, PARTITION_2),
    predetermined=(),
    include_dynamic=False,
    dynamic=(),
):
    no_go = {
        "predetermined": [
            {"name": name, "geometry": _geometry(geometry)}
            for name, geometry in predetermined
        ]
    }
    if include_dynamic:
        no_go["dynamic"] = [
            {"id": index + 1, "geometry": _geometry(geometry)}
            for index, geometry in enumerate(dynamic)
        ]
    return {
        "metadata": {
            "crs": {
                "coordinates": "EPSG:4326",
                "axis_order": ["longitude", "latitude"],
                "planning": PROJECTED_CRS,
            },
            "n_partitions": len(partitions),
            "generation": {"random_seed": 42},
        },
        "boundary": _geometry(boundary),
        "partitions": [
            {"id": index + 1, "geometry": _geometry(partition)}
            for index, partition in enumerate(partitions)
        ],
        "no_go_zones": no_go,
    }


def _lonlat(x: float, y: float) -> tuple[float, float]:
    return TO_WGS84.transform(x, y)


def _profile(vehicle_id: str, x: float, y: float, **overrides):
    lon, lat = _lonlat(x, y)
    values = dict(
        vehicle_id=vehicle_id,
        reference_longitude_deg=lon,
        reference_latitude_deg=lat,
        altitude_m=30.0,
        lateral_footprint_m=2.0,
        lateral_overlap=0.10,
    )
    values.update(overrides)
    return SwarmVehicleMissionProfile(**values)


def _config(**overrides):
    values = dict(
        assignments=(
            SwarmPartitionAssignment(1, "drone-1"),
            SwarmPartitionAssignment(2, "drone-2"),
        ),
        vehicles=(
            _profile("drone-1", 300100.0, 3200500.0),
            _profile("drone-2", 300900.0, 3200500.0),
        ),
    )
    values.update(overrides)
    return SwarmPartitionsAdapterConfig(**values)


class FakePlanner:
    def run(self, requests):
        results = []
        for request in requests:
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


def test_happy_path_builds_generic_definition():
    result = adapt_swarm_partitions_payload(_payload(), _config())
    assert result.algorithm == SWARM_PARTITIONS_ADAPTER_ALGORITHM
    assert result.partition_count == 2
    assert result.component_count == 2
    assert result.definition.vehicle_ids == ("drone-1", "drone-2")


def test_frame_origin_uses_projected_boundary_minimum():
    result = adapt_swarm_partitions_payload(_payload(), _config())
    assert result.frame.origin_easting_m == pytest.approx(300000.0)
    assert result.frame.origin_northing_m == pytest.approx(3200000.0)


def test_frame_uses_exported_planning_crs():
    result = adapt_swarm_partitions_payload(_payload(), _config())
    assert result.frame.projected_crs == PROJECTED_CRS
    assert result.planning_crs == PROJECTED_CRS


def test_random_seed_is_preserved():
    result = adapt_swarm_partitions_payload(_payload(), _config())
    assert result.random_seed == 42


def test_partition_assignments_reach_components():
    result = adapt_swarm_partitions_payload(_payload(), _config())
    assignments = {
        component.source_region_id: component.assigned_vehicle_id
        for component in result.definition.components
    }
    assert assignments == {
        "partition_1": "drone-1",
        "partition_2": "drone-2",
    }


def test_component_ids_are_deterministic():
    result = adapt_swarm_partitions_payload(_payload(), _config())
    assert tuple(component.component_id for component in result.definition.components) == (
        "partition_1_component_1",
        "partition_2_component_1",
    )


def test_planning_specs_use_vehicle_reference_as_explicit_anchor():
    result = adapt_swarm_partitions_payload(_payload(), _config())
    references = {
        reference.vehicle_id: reference.position
        for reference in result.definition.vehicle_references
    }
    for spec, component in zip(
        result.definition.planning_specs,
        result.definition.components,
    ):
        vehicle_id = component.assigned_vehicle_id
        assert spec.start_anchor == references[vehicle_id]
        assert spec.goal_anchor == references[vehicle_id]


def test_planning_parameters_come_from_vehicle_profile():
    config = _config(
        vehicles=(
            _profile(
                "drone-1",
                300100.0,
                3200500.0,
                altitude_m=41.0,
                lateral_footprint_m=3.0,
                lateral_overlap=0.25,
                start_goal_boundary_clearance_m=1.5,
            ),
            _profile("drone-2", 300900.0, 3200500.0),
        )
    )
    result = adapt_swarm_partitions_payload(_payload(), config)
    first = result.definition.planning_specs[0]
    assert first.altitude_m == 41.0
    assert first.lateral_footprint_m == 3.0
    assert first.lateral_overlap == 0.25
    assert first.start_goal_policy.boundary_clearance_m == 1.5


def test_all_vehicles_receive_authoritative_global_safe_area():
    result = adapt_swarm_partitions_payload(_payload(), _config())
    free_spaces = result.definition.free_space_by_vehicle_id
    assert free_spaces["drone-1"].equals(free_spaces["drone-2"])
    assert free_spaces["drone-1"].equals(result.safe_area_local)


def test_dynamic_key_is_not_required():
    result = adapt_swarm_partitions_payload(_payload(include_dynamic=False), _config())
    assert result.dynamic_exclusion_count == 0


def test_optional_dynamic_geometry_is_supported_for_future_reenable():
    obstacle = Polygon(
        [
            (300450, 3200400),
            (300550, 3200400),
            (300550, 3200600),
            (300450, 3200600),
        ]
    )
    left = PARTITION_1.difference(obstacle)
    right = PARTITION_2.difference(obstacle)
    result = adapt_swarm_partitions_payload(
        _payload(
            partitions=(left, right),
            include_dynamic=True,
            dynamic=(obstacle,),
        ),
        _config(),
    )
    assert result.dynamic_exclusion_count == 1
    assert len(result.exclusions_projected) == 1


def test_predetermined_exclusion_is_applied():
    obstacle = Polygon(
        [
            (300450, 3200400),
            (300550, 3200400),
            (300550, 3200600),
            (300450, 3200600),
        ]
    )
    result = adapt_swarm_partitions_payload(
        _payload(
            partitions=(PARTITION_1.difference(obstacle), PARTITION_2.difference(obstacle)),
            predetermined=(("tower", obstacle),),
        ),
        _config(),
    )
    assert result.safe_area_projected.area == pytest.approx(
        BOUNDARY.area - obstacle.area,
        abs=1e-4,
    )


def test_clearance_uses_global_safe_area_rule():
    result = adapt_swarm_partitions_payload(
        _payload(),
        _config(clearance_m=10.0),
    )
    assert result.safe_area_projected.area < BOUNDARY.area
    assert result.safe_area_projected.bounds == pytest.approx(
        (300010.0, 3200010.0, 300990.0, 3200990.0)
    )


def test_multipolygon_partition_preserves_every_component():
    island_a = Polygon([(300000, 3200000), (300200, 3200000), (300200, 3201000), (300000, 3201000)])
    island_b = Polygon([(300300, 3200000), (300500, 3200000), (300500, 3201000), (300300, 3201000)])
    first = MultiPolygon([island_a, island_b])
    gap = Polygon([(300200, 3200000), (300300, 3200000), (300300, 3201000), (300200, 3201000)])
    second = PARTITION_2.union(gap)
    result = adapt_swarm_partitions_payload(
        _payload(partitions=(first, second)),
        _config(),
    )
    assert result.component_ids_by_partition_id[1] == (
        "partition_1_component_1",
        "partition_1_component_2",
    )
    assert result.component_ids_by_partition_id[2] == (
        "partition_2_component_1",
        "partition_2_component_2",
    )
    assert result.component_count == 4


def test_polygon_hole_is_preserved_when_matching_no_go_zone():
    hole = Polygon([(300200, 3200400), (300300, 3200400), (300300, 3200600), (300200, 3200600)])
    first = PARTITION_1.difference(hole)
    result = adapt_swarm_partitions_payload(
        _payload(
            partitions=(first, PARTITION_2),
            predetermined=(("hole", hole),),
        ),
        _config(),
    )
    first_component = result.definition.components[0]
    assert len(first_component.polygon.interiors) == 1


def test_extra_vehicle_profile_becomes_idle_vehicle():
    result = adapt_swarm_partitions_payload(
        _payload(),
        _config(
            vehicles=(
                _profile("drone-1", 300100, 3200500),
                _profile("drone-2", 300900, 3200500),
                _profile("drone-3", 300700, 3200200),
            )
        ),
    )
    assert result.definition.vehicle_ids == ("drone-1", "drone-2", "drone-3")


def test_one_vehicle_may_receive_multiple_partitions():
    result = adapt_swarm_partitions_payload(
        _payload(),
        _config(
            assignments=(
                SwarmPartitionAssignment(1, "drone-1"),
                SwarmPartitionAssignment(2, "drone-1"),
            ),
            vehicles=(_profile("drone-1", 300100, 3200500),),
        ),
    )
    assert {component.assigned_vehicle_id for component in result.definition.components} == {
        "drone-1"
    }


def test_load_json_file(tmp_path: Path):
    path = tmp_path / "mission.json"
    path.write_text(json.dumps(_payload()), encoding="utf-8")
    result = load_swarm_partitions_json(path, _config())
    assert result.partition_count == 2


def test_load_missing_file_rejected(tmp_path: Path):
    with pytest.raises(SwarmPartitionsAdapterError, match="could not read"):
        load_swarm_partitions_json(tmp_path / "missing.json", _config())


def test_load_invalid_json_rejected(tmp_path: Path):
    path = tmp_path / "bad.json"
    path.write_text("{", encoding="utf-8")
    with pytest.raises(SwarmPartitionsAdapterError, match="invalid JSON"):
        load_swarm_partitions_json(path, _config())


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda value: value.pop("boundary"), "missing required"),
        (lambda value: value.__setitem__("extra", 1), "unknown field"),
        (lambda value: value["metadata"].pop("crs"), "missing required"),
        (
            lambda value: value["metadata"]["crs"].__setitem__(
                "coordinates", "EPSG:3857"
            ),
            "must resolve to EPSG:4326",
        ),
        (
            lambda value: value["metadata"]["crs"].__setitem__(
                "axis_order", ["latitude", "longitude"]
            ),
            "axis_order",
        ),
        (
            lambda value: value["metadata"]["crs"].__setitem__(
                "planning", "EPSG:4326"
            ),
            "must be a projected CRS",
        ),
        (
            lambda value: value["metadata"].__setitem__("n_partitions", 3),
            "does not match",
        ),
        (
            lambda value: value["metadata"]["generation"].__setitem__(
                "random_seed", True
            ),
            "must be an integer",
        ),
        (
            lambda value: value.__setitem__("boundary", []),
            "exactly one",
        ),
        (
            lambda value: value["partitions"].__setitem__(
                1, {"id": 1, "geometry": value["partitions"][1]["geometry"]}
            ),
            "duplicate partition",
        ),
        (
            lambda value: value["partitions"][1].__setitem__("id", 3),
            "sequential",
        ),
        (
            lambda value: value["no_go_zones"].pop("predetermined"),
            "missing required",
        ),
    ],
)
def test_schema_errors_are_fail_closed(mutation, message):
    payload = _payload()
    mutation(payload)
    with pytest.raises(SwarmPartitionsAdapterError, match=message):
        adapt_swarm_partitions_payload(payload, _config())


@pytest.mark.parametrize(
    ("coordinate", "message"),
    [
        ([1.0], r"must be \[longitude, latitude\]"),
        ([181.0, 10.0], r"must be in \[-180, 180\]"),
        ([10.0, 91.0], r"must be in \[-90, 90\]"),
        ([float("nan"), 10.0], "must be finite"),
    ],
)
def test_bad_ring_coordinate_rejected(coordinate, message):
    payload = _payload()
    payload["boundary"][0]["exterior"][0] = coordinate
    with pytest.raises(SwarmPartitionsAdapterError, match=message):
        adapt_swarm_partitions_payload(payload, _config())


def test_ring_with_fewer_than_three_points_rejected():
    payload = _payload()
    payload["boundary"][0]["exterior"] = [[76.0, 30.0], [76.1, 30.0]]
    with pytest.raises(SwarmPartitionsAdapterError, match="three distinct"):
        adapt_swarm_partitions_payload(payload, _config())


def test_self_intersecting_polygon_rejected():
    payload = _payload()
    projected = [
        (300000, 3200000),
        (301000, 3201000),
        (301000, 3200000),
        (300000, 3201000),
        (300000, 3200000),
    ]
    payload["boundary"][0]["exterior"] = [
        list(TO_WGS84.transform(x, y)) for x, y in projected
    ]
    with pytest.raises(SwarmPartitionsAdapterError, match="invalid after projection"):
        adapt_swarm_partitions_payload(payload, _config())


def test_partition_outside_boundary_rejected():
    outside = Polygon([(301000, 3200000), (301100, 3200000), (301100, 3200100), (301000, 3200100)])
    payload = _payload(partitions=(PARTITION_1, PARTITION_2.union(outside)))
    with pytest.raises(SwarmPartitionsAdapterError, match="extends outside"):
        adapt_swarm_partitions_payload(payload, _config())


def test_partition_overlap_rejected():
    overlapping = Polygon([(300400, 3200000), (301000, 3200000), (301000, 3201000), (300400, 3201000)])
    with pytest.raises(SwarmPartitionsAdapterError, match="overlap"):
        adapt_swarm_partitions_payload(
            _payload(partitions=(PARTITION_1, overlapping)),
            _config(),
        )


def test_coverage_gap_rejected():
    first = Polygon([(300000, 3200000), (300490, 3200000), (300490, 3201000), (300000, 3201000)])
    with pytest.raises(SwarmPartitionsAdapterError, match="do not cover"):
        adapt_swarm_partitions_payload(
            _payload(partitions=(first, PARTITION_2)),
            _config(),
        )


def test_small_coverage_gap_can_be_explicitly_tolerated():
    first = Polygon([(300000, 3200000), (300499.999, 3200000), (300499.999, 3201000), (300000, 3201000)])
    result = adapt_swarm_partitions_payload(
        _payload(partitions=(first, PARTITION_2)),
        _config(coverage_gap_tolerance_m2=2.0),
    )
    assert result.component_count == 2


def test_minimum_component_area_may_fail_closed_when_partition_disappears():
    with pytest.raises(SwarmPartitionsAdapterError, match="no plannable component"):
        adapt_swarm_partitions_payload(
            _payload(),
            _config(min_component_area_m2=600000.0),
        )


def test_reference_outside_safe_area_rejected():
    config = _config(
        vehicles=(
            _profile("drone-1", 299900, 3200500),
            _profile("drone-2", 300900, 3200500),
        )
    )
    with pytest.raises(SwarmPartitionsAdapterError, match="outside the safe area"):
        adapt_swarm_partitions_payload(_payload(), config)


def test_missing_partition_assignment_rejected():
    config = _config(
        assignments=(SwarmPartitionAssignment(1, "drone-1"),)
    )
    with pytest.raises(SwarmPartitionsAdapterError, match="exactly one entry"):
        adapt_swarm_partitions_payload(_payload(), config)


def test_extra_partition_assignment_rejected():
    config = _config(
        assignments=(
            SwarmPartitionAssignment(1, "drone-1"),
            SwarmPartitionAssignment(2, "drone-2"),
            SwarmPartitionAssignment(3, "drone-1"),
        )
    )
    with pytest.raises(SwarmPartitionsAdapterError, match="unexpected: 3"):
        adapt_swarm_partitions_payload(_payload(), config)


def test_duplicate_assignment_partition_id_rejected():
    with pytest.raises(SwarmPartitionsAdapterError, match="partition IDs must be unique"):
        SwarmPartitionsAdapterConfig(
            assignments=(
                SwarmPartitionAssignment(1, "drone-1"),
                SwarmPartitionAssignment(1, "drone-2"),
            ),
            vehicles=(
                _profile("drone-1", 300100, 3200500),
                _profile("drone-2", 300900, 3200500),
            ),
        )


def test_duplicate_vehicle_profile_id_rejected():
    with pytest.raises(SwarmPartitionsAdapterError, match="profile IDs must be unique"):
        SwarmPartitionsAdapterConfig(
            assignments=(SwarmPartitionAssignment(1, "drone-1"),),
            vehicles=(
                _profile("drone-1", 300100, 3200500),
                _profile("drone-1", 300200, 3200500),
            ),
        )


def test_assignment_unknown_vehicle_rejected():
    with pytest.raises(SwarmPartitionsAdapterError, match="unknown vehicle"):
        SwarmPartitionsAdapterConfig(
            assignments=(SwarmPartitionAssignment(1, "drone-x"),),
            vehicles=(_profile("drone-1", 300100, 3200500),),
        )


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"lateral_footprint_m": 0.0}, "greater than zero"),
        ({"lateral_overlap": 1.0}, r"range \[0, 1\)"),
        ({"reference_longitude_deg": 181.0}, r"\[-180, 180\]"),
        ({"reference_latitude_deg": -91.0}, r"\[-90, 90\]"),
        ({"start_goal_boundary_clearance_m": -1.0}, "non-negative"),
        ({"minimum_start_goal_separation_m": -1.0}, "non-negative"),
        ({"reference_type": "invalid"}, "reference_type"),
    ],
)
def test_vehicle_profile_validation(overrides, message):
    lon, lat = _lonlat(300100, 3200500)
    values = dict(
        vehicle_id="drone-1",
        reference_longitude_deg=lon,
        reference_latitude_deg=lat,
        altitude_m=30.0,
        lateral_footprint_m=2.0,
        lateral_overlap=0.1,
    )
    values.update(overrides)
    with pytest.raises(SwarmPartitionsAdapterError, match=message):
        SwarmVehicleMissionProfile(**values)


def test_duplicate_predetermined_names_rejected():
    obstacle = Polygon([(300100, 3200100), (300200, 3200100), (300200, 3200200), (300100, 3200200)])
    payload = _payload(
        partitions=(PARTITION_1.difference(obstacle), PARTITION_2),
        predetermined=(("same", obstacle), ("same", obstacle)),
    )
    with pytest.raises(SwarmPartitionsAdapterError, match="duplicate predetermined"):
        adapt_swarm_partitions_payload(payload, _config())


def test_empty_predetermined_name_rejected():
    obstacle = Polygon([(300100, 3200100), (300200, 3200100), (300200, 3200200), (300100, 3200200)])
    payload = _payload(
        partitions=(PARTITION_1.difference(obstacle), PARTITION_2),
        predetermined=(("", obstacle),),
    )
    with pytest.raises(SwarmPartitionsAdapterError, match="name must not be empty"):
        adapt_swarm_partitions_payload(payload, _config())


def test_dynamic_ids_must_be_sequential():
    obstacle = Polygon([(300100, 3200100), (300200, 3200100), (300200, 3200200), (300100, 3200200)])
    payload = _payload(
        partitions=(PARTITION_1.difference(obstacle), PARTITION_2),
        include_dynamic=True,
        dynamic=(obstacle,),
    )
    payload["no_go_zones"]["dynamic"][0]["id"] = 2
    with pytest.raises(SwarmPartitionsAdapterError, match="sequential"):
        adapt_swarm_partitions_payload(payload, _config())


def test_summary_is_deterministic():
    first = adapt_swarm_partitions_payload(_payload(), _config())
    second = adapt_swarm_partitions_payload(_payload(), _config())
    assert first.to_summary_dict() == second.to_summary_dict()


def test_end_to_end_convenience_runner_from_payload():
    result = run_swarm_partitions_mission_pipeline(
        _payload(),
        _config(),
        FakePlanner(),
    )
    assert isinstance(result, SwarmPartitionsPipelineResult)
    assert result.mission.active_vehicle_ids == ("drone-1", "drone-2")
    assert len(result.mission.ardupilot_missions) == 2


def test_end_to_end_convenience_runner_from_file(tmp_path: Path):
    path = tmp_path / "mission.json"
    path.write_text(json.dumps(_payload()), encoding="utf-8")
    result = run_swarm_partitions_mission_pipeline(
        path,
        _config(),
        FakePlanner(),
    )
    assert result.adapter.partition_count == 2


def test_end_to_end_propagates_planner_failure():
    class BrokenPlanner:
        def run(self, requests):
            raise RuntimeError("planner down")

    with pytest.raises(GenericMissionPipelineError, match="coverage_planning"):
        run_swarm_partitions_mission_pipeline(
            _payload(),
            _config(),
            BrokenPlanner(),
        )


def test_pipeline_summary_contains_adapter_and_mission_sections():
    result = run_swarm_partitions_mission_pipeline(
        _payload(),
        _config(),
        FakePlanner(),
    )
    summary = result.to_summary_dict()
    assert set(summary) == {"adapter", "mission"}
    assert summary["adapter"]["partition_count"] == 2
    assert summary["mission"]["component_count"] == 2
