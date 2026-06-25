#!/usr/bin/env python3
"""Tests for route serialization and coordinate conversion."""

from copy import deepcopy
import json
import math

import pytest
from pyproj import Transformer

from coverage_mission_pipeline.planning_result import (
    CoveragePlanningResult,
    CoverageWaypoint,
)
from coverage_mission_pipeline.prepared_component import LocalCartesianFrame
from coverage_mission_pipeline.route_record import (
    CoverageRouteRecord,
    GeographicWaypoint,
    ProjectedWaypoint,
    ROUTE_SCHEMA_VERSION,
    RouteRecordError,
    geographic_to_local,
    geographic_to_projected,
    local_to_geographic,
    local_to_projected,
    make_route_records,
    projected_to_geographic,
    projected_to_local,
)


@pytest.fixture
def frame() -> LocalCartesianFrame:
    return LocalCartesianFrame(
        frame_id="map",
        projected_crs="EPSG:32643",
        origin_easting_m=631285.61,
        origin_northing_m=3358862.37,
    )


@pytest.fixture
def result() -> CoveragePlanningResult:
    return CoveragePlanningResult(
        request_id="request-1",
        component_id="partition-1_component_1",
        source_region_id="partition-1",
        assigned_vehicle_id="drone-1",
        frame_id="map",
        response_message="planned 3 waypoints",
        waypoints=(
            CoverageWaypoint(0.0, 0.0, 30.0),
            CoverageWaypoint(10.0, 5.0, 30.0),
            CoverageWaypoint(-2.5, 12.0, 30.0),
        ),
    )


@pytest.fixture
def record(result, frame) -> CoverageRouteRecord:
    return CoverageRouteRecord.from_result(result, frame)


def test_local_to_projected_adds_origin(frame) -> None:
    point = local_to_projected(CoverageWaypoint(10.0, -5.0, 30.0), frame)
    assert point.easting_m == pytest.approx(631295.61)
    assert point.northing_m == pytest.approx(3358857.37)
    assert point.altitude_m == pytest.approx(30.0)


def test_projected_to_local_subtracts_origin(frame) -> None:
    point = projected_to_local(ProjectedWaypoint(631295.61, 3358857.37, 30.0), frame)
    assert point.x_m == pytest.approx(10.0)
    assert point.y_m == pytest.approx(-5.0)
    assert point.z_m == pytest.approx(30.0)


def test_local_projected_round_trip(frame) -> None:
    original = CoverageWaypoint(-123.456, 98.765, -4.0)
    recovered = projected_to_local(local_to_projected(original, frame), frame)
    assert recovered.x_m == pytest.approx(original.x_m, abs=1.0e-9)
    assert recovered.y_m == pytest.approx(original.y_m, abs=1.0e-9)
    assert recovered.z_m == original.z_m


def test_projected_to_geographic_matches_pyproj(frame) -> None:
    projected = ProjectedWaypoint(631285.61, 3358862.37, 30.0)
    converted = projected_to_geographic(projected, frame)
    expected_lon, expected_lat = Transformer.from_crs(
        "EPSG:32643", "EPSG:4326", always_xy=True
    ).transform(projected.easting_m, projected.northing_m)
    assert converted.longitude_deg == pytest.approx(expected_lon, abs=1.0e-10)
    assert converted.latitude_deg == pytest.approx(expected_lat, abs=1.0e-10)


def test_geographic_projected_round_trip(frame) -> None:
    original = ProjectedWaypoint(631310.25, 3358900.75, 42.0)
    recovered = geographic_to_projected(projected_to_geographic(original, frame), frame)
    assert recovered.easting_m == pytest.approx(original.easting_m, abs=1.0e-5)
    assert recovered.northing_m == pytest.approx(original.northing_m, abs=1.0e-5)
    assert recovered.altitude_m == original.altitude_m


def test_local_geographic_round_trip(frame) -> None:
    original = CoverageWaypoint(14.25, -8.75, 12.5)
    recovered = geographic_to_local(local_to_geographic(original, frame), frame)
    assert recovered.x_m == pytest.approx(original.x_m, abs=1.0e-5)
    assert recovered.y_m == pytest.approx(original.y_m, abs=1.0e-5)
    assert recovered.z_m == original.z_m


def test_altitude_is_not_reprojected(frame) -> None:
    original = CoverageWaypoint(1.0, 2.0, -123.5)
    geographic = local_to_geographic(original, frame)
    assert geographic.altitude_m == -123.5


def test_projected_waypoint_rejects_nonfinite() -> None:
    with pytest.raises(RouteRecordError, match="finite"):
        ProjectedWaypoint(math.inf, 1.0, 2.0)


def test_projected_waypoint_rejects_bool() -> None:
    with pytest.raises(RouteRecordError, match="number"):
        ProjectedWaypoint(True, 1.0, 2.0)


def test_geographic_waypoint_rejects_bad_longitude() -> None:
    with pytest.raises(RouteRecordError, match="longitude"):
        GeographicWaypoint(181.0, 0.0, 1.0)


def test_geographic_waypoint_rejects_bad_latitude() -> None:
    with pytest.raises(RouteRecordError, match="latitude"):
        GeographicWaypoint(0.0, -91.0, 1.0)


def test_local_to_projected_rejects_wrong_waypoint(frame) -> None:
    with pytest.raises(RouteRecordError, match="CoverageWaypoint"):
        local_to_projected(object(), frame)


def test_conversion_rejects_wrong_frame() -> None:
    with pytest.raises(RouteRecordError, match="LocalCartesianFrame"):
        local_to_projected(CoverageWaypoint(0.0, 0.0, 0.0), object())


def test_geographic_to_projected_rejects_wrong_waypoint(frame) -> None:
    with pytest.raises(RouteRecordError, match="GeographicWaypoint"):
        geographic_to_projected(object(), frame)


def test_record_from_result_copies_metadata(record, result) -> None:
    assert record.request_id == result.request_id
    assert record.component_id == result.component_id
    assert record.source_region_id == result.source_region_id
    assert record.assigned_vehicle_id == result.assigned_vehicle_id
    assert record.response_message == result.response_message
    assert record.waypoints == result.waypoints


def test_record_from_result_rejects_wrong_result(frame) -> None:
    with pytest.raises(RouteRecordError, match="CoveragePlanningResult"):
        CoverageRouteRecord.from_result(object(), frame)


def test_record_from_result_rejects_frame_mismatch(result) -> None:
    other = LocalCartesianFrame("other", "EPSG:32643", 0.0, 0.0)
    with pytest.raises(RouteRecordError, match="frame mismatch"):
        CoverageRouteRecord.from_result(result, other)


def test_record_rejects_unsafe_identifier(frame) -> None:
    with pytest.raises(RouteRecordError, match="request_id"):
        CoverageRouteRecord(
            "../request",
            "component-1",
            "region-1",
            None,
            frame,
            "ok",
            (CoverageWaypoint(0.0, 0.0, 1.0),),
        )


def test_record_rejects_unsafe_optional_vehicle(frame) -> None:
    with pytest.raises(RouteRecordError, match="assigned_vehicle_id"):
        CoverageRouteRecord(
            "request-1",
            "component-1",
            "region-1",
            "drone one",
            frame,
            "ok",
            (CoverageWaypoint(0.0, 0.0, 1.0),),
        )


def test_record_allows_empty_response_message(result, frame) -> None:
    empty = CoveragePlanningResult(
        result.request_id,
        result.component_id,
        result.source_region_id,
        result.assigned_vehicle_id,
        result.frame_id,
        "",
        result.waypoints,
    )
    assert CoverageRouteRecord.from_result(empty, frame).response_message == ""


def test_record_rejects_empty_waypoints(frame) -> None:
    with pytest.raises(RouteRecordError, match="must not be empty"):
        CoverageRouteRecord(
            "request-1", "component-1", "region-1", None, frame, "ok", ()
        )


def test_record_rejects_wrong_waypoint_members(frame) -> None:
    with pytest.raises(RouteRecordError, match="CoverageWaypoint"):
        CoverageRouteRecord(
            "request-1",
            "component-1",
            "region-1",
            None,
            frame,
            "ok",
            (object(),),
        )


def test_to_dict_stores_only_local_authoritative_coordinates(record) -> None:
    data = record.to_dict()
    assert data["schema_version"] == ROUTE_SCHEMA_VERSION
    assert "waypoints_local_m" in data
    assert "waypoints_projected_m" not in data
    assert "waypoints_geographic" not in data
    assert data["frame"]["projected_crs"] == "EPSG:32643"


def test_json_is_deterministic(record) -> None:
    first = record.to_json()
    second = record.to_json()
    assert first == second
    assert first.endswith("\n")
    assert json.loads(first)["request_id"] == "request-1"


def test_json_round_trip(record) -> None:
    recovered = CoverageRouteRecord.from_json(record.to_json())
    assert recovered == record


def test_dict_rejects_unknown_root_field(record) -> None:
    data = record.to_dict()
    data["surprise"] = 1
    with pytest.raises(RouteRecordError, match="unknown"):
        CoverageRouteRecord.from_dict(data)


def test_dict_rejects_missing_root_field(record) -> None:
    data = record.to_dict()
    del data["frame"]
    with pytest.raises(RouteRecordError, match="missing"):
        CoverageRouteRecord.from_dict(data)


def test_dict_rejects_wrong_schema(record) -> None:
    data = record.to_dict()
    data["schema_version"] = 999
    with pytest.raises(RouteRecordError, match="unsupported schema_version"):
        CoverageRouteRecord.from_dict(data)


def test_dict_rejects_waypoint_unknown_field(record) -> None:
    data = record.to_dict()
    data["waypoints_local_m"][0]["yaw"] = 0.0
    with pytest.raises(RouteRecordError, match="unknown"):
        CoverageRouteRecord.from_dict(data)


def test_dict_rejects_nonarray_waypoints(record) -> None:
    data = record.to_dict()
    data["waypoints_local_m"] = {}
    with pytest.raises(RouteRecordError, match="array"):
        CoverageRouteRecord.from_dict(data)


def test_dict_wraps_invalid_waypoint_error(record) -> None:
    data = record.to_dict()
    data["waypoints_local_m"][1]["x_m"] = "bad"
    with pytest.raises(RouteRecordError, match=r"waypoints_local_m\[1\]"):
        CoverageRouteRecord.from_dict(data)


def test_dict_wraps_invalid_frame_error(record) -> None:
    data = record.to_dict()
    data["frame"]["projected_crs"] = "EPSG:4326"
    with pytest.raises(RouteRecordError, match="frame is invalid"):
        CoverageRouteRecord.from_dict(data)


def test_from_json_rejects_malformed_text() -> None:
    with pytest.raises(RouteRecordError, match="invalid JSON"):
        CoverageRouteRecord.from_json("{")


def test_from_json_rejects_nontext() -> None:
    with pytest.raises(RouteRecordError, match="must be text"):
        CoverageRouteRecord.from_json(123)


def test_write_and_read_round_trip(record, tmp_path) -> None:
    destination = record.write(tmp_path)
    assert destination == tmp_path / "request-1.route.json"
    assert CoverageRouteRecord.read(destination) == record
    assert not list(tmp_path.glob("*.tmp"))


def test_filename_is_safe_and_deterministic(record) -> None:
    assert record.filename == "request-1.route.json"


def test_to_result_round_trip(record, result) -> None:
    assert record.to_result() == result


def test_projected_waypoints_preserve_order(record) -> None:
    projected = record.projected_waypoints()
    assert [point.easting_m for point in projected] == pytest.approx(
        [631285.61, 631295.61, 631283.11]
    )
    assert [point.altitude_m for point in projected] == [30.0, 30.0, 30.0]


def test_geographic_waypoints_preserve_order(record) -> None:
    geographic = record.geographic_waypoints()
    assert len(geographic) == 3
    projected_again = [
        geographic_to_projected(point, record.frame) for point in geographic
    ]
    expected = record.projected_waypoints()
    for actual, target in zip(projected_again, expected):
        assert actual.easting_m == pytest.approx(target.easting_m, abs=1.0e-5)
        assert actual.northing_m == pytest.approx(target.northing_m, abs=1.0e-5)


def test_make_route_records_preserves_every_result(result, frame) -> None:
    second = CoveragePlanningResult(
        "request-2",
        "partition-2_component_1",
        "partition-2",
        "drone-2",
        "map",
        "planned",
        (CoverageWaypoint(4.0, 5.0, 30.0),),
    )
    records = make_route_records(
        [result, second],
        {
            result.component_id: frame,
            second.component_id: frame,
        },
    )
    assert [item.request_id for item in records] == ["request-1", "request-2"]


def test_make_route_records_rejects_missing_frame(result) -> None:
    with pytest.raises(RouteRecordError, match="missing frame"):
        make_route_records([result], {})


def test_make_route_records_rejects_duplicate_component(result, frame) -> None:
    duplicate = CoveragePlanningResult(
        "request-2",
        result.component_id,
        result.source_region_id,
        result.assigned_vehicle_id,
        result.frame_id,
        result.response_message,
        result.waypoints,
    )
    with pytest.raises(RouteRecordError, match="duplicate component_id"):
        make_route_records(
            [result, duplicate],
            {result.component_id: frame},
        )


def test_make_route_records_rejects_empty_batch() -> None:
    with pytest.raises(RouteRecordError, match="must not be empty"):
        make_route_records([], {})


def test_serialized_copy_is_independent(record) -> None:
    data = deepcopy(record.to_dict())
    recovered = CoverageRouteRecord.from_dict(data)
    data["waypoints_local_m"][0]["x_m"] = 999.0
    assert recovered.waypoints[0].x_m == 0.0
