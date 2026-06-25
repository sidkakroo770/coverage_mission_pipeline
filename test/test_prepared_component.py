#!/usr/bin/env python3
"""Tests for the stable prepared-component contract."""

import json

import pytest
from shapely.geometry import MultiPolygon, Polygon, box

from coverage_mission_pipeline.prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
    PreparedComponentError,
    make_prepared_components,
)


@pytest.fixture
def frame() -> LocalCartesianFrame:
    return LocalCartesianFrame(
        frame_id="map",
        projected_crs="epsg:32643",
        origin_easting_m=631285.61,
        origin_northing_m=3358862.37,
    )


def make_component(frame: LocalCartesianFrame) -> PreparedComponent:
    return PreparedComponent(
        component_id="partition-3_component_2",
        source_region_id="partition-3",
        component_index=2,
        assigned_vehicle_id="drone-3",
        frame=frame,
        polygon=box(10.0, 20.0, 50.0, 80.0),
    )


def test_frame_normalizes_projected_crs() -> None:
    result = LocalCartesianFrame("map", "epsg:32643", 1.0, 2.0)
    assert result.projected_crs == "EPSG:32643"


def test_component_round_trip_without_holes(
    frame: LocalCartesianFrame,
) -> None:
    original = make_component(frame)
    restored = PreparedComponent.from_json(original.to_json())

    assert restored.component_id == original.component_id
    assert restored.source_region_id == original.source_region_id
    assert restored.component_index == 2
    assert restored.assigned_vehicle_id == "drone-3"
    assert restored.frame == original.frame
    assert restored.polygon.equals(original.polygon)


def test_component_round_trip_with_hole(
    frame: LocalCartesianFrame,
) -> None:
    polygon = Polygon(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)],
        [[(5.0, 5.0), (5.0, 10.0), (10.0, 10.0), (10.0, 5.0)]],
    )
    original = PreparedComponent(
        "region_component_1",
        "region",
        1,
        frame,
        polygon,
    )

    restored = PreparedComponent.from_dict(original.to_dict())

    assert restored.polygon.equals(polygon)
    assert len(restored.polygon.interiors) == 1


def test_output_uses_unclosed_canonical_rings(
    frame: LocalCartesianFrame,
) -> None:
    value = make_component(frame).to_dict()
    hull = value["polygon"]["hull"]

    assert len(hull) == 4
    assert hull[0] != hull[-1]


def test_clockwise_input_is_normalized_counterclockwise(
    frame: LocalCartesianFrame,
) -> None:
    clockwise = Polygon(
        [(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]
    )
    component = PreparedComponent(
        "region_component_1", "region", 1, frame, clockwise
    )

    assert component.polygon.exterior.is_ccw


def test_serialization_is_deterministic(
    frame: LocalCartesianFrame,
) -> None:
    component = make_component(frame)
    assert component.to_json() == component.to_json()
    assert PreparedComponent.from_json(component.to_json()).to_json() == (
        component.to_json()
    )


def test_atomic_file_round_trip(
    frame: LocalCartesianFrame,
    tmp_path,
) -> None:
    component = make_component(frame)
    destination = tmp_path / component.filename

    returned = component.write(destination)
    restored = PreparedComponent.read(destination)

    assert returned == destination
    assert destination.exists()
    assert not (tmp_path / f".{component.filename}.tmp").exists()
    assert restored.to_json() == component.to_json()


def test_builder_creates_one_record_per_polygon(
    frame: LocalCartesianFrame,
) -> None:
    records = make_prepared_components(
        [box(0.0, 0.0, 1.0, 1.0), box(2.0, 0.0, 3.0, 1.0)],
        source_region_id="partition-7",
        assigned_vehicle_id="drone-7",
        frame=frame,
    )

    assert [record.component_id for record in records] == [
        "partition-7_component_1",
        "partition-7_component_2",
    ]
    assert [record.component_index for record in records] == [1, 2]
    assert all(record.assigned_vehicle_id == "drone-7" for record in records)


def test_builder_allows_unassigned_components(
    frame: LocalCartesianFrame,
) -> None:
    records = make_prepared_components(
        [box(0.0, 0.0, 1.0, 1.0)],
        source_region_id="partition-1",
        frame=frame,
    )
    assert records[0].assigned_vehicle_id is None


def test_multipolygon_is_rejected(frame: LocalCartesianFrame) -> None:
    geometry = MultiPolygon(
        [box(0.0, 0.0, 1.0, 1.0), box(2.0, 0.0, 3.0, 1.0)]
    )
    with pytest.raises(
        PreparedComponentError,
        match="must be exactly one Polygon",
    ):
        PreparedComponent(
            "region_component_1", "region", 1, frame, geometry
        )


def test_invalid_polygon_is_rejected(frame: LocalCartesianFrame) -> None:
    bow_tie = Polygon(
        [(0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)]
    )
    with pytest.raises(PreparedComponentError, match="polygon is invalid"):
        PreparedComponent(
            "region_component_1", "region", 1, frame, bow_tie
        )


@pytest.mark.parametrize(
    "identifier",
    ["", "has space", "../escape", "/absolute", "region/component"],
)
def test_unsafe_identifier_is_rejected(
    identifier: str,
    frame: LocalCartesianFrame,
) -> None:
    with pytest.raises(PreparedComponentError):
        PreparedComponent(
            identifier,
            "region",
            1,
            frame,
            box(0.0, 0.0, 1.0, 1.0),
        )


def test_nonfinite_origin_is_rejected() -> None:
    with pytest.raises(PreparedComponentError, match="must be finite"):
        LocalCartesianFrame("map", "EPSG:32643", float("nan"), 0.0)


def test_geographic_crs_is_rejected() -> None:
    with pytest.raises(PreparedComponentError, match="projected CRS"):
        LocalCartesianFrame("map", "EPSG:4326", 0.0, 0.0)


def test_unknown_root_field_is_rejected(
    frame: LocalCartesianFrame,
) -> None:
    value = make_component(frame).to_dict()
    value["typo"] = True

    with pytest.raises(PreparedComponentError, match="unknown field"):
        PreparedComponent.from_dict(value)


def test_unsupported_schema_version_is_rejected(
    frame: LocalCartesianFrame,
) -> None:
    value = make_component(frame).to_dict()
    value["schema_version"] = 999

    with pytest.raises(PreparedComponentError, match="unsupported"):
        PreparedComponent.from_dict(value)


def test_wrong_axis_order_is_rejected(
    frame: LocalCartesianFrame,
) -> None:
    value = make_component(frame).to_dict()
    value["frame"]["axis_order"] = ["y_north", "x_east"]

    with pytest.raises(PreparedComponentError, match="axis_order"):
        PreparedComponent.from_dict(value)


def test_malformed_ring_is_rejected(
    frame: LocalCartesianFrame,
) -> None:
    value = make_component(frame).to_dict()
    value["polygon"]["hull"] = [[0.0, 0.0], [1.0, 0.0]]

    with pytest.raises(PreparedComponentError, match="three distinct"):
        PreparedComponent.from_dict(value)


def test_closed_input_ring_is_accepted_and_canonicalized(
    frame: LocalCartesianFrame,
) -> None:
    value = make_component(frame).to_dict()
    value["polygon"]["hull"].append(value["polygon"]["hull"][0])

    restored = PreparedComponent.from_dict(value)
    output_hull = restored.to_dict()["polygon"]["hull"]

    assert output_hull[0] != output_hull[-1]
    assert len(output_hull) == 4


def test_invalid_json_reports_location() -> None:
    with pytest.raises(PreparedComponentError, match="line 1, column"):
        PreparedComponent.from_json('{"schema_version":')


def test_json_does_not_emit_nan(frame: LocalCartesianFrame) -> None:
    component = make_component(frame)
    parsed = json.loads(component.to_json())
    assert parsed["frame"]["origin_projected_m"]["easting"] == 631285.61
