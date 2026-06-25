#!/usr/bin/env python3
"""Synthetic tests for mission_geometry_core."""

import pytest
from shapely.geometry import (
    GeometryCollection,
    LineString,
    MultiPolygon,
    Point,
    Polygon,
    box,
)
from shapely.ops import unary_union

from coverage_mission_pipeline.mission_geometry_core import (
    GeometryCoreError,
    clip_partition_to_safe_area,
    create_safe_area,
    extract_polygon_components,
    prepare_partition_components,
)


def assert_area(actual: float, expected: float) -> None:
    assert actual == pytest.approx(expected, abs=1.0e-9)


def test_rectangle_without_exclusions_returns_one_component() -> None:
    boundary = box(0.0, 0.0, 10.0, 10.0)

    components = prepare_partition_components(
        boundary=boundary,
        partition=boundary,
        exclusions=[],
        clearance_m=0.0,
    )

    assert len(components) == 1
    assert_area(components[0].area, 100.0)
    assert len(components[0].interiors) == 0


def test_central_exclusion_becomes_one_hole() -> None:
    boundary = box(0.0, 0.0, 10.0, 10.0)
    exclusion = box(4.0, 4.0, 6.0, 6.0)

    components = prepare_partition_components(
        boundary=boundary,
        partition=boundary,
        exclusions=[exclusion],
        clearance_m=0.0,
    )

    assert len(components) == 1
    assert len(components[0].interiors) == 1
    assert_area(components[0].area, 96.0)


def test_crossing_exclusion_splits_area_into_two_components() -> None:
    boundary = box(0.0, 0.0, 10.0, 10.0)
    exclusion = box(4.0, -1.0, 6.0, 11.0)

    components = prepare_partition_components(
        boundary=boundary,
        partition=boundary,
        exclusions=[exclusion],
        clearance_m=0.0,
    )

    assert len(components) == 2
    assert [component.area for component in components] == pytest.approx(
        [40.0, 40.0]
    )


def test_two_crossing_exclusions_create_three_components() -> None:
    boundary = box(0.0, 0.0, 14.0, 10.0)
    exclusions = [
        box(3.0, -1.0, 4.0, 11.0),
        box(9.0, -1.0, 10.0, 11.0),
    ]

    components = prepare_partition_components(
        boundary=boundary,
        partition=boundary,
        exclusions=exclusions,
        clearance_m=0.0,
    )

    assert len(components) == 3
    assert [component.area for component in components] == pytest.approx(
        [30.0, 50.0, 40.0]
    )


def test_exclusion_outside_boundary_changes_nothing() -> None:
    boundary = box(0.0, 0.0, 10.0, 10.0)
    exclusion = box(20.0, 20.0, 30.0, 30.0)

    components = prepare_partition_components(
        boundary=boundary,
        partition=boundary,
        exclusions=[exclusion],
        clearance_m=0.0,
    )

    assert len(components) == 1
    assert components[0].equals(boundary)


def test_one_exclusion_is_removed_from_every_affected_partition() -> None:
    boundary = box(0.0, 0.0, 20.0, 10.0)
    left_partition = box(0.0, 0.0, 10.0, 10.0)
    right_partition = box(10.0, 0.0, 20.0, 10.0)
    exclusion = box(8.0, 2.0, 12.0, 8.0)

    safe_area = create_safe_area(
        boundary=boundary,
        exclusions=[exclusion],
        clearance_m=0.0,
    )
    left = clip_partition_to_safe_area(left_partition, safe_area)
    right = clip_partition_to_safe_area(right_partition, safe_area)

    assert len(left) == 1
    assert len(right) == 1
    assert_area(left[0].area, 88.0)
    assert_area(right[0].area, 88.0)
    assert not left[0].intersects(exclusion.buffer(-1.0e-9))
    assert not right[0].intersects(exclusion.buffer(-1.0e-9))


def test_clearance_changes_outer_boundary_not_shared_partition_border() -> None:
    boundary = box(0.0, 0.0, 20.0, 10.0)
    left_partition = box(0.0, 0.0, 10.0, 10.0)
    right_partition = box(10.0, 0.0, 20.0, 10.0)

    safe_area = create_safe_area(
        boundary=boundary,
        exclusions=[],
        clearance_m=1.0,
    )
    left = clip_partition_to_safe_area(left_partition, safe_area)
    right = clip_partition_to_safe_area(right_partition, safe_area)

    assert len(left) == 1
    assert len(right) == 1
    assert left[0].bounds == pytest.approx((1.0, 1.0, 10.0, 9.0))
    assert right[0].bounds == pytest.approx((10.0, 1.0, 19.0, 9.0))

    reconstructed = unary_union(left + right)
    assert reconstructed.symmetric_difference(safe_area).area < 1.0e-9


def test_clearance_expands_exclusion_and_shrinks_boundary() -> None:
    boundary = box(0.0, 0.0, 20.0, 20.0)
    exclusion = box(8.0, 8.0, 12.0, 12.0)

    safe_area = create_safe_area(
        boundary=boundary,
        exclusions=[exclusion],
        clearance_m=1.0,
    )

    assert safe_area.bounds == pytest.approx((1.0, 1.0, 19.0, 19.0))
    components = extract_polygon_components(safe_area)
    assert len(components) == 1
    assert len(components[0].interiors) == 1

    # Inset boundary: 18 x 18. Expanded square exclusion: 6 x 6.
    assert_area(components[0].area, (18.0 * 18.0) - (6.0 * 6.0))


def test_partition_completely_removed_returns_empty_list() -> None:
    boundary = box(0.0, 0.0, 10.0, 10.0)
    partition = box(0.0, 0.0, 4.0, 4.0)
    exclusion = box(-1.0, -1.0, 5.0, 5.0)

    safe_area = create_safe_area(
        boundary=boundary,
        exclusions=[exclusion],
        clearance_m=0.0,
    )
    components = clip_partition_to_safe_area(partition, safe_area)

    assert components == []


def test_geometry_collection_preserves_every_polygon() -> None:
    first = box(0.0, 0.0, 1.0, 1.0)
    second = box(2.0, 0.0, 4.0, 1.0)
    geometry = GeometryCollection(
        [
            first,
            LineString([(0.0, 0.0), (3.0, 3.0)]),
            Point(100.0, 100.0),
            MultiPolygon([second]),
        ]
    )

    components = extract_polygon_components(geometry)

    assert len(components) == 2
    assert [component.area for component in components] == pytest.approx(
        [1.0, 2.0]
    )


def test_minimum_component_area_filters_only_when_explicitly_requested() -> None:
    large = box(0.0, 0.0, 10.0, 10.0)
    tiny = box(20.0, 20.0, 20.1, 20.1)
    geometry = MultiPolygon([large, tiny])

    all_components = extract_polygon_components(geometry)
    filtered = extract_polygon_components(
        geometry,
        min_area_m2=0.1,
    )

    assert len(all_components) == 2
    assert len(filtered) == 1
    assert filtered[0].equals(large)


@pytest.mark.parametrize("clearance", [-1.0, float("inf"), float("nan")])
def test_invalid_clearance_is_rejected(clearance: float) -> None:
    with pytest.raises(GeometryCoreError):
        create_safe_area(
            boundary=box(0.0, 0.0, 10.0, 10.0),
            exclusions=[],
            clearance_m=clearance,
        )


def test_clearance_that_erases_boundary_is_rejected() -> None:
    with pytest.raises(
        GeometryCoreError,
        match="boundary is empty after applying inward clearance",
    ):
        create_safe_area(
            boundary=box(0.0, 0.0, 10.0, 10.0),
            exclusions=[],
            clearance_m=6.0,
        )


def test_invalid_bow_tie_polygon_is_rejected() -> None:
    bow_tie = Polygon(
        [
            (0.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (2.0, 0.0),
        ]
    )

    with pytest.raises(GeometryCoreError, match="boundary is invalid"):
        create_safe_area(
            boundary=bow_tie,
            exclusions=[],
            clearance_m=0.0,
        )
