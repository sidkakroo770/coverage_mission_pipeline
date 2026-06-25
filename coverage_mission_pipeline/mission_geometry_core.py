#!/usr/bin/env python3
"""Generic geometry preparation for polygon coverage missions.

This module is intentionally independent of:
- mission_output.json and any input schema;
- ROS messages and service calls;
- drone identifiers and assignment policy;
- coordinate projection.

All geometry passed to this module must already use a Cartesian metric frame.
"""

from __future__ import annotations

import math
from typing import Iterable, List

from shapely.geometry import GeometryCollection, MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.geometry.polygon import orient
from shapely.ops import unary_union
from shapely.validation import explain_validity


class GeometryCoreError(ValueError):
    """Raised when geometry preparation cannot safely continue."""


def _validate_clearance(clearance_m: float) -> float:
    """Return a validated non-negative clearance in metres."""
    if isinstance(clearance_m, bool) or not isinstance(clearance_m, (int, float)):
        raise GeometryCoreError("clearance_m must be a number")

    value = float(clearance_m)
    if not math.isfinite(value) or value < 0.0:
        raise GeometryCoreError(
            "clearance_m must be finite and greater than or equal to zero"
        )
    return value


def _validate_polygonal(
    geometry: BaseGeometry,
    name: str,
    *,
    allow_empty: bool = False,
) -> None:
    """Validate a Polygon/MultiPolygon/GeometryCollection input."""
    if not isinstance(geometry, BaseGeometry):
        raise GeometryCoreError(f"{name} must be a Shapely geometry")

    if geometry.is_empty:
        if allow_empty:
            return
        raise GeometryCoreError(f"{name} must not be empty")

    if not geometry.is_valid:
        raise GeometryCoreError(
            f"{name} is invalid: {explain_validity(geometry)}"
        )

    if geometry.geom_type not in {
        "Polygon",
        "MultiPolygon",
        "GeometryCollection",
    }:
        raise GeometryCoreError(
            f"{name} must be polygonal, got {geometry.geom_type}"
        )


def extract_polygon_components(
    geometry: BaseGeometry,
    *,
    min_area_m2: float = 0.0,
) -> List[Polygon]:
    """Extract every Polygon from polygonal or mixed geometry.

    No component is silently reduced to the largest polygon. Components are
    sorted deterministically by centroid, then by descending area.
    """
    _validate_polygonal(geometry, "geometry", allow_empty=True)

    if isinstance(min_area_m2, bool) or not isinstance(
        min_area_m2, (int, float)
    ):
        raise GeometryCoreError("min_area_m2 must be a number")

    minimum_area = float(min_area_m2)
    if not math.isfinite(minimum_area) or minimum_area < 0.0:
        raise GeometryCoreError(
            "min_area_m2 must be finite and greater than or equal to zero"
        )

    polygons: List[Polygon] = []

    def collect(item: BaseGeometry) -> None:
        if item.is_empty:
            return

        if isinstance(item, Polygon):
            if item.area > minimum_area:
                polygons.append(orient(item, sign=1.0))
            return

        if isinstance(item, MultiPolygon):
            for child in item.geoms:
                collect(child)
            return

        if isinstance(item, GeometryCollection):
            for child in item.geoms:
                if child.geom_type in {
                    "Polygon",
                    "MultiPolygon",
                    "GeometryCollection",
                }:
                    collect(child)
            return

        # Lines and points can legitimately appear in GeometryCollections
        # after overlay operations. They do not define plannable area.

    collect(geometry)

    polygons.sort(
        key=lambda polygon: (
            round(polygon.centroid.x, 12),
            round(polygon.centroid.y, 12),
            -polygon.area,
        )
    )
    return polygons


def create_safe_area(
    boundary: BaseGeometry,
    exclusions: Iterable[BaseGeometry],
    clearance_m: float,
) -> BaseGeometry:
    """Create global safe area from a boundary and exclusions.

    Clearance is applied only to:
    - the outer mission boundary, buffered inward;
    - exclusion boundaries, buffered outward.

    Drone partition boundaries are intentionally not buffered here.
    """
    _validate_polygonal(boundary, "boundary")
    clearance = _validate_clearance(clearance_m)

    exclusion_list = list(exclusions)
    for index, exclusion in enumerate(exclusion_list):
        _validate_polygonal(exclusion, f"exclusions[{index}]")

    safe_boundary = (
        boundary
        if clearance == 0.0
        else boundary.buffer(
            -clearance,
            resolution=16,
            join_style=2,
            mitre_limit=5.0,
        )
    )

    if safe_boundary.is_empty:
        raise GeometryCoreError(
            "boundary is empty after applying inward clearance"
        )

    if not safe_boundary.is_valid:
        raise GeometryCoreError(
            "boundary became invalid after clearance buffering: "
            f"{explain_validity(safe_boundary)}"
        )

    if exclusion_list:
        buffered_exclusions = [
            exclusion
            if clearance == 0.0
            else exclusion.buffer(
                clearance,
                resolution=16,
                join_style=2,
                mitre_limit=5.0,
            )
            for exclusion in exclusion_list
        ]
        safe_area = safe_boundary.difference(
            unary_union(buffered_exclusions)
        )
    else:
        safe_area = safe_boundary

    if safe_area.is_empty:
        raise GeometryCoreError(
            "no safe area remains after applying exclusions"
        )

    if not safe_area.is_valid:
        raise GeometryCoreError(
            "safe area is invalid after exclusion subtraction: "
            f"{explain_validity(safe_area)}"
        )

    return safe_area


def clip_partition_to_safe_area(
    partition: BaseGeometry,
    safe_area: BaseGeometry,
    *,
    min_component_area_m2: float = 0.0,
) -> List[Polygon]:
    """Intersect one unbuffered partition with the global safe area."""
    _validate_polygonal(partition, "partition")
    _validate_polygonal(safe_area, "safe_area")

    clipped = partition.intersection(safe_area)

    if not clipped.is_empty and not clipped.is_valid:
        raise GeometryCoreError(
            "partition clipping produced invalid geometry: "
            f"{explain_validity(clipped)}"
        )

    return extract_polygon_components(
        clipped,
        min_area_m2=min_component_area_m2,
    )


def prepare_partition_components(
    boundary: BaseGeometry,
    partition: BaseGeometry,
    exclusions: Iterable[BaseGeometry],
    clearance_m: float,
    *,
    min_component_area_m2: float = 0.0,
) -> List[Polygon]:
    """Convenience wrapper for one partition."""
    safe_area = create_safe_area(
        boundary=boundary,
        exclusions=exclusions,
        clearance_m=clearance_m,
    )
    return clip_partition_to_safe_area(
        partition=partition,
        safe_area=safe_area,
        min_component_area_m2=min_component_area_m2,
    )
