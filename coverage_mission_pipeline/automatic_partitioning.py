#!/usr/bin/env python3
"""Deterministic equal-route-area partitioning for five-drone coverage."""

from __future__ import annotations

from dataclasses import dataclass
import math

from shapely import affinity
from shapely.geometry import box, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from .mission_geometry_core import extract_polygon_components


class AutomaticPartitioningError(ValueError):
    """Raised when safe automatic partition construction is impossible."""


@dataclass(frozen=True)
class AutomaticPartitioningResult:
    partitions_projected: tuple[BaseGeometry, ...]
    rotation_degrees: float
    route_area_by_partition_m2: tuple[float, ...]


def _principal_angle_degrees(geometry: BaseGeometry) -> float:
    rectangle = geometry.minimum_rotated_rectangle
    coordinates = list(rectangle.exterior.coords)
    edges: list[tuple[float, float, float]] = []
    for start, end in zip(coordinates, coordinates[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        edges.append((math.hypot(dx, dy), dx, dy))
    _, dx, dy = max(edges, key=lambda item: item[0])
    return math.degrees(math.atan2(dy, dx))


def _left_area(geometry: BaseGeometry, x: float, padding: float) -> float:
    minx, miny, _, maxy = geometry.bounds
    return geometry.intersection(
        box(minx - padding, miny - padding, x, maxy + padding)
    ).area


def _cut_for_target(
    geometry: BaseGeometry,
    target_area: float,
    padding: float,
) -> float:
    minx, _, maxx, _ = geometry.bounds
    low = minx
    high = maxx
    for _ in range(80):
        midpoint = (low + high) / 2.0
        if _left_area(geometry, midpoint, padding) < target_area:
            low = midpoint
        else:
            high = midpoint
    return (low + high) / 2.0


def create_equal_route_area_partitions(
    boundary_projected: Polygon,
    route_space_projected: BaseGeometry,
    *,
    partition_count: int = 5,
    relative_area_tolerance: float = 1.0e-6,
) -> AutomaticPartitioningResult:
    """Partition the full boundary using slabs balanced on usable route area.

    Cuts are selected in a principal-axis-aligned frame using the operational
    route-space area. The resulting partition polygons are intersections of the
    full mission boundary with those slabs, so their union still covers the
    original boundary required by the strict adapter contract.
    """
    if partition_count < 1:
        raise AutomaticPartitioningError("partition_count must be positive")
    if boundary_projected.is_empty or not boundary_projected.is_valid:
        raise AutomaticPartitioningError("boundary_projected must be valid and non-empty")
    if route_space_projected.is_empty or not route_space_projected.is_valid:
        raise AutomaticPartitioningError("route_space_projected must be valid and non-empty")
    if not boundary_projected.covers(route_space_projected):
        raise AutomaticPartitioningError("boundary does not cover route space")

    angle = _principal_angle_degrees(route_space_projected)
    origin = boundary_projected.centroid
    rotated_boundary = affinity.rotate(
        boundary_projected,
        -angle,
        origin=(origin.x, origin.y),
        use_radians=False,
    )
    rotated_route_space = affinity.rotate(
        route_space_projected,
        -angle,
        origin=(origin.x, origin.y),
        use_radians=False,
    )

    minx, miny, maxx, maxy = rotated_boundary.bounds
    span = max(maxx - minx, maxy - miny, 1.0)
    padding = span * 2.0 + 1000.0
    total_route_area = rotated_route_space.area
    if total_route_area <= 0.0:
        raise AutomaticPartitioningError("route space has zero area")

    cuts = [minx - padding]
    for index in range(1, partition_count):
        cuts.append(
            _cut_for_target(
                rotated_route_space,
                total_route_area * index / partition_count,
                padding,
            )
        )
    cuts.append(maxx + padding)

    rotated_partitions: list[BaseGeometry] = []
    route_areas: list[float] = []
    for index in range(partition_count):
        slab = box(
            cuts[index],
            miny - padding,
            cuts[index + 1],
            maxy + padding,
        )
        partition = rotated_boundary.intersection(slab)
        if partition.is_empty or not partition.is_valid:
            raise AutomaticPartitioningError(
                f"partition {index + 1} is empty or invalid"
            )
        if not extract_polygon_components(partition):
            raise AutomaticPartitioningError(
                f"partition {index + 1} has no polygon components"
            )
        route_area = rotated_route_space.intersection(slab).area
        if route_area <= 0.0:
            raise AutomaticPartitioningError(
                f"partition {index + 1} has no operational route area"
            )
        rotated_partitions.append(partition)
        route_areas.append(route_area)

    union = unary_union(rotated_partitions)
    coverage_gap = rotated_boundary.difference(union).area
    extra_area = union.difference(rotated_boundary).area
    if coverage_gap > max(1.0e-6, rotated_boundary.area * 1.0e-12):
        raise AutomaticPartitioningError(
            f"automatic partitions leave {coverage_gap:.9f} m^2 uncovered"
        )
    if extra_area > max(1.0e-6, rotated_boundary.area * 1.0e-12):
        raise AutomaticPartitioningError(
            f"automatic partitions extend {extra_area:.9f} m^2 outside boundary"
        )

    for left_index, left in enumerate(rotated_partitions):
        for right in rotated_partitions[left_index + 1:]:
            if left.intersection(right).area > 1.0e-6:
                raise AutomaticPartitioningError("automatic partitions overlap by area")

    target = total_route_area / partition_count
    for index, area in enumerate(route_areas, start=1):
        relative_error = abs(area - target) / target
        if relative_error > relative_area_tolerance:
            raise AutomaticPartitioningError(
                f"partition {index} route-area error {relative_error:.3e} exceeds tolerance"
            )

    partitions = tuple(
        affinity.rotate(
            partition,
            angle,
            origin=(origin.x, origin.y),
            use_radians=False,
        )
        for partition in rotated_partitions
    )

    return AutomaticPartitioningResult(
        partitions_projected=partitions,
        rotation_degrees=angle,
        route_area_by_partition_m2=tuple(route_areas),
    )
