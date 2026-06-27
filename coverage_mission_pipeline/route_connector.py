#!/usr/bin/env python3
"""Flight-safe connector paths through polygonal free space.

The connector planner uses a direct segment whenever the complete segment is
covered by one connected free-space polygon.  When direct visibility is
blocked, it builds an exact polygon-vertex visibility graph and runs A* with a
Euclidean heuristic.  For one simple exclusion this naturally compares the two
boundary directions; for several exclusions it can route through the globally
shortest visible sequence without introducing a raster-resolution parameter.

The supplied free-space geometry is authoritative and must already include all
mission-boundary and exclusion clearances.  This module never buffers it again.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Any, Iterable, Optional

from shapely.geometry import (
    GeometryCollection,
    LineString,
    MultiPolygon,
    Point,
    Polygon,
)
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points, unary_union
from shapely.validation import explain_validity

from .planning_request import LocalPoint2D
from .planning_result import CoverageWaypoint
from .prepared_component import LocalCartesianFrame
from .route_record import CoverageRouteRecord

DIRECT_CONNECTOR_ALGORITHM = "direct_segment_v1"
VISIBILITY_ASTAR_ALGORITHM = "visibility_graph_astar_v1"
TRIVIAL_CONNECTOR_ALGORITHM = "trivial_same_point_v1"

_LENGTH_TOLERANCE_M = 1.0e-8
_ALTITUDE_TOLERANCE_M = 1.0e-6
_EQUAL_COST_TOLERANCE_M = 1.0e-10
ROS_POINT32_ROUTE_REPAIR_TOLERANCE_M = 0.01
_POINT32_INWARD_NUDGE_M = 1.0e-5


class ConnectorPlanningError(ValueError):
    """Raised when no safe, deterministic connector can be produced."""


def _finite_positive_integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 2:
        raise ConnectorPlanningError(f"{name} must be an integer greater than one")
    return value


def _polygon_parts(geometry: BaseGeometry) -> list[Polygon]:
    if geometry.is_empty:
        return []
    if isinstance(geometry, Polygon):
        return [geometry]
    if isinstance(geometry, MultiPolygon):
        parts: list[Polygon] = []
        for child in geometry.geoms:
            parts.extend(_polygon_parts(child))
        return parts
    if isinstance(geometry, GeometryCollection):
        parts = []
        for child in geometry.geoms:
            parts.extend(_polygon_parts(child))
        return parts
    return []


def _canonical_free_space_parts(free_space: Any) -> tuple[Polygon, ...]:
    if not isinstance(free_space, BaseGeometry):
        raise ConnectorPlanningError("free_space must be a Shapely geometry")
    if free_space.is_empty:
        raise ConnectorPlanningError("free_space must not be empty")
    if not free_space.is_valid:
        raise ConnectorPlanningError(
            f"free_space is invalid: {explain_validity(free_space)}"
        )

    raw_parts = _polygon_parts(free_space)
    if not raw_parts:
        raise ConnectorPlanningError(
            "free_space must contain at least one positive-area polygon"
        )
    if any(part.is_empty or part.area <= 0.0 for part in raw_parts):
        raise ConnectorPlanningError(
            "free_space contains an empty or zero-area polygon"
        )

    # Merge touching polygonal pieces, while preserving every polygonal part.
    merged = unary_union(raw_parts)
    if merged.is_empty or not merged.is_valid:
        raise ConnectorPlanningError("free_space polygon union is invalid")
    parts = _polygon_parts(merged)
    if not parts:
        raise ConnectorPlanningError("free_space polygon union has no area")

    parts.sort(
        key=lambda part: (
            float(part.bounds[0]),
            float(part.bounds[1]),
            float(part.bounds[2]),
            float(part.bounds[3]),
            float(part.area),
        )
    )
    return tuple(parts)


def _segment(start: tuple[float, float], goal: tuple[float, float]) -> LineString:
    return LineString([start, goal])


def _distance(left: tuple[float, float], right: tuple[float, float]) -> float:
    return math.hypot(right[0] - left[0], right[1] - left[1])


def _path_length(points: Iterable[tuple[float, float]]) -> float:
    sequence = tuple(points)
    return sum(_distance(left, right) for left, right in zip(sequence, sequence[1:]))


def _ring_vertices(polygon: Polygon) -> tuple[tuple[float, float], ...]:
    vertices: set[tuple[float, float]] = set()
    rings = [polygon.exterior, *polygon.interiors]
    for ring in rings:
        coordinates = list(ring.coords)
        if len(coordinates) >= 2 and coordinates[0] == coordinates[-1]:
            coordinates.pop()
        for coordinate in coordinates:
            x = float(coordinate[0])
            y = float(coordinate[1])
            if not math.isfinite(x) or not math.isfinite(y):
                raise ConnectorPlanningError(
                    "free_space contains non-finite polygon coordinates"
                )
            vertices.add((x, y))
    return tuple(sorted(vertices))


def _common_component(
    free_space: Any,
    start: LocalPoint2D,
    goal: LocalPoint2D,
) -> Polygon:
    parts = _canonical_free_space_parts(free_space)
    start_point = Point(start.x_m, start.y_m)
    goal_point = Point(goal.x_m, goal.y_m)

    start_indices = [index for index, part in enumerate(parts) if part.covers(start_point)]
    goal_indices = [index for index, part in enumerate(parts) if part.covers(goal_point)]
    if not start_indices:
        raise ConnectorPlanningError("connector start is outside free_space")
    if not goal_indices:
        raise ConnectorPlanningError("connector goal is outside free_space")

    common = sorted(set(start_indices).intersection(goal_indices))
    if not common:
        raise ConnectorPlanningError(
            "connector start and goal are not in the same connected free-space "
            "component"
        )
    return parts[common[0]]


def _simplify_collinear(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    if len(points) <= 2:
        return points

    simplified: list[tuple[float, float]] = [points[0]]
    for current, following in zip(points[1:-1], points[2:]):
        previous = simplified[-1]
        first_x = current[0] - previous[0]
        first_y = current[1] - previous[1]
        second_x = following[0] - current[0]
        second_y = following[1] - current[1]
        cross = first_x * second_y - first_y * second_x
        scale = max(
            1.0,
            abs(first_x),
            abs(first_y),
            abs(second_x),
            abs(second_y),
        )
        dot = first_x * second_x + first_y * second_y
        if abs(cross) <= 1.0e-12 * scale * scale and dot >= 0.0:
            continue
        simplified.append(current)
    simplified.append(points[-1])
    return tuple(simplified)


@dataclass(frozen=True)
class ConnectorPlannerConfig:
    """Complexity guard for exact visibility-graph construction."""

    max_visibility_nodes: int = 512

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "max_visibility_nodes",
            _finite_positive_integer(
                self.max_visibility_nodes,
                "max_visibility_nodes",
            ),
        )


@dataclass(frozen=True)
class ConnectorPath:
    """One validated 2D path entirely covered by one free-space component."""

    start: LocalPoint2D
    goal: LocalPoint2D
    waypoints: tuple[LocalPoint2D, ...]
    length_m: float
    algorithm: str

    def __post_init__(self) -> None:
        if not isinstance(self.start, LocalPoint2D):
            raise ConnectorPlanningError("start must be a LocalPoint2D")
        if not isinstance(self.goal, LocalPoint2D):
            raise ConnectorPlanningError("goal must be a LocalPoint2D")
        try:
            points = tuple(self.waypoints)
        except TypeError as exc:
            raise ConnectorPlanningError("waypoints must be iterable") from exc
        if not points:
            raise ConnectorPlanningError("waypoints must not be empty")
        if any(not isinstance(point, LocalPoint2D) for point in points):
            raise ConnectorPlanningError(
                "waypoints must contain only LocalPoint2D objects"
            )
        if points[0] != self.start or points[-1] != self.goal:
            raise ConnectorPlanningError(
                "waypoints must begin at start and end at goal"
            )
        if self.start == self.goal:
            if len(points) != 1:
                raise ConnectorPlanningError(
                    "a same-point connector must contain exactly one waypoint"
                )
        elif len(points) < 2:
            raise ConnectorPlanningError(
                "a non-trivial connector requires at least two waypoints"
            )

        if self.algorithm not in {
            DIRECT_CONNECTOR_ALGORITHM,
            VISIBILITY_ASTAR_ALGORITHM,
            TRIVIAL_CONNECTOR_ALGORITHM,
        }:
            raise ConnectorPlanningError("unsupported connector algorithm")
        if isinstance(self.length_m, bool) or not isinstance(
            self.length_m, (int, float)
        ):
            raise ConnectorPlanningError("length_m must be a number")
        length = float(self.length_m)
        if not math.isfinite(length) or length < 0.0:
            raise ConnectorPlanningError("length_m must be finite and non-negative")
        actual = _path_length((point.x_m, point.y_m) for point in points)
        if not math.isclose(length, actual, rel_tol=0.0, abs_tol=_LENGTH_TOLERANCE_M):
            raise ConnectorPlanningError("length_m does not match waypoint geometry")
        object.__setattr__(self, "waypoints", points)
        object.__setattr__(self, "length_m", length)

    @property
    def is_direct(self) -> bool:
        return self.algorithm in {
            DIRECT_CONNECTOR_ALGORITHM,
            TRIVIAL_CONNECTOR_ALGORITHM,
        }

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "algorithm": self.algorithm,
            "length_m": self.length_m,
            "waypoint_count": len(self.waypoints),
            "start": self.start.to_dict(),
            "goal": self.goal.to_dict(),
            "waypoints": [point.to_dict() for point in self.waypoints],
        }


def _visibility_graph_path(
    component: Polygon,
    start: tuple[float, float],
    goal: tuple[float, float],
    config: ConnectorPlannerConfig,
) -> tuple[tuple[float, float], ...]:
    boundary_vertices = [
        vertex for vertex in _ring_vertices(component) if vertex not in {start, goal}
    ]
    nodes = (start, goal, *boundary_vertices)
    if len(nodes) > config.max_visibility_nodes:
        raise ConnectorPlanningError(
            "visibility graph requires "
            f"{len(nodes)} nodes, exceeding configured maximum "
            f"{config.max_visibility_nodes}"
        )

    adjacency: list[list[tuple[int, float]]] = [[] for _ in nodes]
    for left_index in range(len(nodes)):
        for right_index in range(left_index + 1, len(nodes)):
            left = nodes[left_index]
            right = nodes[right_index]
            distance = _distance(left, right)
            if distance == 0.0:
                continue
            if component.covers(_segment(left, right)):
                adjacency[left_index].append((right_index, distance))
                adjacency[right_index].append((left_index, distance))
    for neighbours in adjacency:
        neighbours.sort(key=lambda item: item[0])

    start_index = 0
    goal_index = 1
    initial_signature = (start_index,)
    queue: list[tuple[float, float, tuple[int, ...], int]] = [
        (
            _distance(start, goal),
            0.0,
            initial_signature,
            start_index,
        )
    ]
    best: dict[int, tuple[float, tuple[int, ...]]] = {
        start_index: (0.0, initial_signature)
    }

    while queue:
        _, cost, signature, node_index = heapq.heappop(queue)
        best_cost, best_signature = best[node_index]
        if cost > best_cost + _EQUAL_COST_TOLERANCE_M:
            continue
        if (
            abs(cost - best_cost) <= _EQUAL_COST_TOLERANCE_M
            and signature != best_signature
        ):
            continue
        if node_index == goal_index:
            return tuple(nodes[index] for index in signature)

        for neighbour_index, edge_length in adjacency[node_index]:
            new_cost = cost + edge_length
            new_signature = signature + (neighbour_index,)
            previous = best.get(neighbour_index)
            should_update = previous is None
            if previous is not None:
                previous_cost, previous_signature = previous
                if new_cost < previous_cost - _EQUAL_COST_TOLERANCE_M:
                    should_update = True
                elif (
                    abs(new_cost - previous_cost) <= _EQUAL_COST_TOLERANCE_M
                    and new_signature < previous_signature
                ):
                    should_update = True
            if not should_update:
                continue
            best[neighbour_index] = (new_cost, new_signature)
            heuristic = _distance(nodes[neighbour_index], goal)
            heapq.heappush(
                queue,
                (
                    new_cost + heuristic,
                    new_cost,
                    new_signature,
                    neighbour_index,
                ),
            )

    raise ConnectorPlanningError(
        "no visibility-graph path exists between connector start and goal"
    )


def plan_connector(
    free_space: BaseGeometry,
    start: LocalPoint2D,
    goal: LocalPoint2D,
    *,
    config: Optional[ConnectorPlannerConfig] = None,
) -> ConnectorPath:
    """Return the shortest visible polygonal connector inside free_space."""
    if not isinstance(start, LocalPoint2D):
        raise ConnectorPlanningError("start must be a LocalPoint2D")
    if not isinstance(goal, LocalPoint2D):
        raise ConnectorPlanningError("goal must be a LocalPoint2D")
    if config is not None and not isinstance(config, ConnectorPlannerConfig):
        raise ConnectorPlanningError("config must be a ConnectorPlannerConfig")
    policy = config or ConnectorPlannerConfig()
    component = _common_component(free_space, start, goal)

    start_xy = (start.x_m, start.y_m)
    goal_xy = (goal.x_m, goal.y_m)
    if start == goal:
        return ConnectorPath(
            start=start,
            goal=goal,
            waypoints=(start,),
            length_m=0.0,
            algorithm=TRIVIAL_CONNECTOR_ALGORITHM,
        )

    if component.covers(_segment(start_xy, goal_xy)):
        return ConnectorPath(
            start=start,
            goal=goal,
            waypoints=(start, goal),
            length_m=_distance(start_xy, goal_xy),
            algorithm=DIRECT_CONNECTOR_ALGORITHM,
        )

    raw_path = _visibility_graph_path(component, start_xy, goal_xy, policy)
    simplified = _simplify_collinear(raw_path)
    for left, right in zip(simplified, simplified[1:]):
        if not component.covers(_segment(left, right)):
            raise ConnectorPlanningError(
                "visibility path validation failed after simplification"
            )
    points = tuple(LocalPoint2D(x, y) for x, y in simplified)
    return ConnectorPath(
        start=start,
        goal=goal,
        waypoints=points,
        length_m=_path_length(simplified),
        algorithm=VISIBILITY_ASTAR_ALGORITHM,
    )


@dataclass(frozen=True)
class RouteConnector:
    """A 3D constant-altitude connector between two ordered route records."""

    from_request_id: str
    to_request_id: str
    path: ConnectorPath
    altitude_m: float

    def __post_init__(self) -> None:
        for name in ("from_request_id", "to_request_id"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value:
                raise ConnectorPlanningError(f"{name} must not be empty")
        if self.from_request_id == self.to_request_id:
            raise ConnectorPlanningError(
                "from_request_id and to_request_id must differ"
            )
        if not isinstance(self.path, ConnectorPath):
            raise ConnectorPlanningError("path must be a ConnectorPath")
        if isinstance(self.altitude_m, bool) or not isinstance(
            self.altitude_m, (int, float)
        ):
            raise ConnectorPlanningError("altitude_m must be a number")
        altitude = float(self.altitude_m)
        if not math.isfinite(altitude):
            raise ConnectorPlanningError("altitude_m must be finite")
        object.__setattr__(self, "altitude_m", altitude)

    @property
    def waypoints(self) -> tuple[CoverageWaypoint, ...]:
        return tuple(
            CoverageWaypoint(point.x_m, point.y_m, self.altitude_m)
            for point in self.path.waypoints
        )

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "from_request_id": self.from_request_id,
            "to_request_id": self.to_request_id,
            "altitude_m": self.altitude_m,
            "path": self.path.to_summary_dict(),
        }


@dataclass(frozen=True)
class ConnectedRouteSequence:
    """Continuous ordered route sequence for exactly one assigned vehicle."""

    vehicle_id: str
    frame: LocalCartesianFrame
    routes: tuple[CoverageRouteRecord, ...]
    connectors: tuple[RouteConnector, ...]
    waypoints: tuple[CoverageWaypoint, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.vehicle_id, str) or not self.vehicle_id:
            raise ConnectorPlanningError("vehicle_id must not be empty")
        if not isinstance(self.frame, LocalCartesianFrame):
            raise ConnectorPlanningError("frame must be a LocalCartesianFrame")
        routes = tuple(self.routes)
        connectors = tuple(self.connectors)
        waypoints = tuple(self.waypoints)
        if not routes:
            raise ConnectorPlanningError("routes must not be empty")
        if any(not isinstance(route, CoverageRouteRecord) for route in routes):
            raise ConnectorPlanningError(
                "routes must contain only CoverageRouteRecord objects"
            )
        if any(not isinstance(item, RouteConnector) for item in connectors):
            raise ConnectorPlanningError(
                "connectors must contain only RouteConnector objects"
            )
        if len(connectors) != len(routes) - 1:
            raise ConnectorPlanningError(
                "connector count must be exactly route count minus one"
            )
        if not waypoints or any(
            not isinstance(point, CoverageWaypoint) for point in waypoints
        ):
            raise ConnectorPlanningError(
                "waypoints must be a non-empty CoverageWaypoint sequence"
            )
        object.__setattr__(self, "routes", routes)
        object.__setattr__(self, "connectors", connectors)
        object.__setattr__(self, "waypoints", waypoints)

    @property
    def total_connector_length_m(self) -> float:
        return sum(connector.path.length_m for connector in self.connectors)

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "vehicle_id": self.vehicle_id,
            "frame": self.frame.to_dict(),
            "route_request_ids": [route.request_id for route in self.routes],
            "connector_count": len(self.connectors),
            "total_connector_length_m": self.total_connector_length_m,
            "waypoint_count": len(self.waypoints),
            "connectors": [item.to_summary_dict() for item in self.connectors],
        }


def _covering_part_indices(
    point: Point,
    free_space_parts: tuple[Polygon, ...],
) -> tuple[int, ...]:
    return tuple(
        index
        for index, part in enumerate(free_space_parts)
        if part.covers(point)
    )


def _nearest_part_point(
    point: Point,
    free_space_parts: tuple[Polygon, ...],
) -> tuple[float, int, Point]:
    candidates: list[tuple[float, int, Point]] = []
    for part_index, part in enumerate(free_space_parts):
        snapped = nearest_points(part, point)[0]
        candidates.append((float(point.distance(snapped)), part_index, snapped))
    return min(candidates, key=lambda item: (item[0], item[1]))


def _snap_point_inside_part(
    point: Point,
    part: Polygon,
    *,
    maximum_correction_m: float,
) -> Point:
    """Return a numerically stable point strictly inside ``part``.

    ``nearest_points(part, point)`` can lie a few ulps outside the polygon even
    though it represents a boundary projection.  Projecting onto a tiny inward
    buffer avoids feeding that ambiguous boundary coordinate back into GEOS.
    The total correction remains bounded by the public Point32 tolerance.
    """
    boundary_point = nearest_points(part, point)[0]
    boundary_distance = float(point.distance(boundary_point))
    remaining = maximum_correction_m - boundary_distance
    if remaining < 0.0:
        raise ConnectorPlanningError(
            "Point32 correction exceeds the configured repair tolerance"
        )

    nudge = min(_POINT32_INWARD_NUDGE_M, remaining * 0.5)
    if nudge > 0.0:
        interior = part.buffer(-nudge)
        if not interior.is_empty:
            candidate = nearest_points(interior, point)[0]
            correction = float(point.distance(candidate))
            if correction <= maximum_correction_m and part.covers(candidate):
                return candidate

    if part.covers(boundary_point):
        return boundary_point

    representative = part.representative_point()
    correction = float(point.distance(representative))
    if correction <= maximum_correction_m and part.covers(representative):
        return representative

    raise ConnectorPlanningError(
        "could not produce a stable Point32 correction inside free_space"
    )


def _normalise_route_inside_free_space(
    route: CoverageRouteRecord,
    free_space_parts: tuple[Polygon, ...],
    connector_config: ConnectorPlannerConfig,
) -> CoverageRouteRecord:
    """Normalize a planner route against the authoritative free-space geometry.

    Planner request polygon vertices are transmitted through ``geometry_msgs/Point32``
    while the authoritative local geometry remains double precision. A planner route
    that terminates on the boundary it received can therefore differ from the source
    boundary by a sub-millimetre amount. Waypoints are snapped to the authoritative
    geometry only when the correction is at most the fixed 1 cm interface tolerance.

    A planner may also return a straight transition between two individually valid
    waypoints that cuts across a concavity, exclusion, or the outer boundary. Such a
    transition is never accepted as flown. When both endpoints lie in the same
    connected free-space component, the unsafe segment is replaced by the exact
    visibility-graph A* connector. Materially outside waypoints and disconnected
    endpoints still fail closed.
    """
    repaired_points: list[CoverageWaypoint] = []
    changed = False

    for index, waypoint in enumerate(route.waypoints):
        raw_point = Point(waypoint.x_m, waypoint.y_m)
        if any(part.covers(raw_point) for part in free_space_parts):
            repaired_points.append(waypoint)
            continue

        distance_m, part_index, _ = _nearest_part_point(
            raw_point, free_space_parts
        )
        if distance_m > ROS_POINT32_ROUTE_REPAIR_TOLERANCE_M:
            raise ConnectorPlanningError(
                f"route {route.request_id!r} waypoint {index} is outside free_space "
                f"by {distance_m:.9f} m, exceeding Point32 repair tolerance "
                f"{ROS_POINT32_ROUTE_REPAIR_TOLERANCE_M:.3f} m"
            )
        snapped = _snap_point_inside_part(
            raw_point,
            free_space_parts[part_index],
            maximum_correction_m=ROS_POINT32_ROUTE_REPAIR_TOLERANCE_M,
        )
        if not free_space_parts[part_index].covers(snapped):
            raise ConnectorPlanningError(
                f"route {route.request_id!r} waypoint {index} could not be "
                "normalised inside free_space"
            )
        repaired_points.append(
            CoverageWaypoint(float(snapped.x), float(snapped.y), waypoint.z_m)
        )
        changed = True

    assembled: list[CoverageWaypoint] = [repaired_points[0]]
    for index, following in enumerate(repaired_points[1:]):
        previous = assembled[-1]
        left = (previous.x_m, previous.y_m)
        right = (following.x_m, following.y_m)
        if left == right:
            assembled.append(following)
            continue

        segment = _segment(left, right)
        if any(part.covers(segment) for part in free_space_parts):
            assembled.append(following)
            continue

        left_point = Point(left)
        right_point = Point(right)
        left_parts = set(_covering_part_indices(left_point, free_space_parts))
        right_parts = set(_covering_part_indices(right_point, free_space_parts))
        common_part_indices = sorted(left_parts.intersection(right_parts))
        if not common_part_indices:
            raise ConnectorPlanningError(
                f"route {route.request_id!r} segment {index} has endpoints in "
                "different connected free-space components"
            )
        repair_part = free_space_parts[common_part_indices[0]]

        repair = plan_connector(
            repair_part,
            LocalPoint2D(previous.x_m, previous.y_m),
            LocalPoint2D(following.x_m, following.y_m),
            config=connector_config,
        )
        repair_tail = repair.waypoints[1:]
        for repair_index, point in enumerate(repair_tail):
            altitude_m = (
                following.z_m
                if repair_index == len(repair_tail) - 1
                else previous.z_m
            )
            assembled.append(CoverageWaypoint(point.x_m, point.y_m, altitude_m))
        changed = True

    repaired = tuple(assembled)
    for index, coordinate in enumerate(
        (point.x_m, point.y_m) for point in repaired
    ):
        if not any(part.covers(Point(coordinate)) for part in free_space_parts):
            raise ConnectorPlanningError(
                f"route {route.request_id!r} waypoint {index} remained outside "
                "free_space after Point32 repair"
            )
    for index, (left, right) in enumerate(
        zip(
            ((point.x_m, point.y_m) for point in repaired),
            ((point.x_m, point.y_m) for point in repaired[1:]),
        )
    ):
        if not any(part.covers(_segment(left, right)) for part in free_space_parts):
            raise ConnectorPlanningError(
                f"route {route.request_id!r} segment {index} remained outside "
                "free_space after Point32 repair"
            )

    if not changed:
        return route
    return CoverageRouteRecord(
        request_id=route.request_id,
        component_id=route.component_id,
        source_region_id=route.source_region_id,
        assigned_vehicle_id=route.assigned_vehicle_id,
        frame=route.frame,
        response_message=route.response_message,
        waypoints=repaired,
    )


def connect_ordered_route_records(
    routes: Iterable[CoverageRouteRecord],
    free_space: BaseGeometry,
    *,
    config: Optional[ConnectorPlannerConfig] = None,
) -> ConnectedRouteSequence:
    """Join ordered per-component routes with safe constant-altitude connectors."""
    try:
        route_list = tuple(routes)
    except TypeError as exc:
        raise ConnectorPlanningError("routes must be iterable") from exc
    if not route_list:
        raise ConnectorPlanningError("routes must not be empty")
    if any(not isinstance(route, CoverageRouteRecord) for route in route_list):
        raise ConnectorPlanningError(
            "routes must contain only CoverageRouteRecord objects"
        )
    if config is not None and not isinstance(config, ConnectorPlannerConfig):
        raise ConnectorPlanningError("config must be a ConnectorPlannerConfig")

    request_ids = [route.request_id for route in route_list]
    if len(set(request_ids)) != len(request_ids):
        raise ConnectorPlanningError("route request IDs must be unique")
    component_ids = [route.component_id for route in route_list]
    if len(set(component_ids)) != len(component_ids):
        raise ConnectorPlanningError("route component IDs must be unique")

    frame = route_list[0].frame
    if any(route.frame != frame for route in route_list[1:]):
        raise ConnectorPlanningError("all routes must use exactly the same frame")

    vehicle_id = route_list[0].assigned_vehicle_id
    if vehicle_id is None:
        raise ConnectorPlanningError("every route must have an assigned vehicle")
    if any(route.assigned_vehicle_id != vehicle_id for route in route_list[1:]):
        raise ConnectorPlanningError(
            "all routes must be assigned to the same vehicle"
        )

    policy = config or ConnectorPlannerConfig()
    free_space_parts = _canonical_free_space_parts(free_space)
    route_list = tuple(
        _normalise_route_inside_free_space(route, free_space_parts, policy)
        for route in route_list
    )

    altitude = route_list[0].waypoints[0].z_m
    for route in route_list:
        for point in route.waypoints:
            if not math.isclose(
                point.z_m,
                altitude,
                rel_tol=0.0,
                abs_tol=_ALTITUDE_TOLERANCE_M,
            ):
                raise ConnectorPlanningError(
                    "all route waypoints must use one common altitude"
                )

    assembled: list[CoverageWaypoint] = list(route_list[0].waypoints)
    connectors: list[RouteConnector] = []
    for previous, following in zip(route_list, route_list[1:]):
        previous_end = previous.waypoints[-1]
        following_start = following.waypoints[0]
        path = plan_connector(
            free_space,
            LocalPoint2D(previous_end.x_m, previous_end.y_m),
            LocalPoint2D(following_start.x_m, following_start.y_m),
            config=policy,
        )
        connector = RouteConnector(
            from_request_id=previous.request_id,
            to_request_id=following.request_id,
            path=path,
            altitude_m=altitude,
        )
        connectors.append(connector)

        connector_points = connector.waypoints
        assembled.extend(connector_points[1:])
        assembled.extend(following.waypoints[1:])

    return ConnectedRouteSequence(
        vehicle_id=vehicle_id,
        frame=frame,
        routes=route_list,
        connectors=tuple(connectors),
        waypoints=tuple(assembled),
    )
