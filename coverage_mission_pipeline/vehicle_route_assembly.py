#!/usr/bin/env python3
"""Globally optimize route direction and assemble one complete vehicle path.

The component visit order is supplied by :class:`VehicleComponentPlan` and is
never changed here.  Each component route may be flown forward or reversed.
A two-state dynamic program finds the globally shortest connector total for
that fixed order, including the vehicle reference-to-first-route connector and,
when requested, the final return-to-reference connector.

All connectors are planned through the authoritative polygonal free space by
``plan_connector``.  The vehicle reference is interpreted at the common route
altitude; takeoff, landing and vertical profiles belong to a later flight-
mission generation layer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Iterable, Optional

from shapely.geometry.base import BaseGeometry

from .planning_request import LocalPoint2D
from .planning_result import CoverageWaypoint
from .route_connector import (
    ConnectorPath,
    ConnectorPlannerConfig,
    ConnectorPlanningError,
    RouteConnector,
    connect_ordered_route_records,
    plan_connector,
)
from .route_record import CoverageRouteRecord
from .vehicle_component_ordering import VehicleComponentPlan

FORWARD_ROUTE_DIRECTION = "forward"
REVERSED_ROUTE_DIRECTION = "reversed"
ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM = "fixed_order_binary_orientation_dp_v1"

_COST_TOLERANCE_M = 1.0e-9
_ALTITUDE_TOLERANCE_M = 1.0e-6
_REFERENCE_ID_PREFIX = "vehicle-reference:"


class VehicleRouteAssemblyError(ValueError):
    """Raised when a complete per-vehicle route cannot be assembled safely."""


def _distance(left: CoverageWaypoint, right: CoverageWaypoint) -> float:
    return math.hypot(right.x_m - left.x_m, right.y_m - left.y_m)


def _waypoint_path_length(points: Iterable[CoverageWaypoint]) -> float:
    sequence = tuple(points)
    return sum(_distance(left, right) for left, right in zip(sequence, sequence[1:]))


def _local(point: CoverageWaypoint) -> LocalPoint2D:
    return LocalPoint2D(point.x_m, point.y_m)


def _route_copy(
    route: CoverageRouteRecord,
    waypoints: tuple[CoverageWaypoint, ...],
) -> CoverageRouteRecord:
    return CoverageRouteRecord(
        request_id=route.request_id,
        component_id=route.component_id,
        source_region_id=route.source_region_id,
        assigned_vehicle_id=route.assigned_vehicle_id,
        frame=route.frame,
        response_message=route.response_message,
        waypoints=waypoints,
    )


@dataclass(frozen=True)
class VehicleRouteAssemblyConfig:
    """Configuration for fixed-order binary route-orientation optimization."""

    return_to_reference: bool = False
    connector_config: ConnectorPlannerConfig = field(
        default_factory=ConnectorPlannerConfig
    )

    def __post_init__(self) -> None:
        if not isinstance(self.return_to_reference, bool):
            raise VehicleRouteAssemblyError("return_to_reference must be a bool")
        if not isinstance(self.connector_config, ConnectorPlannerConfig):
            raise VehicleRouteAssemblyError(
                "connector_config must be a ConnectorPlannerConfig"
            )


@dataclass(frozen=True)
class OrientedRoute:
    """One route record with an explicit forward or reversed traversal."""

    source_route: CoverageRouteRecord
    direction: str

    def __post_init__(self) -> None:
        if not isinstance(self.source_route, CoverageRouteRecord):
            raise VehicleRouteAssemblyError(
                "source_route must be a CoverageRouteRecord"
            )
        if self.direction not in {
            FORWARD_ROUTE_DIRECTION,
            REVERSED_ROUTE_DIRECTION,
        }:
            raise VehicleRouteAssemblyError(
                "direction must be 'forward' or 'reversed'"
            )

    @property
    def waypoints(self) -> tuple[CoverageWaypoint, ...]:
        if self.direction == FORWARD_ROUTE_DIRECTION:
            return self.source_route.waypoints
        return tuple(reversed(self.source_route.waypoints))

    @property
    def start(self) -> LocalPoint2D:
        return _local(self.waypoints[0])

    @property
    def goal(self) -> LocalPoint2D:
        return _local(self.waypoints[-1])

    @property
    def route_length_m(self) -> float:
        return _waypoint_path_length(self.waypoints)

    def to_route_record(self) -> CoverageRouteRecord:
        """Return an immutable route record in the selected traversal order."""
        return _route_copy(self.source_route, self.waypoints)

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "request_id": self.source_route.request_id,
            "component_id": self.source_route.component_id,
            "direction": self.direction,
            "route_length_m": self.route_length_m,
            "start": self.start.to_dict(),
            "goal": self.goal.to_dict(),
            "waypoint_count": len(self.waypoints),
        }


@dataclass(frozen=True)
class CompleteVehicleRoute:
    """A continuous cruise-altitude path from a vehicle reference through routes."""

    component_plan: VehicleComponentPlan
    oriented_routes: tuple[OrientedRoute, ...]
    reference_connector: Optional[RouteConnector]
    inter_route_connectors: tuple[RouteConnector, ...]
    return_connector: Optional[RouteConnector]
    waypoints: tuple[CoverageWaypoint, ...]
    return_to_reference: bool
    algorithm: str = ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM

    def __post_init__(self) -> None:
        if not isinstance(self.component_plan, VehicleComponentPlan):
            raise VehicleRouteAssemblyError(
                "component_plan must be a VehicleComponentPlan"
            )
        if not isinstance(self.return_to_reference, bool):
            raise VehicleRouteAssemblyError("return_to_reference must be a bool")
        if self.algorithm != ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM:
            raise VehicleRouteAssemblyError(
                f"algorithm must be {ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM!r}"
            )

        routes = tuple(self.oriented_routes)
        inter = tuple(self.inter_route_connectors)
        points = tuple(self.waypoints)
        if any(not isinstance(route, OrientedRoute) for route in routes):
            raise VehicleRouteAssemblyError(
                "oriented_routes must contain only OrientedRoute objects"
            )
        if any(not isinstance(item, RouteConnector) for item in inter):
            raise VehicleRouteAssemblyError(
                "inter_route_connectors must contain only RouteConnector objects"
            )
        if any(not isinstance(point, CoverageWaypoint) for point in points):
            raise VehicleRouteAssemblyError(
                "waypoints must contain only CoverageWaypoint objects"
            )
        object.__setattr__(self, "oriented_routes", routes)
        object.__setattr__(self, "inter_route_connectors", inter)
        object.__setattr__(self, "waypoints", points)

        expected_component_ids = self.component_plan.component_ids
        actual_component_ids = tuple(
            route.source_route.component_id for route in routes
        )
        if actual_component_ids != expected_component_ids:
            raise VehicleRouteAssemblyError(
                "oriented route component order must match component_plan"
            )

        if not routes:
            if self.reference_connector is not None or inter or self.return_connector is not None:
                raise VehicleRouteAssemblyError(
                    "an idle vehicle route must not contain connectors"
                )
            if points:
                raise VehicleRouteAssemblyError(
                    "an idle vehicle route must not contain waypoints"
                )
            return

        if not isinstance(self.reference_connector, RouteConnector):
            raise VehicleRouteAssemblyError(
                "a non-idle vehicle route requires a reference_connector"
            )
        if len(inter) != len(routes) - 1:
            raise VehicleRouteAssemblyError(
                "inter-route connector count must equal route count minus one"
            )
        if self.return_to_reference:
            if not isinstance(self.return_connector, RouteConnector):
                raise VehicleRouteAssemblyError(
                    "return_to_reference requires a return_connector"
                )
        elif self.return_connector is not None:
            raise VehicleRouteAssemblyError(
                "return_connector must be None when return_to_reference is false"
            )
        if not points:
            raise VehicleRouteAssemblyError(
                "a non-idle vehicle route requires assembled waypoints"
            )

        vehicle_id = self.component_plan.vehicle_id
        frame = self.component_plan.reference.frame
        for route in routes:
            record = route.source_route
            if record.assigned_vehicle_id != vehicle_id:
                raise VehicleRouteAssemblyError(
                    "every route must be assigned to the component-plan vehicle"
                )
            if record.frame != frame:
                raise VehicleRouteAssemblyError(
                    "every route frame must match the vehicle reference frame"
                )

        reference = self.component_plan.reference.position
        first_start = routes[0].start
        if self.reference_connector.path.start != reference:
            raise VehicleRouteAssemblyError(
                "reference connector must start at the vehicle reference"
            )
        if self.reference_connector.path.goal != first_start:
            raise VehicleRouteAssemblyError(
                "reference connector must end at the first oriented route"
            )

        for index, connector in enumerate(inter):
            if connector.path.start != routes[index].goal:
                raise VehicleRouteAssemblyError(
                    "inter-route connector start does not match preceding route"
                )
            if connector.path.goal != routes[index + 1].start:
                raise VehicleRouteAssemblyError(
                    "inter-route connector goal does not match following route"
                )

        if self.return_connector is not None:
            if self.return_connector.path.start != routes[-1].goal:
                raise VehicleRouteAssemblyError(
                    "return connector must start at the final oriented route"
                )
            if self.return_connector.path.goal != reference:
                raise VehicleRouteAssemblyError(
                    "return connector must end at the vehicle reference"
                )

        expected = _assemble_waypoints(
            routes,
            self.reference_connector,
            inter,
            self.return_connector,
        )
        if points != expected:
            raise VehicleRouteAssemblyError(
                "assembled waypoints do not match routes and connectors"
            )

    @property
    def vehicle_id(self) -> str:
        return self.component_plan.vehicle_id

    @property
    def frame(self):
        return self.component_plan.reference.frame

    @property
    def is_idle(self) -> bool:
        return not self.oriented_routes

    @property
    def route_records(self) -> tuple[CoverageRouteRecord, ...]:
        return tuple(route.to_route_record() for route in self.oriented_routes)

    @property
    def route_directions(self) -> tuple[str, ...]:
        return tuple(route.direction for route in self.oriented_routes)

    @property
    def total_route_length_m(self) -> float:
        return sum(route.route_length_m for route in self.oriented_routes)

    @property
    def total_connector_length_m(self) -> float:
        total = sum(item.path.length_m for item in self.inter_route_connectors)
        if self.reference_connector is not None:
            total += self.reference_connector.path.length_m
        if self.return_connector is not None:
            total += self.return_connector.path.length_m
        return total

    @property
    def total_path_length_m(self) -> float:
        return _waypoint_path_length(self.waypoints)

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "vehicle_id": self.vehicle_id,
            "reference_type": self.component_plan.reference.reference_type,
            "algorithm": self.algorithm,
            "return_to_reference": self.return_to_reference,
            "is_idle": self.is_idle,
            "route_count": len(self.oriented_routes),
            "route_directions": list(self.route_directions),
            "total_route_length_m": self.total_route_length_m,
            "total_connector_length_m": self.total_connector_length_m,
            "total_path_length_m": self.total_path_length_m,
            "waypoint_count": len(self.waypoints),
            "routes": [route.to_summary_dict() for route in self.oriented_routes],
            "reference_connector": (
                None
                if self.reference_connector is None
                else self.reference_connector.to_summary_dict()
            ),
            "inter_route_connectors": [
                item.to_summary_dict() for item in self.inter_route_connectors
            ],
            "return_connector": (
                None
                if self.return_connector is None
                else self.return_connector.to_summary_dict()
            ),
        }


@dataclass(frozen=True)
class _OrientationState:
    cost_m: float
    signature: tuple[int, ...]


def _is_better(candidate: _OrientationState, incumbent: Optional[_OrientationState]) -> bool:
    if incumbent is None:
        return True
    if candidate.cost_m < incumbent.cost_m - _COST_TOLERANCE_M:
        return True
    if abs(candidate.cost_m - incumbent.cost_m) <= _COST_TOLERANCE_M:
        return candidate.signature < incumbent.signature
    return False


def _assemble_waypoints(
    routes: tuple[OrientedRoute, ...],
    reference_connector: RouteConnector,
    inter_connectors: tuple[RouteConnector, ...],
    return_connector: Optional[RouteConnector],
) -> tuple[CoverageWaypoint, ...]:
    assembled: list[CoverageWaypoint] = list(reference_connector.waypoints)
    assembled.extend(routes[0].waypoints[1:])
    for connector, route in zip(inter_connectors, routes[1:]):
        assembled.extend(connector.waypoints[1:])
        assembled.extend(route.waypoints[1:])
    if return_connector is not None:
        assembled.extend(return_connector.waypoints[1:])
    return tuple(assembled)


def _validate_and_order_routes(
    plan: VehicleComponentPlan,
    routes: Iterable[CoverageRouteRecord],
    free_space: BaseGeometry,
    connector_config: ConnectorPlannerConfig,
) -> tuple[CoverageRouteRecord, ...]:
    try:
        values = tuple(routes)
    except TypeError as exc:
        raise VehicleRouteAssemblyError("routes must be iterable") from exc
    for index, route in enumerate(values):
        if not isinstance(route, CoverageRouteRecord):
            raise VehicleRouteAssemblyError(
                f"routes[{index}] must be a CoverageRouteRecord"
            )

    component_ids = [route.component_id for route in values]
    if len(component_ids) != len(set(component_ids)):
        raise VehicleRouteAssemblyError("route component IDs must be unique")
    request_ids = [route.request_id for route in values]
    if len(request_ids) != len(set(request_ids)):
        raise VehicleRouteAssemblyError("route request IDs must be unique")

    expected = plan.component_ids
    actual = set(component_ids)
    if actual != set(expected) or len(values) != len(expected):
        missing = sorted(set(expected) - actual)
        extra = sorted(actual - set(expected))
        details = []
        if missing:
            details.append("missing: " + ", ".join(missing))
        if extra:
            details.append("unexpected: " + ", ".join(extra))
        suffix = "" if not details else " (" + "; ".join(details) + ")"
        raise VehicleRouteAssemblyError(
            "routes must contain exactly one record for every planned component"
            + suffix
        )

    by_component = {route.component_id: route for route in values}
    ordered = tuple(by_component[component_id] for component_id in expected)
    for route in ordered:
        if route.assigned_vehicle_id != plan.vehicle_id:
            raise VehicleRouteAssemblyError(
                f"route {route.request_id!r} is not assigned to vehicle "
                f"{plan.vehicle_id!r}"
            )
        if route.frame != plan.reference.frame:
            raise VehicleRouteAssemblyError(
                f"route {route.request_id!r} frame does not match vehicle reference"
            )
        try:
            # A one-route sequence performs full route/free-space and altitude
            # validation without constructing an inter-route connector.
            connect_ordered_route_records(
                (route,),
                free_space,
                config=connector_config,
            )
        except ConnectorPlanningError as exc:
            raise VehicleRouteAssemblyError(
                f"route {route.request_id!r} is invalid: {exc}"
            ) from exc

    if ordered:
        altitude = ordered[0].waypoints[0].z_m
        for route in ordered:
            for point in route.waypoints:
                if not math.isclose(
                    point.z_m,
                    altitude,
                    rel_tol=0.0,
                    abs_tol=_ALTITUDE_TOLERANCE_M,
                ):
                    raise VehicleRouteAssemblyError(
                        "all vehicle route waypoints must use one common altitude"
                    )
    return ordered


def assemble_vehicle_route(
    component_plan: VehicleComponentPlan,
    routes: Iterable[CoverageRouteRecord],
    free_space: BaseGeometry,
    *,
    config: Optional[VehicleRouteAssemblyConfig] = None,
) -> CompleteVehicleRoute:
    """Choose route directions globally and assemble a continuous vehicle path."""
    if not isinstance(component_plan, VehicleComponentPlan):
        raise VehicleRouteAssemblyError(
            "component_plan must be a VehicleComponentPlan"
        )
    if config is not None and not isinstance(config, VehicleRouteAssemblyConfig):
        raise VehicleRouteAssemblyError(
            "config must be a VehicleRouteAssemblyConfig"
        )
    policy = config or VehicleRouteAssemblyConfig()
    ordered = _validate_and_order_routes(
        component_plan,
        routes,
        free_space,
        policy.connector_config,
    )

    if not ordered:
        return CompleteVehicleRoute(
            component_plan=component_plan,
            oriented_routes=(),
            reference_connector=None,
            inter_route_connectors=(),
            return_connector=None,
            waypoints=(),
            return_to_reference=policy.return_to_reference,
        )

    options = tuple(
        (
            OrientedRoute(route, FORWARD_ROUTE_DIRECTION),
            OrientedRoute(route, REVERSED_ROUTE_DIRECTION),
        )
        for route in ordered
    )
    reference = component_plan.reference.position
    cache: dict[tuple[float, float, float, float], Optional[ConnectorPath]] = {}

    def connector(start: LocalPoint2D, goal: LocalPoint2D) -> Optional[ConnectorPath]:
        key = (start.x_m, start.y_m, goal.x_m, goal.y_m)
        if key not in cache:
            try:
                cache[key] = plan_connector(
                    free_space,
                    start,
                    goal,
                    config=policy.connector_config,
                )
            except ConnectorPlanningError:
                cache[key] = None
        return cache[key]

    states: dict[int, _OrientationState] = {}
    for direction_index, route in enumerate(options[0]):
        path = connector(reference, route.start)
        if path is not None:
            states[direction_index] = _OrientationState(
                path.length_m,
                (direction_index,),
            )
    if not states:
        raise VehicleRouteAssemblyError(
            "vehicle reference cannot reach either orientation of the first route"
        )

    for route_index in range(1, len(options)):
        following_states: dict[int, _OrientationState] = {}
        for current_index, current_route in enumerate(options[route_index]):
            best: Optional[_OrientationState] = None
            for previous_index, previous_state in states.items():
                previous_route = options[route_index - 1][previous_index]
                path = connector(previous_route.goal, current_route.start)
                if path is None:
                    continue
                candidate = _OrientationState(
                    previous_state.cost_m + path.length_m,
                    previous_state.signature + (current_index,),
                )
                if _is_better(candidate, best):
                    best = candidate
            if best is not None:
                following_states[current_index] = best
        states = following_states
        if not states:
            raise VehicleRouteAssemblyError(
                f"no safe connector reaches planned route index {route_index + 1}"
            )

    final_state: Optional[_OrientationState] = None
    for final_index, state in states.items():
        candidate = state
        if policy.return_to_reference:
            return_path = connector(options[-1][final_index].goal, reference)
            if return_path is None:
                continue
            candidate = _OrientationState(
                state.cost_m + return_path.length_m,
                state.signature,
            )
        if _is_better(candidate, final_state):
            final_state = candidate
    if final_state is None:
        raise VehicleRouteAssemblyError(
            "no route orientation can satisfy the requested return to reference"
        )

    oriented = tuple(
        options[index][direction]
        for index, direction in enumerate(final_state.signature)
    )
    altitude = oriented[0].waypoints[0].z_m
    reference_id = _REFERENCE_ID_PREFIX + component_plan.vehicle_id

    reference_path = connector(reference, oriented[0].start)
    if reference_path is None:  # pragma: no cover - protected by DP state
        raise VehicleRouteAssemblyError("selected reference connector disappeared")
    reference_connector = RouteConnector(
        from_request_id=reference_id,
        to_request_id=oriented[0].source_route.request_id,
        path=reference_path,
        altitude_m=altitude,
    )

    inter_connectors: list[RouteConnector] = []
    for previous, following in zip(oriented, oriented[1:]):
        path = connector(previous.goal, following.start)
        if path is None:  # pragma: no cover - protected by DP state
            raise VehicleRouteAssemblyError("selected inter-route connector disappeared")
        inter_connectors.append(
            RouteConnector(
                from_request_id=previous.source_route.request_id,
                to_request_id=following.source_route.request_id,
                path=path,
                altitude_m=altitude,
            )
        )

    return_connector: Optional[RouteConnector] = None
    if policy.return_to_reference:
        path = connector(oriented[-1].goal, reference)
        if path is None:  # pragma: no cover - protected by final selection
            raise VehicleRouteAssemblyError("selected return connector disappeared")
        return_connector = RouteConnector(
            from_request_id=oriented[-1].source_route.request_id,
            to_request_id=reference_id,
            path=path,
            altitude_m=altitude,
        )

    inter_tuple = tuple(inter_connectors)
    assembled = _assemble_waypoints(
        oriented,
        reference_connector,
        inter_tuple,
        return_connector,
    )
    return CompleteVehicleRoute(
        component_plan=component_plan,
        oriented_routes=oriented,
        reference_connector=reference_connector,
        inter_route_connectors=inter_tuple,
        return_connector=return_connector,
        waypoints=assembled,
        return_to_reference=policy.return_to_reference,
    )
