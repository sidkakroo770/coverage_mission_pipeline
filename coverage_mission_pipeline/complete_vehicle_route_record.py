#!/usr/bin/env python3
"""Stable serialization for one fully assembled per-vehicle cruise route.

The record stores one authoritative ordered array of local Cartesian waypoints.
Coverage-route and connector metadata refer to inclusive spans in that array, so
shared endpoints are not serialized repeatedly and cannot drift out of sync.
Projected and WGS84 coordinates are derived on demand from LocalCartesianFrame.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
from typing import Any, Iterable, Mapping, Optional, Union

from .planning_request import LocalPoint2D, PlanningRequestError
from .planning_result import CoverageWaypoint, PlanningResultError
from .prepared_component import (
    LocalCartesianFrame,
    PreparedComponentError,
)
from .route_connector import (
    DIRECT_CONNECTOR_ALGORITHM,
    TRIVIAL_CONNECTOR_ALGORITHM,
    VISIBILITY_ASTAR_ALGORITHM,
    RouteConnector,
)
from .route_record import (
    GeographicWaypoint,
    ProjectedWaypoint,
    RouteRecordError,
    local_to_geographic,
    local_to_projected,
)
from .vehicle_route_assembly import (
    FORWARD_ROUTE_DIRECTION,
    REVERSED_ROUTE_DIRECTION,
    ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM,
    CompleteVehicleRoute,
)

COMPLETE_VEHICLE_ROUTE_SCHEMA_VERSION = 1
COVERAGE_ROUTE_SEGMENT_KIND = "coverage_route"
CONNECTOR_SEGMENT_KIND = "connector"
REFERENCE_CONNECTOR_ROLE = "reference_connector"
INTER_ROUTE_CONNECTOR_ROLE = "inter_route_connector"
RETURN_CONNECTOR_ROLE = "return_connector"

_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_REFERENCE_TYPES = frozenset({"home", "launch", "current_position", "custom"})
_CONNECTOR_ROLES = frozenset(
    {
        REFERENCE_CONNECTOR_ROLE,
        INTER_ROUTE_CONNECTOR_ROLE,
        RETURN_CONNECTOR_ROLE,
    }
)
_CONNECTOR_ALGORITHMS = frozenset(
    {
        DIRECT_CONNECTOR_ALGORITHM,
        TRIVIAL_CONNECTOR_ALGORITHM,
        VISIBILITY_ASTAR_ALGORITHM,
    }
)
_LENGTH_TOLERANCE_M = 1.0e-8
_ALTITUDE_TOLERANCE_M = 1.0e-6
_REFERENCE_ID_PREFIX = "vehicle-reference:"


class CompleteVehicleRouteRecordError(ValueError):
    """Raised when a complete vehicle-route record is malformed or inconsistent."""


def _strict_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CompleteVehicleRouteRecordError(f"{path} must be an object")
    return value


def _strict_keys(
    value: Mapping[str, Any],
    required: set[str],
    path: str,
) -> None:
    actual = set(value.keys())
    missing = sorted(required - actual)
    unknown = sorted(actual - required)
    if missing:
        raise CompleteVehicleRouteRecordError(
            f"{path} is missing required field(s): {', '.join(missing)}"
        )
    if unknown:
        raise CompleteVehicleRouteRecordError(
            f"{path} contains unknown field(s): {', '.join(unknown)}"
        )


def _identifier(value: Any, path: str) -> str:
    if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
        raise CompleteVehicleRouteRecordError(
            f"{path} must match {_ID_PATTERN.pattern!r}"
        )
    return value


def _nonempty_string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise CompleteVehicleRouteRecordError(f"{path} must be a non-empty string")
    return value


def _finite_nonnegative(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CompleteVehicleRouteRecordError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise CompleteVehicleRouteRecordError(
            f"{path} must be finite and non-negative"
        )
    return result


def _nonnegative_integer(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise CompleteVehicleRouteRecordError(
            f"{path} must be a non-negative integer"
        )
    return value


def _distance(left: CoverageWaypoint, right: CoverageWaypoint) -> float:
    return math.hypot(right.x_m - left.x_m, right.y_m - left.y_m)


def _path_length(points: Iterable[CoverageWaypoint]) -> float:
    sequence = tuple(points)
    return sum(_distance(left, right) for left, right in zip(sequence, sequence[1:]))


@dataclass(frozen=True)
class WaypointSpan:
    """Inclusive indexes into the record's single authoritative waypoint array."""

    start_index: int
    end_index: int

    def __post_init__(self) -> None:
        start = _nonnegative_integer(self.start_index, "span.start_index")
        end = _nonnegative_integer(self.end_index, "span.end_index")
        if end < start:
            raise CompleteVehicleRouteRecordError(
                "span.end_index must be greater than or equal to span.start_index"
            )
        object.__setattr__(self, "start_index", start)
        object.__setattr__(self, "end_index", end)

    @property
    def waypoint_count(self) -> int:
        return self.end_index - self.start_index + 1

    def to_dict(self) -> dict[str, int]:
        return {
            "start_index": self.start_index,
            "end_index": self.end_index,
        }

    @classmethod
    def from_dict(cls, value: Any, path: str = "span") -> "WaypointSpan":
        mapping = _strict_mapping(value, path)
        _strict_keys(mapping, {"start_index", "end_index"}, path)
        try:
            return cls(mapping["start_index"], mapping["end_index"])
        except CompleteVehicleRouteRecordError as exc:
            raise CompleteVehicleRouteRecordError(f"{path} is invalid: {exc}") from exc


@dataclass(frozen=True)
class CoverageRouteSpanRecord:
    """Metadata for one oriented coverage route inside the waypoint array."""

    span: WaypointSpan
    request_id: str
    component_id: str
    source_region_id: str
    direction: str
    response_message: str
    length_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.span, WaypointSpan):
            raise CompleteVehicleRouteRecordError("span must be a WaypointSpan")
        object.__setattr__(self, "request_id", _identifier(self.request_id, "request_id"))
        object.__setattr__(
            self,
            "component_id",
            _identifier(self.component_id, "component_id"),
        )
        object.__setattr__(
            self,
            "source_region_id",
            _identifier(self.source_region_id, "source_region_id"),
        )
        if self.direction not in {
            FORWARD_ROUTE_DIRECTION,
            REVERSED_ROUTE_DIRECTION,
        }:
            raise CompleteVehicleRouteRecordError(
                "direction must be 'forward' or 'reversed'"
            )
        if not isinstance(self.response_message, str):
            raise CompleteVehicleRouteRecordError("response_message must be a string")
        object.__setattr__(
            self,
            "length_m",
            _finite_nonnegative(self.length_m, "length_m"),
        )

    @property
    def kind(self) -> str:
        return COVERAGE_ROUTE_SEGMENT_KIND

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "span": self.span.to_dict(),
            "request_id": self.request_id,
            "component_id": self.component_id,
            "source_region_id": self.source_region_id,
            "direction": self.direction,
            "response_message": self.response_message,
            "length_m": self.length_m,
        }

    @classmethod
    def from_dict(cls, value: Any, path: str) -> "CoverageRouteSpanRecord":
        mapping = _strict_mapping(value, path)
        _strict_keys(
            mapping,
            {
                "kind",
                "span",
                "request_id",
                "component_id",
                "source_region_id",
                "direction",
                "response_message",
                "length_m",
            },
            path,
        )
        if mapping["kind"] != COVERAGE_ROUTE_SEGMENT_KIND:
            raise CompleteVehicleRouteRecordError(
                f"{path}.kind must be {COVERAGE_ROUTE_SEGMENT_KIND!r}"
            )
        return cls(
            span=WaypointSpan.from_dict(mapping["span"], f"{path}.span"),
            request_id=mapping["request_id"],
            component_id=mapping["component_id"],
            source_region_id=mapping["source_region_id"],
            direction=mapping["direction"],
            response_message=mapping["response_message"],
            length_m=mapping["length_m"],
        )


@dataclass(frozen=True)
class ConnectorSpanRecord:
    """Metadata for one home or inter-route connector waypoint span."""

    span: WaypointSpan
    role: str
    from_request_id: str
    to_request_id: str
    algorithm: str
    length_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.span, WaypointSpan):
            raise CompleteVehicleRouteRecordError("span must be a WaypointSpan")
        if self.role not in _CONNECTOR_ROLES:
            raise CompleteVehicleRouteRecordError(
                "role must be a supported connector role"
            )
        object.__setattr__(
            self,
            "from_request_id",
            _nonempty_string(self.from_request_id, "from_request_id"),
        )
        object.__setattr__(
            self,
            "to_request_id",
            _nonempty_string(self.to_request_id, "to_request_id"),
        )
        if self.from_request_id == self.to_request_id:
            raise CompleteVehicleRouteRecordError(
                "from_request_id and to_request_id must differ"
            )
        if self.algorithm not in _CONNECTOR_ALGORITHMS:
            raise CompleteVehicleRouteRecordError(
                "algorithm must be a supported connector algorithm"
            )
        object.__setattr__(
            self,
            "length_m",
            _finite_nonnegative(self.length_m, "length_m"),
        )

    @property
    def kind(self) -> str:
        return CONNECTOR_SEGMENT_KIND

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "span": self.span.to_dict(),
            "role": self.role,
            "from_request_id": self.from_request_id,
            "to_request_id": self.to_request_id,
            "algorithm": self.algorithm,
            "length_m": self.length_m,
        }

    @classmethod
    def from_dict(cls, value: Any, path: str) -> "ConnectorSpanRecord":
        mapping = _strict_mapping(value, path)
        _strict_keys(
            mapping,
            {
                "kind",
                "span",
                "role",
                "from_request_id",
                "to_request_id",
                "algorithm",
                "length_m",
            },
            path,
        )
        if mapping["kind"] != CONNECTOR_SEGMENT_KIND:
            raise CompleteVehicleRouteRecordError(
                f"{path}.kind must be {CONNECTOR_SEGMENT_KIND!r}"
            )
        return cls(
            span=WaypointSpan.from_dict(mapping["span"], f"{path}.span"),
            role=mapping["role"],
            from_request_id=mapping["from_request_id"],
            to_request_id=mapping["to_request_id"],
            algorithm=mapping["algorithm"],
            length_m=mapping["length_m"],
        )


VehicleRouteSegmentRecord = Union[CoverageRouteSpanRecord, ConnectorSpanRecord]


def _segment_from_dict(value: Any, path: str) -> VehicleRouteSegmentRecord:
    mapping = _strict_mapping(value, path)
    kind = mapping.get("kind")
    if kind == COVERAGE_ROUTE_SEGMENT_KIND:
        return CoverageRouteSpanRecord.from_dict(mapping, path)
    if kind == CONNECTOR_SEGMENT_KIND:
        return ConnectorSpanRecord.from_dict(mapping, path)
    raise CompleteVehicleRouteRecordError(
        f"{path}.kind must be {COVERAGE_ROUTE_SEGMENT_KIND!r} or "
        f"{CONNECTOR_SEGMENT_KIND!r}"
    )


@dataclass(frozen=True)
class CompleteVehicleRouteRecord:
    """Self-contained, deterministic record of one assembled vehicle route."""

    vehicle_id: str
    frame: LocalCartesianFrame
    reference_type: str
    reference_position: LocalPoint2D
    algorithm: str
    return_to_reference: bool
    segments: tuple[VehicleRouteSegmentRecord, ...]
    waypoints: tuple[CoverageWaypoint, ...]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "vehicle_id",
            _identifier(self.vehicle_id, "vehicle_id"),
        )
        if not isinstance(self.frame, LocalCartesianFrame):
            raise CompleteVehicleRouteRecordError(
                "frame must be a LocalCartesianFrame"
            )
        if self.reference_type not in _REFERENCE_TYPES:
            raise CompleteVehicleRouteRecordError(
                "reference_type must be one of: "
                + ", ".join(sorted(_REFERENCE_TYPES))
            )
        if not isinstance(self.reference_position, LocalPoint2D):
            raise CompleteVehicleRouteRecordError(
                "reference_position must be a LocalPoint2D"
            )
        if self.algorithm != ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM:
            raise CompleteVehicleRouteRecordError(
                f"algorithm must be {ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM!r}"
            )
        if not isinstance(self.return_to_reference, bool):
            raise CompleteVehicleRouteRecordError(
                "return_to_reference must be a bool"
            )
        try:
            segments = tuple(self.segments)
        except TypeError as exc:
            raise CompleteVehicleRouteRecordError("segments must be iterable") from exc
        if any(
            not isinstance(segment, (CoverageRouteSpanRecord, ConnectorSpanRecord))
            for segment in segments
        ):
            raise CompleteVehicleRouteRecordError(
                "segments contain an unsupported record type"
            )
        try:
            waypoints = tuple(self.waypoints)
        except TypeError as exc:
            raise CompleteVehicleRouteRecordError("waypoints must be iterable") from exc
        if any(not isinstance(point, CoverageWaypoint) for point in waypoints):
            raise CompleteVehicleRouteRecordError(
                "waypoints must contain only CoverageWaypoint objects"
            )
        object.__setattr__(self, "segments", segments)
        object.__setattr__(self, "waypoints", waypoints)
        self._validate_structure()

    def _validate_structure(self) -> None:
        if not self.segments:
            if self.waypoints:
                raise CompleteVehicleRouteRecordError(
                    "an idle record must not contain waypoints"
                )
            return
        if not self.waypoints:
            raise CompleteVehicleRouteRecordError(
                "a non-idle record must contain waypoints"
            )

        first = self.waypoints[0]
        if not math.isclose(
            first.x_m,
            self.reference_position.x_m,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ) or not math.isclose(
            first.y_m,
            self.reference_position.y_m,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise CompleteVehicleRouteRecordError(
                "the first waypoint must coincide with the vehicle reference"
            )
        if self.return_to_reference:
            last = self.waypoints[-1]
            if not math.isclose(
                last.x_m,
                self.reference_position.x_m,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            ) or not math.isclose(
                last.y_m,
                self.reference_position.y_m,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            ):
                raise CompleteVehicleRouteRecordError(
                    "return_to_reference requires the final waypoint at the reference"
                )

        altitude = first.z_m
        for index, point in enumerate(self.waypoints[1:], start=1):
            if not math.isclose(
                point.z_m,
                altitude,
                rel_tol=0.0,
                abs_tol=_ALTITUDE_TOLERANCE_M,
            ):
                raise CompleteVehicleRouteRecordError(
                    f"waypoint {index} has inconsistent cruise altitude"
                )

        for index, segment in enumerate(self.segments):
            if segment.span.end_index >= len(self.waypoints):
                raise CompleteVehicleRouteRecordError(
                    f"segments[{index}].span exceeds the waypoint array"
                )
            if index == 0:
                if segment.span.start_index != 0:
                    raise CompleteVehicleRouteRecordError(
                        "the first segment must start at waypoint index zero"
                    )
            elif segment.span.start_index != self.segments[index - 1].span.end_index:
                raise CompleteVehicleRouteRecordError(
                    "adjacent segments must share exactly their boundary waypoint"
                )
            actual = _path_length(self.segment_waypoints(segment))
            if not math.isclose(
                segment.length_m,
                actual,
                rel_tol=0.0,
                abs_tol=_LENGTH_TOLERANCE_M,
            ):
                raise CompleteVehicleRouteRecordError(
                    f"segments[{index}].length_m does not match its waypoint span"
                )
        if self.segments[-1].span.end_index != len(self.waypoints) - 1:
            raise CompleteVehicleRouteRecordError(
                "the final segment must end at the final waypoint"
            )

        route_segments = self.route_segments
        connector_segments = self.connector_segments
        if not route_segments:
            raise CompleteVehicleRouteRecordError(
                "a non-idle record requires at least one coverage route segment"
            )
        request_ids = [segment.request_id for segment in route_segments]
        component_ids = [segment.component_id for segment in route_segments]
        if len(request_ids) != len(set(request_ids)):
            raise CompleteVehicleRouteRecordError(
                "coverage route request IDs must be unique"
            )
        if len(component_ids) != len(set(component_ids)):
            raise CompleteVehicleRouteRecordError(
                "coverage route component IDs must be unique"
            )

        expected_count = 2 * len(route_segments) + int(self.return_to_reference)
        if len(self.segments) != expected_count:
            raise CompleteVehicleRouteRecordError(
                "segment count does not match route count and return policy"
            )

        reference_id = _REFERENCE_ID_PREFIX + self.vehicle_id
        route_index = 0
        for index, segment in enumerate(self.segments):
            if index % 2 == 0:
                if not isinstance(segment, ConnectorSpanRecord):
                    raise CompleteVehicleRouteRecordError(
                        "connector and coverage-route segments must alternate"
                    )
                if index == 0:
                    expected_role = REFERENCE_CONNECTOR_ROLE
                    expected_from = reference_id
                    expected_to = route_segments[0].request_id
                elif index == len(self.segments) - 1 and self.return_to_reference:
                    expected_role = RETURN_CONNECTOR_ROLE
                    expected_from = route_segments[-1].request_id
                    expected_to = reference_id
                else:
                    expected_role = INTER_ROUTE_CONNECTOR_ROLE
                    previous_route = route_segments[route_index - 1]
                    following_route = route_segments[route_index]
                    expected_from = previous_route.request_id
                    expected_to = following_route.request_id
                if segment.role != expected_role:
                    raise CompleteVehicleRouteRecordError(
                        f"connector segment {index} has the wrong role"
                    )
                if (
                    segment.from_request_id != expected_from
                    or segment.to_request_id != expected_to
                ):
                    raise CompleteVehicleRouteRecordError(
                        f"connector segment {index} has an inconsistent endpoint chain"
                    )
            else:
                if not isinstance(segment, CoverageRouteSpanRecord):
                    raise CompleteVehicleRouteRecordError(
                        "connector and coverage-route segments must alternate"
                    )
                if segment is not route_segments[route_index]:
                    raise CompleteVehicleRouteRecordError(
                        "coverage route segment ordering is inconsistent"
                    )
                route_index += 1

        if len(connector_segments) != len(route_segments) - 1 + 1 + int(
            self.return_to_reference
        ):
            raise CompleteVehicleRouteRecordError(
                "connector count does not match route count and return policy"
            )
        segment_total = sum(segment.length_m for segment in self.segments)
        if not math.isclose(
            segment_total,
            self.total_path_length_m,
            rel_tol=0.0,
            abs_tol=_LENGTH_TOLERANCE_M,
        ):
            raise CompleteVehicleRouteRecordError(
                "segment lengths do not reproduce the assembled path length"
            )

    @property
    def is_idle(self) -> bool:
        return not self.segments

    @property
    def route_segments(self) -> tuple[CoverageRouteSpanRecord, ...]:
        return tuple(
            segment
            for segment in self.segments
            if isinstance(segment, CoverageRouteSpanRecord)
        )

    @property
    def connector_segments(self) -> tuple[ConnectorSpanRecord, ...]:
        return tuple(
            segment
            for segment in self.segments
            if isinstance(segment, ConnectorSpanRecord)
        )

    @property
    def route_directions(self) -> tuple[str, ...]:
        return tuple(segment.direction for segment in self.route_segments)

    @property
    def total_route_length_m(self) -> float:
        return sum(segment.length_m for segment in self.route_segments)

    @property
    def total_connector_length_m(self) -> float:
        return sum(segment.length_m for segment in self.connector_segments)

    @property
    def total_path_length_m(self) -> float:
        return _path_length(self.waypoints)

    def segment_waypoints(
        self,
        segment: VehicleRouteSegmentRecord,
    ) -> tuple[CoverageWaypoint, ...]:
        if not isinstance(segment, (CoverageRouteSpanRecord, ConnectorSpanRecord)):
            raise CompleteVehicleRouteRecordError(
                "segment must be a route or connector span record"
            )
        if segment not in self.segments:
            raise CompleteVehicleRouteRecordError(
                "segment does not belong to this complete route record"
            )
        return self.waypoints[
            segment.span.start_index : segment.span.end_index + 1
        ]

    def projected_waypoints(self) -> tuple[ProjectedWaypoint, ...]:
        return tuple(local_to_projected(point, self.frame) for point in self.waypoints)

    def geographic_waypoints(self) -> tuple[GeographicWaypoint, ...]:
        return tuple(local_to_geographic(point, self.frame) for point in self.waypoints)

    @classmethod
    def from_complete_route(
        cls,
        route: CompleteVehicleRoute,
    ) -> "CompleteVehicleRouteRecord":
        if not isinstance(route, CompleteVehicleRoute):
            raise CompleteVehicleRouteRecordError(
                "route must be a CompleteVehicleRoute"
            )
        reference = route.component_plan.reference
        if route.is_idle:
            return cls(
                vehicle_id=route.vehicle_id,
                frame=route.frame,
                reference_type=reference.reference_type,
                reference_position=reference.position,
                algorithm=route.algorithm,
                return_to_reference=route.return_to_reference,
                segments=(),
                waypoints=(),
            )

        segments: list[VehicleRouteSegmentRecord] = []
        cursor = 0

        def add_connector(connector: RouteConnector, role: str) -> None:
            nonlocal cursor
            count = len(connector.waypoints)
            span = WaypointSpan(cursor, cursor + count - 1)
            segments.append(
                ConnectorSpanRecord(
                    span=span,
                    role=role,
                    from_request_id=connector.from_request_id,
                    to_request_id=connector.to_request_id,
                    algorithm=connector.path.algorithm,
                    length_m=connector.path.length_m,
                )
            )
            cursor = span.end_index

        def add_route(route_index: int) -> None:
            nonlocal cursor
            oriented = route.oriented_routes[route_index]
            count = len(oriented.waypoints)
            span = WaypointSpan(cursor, cursor + count - 1)
            source = oriented.source_route
            segments.append(
                CoverageRouteSpanRecord(
                    span=span,
                    request_id=source.request_id,
                    component_id=source.component_id,
                    source_region_id=source.source_region_id,
                    direction=oriented.direction,
                    response_message=source.response_message,
                    length_m=oriented.route_length_m,
                )
            )
            cursor = span.end_index

        assert route.reference_connector is not None
        add_connector(route.reference_connector, REFERENCE_CONNECTOR_ROLE)
        add_route(0)
        for index, connector in enumerate(route.inter_route_connectors, start=1):
            add_connector(connector, INTER_ROUTE_CONNECTOR_ROLE)
            add_route(index)
        if route.return_connector is not None:
            add_connector(route.return_connector, RETURN_CONNECTOR_ROLE)

        if cursor != len(route.waypoints) - 1:
            raise CompleteVehicleRouteRecordError(
                "assembled segment spans do not cover the source route waypoints"
            )
        return cls(
            vehicle_id=route.vehicle_id,
            frame=route.frame,
            reference_type=reference.reference_type,
            reference_position=reference.position,
            algorithm=route.algorithm,
            return_to_reference=route.return_to_reference,
            segments=tuple(segments),
            waypoints=route.waypoints,
        )

    def to_summary_dict(self) -> dict[str, Any]:
        return {
            "vehicle_id": self.vehicle_id,
            "reference_type": self.reference_type,
            "algorithm": self.algorithm,
            "return_to_reference": self.return_to_reference,
            "is_idle": self.is_idle,
            "route_count": len(self.route_segments),
            "connector_count": len(self.connector_segments),
            "route_directions": list(self.route_directions),
            "total_route_length_m": self.total_route_length_m,
            "total_connector_length_m": self.total_connector_length_m,
            "total_path_length_m": self.total_path_length_m,
            "waypoint_count": len(self.waypoints),
            "component_ids": [segment.component_id for segment in self.route_segments],
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": COMPLETE_VEHICLE_ROUTE_SCHEMA_VERSION,
            "vehicle_id": self.vehicle_id,
            "frame": self.frame.to_dict(),
            "reference": {
                "reference_type": self.reference_type,
                "position_local_m": self.reference_position.to_dict(),
            },
            "algorithm": self.algorithm,
            "return_to_reference": self.return_to_reference,
            "segments": [segment.to_dict() for segment in self.segments],
            "waypoints_local_m": [point.to_dict() for point in self.waypoints],
        }

    @classmethod
    def from_dict(cls, value: Any) -> "CompleteVehicleRouteRecord":
        mapping = _strict_mapping(value, "root")
        _strict_keys(
            mapping,
            {
                "schema_version",
                "vehicle_id",
                "frame",
                "reference",
                "algorithm",
                "return_to_reference",
                "segments",
                "waypoints_local_m",
            },
            "root",
        )
        if mapping["schema_version"] != COMPLETE_VEHICLE_ROUTE_SCHEMA_VERSION:
            raise CompleteVehicleRouteRecordError(
                f"unsupported schema_version: {mapping['schema_version']!r}"
            )
        try:
            frame = LocalCartesianFrame.from_dict(mapping["frame"])
        except PreparedComponentError as exc:
            raise CompleteVehicleRouteRecordError(f"frame is invalid: {exc}") from exc

        reference = _strict_mapping(mapping["reference"], "reference")
        _strict_keys(
            reference,
            {"reference_type", "position_local_m"},
            "reference",
        )
        position = _strict_mapping(
            reference["position_local_m"],
            "reference.position_local_m",
        )
        _strict_keys(
            position,
            {"x_m", "y_m"},
            "reference.position_local_m",
        )
        try:
            local_reference = LocalPoint2D(position["x_m"], position["y_m"])
        except PlanningRequestError as exc:
            raise CompleteVehicleRouteRecordError(
                f"reference.position_local_m is invalid: {exc}"
            ) from exc

        raw_segments = mapping["segments"]
        if not isinstance(raw_segments, list):
            raise CompleteVehicleRouteRecordError("segments must be an array")
        segments = tuple(
            _segment_from_dict(raw, f"segments[{index}]")
            for index, raw in enumerate(raw_segments)
        )

        raw_waypoints = mapping["waypoints_local_m"]
        if not isinstance(raw_waypoints, list):
            raise CompleteVehicleRouteRecordError(
                "waypoints_local_m must be an array"
            )
        waypoints: list[CoverageWaypoint] = []
        for index, raw in enumerate(raw_waypoints):
            point = _strict_mapping(raw, f"waypoints_local_m[{index}]")
            _strict_keys(
                point,
                {"x_m", "y_m", "z_m"},
                f"waypoints_local_m[{index}]",
            )
            try:
                waypoints.append(
                    CoverageWaypoint(point["x_m"], point["y_m"], point["z_m"])
                )
            except PlanningResultError as exc:
                raise CompleteVehicleRouteRecordError(
                    f"waypoints_local_m[{index}] is invalid: {exc}"
                ) from exc

        return cls(
            vehicle_id=mapping["vehicle_id"],
            frame=frame,
            reference_type=reference["reference_type"],
            reference_position=local_reference,
            algorithm=mapping["algorithm"],
            return_to_reference=mapping["return_to_reference"],
            segments=segments,
            waypoints=tuple(waypoints),
        )

    def to_json(self) -> str:
        return json.dumps(
            self.to_dict(),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        ) + "\n"

    @classmethod
    def from_json(cls, text: str) -> "CompleteVehicleRouteRecord":
        if not isinstance(text, str):
            raise CompleteVehicleRouteRecordError("JSON input must be text")
        try:
            value = json.loads(text)
        except json.JSONDecodeError as exc:
            raise CompleteVehicleRouteRecordError(
                f"invalid JSON at line {exc.lineno}, "
                f"column {exc.colno}: {exc.msg}"
            ) from exc
        return cls.from_dict(value)

    @property
    def filename(self) -> str:
        return f"{self.vehicle_id}.complete-route.json"

    def write(self, path: Path | str) -> Path:
        destination = Path(path)
        if destination.exists() and destination.is_dir():
            destination = destination / self.filename
        elif not destination.suffix:
            destination = destination / self.filename
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(
            f".{destination.name}.{os.getpid()}.tmp"
        )
        try:
            temporary.write_text(self.to_json(), encoding="utf-8")
            os.replace(temporary, destination)
        finally:
            if temporary.exists():
                temporary.unlink()
        return destination

    @classmethod
    def read(cls, path: Path | str) -> "CompleteVehicleRouteRecord":
        try:
            text = Path(path).read_text(encoding="utf-8")
        except OSError as exc:
            raise CompleteVehicleRouteRecordError(
                f"could not read complete vehicle route record: {exc}"
            ) from exc
        return cls.from_json(text)


def make_complete_vehicle_route_records(
    routes: Iterable[CompleteVehicleRoute],
) -> tuple[CompleteVehicleRouteRecord, ...]:
    """Convert every vehicle route exactly once and return records by vehicle ID."""
    try:
        values = tuple(routes)
    except TypeError as exc:
        raise CompleteVehicleRouteRecordError("routes must be iterable") from exc
    if not values:
        raise CompleteVehicleRouteRecordError("routes must not be empty")
    for index, route in enumerate(values):
        if not isinstance(route, CompleteVehicleRoute):
            raise CompleteVehicleRouteRecordError(
                f"routes[{index}] must be a CompleteVehicleRoute"
            )
    vehicle_ids = [route.vehicle_id for route in values]
    if len(vehicle_ids) != len(set(vehicle_ids)):
        raise CompleteVehicleRouteRecordError(
            "complete vehicle route IDs must be unique"
        )
    return tuple(
        CompleteVehicleRouteRecord.from_complete_route(route)
        for route in sorted(values, key=lambda item: item.vehicle_id)
    )
