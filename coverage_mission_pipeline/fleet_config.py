#!/usr/bin/env python3
"""Strict fleet connection configuration for transactional MAVLink uploads."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping

import yaml

FLEET_CONFIG_SCHEMA_VERSION = 2
_PROFILE_VALUES = frozenset({"sitl", "hardware"})
_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class FleetConfigError(ValueError):
    """Raised when a fleet configuration is malformed or ambiguous."""


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise FleetConfigError(f"{path} must be an object")
    return value


def _strict_keys(
    value: Mapping[str, Any],
    *,
    required: set[str],
    optional: set[str] = set(),
    path: str,
) -> None:
    actual = set(value.keys())
    missing = sorted(required - actual)
    unknown = sorted(actual - required - optional)
    if missing:
        raise FleetConfigError(
            f"{path} is missing required field(s): {', '.join(missing)}"
        )
    if unknown:
        raise FleetConfigError(
            f"{path} contains unknown field(s): {', '.join(unknown)}"
        )


def _identifier(value: Any, path: str) -> str:
    if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
        raise FleetConfigError(f"{path} must match {_ID_PATTERN.pattern!r}")
    return value


def _nonempty_string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise FleetConfigError(f"{path} must be a non-empty string")
    return value.strip()


def _positive_int(value: Any, path: str, *, maximum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise FleetConfigError(f"{path} must be a positive integer")
    if maximum is not None and value > maximum:
        raise FleetConfigError(f"{path} must be <= {maximum}")
    return value


def _positive_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FleetConfigError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise FleetConfigError(f"{path} must be finite and greater than zero")
    return result


def _nonnegative_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FleetConfigError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise FleetConfigError(f"{path} must be finite and non-negative")
    return result


@dataclass(frozen=True)
class FleetTimeouts:
    heartbeat_s: float = 15.0
    ready_s: float = 120.0
    ready_heartbeats: int = 3
    startup_grace_s: float = 5.0
    clear_ack_s: float = 10.0
    item_request_s: float = 10.0
    upload_ack_s: float = 15.0
    download_item_s: float = 10.0
    retries: int = 3

    def __post_init__(self) -> None:
        for name in (
            "heartbeat_s",
            "ready_s",
            "clear_ack_s",
            "item_request_s",
            "upload_ack_s",
            "download_item_s",
        ):
            object.__setattr__(
                self,
                name,
                _positive_number(getattr(self, name), f"timeouts.{name}"),
            )
        object.__setattr__(
            self,
            "startup_grace_s",
            _nonnegative_number(
                self.startup_grace_s,
                "timeouts.startup_grace_s",
            ),
        )
        object.__setattr__(
            self,
            "ready_heartbeats",
            _positive_int(
                self.ready_heartbeats,
                "timeouts.ready_heartbeats",
                maximum=20,
            ),
        )
        object.__setattr__(
            self,
            "retries",
            _positive_int(self.retries, "timeouts.retries", maximum=20),
        )

    @classmethod
    def from_dict(cls, value: Any) -> "FleetTimeouts":
        mapping = _mapping(value, "timeouts")
        _strict_keys(
            mapping,
            required={
                "heartbeat_s",
                "ready_s",
                "ready_heartbeats",
                "startup_grace_s",
                "clear_ack_s",
                "item_request_s",
                "upload_ack_s",
                "download_item_s",
                "retries",
            },
            path="timeouts",
        )
        return cls(**mapping)

    def to_dict(self) -> dict[str, Any]:
        return {
            "heartbeat_s": self.heartbeat_s,
            "ready_s": self.ready_s,
            "ready_heartbeats": self.ready_heartbeats,
            "startup_grace_s": self.startup_grace_s,
            "clear_ack_s": self.clear_ack_s,
            "item_request_s": self.item_request_s,
            "upload_ack_s": self.upload_ack_s,
            "download_item_s": self.download_item_s,
            "retries": self.retries,
        }


@dataclass(frozen=True)
class FleetVehicleConfig:
    vehicle_id: str
    endpoint: str
    system_id: int
    component_id: int = 1

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "vehicle_id",
            _identifier(self.vehicle_id, "vehicle.vehicle_id"),
        )
        object.__setattr__(
            self,
            "endpoint",
            _nonempty_string(self.endpoint, "vehicle.endpoint"),
        )
        object.__setattr__(
            self,
            "system_id",
            _positive_int(self.system_id, "vehicle.system_id", maximum=255),
        )
        object.__setattr__(
            self,
            "component_id",
            _positive_int(
                self.component_id,
                "vehicle.component_id",
                maximum=255,
            ),
        )

    @classmethod
    def from_dict(cls, value: Any, path: str) -> "FleetVehicleConfig":
        mapping = _mapping(value, path)
        _strict_keys(
            mapping,
            required={"vehicle_id", "endpoint", "system_id"},
            optional={"component_id"},
            path=path,
        )
        try:
            return cls(**mapping)
        except FleetConfigError as exc:
            raise FleetConfigError(f"{path} is invalid: {exc}") from exc

    def to_dict(self) -> dict[str, Any]:
        return {
            "vehicle_id": self.vehicle_id,
            "endpoint": self.endpoint,
            "system_id": self.system_id,
            "component_id": self.component_id,
        }


@dataclass(frozen=True)
class FleetUploadConfig:
    profile: str
    source_system: int
    source_component: int
    allow_duplicate_missions: bool
    timeouts: FleetTimeouts
    vehicles: tuple[FleetVehicleConfig, ...]

    def __post_init__(self) -> None:
        if self.profile not in _PROFILE_VALUES:
            raise FleetConfigError(
                f"profile must be one of: {', '.join(sorted(_PROFILE_VALUES))}"
            )
        object.__setattr__(
            self,
            "source_system",
            _positive_int(self.source_system, "source_system", maximum=255),
        )
        object.__setattr__(
            self,
            "source_component",
            _positive_int(self.source_component, "source_component", maximum=255),
        )
        if not isinstance(self.allow_duplicate_missions, bool):
            raise FleetConfigError("allow_duplicate_missions must be a bool")
        if not isinstance(self.timeouts, FleetTimeouts):
            raise FleetConfigError("timeouts must be FleetTimeouts")
        try:
            vehicles = tuple(self.vehicles)
        except TypeError as exc:
            raise FleetConfigError("vehicles must be an array") from exc
        if not vehicles:
            raise FleetConfigError("vehicles must not be empty")
        if any(not isinstance(vehicle, FleetVehicleConfig) for vehicle in vehicles):
            raise FleetConfigError("vehicles contains an invalid entry")
        object.__setattr__(self, "vehicles", vehicles)

        def _duplicates(values: list[Any]) -> list[Any]:
            seen: set[Any] = set()
            duplicates: set[Any] = set()
            for item in values:
                if item in seen:
                    duplicates.add(item)
                seen.add(item)
            return sorted(duplicates)

        checks = {
            "vehicle_id": [vehicle.vehicle_id for vehicle in vehicles],
            "endpoint": [vehicle.endpoint for vehicle in vehicles],
            "system_id": [vehicle.system_id for vehicle in vehicles],
        }
        for label, values in checks.items():
            duplicates = _duplicates(values)
            if duplicates:
                raise FleetConfigError(
                    f"vehicles contains duplicate {label}(s): "
                    + ", ".join(str(item) for item in duplicates)
                )

    @classmethod
    def from_dict(cls, value: Any) -> "FleetUploadConfig":
        mapping = _mapping(value, "root")
        _strict_keys(
            mapping,
            required={
                "schema_version",
                "profile",
                "source_system",
                "source_component",
                "allow_duplicate_missions",
                "timeouts",
                "vehicles",
            },
            path="root",
        )
        if mapping["schema_version"] != FLEET_CONFIG_SCHEMA_VERSION:
            raise FleetConfigError(
                f"unsupported schema_version: {mapping['schema_version']!r}"
            )
        raw_vehicles = mapping["vehicles"]
        if not isinstance(raw_vehicles, list):
            raise FleetConfigError("vehicles must be an array")
        return cls(
            profile=mapping["profile"],
            source_system=mapping["source_system"],
            source_component=mapping["source_component"],
            allow_duplicate_missions=mapping["allow_duplicate_missions"],
            timeouts=FleetTimeouts.from_dict(mapping["timeouts"]),
            vehicles=tuple(
                FleetVehicleConfig.from_dict(item, f"vehicles[{index}]")
                for index, item in enumerate(raw_vehicles)
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": FLEET_CONFIG_SCHEMA_VERSION,
            "profile": self.profile,
            "source_system": self.source_system,
            "source_component": self.source_component,
            "allow_duplicate_missions": self.allow_duplicate_missions,
            "timeouts": self.timeouts.to_dict(),
            "vehicles": [vehicle.to_dict() for vehicle in self.vehicles],
        }

    def to_json(self) -> str:
        return json.dumps(
            self.to_dict(),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        ) + "\n"


def load_fleet_upload_config(path: Path | str) -> FleetUploadConfig:
    source = Path(path)
    try:
        text = source.read_text(encoding="utf-8")
    except OSError as exc:
        raise FleetConfigError(f"could not read fleet configuration: {exc}") from exc

    try:
        if source.suffix.lower() == ".json":
            value = json.loads(text)
        elif source.suffix.lower() in {".yaml", ".yml"}:
            value = yaml.safe_load(text)
        else:
            raise FleetConfigError(
                "fleet configuration filename must end in .json, .yaml or .yml"
            )
    except (json.JSONDecodeError, yaml.YAMLError) as exc:
        raise FleetConfigError(f"could not parse fleet configuration: {exc}") from exc

    return FleetUploadConfig.from_dict(value)
