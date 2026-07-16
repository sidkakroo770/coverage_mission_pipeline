from __future__ import annotations

import time

import pytest

from coverage_mission_pipeline.variable_fleet_runner import (
    FleetExecutionOptions,
    VehicleState,
    VariableFleetExecutionError,
    launch_gate_status,
    update_pairwise_separation,
)


def _state(
    vehicle_id: str,
    *,
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
) -> VehicleState:
    state = VehicleState(
        vehicle_id=vehicle_id,
        endpoint="tcp:127.0.0.1:5760",
        system_id=1,
        component_id=1,
        expected_wire_item_count=5,
        connection=object(),
    )
    now = time.monotonic()
    state.home = (30.0, 76.0)
    state.position = (latitude_deg, longitude_deg, altitude_m)
    state.armed = True
    state.launched = True
    state.last_heartbeat_s = now
    state.last_position_s = now
    return state


def test_execution_options_reject_invalid_values() -> None:
    with pytest.raises(ValueError):
        FleetExecutionOptions(fleet_timeout_s=0.0)


def test_first_vehicle_launch_gate_passes() -> None:
    passed, reason = launch_gate_status([], FleetExecutionOptions())
    assert passed is True
    assert reason == "first vehicle"


def test_launch_gate_requires_previous_vehicle_clear() -> None:
    options = FleetExecutionOptions(
        launch_clearance_altitude_m=12.0,
        launch_clearance_home_distance_m=30.0,
    )
    state = _state(
        "drone-1",
        latitude_deg=30.0004,
        longitude_deg=76.0,
        altitude_m=20.0,
    )
    passed, _ = launch_gate_status(
        [state],
        options,
        now_s=time.monotonic(),
    )
    assert passed is True

    state.position = (30.00001, 76.0, 20.0)
    passed, reason = launch_gate_status(
        [state],
        options,
        now_s=time.monotonic(),
    )
    assert passed is False
    assert "from HOME" in reason


def test_launch_gate_rejects_completed_predecessor() -> None:
    state = _state(
        "drone-1",
        latitude_deg=30.0004,
        longitude_deg=76.0,
        altitude_m=20.0,
    )
    state.armed = False
    state.completed = True
    passed, reason = launch_gate_status(
        [state],
        FleetExecutionOptions(),
        now_s=time.monotonic(),
    )
    assert passed is False
    assert "before all" in reason


def test_pairwise_separation_tracks_minimum() -> None:
    left = _state(
        "drone-1",
        latitude_deg=30.0,
        longitude_deg=76.0,
        altitude_m=20.0,
    )
    right = _state(
        "drone-2",
        latitude_deg=30.00005,
        longitude_deg=76.0,
        altitude_m=22.0,
    )
    separation = {
        "overall_minimum": None,
        "pairs": {},
        "warning_count": 0,
        "printed_warnings": set(),
    }
    update_pairwise_separation(
        [left, right],
        time.monotonic(),
        separation,
        FleetExecutionOptions(),
    )
    assert "drone-1|drone-2" in separation["pairs"]
    assert separation["overall_minimum"]["minimum_3d_separation_m"] > 0.0
