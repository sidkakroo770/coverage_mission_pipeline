from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import ModuleType



class _MavlinkConstants:
    """Minimal lazy MAVLink constant provider for importing the runner."""

    def __getattr__(self, name: str) -> int:
        return 0


def _install_pymavlink_stub() -> None:
    """Install only the interface required while importing the runner."""

    package = ModuleType("pymavlink")
    mavutil = ModuleType("pymavlink.mavutil")
    mavutil.mavlink = _MavlinkConstants()

    package.mavutil = mavutil

    sys.modules["pymavlink"] = package
    sys.modules["pymavlink.mavutil"] = mavutil


RUNNER_PATH = (
    Path(__file__).resolve().parents[1]
    / "demo/noida_stage21/scripts/stage21c_run_fleet.py"
)


def _load_runner():
    _install_pymavlink_stub()

    spec = importlib.util.spec_from_file_location(
        "stage21c_run_fleet_sequence_test",
        RUNNER_PATH,
    )
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class _MissionCurrent:
    def __init__(self, system_id: int, sequence: int) -> None:
        self._system_id = system_id
        self.seq = sequence

    def get_srcSystem(self) -> int:
        return self._system_id


class _FakeMav:
    def __init__(self, connection) -> None:
        self._connection = connection
        self.requests: list[tuple] = []

    def mission_set_current_send(self, *args) -> None:
        self.requests.append(args)

        # Reproduce the observed race: another old seq=0 update arrives after
        # the request, followed by ArduPilot's seq=1 acknowledgement.
        if len(self.requests) == 1:
            self._connection.incoming.extend(
                [
                    _MissionCurrent(1, 0),
                    _MissionCurrent(1, 1),
                ]
            )


class _FakeConnection:
    def __init__(self) -> None:
        # One seq=0 packet was queued before the request.
        self.incoming = [_MissionCurrent(1, 0)]
        self.mav = _FakeMav(self)

    def recv_match(
        self,
        *,
        type,
        blocking,
        timeout=None,
    ):
        assert type == "MISSION_CURRENT"

        if self.incoming:
            return self.incoming.pop(0)

        return None


def test_set_current_ignores_stale_lower_sequence(monkeypatch) -> None:
    runner = _load_runner()

    clock = {"value": 100.0}

    def monotonic() -> float:
        clock["value"] += 0.01
        return clock["value"]

    monkeypatch.setattr(runner.time, "monotonic", monotonic)
    monkeypatch.setattr(runner, "COMMAND_TIMEOUT_S", 2.0)

    connection = _FakeConnection()
    state = {
        "vehicle_id": "drone-2",
        "system_id": 1,
        "component_id": 1,
        "connection": connection,
        "current_sequence": 0,
        "highest_sequence": 0,
    }

    observed = runner.set_current_mission_item(state, 1)

    assert observed == 1
    assert state["current_sequence"] == 1
    assert state["highest_sequence"] == 1
    assert len(connection.mav.requests) == 1


def test_set_current_times_out_if_requested_sequence_never_arrives(
    monkeypatch,
) -> None:
    runner = _load_runner()

    clock = {"value": 200.0}

    def monotonic() -> float:
        clock["value"] += 0.30
        return clock["value"]

    monkeypatch.setattr(runner.time, "monotonic", monotonic)
    monkeypatch.setattr(runner, "COMMAND_TIMEOUT_S", 1.0)

    connection = _FakeConnection()

    def only_stale(*args) -> None:
        connection.mav.requests.append(args)
        connection.incoming.append(_MissionCurrent(1, 0))

    connection.mav.mission_set_current_send = only_stale

    state = {
        "vehicle_id": "drone-2",
        "system_id": 1,
        "component_id": 1,
        "connection": connection,
        "current_sequence": 0,
        "highest_sequence": 0,
    }

    try:
        runner.set_current_mission_item(state, 1)
    except runner.FleetExecutionError as exc:
        assert "requesting sequence 1" in str(exc)
        assert "last observed 0" in str(exc)
    else:
        raise AssertionError("expected FleetExecutionError")
