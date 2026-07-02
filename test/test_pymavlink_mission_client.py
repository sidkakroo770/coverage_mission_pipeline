from types import SimpleNamespace

import pytest

from coverage_mission_pipeline.fleet_config import FleetUploadConfig
from coverage_mission_pipeline.mission_protocol import MissionUploadError
from coverage_mission_pipeline.pymavlink_mission_client import PymavlinkMissionClient


class FakeHeartbeat:
    def __init__(self, state, *, armed=False, system_id=1, component_id=1):
        self.system_status = state
        self.base_mode = 128 if armed else 0
        self.autopilot = 3
        self.type = 2
        self._system_id = system_id
        self._component_id = component_id

    def get_srcSystem(self):
        return self._system_id

    def get_srcComponent(self):
        return self._component_id

    def get_type(self):
        return "HEARTBEAT"


class FakeConnection:
    def __init__(self, first, following):
        self.first = first
        self.following = list(following)
        self.recv_calls = 0
        self.closed = False
        self.mav = SimpleNamespace()

    def wait_heartbeat(self, timeout):
        return self.first

    def recv_match(self, *, type=None, blocking=False, timeout=None):
        self.recv_calls += 1
        if not self.following:
            return None
        return self.following.pop(0)

    def close(self):
        self.closed = True


class FakeMavlink:
    MAV_STATE_BOOT = 1
    MAV_STATE_CALIBRATING = 2
    MAV_STATE_STANDBY = 3
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_MISSION_ACCEPTED = 0
    MAV_MISSION_TYPE_MISSION = 0
    enums = {
        "MAV_STATE": {
            1: SimpleNamespace(name="MAV_STATE_BOOT"),
            2: SimpleNamespace(name="MAV_STATE_CALIBRATING"),
            3: SimpleNamespace(name="MAV_STATE_STANDBY"),
        },
        "MAV_MISSION_RESULT": {
            0: SimpleNamespace(name="MAV_MISSION_ACCEPTED"),
        },
    }


class FakeMavutil:
    mavlink = FakeMavlink

    def __init__(self, connection):
        self.connection = connection

    def mavlink_connection(self, *args, **kwargs):
        return self.connection


def config(*, ready_heartbeats=3, startup_grace_s=0):
    return FleetUploadConfig.from_dict(
        {
            "schema_version": 2,
            "profile": "sitl",
            "source_system": 250,
            "source_component": 190,
            "allow_duplicate_missions": False,
            "timeouts": {
                "heartbeat_s": 1,
                "ready_s": 2,
                "ready_heartbeats": ready_heartbeats,
                "startup_grace_s": startup_grace_s,
                "clear_ack_s": 1,
                "item_request_s": 1,
                "upload_ack_s": 1,
                "download_item_s": 1,
                "retries": 1,
            },
            "vehicles": [
                {
                    "vehicle_id": "drone-1",
                    "endpoint": "tcp:127.0.0.1:5760",
                    "system_id": 1,
                    "component_id": 1,
                }
            ],
        }
    )


def make_client(connection, fleet=None):
    fleet = fleet or config()
    client = PymavlinkMissionClient(fleet.vehicles[0], fleet)
    fake_mavutil = FakeMavutil(connection)
    client._import_pymavlink = lambda: fake_mavutil
    return client


def test_connect_waits_beyond_first_heartbeat_until_stable_standby():
    connection = FakeConnection(
        FakeHeartbeat(FakeMavlink.MAV_STATE_BOOT),
        [
            FakeHeartbeat(FakeMavlink.MAV_STATE_STANDBY),
            FakeHeartbeat(FakeMavlink.MAV_STATE_STANDBY),
            FakeHeartbeat(FakeMavlink.MAV_STATE_STANDBY),
        ],
    )
    identity = make_client(connection).connect()
    assert identity.system_id == 1
    assert connection.recv_calls == 3
    assert connection.closed is False


def test_transitional_heartbeat_resets_readiness_streak():
    connection = FakeConnection(
        FakeHeartbeat(FakeMavlink.MAV_STATE_BOOT),
        [
            FakeHeartbeat(FakeMavlink.MAV_STATE_STANDBY),
            FakeHeartbeat(FakeMavlink.MAV_STATE_CALIBRATING),
            FakeHeartbeat(FakeMavlink.MAV_STATE_STANDBY),
            FakeHeartbeat(FakeMavlink.MAV_STATE_STANDBY),
            FakeHeartbeat(FakeMavlink.MAV_STATE_STANDBY),
        ],
    )
    make_client(connection).connect()
    assert connection.recv_calls == 5


def test_connect_refuses_armed_vehicle_during_readiness_wait():
    connection = FakeConnection(
        FakeHeartbeat(FakeMavlink.MAV_STATE_BOOT),
        [FakeHeartbeat(FakeMavlink.MAV_STATE_STANDBY, armed=True)],
    )
    with pytest.raises(MissionUploadError, match="became armed"):
        make_client(connection).connect()
    assert connection.closed is True


class FakeMissionMessage:
    def __init__(self, message_type, *, seq=0, result=0):
        self._message_type = message_type
        self.seq = seq
        self.type = result

    def get_type(self):
        return self._message_type


def test_legacy_mission_request_is_answered_with_mission_item_int():
    fleet = config(ready_heartbeats=1)
    client = PymavlinkMissionClient(fleet.vehicles[0], fleet)
    client._mavutil = FakeMavutil(None)
    client._identity = SimpleNamespace(system_id=1, component_id=1)
    replies = iter(
        [
            FakeMissionMessage("MISSION_REQUEST", seq=0),
            FakeMissionMessage("MISSION_ACK", result=0),
        ]
    )
    client._drain = lambda: None
    client._send_count = lambda count: None
    client._recv = lambda types, timeout: next(replies)
    sent_int = []
    sent_float = []
    client._send_item_int = lambda record: sent_int.append(record.seq)
    client._send_item_float = lambda record: sent_float.append(record.seq)

    from coverage_mission_pipeline.mission_fingerprint import MissionItemRecord

    client.upload_mission(
        (
            MissionItemRecord(
                seq=0,
                frame=0,
                command=16,
                autocontinue=1,
                param1=0.0,
                param2=0.0,
                param3=0.0,
                param4=0.0,
                x=286133972,
                y=774211727,
                z=0.0,
            ),
        )
    )

    assert sent_int == [0]
    assert sent_float == []
