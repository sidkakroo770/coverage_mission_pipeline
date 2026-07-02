from pathlib import Path

import pytest

from coverage_mission_pipeline.ardupilot_mission import (
    ArduPilotMission,
    ArduPilotMissionItem,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
)
from coverage_mission_pipeline.fleet_config import FleetUploadConfig
from coverage_mission_pipeline.mission_fingerprint import MissionItemRecord
from coverage_mission_pipeline.mission_protocol import (
    MissionUploadError,
    VehicleIdentity,
    load_vehicle_missions,
    upload_and_verify_fleet,
)


def config(allow_duplicates=False):
    return FleetUploadConfig.from_dict(
        {
            "schema_version": 2,
            "profile": "sitl",
            "source_system": 250,
            "source_component": 190,
            "allow_duplicate_missions": allow_duplicates,
            "timeouts": {
                "heartbeat_s": 1,
                "ready_s": 2,
                "ready_heartbeats": 1,
                "startup_grace_s": 0,
                "clear_ack_s": 1,
                "item_request_s": 1,
                "upload_ack_s": 1,
                "download_item_s": 1,
                "retries": 1,
            },
            "vehicles": [
                {
                    "vehicle_id": "drone-1",
                    "endpoint": "fake:1",
                    "system_id": 1,
                },
                {
                    "vehicle_id": "drone-2",
                    "endpoint": "fake:2",
                    "system_id": 2,
                },
            ],
        }
    )


def make_mission(
    vehicle_id,
    offset,
    terminal_command=MAV_CMD_NAV_RETURN_TO_LAUNCH,
):
    def item(seq, command, lat=0.0, lon=0.0, alt=0.0):
        return ArduPilotMissionItem(
            seq=seq,
            current=1 if seq == 0 else 0,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=command,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            latitude_deg=lat,
            longitude_deg=lon,
            altitude_m=alt,
            autocontinue=1,
        )

    if terminal_command == MAV_CMD_NAV_LAND:
        terminal = item(
            3,
            MAV_CMD_NAV_LAND,
            28.0,
            77.0,
            0.0,
        )
    else:
        terminal = item(
            3,
            MAV_CMD_NAV_RETURN_TO_LAUNCH,
        )

    return ArduPilotMission(
        vehicle_id=vehicle_id,
        items=(
            item(0, MAV_CMD_NAV_TAKEOFF, 28.0, 77.0, 20.0),
            item(
                1,
                MAV_CMD_NAV_WAYPOINT,
                28.0 + offset,
                77.1,
                20.0,
            ),
            item(
                2,
                MAV_CMD_NAV_WAYPOINT,
                28.0,
                77.0,
                20.0,
            ),
            terminal,
        ),
    )


def write_bundle(root: Path, duplicate=False):
    directory = root / "ardupilot"
    directory.mkdir(parents=True)
    make_mission("drone-1", 0.01).write_json(directory)
    make_mission("drone-2", 0.01 if duplicate else 0.02).write_json(directory)


class FakeClient:
    def __init__(self, vehicle, fleet, state):
        self.vehicle = vehicle
        self.fleet = fleet
        self.state = state
        self.uploaded = ()
        self.closed = False

    def connect(self):
        self.state.append((self.vehicle.vehicle_id, "connect"))
        return VehicleIdentity(self.vehicle.system_id, self.vehicle.component_id)

    def is_armed(self):
        return False

    def clear_mission(self):
        self.state.append((self.vehicle.vehicle_id, "clear"))

    def upload_mission(self, records):
        self.uploaded = records
        self.state.append((self.vehicle.vehicle_id, "upload"))

    def download_mission(self):
        self.state.append((self.vehicle.vehicle_id, "download"))
        return self.uploaded

    def close(self):
        self.closed = True
        self.state.append((self.vehicle.vehicle_id, "close"))


def test_fleet_upload_is_sequential_and_verified(tmp_path: Path):
    write_bundle(tmp_path)
    events = []

    def factory(vehicle, fleet):
        return FakeClient(vehicle, fleet, events)

    report_path = tmp_path / "report.json"
    report = upload_and_verify_fleet(tmp_path, config(), report_path, factory)
    assert report["all_verified"] is True
    assert report["schema_version"] == 2
    assert report["vehicles"][0]["expected_item_count"] == 4
    assert report["vehicles"][0]["expected_wire_item_count"] == 5
    assert report["vehicles"][0]["readback_wire_item_count"] == 5
    assert report_path.is_file()
    assert events == [
        ("drone-1", "connect"),
        ("drone-1", "clear"),
        ("drone-1", "upload"),
        ("drone-1", "download"),
        ("drone-1", "close"),
        ("drone-2", "connect"),
        ("drone-2", "clear"),
        ("drone-2", "upload"),
        ("drone-2", "download"),
        ("drone-2", "close"),
    ]


def test_duplicate_missions_fail_before_any_connection(tmp_path: Path):
    write_bundle(tmp_path, duplicate=True)
    events = []

    def factory(vehicle, fleet):
        events.append((vehicle.vehicle_id, "factory"))
        return FakeClient(vehicle, fleet, events)

    with pytest.raises(MissionUploadError, match="duplicate mission"):
        upload_and_verify_fleet(
            tmp_path,
            config(),
            tmp_path / "report.json",
            factory,
        )
    assert events == []


def test_readback_mismatch_refuses_verification(tmp_path: Path):
    write_bundle(tmp_path)

    class CorruptClient(FakeClient):
        def download_mission(self):
            records = list(self.uploaded)
            item = records[1]
            records[1] = MissionItemRecord(
                seq=item.seq,
                frame=item.frame,
                command=item.command,
                autocontinue=item.autocontinue,
                param1=item.param1,
                param2=item.param2,
                param3=item.param3,
                param4=item.param4,
                x=item.x + 1,
                y=item.y,
                z=item.z,
            )
            return tuple(records)

    with pytest.raises(MissionUploadError, match="readback verification failed"):
        upload_and_verify_fleet(
            tmp_path,
            config(),
            tmp_path / "report.json",
            lambda vehicle, fleet: CorruptClient(vehicle, fleet, []),
        )
    assert not (tmp_path / "report.json").exists()


def test_mission_identity_mismatch_is_rejected(tmp_path: Path):
    directory = tmp_path / "ardupilot"
    directory.mkdir()
    mission = make_mission("drone-X", 0.01)
    (directory / "drone-1.ardupilot-mission.json").write_text(
        mission.to_json(), encoding="utf-8"
    )
    make_mission("drone-2", 0.02).write_json(directory)
    with pytest.raises(MissionUploadError, match="mission identity mismatch"):
        load_vehicle_missions(tmp_path, config())

def test_land_at_reference_missions_are_accepted(tmp_path: Path):
    directory = tmp_path / "ardupilot"
    directory.mkdir()

    make_mission(
        "drone-1",
        0.01,
        MAV_CMD_NAV_LAND,
    ).write_json(directory)

    make_mission(
        "drone-2",
        0.02,
        MAV_CMD_NAV_LAND,
    ).write_json(directory)

    loaded = load_vehicle_missions(tmp_path, config())

    assert len(loaded) == 2
    assert all(
        item.mission.items[-1].command == MAV_CMD_NAV_LAND
        for item in loaded
    )


def test_land_away_from_reference_is_rejected(tmp_path: Path):
    directory = tmp_path / "ardupilot"
    directory.mkdir()

    mission = make_mission(
        "drone-1",
        0.01,
        MAV_CMD_NAV_LAND,
    )
    original = mission.items[-1]

    bad_land = ArduPilotMissionItem(
        seq=original.seq,
        current=original.current,
        frame=original.frame,
        command=original.command,
        param1=original.param1,
        param2=original.param2,
        param3=original.param3,
        param4=original.param4,
        latitude_deg=28.001,
        longitude_deg=original.longitude_deg,
        altitude_m=original.altitude_m,
        autocontinue=original.autocontinue,
    )

    bad_mission = ArduPilotMission(
        vehicle_id="drone-1",
        items=(
            *mission.items[:-1],
            bad_land,
        ),
    )

    bad_mission.write_json(directory)

    make_mission(
        "drone-2",
        0.02,
        MAV_CMD_NAV_LAND,
    ).write_json(directory)

    with pytest.raises(
        MissionUploadError,
        match="LAND coordinates must match",
    ):
        load_vehicle_missions(tmp_path, config())


def test_rtl_without_return_waypoint_is_rejected(tmp_path: Path):
    directory = tmp_path / "ardupilot"
    directory.mkdir()

    safe = make_mission("drone-1", 0.01)
    unsafe = ArduPilotMission(
        vehicle_id="drone-1",
        items=(
            safe.items[0],
            ArduPilotMissionItem(
                seq=1,
                current=0,
                frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=MAV_CMD_NAV_WAYPOINT,
                param1=0.0,
                param2=0.0,
                param3=0.0,
                param4=0.0,
                latitude_deg=28.01,
                longitude_deg=77.1,
                altitude_m=20.0,
                autocontinue=1,
            ),
            ArduPilotMissionItem(
                seq=2,
                current=0,
                frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=MAV_CMD_NAV_RETURN_TO_LAUNCH,
                param1=0.0,
                param2=0.0,
                param3=0.0,
                param4=0.0,
                latitude_deg=0.0,
                longitude_deg=0.0,
                altitude_m=0.0,
                autocontinue=1,
            ),
        ),
    )

    unsafe.write_json(directory)
    make_mission("drone-2", 0.02).write_json(directory)

    with pytest.raises(
        MissionUploadError,
        match="terminal waypoint must match",
    ):
        load_vehicle_missions(tmp_path, config())
