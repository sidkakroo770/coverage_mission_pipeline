from coverage_mission_pipeline.ardupilot_mission import (
    ArduPilotMission,
    ArduPilotMissionItem,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
)
from coverage_mission_pipeline.mission_fingerprint import (
    MissionItemRecord,
    compare_mission_records,
    mission_fingerprint,
    records_from_ardupilot_mission,
    semantic_records_from_wire_readback,
    wire_records_from_semantic,
)


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


def mission():
    return ArduPilotMission(
        vehicle_id="drone-1",
        items=(
            item(0, MAV_CMD_NAV_TAKEOFF, 28.6133972178, 77.4211727161, 20.0),
            item(1, MAV_CMD_NAV_WAYPOINT, 28.614, 77.422, 20.0),
            item(2, MAV_CMD_NAV_RETURN_TO_LAUNCH),
        ),
    )


def test_expected_mission_is_quantized_to_mavlink_precision():
    records = records_from_ardupilot_mission(mission())
    assert records[0].x == 286133972
    assert records[0].y == 774211727
    assert len(mission_fingerprint(records)) == 64


def test_int_frame_variant_is_fingerprint_equivalent():
    expected = records_from_ardupilot_mission(mission())
    readback = tuple(
        MissionItemRecord(
            seq=item.seq,
            frame=6 if item.frame == 3 else item.frame,
            command=item.command,
            autocontinue=item.autocontinue,
            param1=item.param1,
            param2=item.param2,
            param3=item.param3,
            param4=item.param4,
            x=item.x,
            y=item.y,
            z=item.z,
        )
        for item in expected
    )
    assert compare_mission_records(expected, readback) == ()
    assert mission_fingerprint(expected) == mission_fingerprint(readback)


def test_one_coordinate_unit_difference_is_detected():
    expected = records_from_ardupilot_mission(mission())
    changed = list(expected)
    original = changed[1]
    changed[1] = MissionItemRecord(
        seq=original.seq,
        frame=original.frame,
        command=original.command,
        autocontinue=original.autocontinue,
        param1=original.param1,
        param2=original.param2,
        param3=original.param3,
        param4=original.param4,
        x=original.x + 1,
        y=original.y,
        z=original.z,
    )
    differences = compare_mission_records(expected, tuple(changed))
    assert any("x_e7" in difference for difference in differences)
    assert mission_fingerprint(expected) != mission_fingerprint(changed)


def test_wire_records_reserve_home_and_shift_semantic_takeoff():
    semantic = records_from_ardupilot_mission(mission())
    wire = wire_records_from_semantic(semantic)
    assert len(wire) == len(semantic) + 1
    assert wire[0].seq == 0
    assert wire[0].frame == 0
    assert wire[0].command == MAV_CMD_NAV_WAYPOINT
    assert wire[1].seq == 1
    assert wire[1].command == MAV_CMD_NAV_TAKEOFF
    assert wire[-1].command == MAV_CMD_NAV_RETURN_TO_LAUNCH


def test_dynamic_ardupilot_home_is_excluded_from_semantic_fingerprint():
    semantic = records_from_ardupilot_mission(mission())
    wire = list(wire_records_from_semantic(semantic))
    home = wire[0]
    wire[0] = MissionItemRecord(
        seq=0,
        frame=0,
        command=MAV_CMD_NAV_WAYPOINT,
        autocontinue=home.autocontinue,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        x=0,
        y=0,
        z=0.0,
    )
    readback = semantic_records_from_wire_readback(tuple(wire))
    assert compare_mission_records(semantic, readback) == ()
    assert mission_fingerprint(semantic) == mission_fingerprint(readback)


def test_wire_readback_rejects_missing_home_slot():
    semantic = records_from_ardupilot_mission(mission())
    try:
        semantic_records_from_wire_readback(semantic)
    except Exception as exc:
        assert "HOME" in str(exc)
    else:
        raise AssertionError("missing HOME slot was accepted")



def test_rtl_global_frame_readback_is_semantically_equivalent():
    expected = records_from_ardupilot_mission(mission())
    readback = list(expected)

    rtl = readback[-1]
    assert rtl.command == MAV_CMD_NAV_RETURN_TO_LAUNCH
    assert rtl.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT

    readback[-1] = MissionItemRecord(
        seq=rtl.seq,
        frame=0,
        command=rtl.command,
        autocontinue=rtl.autocontinue,
        param1=rtl.param1,
        param2=rtl.param2,
        param3=rtl.param3,
        param4=rtl.param4,
        x=rtl.x,
        y=rtl.y,
        z=rtl.z,
    )

    assert compare_mission_records(expected, tuple(readback)) == ()
    assert mission_fingerprint(expected) == mission_fingerprint(
        tuple(readback)
    )


def test_positional_waypoint_frame_difference_is_still_detected():
    expected = records_from_ardupilot_mission(mission())
    changed = list(expected)

    waypoint = changed[1]
    assert waypoint.command == MAV_CMD_NAV_WAYPOINT
    assert waypoint.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT

    changed[1] = MissionItemRecord(
        seq=waypoint.seq,
        frame=0,
        command=waypoint.command,
        autocontinue=waypoint.autocontinue,
        param1=waypoint.param1,
        param2=waypoint.param2,
        param3=waypoint.param3,
        param4=waypoint.param4,
        x=waypoint.x,
        y=waypoint.y,
        z=waypoint.z,
    )

    differences = compare_mission_records(expected, tuple(changed))

    assert any(
        "item 1 field frame differs" in difference
        for difference in differences
    )
    assert mission_fingerprint(expected) != mission_fingerprint(
        tuple(changed)
    )

def _record_with_param4(
    command: int,
    param4: float,
) -> MissionItemRecord:
    return MissionItemRecord(
        seq=0,
        frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=command,
        autocontinue=1,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=param4,
        x=286133972,
        y=774211727,
        z=0.0,
    )


def test_land_default_param4_readback_one_is_equivalent():
    expected = (
        _record_with_param4(
            MAV_CMD_NAV_LAND,
            0.0,
        ),
    )

    readback = (
        _record_with_param4(
            MAV_CMD_NAV_LAND,
            1.0,
        ),
    )

    assert compare_mission_records(
        expected,
        readback,
    ) == ()

    assert (
        mission_fingerprint(expected)
        == mission_fingerprint(readback)
    )


def test_land_nondefault_param4_difference_is_detected():
    expected = (
        _record_with_param4(
            MAV_CMD_NAV_LAND,
            0.0,
        ),
    )

    changed = (
        _record_with_param4(
            MAV_CMD_NAV_LAND,
            2.0,
        ),
    )

    differences = compare_mission_records(
        expected,
        changed,
    )

    assert any(
        "param4_f32" in difference
        for difference in differences
    )

    assert (
        mission_fingerprint(expected)
        != mission_fingerprint(changed)
    )


def test_waypoint_param4_difference_remains_detected():
    expected = (
        _record_with_param4(
            MAV_CMD_NAV_WAYPOINT,
            0.0,
        ),
    )

    changed = (
        _record_with_param4(
            MAV_CMD_NAV_WAYPOINT,
            1.0,
        ),
    )

    differences = compare_mission_records(
        expected,
        changed,
    )

    assert any(
        "param4_f32" in difference
        for difference in differences
    )

    assert (
        mission_fingerprint(expected)
        != mission_fingerprint(changed)
    )
