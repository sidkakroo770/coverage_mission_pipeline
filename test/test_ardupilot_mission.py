#!/usr/bin/env python3
"""Tests for the strict ArduPilot mission model and QGC WPL 110 exporter."""

import json
import math
from pathlib import Path

import pytest

from coverage_mission_pipeline.ardupilot_mission import (
    ARDUPILOT_MISSION_SCHEMA_VERSION,
    END_ACTION_LAND_AT_REFERENCE,
    END_ACTION_NONE,
    END_ACTION_RTL,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
    QGC_WPL_110_HEADER,
    ArduPilotMission,
    ArduPilotMissionBuildConfig,
    ArduPilotMissionError,
    ArduPilotMissionItem,
    build_ardupilot_mission,
    build_ardupilot_missions,
)
from coverage_mission_pipeline.complete_vehicle_route_record import (
    ConnectorSpanRecord,
    CoverageRouteSpanRecord,
    CompleteVehicleRouteRecord,
    WaypointSpan,
    REFERENCE_CONNECTOR_ROLE,
    RETURN_CONNECTOR_ROLE,
)
from coverage_mission_pipeline.planning_request import LocalPoint2D
from coverage_mission_pipeline.planning_result import CoverageWaypoint
from coverage_mission_pipeline.prepared_component import LocalCartesianFrame
from coverage_mission_pipeline.route_connector import DIRECT_CONNECTOR_ALGORITHM
from coverage_mission_pipeline.vehicle_route_assembly import (
    FORWARD_ROUTE_DIRECTION,
    ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM,
)


def frame() -> LocalCartesianFrame:
    return LocalCartesianFrame("map", "EPSG:32643", 631285.61, 3358862.37)


def route_record(*, return_to_reference: bool = False, vehicle_id: str = "drone-1", altitude: float = 30.0) -> CompleteVehicleRouteRecord:
    points = [
        CoverageWaypoint(0.0, 0.0, altitude),
        CoverageWaypoint(5.0, 0.0, altitude),
        CoverageWaypoint(10.0, 0.0, altitude),
    ]
    segments = [
        ConnectorSpanRecord(
            WaypointSpan(0, 1),
            REFERENCE_CONNECTOR_ROLE,
            f"vehicle-reference:{vehicle_id}",
            "request-a",
            DIRECT_CONNECTOR_ALGORITHM,
            5.0,
        ),
        CoverageRouteSpanRecord(
            WaypointSpan(1, 2),
            "request-a",
            "component-a",
            "region-a",
            FORWARD_ROUTE_DIRECTION,
            "planned",
            5.0,
        ),
    ]
    if return_to_reference:
        points.append(CoverageWaypoint(0.0, 0.0, altitude))
        segments.append(
            ConnectorSpanRecord(
                WaypointSpan(2, 3),
                RETURN_CONNECTOR_ROLE,
                "request-a",
                f"vehicle-reference:{vehicle_id}",
                DIRECT_CONNECTOR_ALGORITHM,
                10.0,
            )
        )
    return CompleteVehicleRouteRecord(
        vehicle_id=vehicle_id,
        frame=frame(),
        reference_type="home",
        reference_position=LocalPoint2D(0.0, 0.0),
        algorithm=ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM,
        return_to_reference=return_to_reference,
        segments=tuple(segments),
        waypoints=tuple(points),
    )


def idle_record() -> CompleteVehicleRouteRecord:
    return CompleteVehicleRouteRecord(
        vehicle_id="idle-drone",
        frame=frame(),
        reference_type="home",
        reference_position=LocalPoint2D(0.0, 0.0),
        algorithm=ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM,
        return_to_reference=False,
        segments=(),
        waypoints=(),
    )


def mission_item(command: int = MAV_CMD_NAV_WAYPOINT, **overrides) -> ArduPilotMissionItem:
    values = dict(
        seq=0,
        current=1,
        frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        latitude_deg=30.0,
        longitude_deg=76.0,
        altitude_m=30.0,
        autocontinue=1,
    )
    if command == MAV_CMD_NAV_RETURN_TO_LAUNCH:
        values.update(latitude_deg=0.0, longitude_deg=0.0, altitude_m=0.0)
    if command == MAV_CMD_NAV_LAND:
        values.update(altitude_m=0.0)
    values.update(overrides)
    return ArduPilotMissionItem(**values)


def basic_mission() -> ArduPilotMission:
    return ArduPilotMission(
        "drone-1",
        (
            mission_item(MAV_CMD_NAV_TAKEOFF),
            mission_item(MAV_CMD_NAV_WAYPOINT, seq=1, current=0),
            mission_item(
                MAV_CMD_NAV_RETURN_TO_LAUNCH,
                seq=2,
                current=0,
            ),
        ),
    )


def test_constants_match_mavlink_values() -> None:
    assert MAV_FRAME_GLOBAL_RELATIVE_ALT == 3
    assert MAV_CMD_NAV_WAYPOINT == 16
    assert MAV_CMD_NAV_RETURN_TO_LAUNCH == 20
    assert MAV_CMD_NAV_LAND == 21
    assert MAV_CMD_NAV_TAKEOFF == 22


def test_build_default_command_sequence() -> None:
    mission = build_ardupilot_mission(route_record())
    assert [item.command for item in mission.items] == [
        MAV_CMD_NAV_TAKEOFF,
        MAV_CMD_NAV_WAYPOINT,
        MAV_CMD_NAV_WAYPOINT,
        MAV_CMD_NAV_RETURN_TO_LAUNCH,
    ]


def test_build_uses_home_relative_frame_for_every_item() -> None:
    mission = build_ardupilot_mission(route_record())
    assert {item.frame for item in mission.items} == {MAV_FRAME_GLOBAL_RELATIVE_ALT}
    assert mission.altitude_reference == "home_relative"


def test_build_sets_first_item_current_only() -> None:
    mission = build_ardupilot_mission(route_record())
    assert [item.current for item in mission.items] == [1, 0, 0, 0]


def test_build_sets_all_items_autocontinue() -> None:
    assert all(item.autocontinue == 1 for item in build_ardupilot_mission(route_record()).items)


def test_build_sequence_numbers_are_consecutive() -> None:
    mission = build_ardupilot_mission(route_record())
    assert [item.seq for item in mission.items] == list(range(len(mission.items)))


def test_build_takeoff_uses_reference_coordinates() -> None:
    route = route_record()
    first = route.geographic_waypoints()[0]
    takeoff = build_ardupilot_mission(route).items[0]
    assert takeoff.latitude_deg == pytest.approx(first.latitude_deg)
    assert takeoff.longitude_deg == pytest.approx(first.longitude_deg)
    assert takeoff.altitude_m == pytest.approx(30.0)


def test_build_skips_initial_reference_waypoint_by_default() -> None:
    route = route_record()
    geographic = route.geographic_waypoints()
    mission = build_ardupilot_mission(route)
    waypoints = mission.waypoint_items
    assert len(waypoints) == len(route.waypoints) - 1
    assert waypoints[0].latitude_deg == pytest.approx(geographic[1].latitude_deg)


def test_build_can_keep_initial_reference_waypoint() -> None:
    route = route_record()
    mission = build_ardupilot_mission(
        route,
        config=ArduPilotMissionBuildConfig(skip_initial_reference_waypoint=False),
    )
    assert len(mission.waypoint_items) == len(route.waypoints)
    assert mission.waypoint_items[0].latitude_deg == pytest.approx(
        route.geographic_waypoints()[0].latitude_deg
    )


def test_build_can_omit_takeoff() -> None:
    mission = build_ardupilot_mission(
        route_record(),
        config=ArduPilotMissionBuildConfig(
            include_takeoff=False,
            skip_initial_reference_waypoint=False,
        ),
    )
    assert mission.items[0].command == MAV_CMD_NAV_WAYPOINT
    assert all(item.command != MAV_CMD_NAV_TAKEOFF for item in mission.items)


def test_build_none_end_action() -> None:
    mission = build_ardupilot_mission(
        route_record(),
        config=ArduPilotMissionBuildConfig(end_action=END_ACTION_NONE),
    )
    assert mission.end_action == END_ACTION_NONE
    assert mission.items[-1].command == MAV_CMD_NAV_WAYPOINT


def test_build_rtl_end_action() -> None:
    mission = build_ardupilot_mission(route_record())
    assert mission.end_action == END_ACTION_RTL
    rtl = mission.items[-1]
    assert rtl.command == MAV_CMD_NAV_RETURN_TO_LAUNCH
    assert rtl.latitude_deg == 0.0
    assert rtl.longitude_deg == 0.0
    assert rtl.altitude_m == 0.0


def test_build_land_at_reference() -> None:
    mission = build_ardupilot_mission(
        route_record(return_to_reference=True),
        config=ArduPilotMissionBuildConfig(
            end_action=END_ACTION_LAND_AT_REFERENCE
        ),
    )
    land = mission.items[-1]
    first = route_record(return_to_reference=True).geographic_waypoints()[0]
    assert mission.end_action == END_ACTION_LAND_AT_REFERENCE
    assert land.command == MAV_CMD_NAV_LAND
    assert land.latitude_deg == pytest.approx(first.latitude_deg)
    assert land.longitude_deg == pytest.approx(first.longitude_deg)
    assert land.altitude_m == 0.0


def test_land_at_reference_requires_return_route() -> None:
    with pytest.raises(ArduPilotMissionError, match="return_to_reference"):
        build_ardupilot_mission(
            route_record(),
            config=ArduPilotMissionBuildConfig(
                end_action=END_ACTION_LAND_AT_REFERENCE
            ),
        )


def test_waypoint_hold_time_is_applied() -> None:
    mission = build_ardupilot_mission(
        route_record(),
        config=ArduPilotMissionBuildConfig(waypoint_hold_s=2.5),
    )
    assert all(item.param1 == 2.5 for item in mission.waypoint_items)


def test_builder_rejects_idle_route() -> None:
    with pytest.raises(ArduPilotMissionError, match="no route"):
        build_ardupilot_mission(idle_record())


def test_builder_rejects_wrong_route_type() -> None:
    with pytest.raises(ArduPilotMissionError, match="CompleteVehicleRouteRecord"):
        build_ardupilot_mission(object())


def test_builder_rejects_wrong_config_type() -> None:
    with pytest.raises(ArduPilotMissionError, match="BuildConfig"):
        build_ardupilot_mission(route_record(), config=object())


@pytest.mark.parametrize("altitude", [0.0, -1.0, 0.999])
def test_builder_rejects_altitude_below_default_minimum(altitude: float) -> None:
    with pytest.raises(ArduPilotMissionError, match="below"):
        build_ardupilot_mission(route_record(altitude=altitude))


def test_builder_accepts_configured_lower_positive_minimum() -> None:
    mission = build_ardupilot_mission(
        route_record(altitude=0.5),
        config=ArduPilotMissionBuildConfig(minimum_relative_altitude_m=0.1),
    )
    assert mission.items[0].altitude_m == pytest.approx(0.5)


@pytest.mark.parametrize("action", ["RTL", "home", "land"])
def test_config_rejects_unknown_end_action(action: str) -> None:
    with pytest.raises(ArduPilotMissionError, match="end_action"):
        ArduPilotMissionBuildConfig(end_action=action)


@pytest.mark.parametrize("hold", [-1.0, math.inf, math.nan, True, "1"])
def test_config_rejects_invalid_hold_time(hold) -> None:
    with pytest.raises(ArduPilotMissionError):
        ArduPilotMissionBuildConfig(waypoint_hold_s=hold)


@pytest.mark.parametrize("minimum", [0.0, -1.0, math.inf, math.nan, True])
def test_config_rejects_invalid_minimum_altitude(minimum) -> None:
    with pytest.raises(ArduPilotMissionError):
        ArduPilotMissionBuildConfig(minimum_relative_altitude_m=minimum)


def test_config_rejects_skip_without_takeoff() -> None:
    with pytest.raises(ArduPilotMissionError, match="requires"):
        ArduPilotMissionBuildConfig(
            include_takeoff=False,
            skip_initial_reference_waypoint=True,
        )


@pytest.mark.parametrize("field", ["include_takeoff", "skip_initial_reference_waypoint"])
def test_config_rejects_non_bool_flags(field: str) -> None:
    kwargs = {field: 1}
    with pytest.raises(ArduPilotMissionError, match="bool"):
        ArduPilotMissionBuildConfig(**kwargs)


def test_item_command_name() -> None:
    assert mission_item().command_name == "MAV_CMD_NAV_WAYPOINT"
    assert mission_item(MAV_CMD_NAV_TAKEOFF).command_name == "MAV_CMD_NAV_TAKEOFF"
    assert mission_item(MAV_CMD_NAV_LAND).command_name == "MAV_CMD_NAV_LAND"
    assert (
        mission_item(MAV_CMD_NAV_RETURN_TO_LAUNCH).command_name
        == "MAV_CMD_NAV_RETURN_TO_LAUNCH"
    )


@pytest.mark.parametrize("value", [-1, 1.5, True])
def test_item_rejects_invalid_seq(value) -> None:
    with pytest.raises(ArduPilotMissionError, match="seq"):
        mission_item(seq=value)


@pytest.mark.parametrize("field", ["current", "autocontinue"])
@pytest.mark.parametrize("value", [-1, 2, True, 0.0])
def test_item_rejects_invalid_binary_fields(field: str, value) -> None:
    with pytest.raises(ArduPilotMissionError):
        mission_item(**{field: value})


@pytest.mark.parametrize("frame_value", [0, 6, 10, True, 3.0])
def test_item_rejects_non_relative_frame(frame_value) -> None:
    with pytest.raises(ArduPilotMissionError, match="GLOBAL_RELATIVE"):
        mission_item(frame=frame_value)


@pytest.mark.parametrize("command", [0, 17, 999, True, 16.0])
def test_item_rejects_unsupported_command(command) -> None:
    with pytest.raises(ArduPilotMissionError, match="command"):
        mission_item(command=command)


@pytest.mark.parametrize("latitude", [-90.1, 90.1])
def test_item_rejects_invalid_latitude(latitude: float) -> None:
    with pytest.raises(ArduPilotMissionError, match="latitude"):
        mission_item(latitude_deg=latitude)


@pytest.mark.parametrize("longitude", [-180.1, 180.1])
def test_item_rejects_invalid_longitude(longitude: float) -> None:
    with pytest.raises(ArduPilotMissionError, match="longitude"):
        mission_item(longitude_deg=longitude)


@pytest.mark.parametrize("name", ["param2", "param3", "param4"])
def test_waypoint_rejects_nonzero_unsupported_parameters(name: str) -> None:
    with pytest.raises(ArduPilotMissionError, match="param2"):
        mission_item(**{name: 1.0})


def test_waypoint_rejects_negative_hold() -> None:
    with pytest.raises(ArduPilotMissionError, match="hold"):
        mission_item(param1=-1.0)


@pytest.mark.parametrize("altitude", [0.0, -1.0])
def test_waypoint_rejects_nonpositive_altitude(altitude: float) -> None:
    with pytest.raises(ArduPilotMissionError, match="greater than zero"):
        mission_item(altitude_m=altitude)


@pytest.mark.parametrize("parameter", ["param1", "param2", "param3", "param4"])
def test_takeoff_rejects_nonzero_parameters(parameter: str) -> None:
    with pytest.raises(ArduPilotMissionError, match="takeoff"):
        mission_item(MAV_CMD_NAV_TAKEOFF, **{parameter: 1.0})


def test_takeoff_rejects_nonpositive_altitude() -> None:
    with pytest.raises(ArduPilotMissionError, match="takeoff"):
        mission_item(MAV_CMD_NAV_TAKEOFF, altitude_m=0.0)


def test_rtl_rejects_coordinates() -> None:
    with pytest.raises(ArduPilotMissionError, match="zero"):
        mission_item(MAV_CMD_NAV_RETURN_TO_LAUNCH, latitude_deg=1.0)


def test_rtl_rejects_parameters() -> None:
    with pytest.raises(ArduPilotMissionError, match="zero"):
        mission_item(MAV_CMD_NAV_RETURN_TO_LAUNCH, param1=1.0)


def test_land_rejects_nonzero_altitude() -> None:
    with pytest.raises(ArduPilotMissionError, match="zero"):
        mission_item(MAV_CMD_NAV_LAND, altitude_m=1.0)


def test_mission_requires_items() -> None:
    with pytest.raises(ArduPilotMissionError, match="must not be empty"):
        ArduPilotMission("drone-1", ())


def test_mission_rejects_wrong_item_type() -> None:
    with pytest.raises(ArduPilotMissionError, match="ArduPilotMissionItem"):
        ArduPilotMission("drone-1", (object(),))


def test_mission_rejects_nonconsecutive_sequence() -> None:
    with pytest.raises(ArduPilotMissionError, match="consecutive"):
        ArduPilotMission(
            "drone-1",
            (
                mission_item(MAV_CMD_NAV_TAKEOFF),
                mission_item(MAV_CMD_NAV_WAYPOINT, seq=2, current=0),
            ),
        )


def test_mission_rejects_current_on_later_item() -> None:
    with pytest.raises(ArduPilotMissionError, match="current"):
        ArduPilotMission(
            "drone-1",
            (
                mission_item(MAV_CMD_NAV_TAKEOFF),
                mission_item(MAV_CMD_NAV_WAYPOINT, seq=1, current=1),
            ),
        )


def test_mission_rejects_first_current_zero() -> None:
    with pytest.raises(ArduPilotMissionError, match="current"):
        ArduPilotMission(
            "drone-1",
            (mission_item(current=0),),
        )


def test_mission_rejects_autocontinue_zero() -> None:
    with pytest.raises(ArduPilotMissionError, match="autocontinue"):
        ArduPilotMission(
            "drone-1",
            (mission_item(autocontinue=0),),
        )


def test_mission_rejects_takeoff_after_first() -> None:
    with pytest.raises(ArduPilotMissionError, match="TAKEOFF"):
        ArduPilotMission(
            "drone-1",
            (
                mission_item(),
                mission_item(MAV_CMD_NAV_TAKEOFF, seq=1, current=0),
            ),
        )


def test_mission_rejects_terminal_before_last() -> None:
    with pytest.raises(ArduPilotMissionError, match="final"):
        ArduPilotMission(
            "drone-1",
            (
                mission_item(MAV_CMD_NAV_RETURN_TO_LAUNCH),
                mission_item(seq=1, current=0),
            ),
        )


def test_mission_requires_waypoint_command() -> None:
    with pytest.raises(ArduPilotMissionError, match="waypoint"):
        ArduPilotMission("drone-1", (mission_item(MAV_CMD_NAV_TAKEOFF),))


def test_mission_rejects_invalid_vehicle_id() -> None:
    with pytest.raises(ArduPilotMissionError, match="vehicle_id"):
        ArduPilotMission("bad id", (mission_item(),))


def test_mission_rejects_other_altitude_reference() -> None:
    with pytest.raises(ArduPilotMissionError, match="home_relative"):
        ArduPilotMission("drone-1", (mission_item(),), "amsl")


def test_json_round_trip() -> None:
    mission = basic_mission()
    assert ArduPilotMission.from_json(mission.to_json()) == mission


def test_json_is_deterministic() -> None:
    mission = basic_mission()
    assert mission.to_json() == mission.to_json()
    data = json.loads(mission.to_json())
    assert data["schema_version"] == ARDUPILOT_MISSION_SCHEMA_VERSION
    assert data["altitude_reference"] == "home_relative"


def test_json_rejects_unknown_root_field() -> None:
    data = basic_mission().to_dict()
    data["unknown"] = 1
    with pytest.raises(ArduPilotMissionError, match="unknown"):
        ArduPilotMission.from_dict(data)


def test_json_rejects_unknown_item_field() -> None:
    data = basic_mission().to_dict()
    data["items"][0]["unknown"] = 1
    with pytest.raises(ArduPilotMissionError, match="unknown"):
        ArduPilotMission.from_dict(data)


def test_json_rejects_wrong_schema() -> None:
    data = basic_mission().to_dict()
    data["schema_version"] = 99
    with pytest.raises(ArduPilotMissionError, match="schema_version"):
        ArduPilotMission.from_dict(data)


def test_json_rejects_invalid_text() -> None:
    with pytest.raises(ArduPilotMissionError, match="invalid JSON"):
        ArduPilotMission.from_json("{")


def test_wpl_header_and_field_count() -> None:
    text = basic_mission().to_qgc_wpl_110()
    lines = text.splitlines()
    assert lines[0] == QGC_WPL_110_HEADER
    assert all(len(line.split()) == 12 for line in lines[1:])


def test_wpl_is_tab_separated() -> None:
    line = basic_mission().to_qgc_wpl_110().splitlines()[1]
    assert line.count("\t") == 11


def test_wpl_round_trip() -> None:
    mission = basic_mission()
    parsed = ArduPilotMission.from_qgc_wpl_110(
        mission.to_qgc_wpl_110(), vehicle_id="drone-1"
    )
    assert parsed == mission


def test_wpl_parser_ignores_comments_and_blank_lines() -> None:
    mission = basic_mission()
    lines = mission.to_qgc_wpl_110().splitlines()
    text = "\n".join([lines[0], "# generated", "", *lines[1:]]) + "\n"
    assert ArduPilotMission.from_qgc_wpl_110(text, vehicle_id="drone-1") == mission


def test_wpl_parser_rejects_wrong_header() -> None:
    with pytest.raises(ArduPilotMissionError, match="begin"):
        ArduPilotMission.from_qgc_wpl_110("QGC WPL 100\n", vehicle_id="drone-1")


def test_wpl_parser_rejects_wrong_field_count() -> None:
    with pytest.raises(ArduPilotMissionError, match="12 fields"):
        ArduPilotMission.from_qgc_wpl_110(
            QGC_WPL_110_HEADER + "\n0 1 3\n", vehicle_id="drone-1"
        )


def test_wpl_parser_rejects_non_numeric_value() -> None:
    text = QGC_WPL_110_HEADER + "\n0\t1\t3\t16\tx\t0\t0\t0\t30\t76\t30\t1\n"
    with pytest.raises(ArduPilotMissionError, match="invalid"):
        ArduPilotMission.from_qgc_wpl_110(text, vehicle_id="drone-1")


def test_wpl_has_no_negative_zero() -> None:
    item = mission_item(param1=-0.0)
    assert "-0.0000000000" not in item.to_wpl_line()


def test_write_json_atomic_round_trip(tmp_path: Path) -> None:
    mission = basic_mission()
    path = mission.write_json(tmp_path)
    assert path.name == "drone-1.ardupilot-mission.json"
    assert ArduPilotMission.read_json(path) == mission
    assert not list(tmp_path.glob("*.tmp"))


def test_write_wpl_atomic_round_trip(tmp_path: Path) -> None:
    mission = basic_mission()
    path = mission.write_qgc_wpl_110(tmp_path)
    assert path.name == "drone-1.waypoints"
    assert ArduPilotMission.read_qgc_wpl_110(path, vehicle_id="drone-1") == mission
    assert not list(tmp_path.glob("*.tmp"))


def test_write_explicit_paths(tmp_path: Path) -> None:
    mission = basic_mission()
    json_path = mission.write_json(tmp_path / "custom.json")
    wpl_path = mission.write_qgc_wpl_110(tmp_path / "custom.waypoints")
    assert json_path.name == "custom.json"
    assert wpl_path.name == "custom.waypoints"


def test_batch_sorted_by_vehicle_id() -> None:
    missions = build_ardupilot_missions(
        (
            route_record(vehicle_id="drone-2"),
            route_record(vehicle_id="drone-1"),
        )
    )
    assert [mission.vehicle_id for mission in missions] == ["drone-1", "drone-2"]


def test_batch_rejects_empty_input() -> None:
    with pytest.raises(ArduPilotMissionError, match="must not be empty"):
        build_ardupilot_missions(())


def test_batch_rejects_duplicate_vehicle_ids() -> None:
    with pytest.raises(ArduPilotMissionError, match="unique"):
        build_ardupilot_missions((route_record(), route_record()))


def test_batch_rejects_idle_instead_of_dropping_it() -> None:
    with pytest.raises(ArduPilotMissionError, match="no route"):
        build_ardupilot_missions((route_record(), idle_record()))


def test_batch_rejects_wrong_item_type() -> None:
    with pytest.raises(ArduPilotMissionError, match="CompleteVehicleRouteRecord"):
        build_ardupilot_missions((object(),))


def test_single_waypoint_after_skip_is_preserved() -> None:
    # Construct the smallest valid complete record: one trivial connector point and
    # one one-point route sharing the same authoritative waypoint.
    one = CompleteVehicleRouteRecord(
        vehicle_id="drone-1",
        frame=frame(),
        reference_type="home",
        reference_position=LocalPoint2D(0.0, 0.0),
        algorithm=ROUTE_DIRECTION_OPTIMIZATION_ALGORITHM,
        return_to_reference=False,
        segments=(
            ConnectorSpanRecord(
                WaypointSpan(0, 0),
                REFERENCE_CONNECTOR_ROLE,
                "vehicle-reference:drone-1",
                "request-a",
                "trivial_same_point_v1",
                0.0,
            ),
            CoverageRouteSpanRecord(
                WaypointSpan(0, 0),
                "request-a",
                "component-a",
                "region-a",
                FORWARD_ROUTE_DIRECTION,
                "planned",
                0.0,
            ),
        ),
        waypoints=(CoverageWaypoint(0.0, 0.0, 30.0),),
    )
    mission = build_ardupilot_mission(one)
    assert len(mission.waypoint_items) == 1
    assert mission.items[0].command == MAV_CMD_NAV_TAKEOFF


def test_generated_geographic_points_match_route_conversion() -> None:
    route = route_record()
    geographic = route.geographic_waypoints()
    mission = build_ardupilot_mission(route)
    assert [item.latitude_deg for item in mission.waypoint_items] == pytest.approx(
        [point.latitude_deg for point in geographic[1:]]
    )
    assert [item.longitude_deg for item in mission.waypoint_items] == pytest.approx(
        [point.longitude_deg for point in geographic[1:]]
    )


def test_export_does_not_modify_source_record() -> None:
    route = route_record()
    before = route.to_json()
    build_ardupilot_mission(route)
    assert route.to_json() == before
