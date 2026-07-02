from __future__ import annotations

import json
from pathlib import Path
import zipfile

import pytest
from shapely.geometry import Polygon
from shapely.ops import unary_union

from coverage_mission_pipeline.kml_input import KmlInputError, load_kml_mission_input
from coverage_mission_pipeline.kml_product import (
    build_kml_product_artifacts,
    write_kml_product_artifacts,
)
from coverage_mission_pipeline.map_overlay import write_input_overlay
from coverage_mission_pipeline.swarm_mission_config import (
    SwarmMissionOperationalConfig,
)
from coverage_mission_pipeline.swarm_partitions_adapter import (
    adapt_swarm_partitions_payload,
)


def polygon_xml(name: str, coordinates: list[tuple[float, float]]) -> str:
    values = " ".join(f"{lon},{lat},0" for lon, lat in coordinates)
    return f"""
    <Placemark>
      <name>{name}</name>
      <Polygon>
        <outerBoundaryIs><LinearRing><coordinates>{values}</coordinates></LinearRing></outerBoundaryIs>
      </Polygon>
    </Placemark>
    """


def point_xml(name: str, coordinate: tuple[float, float]) -> str:
    lon, lat = coordinate
    return f"<Placemark><name>{name}</name><Point><coordinates>{lon},{lat},0</coordinates></Point></Placemark>"


def kml_document(*placemarks: str) -> str:
    return (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>'
        + "".join(placemarks)
        + "</Document></kml>"
    )


def site_kml(*, named_boundary: bool = True, explicit_home: bool = True) -> str:
    boundary_name = "BOUNDARY" if named_boundary else "site"
    boundary = [
        (77.0000, 28.0000),
        (77.0120, 28.0000),
        (77.0120, 28.0100),
        (77.0000, 28.0100),
        (77.0000, 28.0000),
    ]
    exclusion = [
        (77.0040, 28.0030),
        (77.0060, 28.0030),
        (77.0060, 28.0050),
        (77.0040, 28.0050),
        (77.0040, 28.0030),
    ]
    values = [
        polygon_xml(boundary_name, boundary),
        polygon_xml("NO_GO: building", exclusion),
    ]
    if explicit_home:
        values.append(point_xml("HOME", (77.0010, 28.0010)))
    return kml_document(*values)


def test_load_explicit_kml_and_select_utm(tmp_path: Path) -> None:
    path = tmp_path / "site.kml"
    path.write_text(site_kml(), encoding="utf-8")

    mission = load_kml_mission_input(path)

    assert mission.planning_crs == "EPSG:32643"
    assert mission.exclusion_count == 1
    assert mission.supplied_partition_count == 0
    assert mission.home_was_explicit is True
    assert mission.route_space_projected.area < mission.safe_area_projected.area
    assert mission.safe_area_projected.area < mission.boundary_projected.area


def test_automatic_boundary_and_home_are_deterministic(tmp_path: Path) -> None:
    path = tmp_path / "site.kml"
    path.write_text(
        site_kml(named_boundary=False, explicit_home=False),
        encoding="utf-8",
    )

    first = load_kml_mission_input(path)
    second = load_kml_mission_input(path)

    assert first.home_was_explicit is False
    assert first.home_longitude_deg == pytest.approx(second.home_longitude_deg)
    assert first.home_latitude_deg == pytest.approx(second.home_latitude_deg)


def test_kmz_doc_kml_is_supported(tmp_path: Path) -> None:
    path = tmp_path / "site.kmz"
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr("doc.kml", site_kml())

    mission = load_kml_mission_input(path)
    assert mission.exclusion_count == 1


def test_ambiguous_outer_polygons_fail_closed(tmp_path: Path) -> None:
    left = [
        (77.0, 28.0),
        (77.002, 28.0),
        (77.002, 28.002),
        (77.0, 28.002),
        (77.0, 28.0),
    ]
    right = [
        (77.01, 28.0),
        (77.012, 28.0),
        (77.012, 28.002),
        (77.01, 28.002),
        (77.01, 28.0),
    ]
    path = tmp_path / "ambiguous.kml"
    path.write_text(
        kml_document(polygon_xml("one", left), polygon_xml("two", right)),
        encoding="utf-8",
    )

    with pytest.raises(KmlInputError, match="unambiguous boundary"):
        load_kml_mission_input(path)


def test_explicit_home_outside_route_space_is_rejected(tmp_path: Path) -> None:
    text = site_kml().replace("77.001,28.001,0", "77.0045,28.004,0")
    path = tmp_path / "bad-home.kml"
    path.write_text(text, encoding="utf-8")

    with pytest.raises(KmlInputError, match="explicit HOME"):
        load_kml_mission_input(path)


def test_build_artifacts_are_adapter_compatible(tmp_path: Path) -> None:
    path = tmp_path / "site.kml"
    path.write_text(site_kml(), encoding="utf-8")

    artifacts = build_kml_product_artifacts(
        path,
        min_component_area_m2=1.0,
    )

    assert artifacts.automatic_partitioning is not None
    assert len(artifacts.mission_output["partitions"]) == 5
    route_areas = artifacts.automatic_partitioning.route_area_by_partition_m2
    assert max(route_areas) - min(route_areas) < 1.0e-3

    config = SwarmMissionOperationalConfig.from_dict(
        artifacts.operational_config
    )
    result = adapt_swarm_partitions_payload(
        artifacts.mission_output,
        config.adapter,
    )
    assert len(result.component_ids_by_partition_id) == 5

    output = write_kml_product_artifacts(artifacts, tmp_path / "output")
    overlay = write_input_overlay(
        artifacts.mission_output,
        artifacts.operational_config,
        output / "map-input-overlay.kml",
    )
    assert (output / "mission_output.json").is_file()
    assert (output / "swarm_mission.yaml").is_file()
    assert overlay.is_file()


def test_automatic_partitions_cover_boundary_without_area_overlap(tmp_path: Path) -> None:
    path = tmp_path / "site.kml"
    path.write_text(site_kml(), encoding="utf-8")
    artifacts = build_kml_product_artifacts(path, min_component_area_m2=1.0)

    records = artifacts.mission_output["partitions"]
    polygons = []
    for partition in records:
        components = [
            Polygon(record["exterior"], record["holes"])
            for record in partition["geometry"]
        ]
        polygons.append(unary_union(components))

    boundary = Polygon(
        artifacts.mission_output["boundary"][0]["exterior"],
        artifacts.mission_output["boundary"][0]["holes"],
    )
    assert boundary.difference(unary_union(polygons)).area < 2.0e-9
    for left_index, left in enumerate(polygons):
        for right in polygons[left_index + 1:]:
            assert left.intersection(right).area < 1.0e-12


def test_written_json_has_strict_five_partition_contract(tmp_path: Path) -> None:
    source = tmp_path / "site.kml"
    source.write_text(site_kml(), encoding="utf-8")
    artifacts = build_kml_product_artifacts(source, min_component_area_m2=1.0)
    output = write_kml_product_artifacts(artifacts, tmp_path / "out")

    payload = json.loads((output / "mission_output.json").read_text(encoding="utf-8"))
    assert payload["metadata"]["n_partitions"] == 5
    assert [item["id"] for item in payload["partitions"]] == [1, 2, 3, 4, 5]
    assert payload["no_go_zones"]["predetermined"][0]["name"] == "NO_GO: building"

def test_generated_overlay_can_be_reimported(tmp_path: Path) -> None:
    source = tmp_path / "site.kml"
    source.write_text(site_kml(), encoding="utf-8")
    artifacts = build_kml_product_artifacts(source, min_component_area_m2=1.0)
    overlay = write_input_overlay(
        artifacts.mission_output,
        artifacts.operational_config,
        tmp_path / "overlay.kml",
    )

    imported = load_kml_mission_input(overlay)
    assert imported.supplied_partition_count == 5
    assert imported.exclusion_count == 1
    assert imported.home_was_explicit is True
