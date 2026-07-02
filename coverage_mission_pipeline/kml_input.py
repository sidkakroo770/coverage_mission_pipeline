#!/usr/bin/env python3
"""Deterministic KML/KMZ mission-area import for the coverage product.

The importer intentionally supports polygonal mission geometry and one optional
HOME point. It rejects ambiguous documents before ROS, SITL, or any vehicle
connection is started.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Iterable, Optional
from xml.etree import ElementTree
import zipfile

from pyproj import CRS, Transformer
from shapely.geometry import MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.geometry.polygon import orient
from shapely.ops import transform, unary_union
from shapely.validation import explain_validity

from .mission_geometry_core import (
    GeometryCoreError,
    create_operational_route_space,
    create_safe_area,
    extract_polygon_components,
)


_KML_SUFFIXES = frozenset({".kml", ".kmz"})
_PARTITION_PATTERN = re.compile(r"^PARTITION[ _-]?([1-9][0-9]*)(?:\s*[: _-].*)?$", re.IGNORECASE)
_NO_GO_PATTERN = re.compile(r"^(?:NO[ _-]?GO|EXCLUSION)(?:\s*[: _-].*)?$", re.IGNORECASE)
_BOUNDARY_PATTERN = re.compile(r"^BOUNDARY(?:\s*[: _-].*)?$", re.IGNORECASE)
_HOME_PATTERN = re.compile(r"^(?:HOME|LAUNCH|TAKEOFF)$", re.IGNORECASE)


class KmlInputError(ValueError):
    """Raised when a KML/KMZ document cannot be interpreted safely."""


@dataclass(frozen=True)
class KmlPolygonFeature:
    name: str
    geometry_wgs84: BaseGeometry


@dataclass(frozen=True)
class KmlPointFeature:
    name: str
    longitude_deg: float
    latitude_deg: float


@dataclass(frozen=True)
class KmlMissionInput:
    source_path: Path
    planning_crs: str
    boundary_wgs84: Polygon
    exclusions_wgs84: tuple[tuple[str, BaseGeometry], ...]
    supplied_partitions_wgs84: tuple[tuple[int, BaseGeometry], ...]
    home_longitude_deg: float
    home_latitude_deg: float
    home_was_explicit: bool
    boundary_projected: Polygon
    exclusions_projected: tuple[BaseGeometry, ...]
    safe_area_projected: BaseGeometry
    route_space_projected: BaseGeometry

    @property
    def exclusion_count(self) -> int:
        return len(self.exclusions_wgs84)

    @property
    def supplied_partition_count(self) -> int:
        return len(self.supplied_partitions_wgs84)


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _direct_child_text(element: ElementTree.Element, name: str) -> str:
    for child in element:
        if _local_name(child.tag) == name:
            return (child.text or "").strip()
    return ""


def _coordinate_tokens(text: str, path: str) -> list[tuple[float, float]]:
    coordinates: list[tuple[float, float]] = []
    for index, token in enumerate(text.replace("\n", " ").split()):
        fields = token.split(",")
        if len(fields) < 2:
            raise KmlInputError(f"{path}[{index}] must contain longitude,latitude")
        try:
            longitude = float(fields[0])
            latitude = float(fields[1])
        except ValueError as exc:
            raise KmlInputError(f"{path}[{index}] contains a non-numeric coordinate") from exc
        if not math.isfinite(longitude) or not math.isfinite(latitude):
            raise KmlInputError(f"{path}[{index}] must be finite")
        if not -180.0 <= longitude <= 180.0:
            raise KmlInputError(f"{path}[{index}] longitude must be in [-180, 180]")
        if not -90.0 <= latitude <= 90.0:
            raise KmlInputError(f"{path}[{index}] latitude must be in [-90, 90]")
        coordinates.append((longitude, latitude))
    return coordinates


def _ring_from_boundary(
    boundary_element: ElementTree.Element,
    path: str,
) -> list[tuple[float, float]]:
    coordinates_element: Optional[ElementTree.Element] = None
    for descendant in boundary_element.iter():
        if _local_name(descendant.tag) == "coordinates":
            coordinates_element = descendant
            break
    if coordinates_element is None:
        raise KmlInputError(f"{path} has no coordinates")
    coordinates = _coordinate_tokens(coordinates_element.text or "", path)
    if len(coordinates) >= 2 and coordinates[0] == coordinates[-1]:
        coordinates.pop()
    if len(set(coordinates)) < 3:
        raise KmlInputError(f"{path} must contain at least three distinct points")
    return coordinates


def _polygon_from_element(
    element: ElementTree.Element,
    path: str,
) -> Polygon:
    outer: Optional[list[tuple[float, float]]] = None
    holes: list[list[tuple[float, float]]] = []

    for child in element:
        name = _local_name(child.tag)
        if name == "outerBoundaryIs":
            if outer is not None:
                raise KmlInputError(f"{path} contains more than one outer boundary")
            outer = _ring_from_boundary(child, f"{path}.outerBoundaryIs")
        elif name == "innerBoundaryIs":
            holes.append(_ring_from_boundary(child, f"{path}.innerBoundaryIs"))

    if outer is None:
        raise KmlInputError(f"{path} has no outerBoundaryIs")

    polygon = Polygon(outer, holes)
    if polygon.is_empty or polygon.area <= 0.0:
        raise KmlInputError(f"{path} must have positive area")
    if not polygon.is_valid:
        raise KmlInputError(f"{path} is invalid: {explain_validity(polygon)}")
    return orient(polygon, sign=1.0)


def _point_from_element(
    element: ElementTree.Element,
    path: str,
) -> tuple[float, float]:
    coordinates_element: Optional[ElementTree.Element] = None
    for descendant in element.iter():
        if _local_name(descendant.tag) == "coordinates":
            coordinates_element = descendant
            break
    if coordinates_element is None:
        raise KmlInputError(f"{path} has no coordinates")
    coordinates = _coordinate_tokens(coordinates_element.text or "", path)
    if len(coordinates) != 1:
        raise KmlInputError(f"{path} must contain exactly one coordinate")
    return coordinates[0]


def _read_document_bytes(path: Path) -> bytes:
    suffix = path.suffix.lower()
    if suffix not in _KML_SUFFIXES:
        raise KmlInputError("input path must end in .kml or .kmz")
    try:
        if suffix == ".kml":
            return path.read_bytes()
        with zipfile.ZipFile(path) as archive:
            members = sorted(
                name for name in archive.namelist()
                if name.lower().endswith(".kml") and not name.endswith("/")
            )
            if not members:
                raise KmlInputError("KMZ archive contains no KML document")
            preferred = [name for name in members if Path(name).name.lower() == "doc.kml"]
            selected = preferred[0] if preferred else members[0]
            return archive.read(selected)
    except KmlInputError:
        raise
    except (OSError, zipfile.BadZipFile, KeyError) as exc:
        raise KmlInputError(f"could not read KML/KMZ input: {exc}") from exc


def _parse_features(path: Path) -> tuple[list[KmlPolygonFeature], list[KmlPointFeature]]:
    try:
        root = ElementTree.fromstring(_read_document_bytes(path))
    except ElementTree.ParseError as exc:
        raise KmlInputError(f"invalid XML at line {exc.position[0]}: {exc}") from exc

    polygons: list[KmlPolygonFeature] = []
    points: list[KmlPointFeature] = []

    placemarks = [element for element in root.iter() if _local_name(element.tag) == "Placemark"]
    for placemark_index, placemark in enumerate(placemarks):
        name = _direct_child_text(placemark, "name") or f"Placemark {placemark_index + 1}"
        polygon_values: list[Polygon] = []
        point_values: list[tuple[float, float]] = []

        for descendant in placemark.iter():
            local = _local_name(descendant.tag)
            if local == "Polygon":
                polygon_values.append(
                    _polygon_from_element(
                        descendant,
                        f"Placemark[{placemark_index}]({name!r}).Polygon[{len(polygon_values)}]",
                    )
                )
            elif local == "Point":
                point_values.append(
                    _point_from_element(
                        descendant,
                        f"Placemark[{placemark_index}]({name!r}).Point[{len(point_values)}]",
                    )
                )

        if polygon_values:
            geometry: BaseGeometry
            if len(polygon_values) == 1:
                geometry = polygon_values[0]
            else:
                geometry = MultiPolygon(polygon_values)
                if not geometry.is_valid:
                    union = unary_union(polygon_values)
                    if union.is_empty or not union.is_valid:
                        raise KmlInputError(
                            f"Placemark {name!r} contains invalid overlapping polygons"
                        )
                    geometry = union
            polygons.append(KmlPolygonFeature(name=name, geometry_wgs84=geometry))

        for longitude, latitude in point_values:
            points.append(
                KmlPointFeature(
                    name=name,
                    longitude_deg=longitude,
                    latitude_deg=latitude,
                )
            )

    if not polygons:
        raise KmlInputError("KML contains no Polygon geometry")
    return polygons, points


def _utm_crs_for_boundary(boundary: Polygon) -> CRS:
    min_lon, min_lat, max_lon, max_lat = boundary.bounds
    if max_lon - min_lon > 6.0:
        raise KmlInputError("mission boundary spans more than one practical UTM zone")
    centroid = boundary.centroid
    longitude = float(centroid.x)
    latitude = float(centroid.y)
    if latitude < -80.0 or latitude > 84.0:
        raise KmlInputError("automatic UTM selection supports latitudes from -80 to 84 degrees")
    zone = int(math.floor((longitude + 180.0) / 6.0)) + 1
    zone = max(1, min(60, zone))
    epsg = (32600 if latitude >= 0.0 else 32700) + zone
    return CRS.from_epsg(epsg)


def _project_geometry(geometry: BaseGeometry, transformer: Transformer) -> BaseGeometry:
    projected = transform(transformer.transform, geometry)
    if projected.is_empty or not projected.is_valid:
        raise KmlInputError("geometry became invalid during projection")
    return projected


def _polygon_parts(geometry: BaseGeometry) -> list[Polygon]:
    parts = extract_polygon_components(geometry)
    if not parts:
        raise KmlInputError("geometry has no polygon components")
    return parts


def _covers_with_tolerance(container: BaseGeometry, child: BaseGeometry) -> bool:
    # KML writers commonly round coordinates to 8-9 decimal places.  Permit
    # only sub-millimetre WGS84 serialization noise while still rejecting
    # materially out-of-bounds geometry.
    return container.buffer(1.0e-9).covers(child)


def _classify_polygon_features(
    features: Iterable[KmlPolygonFeature],
    expected_partition_count: int,
) -> tuple[Polygon, list[tuple[str, BaseGeometry]], dict[int, BaseGeometry]]:
    explicit_boundaries: list[KmlPolygonFeature] = []
    explicit_exclusions: list[KmlPolygonFeature] = []
    unclassified: list[KmlPolygonFeature] = []
    partitions: dict[int, list[BaseGeometry]] = {}

    for feature in features:
        name = feature.name.strip()
        partition_match = _PARTITION_PATTERN.fullmatch(name)
        if _BOUNDARY_PATTERN.fullmatch(name):
            explicit_boundaries.append(feature)
        elif partition_match:
            partition_id = int(partition_match.group(1))
            partitions.setdefault(partition_id, []).append(feature.geometry_wgs84)
        elif _NO_GO_PATTERN.fullmatch(name):
            explicit_exclusions.append(feature)
        else:
            unclassified.append(feature)

    if len(explicit_boundaries) > 1:
        raise KmlInputError("KML contains more than one explicitly named BOUNDARY")

    if explicit_boundaries:
        boundary_feature = explicit_boundaries[0]
    else:
        candidates: list[KmlPolygonFeature] = []
        all_geometries = [feature.geometry_wgs84 for feature in unclassified]
        all_geometries += [feature.geometry_wgs84 for feature in explicit_exclusions]
        all_geometries += [geometry for values in partitions.values() for geometry in values]
        for feature in unclassified:
            if all(
                geometry is feature.geometry_wgs84
                or _covers_with_tolerance(feature.geometry_wgs84, geometry)
                for geometry in all_geometries
            ):
                candidates.append(feature)
        if len(candidates) != 1:
            raise KmlInputError(
                "could not identify one unambiguous boundary; name it BOUNDARY"
            )
        boundary_feature = candidates[0]
        unclassified = [feature for feature in unclassified if feature is not boundary_feature]

    boundary_parts = _polygon_parts(boundary_feature.geometry_wgs84)
    if len(boundary_parts) != 1:
        raise KmlInputError("mission boundary must be one connected Polygon")
    boundary = boundary_parts[0]

    exclusions: list[tuple[str, BaseGeometry]] = [
        (feature.name, feature.geometry_wgs84)
        for feature in explicit_exclusions
    ]
    exclusions.extend((feature.name, feature.geometry_wgs84) for feature in unclassified)

    combined_partitions: dict[int, BaseGeometry] = {}
    if partitions:
        expected = set(range(1, expected_partition_count + 1))
        actual = set(partitions)
        if actual != expected:
            raise KmlInputError(
                "supplied partition IDs must be exactly 1 through "
                f"{expected_partition_count}; found {sorted(actual)}"
            )
        for partition_id, geometries in partitions.items():
            geometry = unary_union(geometries)
            if geometry.is_empty or not geometry.is_valid:
                raise KmlInputError(f"PARTITION_{partition_id} is invalid")
            combined_partitions[partition_id] = geometry

    return boundary, exclusions, combined_partitions


def _select_home(
    points: Iterable[KmlPointFeature],
    route_space_projected: BaseGeometry,
    to_projected: Transformer,
    to_wgs84: Transformer,
) -> tuple[float, float, bool]:
    explicit = [point for point in points if _HOME_PATTERN.fullmatch(point.name.strip())]
    if len(explicit) > 1:
        raise KmlInputError("KML contains more than one HOME/LAUNCH/TAKEOFF point")
    if explicit:
        point = explicit[0]
        projected = Point(
            *to_projected.transform(point.longitude_deg, point.latitude_deg)
        )
        if not route_space_projected.covers(projected):
            raise KmlInputError("explicit HOME must lie inside operational route space")
        return point.longitude_deg, point.latitude_deg, True

    representative = route_space_projected.representative_point()
    longitude, latitude = to_wgs84.transform(representative.x, representative.y)
    return float(longitude), float(latitude), False


def load_kml_mission_input(
    path: Path | str,
    *,
    clearance_m: float = 10.0,
    tracking_margin_m: float = 2.0,
    expected_partition_count: int = 5,
) -> KmlMissionInput:
    """Load, classify, project, and safety-check one KML/KMZ mission area."""
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise KmlInputError(f"input file does not exist: {source}")
    if expected_partition_count < 1:
        raise KmlInputError("expected_partition_count must be positive")

    polygon_features, point_features = _parse_features(source)
    boundary, exclusions, partitions = _classify_polygon_features(
        polygon_features,
        expected_partition_count,
    )

    planning_crs = _utm_crs_for_boundary(boundary)
    to_projected = Transformer.from_crs("EPSG:4326", planning_crs, always_xy=True)
    to_wgs84 = Transformer.from_crs(planning_crs, "EPSG:4326", always_xy=True)

    boundary_projected = _project_geometry(boundary, to_projected)
    boundary_parts = _polygon_parts(boundary_projected)
    if len(boundary_parts) != 1:
        raise KmlInputError("projected boundary must remain one connected Polygon")
    boundary_projected_polygon = boundary_parts[0]

    exclusion_projected = tuple(
        _project_geometry(geometry, to_projected)
        for _, geometry in exclusions
    )
    for (name, _), geometry in zip(exclusions, exclusion_projected):
        outside_area = geometry.difference(boundary_projected_polygon).area
        if outside_area > 1.0e-4:
            raise KmlInputError(
                f"exclusion {name!r} extends outside the boundary by "
                f"{outside_area:.6f} m^2"
            )

    projected_supplied_partitions = {
        partition_id: _project_geometry(geometry, to_projected)
        for partition_id, geometry in partitions.items()
    }
    for partition_id, geometry in projected_supplied_partitions.items():
        outside_area = geometry.difference(boundary_projected_polygon).area
        if outside_area > 1.0e-4:
            raise KmlInputError(
                f"PARTITION_{partition_id} extends outside the boundary by "
                f"{outside_area:.6f} m^2"
            )

    try:
        safe_area = create_safe_area(
            boundary_projected_polygon,
            exclusion_projected,
            clearance_m,
        )
        route_space = create_operational_route_space(
            safe_area,
            tracking_margin_m,
        )
    except GeometryCoreError as exc:
        raise KmlInputError(str(exc)) from exc

    home_lon, home_lat, explicit_home = _select_home(
        point_features,
        route_space,
        to_projected,
        to_wgs84,
    )

    return KmlMissionInput(
        source_path=source,
        planning_crs=planning_crs.to_string(),
        boundary_wgs84=boundary,
        exclusions_wgs84=tuple(exclusions),
        supplied_partitions_wgs84=tuple(sorted(partitions.items())),
        home_longitude_deg=home_lon,
        home_latitude_deg=home_lat,
        home_was_explicit=explicit_home,
        boundary_projected=boundary_projected_polygon,
        exclusions_projected=exclusion_projected,
        safe_area_projected=safe_area,
        route_space_projected=route_space,
    )
