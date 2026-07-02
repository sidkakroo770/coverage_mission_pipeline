#!/usr/bin/env python3
"""KML overlay generation from strict product inputs."""

from __future__ import annotations

import html
import json
from pathlib import Path
from typing import Any


_PARTITION_COLOURS = (
    "550000ff",
    "5500ff00",
    "55ff0000",
    "5500ffff",
    "55ff00ff",
)


def _coordinates(points: list[list[float]], altitude: float = 0.0) -> str:
    return " ".join(
        f"{float(longitude):.15f},{float(latitude):.15f},{altitude:.3f}"
        for longitude, latitude in points
    )


def _polygon_placemark(name: str, style: str, record: dict[str, Any]) -> str:
    holes = "".join(
        "<innerBoundaryIs><LinearRing><coordinates>"
        + _coordinates(hole)
        + "</coordinates></LinearRing></innerBoundaryIs>"
        for hole in record["holes"]
    )
    return (
        "<Placemark><name>" + html.escape(name) + "</name><styleUrl>#" + style
        + "</styleUrl><Polygon><outerBoundaryIs><LinearRing><coordinates>"
        + _coordinates(record["exterior"])
        + "</coordinates></LinearRing></outerBoundaryIs>"
        + holes
        + "</Polygon></Placemark>"
    )


def write_input_overlay(
    mission_output: dict[str, Any],
    operational_config: dict[str, Any],
    destination: Path | str,
) -> Path:
    path = Path(destination).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)

    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>',
        '<name>Coverage swarm input</name>',
        '<Style id="boundary"><LineStyle><color>ffffffff</color><width>4</width></LineStyle><PolyStyle><color>1200ffff</color></PolyStyle></Style>',
        '<Style id="exclusion"><LineStyle><color>ff0000ff</color><width>4</width></LineStyle><PolyStyle><color>550000ff</color></PolyStyle></Style>',
        '<Style id="home"><IconStyle><scale>1.3</scale></IconStyle></Style>',
    ]
    for index, colour in enumerate(_PARTITION_COLOURS, start=1):
        parts.append(
            f'<Style id="partition-{index}"><LineStyle><color>{colour}</color><width>3</width></LineStyle><PolyStyle><color>{colour}</color></PolyStyle></Style>'
        )

    for index, record in enumerate(mission_output["boundary"], start=1):
        parts.append(_polygon_placemark(f"BOUNDARY {index}", "boundary", record))
    for zone in mission_output["no_go_zones"]["predetermined"]:
        for index, record in enumerate(zone["geometry"], start=1):
            parts.append(
                _polygon_placemark(
                    f"NO_GO: {zone['name']} {index}",
                    "exclusion",
                    record,
                )
            )
    for partition in mission_output["partitions"]:
        partition_id = int(partition["id"])
        for component_index, record in enumerate(partition["geometry"], start=1):
            parts.append(
                _polygon_placemark(
                    f"PARTITION_{partition_id} component {component_index}",
                    f"partition-{partition_id}",
                    record,
                )
            )

    reference = operational_config["vehicles"][0]["reference"]
    parts.append(
        "<Placemark><name>HOME</name><styleUrl>#home</styleUrl><Point><coordinates>"
        f"{float(reference['longitude_deg']):.9f},{float(reference['latitude_deg']):.9f},0"
        "</coordinates></Point></Placemark>"
    )
    parts.append("</Document></kml>")
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")
    return path
