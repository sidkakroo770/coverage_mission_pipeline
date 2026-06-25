#!/usr/bin/env python3
"""Tests for the synthetic live-service smoke request."""

import pytest

from coverage_mission_pipeline.plan_coverage_smoke_client import (
    build_smoke_request,
)
from coverage_mission_pipeline.planning_request import PlanningRequestError


def test_builds_valid_synthetic_request():
    request = build_smoke_request(
        altitude_m=5.0,
        footprint_m=1.0,
        overlap=0.1,
    )
    assert request.request_id == "smoke_request_1"
    assert request.component.component_id == "smoke_component_1"
    assert request.component.frame.frame_id == "map"
    assert request.component.polygon.area == pytest.approx(200.0)
    assert request.start.x_m == 1.0
    assert request.goal.x_m == 19.0


def test_smoke_request_uses_explicit_parameters():
    request = build_smoke_request(
        altitude_m=12.5,
        footprint_m=2.0,
        overlap=0.25,
    )
    assert request.altitude_m == 12.5
    assert request.lateral_footprint_m == 2.0
    assert request.lateral_overlap == 0.25


def test_smoke_request_rejects_invalid_footprint():
    with pytest.raises(PlanningRequestError):
        build_smoke_request(altitude_m=5.0, footprint_m=0.0, overlap=0.1)


def test_smoke_request_rejects_invalid_overlap():
    with pytest.raises(PlanningRequestError):
        build_smoke_request(altitude_m=5.0, footprint_m=1.0, overlap=1.0)
