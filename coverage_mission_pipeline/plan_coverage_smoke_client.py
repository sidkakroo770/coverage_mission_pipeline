#!/usr/bin/env python3
"""Live one-request smoke test for the /plan_coverage service."""

from __future__ import annotations

import argparse
import sys
from typing import Sequence

from shapely.geometry import Polygon

from .planning_request import CoveragePlanningRequest, LocalPoint2D
from .prepared_component import LocalCartesianFrame, PreparedComponent
from .sequential_plan_coverage_client import (
    RclpyPlanCoverageTransport,
    SequentialClientConfig,
    SequentialPlanCoverageRunner,
)


def build_smoke_request(
    *,
    altitude_m: float,
    footprint_m: float,
    overlap: float,
) -> CoveragePlanningRequest:
    component = PreparedComponent(
        component_id="smoke_component_1",
        source_region_id="smoke_region",
        component_index=1,
        assigned_vehicle_id=None,
        frame=LocalCartesianFrame(
            frame_id="map",
            projected_crs="EPSG:32643",
            origin_easting_m=0.0,
            origin_northing_m=0.0,
        ),
        polygon=Polygon(
            [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]
        ),
    )
    return CoveragePlanningRequest.for_component(
        component,
        request_id="smoke_request_1",
        start=LocalPoint2D(1.0, 1.0),
        goal=LocalPoint2D(19.0, 1.0),
        altitude_m=altitude_m,
        lateral_footprint_m=footprint_m,
        lateral_overlap=overlap,
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service-name", default="/plan_coverage")
    parser.add_argument("--service-wait-timeout", type=float, default=10.0)
    parser.add_argument("--request-timeout", type=float, default=30.0)
    parser.add_argument("--altitude", type=float, default=5.0)
    parser.add_argument("--footprint", type=float, default=1.0)
    parser.add_argument("--overlap", type=float, default=0.10)
    args = parser.parse_args(argv)

    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        print(
            "FAILED: rclpy is unavailable; source /opt/ros/humble/setup.zsh",
            file=sys.stderr,
        )
        return 1

    node = None
    try:
        config = SequentialClientConfig(
            service_name=args.service_name,
            service_wait_timeout_s=args.service_wait_timeout,
            request_timeout_s=args.request_timeout,
            node_name="coverage_plan_smoke_client",
        )
        request = build_smoke_request(
            altitude_m=args.altitude,
            footprint_m=args.footprint,
            overlap=args.overlap,
        )

        rclpy.init(args=None)
        node = Node(config.node_name)
        transport = RclpyPlanCoverageTransport(
            node,
            service_name=config.service_name,
        )
        result = SequentialPlanCoverageRunner(
            transport,
            config,
        ).run([request])[0]
        print(
            "PASS live PlanCoverage smoke test: "
            f"{len(result.waypoints)} waypoints; "
            f"message={result.response_message!r}"
        )
        return 0
    except Exception as exc:
        print(f"FAILED live PlanCoverage smoke test: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
