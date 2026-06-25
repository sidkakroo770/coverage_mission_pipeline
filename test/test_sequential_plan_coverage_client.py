#!/usr/bin/env python3
"""Tests for fail-closed sequential planner orchestration."""

import sys
import types

import pytest
from shapely.geometry import Polygon

from coverage_mission_pipeline.planning_request import (
    CoveragePlanningRequest,
    LocalPoint2D,
)
from coverage_mission_pipeline.prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
)
from coverage_mission_pipeline.sequential_plan_coverage_client import (
    PlanCoverageClientError,
    PlannerRequestTimeoutError,
    RclpyPlanCoverageTransport,
    SequentialClientConfig,
    SequentialPlanCoverageRunner,
    SequentialPlanningError,
)


class FakeHeader:
    def __init__(self):
        self.frame_id = "map"


class FakePosition:
    def __init__(self, x, y, z=5.0):
        self.x = x
        self.y = y
        self.z = z


class FakePose:
    def __init__(self, x, y, z=5.0):
        self.position = FakePosition(x, y, z)


class FakePoseArray:
    def __init__(self):
        self.header = FakeHeader()
        self.poses = [FakePose(1, 1), FakePose(2, 2)]


class FakeResponse:
    def __init__(self, success=True, message="planned"):
        self.success = success
        self.message = message
        self.waypoints = FakePoseArray()


def make_request(index, *, request_id=None, component_id=None):
    component = PreparedComponent(
        component_id or f"component-{index}",
        f"region-{index}",
        1,
        LocalCartesianFrame("map", "EPSG:32643", 0, 0),
        Polygon([(0, 0), (10, 0), (10, 10), (0, 10)]),
        f"drone-{index}",
    )
    return CoveragePlanningRequest(
        request_id or f"request-{index}",
        component,
        LocalPoint2D(1, 1),
        LocalPoint2D(9, 1),
        5.0,
        1.0,
        0.1,
    )


class FakeTransport:
    def __init__(self, responses=None, available=True, wait_error=None):
        self.responses = list(responses or [])
        self.available = available
        self.wait_error = wait_error
        self.wait_timeouts = []
        self.calls = []
        self.active = False

    def wait_for_service(self, timeout_s):
        self.wait_timeouts.append(timeout_s)
        if self.wait_error:
            raise self.wait_error
        return self.available

    def call(self, request, timeout_s):
        assert not self.active
        self.active = True
        try:
            self.calls.append((request.request_id, timeout_s))
            item = self.responses.pop(0)
            if isinstance(item, BaseException):
                raise item
            return item
        finally:
            self.active = False


def test_config_defaults():
    config = SequentialClientConfig()
    assert config.service_name == "/plan_coverage"
    assert config.service_wait_timeout_s == 10.0
    assert config.request_timeout_s == 30.0


@pytest.mark.parametrize("value", [0, -1, float("inf"), True, "1"])
def test_config_rejects_invalid_wait_timeout(value):
    with pytest.raises(PlanCoverageClientError):
        SequentialClientConfig(service_wait_timeout_s=value)


@pytest.mark.parametrize("value", [0, -1, float("nan"), None])
def test_config_rejects_invalid_request_timeout(value):
    with pytest.raises(PlanCoverageClientError):
        SequentialClientConfig(request_timeout_s=value)


@pytest.mark.parametrize("name", ["", "bad-name", "9bad", "has space"])
def test_config_rejects_invalid_node_name(name):
    with pytest.raises(PlanCoverageClientError, match="node_name"):
        SequentialClientConfig(node_name=name)


def test_runner_rejects_empty_batch():
    with pytest.raises(PlanCoverageClientError, match="must not be empty"):
        SequentialPlanCoverageRunner(FakeTransport()).run([])


def test_runner_rejects_nonrequest_item():
    with pytest.raises(PlanCoverageClientError, match="batch item"):
        SequentialPlanCoverageRunner(FakeTransport()).run([object()])


def test_runner_rejects_duplicate_request_ids():
    requests = [make_request(1), make_request(2, request_id="request-1")]
    with pytest.raises(PlanCoverageClientError, match="request IDs"):
        SequentialPlanCoverageRunner(FakeTransport()).run(requests)


def test_runner_rejects_duplicate_component_ids():
    requests = [make_request(1), make_request(2, component_id="component-1")]
    with pytest.raises(PlanCoverageClientError, match="component IDs"):
        SequentialPlanCoverageRunner(FakeTransport()).run(requests)


def test_runner_fails_when_service_unavailable():
    transport = FakeTransport(available=False)
    with pytest.raises(SequentialPlanningError) as captured:
        SequentialPlanCoverageRunner(transport).run([make_request(1)])
    assert captured.value.failed_request_id is None
    assert transport.calls == []


def test_runner_wraps_service_discovery_exception():
    transport = FakeTransport(wait_error=RuntimeError("discovery broke"))
    with pytest.raises(SequentialPlanningError, match="discovery broke"):
        SequentialPlanCoverageRunner(transport).run([make_request(1)])


def test_runner_executes_in_input_order():
    requests = [make_request(1), make_request(2), make_request(3)]
    transport = FakeTransport([FakeResponse(), FakeResponse(), FakeResponse()])
    results = SequentialPlanCoverageRunner(transport).run(requests)
    assert [result.request_id for result in results] == [
        "request-1",
        "request-2",
        "request-3",
    ]
    assert [call[0] for call in transport.calls] == [
        "request-1",
        "request-2",
        "request-3",
    ]


def test_runner_uses_configured_timeouts():
    config = SequentialClientConfig(
        service_wait_timeout_s=4.5,
        request_timeout_s=8.25,
    )
    transport = FakeTransport([FakeResponse()])
    SequentialPlanCoverageRunner(transport, config).run([make_request(1)])
    assert transport.wait_timeouts == [4.5]
    assert transport.calls == [("request-1", 8.25)]


def test_runner_stops_after_first_failed_response():
    requests = [make_request(1), make_request(2), make_request(3)]
    transport = FakeTransport(
        [FakeResponse(), FakeResponse(False, "planner failed"), FakeResponse()]
    )
    with pytest.raises(SequentialPlanningError) as captured:
        SequentialPlanCoverageRunner(transport).run(requests)
    error = captured.value
    assert error.failed_request_id == "request-2"
    assert error.failed_index == 1
    assert len(error.completed_results) == 1
    assert [call[0] for call in transport.calls] == ["request-1", "request-2"]


def test_runner_stops_after_timeout():
    requests = [make_request(1), make_request(2), make_request(3)]
    transport = FakeTransport(
        [FakeResponse(), PlannerRequestTimeoutError("late"), FakeResponse()]
    )
    with pytest.raises(SequentialPlanningError, match="late") as captured:
        SequentialPlanCoverageRunner(transport).run(requests)
    assert captured.value.failed_request_id == "request-2"
    assert len(captured.value.completed_results) == 1
    assert len(transport.calls) == 2


def test_runner_wraps_transport_exception():
    transport = FakeTransport([RuntimeError("socket broke")])
    with pytest.raises(SequentialPlanningError, match="socket broke"):
        SequentialPlanCoverageRunner(transport).run([make_request(1)])


def test_rclpy_transport_rejects_missing_node():
    with pytest.raises(PlanCoverageClientError, match="node"):
        RclpyPlanCoverageTransport(None)


def test_rclpy_transport_with_fake_ros(monkeypatch):
    class FakePlanCoverage:
        pass

    srv_module = types.ModuleType("polygon_coverage_msgs.srv")
    srv_module.PlanCoverage = FakePlanCoverage
    package_module = types.ModuleType("polygon_coverage_msgs")
    package_module.srv = srv_module
    monkeypatch.setitem(sys.modules, "polygon_coverage_msgs", package_module)
    monkeypatch.setitem(sys.modules, "polygon_coverage_msgs.srv", srv_module)

    class FakeFuture:
        def done(self):
            return True

        def cancelled(self):
            return False

        def exception(self):
            return None

        def result(self):
            return FakeResponse()

    class FakeClient:
        def wait_for_service(self, timeout_sec):
            return timeout_sec == 2.0

        def call_async(self, request):
            return FakeFuture()

    class FakeClockNow:
        def to_msg(self):
            return object()

    class FakeClock:
        def now(self):
            return FakeClockNow()

    class FakeNode:
        def create_client(self, service_type, service_name):
            assert service_type is FakePlanCoverage
            assert service_name == "/plan_coverage"
            return FakeClient()

        def get_clock(self):
            return FakeClock()

    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.spin_until_future_complete = lambda *args, **kwargs: None
    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setattr(
        "coverage_mission_pipeline.sequential_plan_coverage_client.planning_request_to_ros",
        lambda request, stamp: object(),
    )

    transport = RclpyPlanCoverageTransport(FakeNode())
    assert transport.wait_for_service(2.0)
    assert isinstance(transport.call(make_request(1), 3.0), FakeResponse)
