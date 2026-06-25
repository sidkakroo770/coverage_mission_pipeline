#!/usr/bin/env python3
"""Sequential fail-closed execution of validated PlanCoverage requests.

The orchestration layer is transport-agnostic and unit-testable without ROS.
The RclpyPlanCoverageTransport adapter performs actual service discovery and
one synchronous call at a time.  A timeout or any malformed response aborts
the batch; no subsequent request is sent.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Any, Iterable, Optional, Protocol, Sequence

from .planning_request import CoveragePlanningRequest
from .planning_result import (
    CoveragePlanningResult,
    PlanningResultError,
    plan_coverage_response_to_result,
)
from .ros_plan_coverage_conversion import planning_request_to_ros


class PlanCoverageClientError(RuntimeError):
    """Base error for transport and sequential execution failures."""


class PlannerServiceUnavailableError(PlanCoverageClientError):
    """Raised when the service is not available before the configured deadline."""


class PlannerRequestTimeoutError(PlanCoverageClientError):
    """Raised when one service request does not complete before its deadline."""


class PlannerTransportError(PlanCoverageClientError):
    """Raised when the ROS transport fails independently of planner success."""


class SequentialPlanningError(PlanCoverageClientError):
    """Fail-closed batch error retaining completed results for diagnostics only."""

    def __init__(
        self,
        message: str,
        *,
        failed_request_id: Optional[str],
        failed_index: Optional[int],
        completed_results: Sequence[CoveragePlanningResult],
    ) -> None:
        super().__init__(message)
        self.failed_request_id = failed_request_id
        self.failed_index = failed_index
        self.completed_results = tuple(completed_results)


class PlanCoverageTransport(Protocol):
    """Minimal transport required by the sequential runner."""

    def wait_for_service(self, timeout_s: float) -> bool:
        ...

    def call(
        self,
        request: CoveragePlanningRequest,
        timeout_s: float,
    ) -> Any:
        ...


_NODE_NAME_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _positive_finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlanCoverageClientError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise PlanCoverageClientError(
            f"{name} must be finite and greater than zero"
        )
    return result


@dataclass(frozen=True)
class SequentialClientConfig:
    """Validated service and timeout configuration."""

    service_name: str = "/plan_coverage"
    service_wait_timeout_s: float = 10.0
    request_timeout_s: float = 30.0
    node_name: str = "coverage_mission_pipeline_client"

    def __post_init__(self) -> None:
        if not isinstance(self.service_name, str) or not self.service_name.strip():
            raise PlanCoverageClientError("service_name must not be empty")
        object.__setattr__(self, "service_name", self.service_name.strip())
        object.__setattr__(
            self,
            "service_wait_timeout_s",
            _positive_finite(
                self.service_wait_timeout_s,
                "service_wait_timeout_s",
            ),
        )
        object.__setattr__(
            self,
            "request_timeout_s",
            _positive_finite(self.request_timeout_s, "request_timeout_s"),
        )
        if (
            not isinstance(self.node_name, str)
            or not _NODE_NAME_PATTERN.fullmatch(self.node_name)
        ):
            raise PlanCoverageClientError(
                f"node_name must match {_NODE_NAME_PATTERN.pattern!r}"
            )


class SequentialPlanCoverageRunner:
    """Execute a finite request batch strictly one request at a time."""

    def __init__(
        self,
        transport: PlanCoverageTransport,
        config: SequentialClientConfig | None = None,
    ) -> None:
        if transport is None:
            raise PlanCoverageClientError("transport must not be None")
        self.transport = transport
        self.config = config or SequentialClientConfig()

    @staticmethod
    def _validated_batch(
        requests: Iterable[CoveragePlanningRequest],
    ) -> tuple[CoveragePlanningRequest, ...]:
        try:
            batch = tuple(requests)
        except TypeError as exc:
            raise PlanCoverageClientError("requests must be iterable") from exc
        if not batch:
            raise PlanCoverageClientError("requests must not be empty")
        if any(not isinstance(item, CoveragePlanningRequest) for item in batch):
            raise PlanCoverageClientError(
                "every batch item must be a CoveragePlanningRequest"
            )

        request_ids = [item.request_id for item in batch]
        if len(set(request_ids)) != len(request_ids):
            raise PlanCoverageClientError(
                "request IDs must be unique within a batch"
            )
        component_ids = [item.component.component_id for item in batch]
        if len(set(component_ids)) != len(component_ids):
            raise PlanCoverageClientError(
                "component IDs must be unique within a batch"
            )
        return batch

    def run(
        self,
        requests: Iterable[CoveragePlanningRequest],
    ) -> tuple[CoveragePlanningResult, ...]:
        batch = self._validated_batch(requests)
        completed: list[CoveragePlanningResult] = []

        try:
            available = self.transport.wait_for_service(
                self.config.service_wait_timeout_s
            )
        except Exception as exc:
            raise SequentialPlanningError(
                f"service discovery failed: {exc}",
                failed_request_id=None,
                failed_index=None,
                completed_results=completed,
            ) from exc
        if not available:
            raise SequentialPlanningError(
                "coverage service was unavailable before the deadline",
                failed_request_id=None,
                failed_index=None,
                completed_results=completed,
            )

        for index, request in enumerate(batch):
            try:
                raw_response = self.transport.call(
                    request,
                    self.config.request_timeout_s,
                )
                result = plan_coverage_response_to_result(
                    request,
                    raw_response,
                )
            except Exception as exc:
                raise SequentialPlanningError(
                    f"request {request.request_id!r} failed: {exc}",
                    failed_request_id=request.request_id,
                    failed_index=index,
                    completed_results=completed,
                ) from exc
            completed.append(result)

        return tuple(completed)


class RclpyPlanCoverageTransport:
    """Actual rclpy transport for a standalone, non-spinning client node.

    The supplied node must not already be managed by another spinning executor
    while call() is running.  The runner aborts the batch after a timeout, so it
    never sends a later request while the timed-out server call may still exist.
    """

    def __init__(self, node: Any, service_name: str = "/plan_coverage") -> None:
        if node is None:
            raise PlanCoverageClientError("node must not be None")
        if not isinstance(service_name, str) or not service_name.strip():
            raise PlanCoverageClientError("service_name must not be empty")
        try:
            from polygon_coverage_msgs.srv import PlanCoverage
        except ImportError as exc:
            raise PlanCoverageClientError(
                "PlanCoverage interface is unavailable; source the ROS 2 and "
                "coverage workspace setup files"
            ) from exc
        try:
            self._client = node.create_client(
                PlanCoverage,
                service_name.strip(),
            )
        except Exception as exc:
            raise PlanCoverageClientError(
                f"failed to create service client: {exc}"
            ) from exc
        self._node = node

    def wait_for_service(self, timeout_s: float) -> bool:
        timeout = _positive_finite(timeout_s, "timeout_s")
        try:
            return bool(self._client.wait_for_service(timeout_sec=timeout))
        except Exception as exc:
            raise PlannerTransportError(
                f"wait_for_service failed: {exc}"
            ) from exc

    def call(
        self,
        request: CoveragePlanningRequest,
        timeout_s: float,
    ) -> Any:
        timeout = _positive_finite(timeout_s, "timeout_s")
        if not isinstance(request, CoveragePlanningRequest):
            raise PlannerTransportError(
                "request must be a CoveragePlanningRequest"
            )
        try:
            import rclpy

            stamp = self._node.get_clock().now().to_msg()
            ros_request = planning_request_to_ros(request, stamp=stamp)
            future = self._client.call_async(ros_request)
            rclpy.spin_until_future_complete(
                self._node,
                future,
                timeout_sec=timeout,
            )
        except Exception as exc:
            raise PlannerTransportError(
                f"service invocation failed: {exc}"
            ) from exc

        if not future.done():
            future.cancel()
            raise PlannerRequestTimeoutError(
                f"request timed out after {timeout:.3f} seconds"
            )
        if future.cancelled():
            raise PlannerTransportError("service future was cancelled")

        try:
            exception = future.exception()
        except Exception as exc:
            raise PlannerTransportError(
                f"could not inspect service future: {exc}"
            ) from exc
        if exception is not None:
            raise PlannerTransportError(
                f"service future failed: {exception}"
            ) from exception

        try:
            response = future.result()
        except Exception as exc:
            raise PlannerTransportError(
                f"could not read service response: {exc}"
            ) from exc
        if response is None:
            raise PlannerTransportError("service returned no response")
        return response
