# Coverage Mission Pipeline

Higher-level mission geometry preparation and orchestration for the
`polygon_coverage_ros2` planner.

## Current scope

The repository currently contains eight layers:

1. **Geometry core**
   - applies clearance to the global mission boundary and exclusions;
   - leaves shared partition boundaries unbuffered;
   - clips partitions to the global safe area;
   - preserves every connected polygon component;
   - rejects invalid geometry explicitly.

2. **Prepared-component contract**
   - represents exactly one connected Polygon per planner request;
   - records the local Cartesian frame and projected origin;
   - supports optional vehicle assignment;
   - serializes to deterministic, strict JSON;
   - rejects MultiPolygons, unknown fields and unsafe identifiers.

3. **ROS polygon conversion**
   - converts one prepared component to `PolygonWithHolesStamped`;
   - preserves hull and hole orientation;
   - uses the component's actual frame ID;
   - validates altitude, timestamp and Point32 representability;
   - has no ROS node and makes no service call.

4. **Coverage-planning request model**
   - requires explicit start and goal points;
   - validates that both lie in connected free space, not inside holes;
   - validates altitude, positive footprint and overlap in `[0, 1)`;
   - permits boundary points and identical start/goal points;
   - deliberately does not choose a start/goal policy.

5. **PlanCoverage request conversion**
   - creates a complete generated `PlanCoverage.Request` message;
   - gives polygon, start and goal the same frame, stamp and altitude;
   - sets valid identity quaternions.

6. **PlanCoverage response validation**
   - rejects explicit planner failures and malformed success responses;
   - validates frame consistency, finite waypoints and constant altitude;
   - detaches ROS messages into immutable route-result objects.

7. **Sequential fail-closed client**
   - waits for `/plan_coverage` with a finite deadline;
   - sends exactly one request at a time in deterministic input order;
   - aborts the entire batch after the first timeout, rejection or bad response;
   - never sends a later request after an uncertain timed-out call;
   - retains completed results only for failure diagnostics.

8. **Live synthetic smoke client**
   - sends one explicit 20 m by 10 m rectangle to the real service;
   - does not depend on `mission_output.json`;
   - prints the returned waypoint count or exits nonzero on failure.

The package does not yet parse `mission_output.json`, choose production
start/goal poses, join disconnected route groups or generate flight missions.

## Unit tests

```bash
python3 -m pytest -v test
```

## Live service smoke test

Start the planner in one sourced terminal:

```bash
ros2 run polygon_coverage_ros2 coverage_planner
```

Then, in a second sourced terminal:

```bash
ros2 run coverage_mission_pipeline plan_coverage_smoke_client
```
