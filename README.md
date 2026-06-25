# Coverage Mission Pipeline

Higher-level mission geometry preparation and orchestration for the
`polygon_coverage_ros2` planner.

## Current scope

The repository currently contains thirteen layers:

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

9. **Explicit-anchor start/goal policy**
   - accepts start and goal reference anchors in the component local frame;
   - keeps already-feasible anchors unchanged;
   - projects external or hole-contained anchors to the nearest feasible point;
   - supports an explicit minimum distance from hull and hole boundaries;
   - can enforce a minimum selected start/goal separation;
   - does not infer vehicle homes, component order or route connectors.

10. **Georeferenced route-record contract**
    - attaches the complete local Cartesian frame to each successful route;
    - serializes one authoritative local-coordinate waypoint sequence;
    - derives projected coordinates by restoring the recorded origin;
    - converts projected coordinates to and from WGS84 longitude/latitude;
    - preserves waypoint order and altitude without silently dropping results;
    - writes deterministic strict JSON atomically.

11. **Vehicle references and component visit ordering**
    - stores explicit home, launch, current-position or custom references;
    - accepts local, projected or WGS84 reference coordinates;
    - validates exact local-frame compatibility with assigned components;
    - rejects duplicate IDs, missing assignments and positive-area overlaps;
    - greedily orders every component by straight-line distance lower bound;
    - resolves equal-distance candidates by component ID, independent of input order;
    - returns an idle plan for referenced vehicles with no assigned components;
    - records proxy transition points but does not construct flight connectors.

12. **Flight-safe route connectors**
    - uses a direct segment when the entire segment lies in one free-space component;
    - otherwise builds a polygon-vertex visibility graph and runs deterministic A*;
    - naturally chooses the shorter clockwise or anticlockwise side for one exclusion;
    - handles multiple exclusions and concave mission boundaries without a grid resolution;
    - treats the supplied safe area as authoritative and never applies clearance twice;
    - fails closed when endpoints are disconnected or graph complexity exceeds its limit;
    - joins ordered route records at one common altitude without duplicate endpoints;
    - validates every route waypoint and segment before assembly.

13. **Complete per-vehicle route assembly**
    - preserves the component order selected by the vehicle-ordering layer;
    - treats every component route as a forward/reversed binary choice;
    - uses dynamic programming to minimize the global connector total for that order;
    - includes the vehicle-reference-to-first-route connector in optimization;
    - optionally includes the final route-to-reference connector;
    - plans every transition through the authoritative free-space geometry;
    - resolves equal-cost orientation sequences deterministically in favour of forward order;
    - assembles one continuous cruise-altitude waypoint sequence without duplicate joins;
    - supports idle vehicles without fabricating waypoints or connectors.

The package does not yet parse `mission_output.json`, parse a mission-specific vehicle/reference schema, serialize complete vehicle-route assemblies, or generate ArduPilot flight missions.

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
