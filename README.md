# Coverage Mission Pipeline

Higher-level mission geometry preparation and orchestration for the
`polygon_coverage_ros2` planner.

## Current scope

The repository currently contains five layers:

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
   - sets valid identity quaternions;
   - performs no service discovery or service call.

The package does not yet parse `mission_output.json`, choose start/goal poses,
call `/plan_coverage`, join route groups or generate flight missions.

## Test

```bash
python3 -m pytest -v test
```
