# Coverage Mission Pipeline

Higher-level mission geometry preparation and orchestration for the
`polygon_coverage_ros2` planner.

## Current scope

The repository currently contains two ROS-independent layers:

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

The package does not yet parse `mission_output.json`, call ROS services, select
start/goal poses or plan routes.

## Test

```bash
python3 -m pytest -v test
```
