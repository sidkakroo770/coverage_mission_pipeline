# Coverage Mission Pipeline

Higher-level mission geometry preparation and orchestration for the
`polygon_coverage_ros2` planner.

## Stage 1

The repository currently contains a ROS-independent geometry core that:

- applies clearance to the global mission boundary and exclusions;
- leaves shared partition boundaries unbuffered;
- clips partitions to the global safe area;
- preserves every connected polygon component;
- rejects invalid geometry explicitly.

## Test

```bash
python3 -m pytest -v test/test_mission_geometry_core.py
```
