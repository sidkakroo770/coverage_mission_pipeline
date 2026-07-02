# Architecture

## End-to-end flow

1. Parse boundary, exclusions, partitions, CRS metadata and assignments.
2. Construct global safe space; do not buffer shared partition borders.
3. Clip each partition and preserve every connected component.
4. Send one connected polygon per `PlanCoverage` request.
5. Convert planner output into immutable georeferenced route records.
6. Order components and optimize forward/reversed route orientation.
7. Connect transitions through authoritative free space using direct segments or a polygon-vertex visibility graph with deterministic A*.
8. Build one continuous route per vehicle, including the return to HOME.
9. Export TAKEOFF, WAYPOINT and terminal LAND mission items.
10. Upload over independent MAVLink endpoints, download and compare fingerprints.
11. Run a no-arming identity, HOME, position and mission-count preflight.
12. Execute with a staggered shared-HOME launch gate and separation monitoring.

## Important contracts

### Geometry

Clipping can turn one partition into several disconnected polygons. All components are retained. The planner receives exactly one connected Polygon at a time.

### Safe connectors

The supplied free-space geometry is authoritative. Connectors never apply clearance a second time. A direct segment is used only when the complete segment is contained; otherwise, deterministic visibility-graph A* routes around polygon vertices.

### Semantic versus wire missions

The semantic mission starts with TAKEOFF. MAVLink transport prepends synthetic HOME at wire sequence 0. E7 coordinates remain authoritative, and readback normalization is command-specific and tested.

### Terminal action

Implicit RTL is not considered a safe connector. The mission must geometrically return to HOME before explicit LAND.

## Single-terminal process model

```text
coverage-swarm
├── sim_vehicle.py -n 5
├── five MAVProxy routers
├── observer-only MAVProxy map
├── transactional uploader
├── no-arming preflight
└── staggered fleet executor
```

The checked-in demo assets are installed into the ROS package share directory, so a fresh clone does not depend on the developer's private `mission_runs` tree.
