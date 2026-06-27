# Coverage Mission Pipeline

Higher-level mission geometry preparation and orchestration for the
`polygon_coverage_ros2` planner.

## Current scope

The repository currently contains eighteen layers:

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

14. **Complete vehicle-route record**
    - serializes the optimized per-vehicle route to deterministic strict JSON;
    - stores one authoritative local-coordinate waypoint array;
    - represents routes and connectors as inclusive spans into that array;
    - preserves component order, route direction and connector algorithms;
    - validates span continuity, endpoint chains, path lengths and cruise altitude;
    - derives projected and WGS84 waypoints on demand from the stored frame;
    - supports idle vehicles and atomic file writing;
    - avoids duplicated coordinate arrays that could drift out of sync.

15. **ArduPilot mission model and exporter**
    - converts a complete vehicle-route record to WGS84 Copter mission items;
    - interprets stored route altitude explicitly as home-relative altitude;
    - uses `MAV_FRAME_GLOBAL_RELATIVE_ALT` for every exported item;
    - supports TAKEOFF, WAYPOINT, RTL and reference-only LAND termination;
    - validates consecutive sequence numbers, current flags and command parameters;
    - serializes deterministic strict mission JSON;
    - exports and parses the twelve-field `QGC WPL 110` waypoint format;
    - rejects idle vehicles instead of silently omitting them;
    - writes JSON and `.waypoints` files atomically.

16. **Generic end-to-end mission pipeline**
    - accepts prepared components, explicit vehicle references, explicit planning specs and authoritative free-space geometry;
    - deterministically orders components and builds validated planner requests;
    - executes an injected sequential planner without coupling orchestration to a ROS graph;
    - verifies exact request/result identity and ordering before accepting planner output;
    - creates component route records, complete vehicle routes and complete route records;
    - builds one ArduPilot mission for every active vehicle while preserving idle vehicles explicitly;
    - publishes component routes, complete routes, mission JSON, QGC WPL 110 files and a deterministic manifest;
    - atomically publishes the output directory only after every artifact succeeds;
    - remains independent of any mission-specific input adapter.

17. **Swarm-Partitions JSON adapter**
    - consumes the bug-fixed JSON contract exported by `atissss/Swarm-Partitions`;
    - validates separate coordinate and planning CRS metadata and longitude/latitude axis order;
    - reconstructs Polygon and MultiPolygon geometry while preserving every interior hole;
    - accepts the currently committed exporter with no `dynamic` field and also supports that field when re-enabled;
    - applies clearance globally to the mission boundary and no-go zones, never to shared partition borders;
    - clips each unbuffered partition to the global safe area and preserves every connected component;
    - rejects partition overlaps, missing coverage, malformed rings, unsafe CRS metadata and incomplete assignments;
    - requires explicit partition-to-vehicle assignments, vehicle references and coverage-planner parameters;
    - produces a complete `GenericMissionDefinition` and can invoke the Stage 13 pipeline directly.

18. **Operational mission configuration**
    - stores the adapter and generic-pipeline policies in strict versioned JSON or YAML;
    - defines partition assignments, vehicle references, coverage altitude, footprint and overlap without editing Python;
    - defines global clearance, component-area and coverage-validation tolerances;
    - configures A* visibility-node limits, optional return-to-reference and idle-vehicle policy;
    - configures ArduPilot takeoff, waypoint hold, minimum altitude and terminal action;
    - validates cross-layer constraints such as LAND requiring a return-to-reference route;
    - rejects unknown fields and unsafe YAML tags;
    - writes configuration files atomically and canonicalizes assignment and vehicle ordering.

19. **Production ROS mission CLI**
    - consumes a Swarm-Partitions JSON file, operational JSON/YAML config and new output directory;
    - validates both input contracts completely before initializing ROS;
    - creates a dedicated ROS node and calls `/plan_coverage` strictly sequentially;
    - exposes explicit service discovery and per-request deadlines;
    - destroys the node and shuts down ROS on success or failure;
    - never overwrites an existing output path;
    - publishes the complete artifact directory atomically only after every stage succeeds;
    - records SHA-256 hashes of both inputs, normalized configuration, ROS client settings and run summaries.

The package does not yet upload missions to vehicles or execute SITL verification.

## Operational configuration

Copy and edit the checked-in example instead of constructing adapter objects in
Python:

```bash
cp config/swarm_mission.example.yaml swarm_mission.yaml
```

Load it with:

```python
from coverage_mission_pipeline import load_swarm_mission_operational_config

config = load_swarm_mission_operational_config("swarm_mission.yaml")
adapter_config = config.adapter
pipeline_config = config.pipeline
```

The same file will be consumed by the production ROS command-line entry point.
Both `.json` and `.yaml`/`.yml` are supported.

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


## Production ROS mission command

Start the planner in one sourced terminal:

```bash
ros2 run polygon_coverage_ros2 coverage_planner
```

Then run the complete pipeline from another sourced terminal:

```bash
ros2 run coverage_mission_pipeline run_swarm_mission \
  --mission-json /path/to/mission_output.json \
  --config /path/to/swarm_mission.yaml \
  --output /path/to/new-output-directory
```

The output directory must not already exist. The command validates geometry and
configuration before starting ROS, calls the planner one component at a time,
and publishes the final directory only after every route and ArduPilot export
succeeds. In addition to the generic mission manifest, the bundle contains
`production-run.json` and `operational-config.normalized.json` for traceability.
