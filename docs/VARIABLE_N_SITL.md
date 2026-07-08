# Arbitrary-N one-terminal ArduCopter SITL

The `feature/variable-n-drones` branch supports a complete KML-to-SITL command:

```zsh
coverage-swarm simulate ~/Downloads/CSED_mission.kml \
  --drones 3 \
  --execute \
  --replace-running
```

Run without `--execute` first to generate all missions, start the SITL fleet,
upload every mission, download every mission again, verify all fingerprints,
and stop without arming:

```zsh
coverage-swarm simulate ~/Downloads/CSED_mission.kml \
  --drones 3 \
  --replace-running
```

## What is dynamic

For `N` vehicles, the supervisor creates:

- `N` equal-route-area partitions when the KML does not supply partitions;
- `drone-1` through `drone-N`;
- `N` boustrophedon coverage routes;
- `N` ArduPilot mission files;
- `N` direct ArduCopter SITL processes;
- system IDs `1..N`;
- SITL TCP ports `5760, 5770, 5780, ...`;
- a run-local fleet upload configuration;
- transactional upload/readback verification for every mission;
- one long-lived MAVLink execution connection per vehicle;
- pairwise airborne separation statistics for every vehicle pair.

MAVLink system ID `255` is reserved for the execution GCS, so the SITL command
accepts at most 254 vehicles. CPU and memory will normally impose a much lower
practical limit.

## Required KML content

Execution requires an explicit HOME placemark. The KML may contain:

- one outer mission boundary;
- zero or more exclusion/no-go zones;
- supplied partitions, or no supplied partitions for automatic equal-area
  partitioning;
- one explicit HOME.

The supervisor refuses execution when generated missions do not all:

- start with TAKEOFF;
- return through a waypoint at the shared HOME;
- end with LAND at HOME;
- use LAND altitude zero;
- avoid RTL commands;
- identify exactly `drone-1..drone-N`.

## SITL transport

Each ArduCopter binary is launched directly instead of through `sim_vehicle.py`
or MAVProxy.

| Vehicle | Instance | System ID | SERIAL0 TCP |
|---|---:|---:|---:|
| drone-1 | 0 | 1 | 5760 |
| drone-2 | 1 | 2 | 5770 |
| drone-3 | 2 | 3 | 5780 |
| drone-N | N-1 | N | 5760 + 10 × (N-1) |

Port readiness is checked passively with `ss`. The supervisor never opens and
closes a MAVLink TCP connection merely to test readiness.

## Flight-readiness sequence

Before any vehicle is armed, the executor verifies every vehicle:

1. is connected at the expected endpoint and identity;
2. is disarmed;
3. contains the verified wire mission item count;
4. reports the expected HOME;
5. accepts requested telemetry intervals;
6. has mission sequence 1 selected;
7. has GPS fix type 3 or better;
8. reports stable absolute or predicted-absolute EKF horizontal position;
9. can enter LOITER while disarmed;
10. reports healthy enabled pre-arm checks.

Immediately before each individual launch, it then:

1. verifies the vehicle is on the ground at HOME;
2. sends centered simulated RC input with throttle at 1000 µs;
3. verifies ArduPilot received RC channel 3 near 1000 µs;
4. arms normally in LOITER;
5. changes to AUTO;
6. sends `MAV_CMD_MISSION_START`;
7. releases the RC override.

The code does not use force arm and does not disable arming checks.

## Shared-HOME launch gate

Vehicles launch in numeric order. Before `drone-K` launches, every previously
launched vehicle must remain:

- armed;
- telemetry-fresh;
- above the configured clearance altitude;
- farther than the configured horizontal distance from HOME.

Defaults:

```text
launch clearance altitude:      12 m
launch clearance HOME distance: 30 m
minimum launch spacing:          5 s
```

For a very small mission area, a vehicle may never become 30 m clear of HOME.
Lower the gate only after inspecting the geometry:

```zsh
coverage-swarm simulate mission.kml \
  --drones 3 \
  --execute \
  --replace-running \
  --launch-clearance-home-distance-m 15
```

If a preceding vehicle finishes and returns to the shared HOME before all
vehicles launch, the supervisor fails closed instead of launching through an
occupied shared takeoff/landing point.

## Separation monitoring

The executor records:

- minimum 3D separation for every airborne vehicle pair;
- the fleet-wide minimum pair;
- horizontal and vertical distances at each minimum;
- warning counts;
- full telemetry CSV;
- event JSONL;
- periodic checkpoint JSON;
- final execution report JSON.

Warnings are observational. This is not an active collision-avoidance system.

Default warning condition:

```text
horizontal separation < 10 m
and
vertical separation < 5 m
```

Tune the reporting thresholds with:

```zsh
--separation-warning-horizontal-m 15
--separation-warning-vertical-m 8
```

## Output

Each run is written under:

```text
~/coverage_ws/mission_runs/<kml-name>_<N>drone-<timestamp>/
```

Important files include:

```text
prepared/mission_output.json
prepared/swarm_mission.yaml
prepared/map-input-overlay.kml
planned/ardupilot/drone-N.ardupilot-mission.json
fleet_sitl_direct.yaml
fleet-upload-report.json
fleet-execution-report.json
fleet-checkpoint.json
fleet-telemetry.csv
fleet-events.jsonl
logs/coverage-planner.log
logs/sitl/drone-N.log
run-state.json
```

## Scope boundary

This command is an ArduCopter SITL supervisor. It is not yet the Raspberry Pi
5 plus physical Pixhawk launcher.

The hardware version must not launch SITL processes or assume simulated RC
overrides. It should retain:

- explicit HOME validation;
- mission semantic validation;
- upload/readback fingerprint verification;
- identity checks;
- GPS/EKF readiness;
- normal pre-arm checks;
- landing and disarm confirmation;
- telemetry freshness and failsafe handling.

Perform propeller-off bench tests before any restrained or outdoor test.

## Live map

Use `--map` to open a live MAVProxy map during the arbitrary-N simulation:

```zsh
coverage-swarm simulate mission.kml \
  --drones 5 \
  --execute \
  --replace-running \
  --map
```

The map connects to the secondary SITL telemetry ports instead of SERIAL0.
This keeps the direct TCP links for upload and execution free.

| Vehicle | Upload/execution SERIAL0 | Live-map telemetry port |
|---|---:|---:|
| drone-1 | 5760 | 5762 |
| drone-2 | 5770 | 5772 |
| drone-3 | 5780 | 5782 |
| drone-N | 5760 + 10 × (N-1) | 5762 + 10 × (N-1) |

The map loads the generated overlay:

```text
prepared/map-input-overlay.kml
```

Use `--hold-map` with `--map` to keep the live map open after a successful run
until Enter is pressed:

```zsh
coverage-swarm simulate mission.kml \
  --drones 5 \
  --execute \
  --replace-running \
  --map \
  --hold-map
```

Use `--keep-run` when you want the SITL vehicles and map process to remain alive
after success for manual inspection.

## Live map secondary-port startup note

ArduCopter SITL opens its primary SERIAL0 TCP listener first (`5760`, `5770`,
`5780`, ...). Secondary telemetry listeners such as `5762`, `5772`, `5782`, ...
may appear only after the first MAVLink client has connected to each primary
port. Therefore the arbitrary-N supervisor waits for primary ports, performs
verified upload/readback, then waits for secondary live-map ports and starts the
MAVProxy map before AUTO execution. This avoids a false startup failure while
keeping upload/execution direct TCP links separate from map telemetry.
