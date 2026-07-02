# Safety model and limitations

## Fail-closed properties

- Malformed or unknown geometry fields are rejected.
- Polygon components are preserved rather than silently discarded.
- Planner responses are checked for success, frame consistency, finite values and altitude consistency.
- Every waypoint and connector segment is checked against authoritative free space.
- Disconnected endpoints fail instead of producing an unsafe straight connector.
- LAND is accepted only after geometric return to HOME.
- Upload succeeds only after complete readback fingerprint verification.
- Validation, generation, upload and preflight do not arm vehicles.
- Fleet execution requires explicit `--execute`.
- Shared-HOME launch is staggered behind altitude and distance gates.

## The Stage 20D defect

The original coverage routes were safe, but terminal RTL allowed ArduPilot to choose a straight return. Drone 2 and Drone 4 crossed No-Go 2.

The fix required:

1. a planned return-to-reference connector through the same free space;
2. validation that the preterminal waypoint is HOME;
3. explicit LAND at HOME;
4. a new five-drone flight and containment audit.

## What the evidence proves

For the checked-in SITL scenario, all five missions completed and all sampled points and interpolated telemetry segments remained inside physical safe space.

## What it does not prove

It does not establish physical-aircraft airworthiness, certified collision avoidance, real GNSS robustness, radio reliability, wind tolerance or regulatory permission. Hardware deployment needs independent geofencing, operator override, physical separation procedures and scenario-specific testing.
