# Launch-post drafts

## LinkedIn

I built a safety-first ROS 2 mission pipeline that can run a five-drone ArduPilot coverage mission from one terminal.

The part I am most proud of is not just generating sweep paths. The pipeline handles disconnected polygon components, concave boundaries, no-go zones, safe transitions, route orientation, MAVLink precision, independent upload/readback, staggered launch and explicit LAND at HOME.

During testing, I found an architectural failure: two drones had safe coverage routes but crossed an exclusion zone during ArduPilot's implicit RTL. I replaced terminal RTL with a visibility-graph return through the same authoritative safe space, reran the complete fleet, and audited every sampled trajectory segment.

Final audited demo:
- 5/5 missions completed, landed and disarmed
- 0 unsafe planned or sampled flown segments
- 0 runtime separation warnings
- 0.919 m worst observed tracking error against a 2 m reserve
- 24.396 m minimum observed fleet separation

The repository includes the reproducible demo, validation evidence, architecture notes and one-command live-map simulation.

https://github.com/sidkakroo770/coverage_mission_pipeline

## X / Twitter

Built a ROS 2 + ArduPilot five-drone coverage pipeline that runs from one terminal: safe geometry, visibility-graph connectors, transactional MAVLink upload/readback, staggered launch, live map and LAND at HOME.

The best bug: safe coverage, unsafe implicit RTL. Fixed the architecture, reran five-drone SITL, audited the trajectories: 0 unsafe segments and 0 separation warnings.

https://github.com/sidkakroo770/coverage_mission_pipeline
