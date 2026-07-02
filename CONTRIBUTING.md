# Contributing

Build the workspace using `docs/REPRODUCE.md`, then run:

```bash
python3 -m pytest -q test
```

Changes to geometry, connectors, mission semantics or readback normalization should include a focused regression test and preserve fail-closed behaviour.

Do not silently discard polygon components, buffer shared partition borders, weaken segment containment, disable ArduPilot arming checks, or ignore readback differences without command-specific evidence.
