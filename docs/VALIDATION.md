# Validation evidence

| Stage | Result | Meaning |
|---|---|---|
| 19A | PASS | Transactional upload/readback |
| 20A | PASS | Five-drone read-only preflight |
| 20B | PASS | Single-drone execution and DataFlash audit |
| 20C | PASS | Five-drone staggered execution |
| 20D | **FAIL** | Drone 2 and Drone 4 crossed No-Go 2 during RTL |
| 21A/B | PASS | Safe-return policy and static audit |
| 21C | PASS | Corrected five-drone execution |
| 21D | PASS | Authoritative safe-space audit |

## Stage 21D result

| Vehicle | Planned unsafe points / segments | Flown unsafe points / segments | Max tracking error | Final HOME error |
|---|---:|---:|---:|---:|
| Drone 1 | 0 / 0 | 0 / 0 | 0.919 m | 0.022 m |
| Drone 2 | 0 / 0 | 0 / 0 | 0.819 m | 0.045 m |
| Drone 3 | 0 / 0 | 0 / 0 | 0.791 m | 0.030 m |
| Drone 4 | 0 / 0 | 0 / 0 | 0.794 m | 0.041 m |
| Drone 5 | 0 / 0 | 0 / 0 | 0.843 m | 0.049 m |

Former failure area:

| Vehicle | Closest to raw No-Go 2 | Closest to 10 m buffer |
|---|---:|---:|
| Drone 2 | 12.449 m | 0.670 m |
| Drone 4 | 11.818 m | 0.785 m |

Minimum observed fleet separation was **24.396 m**, with zero runtime warnings.

Machine-readable evidence is under `demo/noida_stage21/reports`.

The flown audit sampled telemetry at roughly 0.5 s and tested straight segments between samples. It is strong SITL evidence, not continuous-time formal verification.
