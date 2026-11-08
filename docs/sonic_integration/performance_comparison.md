# Performance: no whole-body controller comparison yet
| Measurement | Locomotion flat | Locomotion factory | SONIC |
|---|---:|---:|---|
| Mean loop FPS, accelerated self-test | 34.79 | 36.69 | Not measured closed-loop |
| Physics steps / wall second | 139.17 | 146.75 | Not measured |
| Peak whole-GPU used MiB | 4125.0 | 4389.0 | Not measured with renderer |
| Offline CPU idle planner latency | N/A | N/A | 43.87 ms mean, 3 calls |

Configured physics 200Hz, control 50Hz. Loop/render request FPS is not GPU presentation FPS. Whole-device memory includes unrelated apps. Planner mean exceeds the 20ms control period, but planner need not run at policy rate; an asynchronous planner/reference buffer requires its own validation. This measurement alone does not prove failure of 50Hz whole-body control.
Encoder/decoder latency, end-to-end SONIC control latency, CPU RAM, per-physics-step execution time, SONIC falls/success rate and comparative tracking errors were not measured. Baseline telemetry contains real target/actual velocity, orientation, joints and per-call locomotion inference_ms. No controller ranking is claimed. GPU-memory savings from CPU inference are plausible but not measured as a combined SONIC scene.
