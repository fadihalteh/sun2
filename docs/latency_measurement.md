# Realtime Latency Measurement and Evidence

This document presents quantitative evidence for the software-side latency of the system running on Raspberry Pi class Linux hardware.

The measured processing path is:

**Camera → FrameQueue → SunTracker → (state-dependent Controller) → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver**

Latency data is collected at runtime using monotonic timestamps and written to `artefacts/latency.csv`.

---

## 1. Measured Results

Measurement extracted from runtime output:

- Frames processed: 419
- Data source: LatencyMonitor summary

| Metric | Average (ms) | Minimum (ms) | Maximum (ms) | Jitter (ms) |
|---|---:|---:|---:|---:|
| **L_total** | **8.369570** | 6.829599 | 14.565364 | 7.735765 |
| L_vision (capture → estimate) | 8.242496 | 6.755912 | 14.530086 | 7.774174 |
| L_control (estimate → control) | 0.014822 | 0.006038 | 0.363637 | 0.357599 |
| L_actuation (control → actuate) | 0.112253 | 0.019426 | 3.095969 | 3.076543 |

---

## 2. Pipeline Interpretation

The latency measurements correspond to the following stages:

- **L_vision** — frame arrival → SunTracker output; includes frame handling and vision processing
- **L_control** — Controller + ManualImuCoordinator; includes state-dependent execution (skipped in MANUAL)
- **L_actuation** — Kinematics3RRS → ActuatorManager → ServoDriver
- **L_total** — full path from frame reception to actuator command issuance

---

## 3. Timestamp Strategy

Timestamps are recorded using `std::chrono::steady_clock`. Per frame:

```cpp
t_capture  — frame received
t_estimate — SunTracker output
t_control  — control / coordination complete
t_actuate  — actuator command issued
```

All timestamps are associated with a frame identifier and stored until the full pipeline completes.

---

## 4. Latency Definitions

| Metric | Definition |
|---|---|
| L_vision | `t_estimate - t_capture` |
| L_control | `t_control - t_estimate` |
| L_actuation | `t_actuate - t_control` |
| L_total | `t_actuate - t_capture` |

Total latency is measured directly, not inferred.

---

## 5. Measurement Method

Latency is collected using the runtime `LatencyMonitor`:

- timestamps recorded at each pipeline stage
- per-frame data aggregated
- summary statistics computed at shutdown
- raw data exported to CSV (`artefacts/latency.csv`)

Execution characteristics during measurement:

- camera produces frames via callback
- control thread blocks on frame queue
- actuator thread blocks on command queue
- no polling or sleep-based scheduling is used in the processing path

---

## 6. Observations

### 6.1 Dominant Latency Source

- vision processing dominates total latency (~8.24 ms average)
- control stage is negligible (~0.015 ms)
- actuation stage is small but variable (~0.11 ms average)

### 6.2 End-to-End Behaviour

- average latency ≈ 8.37 ms
- worst-case latency ≈ 14.57 ms
- jitter ≈ 7.74 ms

### 6.3 Throughput Context

At 30 Hz, frame period ≈ 33 ms. Measured latency remains well below one frame period, meaning:

- processing completes before the next frame arrives
- no systematic backlog accumulation occurs

---

## 7. Architectural Implications

The measured data supports:

- separation of vision, control, and actuation stages
- effectiveness of bounded queues (capacity = 1)
- responsiveness due to freshest-data policy
- minimal contribution from control logic
- stable end-to-end behaviour under continuous input

---

## 8. Scope of Measurement

The measurements cover **software-side latency only**.

Excluded:

- camera sensor exposure time
- kernel buffering before userspace
- servo mechanical response
- physical platform motion
- environmental disturbances

The results therefore represent userspace processing latency from frame arrival to command issuance.

---

## 9. Summary

| | |
|---|---|
| Average latency | ≈ 8.37 ms |
| Worst-case latency | ≈ 14.57 ms |
| Bounded jitter | ≈ 7.74 ms |

The system demonstrates consistent end-to-end processing with bounded latency and no accumulation. The dominant cost lies in the vision stage, with minimal overhead in control and actuation. The pipeline delivers stable and predictable software-side responsiveness under the measured conditions.