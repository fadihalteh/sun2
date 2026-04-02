# Realtime Analysis

## Purpose

This document analyses the realtime-relevant design choices in the repository. All observations are grounded in the current source files visible in the snapshot.

---

## 1. Runtime Model

The system uses a Linux userspace, event-driven runtime model built around:

- `poll()` on Linux file descriptors for the application event loop
- blocking `condition_variable` waits for inter-thread data transfer
- GPIO edge callbacks for hardware sensor wakeup
- explicit state machine control with no implicit transitions

There is no top-level polling loop, no `sleep()`-based pacing, and no busy-waiting anywhere in the processing path.

---

## 2. Application Event Loop — `LinuxEventLoop`

`src/app/LinuxEventLoop.cpp` implements the headless runtime lifecycle. It multiplexes three event sources in a single `poll()` call with an infinite timeout (`-1`):

| File Descriptor | Source | Purpose |
|---|---|---|
| `signalfd` | `SIGINT` / `SIGTERM` | Clean shutdown on OS signal |
| `timerfd` (`CLOCK_MONOTONIC`) | Configurable tick rate (default 30 Hz) | CLI servicing |
| `stdin` (dup'd) | Terminal | Keyboard command input |

The loop does not wake on anything else. Between events it is fully blocked in the kernel.

This means the headless runtime has zero CPU cost while idle — it is structurally incapable of a busy-wait.

---

## 3. Inter-Thread Queues — `ThreadSafeQueue`

`src/common/ThreadSafeQueue.hpp` is the transport between the camera, control, and actuator threads. The implementation uses `std::mutex` and `std::condition_variable` with blocking `wait_pop()`.

Two queue instances are declared in `SystemManager`:

| Queue | Type | Capacity | Push Policy |
|---|---|---|---|
| `frame_q_` | `ThreadSafeQueue<FrameEvent>` | 2 | `push_latest()` — drops oldest |
| `cmd_q_` | `ThreadSafeQueue<ActuatorCommand>` | 8 | `push_latest()` — drops oldest |

The `push_latest()` policy is appropriate for this system: if a consumer thread falls behind, it processes the most recent data rather than working through a stale backlog. The bounded capacity prevents unbounded memory growth under load.

---

## 4. Worker Threads

`SystemManager` owns two `std::thread` workers:

### Control Thread — `controlLoop_()`

The control thread blocks on `frame_q_.wait_pop()`. The same blocking wakeups are now reused for both automatic and manual processing:

- in SEARCHING/TRACKING, the thread forwards the frame into `SunTracker`
- in MANUAL, the thread calls `submitManualSetpointFromControlTick_(...)` and builds the current manual setpoint from stored GUI target state or the latest potentiometer sample

This means manual mode is continuous without introducing a separate timer loop, GUI-driven control loop, or polling thread.

### Actuator Thread — `actuatorLoop_()`

The actuator thread blocks on `cmd_q_.wait_pop()` and applies:

`ActuatorManager -> ServoDriver`

There is no alternative actuation shortcut.

---

## 5. Sensor Wakeups

### Camera

- `SimulatedPublisher` uses `timerfd` + `poll()` + `eventfd`
- `LibcameraPublisher` uses callback delivery from the libcamera wrapper path

### Manual potentiometer input

`ADS1115ManualInput` uses the ADS1115 ALERT/RDY GPIO edge to wake the acquisition path. The callback updates the latest manual potentiometer sample, but does not directly perform downstream kinematics work.

### IMU

`Mpu6050Publisher` uses a GPIO data-ready interrupt path. IMU samples are forwarded into `ManualImuCoordinator`.

---

## 6. Automatic and Manual Paths

### Automatic path

```text
Frame callback
→ frame_q_.push_latest(...)
→ control thread wait_pop()
→ SunTracker
→ Controller
→ ManualImuCoordinator::applyImuCorrection(...)
→ Kinematics3RRS
→ cmd_q_.push_latest(...)
→ actuator thread wait_pop()
→ ActuatorManager
→ ServoDriver
```

### Manual path after refactor

```text
GUI slider valueChanged / ADS1115 callback
→ store latest manual state only
→ control thread wait_pop()
→ submitManualSetpointFromControlTick_(...)
→ ManualImuCoordinator builds manual setpoint
→ Kinematics3RRS
→ cmd_q_.push_latest(...)
→ actuator thread wait_pop()
→ ActuatorManager
→ ServoDriver
```

This is not identical to the automatic path, but it is materially cleaner than a direct GUI/callback-to-kinematics shortcut and keeps timing ownership inside the blocking-wakeup runtime.

---

## 7. Qt GUI Timing

Qt timers are used for GUI refresh and charts, not for reliable control timing.

The GUI manual sliders now update the stored GUI target on `valueChanged`, but the GUI still does not own the control cadence. The actual continuous command submission happens in the control thread on the same blocking wakeups used elsewhere in the runtime.

This is the critical distinction:

- **GUI:** updates desired manual state
- **control thread:** turns that state into continuous actuator-driving setpoints

---

## 8. IMU Policy in Manual Mode

GUI manual mode does **not** apply live IMU correction by default. This is deliberate:

- it keeps operator-selected manual targets stable
- it avoids injecting IMU noise into every GUI manual update
- it reduces the risk of chatter or vibration

Pot/manual mode can still use IMU-assisted behaviour according to the configured policy.

---

## 9. Shutdown Behaviour

`stop()`:

- stops the camera
- stops optional backends
- stops the frame and command queues
- joins the worker threads
- applies neutral/park logic
- stops the servo driver

Because the queues notify blocked waiters on stop, worker threads wake cleanly and do not hang in shutdown.

---
## 10. Summary

The runtime follows a single, consistent execution model across the entire system:

- all long-lived threads block on kernel-backed primitives (`poll`, `condition_variable`)
- all wakeups originate from real events (file descriptors, queue notifications, GPIO edges)
- no component introduces sleep-based pacing, polling loops, or independent timing sources

Control authority is centralised in the control thread. It is the only component responsible for transforming input state into actuator commands.

All input sources, including camera frames, manual GUI inputs, and hardware sensor data, are first reduced to state and then processed within the control thread. No external callback or UI event directly drives kinematics or actuator output.

As a result:

- automatic and manual operation share the same execution pipeline
- actuator commands originate from a single thread with deterministic ordering
- the GUI remains strictly non-realtime and does not influence control timing

The system is fully event-driven, with clear ownership of execution, data flow, and actuation, and without hidden timing paths or parallel control mechanisms.