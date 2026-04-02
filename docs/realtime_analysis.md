# Realtime Analysis

## Purpose

This document analyses the realtime-relevant design choices in the repository. All observations are grounded in the actual source files visible in the snapshot.

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

The loop does not wake on anything else. Between events it is fully blocked in the kernel. The comment in the source is explicit: *"The event loop blocks in poll() and only wakes on timer, stdin, or signal activity."*

This means the headless runtime has zero CPU cost while idle — it is structurally incapable of a busy-wait.

---

## 3. Inter-Thread Queues — `ThreadSafeQueue`

`src/common/ThreadSafeQueue.hpp` is the transport between the camera, control, and actuator threads. The implementation uses `std::mutex` and `std::condition_variable` with blocking `wait_pop()`.

Two queue instances are declared in `SystemManager`:

| Queue | Type | Capacity | Push Policy |
|---|---|---|---|
| `frame_q_` | `ThreadSafeQueue<FrameEvent>` | 2 | `push_latest()` — drops oldest |
| `cmd_q_` | `ThreadSafeQueue<ActuatorCommand>` | 8 | `push_latest()` — drops oldest |

The `push_latest()` policy is the correct choice for this system: if a consumer thread falls behind, it processes the most recent data rather than working through a stale backlog. The bounded capacity prevents unbounded memory growth under load.

`stop()` sets a flag and calls `cv_.notify_all()`, which guarantees blocked threads are always woken cleanly on shutdown.

---

## 4. Worker Threads

`SystemManager` owns two `std::thread` workers:

### Control Thread — `controlLoop_()`

```cpp
while (true) {
    const auto item = frame_q_.wait_pop();
    if (!item.has_value()) { break; }
    ...
    tracker_.onFrame(*item);
}
```

Blocks unconditionally on `frame_q_.wait_pop()`. Wakes only when a frame arrives or the queue is stopped. There is no sleep, no poll interval, no timeout.

### Actuator Thread — `actuatorLoop_()`

```cpp
while (true) {
    const auto item = cmd_q_.wait_pop();
    if (!item.has_value()) { break; }
    ...
    actuatorMgr_.onCommand(*item);
}
```

Same structure — blocks on `cmd_q_.wait_pop()` and wakes only on data or stop.

Both threads are joined cleanly on shutdown.

---

## 5. Camera Callback and Frame Ingestion

Incoming frames are delivered via camera callback and pushed into `frame_q_` using `push_latest()`:

```cpp
void SystemManager::onFrame_(const FrameEvent& fe) {
    latency_.onCapture(fe.frame_id, fe.t_capture);
    ...
    (void)frame_q_.push_latest(fe);
}
```

The `LatencyMonitor` timestamp (`t_capture`) is recorded at the moment the frame enters the pipeline, before any processing. This is the correct point for measuring software-side latency from frame arrival to actuator output.

---

## 6. Hardware Sensor Event Paths

### ADS1115 Manual Input

`src/sensors/manual/ADS1115ManualInput.cpp` uses the ALERT/RDY GPIO pin as a hardware interrupt. The source comment is explicit: *"Configure the ADC once, then let the ALERT/RDY pin wake the sample callback."*

A falling edge on the GPIO pin triggers the registered callback via `libgpiod`:

```cpp
drdy_pin_->registerCallback([this](const gpiod::edge_event& event) { ... });
```

No polling of the ADC register. Samples arrive only when the hardware asserts the pin.

### MPU6050 IMU

`src/sensors/imu/Mpu6050Publisher.cpp` uses the same pattern. The source comment: *"The GPIO edge is the wake-up event; I2C transfer only happens after the edge arrives."*

```cpp
gpio_pin_.registerCallback([this](const gpiod::edge_event& event) {
    onGpioEvent_(event);
});
```

I2C register reads happen only after a rising or falling edge is received. This is the correct pattern for interrupt-driven IMU integration.

---

## 7. Simulated Camera Path

`src/sensors/SimulatedPublisher.cpp` is the non-hardware camera backend. The original version of this analysis described it as sleep-based. **That is no longer accurate.** The source comment in the current snapshot is unambiguous:

> *"The simulator still uses the normal camera callback path, but it now sleeps on Linux file descriptors instead of using sleep-based pacing."*

The worker thread multiplexes a `timerfd` and a wakeup `eventfd` via `poll()`:

```cpp
pollfd fds[2]{};
fds[0].fd = timer_fd_;   // timerfd: frame pacing
fds[1].fd = wake_fd_;    // eventfd: clean stop signal
...
const int ret = ::poll(fds, 2, -1);
```

The stop path writes to the `eventfd`, which wakes the blocked `poll()` immediately. This is the same structural pattern as `LinuxEventLoop` — blocking on file descriptors, not sleeping. The simulated path no longer diverges from the production event model.

---

## 8. GUI Role

`src/qt/MainWindow.cpp` registers observers on the runtime and uses Qt timers for view refresh. The main processing pipeline is independent of the GUI — the GUI observes and commands the runtime but does not own the realtime execution path. This is architecturally correct for a system where the GUI must remain optional.

---

## 9. Honest Weaknesses

- `SystemManager` is a broad orchestration class. An ideal design might narrow its scope further.
- `src/qt/main_qt.cpp` contains runtime configuration policy that could be pushed closer to `SystemFactory`.
- The snapshot alone cannot prove hosted CI history, physical actuator behaviour, or worst-case scheduling behaviour under all OS load conditions.

These are real observations but do not affect the correctness of the core event-driven structure.

---

## 10. Summary

Every blocking point in the system is a proper Linux blocking primitive:

| Component | Blocking Mechanism |
|---|---|
| `LinuxEventLoop` | `poll()` on `signalfd`, `timerfd`, `stdin` |
| Control thread | `condition_variable::wait()` via `frame_q_.wait_pop()` |
| Actuator thread | `condition_variable::wait()` via `cmd_q_.wait_pop()` |
| ADS1115 input | GPIO ALERT/RDY edge via `libgpiod` callback |
| MPU6050 IMU | GPIO data-ready edge via `libgpiod` callback |
| SimulatedPublisher | `poll()` on `timerfd` + `eventfd` |

There is no polling loop, no sleep-based pacing, and no busy-waiting anywhere in the system. The architecture is consistently event-driven from the application event loop down to the hardware sensor backends.