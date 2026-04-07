# System State Machine

This document defines the runtime behaviour of the system as a state machine. It describes the implemented execution flow, including all states, transitions, and operational behaviour.

---

## 1. States

| State | Meaning | Outputs |
|---|---|---|
| IDLE | System not running | No motion |
| STARTUP | Initialisation in progress | Startup sequence in progress |
| NEUTRAL | Transitional safe positioning state | Configured startup park applied |
| SEARCHING | Target not confidently detected | Continuous automatic processing with safe behaviour |
| TRACKING | Target detected with sufficient confidence | Normal closed-loop automatic updates |
| MANUAL | User controls setpoint | Manual target is applied continuously through the control thread |
| STOPPING | Shutdown in progress | Controlled stop sequence |
| FAULT | Failure state | Outputs stopped or held safe |

---

## 2. Transition Rules

| From | To | Trigger |
|---|---|---|
| IDLE | STARTUP | `start()` called |
| STARTUP | FAULT | camera null, driver start failure, manual backend failure, or camera start failure |
| STARTUP | NEUTRAL | successful initialisation |
| NEUTRAL | SEARCHING | startup park applied and `startup_mode == Auto` |
| NEUTRAL | MANUAL | startup park applied and `startup_mode == Manual` |
| SEARCHING | TRACKING | confidence ≥ threshold |
| TRACKING | SEARCHING | confidence < threshold |
| SEARCHING | MANUAL | `enterManual()` |
| TRACKING | MANUAL | `enterManual()` |
| NEUTRAL | MANUAL | `enterManual()` |
| MANUAL | SEARCHING | `exitManual()` |
| SEARCHING | STOPPING | `stop()` called |
| TRACKING | STOPPING | `stop()` called |
| MANUAL | STOPPING | `stop()` called |
| NEUTRAL | STOPPING | `stop()` called |
| STARTUP | STOPPING | `stop()` during startup |
| FAULT | STOPPING | `stop()` called |
| STOPPING | IDLE | shutdown complete |
| ANY ACTIVE STATE | FAULT | critical runtime failure |

---

## 3. State Descriptions

### IDLE

The system is inactive.

- camera is not running
- worker threads are not active
- no actuator commands are produced

### STARTUP

Initialisation phase.

- system marked as running
- camera null check performed
- actuator driver started
- queues reset
- worker threads started
- backend coordinator started
- camera streaming started
- failures lead to FAULT

### NEUTRAL

Short transitional state.

- startup park is applied using predefined actuator values
- transitions to SEARCHING (default) or MANUAL (if `startup_mode == Manual`)

### SEARCHING

System is active but target confidence is low.

- frames are processed continuously
- full automatic pipeline remains active
- motion remains bounded
- transitions to TRACKING when confidence increases

### TRACKING

System operates in closed-loop tracking mode.

- each frame triggers full automatic processing path
- controller, correction, kinematics, and actuator stages are active
- continuous actuator updates
- transitions back to SEARCHING if confidence drops

### MANUAL

User-controlled mode.

- activated via `enterManual()` or by `startup_mode == Manual`
- automatic controller updates are disabled
- the control thread handles only the automatic pipeline — it is **not involved** in manual mode dispatch

**Pot-driven manual (`ManualCommandSource::Pot`):**
- ADS1115 ALERT/RDY GPIO edge fires → `onManualPotSample_()` callback in the ADS1115 thread
- setpoint built via `ManualImuCoordinator` and dispatched **directly** to `Kinematics3RRS`
- timing is driven by the ADS1115 conversion rate — independent of camera frames

**GUI-driven manual (`ManualCommandSource::Gui`):**
- `setManualSetpoint()` → `GuiManualDispatcher::setSetpoint()` → push to bounded queue
- `GuiManualDispatcher` worker thread wakes **immediately** and dispatches directly to `Kinematics3RRS`
- timing is driven by operator input rate — independent of camera frames

Both paths use the same downstream `Kinematics3RRS → ActuatorManager → ServoDriver` pipeline.

IMU correction is applied via `ManualImuCoordinator::applyImuCorrection()` in both paths.

### STOPPING

Shutdown sequence.

- system marked as not running
- camera stopped
- backend coordinator stopped
- frame and command queues stopped
- processing threads joined
- neutral command applied
- driver stopped
- transitions to IDLE

### FAULT

Failure state.

- triggered by null camera, driver failure, backend failure, camera start failure, or invalid kinematics result
- actuator commands are halted or suppressed
- system requires explicit stop to recover

---

## 4. Runtime State Graph

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> STARTUP: start()

    STARTUP --> NEUTRAL
    STARTUP --> FAULT

    NEUTRAL --> SEARCHING: startup_mode == Auto
    NEUTRAL --> MANUAL: startup_mode == Manual

    SEARCHING --> TRACKING
    TRACKING --> SEARCHING

    SEARCHING --> MANUAL
    TRACKING --> MANUAL
    NEUTRAL --> MANUAL

    MANUAL --> SEARCHING

    SEARCHING --> FAULT
    TRACKING --> FAULT
    MANUAL --> FAULT
    NEUTRAL --> FAULT

    STARTUP --> STOPPING
    NEUTRAL --> STOPPING
    SEARCHING --> STOPPING
    TRACKING --> STOPPING
    MANUAL --> STOPPING
    FAULT --> STOPPING

    STOPPING --> IDLE
```

---

## 5. Startup and Shutdown Flow

```mermaid
flowchart TD
    A["start()"] --> B["set STARTUP"]
    B --> C{"camera null?"}
    C -- yes --> F1["set FAULT"]

    C -- no --> D{"driver start ok?"}
    D -- no --> F2["set FAULT"]

    D -- yes --> E["reset queues / reset manual state"]
    E --> G["start worker threads + GuiManualDispatcher"]

    G --> H{"backends start ok?"}
    H -- fail --> F3["stop and set FAULT"]

    H -- ok --> I{"camera start ok?"}
    I -- no --> F4["stop and set FAULT"]

    I -- yes --> J["set NEUTRAL"]
    J --> K["apply startup park"]

    K --> L{"startup_mode?"}
    L -- Manual --> M["set MANUAL"]
    L -- Auto --> N["set SEARCHING"]

    O["stop()"] --> P["set STOPPING"]
    P --> Q["camera stop"]
    Q --> R["backends stop"]
    R --> R2["stop GuiManualDispatcher"]
    R2 --> S["stop queues and join threads"]
    S --> T["apply neutral command"]
    T --> U["driver stop"]
    U --> W["set IDLE"]
```

---

## 6. Automatic vs Manual Processing

```mermaid
flowchart TD
    subgraph AUTO["Automatic path — control thread"]
      A[Frame event] --> B[frame queue push_latest]
      B --> C[control thread wait_pop]
      C --> D{state?}
      D -- SEARCHING/TRACKING --> E[SunTracker.onFrame]
      E --> F[SunEstimate callback]
      F --> G[Controller.onEstimate]
      G --> H[ManualImuCoordinator.applyImuCorrection]
      H --> I[Kinematics3RRS.onSetpoint]
      D -- MANUAL or FAULT --> Z[drain frame, no processing]
    end

    subgraph POT["Pot manual path — ADS1115 callback thread"]
      Q[ADS1115 ALERT/RDY GPIO edge] --> R[onManualPotSample_]
      R --> R2{state == MANUAL
and source == Pot?}
      R2 -- yes --> S[buildManualSetpointFromPot]
      S --> S2[applyImuCorrection]
      S2 --> I
      R2 -- no --> RX[discard]
    end

    subgraph GUI["GUI manual path — GuiManualDispatcher thread"]
      P[setManualSetpoint called] --> P2[GuiManualDispatcher.setSetpoint]
      P2 --> P3[push_latest to bounded queue]
      P3 --> P4[worker thread wakes immediately]
      P4 --> P5{state == MANUAL
and source == Gui?}
      P5 -- yes --> T[buildManualSetpointFromGui]
      T --> T2[applyImuCorrection]
      T2 --> I
      P5 -- no --> PX[discard]
    end

    subgraph ACT["Actuator path — actuator thread"]
      I --> J[command queue push_latest]
      J --> K[actuator thread wait_pop]
      K --> L[ActuatorManager.onCommand]
      L --> M[ServoDriver.apply]
    end
```

---

## 7. Implementation Notes

- automatic processing runs only in SEARCHING and TRACKING
- the control thread handles only the automatic path — it is not involved in manual dispatch
- pot manual commands are event-driven from the ADS1115 ALERT/RDY GPIO edge
- GUI manual commands are event-driven from the `GuiManualDispatcher` worker thread
- both manual paths dispatch directly to `Kinematics3RRS` without camera-frame dependency
- GUI manual mode does not use Qt timers as a control timing source
- invalid kinematic results prevent actuation and trigger FAULT
- state transitions are explicit and centrally controlled in `SystemManager`
- `BackendCoordinator` starts and stops ADS1115 and IMU backends during STARTUP and STOPPING