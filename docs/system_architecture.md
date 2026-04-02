# System Architecture

## Overview

The system is structured as a staged, event-driven pipeline in Linux userspace. Frame acquisition, tracking, control, kinematic mapping, and actuator output are separated into distinct classes so that each stage has a clear responsibility and a bounded interface. This is consistent with the taught ENG5220 architecture of sensor events propagating through callbacks and typed data transformations to control outputs.

The implemented processing path is:

**ICamera → SunTracker → Controller → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver**

`SystemManager` orchestrates this pipeline, manages runtime state, starts and stops backends, and coordinates the worker threads. `SystemFactory` acts as the composition root that assembles the concrete runtime graph.

This class structure is stronger than a monolithic alternative because it separates:

- sensor acquisition from estimation
- estimation from control
- control from mechanism-specific mapping
- mechanism mapping from actuator safety conditioning
- actuator conditioning from final hardware output
- runtime orchestration from backend implementation details

That separation improves maintainability, reduces coupling, and makes the realtime event flow explicit.

---

## Architectural Rationale

The core design decision is to preserve a forward-only event path through specialised classes instead of building one large tracker class. In this repository, each stage transforms one well-defined form of data into the next:

- the camera backend emits frames
- the tracker estimates target position and confidence
- the controller converts that estimate into a platform command
- the manual/IMU coordination layer adjusts command ownership and optional correction
- the kinematics layer maps platform motion into actuator-space commands
- the actuator manager applies command conditioning and output safety policy
- the servo driver performs final calibrated output to the physical device

This staged structure is appropriate for ENG5220 because the taught approach is event-driven userspace code built around callbacks, blocking waits, and clean class interfaces rather than polling loops or single-threaded delay-based control.

---

## Diagram 1 — UML Class Diagram

```mermaid
classDiagram
direction LR

class SystemManager {
  +start() bool
  +stop() void
  +state() TrackerState
  +enterManual() void
  +exitManual() void
  +setManualSetpoint(tilt_rad, pan_rad) void
  +setTrackerThreshold(thr) void
  +setManualCommandSource(src) void
  +registerFrameObserver(cb) void
  +registerEstimateObserver(cb) void
  +registerSetpointObserver(cb) void
  +registerCommandObserver(cb) void
  +registerLatencyObserver(cb) void
  -onFrame_(fe) void
  -controlLoop_() void
  -actuatorLoop_() void
  -setState_(s) void
  -applyNeutralOnce_() void
  -applyParkOnce_(servo_deg) void
}

class ICamera {
  <<interface>>
  +registerFrameCallback(cb) void
  +start() bool
  +stop() void
  +isRunning() bool
}

class LibcameraPublisher
class SimulatedPublisher
ICamera <|.. LibcameraPublisher
ICamera <|.. SimulatedPublisher

class ThreadSafeQueue~T~ {
  +ThreadSafeQueue(capacity)
  +wait_pop() optional~T~
  +push_latest(item) bool
  +stop() void
  +reset() void
  +clear() void
}

class SunTracker {
  +registerEstimateCallback(cb) void
  +setThreshold(thr) void
  +onFrame(fe) void
}

class Controller {
  +registerSetpointCallback(cb) void
  +onEstimate(est) void
}

class Kinematics3RRS {
  +registerCommandCallback(cb) void
  +onSetpoint(sp) void
}

class ActuatorManager {
  +registerSafeCommandCallback(cb) void
  +onCommand(cmd) void
}

class ServoDriver {
  +start() bool
  +stop() void
  +apply(cmd) void
}

class LatencyMonitor {
  +onCapture(frame_id, t) void
  +onEstimate(frame_id, t) void
  +onControl(frame_id, t) void
  +onActuate(frame_id, t) void
}

class ManualImuCoordinator {
  +setManualCommandSource(src) void
  +updateImuSample(sample) void
  +applyImuCorrection(sp) PlatformSetpoint
  +buildManualSetpointFromPot(sample, state, frame_id, t) PlatformSetpoint
  +buildManualSetpointFromGui(tilt, pan, frame_id, t) PlatformSetpoint
}

class ADS1115ManualInput {
  +registerCallback(cb) void
  +start() bool
  +stop() void
}

class Mpu6050Publisher {
  +registerEventCallback(cb) void
  +start() bool
  +stop() void
}

SystemManager o-- ICamera
SystemManager o-- SunTracker
SystemManager o-- Controller
SystemManager o-- Kinematics3RRS
SystemManager o-- ActuatorManager
SystemManager o-- ServoDriver
SystemManager o-- LatencyMonitor
SystemManager o-- ManualImuCoordinator
SystemManager o-- ADS1115ManualInput
SystemManager o-- Mpu6050Publisher

SystemManager o-- ThreadSafeQueue~FrameEvent~ : frame_q_ cap=1
SystemManager o-- ThreadSafeQueue~ActuatorCommand~ : cmd_q_ cap=1

ICamera ..> FrameEvent : emits via callback
SunTracker ..> SunEstimate : emits via callback
Controller ..> PlatformSetpoint : emits via callback
Kinematics3RRS ..> ActuatorCommand : emits via callback
ActuatorManager ..> ActuatorCommand : emits safe callback
ADS1115ManualInput ..> ManualPotSample : emits via callback
Mpu6050Publisher ..> ImuSample : emits via callback
ManualImuCoordinator ..> PlatformSetpoint : adjusts / builds
```

---

## Diagram 2.1 — Sequence Diagram — Automatic Runtime Pipeline

```mermaid
sequenceDiagram
autonumber

participant Cam as Camera backend
participant SM as SystemManager
participant FQ as FrameQueue cap=1
participant CT as Control thread
participant ST as SunTracker
participant C as Controller
participant MIC as ManualImuCoordinator
participant K as Kinematics3RRS
participant CQ as CommandQueue cap=1
participant AT as Actuator thread
participant AM as ActuatorManager
participant SD as ServoDriver
participant LM as LatencyMonitor

SM->>Cam: registerFrameCallback(onFrame_)
SM->>ST: registerEstimateCallback(...)
SM->>C: registerSetpointCallback(...)
SM->>K: registerCommandCallback(...)
SM->>AM: registerSafeCommandCallback(...)
SM->>SD: start()
SM->>Cam: start()
SM->>SM: setState(SEARCHING)

Cam-->>SM: FrameCallback(FrameEvent)
SM->>LM: onCapture(frame_id, t_capture)
SM->>FQ: push_latest(FrameEvent)

CT->>FQ: wait_pop()
FQ-->>CT: FrameEvent
CT->>ST: onFrame(FrameEvent)

ST-->>CT: EstimateCallback(SunEstimate)
CT->>LM: onEstimate(frame_id, t_estimate)

alt confidence >= threshold
  CT->>SM: setState(TRACKING)
else confidence < threshold
  CT->>SM: setState(SEARCHING)
end

CT->>C: onEstimate(SunEstimate)
C-->>CT: SetpointCallback(PlatformSetpoint)
CT->>LM: onControl(frame_id, t_control)

CT->>MIC: applyImuCorrection(PlatformSetpoint)
MIC-->>CT: corrected PlatformSetpoint

CT->>K: onSetpoint(corrected PlatformSetpoint)
K-->>CT: CommandCallback(ActuatorCommand)
CT->>CQ: push_latest(ActuatorCommand)

AT->>CQ: wait_pop()
CQ-->>AT: ActuatorCommand
AT->>AM: onCommand(ActuatorCommand)

AM-->>AT: SafeCommandCallback(ActuatorCommand)
AT->>LM: onActuate(frame_id, t_actuate)
AT->>SD: apply(ActuatorCommand)
```
---

## Diagram 2.2 — Sequence Diagram — Manual Input + IMU Update Paths
```mermaid
sequenceDiagram
autonumber

participant ADS as ADS1115ManualInput
participant IMU as Mpu6050Publisher
participant SM as SystemManager
participant MIC as ManualImuCoordinator
participant K as Kinematics3RRS
participant CQ as CommandQueue cap=1
participant AT as Actuator thread
participant AM as ActuatorManager
participant SD as ServoDriver
participant LM as LatencyMonitor
participant User as Qt / CLI User

IMU-->>SM: Imu callback(sample)
SM->>MIC: updateImuSample(sample)

User->>SM: enterManual()
SM->>MIC: setManualCommandSource(Pot or GUI)
SM->>SM: setState(MANUAL)

alt Pot-controlled manual mode
  ADS-->>SM: Manual sample callback(sample)
  SM->>MIC: buildManualSetpointFromPot(sample, state, frame_id, t)
  MIC-->>SM: PlatformSetpoint
else GUI-controlled manual mode
  User->>SM: setManualSetpoint(tilt, pan)
  SM->>MIC: buildManualSetpointFromGui(tilt, pan, frame_id, t)
  MIC-->>SM: PlatformSetpoint
end

SM->>LM: onControl(frame_id, t_control)
SM->>K: onSetpoint(PlatformSetpoint)
K-->>SM: CommandCallback(ActuatorCommand)
SM->>CQ: push_latest(ActuatorCommand)

AT->>CQ: wait_pop()
CQ-->>AT: ActuatorCommand
AT->>AM: onCommand(ActuatorCommand)
AM-->>AT: SafeCommandCallback(ActuatorCommand)
AT->>LM: onActuate(frame_id, t_actuate)
AT->>SD: apply(ActuatorCommand)

User->>SM: exitManual()
SM->>SM: setState(SEARCHING)
```
---
## Diagram 3 — Component Diagram

```mermaid
flowchart LR
  subgraph EntryPoints["Entry Points"]
    CLI["solar_tracker (CLI)"]
    QT["solar_tracker_qt (Qt UI)"]
  end

  subgraph Core["Core Realtime Pipeline"]
    SM["SystemManager"]
    FQ["FrameQueue cap=1<br/>ThreadSafeQueue&lt;FrameEvent&gt;"]
    CQ["CommandQueue cap=1<br/>ThreadSafeQueue&lt;ActuatorCommand&gt;"]
    ST["SunTracker"]
    C["Controller"]
    MIC["ManualImuCoordinator"]
    K["Kinematics3RRS"]
    AM["ActuatorManager"]
    SD["ServoDriver"]
    LM["LatencyMonitor"]
  end

  subgraph Vision["Camera Backends"]
    ICam["ICamera"]
    Lib["LibcameraPublisher"]
    Sim["SimulatedPublisher"]
  end

  subgraph ManualInput["Manual Input"]
    ADS["ADS1115ManualInput"]
    Pot["Tilt / Pan Pots"]
  end

  subgraph ImuFeedback["IMU Feedback"]
    MPU["Mpu6050Publisher"]
  end

  CLI --> SM
  QT --> SM

  Lib -- implements --> ICam
  Sim -- implements --> ICam
  ICam -->|Frame callback| SM

  SM --> FQ
  FQ -->|wait_pop in control thread| ST
  ST --> C
  C --> MIC
  MIC --> K
  K --> CQ
  CQ -->|wait_pop in actuator thread| AM
  AM --> SD

  ADS -->|manual sample callback| SM
  Pot --> ADS
  MPU -->|IMU callback| SM
  SM --> MIC

  SM --> LM
```
---
## Diagram 4 — Threaded Event Architecture
```mermaid
flowchart TB
  CAM["Camera backend<br/>libcamera or simulated"]
  ADS["ADS1115 manual input"]
  MPU["MPU6050 IMU"]
  PI["Qt / CLI user input"]

  subgraph T1["Callback / producer side"]
    CAMCB["Frame callback"]
    ADSCB["Manual sample callback"]
    MPUCB["IMU callback"]
  end

  subgraph T2["Control thread"]
    FQ["FrameQueue cap=1"]
    ST["SunTracker"]
    CTRL["Controller"]
    MIC["ManualImuCoordinator"]
    KIN["Kinematics3RRS"]
    CQ["CommandQueue cap=1"]
  end

  subgraph T3["Actuator thread"]
    ACT["ActuatorManager"]
    SERVO["ServoDriver"]
  end

  CAM --> CAMCB --> FQ
  FQ --> ST --> CTRL --> MIC --> KIN --> CQ
  CQ --> ACT --> SERVO

  ADS --> ADSCB --> MIC
  MPU --> MPUCB --> MIC
  PI --> MIC
```
---

## Component Responsibilities

### SystemManager

Coordinates runtime execution, manages threads, processes incoming events, and controls system state. It connects all pipeline stages and ensures correct execution order.

### ICamera and Camera Backends

Provide frame acquisition. Multiple implementations can be used without affecting downstream processing.

### SunTracker

Processes image data and produces a target estimate including position and confidence.

### Controller

Transforms the target estimate into a platform-level motion command.

### ManualImuCoordinator

Handles manual input and optional IMU-based adjustments. It determines command ownership and applies corrections where required.

### Kinematics3RRS

Maps platform motion commands into actuator-specific commands based on mechanism geometry.

### ActuatorManager

Applies conditioning to actuator commands such as clamping and rate limiting to ensure safe output.

### ServoDriver and PCA9685

Convert actuator commands into hardware signals and communicate with the PWM controller.

### Input Devices

- `ADS1115ManualInput` provides potentiometer-based manual control input
- `Mpu6050Publisher` provides IMU data for orientation feedback

---

## Architectural Summary

The system is built as a pipeline of independent processing stages connected through well-defined interfaces. Each stage transforms data and passes it forward without direct knowledge of internal details of other components.

This structure enables:

- predictable data flow
- clear separation between computation and hardware access
- safe multi-threaded execution
- straightforward extension and modification of individual stages