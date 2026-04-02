# SOLID Justification

This document explains how the class structure in the current implementation supports the project’s realtime and software-engineering goals. The argument here is not that every class exists because of a textbook rule. The argument is that the specific class boundaries in this repository reduce timing risk, reduce coupling to hardware, improve testability, and make the event-driven architecture defendable against the ENG5220 criteria.

The core runtime pipeline implemented in the current codebase is:

**ICamera → SunTracker → Controller → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver**

`SystemManager` is the runtime orchestrator around that pipeline. `SystemFactory` is the composition root. `LinuxEventLoop` and the Qt GUI are application-side control surfaces, not the realtime processing core.

The most important design decision in this repository is that the code is not structured as one giant “tracker” class. Instead, each stage has a narrow boundary and communicates forward through callbacks or explicit method calls with typed data. That is a better fit for this project than a monolithic design because this system combines:

- Linux userspace blocking/event-driven I/O
- camera acquisition
- vision estimation
- control
- optional manual input and IMU correction policy
- 3-RRS mechanism mapping
- actuator safety conditioning
- final servo hardware output
- CLI and Qt application control

If those concerns were merged into one class, the result would be harder to test, harder to explain, harder to maintain, and more dangerous in a realtime project because timing-sensitive code would become mixed with configuration, GUI, and hardware details.

---

## Why SOLID is useful in this specific project

SOLID matters here for four concrete reasons.

First, the project must keep hardware-facing code isolated from pure logic. Camera backends, I2C, GPIO-backed manual input, IMU publishing, and servo output are all volatile and platform-specific. Vision, control, kinematics, and mapping logic should not depend on those details.

Second, the project must preserve an event-driven structure. The taught architecture is sensor event → callback-driven processing → setter/output. Clear class boundaries make that architecture explicit. Without those boundaries, it becomes easy to slip into polling, global-state sharing, or a giant loop that mixes acquisition, processing, and output.

Third, the project must be testable without full hardware. A class boundary is not just a style choice here; it is what allows fake or simulated inputs, unit tests for pure logic, and hardware-specific tests to coexist.

Fourth, the project must be explainable at assessment level. The marking criteria do not reward a working blob of code. They reward justified class structure, clear encapsulation, safe interfaces, reliable realtime behaviour, and maintainability. The class structure in this repository is strongest where it separates policy, computation, orchestration, and hardware.

---

## `ICamera`

### Why this class boundary is correct

`ICamera` is the abstraction for “a source of frames”. That boundary is correct because the rest of the system does not care whether frames come from a Raspberry Pi camera stack or a simulated publisher. The rest of the system only needs a typed frame source and a callback registration mechanism.

If the higher-level runtime depended directly on libcamera code, the vision pipeline would become Linux-camera-specific, tests would become harder to run without hardware, and `SystemManager` would take on platform-specific responsibilities that do not belong there.

### SOLID justification

**Single Responsibility Principle**

`ICamera` has exactly one architectural responsibility: define the contract for frame delivery. It does not acquire frames itself, does not process them, and does not know anything about tracking or control.

That is better than putting camera implementation details directly into orchestration code because it prevents the runtime manager from becoming a mixed abstraction that both controls application lifecycle and performs sensor I/O.

**Interface Segregation Principle**

The interface is intentionally narrow. A camera source should expose camera operations only. It should not force downstream code to know about device configuration internals, GUI display logic, or tracking functions.

That is better than a broad “sensor interface” with unrelated responsibilities because it keeps camera clients dependent only on what they actually use.

**Dependency Inversion Principle**

Higher-level code depends on `ICamera`, not on `LibcameraPublisher` or `SimulatedPublisher`. That is the right direction of dependency for this project. The runtime policy layer should not be forced to rework itself whenever the acquisition backend changes.

### Why it is better than the alternative

A concrete-only design would couple the runtime directly to one acquisition backend. That would make the whole pipeline harder to test and would mix hardware choice with control architecture. For this project, that would be a weaker design both technically and academically.

---

## `LibcameraPublisher`

### Why this class exists

`LibcameraPublisher` isolates the real Raspberry Pi camera backend behind the `ICamera` abstraction. Its job is not to participate in control logic. Its job is to acquire real frames and publish them into the pipeline.

That separation is important because libcamera is a complex, platform-specific dependency. The rest of the project should not be contaminated by its API or lifecycle details.

### SOLID justification

**Single Responsibility Principle**

Its responsibility is frame acquisition from the real camera path. It should not detect the sun, compute setpoints, manage servo output, or decide system state.

That is better than embedding libcamera handling into `SystemManager` because the runtime orchestrator would otherwise become both a hardware wrapper and a policy manager.

**Liskov Substitution Principle**

It is valid anywhere an `ICamera` is expected. That matters in this project because the rest of the pipeline can remain unchanged whether the input source is real or simulated.

**Dependency Inversion Principle**

The rest of the system depends on the abstract frame source contract, while `LibcameraPublisher` absorbs the low-level dependency.

### Why it is better than the alternative

If real camera code were spread across several classes, the hardware dependency would leak upward and sideways. That would make failure handling, testing, and portability worse. Keeping it in one backend class is the cleaner architecture.

---

## `SimulatedPublisher`

### Why this class exists

`SimulatedPublisher` provides a software-only frame source that still fits the same pipeline contract as the real camera. That is a strong design choice for this repository because it allows development and testing without requiring physical camera hardware.

This is not redundant with `LibcameraPublisher`. It serves a different operational role while preserving the same interface boundary.

### SOLID justification

**Single Responsibility Principle**

Its responsibility is to generate synthetic frames and publish them as camera-like events. It does not control actuators, perform vision, or manage application state.

**Liskov Substitution Principle**

It can replace `LibcameraPublisher` wherever `ICamera` is consumed. That substitution matters because it lets the exact same downstream pipeline run against simulated input.

**Open/Closed Principle**

The system is open to adding further camera implementations without rewriting the consumer pipeline. The abstraction boundary is what makes this possible.

### Why it is better than the alternative

The weaker alternative is to bury simulation behind conditionals inside `SystemManager` or `main()`. That would scatter backend selection logic across the runtime and make simulation a second-class path rather than a real implementation. This class-based design is stronger.

---

## `SunTracker`

### Why this class exists

`SunTracker` is the vision stage. Its responsibility is to convert a `FrameEvent` into a `SunEstimate`. That boundary is technically correct because vision estimation is its own problem: validate frame shape, interpret pixel intensities, segment the target, and emit an estimate.

It should not know anything about actuator limits, servo channels, manual control mode, or platform kinematics.

### SOLID justification

**Single Responsibility Principle**

`SunTracker` has one engineering reason to change: the image-processing method for target detection. If tracking logic changes, this class should change. If servo calibration changes, this class should not change. That is exactly the kind of responsibility isolation the project needs.

This is better than folding vision into `SystemManager` because then the runtime orchestrator would also have to change whenever thresholding, centroid logic, or confidence rules change.

**Open/Closed Principle**

The detector implementation can evolve internally without forcing redesign of control or actuation classes, as long as the `SunEstimate` contract remains stable.

That is better than exposing internal pixel-processing assumptions to the rest of the system.

**Encapsulation and safe data management**

The tracker validates the frame contract before reading it. That matters in this project because frame layout errors are a genuine fault risk. The class therefore encapsulates not just the algorithm but also the safe handling of frame storage assumptions.

### Why it is better than the alternative

A “tracker” function buried inside a runtime loop would mix vision logic with thread, state, and actuator concerns. That would make faults harder to localise and would weaken both the design argument and the test story.

---

## `Controller`

### Why this class exists

`Controller` converts a tracking estimate into a platform setpoint. That is a separate engineering stage from vision and from mechanism mapping. It is the correct place for confidence gating, deadband behaviour, and command generation in platform coordinates.

The controller should not compute servo angles and should not talk to hardware. Those are different responsibilities with different reasons to change.

### SOLID justification

**Single Responsibility Principle**

The controller owns the control-law decision from estimate space to platform-command space. That makes tuning and behavioural reasoning possible without dragging hardware and mechanism geometry into the same class.

This is better than combining controller and kinematics because a bad final motion would then be harder to diagnose: was the issue the control law or the mechanism transform?

**Open/Closed Principle**

Controller behaviour can be refined by changing gains, deadband policy, saturation rules, or confidence handling without rewriting downstream classes.

**Dependency Inversion Principle**

The controller depends on typed upstream estimate data, not on camera backends or servo implementations. That direction of dependency is correct because control logic is a policy layer, not a hardware layer.

### Why it is better than the alternative

A weaker design would send image error directly to servo commands. That would collapse control and mechanism mapping into one opaque stage and make both tuning and academic justification much harder.

---

## `ManualImuCoordinator`

### Why this class exists

`ManualImuCoordinator` is one of the most important design improvements in the current repository because it prevents `SystemManager` from absorbing every piece of manual-input and IMU-related policy.

Its responsibilities are explicitly documented in the implementation:

- mapping manual potentiometer samples to platform setpoints
- selecting whether potentiometer or GUI owns manual commands
- storing latest IMU sample and tilt estimate
- applying optional IMU correction to controller setpoints

That combination is coherent because all of those concerns are policy decisions about command formation, not hardware I/O and not top-level runtime orchestration.

### SOLID justification

**Single Responsibility Principle**

Its responsibility is not “manual plus IMU because they happened to fit somewhere”. Its responsibility is command policy coordination around manual control ownership and tilt-feedback correction. That is one policy boundary.

This is better than putting all of this into `SystemManager` because `SystemManager` already has to own lifecycle, worker startup/shutdown, queue consumption, and state transitions. Without this coordinator class, `SystemManager` would become a god class.

**Open/Closed Principle**

Different IMU feedback policies or command ownership policies can be introduced by modifying this policy layer without reworking the runtime orchestration layer.

**Dependency Inversion Principle**

The coordinator depends on pure mapping helpers (`ManualInputMapper`, `ImuTiltEstimator`, `ImuFeedbackMapper`) rather than directly embedding those algorithms in runtime code. That is the right dependency direction: orchestration uses policy objects, not the other way around.

### Why it is better than the alternative

The obvious bad alternative is a swollen `SystemManager` with interleaved branches for manual mode, GUI ownership, pot ownership, IMU correction, and state-dependent command generation. That alternative would be harder to test, harder to reason about, and harder to defend as SOLID.

This class is better because it removes policy branching from the orchestration class and keeps it in a focused coordination module.

---

## `ManualInputMapper`

### Why this class exists

`ManualInputMapper` exists to convert raw manual input values into platform commands. That mapping is pure logic. It should not live inside the ADS1115 input backend and should not live inside `SystemManager`.

### SOLID justification

**Single Responsibility Principle**

It changes only if the mapping from manual inputs to commanded motion changes.

That is better than placing the mapping inside the hardware input class because hardware acquisition and command semantics are different reasons to change.

**Dependency Inversion Principle**

Higher-level policy code uses a pure mapper rather than re-implementing scaling logic inline.

### Why it is better than the alternative

If raw ADC semantics and motion semantics were mixed together inside one device wrapper, the class would become half hardware driver, half control policy. That would be a worse architectural boundary.

---

## `ImuTiltEstimator`

### Why this class exists

`ImuTiltEstimator` converts reduced IMU acceleration data into a tilt estimate. It is intentionally stateless and mathematical. That is exactly the right design because tilt estimation is an algorithm, not a hardware or lifecycle concern.

### SOLID justification

**Single Responsibility Principle**

It changes only if the tilt estimation method changes.

**Dependency Inversion Principle**

Policy code depends on a pure estimator instead of directly embedding trigonometric logic inside runtime code.

### Why it is better than the alternative

Embedding tilt estimation into an IMU publisher or into `SystemManager` would mix sensor transport with interpretation. That would make the code less testable and blur the boundary between acquisition and estimation.

---

## `ImuFeedbackMapper`

### Why this class exists

`ImuFeedbackMapper` turns a measured tilt into a bounded correction term. This is pure control-support logic. It is not a sensor class and not a runtime class.

### SOLID justification

**Single Responsibility Principle**

It changes only when the correction policy changes: gain, deadband, or correction bounds.

**Open/Closed Principle**

Correction strategy can be refined without changing orchestration, backend startup, or servo output code.

### Why it is better than the alternative

If correction logic were embedded directly inside `Controller` or `SystemManager`, then control policy changes would drag unrelated code into the same modification set. This class keeps that correction behaviour explicit and testable.

---

## `Kinematics3RRS`

### Why this class exists

`Kinematics3RRS` transforms a platform setpoint into actuator commands for the 3-RRS mechanism. This must be separate from the controller because it is a mechanism-specific mapping, not a control-law decision.

This boundary is essential in a Stewart-platform-like project. Vision and control decide what the platform should do. Kinematics decides how this particular mechanism must move to achieve that.

### SOLID justification

**Single Responsibility Principle**

Its responsibility is mechanism mapping only. Geometry, calibration, and inverse-kinematics behaviour belong here. They do not belong in the controller and do not belong in the servo driver.

This is better than a merged controller/kinematics class because control tuning and mechanical geometry are fundamentally different causes of change.

**Open/Closed Principle**

Mechanism details can be recalibrated or reformulated here while preserving the upstream platform-command contract.

**Encapsulation**

The geometry and calibration parameters are contained in a dedicated configuration structure. That is better than scattering these constants through controller or hardware code.

### Why it is better than the alternative

The weaker alternative is direct controller-to-servo mapping. That would throw away the intermediate physical meaning of the platform setpoint and make the mechanism model implicit and opaque. This class is the stronger and more defensible architecture.

---

## `ActuatorManager`

### Why this class exists

`ActuatorManager` is the actuator safety-conditioning stage. It clamps outputs and applies slew-rate limiting before the final servo driver sees commands.

This is a very strong class boundary in the current repository because it makes safety shaping explicit rather than burying it inside the final hardware driver.

### SOLID justification

**Single Responsibility Principle**

Its responsibility is safety conditioning of actuator commands. It should not own hardware and should not own worker threads. That separation is explicitly stated in the class design and is the correct design for this project.

This is better than putting clamping and slew limiting into `ServoDriver` because the servo driver should be the final output stage, not the place where high-level motion safety policy is decided.

**Open/Closed Principle**

Safety policy can be changed here without redesigning PCA9685 access or degree-to-pulse mapping.

**Dependency Inversion Principle**

Downstream hardware receives already-conditioned commands. Hardware code therefore depends on a cleaner and safer input contract.

### Why it is better than the alternative

If safety shaping were hidden inside the hardware driver, then hardware output and motion-policy reasoning would be mixed. That would make testing harder and would hide a critical safety stage from the architecture. This class is better because it keeps safety conditioning as a first-class pipeline stage.

---

## `ServoDriver`

### Why this class exists

`ServoDriver` is the final output stage. Its documented responsibilities are:

- final degree-to-pulse mapping
- PCA9685 hardware access policy
- applying already-conditioned commands
- startup/stop parking

Its documented non-responsibilities are equally important:

- no control logic
- no kinematics
- no realtime scheduling
- no queueing

That is a very strong architectural boundary. It is exactly the kind of explicit responsibility statement that helps defend the class design.

### SOLID justification

**Single Responsibility Principle**

Its responsibility is final servo output handling, not command generation. It is the correct place for channel calibration, inversion, pulse conversion, and startup policy. It is not the correct place for deciding whether a command is safe or whether the sun has been detected.

This is better than a combined actuator controller because it keeps electrical/hardware mapping separate from motion semantics.

**Open/Closed Principle**

Different startup policies, parking policies, or calibration values can be supported without rewriting upstream control or kinematics logic.

**Dependency Inversion Principle**

The rest of the pipeline can reason in actuator-command units without depending on PCA9685 register access.

### Why it is better than the alternative

A weaker alternative would be one giant “servo subsystem” class that also did safety shaping, command generation, and hardware access. That would collapse several distinct reasons to change into one module. The current separation is better.

---

## `PCA9685`

### Why this class exists

`PCA9685` exists to encapsulate the low-level PWM chip behaviour. That is appropriate because register-level I2C interaction is not the responsibility of `ServoDriver` and certainly not the responsibility of control or orchestration code.

### SOLID justification

**Single Responsibility Principle**

It changes when the PWM chip access code changes, not when control policy changes.

**Dependency Inversion Principle**

Higher-level output code uses a device abstraction layer rather than embedding chip access everywhere.

### Why it is better than the alternative

Without a dedicated chip wrapper, low-level register access would leak into broader code. That would weaken readability, testability, and hardware substitution.

---

## `SystemManager`

### Why this class exists

`SystemManager` is the runtime orchestration boundary. That is the correct role for it. It composes the pipeline, owns worker threads, coordinates queue-driven processing, starts and stops optional backends, manages the tracker state machine, and exposes observer registration for external surfaces such as the GUI.

This class is large, but it is still architecturally justified because orchestration is a real responsibility in this project. What matters is whether it remains orchestration or turns into computation and hardware logic. In the current implementation, the most important thing is that several responsibilities have already been pushed out of it into dedicated classes.

### SOLID justification

**Single Responsibility Principle**

Its responsibility is runtime orchestration, not low-level device access and not pure algorithmic mapping. That is why classes such as `SunTracker`, `Controller`, `ManualImuCoordinator`, `Kinematics3RRS`, `ActuatorManager`, and `ServoDriver` are so important: they stop `SystemManager` from becoming an unbounded god class.

This is better than a single mega-class because the runtime manager can focus on lifecycle and event flow rather than the internals of every stage.

**Dependency Inversion Principle**

`SystemManager` depends primarily on stage abstractions and owned stage objects rather than embedding the low-level implementation details of every operation directly in itself.

**Encapsulation and safe interfaces**

It exposes observer registration and explicit control methods rather than encouraging direct data access. That is the correct encapsulation model for an event-driven system.

### Why it is better than the alternative

The bad alternative is an Arduino-style master loop or a monolithic class that performs acquisition, tracking, control, manual policy, IMU correction, kinematics, and hardware output inline. That alternative would be worse for timing, worse for testing, worse for maintainability, and weaker against the course criteria.

The current structure is stronger because `SystemManager` remains the coordinator around specialised stages rather than absorbing them.

---

## `ThreadSafeQueue`

### Why this class exists

`ThreadSafeQueue` is the blocking handoff mechanism between event-producing and event-consuming stages. That is the right role for it in this architecture because queueing and waiting are infrastructure concerns, not the responsibility of vision or control classes.

Its most important architectural value is not “being a queue”. Its value is that it provides a blocking wait interface and an explicit freshness-preserving `push_latest()` mode for bounded pipelines.

### SOLID justification

**Single Responsibility Principle**

Its responsibility is inter-thread queue handoff. It does not perform tracking, control, or hardware access.

**Open/Closed Principle**

The queue policy is encapsulated in the queue class rather than being reimplemented ad hoc throughout the codebase.

**Safe data management**

This class is important for the course criteria because it uses STL-managed storage, explicit stop/reset behaviour, blocking waits, and bounded latest-only insertion policy. That is much safer than manual shared-state polling.

### Why it is better than the alternative

The weaker alternative is shared mutable state plus polling loops. That would be less deterministic, more CPU-wasteful, and harder to reason about. A blocking queue is the better design for this repository.

---

## `LinuxEventLoop`

### Why this class exists

`LinuxEventLoop` keeps `main()` small and isolates headless application control events from the realtime processing core. That is the right boundary because CLI shutdown handling and OS event waiting are application-shell concerns, not part of the tracking pipeline.

### SOLID justification

**Single Responsibility Principle**

It is responsible for the headless application event loop only. It is explicitly not the sensor/control loop.

This is better than pushing CLI shutdown logic into `SystemManager` because orchestration of the tracker runtime and shell-level process control are not the same job.

### Why it is better than the alternative

Without this class, `main()` or `SystemManager` would take on extra process-control responsibilities. That would muddy the design and weaken the architecture argument.

---

## `SystemFactory`

### Why this class exists

`SystemFactory` is the composition root. It selects and constructs the correct runtime graph while keeping that assembly logic out of `main()`.

That is exactly the right architectural role for a factory here. The project has multiple backend choices and many cooperating classes. Construction logic should not be smeared across application entry points.

### SOLID justification

**Single Responsibility Principle**

Its responsibility is runtime assembly. It should not run the system, process frames, or operate hardware directly.

**Dependency Inversion Principle**

It centralises where concrete types are chosen while keeping the running system dependent on narrower interfaces and stage contracts.

### Why it is better than the alternative

A weaker alternative is to assemble the full runtime directly in `main()` or across several unrelated files. That would make construction logic harder to maintain and harder to explain. A composition root is the cleaner design.

---

## Why callbacks are the correct inter-class design here

The project architecture is strongest where it uses callbacks to push events forward rather than getters to pull data backward.

That matters because this is a realtime event-driven system. When a frame arrives, the system should process that event and propagate the result forward. It should not rely on downstream code polling upstream objects for “latest values”. Polling would weaken responsiveness, blur timing ownership, and encourage stale shared-state designs.

Callbacks are therefore not just a stylistic preference here. They are the correct fit for the taught architecture and for this repository’s pipeline structure.

Using callbacks between stages is better than getter-driven designs because:

- event timing remains explicit
- freshness is preserved
- ownership boundaries stay cleaner
- stage coupling is reduced
- blocking/event-driven sensor paths can wake a worker and push data forward immediately

In this project, callback boundaries support both realtime reasoning and SOLID reasoning. They help keep classes small, event-focused, and decoupled.

---

## Why setters remain the correct output-side design

On the output side, setter-style interfaces remain the right choice. Upstream logic should emit typed commands into downstream stages. It should not manipulate hardware internals.

That is why the output side of the pipeline is stronger with:

- `Controller` producing platform setpoints
- `Kinematics3RRS` producing actuator commands
- `ActuatorManager` conditioning those commands
- `ServoDriver` applying final hardware-oriented output behaviour

Each step hands forward a more concrete command. That staged setter-style flow is better than letting an upstream class reach into downstream internals.

---

## Why this class structure is stronger than a monolithic alternative

A monolithic tracker class would be worse in every way that matters for this project.

It would:

- mix Linux I/O, camera backend selection, vision, control, kinematics, safety logic, and servo hardware access
- make unit testing much harder
- make failure localisation harder
- increase the risk of accidental polling-style or mixed-timing designs
- weaken the argument for encapsulation and maintainability
- make the system harder to extend with alternative backends or policies

The current class structure is stronger because it preserves distinct architectural layers:

- acquisition boundary
- estimation boundary
- control boundary
- policy coordination boundary
- mechanism mapping boundary
- safety boundary
- final hardware boundary
- orchestration boundary
- application-shell boundary

That separation is exactly what allows the project to argue for both realtime correctness and software-engineering quality.

---

## Final judgement

The strongest SOLID choices in this repository are not abstract textbook gestures. They are the class boundaries that prevent the project from collapsing into one tightly coupled runtime blob.

The most defensible decisions are:

- `ICamera` as the acquisition abstraction
- backend isolation in `LibcameraPublisher` and `SimulatedPublisher`
- pure logic separation in `SunTracker`, `Controller`, `ManualInputMapper`, `ImuTiltEstimator`, and `ImuFeedbackMapper`
- policy isolation in `ManualImuCoordinator`
- mechanism separation in `Kinematics3RRS`
- explicit safety conditioning in `ActuatorManager`
- final hardware mapping isolation in `ServoDriver` and `PCA9685`
- orchestration centralisation in `SystemManager`
- composition isolation in `SystemFactory`

That is why this class structure is stronger than the obvious alternatives. It improves maintainability, preserves event-driven design, isolates hardware volatility, supports testing, and makes the system defendable against the ENG5220 structure and realtime criteria.