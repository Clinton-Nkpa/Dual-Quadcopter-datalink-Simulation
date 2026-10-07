# Robust Nonlinear Flight-Control Development TODO

**Project:** Dual-Quadcopter Simulation
**Status date:** 7 October 2026
**Purpose:** Turn the current simulation baseline and the 12 stated software/hardware priorities into a sequenced, evidence-based development plan.

## 1. Objective

The next major engineering question is:

> Can the controller stabilise a physically imperfect simulated aircraft when its actuators and measurements are imperfect?

The immediate work is to strengthen the single-aircraft simulation and make its controller implementation testable before H743 integration. This is not yet dual-aircraft coordination work. The software distinguishes plant **true state** from controller **estimated state**: truth is available to plant and evaluation code, while the imperfect-sensor controller path receives estimator output only. The nominal profile remains an explicit ideal-observation baseline.

```text
                          ┌──────────────────┐
                          │   6-DoF plant    │
                          └────────┬─────────┘
                                   │
                     ┌─────────────┴───────────────┐
                     ▼                             ▼
              true plant state               sensor models
              (score/log only)          noise, bias, scale, delay,
                     │                  drift and fault injection
                     ▼                             ▼
              evaluation only                state estimator
                                                   │
                                                   ▼
                                          estimated vehicle state
                                                   │
                                                   ▼
                                             controller
                                                   │
                                                   ▼
                                           motor commands
                                                   │
                                                   ▼
                                       motor / ESC / rotor model
                                                   │
                                         thrust and torque
                                                   └──────► 6-DoF plant
```

The architecture above is partly implemented. The nominal profile deliberately uses an ideal-state adapter. The imperfect profile feeds seeded IMU/barometer measurements through a simple attitude/altitude estimator, and the controller receives only that estimator output. This is a functional software separation, not a validated or flight-grade estimator.

## 2. What Python can cover

Five of the twelve priorities can be exercised directly inside a Python simulation: **#1, #2, #5, #6, and #7**. Of these, the existing code only partially covers **#2 and #5**. Priorities **#3 and #4** can also be completed before H743 hardware is available, but they require a host C compiler and a Python-to-C test harness rather than Python simulation alone. The software-only portion of **#10 (SITL)** can also be tried using an external flight stack and simulator; its HIL portion needs hardware.

| Priority | Work | Python simulation can establish | Boundary / current status |
|---:|---|---|---|
| 1 | Hardware-resource manifest | Schema validity, internal consistency, resource estimates, and traceability checks | It cannot prove board pins or fitted components. Manifest/tooling exist; all ten physical assignments remain unresolved pending exact board revision and authoritative evidence. |
| 2 | Python nonlinear reference controller | Controller response to setpoints and simulated states | Cascaded baseline and explicit controller interface exist. Nominal profile uses ideal observation; imperfect profile uses estimated state. Gains are not aircraft-tuned. |
| 3 | Portable C reproduction | Not Python-only; Python can prepare inputs and drive a host-built C implementation | Portable C core and GCC host build implemented; no H743 required for host parity. |
| 4 | Python ↔ C numerical equivalence | Replay identical inputs and compare pre-HAL outputs | Deterministic replay harness and preserved corpus implemented; host numerical parity only. |
| 5 | 6-DoF plant + motor/ESC model | Rigid-body motion, motor response, thrust, torque, limits, and selected actuator effects | A baseline NED/FRD plant, RK4 integration, first-order rotor-speed response, mixer, and quadratic thrust/reaction-torque model exist. ESC, battery/voltage, and identified parameters are incomplete. |
| 6 | Sensor/actuator fault injection | Simulated noise, bias, delay, drift, scale error, mismatch, saturation, dropout, lag variation, or degraded motors | Seeded demonstration models, estimator, timing effects, command faults, and per-motor lag scaling are implemented. Values remain illustrative. |
| 7 | Motor order / rotation direction | Check software motor indexing, mixer mapping, and expected force/moment signs | Model-level checks pass. Actual motor wiring and spin direction still need physical validation. |
| 8 | Fixed-rate H743 timing | Model nominal rate, jitter, missed updates, and scheduling effects | This can explore controller sensitivity, but only the target MCU can establish actual execution time and deadline behavior. |
| 9 | H743 peripheral / driver bring-up | Mock interfaces and test software contracts | Actual IMU, timer, UART, receiver, and ESC driver bring-up requires the selected board and connected devices. |
| 10 | SITL / HIL | SITL integration is potentially software-only with an external simulator/flight stack | This repository does not currently provide that integration. HIL requires the flight controller and a supported interface/setup. |
| 11 | Props-off validation | Checklists and expected state-machine behavior can be rehearsed in software | Real props-off checks require the aircraft, board, power setup, and safe bench procedure. |
| 12 | Flight testing | Simulated maneuvers and disturbances | Real flight requires the completed aircraft, passed earlier gates, and a controlled test environment. |

## 3. Verified baseline and execution notes

The current 6-DoF runner was executed for the existing attitude-step scenario using the repository virtual environment:

```bash
.venv/bin/python -m private.integration.run_quadrotor_6dof \
  --scenario attitude-step \
  --duration 8 \
  --dt 0.002 \
  --csv /tmp/priority-boundary-quad.csv
```

The run completed 8 simulated seconds at a 0.002 s integration step and reported a final altitude of 2.004 m for the 2.0 m setpoint. This shows that the present simulation path executes. It does **not** establish calibrated parameters, robustness, sensor-based control, stability margins, or flight readiness. On this host, `python` was not on `PATH`; invoking `.venv/bin/python` worked. Use the project virtual environment (or activate it) when following launch commands.

Relevant current files:

- `simulation/quadrotor_6dof.py` — NED/FRD state, rotor wrench, mixer allocation, motor-speed response, and rigid-body integration (public plant model).
- `private/controller/quadrotor_flight_controller.py` — local cascaded attitude/rate and altitude controller; excluded from the public repository.
- `private/integration/run_quadrotor_6dof.py` — local closed-loop scenarios and CSV telemetry; depends on private modules.
- `private/integration/visualize_quadrotor_6dof.py` — local animation of the closed-loop run.

The default mass, geometry, inertia, thrust and yaw coefficients, drag, and motor limits are illustrative values. They must not be presented as measured aircraft properties.

## 3.1 Work completed against this plan

The following software work has now been added in this checkout. The entries below report implementation and execution evidence only; they do not close the hardware gates.

- The public plant is in `simulation/quadrotor_6dof.py`. The H743 resource manifest and validator are retained under local-only `private/hardware/` and `private/tools/`; the manifest lists ten logical resources, but all physical assignments remain unresolved pending exact board evidence.
- The public API contract is in `interfaces/vehicle_interfaces.py`. The controller, estimator and closed-loop runner are local-only under `private/controller/`, `private/estimator/` and `private/integration/`. The nominal runner uses an explicit ideal-state adapter; the imperfect profile routes sensor samples through an estimator. The nominal path remains idealized, not sensor-based.
- I ported the current Python controller equations into the portable host C module `private/controller/flight_controller.c/.h`. With GCC 13.3 and `-Wall -Wextra -Werror`, `private/tools/compare_controller.py` compiled the C module and compared 504 sequential cases. Maximum motor-command difference was `9.38e-13 rad/s` against an absolute tolerance of `1e-8`; saturation flags and integral state matched, and both implementations rejected a zero timestep. The input/output corpus is retained locally in `private/verification/controller_equivalence_vectors.json`. This is host numerical parity, not H743 timing or firmware validation.
- Public model checks in `simulation/tests/verify_model_mapping.py` checked all four individual motor force/moment sign conventions, 100 allocation round-trips, and infeasible-wrench saturation detection. These checks establish consistency inside the simulator only; they do not verify physical motor wiring or spin direction.
- `simulation/tests/verify_plant.py` checked exact hover and timestep refinement. An exact hover rotor-speed initial condition remained stationary for 1.0 simulated second. Refining the RK4 step from 0.004 s to 0.002 s reduced the final-state difference relative to a 0.001 s reference from `1.86e-7` to `1.05e-8` in the selected short transient. These are internal numerical checks with illustrative parameters.
- I added seeded IMU/barometer models, an attitude/altitude estimator, motor-command mismatch/limit/delay/stuck-motor hooks, and controller update jitter/missed-tick modeling. The CSV now records truth, measurements, estimates, requested wrench, motor speeds and commands, applied commands, fault/source flags, random seed, and update counts. Noise/bias/quantization values are demonstration settings, not measured specifications.
- I executed separate and combined fault runs. The four-second sensor-only case ended at about 1.89 m altitude with roughly 10.7° roll and 9.5° pitch; the four-second 2% motor-command mismatch case ended at about 1.97 m with roughly 4.8° pitch error. The eight-second noisy-sensor plus mismatch/dropout case completed at about 1.87 m. These runs show the fault paths execute, but no pass/fail envelope has been declared, and these errors are not evidence of robust stabilization. A one-second motor-2-loss run diverged rapidly, as a fault response study should expose; no recovery claim is made.
- I exercised the scheduler model for two simulated seconds with a four-step nominal period, one-step jitter, and every tenth scheduled tick missed. It reported 226 controller updates and 25 missed ticks. This is an assumed software schedule and says nothing about actual H743 interrupt timing.

No exact H743 board/PCB revision is identified in the repository, and no `/dev/ttyACM*` or `/dev/ttyUSB*` device nodes were present during this check. PX4, `sim_vehicle.py`, Gazebo `gz`, and `arducopter` executables were also not available on `PATH`, so SITL/HIL integration could not be performed here. Prior hardware-manifest entries remain unresolved until the board evidence is supplied and checked.

## 4. Detailed work packages

The priority numbers below preserve the requested order. Each package separates software evidence from claims that require hardware.

### Priority 1 — Hardware-resource manifest

**Goal:** Create a versioned source of truth for the intended H743 target, sensors, buses, timers, serial ports, receiver, ESC outputs, motor indices, power system, and software ownership of each resource.

- [ ] Identify the exact flight-controller board model and PCB revision.
- [ ] Record each resource with name, MCU pin/peripheral, bus/instance, direction, intended device, source document, confidence, and unresolved status.
- [x] Validate the manifest's schema, duplicate pins/timers/buses, missing assignments, and conflicts automatically.
- [x] Keep confirmed, inferred, and unresolved entries distinct. Do not turn contradictory pinout documentation into wiring instructions.
- [ ] Trace each claim using this evidence order: exact board schematic/PCB revision; MCU datasheet and reference manual; physical continuity/electrical measurement; firmware hardware definition/source; build output/runtime detection; README/wiring documentation.

**Acceptance evidence:** A checked-in manifest, a machine-readable validation report, and a source reference for every assignment. Unresolved entries remain visibly unresolved until verified.

**Python boundary:** Python can validate and report the manifest. It cannot certify that the physical board matches the files.

### Priority 2 — Python nonlinear reference controller

**Goal:** Preserve a deterministic, inspectable reference controller with explicit units, frames, setpoints, limits, and outputs. This reference is the numerical specification for the later C port.

- [x] Define stable `ControllerState`, setpoint, controller configuration, and `MotorCommand` interfaces.
- [x] Document units and conventions: NED world frame, FRD body frame, radians internally, motor numbering, and the meaning of every sign.
- [x] Separate controller logic from simulator access and logging.
- [x] Separate the controller-facing estimated-state interface from plant truth; only the explicit nominal adapter supplies ideal state, and the imperfect path has no truth-state fallback.
- [x] Preserve the present bounded-integral controller as a baseline for anti-windup comparisons.
- [x] Define controller reset, initialization, invalid-input, and saturation behavior.

**Acceptance evidence:** Reproducible setpoint scenarios, finite command outputs, explicit interface documentation, and logs that record the controller input state and output commands. Controller gains remain provisional until parameters are measured and robustness criteria are met.

### Priority 3 — Portable C reproduction

**Goal:** Reproduce the validated controller equations in a host-buildable C core before adding STM32-specific drivers.

- [x] Implement a portable controller module with no STM32 HAL dependency.
- [x] Match Python units, frame conventions, motor order, initialization, saturation, and update timestep.
- [x] Define explicit JSON input/output serialization for deterministic replay cases.
- [x] Compile with a host C compiler using warnings enabled; record compiler/version and build command.
- [x] Keep hardware conversion (PWM/DShot units, timer registers, sensor buses) outside the mathematical controller core.

**Acceptance evidence:** A repeatable host build, documented C API, and input/output examples that can be fed to both reference and C implementations.

**Boundary:** This is host software work, not a Python-only simulation task. It does not require H743 hardware.

### Priority 4 — Python ↔ C numerical equivalence

**Goal:** Show that the same controller state, setpoint, timestep, and controller history yield matching Python and C outputs before hardware integration.

- [ ] Add deterministic replay vectors spanning hover, attitude steps, combined commands, yaw wrap, rate limiting, integral limits, and invalid input handling.
- [x] Feed identical values, initialization, and update order to Python and host C.
- [x] Compare per-motor commands and mixer-saturation flags; requested wrench comparison remains future work.
- [x] Define the absolute command tolerance before evaluation (`1e-8 rad/s`) and document the host double-precision build.
- [x] Save the deterministic passing corpus; add a failure-artifact path if a mismatch occurs.

**Acceptance evidence:** A passing equivalence report over the declared cases, with maximum absolute/relative command error and a preserved test corpus. Do not claim parity based only on matching one hover point.

### Priority 5 — 6-DoF plant and motor/ESC model

**Goal:** Improve and validate the simulated aircraft dynamics and actuator chain while keeping parameters identified versus illustrative clearly labeled.

- [x] Verify level hover force balance from an exact hover rotor-speed initial condition.
- [x] Check roll, pitch, yaw, and collective force/moment signs one axis at a time.
- [x] Check combined commands and motor allocation at unsaturated and saturated points.
- [x] Add software hooks for command mismatch/limits/delay/stuck motor and per-motor rise/fall lag variation; values are illustrative and not calibrated.
- [ ] Refine motor/ESC command limits, rise/fall lag, deadband, and nonlinear response as supported by data.
- [ ] Add battery voltage as an explicit input to available motor authority and thrust only when the relationship is parameterized or clearly marked as a bounded assumption.
- [x] Compare responses across integration timestep sizes to detect numerical sensitivity.
- [ ] Replace illustrative mass, center of gravity, arm geometry, inertia, and propulsor coefficients with measured or defensibly estimated values when available.

**Acceptance evidence:** Hover and maneuver traces, force/moment sign table, saturation cases, parameter provenance, and timestep-convergence results. Simulation checks validate code consistency, not real aerodynamic accuracy.

### Priority 6 — Sensor and actuator fault injection

**Goal:** Determine how modeled imperfections affect the controller and estimator, using seeded, repeatable scenarios.

- [x] Add isolated sensor model settings for noise, constant bias, drift, scale error, quantization, delay, and dropped/stale samples.
- [x] Add isolated actuator cases: per-motor lag mismatch, output limit, saturation, degraded thrust, command delay, and motor dropout.
- [x] Validate sensor-model output statistics against configured distributions and preserve random seeds.
- [ ] Test one fault at a time before combinations; state explicitly when a fault may be uncontrollable (for example, total motor loss).
- [ ] Log true state, sensor samples, estimator output, setpoint, requested/achieved wrench, motor commands/speeds, saturation and fault flags, voltage, and seed.
- [x] Ensure the imperfect-sensor controller path sees only sensor-derived estimated state once the estimator is connected.

**Acceptance evidence:** Reproducible fault cases, validated sensor statistics, recorded estimator/tracking errors, and clear classifications of detectable, tolerable, or uncontrollable cases. No fault case is assumed recoverable without evidence.

### Priority 7 — Motor order and rotation-direction validation

**Goal:** Prevent frame, indexing, or sign mistakes from being hidden by aggregate telemetry.

- [x] Document model motor index, frame position, unknown physical spin direction, and reaction-torque sign in the manifest mapping table.
- [x] In Python, command one motor at a time and verify expected collective, roll, pitch, and yaw wrench changes.
- [x] Test the allocator and inverse mapping with individual and combined requests, including infeasible/saturated requests.
- [x] Keep simulated direction checks clearly labeled as software checks.
- [ ] At hardware stage, confirm actual motor order and rotation direction with propellers removed and the power procedure defined.

**Acceptance evidence:** Automated model-level mapping/sign report plus a separate signed hardware checklist when the aircraft is available. Python cannot establish physical wiring or rotor spin direction.

### Priority 8 — Fixed-rate H743 timing

**Goal:** Design a fixed-period controller update and later demonstrate it meets the selected MCU's timing requirements.

- [ ] Specify the target update period, scheduling assumptions, timestamp source, missed-deadline behavior, and watchdog/failsafe response.
- [x] In simulation/host replay, inject controller update-period jitter and missed updates to examine controller sensitivity.
- [ ] On the actual H743, measure worst-case execution time under representative load using a hardware timer or GPIO instrumentation.
- [ ] Verify deadline margin, interrupt priorities, data ownership, and behavior after stale sensor input or an overrun.

**Acceptance evidence:** A software timing model followed by measured target timing traces and an explicit deadline pass criterion. A Python wall-clock benchmark is not H743 timing evidence.

### Priority 9 — H743 peripheral and driver bring-up

**Goal:** Connect tested software interfaces to the selected board's actual IMU, timers, receiver, telemetry link, and ESC outputs.

- [ ] Freeze the board revision and reconcile the resource manifest before assigning pins.
- [ ] Bring up one peripheral at a time and record detected device identity, bus settings, sample/update rate, errors, and recovery behavior.
- [ ] Validate sensor orientation, units, bias/calibration path, and timestamps before exposing measurements to the estimator.
- [ ] Validate timer output mapping and safe output states before connecting powered actuators.
- [ ] Add watchdog, arming interlocks, link-loss behavior, low-voltage behavior, and emergency disarm paths.

**Acceptance evidence:** Board-specific build/runtime detection, captured logs, pin/resource evidence, and documented safe output behavior. Driver mocks are useful for software contracts but do not complete bring-up.

### Priority 10 — SITL / HIL

**Goal:** Validate integration with an autopilot/flight stack and, later, the actual controller hardware in a closed simulation loop.

- [ ] Select and version a compatible SITL/flight-stack setup; document vehicle parameters, transport, and protocol.
- [ ] Connect the Python reference controller or portable C core through a defined interface and check units, update rates, and message timeouts.
- [ ] Test arming, mode changes, setpoint tracking, telemetry loss, stale state, and failsafe transitions in SITL.
- [ ] Define a supported HIL path for the H743, simulator, sensors, and actuator outputs before wiring it.
- [ ] In HIL, log synchronized simulator truth, hardware sensor/estimate data, controller outputs, and timing/transport status.

**Acceptance evidence:** Repeatable SITL scenario logs and then separately approved HIL logs with no unexplained data-path or deadline errors. SITL does not validate H743 peripherals; HIL requires physical hardware.

### Priority 11 — Props-off validation

**Goal:** Check hardware interfaces and safety behavior with propellers removed before any powered-propeller test.

- [ ] Write and review a bench procedure covering battery isolation, propeller removal, arming, emergency stop, and safe handling.
- [ ] Confirm board boot, sensor orientation, receiver inputs, modes, telemetry, motor numbering, output limits, watchdog, link loss, low-voltage response, and disarm.
- [ ] Verify output behavior with a safe test method appropriate to the ESC and motor setup; keep propellers off.
- [ ] Capture a signed checklist and timestamped logs; stop on unexplained motor output or safety-state behavior.

**Acceptance evidence:** Completed bench checklist and traceable output/failsafe evidence. Software simulation can rehearse logic but cannot replace these physical checks.

### Priority 12 — Flight testing

**Goal:** Progress from a validated single aircraft to controlled flight only after all preceding evidence gates pass.

- [ ] Define a controlled, permitted test environment, aircraft configuration, operating limits, pilot override, and abort criteria.
- [ ] Use incremental single-aircraft test cards, beginning with low-risk behavior and reviewing logs between steps.
- [ ] Confirm that each anomaly has a bounded response and that failsafes have already been exercised on the bench/SITL/HIL path.
- [ ] Add a second aircraft only after each aircraft passes its own flight gates; then separately test link loss, separation, and cooperative mission behavior.

**Acceptance evidence:** Approved test cards, repeatable logs, reviewed anomalies, and explicit sign-off at each expansion of the test envelope. Simulation and software parity alone are not flight authorization.

## 5. Recommended implementation sequence and gates

The priority order is retained, with a few dependencies made explicit:

1. **Software foundation (1–4):** establish the resource manifest and Python interface, port controller equations into host-buildable C, and compare both implementations with the same replay vectors. Work on #3–4 can proceed before H743 hardware, but it depends on a stable Python controller contract.
2. **Plant and robustness (5–7):** improve the plant/actuator model, inject sensor and actuator imperfections, and verify software motor mapping. Keep true-state scoring separate from the controller input path.
3. **Target integration (8–9):** model timing sensitivity first, then measure fixed-rate timing and bring up H743 peripherals on the actual board.
4. **System validation (10–12):** validate in SITL, then HIL, then props-off on hardware, and only then progress to controlled flight.

### Gate A — Python reference ready for comparison

- [ ] Units, frames, motor numbering, controller state, and reset behavior are documented.
- [x] Baseline cases are deterministic and logged.
- [x] Controller input is an explicit state interface that can carry estimated state without exposing plant internals.

### Gate B — Host C matches Python

- [x] Declared generated replay set passes the declared numeric tolerance.
- [x] Saturation, initialization, update ordering, yaw angle wrapping, and invalid-timestep behavior match in the exercised cases.

### Gate C — Nonlinear robust envelope established

- [x] The baseline plant passes equilibrium, sign, and timestep checks; this does not validate physical accuracy.
- [x] Configured sensor model statistics are verified independently against seeded distributions.
- [x] The imperfect-sensor controller path consumes estimated state only; the nominal ideal-observation path is explicitly marked.
- [ ] Pass/fail bounds for tracking error, estimator error, recovery time, saturation duration, and timing are declared before parameter sweeps.
- [ ] Results preserve configuration, seed, model revision, and relevant truth/measurement/command logs.

### Gate D — H743 integration and physical checks

- [ ] Exact board/resources are verified and the H743 timing budget is measured on target.
- [ ] Peripheral, actuator, watchdog, and failsafe checks pass in documented bench/SITL/HIL stages.
- [ ] Props-off checks pass before any flight test.

## 6. Constraints and decision rules

- **Illustrative parameters:** Current model constants are not measured aircraft data. Keep assumptions explicit, and do not tune hardware from them as if they were identified values.
- **True-state separation:** The nominal baseline intentionally uses ideal plant-state observation. The imperfect profile uses the estimator output only. The current estimator is simple and not flight-grade, so sensor-fault results remain preliminary.
- **Actuator-model scope:** The current model contains rotor-speed lag and a mixer, not a complete ESC, PWM/DShot, voltage, or propeller-inflow model.
- **Board ambiguity:** Hardware resources cannot be finalized from conflicting README/pinout entries. Confirm the exact board revision and use the resource-evidence chain in Priority 1.
- **Software versus target evidence:** Host C tests establish numerical behavior on the host. Python timing simulations establish sensitivity to assumed jitter. Neither establishes STM32H743 execution time, peripheral correctness, or safety behavior.
- **Fault limits:** A simulation can reveal that a fault exceeds control authority. Do not label total motor-loss or other severe faults recoverable without a validated controllability basis.
- **Flight boundary:** Do not advance from software evidence directly to free flight. Preserve SITL/HIL, props-off, and controlled test gates as separate evidence stages.

## 7. Immediate next actions

1. [x] Define an initial manifest schema and populate known logical resources with explicit unresolved status; authoritative physical citations still need the exact board documents.
2. [x] Freeze the controller state/setpoint/output interface and establish a deterministic replay harness; the corpus remains local under `private/verification/controller_equivalence_vectors.json` and is excluded from the public repository.
3. [x] Implement and host-build a portable C controller with the same equations and explicit double-precision types.
4. [x] Add Python-driven equivalence checks and declare the `1e-8 rad/s` command tolerance.
5. [x] Verify plant hover balance, model/mixer signs, and timestep refinement; continue motor/ESC calibration and voltage modeling when evidence is available.
6. [x] Add seeded sensor and actuator fault injection, estimator module, controller-facing estimate interface, per-motor lag variation, and sensor-statistics verification. More fault characterization and the robustness envelope remain open.
7. [ ] Define robustness pass/fail limits before broad sweeps; then proceed through target timing, peripherals, SITL/HIL, props-off, and flight gates.

**Current claim boundary:** The nominal eight-second attitude-step run completed at 2.004 m for a 2.0 m target. The portable C controller matches Python across 504 replay cases to a maximum command difference of `9.38e-13 rad/s`; the 20,000-sample sensor-statistics check, model mapping checks, one-second hover equilibrium, and timestep refinement also pass. Imperfect sensor, actuator-lag mismatch, dropout, and scheduler perturbation paths execute, but no robustness pass/fail envelope has been declared. These results do not establish calibrated dynamics, H743 timing, hardware operation, or flight performance.
