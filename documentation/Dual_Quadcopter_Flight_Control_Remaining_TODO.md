# Dual-Quadcopter Flight-Control Development
## Bespoke Remaining TODO & Qualification Plan
**Status: 7 October 2026**

This plan treats the remaining work as a qualification programme: turning the existing simulation, controller implementation, verification framework and hardware plan into an evidenced, bounded and eventually hardware-verifiable flight-control system.

---

# PHASE 0 — Freeze the engineering contract

## 0.1 Coordinate, frame and unit conventions
- [ ] Formally document NED world frame and FRD body frame.
- [ ] Freeze roll/pitch/yaw sign conventions.
- [ ] Freeze radians, rad/s, m, m/s, m/s², N and N·m units.
- [ ] Freeze motor numbering and positive rotation directions.
- [ ] Freeze thrust and torque sign conventions.
- [ ] Create authoritative `CONVENTIONS.md`.
- [ ] Ensure Python, C and H743 firmware use the same conventions.
- [ ] Add assertions/tests for frame and unit assumptions.
- [ ] Document true, measured, estimated, commanded and achieved state.

**Acceptance:** A new developer can implement the controller without guessing a sign, axis, unit or frame.

# PHASE 1 — Hardware resource evidence

## 1.1 Identify the exact H743 hardware
- [ ] Identify board manufacturer, model and PCB revision.
- [ ] Identify exact MCU variant and oscillator.
- [ ] Identify IMU(s), barometer, ADCs, UARTs, SPI/I²C buses, timers, DMA, PWM/DShot outputs and CAN if required.
- [ ] Obtain exact schematic/PCB documentation.
- [ ] Record board revision in the resource manifest.

## 1.2 Evidence chain
For every physical resource:

**Claim → source → verification**

Example: `Motor 1 → TIMx_CHy → board schematic → physical continuity → firmware configuration`

- [ ] Cite schematic evidence.
- [ ] Cross-check MCU alternate-function mapping.
- [ ] Cross-check firmware resource declaration.
- [ ] Physically verify critical pins where necessary.
- [ ] Record unresolved resources explicitly.

## 1.3 Reference-board separation
- [ ] Separate reference architecture from the actual board.
- [ ] Do not inherit pin assignments from a README/hwdef without verification.
- [ ] Resolve contradictory IMU, motor, UART and timer assignments.
- [ ] Mark assignments verified, inferred, provisional or unresolved.

## 1.4 Resource-manifest gate
- [ ] Strict validation fails if a required flight resource is unresolved.
- [ ] Simulation/host validation remains independent of unresolved hardware resources.
- [ ] Produce final machine-readable manifest.

**Gate:** No hardware-specific controller claim until the resource manifest is completely evidenced.

# PHASE 2 — Complete deterministic controller equivalence

The Python/C foundation already exists, including 504 sequential cases and a maximum motor-command difference of approximately `9.38 × 10⁻13 rad/s`.

## 2.1 Expand deterministic replay corpus
- [ ] Hover.
- [ ] Roll step.
- [ ] Pitch step.
- [ ] Yaw step.
- [ ] Combined attitude command.
- [ ] Positive and negative yaw wrap.
- [ ] Body-rate limiting.
- [ ] Integral accumulation and limiting.
- [ ] Saturation and recovery.
- [ ] Invalid timestep.
- [ ] Invalid/non-finite input.
- [ ] Large attitude error.
- [ ] Zero/near-zero thrust.
- [ ] Maximum allowable command.

## 2.2 Compare internal results
- [ ] Attitude/rate errors.
- [ ] Integral state.
- [ ] Requested torque.
- [ ] Collective thrust.
- [ ] Requested wrench.
- [ ] Motor commands.
- [ ] Saturation flags.
- [ ] Controller validity/error state.

## 2.3 Golden corpus
- [ ] Store public test vectors under `simulation/tests/`; keep private controller input/output vectors under local-only `private/verification/`.
- [ ] Version-control them.
- [ ] Automatically run them locally/under CI.
- [ ] Fail on tolerance violations.
- [ ] Preserve failure artifacts.

## 2.4 Numerical tolerances
- [ ] Document the `1e-8 rad/s` numerical tolerance.
- [ ] Distinguish numerical tolerance from behavioural and physical tolerances.

**Gate B:** Python and C remain equivalent across the expanded corpus.

# PHASE 3 — Make the vehicle model defensible

## 3.1 Mass
- [ ] Measure complete aircraft mass.
- [ ] Measure frame, motors, ESC, FC, battery, wiring, sensors, Raspberry Pi and payload.
- [ ] Record operational mass range.
- [ ] Define nominal simulation mass and uncertainty.

## 3.2 Centre of gravity
- [ ] Measure CG in body coordinates.
- [ ] Record CG relative to geometric centre, motor plane and IMU.
- [ ] Model CG offset.
- [ ] Test sensitivity to CG displacement.

## 3.3 Geometry
- [ ] Measure motor-to-centre distances.
- [ ] Measure arm length and motor locations.
- [ ] Measure propeller diameter and pitch.
- [ ] Record body, IMU and battery locations.

## 3.4 Inertia
Establish a defensible inertia tensor:

J = [[Jx, Jxy, Jxz], [Jyx, Jy, Jyz], [Jzx, Jzy, Jz]]

- [ ] Obtain defensible inertia estimates.
- [ ] Determine whether diagonal-only inertia is adequate.
- [ ] Document assumptions.
- [ ] Add uncertainty bounds.

## 3.5 Motor/propeller model
Use defensible values for:

T = kT * omega²
Q = kQ * omega²

- [ ] Determine thrust coefficient.
- [ ] Determine reaction-torque coefficient.
- [ ] Determine maximum rotor speed.
- [ ] Determine minimum controllable speed.
- [ ] Determine motor response time.
- [ ] Determine ESC response delay.
- [ ] Determine command-to-speed dynamics.
- [ ] Determine voltage dependence.

## 3.6 Battery model
- [ ] Add battery voltage input.
- [ ] Model nominal voltage.
- [ ] Model loaded voltage and sag.
- [ ] Add state-of-charge approximation.
- [ ] Model voltage-dependent motor capability.

At minimum: `V(t) → omega_max(t) → T_max(t)`

## 3.7 Propulsion validation
- [ ] Compare predicted thrust with manufacturer/bench data.
- [ ] Compare predicted motor response with measured response.
- [ ] Record uncertainty.

**Gate C prerequisite:** Plant parameters have a documented evidence basis.

# PHASE 4 — Define the nonlinear test envelope

Before broad sweeps, define pass/fail criteria.

## 4.1 Attitude tracking
- [ ] Maximum attitude error.
- [ ] RMS attitude error.
- [ ] Rise time.
- [ ] Settling time.
- [ ] Overshoot.
- [ ] Steady-state error.

## 4.2 Body-rate tracking
- [ ] Maximum rate error.
- [ ] RMS rate error.
- [ ] Recovery time.

## 4.3 Altitude
- [ ] Maximum deviation.
- [ ] Steady-state deviation.
- [ ] Settling time.

## 4.4 Estimator
- [ ] Roll error.
- [ ] Pitch error.
- [ ] Yaw error.
- [ ] Altitude error.
- [ ] Velocity error.

## 4.5 Actuator utilisation
- [ ] Maximum motor command.
- [ ] Time spent saturated.
- [ ] Saturation recovery time.
- [ ] Minimum controllable motor speed.

## 4.6 Timing
- [ ] Nominal loop period.
- [ ] Maximum jitter.
- [ ] Missed deadline count.
- [ ] Watchdog response.

## 4.7 Fault response
- [ ] Detection time.
- [ ] Recovery time.
- [ ] Acceptable degraded-state error.
- [ ] Uncontrollable classification.

# PHASE 5 — Robustness campaign

Only after pass/fail criteria are frozen.

## 5.1 Parameter sweeps
- [ ] Mass.
- [ ] CG position.
- [ ] Inertia.
- [ ] Thrust coefficient.
- [ ] Torque coefficient.
- [ ] Motor response time.
- [ ] Motor maximum speed.
- [ ] Battery voltage.
- [ ] Aerodynamic drag.
- [ ] Controller gains.

## 5.2 Disturbances
- [ ] External force.
- [ ] External torque.
- [ ] Wind-like disturbance.
- [ ] Transient disturbance.
- [ ] Sustained disturbance.

## 5.3 Command envelope
- [ ] Small attitude commands.
- [ ] Aggressive commands.
- [ ] Simultaneous roll/pitch.
- [ ] Yaw during translation.
- [ ] Altitude + attitude.
- [ ] Maximum feasible commands.
- [ ] Infeasible commands.

## 5.4 Monte Carlo
- [ ] Randomise mass, CG, inertia, sensor noise/bias, actuator delay, motor mismatch and disturbance.
- [ ] Record `seed → parameters → outcome`.
- [ ] Make every failure reproducible.

## 5.5 Robustness envelope
- [ ] Quantify the operating region in which defined tracking, saturation and timing limits remain satisfied.

# PHASE 6 — Estimator qualification

## 6.1 Sensor models
- [ ] IMU noise.
- [ ] Accelerometer bias.
- [ ] Gyro bias/drift.
- [ ] Barometer noise/bias.
- [ ] Sample-rate mismatch.
- [ ] Timestamp jitter.

## 6.2 Estimator performance
- [ ] Attitude error.
- [ ] Rate error.
- [ ] Altitude error.
- [ ] Convergence time.
- [ ] Drift.
- [ ] Recovery after disturbance.

## 6.3 Estimated-state controller
- [ ] Compare true-state and estimated-state control under identical conditions.
- [ ] Quantify degradation.
- [ ] Define acceptable degradation.
- [ ] Identify unstable regions.

## 6.4 Sensor dropout
- [ ] Gyro dropout.
- [ ] Accelerometer dropout.
- [ ] Barometer dropout.
- [ ] Intermittent samples.
- [ ] Stale measurements.
- [ ] Invalid measurements.

## 6.5 Fault classification
Classify each fault as:
1. recoverable
2. degraded but controllable
3. uncontrollable
4. immediate failsafe required

# PHASE 7 — Actuator/fault qualification

Test one fault at a time first.

## Motor faults
- [ ] Command mismatch.
- [ ] Response delay.
- [ ] Saturation.
- [ ] Reduced thrust.
- [ ] Stuck motor.
- [ ] Complete motor loss.
- [ ] Intermittent motor loss.

For each:
- [ ] Detection.
- [ ] Controller reaction.
- [ ] State trajectory.
- [ ] Saturation.
- [ ] Recovery.
- [ ] Final condition.
- [ ] Classification.

## Battery faults
- [ ] Voltage sag.
- [ ] Reduced maximum thrust.
- [ ] Low-voltage condition.
- [ ] Sudden voltage drop.

## Combined faults
Only after individual faults are understood:
- [ ] Sensor + motor.
- [ ] Sensor + timing.
- [ ] Motor + battery.
- [ ] Estimator + actuator.
- [ ] Multiple motor degradation.

# PHASE 8 — Timing and real-time qualification

## 8.1 Timing contract
- [ ] Target controller frequency and period.
- [ ] Estimator frequency.
- [ ] IMU sampling frequency.
- [ ] Motor-output frequency.
- [ ] Telemetry frequency.
- [ ] Timestamp source.
- [ ] Scheduler assumptions.

## 8.2 Deadline behaviour
Define what happens when `execution time > loop period`:
- [ ] Skip update?
- [ ] Reuse command?
- [ ] Watchdog?
- [ ] Emergency state?
- [ ] Failsafe?
- [ ] Log event?

## 8.3 Simulation timing tests
- [ ] Nominal period.
- [ ] Jitter.
- [ ] Missed tick.
- [ ] Burst of missed ticks.
- [ ] Prolonged timing degradation.

## 8.4 H743 WCET
- [ ] Instrument controller entry/exit.
- [ ] Measure minimum, mean and worst execution time.
- [ ] Measure interrupt interference.
- [ ] Measure estimator, IMU, mixer/output and total loop time.
- [ ] Calculate CPU utilisation: `T_WCET / T_loop`.

**Gate D prerequisite:** Demonstrated real-time margin on actual H743 hardware.

# PHASE 9 — H743 hardware integration

## 9.1 Hardware abstraction
- [ ] IMU driver.
- [ ] Barometer driver.
- [ ] Timer.
- [ ] PWM/DShot.
- [ ] UART.
- [ ] DMA.
- [ ] ADC.
- [ ] Watchdog.
- [ ] Hardware timestamping.

## 9.2 Controller pipeline
```text
IMU
 ↓
Sensor processing
 ↓
Estimator
 ↓
VehicleState
 ↓
C Controller
 ↓
Wrench
 ↓
Mixer
 ↓
MotorCommand
 ↓
ESC
```

## 9.3 Preserve portability
Keep controller code separate from hardware drivers.

# PHASE 10 — Motor/ESC physical verification

**Props OFF.**

## 10.1 Motor numbering
- [ ] Physically identify motors 1–4.
- [ ] Verify against software numbering.

## 10.2 Rotation direction
- [ ] Verify each motor.
- [ ] Compare with mixer model.
- [ ] Record CW/CCW.
- [ ] Verify prop orientation separately.

## 10.3 Command mapping
- [ ] Verify output channel.
- [ ] Verify no cross-wiring.
- [ ] Verify minimum command.
- [ ] Verify maximum command.
- [ ] Verify failsafe command.

## 10.4 Physical mixer validation
Inject:
- [ ] Pure thrust.
- [ ] Pure roll.
- [ ] Pure pitch.
- [ ] Pure yaw.

Verify physical motor response matches the mathematical mixer.

# PHASE 11 — Props-off HIL / bench validation

## 11.1 Closed-loop hardware test
```text
Python plant
      ↓
simulated sensors
      ↓
H743 estimator
      ↓
H743 controller
      ↓
motor commands
      ↓
logged actuator response
      ↓
Python plant
```

## 11.2 Verify
- [ ] Timing.
- [ ] Sensor interfaces.
- [ ] Estimator.
- [ ] Controller.
- [ ] Mixer.
- [ ] Failsafe.
- [ ] Watchdog.
- [ ] Logging.
- [ ] Telemetry.

## 11.3 Triple comparison
```text
Python reference
        ↕
host C
        ↕
H743 C implementation
```
Any discrepancy must be explainable.

# PHASE 12 — First powered test

## 12.1 Controlled environment
- [ ] Physical restraint/test rig.
- [ ] Emergency power cutoff.
- [ ] Props initially removed.
- [ ] Appropriate remote failsafe.
- [ ] Logging active.
- [ ] Telemetry active.
- [ ] Battery monitoring active.

## 12.2 Incremental tests
- [ ] Boot.
- [ ] Sensor initialisation.
- [ ] Estimator convergence.
- [ ] Arm/disarm.
- [ ] Individual motor output.
- [ ] Mixer.
- [ ] Low-power motor test.
- [ ] Restrained thrust test.

## 12.3 Stop conditions
Define before testing:
- [ ] Excessive attitude estimate.
- [ ] Unexpected motor acceleration.
- [ ] Sensor failure.
- [ ] Timing violation.
- [ ] Watchdog fault.
- [ ] Communication loss.
- [ ] Battery abnormality.
- [ ] Command mismatch.

# PHASE 13 — Controlled flight qualification

## Flight 1 — Basic hover
- [ ] Low altitude.
- [ ] Manual/assisted control.
- [ ] Complete telemetry.

Measure attitude, rates, altitude, motor outputs, voltage, estimator error, timing and saturation.

## Flight 2 — Repeat hover
- [ ] Repeatability.
- [ ] Compare against Flight 1.

## Flight 3 — Small attitude commands
- [ ] Roll.
- [ ] Pitch.
- [ ] Yaw.

## Flight 4 — Combined manoeuvres
- [ ] Roll + pitch.
- [ ] Altitude + attitude.
- [ ] Yaw + translation.

## Flight 5 — Disturbance testing
- [ ] Only after previous flights are demonstrably stable.

# PHASE 14 — Dual-aircraft architecture

This comes after single-aircraft qualification.

## 14.1 A/B reproducibility
- [ ] Identical hardware configuration.
- [ ] Identical firmware.
- [ ] Identical controller parameters.
- [ ] Independent logs.

## 14.2 Inter-aircraft data link
Define:
```text
Aircraft ID
timestamp
position
velocity
attitude
body rates
altitude
battery
health
mission state
fault state
link state
```

## 14.3 Communication behaviour
- [ ] Nominal communication.
- [ ] Latency.
- [ ] Packet loss.
- [ ] Packet reordering.
- [ ] Stale data.
- [ ] Temporary link loss.
- [ ] Permanent link loss.

## 14.4 Critical rule
Loss of Aircraft B must never destabilise Aircraft A, and vice versa.

# PHASE 15 — Autonomy / CV

Only after flight-control qualification.

## 15.1 Companion computer
- [ ] Raspberry Pi integration.
- [ ] MAVLink.
- [ ] Telemetry.
- [ ] Logging.
- [ ] Mission interface.

## 15.2 Positioning
- [ ] Optical flow.
- [ ] LiDAR altitude.
- [ ] Sensor fusion.
- [ ] Position confidence.

## 15.3 High-level autonomy
- [ ] Waypoint generation.
- [ ] Trajectory commands.
- [ ] Mission state machine.
- [ ] Inter-aircraft coordination.

## 15.4 CV
- [ ] OAK-D or equivalent.
- [ ] Object detection.
- [ ] Visual tracking.
- [ ] Relative aircraft localisation.
- [ ] Cooperative perception.

---

# ACTUAL CRITICAL PATH

```text
CURRENT STATE
     │
     ▼
0. Engineering contract
     │
     ▼
1. H743 resource evidence
     │
     ▼
2. Python ↔ C replay
     │
     ▼
3. Defensible plant
     │
     ▼
4. Define pass/fail criteria
     │
     ▼
5. Robustness campaign
     │
     ▼
6. Estimator qualification
     │
     ▼
7. Fault qualification
     │
     ▼
8. Timing specification
     │
     ▼
9. H743 integration
     │
     ▼
10. Props-off validation
     │
     ▼
11. HIL / bench
     │
     ▼
12. Controlled flight
     │
     ▼
13. Aircraft A/B testing
     │
     ▼
14. Cooperative autonomy
     │
     ▼
15. CV / advanced autonomy
```

# IMMEDIATE BACKLOG

## 🔴 P0 — Do next

1. [ ] Freeze NED/FRD, units, motor numbering and reset behaviour.
2. [ ] Complete deterministic replay corpus.
3. [ ] Define quantitative pass/fail criteria.
4. [ ] Identify exact H743 board/PCB revision.
5. [ ] Finish hardware resource evidence chain.
6. [ ] Lock the resource manifest.
7. [ ] Measure/derive defensible mass and CG.
8. [ ] Establish geometry and inertia.
9. [ ] Establish defensible motor/prop coefficients.
10. [ ] Add battery voltage to the plant.

## 🟠 P1 — Qualification

11. [ ] Refine motor/ESC dynamic model.
12. [ ] Run single-fault campaign.
13. [ ] Classify recoverable/degraded/uncontrollable faults.
14. [ ] Complete estimator error analysis.
15. [ ] Complete sensor dropout testing.
16. [ ] Define robustness envelope.
17. [ ] Run deterministic parameter sweeps.
18. [ ] Run Monte Carlo campaign.
19. [ ] Quantify saturation behaviour.
20. [ ] Quantify timing behaviour.

## 🟡 P2 — Hardware

21. [ ] Define H743 timing contract.
22. [ ] Implement/verify hardware abstraction.
23. [ ] Measure H743 WCET.
24. [ ] Verify physical motor numbering.
25. [ ] Verify physical motor direction.
26. [ ] Verify ESC/output mapping.
27. [ ] Props-off controller test.
28. [ ] HIL.
29. [ ] Hardware-vs-Python replay comparison.

## 🟢 P3 — Flight

30. [ ] Bench validation.
31. [ ] Restrained thrust testing.
32. [ ] First controlled hover.
33. [ ] Repeat hover.
34. [ ] Attitude-step flight testing.
35. [ ] Combined manoeuvre testing.
36. [ ] Flight robustness envelope.

## 🔵 P4 — Dual aircraft

37. [ ] Aircraft A/B reproducibility.
38. [ ] MAVLink state exchange.
39. [ ] Inter-aircraft heartbeat/health.
40. [ ] Link-loss behaviour.
41. [ ] Cooperative waypoint exchange.
42. [ ] Coordinated trajectory execution.

## 🟣 P5 — Advanced autonomy

43. [ ] Raspberry Pi autonomy layer.
44. [ ] Optical-flow/LiDAR fusion.
45. [ ] Mission state machine.
46. [ ] Cooperative navigation.
47. [ ] OAK-D/CV integration.
48. [ ] Relative visual localisation.
49. [ ] Cooperative perception.

# Engineering objective

The project has moved from:

> **“Can I build a flight controller?”**

toward:

> **“Can I demonstrate, quantitatively and reproducibly, the conditions under which my flight controller works, degrades, and must declare a fault?”**

The next milestone is therefore **qualification of what already exists**, rather than adding more features.
