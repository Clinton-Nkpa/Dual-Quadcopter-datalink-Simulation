# Dual-Quadcopter Simulation

Linux-based simulation and analysis groundwork for a future dual-quadcopter project. The public repository contains educational flow simulations, a reusable 6-DoF plant and actuator/sensor models, analysis examples, telemetry monitoring, interface definitions, and development documentation.

The public repository deliberately excludes the flight-controller implementation, estimator implementation, H743-specific configuration, hardware resource manifest, and proprietary tuning. Those remain in the local `private/` directory and are ignored by Git. See [FILE_CLASSIFICATION.md](documentation/FILE_CLASSIFICATION.md) for the exact boundary.

## Repository layout

```text
simulation/
├── quadrotor_6dof.py       # NED/FRD rigid-body plant, motor lag, mixer and rotor wrench
├── actuator_faults.py      # Simulation-only actuator perturbations
├── sensor_models.py        # Seeded IMU/barometer models
├── simulation_timing.py    # Scheduler jitter and missed-update model
├── cfd/                    # Smoke, Coanda, backward-facing-step demos and CUDA notebook
└── tests/                  # Plant, mapping and sensor-model checks
analysis/                   # Pitch phase portrait and state-space example
telemetry/                  # Read-only Streamlit/MAVLink dashboard
interfaces/                 # Controller-facing state, setpoint and command API
documentation/              # Development report, TODO and file classification
README.md
```

## Setup

Use Python 3.10 or newer. Create an environment and install the base packages:

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\\Scripts\\activate
python -m pip install -r requirements.txt
```

The flow demos use PyTorch. For CUDA, install a PyTorch build compatible with the NVIDIA driver using the [official PyTorch install selector](https://pytorch.org/get-started/locally/), then install the remaining requirements.

For the optional telemetry dashboard:

```bash
python -m pip install -r telemetry/requirements-telemetry.txt
```

## Public simulation

The 6-DoF plant uses world NED coordinates and body FRD coordinates. Euler angles are radians internally using a ZYX rotation; body rates are rad/s. Positive altitude is `-z_NED`, and level collective thrust acts along body `-Z`. The software motor order is front-left, front-right, rear-right, rear-left. The model's mass, geometry, inertia, motor limits, drag, thrust and reaction-torque coefficients are illustrative and are not measured aircraft properties.

The plant models rigid-body translation and rotation, first-order rotor-speed response, a motor mixer, quadratic thrust/reaction torque, gravity, and body-frame linear drag. Sensor and actuator perturbation modules support reproducible software experiments. Physical wiring, motor spin direction, sensor behaviour, ESC response and flight stability require separate hardware evidence.

Run the public model checks from the repository root:

```bash
python simulation/tests/verify_plant.py
python simulation/tests/verify_model_mapping.py
python simulation/tests/verify_sensor_statistics.py
```

The full closed-loop controller runner, estimator, and its C equivalence tool are retained locally under `private/` and are not part of the public repository. The public 6-DoF package is the plant and supporting model layer; it does not claim to provide deployable flight-control firmware.

## Flow simulations

Run the 3D smoke-flow demo:

```bash
python simulation/cfd/torchFileTest.py --out-dir work/smoke
```

Run the curved-surface Coanda demo:

```bash
python simulation/cfd/coandaFlowTest.py --out-dir work/coanda
```

Run the 2D backward-facing-step solver with JSON diagnostics:

```bash
python simulation/cfd/readable_backward_facing_step_solver-3.py \
  --output work/bfs.png --report work/bfs_validation.json
```

Each script has reduced-run options for quick checks; pass `--no-vscode` to prevent it from opening generated output in VS Code. The CUDA notebook is `simulation/cfd/cuda_smoke_test.ipynb`. These are educational visualization experiments, not benchmark-quality CFD solvers.

## Analysis

The standalone pitch-state-space example is `analysis/rotor_pitch_axis_state_space.py`. Run the phase portrait with:

```bash
python -m analysis.helicopter_pitch_phase_portrait_aggressive_damping
```

These examples are illustrative analyses, not identified aircraft models or formal stability qualification.

## Telemetry dashboard

Launch the read-only Streamlit dashboard:

```bash
streamlit run telemetry/telemetry_dashboard.py
```

Choose **Demo** to preview synthetic values. For live MAVLink telemetry, choose **MAVLink** and connect through a UDP listener such as `udpin:0.0.0.0:14550`, or select a serial device and baud rate. The dashboard displays vehicle/link state and telemetry; it does not send flight commands.

## Documentation and status

- [Development report](documentation/reports/Linux_Quadrotor_Flight_Control_Development_Report.docx)
- [Remaining qualification TODO](documentation/Dual_Quadcopter_Flight_Control_Remaining_TODO.md)
- [Public/private file classification](documentation/FILE_CLASSIFICATION.md)

The nonlinear model and fault hooks have software-level execution checks, but quantitative robustness limits have not yet been established. The project still needs evidence-based plant parameters, a defined robustness envelope, estimator qualification, and later hardware-specific timing, peripheral, props-off, HIL, and flight validation. H743 hardware operation and flight performance are not demonstrated by this public code.
