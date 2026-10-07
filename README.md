# Dual-Quadcopter Simulation

Linux simulation and flight-control development groundwork for a proposed dual-quadcopter project. The repository includes two 3D PyTorch flow-visualization experiments, a 2D NumPy backward-facing-step solver, and an illustrative single-axis pitch model.

## Intended aircraft architecture

The design brief describes two near-identical aircraft, each with a deterministic H743-class flight controller, IMU, four-motor/4-in-1 ESC powertrain, ELRS pilot link, and a Raspberry Pi companion for autonomy, telemetry, logging, and inter-aircraft coordination. This is a development target; the simulations below do not implement or validate the complete vehicle or cooperative flight stack.

## Setup

Use Python 3.10 or newer. Create an environment and install the packages:

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\\Scripts\\activate
```

For a CPU run, install requirements with `python -m pip install -r requirements.txt`. For CUDA, first install the PyTorch build that matches the installed NVIDIA driver and CUDA runtime using the [official PyTorch install selector](https://pytorch.org/get-started/locally/), then run `python -m pip install -r requirements.txt`. Both PyTorch simulations select CUDA when `torch.cuda.is_available()` is true and otherwise run on CPU.

## Run

Run the smoke-flow simulation (100 steps by default):

```bash
python torchFileTest.py
```

It writes `smoke.gif`, `smoke_3d_plane.gif`, `smoke_slice.png`, `smoke_slice.pt`, and `smoke_telemetry.json`. Add `--no-vscode` to skip opening the telemetry report.

Run the curved-surface Coanda visualization:

```bash
python coandaFlowTest.py
```

It writes side-view and curved-surface GIFs, a final slice, and `coanda_telemetry.json`. Add `--no-vscode` to skip opening the report.

Run the 2D backward-facing-step solver with JSON validation:

```bash
python readable_backward_facing_step_solver-3.py
```

The solver produces a render and `bfs_validation_report.json`. Add `--no-vscode` to skip opening the result.

The optional CUDA notebook is `cuda_smoke_test.ipynb`. It reports device availability and performs a small tensor operation; it is a runtime check, not part of the flow models.

## Telemetry monitor

A read-only Streamlit dashboard can show generated demo values or live MAVLink telemetry from a vehicle/SITL connection.

Install its optional dependencies and launch it:

```bash
python -m pip install -r requirements-telemetry.txt
streamlit run telemetry_dashboard.py
```

Choose **Demo** to preview the dashboard without a vehicle. For live telemetry, choose **MAVLink**, then connect to a UDP listener such as `udpin:0.0.0.0:14550`, or enter a serial device such as `/dev/ttyUSB0` and select the matching baud rate. Forward the autopilot's MAVLink stream to the selected UDP port when using UDP. The dashboard displays heartbeat/link state, flight mode and armed state, battery, relative altitude, speed, attitude, GPS, status text, and recent altitude/speed trends. It is telemetry-only and does not send flight commands.

## Reconstructed behavior

- `torchFileTest.py`: 3D semi-Lagrangian advection, pressure projection, vorticity confinement, and density/velocity visualizations.
- `coandaFlowTest.py`: 3D jet injection over a curved surface, near-wall attachment and alignment forces, pressure projection, tracer visualization, and animations.
- `readable_backward_facing_step_solver-3.py`: 2D explicit predictor and pressure-Poisson projection with a step obstacle, plotting, and JSON validation diagnostics.

These are educational visualization experiments. They are not a calibrated physical model or a benchmark-quality CFD solver, and they do not establish a validated flight-stability controller. The first-person development report in `docs/Linux_Quadrotor_Flight_Control_Development_Report.docx` summarizes the CUDA runs, constraints, and simulation-to-hardware roadmap.