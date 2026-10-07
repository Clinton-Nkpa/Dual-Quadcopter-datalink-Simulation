# Public/private file classification

The GitHub repository is public. Files under the public directories below are intended for publication. The local `private/` directory is ignored by Git and must not be staged or pushed.

## Public

| Path | Contents |
| --- | --- |
| `simulation/` | 6-DoF plant, actuator and sensor simulation models, timing model, educational CFD demos, and public model checks. |
| `analysis/` | Pitch state-space and phase-portrait examples. |
| `telemetry/` | Read-only MAVLink/Streamlit telemetry dashboard and its optional requirements. |
| `documentation/` | Architecture/development report, qualification TODO, and this classification record. |
| `interfaces/` | Public controller-facing state, setpoint, and motor-command data contract; no controller equations. |
| `README.md`, `requirements.txt` | Public project overview and base dependencies. |

## Local/private; excluded from GitHub

| Local path | Contents |
| --- | --- |
| `private/controller/` | Python cascaded controller and portable C controller source/header. |
| `private/estimator/` | State-estimator implementation. |
| `private/integration/` | Closed-loop runner and visualization that depend on the private controller/estimator. |
| `private/hardware/` | H743 resource manifest and its validation report. |
| `private/tools/` | Controller equivalence and hardware-manifest validation tools. |
| `private/verification/` | Controller replay vectors. |
| `private/` tuning/configuration | Proprietary controller tuning and future board-specific code belong here. |

The root `.gitignore` excludes `private/`, `firmware/`, `hardware/`, and local verification/output directories. The public `interfaces/` folder is only an API/data contract; it is not a copy of the private controller implementation. Physical H743 pin assignments remain unresolved and are not published as wiring guidance.
