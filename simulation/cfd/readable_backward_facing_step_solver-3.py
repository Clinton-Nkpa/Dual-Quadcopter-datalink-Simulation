##20:12 pm 17th May 2026
#todo: how to display results of execution report visually on a 2d grid? maybe a heatmap of divergence, or a quiver plot of velocity vectors, or a contour plot of pressure? could also overlay the original image with the rendered image to visually check for differences.
"""
Readable Backward-Facing Step CFD Solver
=======================================

A single-file, readable remodel of the notebook-style backward-facing-step CFD
example. The goal is clarity: explicit sections, named configuration values,
small functions, and a simple projection method for 2D incompressible flow.

What it solves
--------------
2D incompressible Navier-Stokes flow over a backward-facing step:

    du/dt + u du/dx + v du/dy = -1/rho dp/dx + nu Laplacian(u)
    dv/dt + u dv/dx + v dv/dy = -1/rho dp/dy + nu Laplacian(v)
    div(u, v) = 0

Numerical approach
------------------
- Collocated Cartesian grid
- Explicit advection-diffusion predictor step
- Pressure Poisson projection for incompressibility
- No-slip walls and step surface
- Uniform inlet velocity above the step
- Zero-gradient outlet

This is not a byte-for-byte conversion of the original notebook. It is a clean,
standalone, portfolio-friendly version designed to be easier to read, modify,
and explain.

Run:
    python readable_backward_facing_step_solver.py

Outputs:
    bfs_velocity_pressure.png
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


Array = np.ndarray

# -----------------------------------------------------------------------------
# 1. Configuration
# -----------------------------------------------------------------------------

@dataclass(frozen=True)
class SolverConfig:
    """Physical, numerical, and output settings for the simulation."""

    # Domain size
    length: float = 6.0
    height: float = 1.0

    # Backward-facing step geometry.
    # The solid block occupies x <= step_length and y <= step_height.
    step_length: float = 1.0
    step_height: float = 0.5

    # Mesh resolution
    nx: int = 181
    ny: int = 61

    # Fluid properties
    rho: float = 1.0
    nu: float = 0.01

    # Boundary condition / forcing
    inlet_velocity: float = 1.0

    # Time stepping
    dt: float = 0.001
    n_steps: int = 2500
    pressure_iterations: int = 80

    # Reporting / plotting
    report_every: int = 250
    output_image: str = "bfs_velocity_pressure.png"
    validation_report: str = "bfs_validation_report.json"

    # Validation thresholds. These are sanity checks for this educational solver,
    # not benchmark CFD tolerances.
    max_divergence_mean: float = 2.5
    max_divergence_l2: float = 180.0
    min_recirculation_fraction: float = 0.001
    min_image_std: float = 0.01
    min_image_width: int = 900
    min_image_height: int = 250


@dataclass
class SimulationResult:
    """Field data and grid metadata produced by the solver."""

    X: Array
    Y: Array
    u: Array
    v: Array
    p: Array
    solid: Array
    dx: float
    dy: float


# -----------------------------------------------------------------------------
# 2. Grid and geometry
# -----------------------------------------------------------------------------

def build_grid(cfg: SolverConfig) -> Tuple[Array, Array, float, float]:
    """Return meshgrid arrays and grid spacing."""
    x = np.linspace(0.0, cfg.length, cfg.nx)
    y = np.linspace(0.0, cfg.height, cfg.ny)
    dx = x[1] - x[0]
    dy = y[1] - y[0]
    X, Y = np.meshgrid(x, y)
    return X, Y, dx, dy


def build_solid_mask(cfg: SolverConfig, X: Array, Y: Array) -> Array:
    """Boolean mask for the solid step/blockage region."""
    return (X <= cfg.step_length) & (Y <= cfg.step_height)


# -----------------------------------------------------------------------------
# 3. Boundary conditions
# -----------------------------------------------------------------------------

def apply_velocity_boundary_conditions(
    u: Array,
    v: Array,
    cfg: SolverConfig,
    X: Array,
    Y: Array,
    solid: Array,
) -> None:
    """Apply inlet, outlet, wall, and solid-step velocity conditions in-place."""

    # Solid body: no slip inside the step.
    u[solid] = 0.0
    v[solid] = 0.0

    # No-slip top and bottom walls.
    u[0, :] = 0.0
    v[0, :] = 0.0
    u[-1, :] = 0.0
    v[-1, :] = 0.0

    # Inlet: uniform flow only above the step height.
    inlet_open = Y[:, 0] > cfg.step_height
    u[inlet_open, 0] = cfg.inlet_velocity
    v[inlet_open, 0] = 0.0

    # Inlet wall below step height is solid/no-slip.
    inlet_blocked = ~inlet_open
    u[inlet_blocked, 0] = 0.0
    v[inlet_blocked, 0] = 0.0

    # Outlet: zero streamwise gradient.
    u[:, -1] = u[:, -2]
    v[:, -1] = v[:, -2]

    # Re-apply solid after outlet/inlet rules.
    u[solid] = 0.0
    v[solid] = 0.0


def apply_pressure_boundary_conditions(p: Array, solid: Array) -> None:
    """Apply simple pressure boundary conditions in-place."""

    # Zero normal-gradient on walls/inlet.
    p[:, 0] = p[:, 1]
    p[0, :] = p[1, :]
    p[-1, :] = p[-2, :]

    # Reference outlet pressure.
    p[:, -1] = 0.0

    # Keep pressure inside solid from contaminating neighbouring values.
    p[solid] = 0.0


# -----------------------------------------------------------------------------
# 4. Numerical kernels
# -----------------------------------------------------------------------------

def central_gradient_x(phi: Array, dx: float) -> Array:
    grad = np.zeros_like(phi)
    grad[:, 1:-1] = (phi[:, 2:] - phi[:, :-2]) / (2.0 * dx)
    return grad


def central_gradient_y(phi: Array, dy: float) -> Array:
    grad = np.zeros_like(phi)
    grad[1:-1, :] = (phi[2:, :] - phi[:-2, :]) / (2.0 * dy)
    return grad


def laplacian(phi: Array, dx: float, dy: float) -> Array:
    lap = np.zeros_like(phi)
    lap[1:-1, 1:-1] = (
        (phi[1:-1, 2:] - 2.0 * phi[1:-1, 1:-1] + phi[1:-1, :-2]) / dx**2
        + (phi[2:, 1:-1] - 2.0 * phi[1:-1, 1:-1] + phi[:-2, 1:-1]) / dy**2
    )
    return lap


def divergence(u: Array, v: Array, dx: float, dy: float) -> Array:
    return central_gradient_x(u, dx) + central_gradient_y(v, dy)


def predictor_step(
    u: Array,
    v: Array,
    cfg: SolverConfig,
    dx: float,
    dy: float,
    fluid: Array,
) -> Tuple[Array, Array]:
    """Explicit advection-diffusion predictor without pressure."""

    du_dx = central_gradient_x(u, dx)
    du_dy = central_gradient_y(u, dy)
    dv_dx = central_gradient_x(v, dx)
    dv_dy = central_gradient_y(v, dy)

    adv_u = u * du_dx + v * du_dy
    adv_v = u * dv_dx + v * dv_dy

    diff_u = cfg.nu * laplacian(u, dx, dy)
    diff_v = cfg.nu * laplacian(v, dx, dy)

    u_star = u.copy()
    v_star = v.copy()
    u_star[fluid] = u[fluid] + cfg.dt * (-adv_u[fluid] + diff_u[fluid])
    v_star[fluid] = v[fluid] + cfg.dt * (-adv_v[fluid] + diff_v[fluid])

    return u_star, v_star


def solve_pressure_poisson(
    p: Array,
    rhs: Array,
    cfg: SolverConfig,
    dx: float,
    dy: float,
    fluid: Array,
    solid: Array,
) -> Array:
    """Jacobi pressure Poisson solve for the projection step."""

    pn = p.copy()
    dx2 = dx * dx
    dy2 = dy * dy
    denominator = 2.0 * (dx2 + dy2)

    for _ in range(cfg.pressure_iterations):
        pn[:] = p
        p[1:-1, 1:-1] = (
            (pn[1:-1, 2:] + pn[1:-1, :-2]) * dy2
            + (pn[2:, 1:-1] + pn[:-2, 1:-1]) * dx2
            - rhs[1:-1, 1:-1] * dx2 * dy2
        ) / denominator

        p[~fluid] = 0.0
        apply_pressure_boundary_conditions(p, solid)

    return p


def projection_step(
    u_star: Array,
    v_star: Array,
    p: Array,
    cfg: SolverConfig,
    dx: float,
    dy: float,
    fluid: Array,
    solid: Array,
) -> Tuple[Array, Array, Array]:
    """Project predicted velocity onto a divergence-free field."""

    rhs = cfg.rho / cfg.dt * divergence(u_star, v_star, dx, dy)
    rhs[~fluid] = 0.0

    p = solve_pressure_poisson(p, rhs, cfg, dx, dy, fluid, solid)

    dp_dx = central_gradient_x(p, dx)
    dp_dy = central_gradient_y(p, dy)

    u = u_star.copy()
    v = v_star.copy()
    u[fluid] = u_star[fluid] - cfg.dt / cfg.rho * dp_dx[fluid]
    v[fluid] = v_star[fluid] - cfg.dt / cfg.rho * dp_dy[fluid]

    return u, v, p


# -----------------------------------------------------------------------------
# 5. Simulation driver
# -----------------------------------------------------------------------------

def run_simulation(cfg: SolverConfig) -> SimulationResult:
    """Run the full backward-facing-step simulation."""

    X, Y, dx, dy = build_grid(cfg)
    solid = build_solid_mask(cfg, X, Y)
    fluid = ~solid

    u = np.zeros((cfg.ny, cfg.nx), dtype=float)
    v = np.zeros_like(u)
    p = np.zeros_like(u)

    apply_velocity_boundary_conditions(u, v, cfg, X, Y, solid)
    apply_pressure_boundary_conditions(p, solid)

    previous_divergence = np.inf

    for step in range(1, cfg.n_steps + 1):
        u_star, v_star = predictor_step(u, v, cfg, dx, dy, fluid)
        apply_velocity_boundary_conditions(u_star, v_star, cfg, X, Y, solid)

        u, v, p = projection_step(u_star, v_star, p, cfg, dx, dy, fluid, solid)
        apply_velocity_boundary_conditions(u, v, cfg, X, Y, solid)

        if step % cfg.report_every == 0 or step == 1:
            div_norm = np.linalg.norm(divergence(u, v, dx, dy)[fluid])
            change = abs(previous_divergence - div_norm)
            previous_divergence = div_norm
            print(f"step={step:5d} | divergence_norm={div_norm:.6e} | change={change:.3e}")

    return SimulationResult(X=X, Y=Y, u=u, v=v, p=p, solid=solid, dx=dx, dy=dy)


# -----------------------------------------------------------------------------
# 6. Post-processing
# -----------------------------------------------------------------------------

def plot_results(
    X: Array,
    Y: Array,
    u: Array,
    v: Array,
    p: Array,
    solid: Array,
    cfg: SolverConfig,
) -> Path:
    """Create a compact pressure + velocity plot."""

    speed = np.sqrt(u**2 + v**2)
    p_plot = np.ma.masked_where(solid, p)
    speed_plot = np.ma.masked_where(solid, speed)

    fig, ax = plt.subplots(figsize=(12, 4.5))
    pressure = ax.contourf(X, Y, p_plot, levels=40, alpha=0.85)
    fig.colorbar(pressure, ax=ax, label="pressure")

    skip_y = max(1, cfg.ny // 30)
    skip_x = max(1, cfg.nx // 50)
    ax.quiver(
        X[::skip_y, ::skip_x],
        Y[::skip_y, ::skip_x],
        u[::skip_y, ::skip_x],
        v[::skip_y, ::skip_x],
        speed_plot[::skip_y, ::skip_x],
        scale=35,
        width=0.0025,
    )

    step_patch = plt.Rectangle(
        (0.0, 0.0),
        cfg.step_length,
        cfg.step_height,
        linewidth=1.5,
        edgecolor="black",
        facecolor="black",
        alpha=0.35,
        label="solid step",
    )
    ax.add_patch(step_patch)

    ax.set_title("Backward-facing step: pressure field and velocity vectors")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0.0, cfg.length)
    ax.set_ylim(0.0, cfg.height)
    ax.legend(loc="upper right")
    fig.tight_layout()

    output_path = Path(cfg.output_image)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    return output_path


# -----------------------------------------------------------------------------
# 7. Validation
# -----------------------------------------------------------------------------

def compute_numerical_diagnostics(result: SimulationResult, cfg: SolverConfig) -> Dict[str, float]:
    """Return scalar diagnostics used to judge solver/render plausibility."""

    X, Y = result.X, result.Y
    u, v, p = result.u, result.v, result.p
    solid = result.solid
    fluid = ~solid

    speed = np.sqrt(u**2 + v**2)
    div = divergence(u, v, result.dx, result.dy)

    inlet_open = Y[:, 0] > cfg.step_height
    near_wall_downstream = (
        (X > cfg.step_length)
        & (X < cfg.step_length + 2.5 * cfg.step_height)
        & (Y < cfg.step_height + 0.25)
        & fluid
    )
    if np.any(near_wall_downstream):
        recirculation_fraction = float(np.mean(u[near_wall_downstream] < 0.0))
    else:
        recirculation_fraction = 0.0

    max_speed = float(np.max(speed[fluid]))
    return {
        "finite_u": bool(np.isfinite(u).all()),
        "finite_v": bool(np.isfinite(v).all()),
        "finite_p": bool(np.isfinite(p).all()),
        "dx": float(result.dx),
        "dy": float(result.dy),
        "reynolds_inlet_gap": float(cfg.inlet_velocity * (cfg.height - cfg.step_height) / cfg.nu),
        "cfl": float(max_speed * cfg.dt / min(result.dx, result.dy)),
        "diffusion_number": float(cfg.nu * cfg.dt * (1.0 / result.dx**2 + 1.0 / result.dy**2)),
        "max_speed": max_speed,
        "mean_speed": float(np.mean(speed[fluid])),
        "pressure_min": float(np.min(p[fluid])),
        "pressure_max": float(np.max(p[fluid])),
        "pressure_range": float(np.ptp(p[fluid])),
        "divergence_l2": float(np.linalg.norm(div[fluid])),
        "divergence_mean_abs": float(np.mean(np.abs(div[fluid]))),
        "divergence_max_abs": float(np.max(np.abs(div[fluid]))),
        "solid_max_speed": float(np.max(speed[solid])) if np.any(solid) else 0.0,
        "inlet_open_mean_u": float(np.mean(u[inlet_open, 0])) if np.any(inlet_open) else 0.0,
        "inlet_open_max_abs_v": float(np.max(np.abs(v[inlet_open, 0]))) if np.any(inlet_open) else 0.0,
        "recirculation_fraction": recirculation_fraction,
    }


def compute_render_diagnostics(output_path: Path) -> Dict[str, float]:
    """Inspect the generated PNG for basic rendered-output correctness."""

    image = plt.imread(output_path)
    if image.ndim == 2:
        rgb = np.repeat(image[:, :, None], 3, axis=2)
    else:
        rgb = image[:, :, :3]

    if np.issubdtype(rgb.dtype, np.integer):
        rgb = rgb.astype(float) / 255.0

    luminance = rgb.mean(axis=2)
    dark_pixels = np.all(rgb < 0.12, axis=2)
    bright_pixels = np.all(rgb > 0.92, axis=2)

    return {
        "image_exists": output_path.exists(),
        "image_width": int(rgb.shape[1]),
        "image_height": int(rgb.shape[0]),
        "image_channels": int(rgb.shape[2]),
        "image_min": float(np.min(rgb)),
        "image_max": float(np.max(rgb)),
        "image_mean": float(np.mean(rgb)),
        "image_std": float(np.std(rgb)),
        "luminance_std": float(np.std(luminance)),
        "dark_pixel_fraction": float(np.mean(dark_pixels)),
        "bright_pixel_fraction": float(np.mean(bright_pixels)),
        "unique_color_estimate": int(np.unique((rgb.reshape(-1, 3) * 255).astype(np.uint8), axis=0).shape[0]),
    }


def validate_output(
    numerical: Dict[str, float],
    render: Dict[str, float],
    cfg: SolverConfig,
) -> Tuple[bool, List[Dict[str, object]]]:
    """Return pass/fail plus individual validation checks."""

    checks = [
        {
            "name": "finite fields",
            "passed": bool(numerical["finite_u"] and numerical["finite_v"] and numerical["finite_p"]),
            "value": None,
            "limit": "all field values must be finite",
        },
        {
            "name": "solid no-slip velocity",
            "passed": numerical["solid_max_speed"] < 1e-12,
            "value": numerical["solid_max_speed"],
            "limit": "< 1e-12",
        },
        {
            "name": "inlet profile retained",
            "passed": abs(numerical["inlet_open_mean_u"] - cfg.inlet_velocity) < 1e-12,
            "value": numerical["inlet_open_mean_u"],
            "limit": f"within 1e-12 of {cfg.inlet_velocity}",
        },
        {
            "name": "inlet vertical velocity",
            "passed": numerical["inlet_open_max_abs_v"] < 1e-12,
            "value": numerical["inlet_open_max_abs_v"],
            "limit": "< 1e-12",
        },
        {
            "name": "divergence mean",
            "passed": numerical["divergence_mean_abs"] < cfg.max_divergence_mean,
            "value": numerical["divergence_mean_abs"],
            "limit": f"< {cfg.max_divergence_mean}",
        },
        {
            "name": "divergence L2",
            "passed": numerical["divergence_l2"] < cfg.max_divergence_l2,
            "value": numerical["divergence_l2"],
            "limit": f"< {cfg.max_divergence_l2}",
        },
        {
            "name": "recirculation visible",
            "passed": numerical["recirculation_fraction"] >= cfg.min_recirculation_fraction,
            "value": numerical["recirculation_fraction"],
            "limit": f">= {cfg.min_recirculation_fraction}",
        },
        {
            "name": "rendered image exists",
            "passed": bool(render["image_exists"]),
            "value": render["image_exists"],
            "limit": "True",
        },
        {
            "name": "rendered image width",
            "passed": render["image_width"] >= cfg.min_image_width,
            "value": render["image_width"],
            "limit": f">= {cfg.min_image_width}",
        },
        {
            "name": "rendered image height",
            "passed": render["image_height"] >= cfg.min_image_height,
            "value": render["image_height"],
            "limit": f">= {cfg.min_image_height}",
        },
        {
            "name": "render is not blank",
            "passed": render["image_std"] >= cfg.min_image_std,
            "value": render["image_std"],
            "limit": f">= {cfg.min_image_std}",
        },
        {
            "name": "solid/axes dark pixels present",
            "passed": render["dark_pixel_fraction"] > 0.001,
            "value": render["dark_pixel_fraction"],
            "limit": "> 0.001",
        },
        {
            "name": "pressure color variation present",
            "passed": render["unique_color_estimate"] > 128,
            "value": render["unique_color_estimate"],
            "limit": "> 128",
        },
    ]

    return all(bool(check["passed"]) for check in checks), checks


def save_validation_report(
    output_path: Path,
    numerical: Dict[str, float],
    render: Dict[str, float],
    checks: List[Dict[str, object]],
    passed: bool,
) -> Path:
    """Write a machine-readable validation report beside the render."""

    report = {
        "passed": passed,
        "numerical": numerical,
        "render": render,
        "checks": checks,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return output_path


# -----------------------------------------------------------------------------
# 8. Entrypoint
# -----------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Backward-facing-step solver with render validation.")
    parser.add_argument("--steps", type=int, default=None, help="Override the number of time steps.")
    parser.add_argument("--pressure-iterations", type=int, default=None, help="Override Jacobi pressure iterations.")
    parser.add_argument("--output", type=str, default=None, help="Output PNG path.")
    parser.add_argument("--report", type=str, default=None, help="Validation JSON path.")
    parser.add_argument("--skip-validation", action="store_true", help="Only run and render; do not validate output.")
    parser.add_argument("--no-vscode", action="store_true", help="Do not open the output in VS Code.")
    return parser.parse_args()


def open_in_vscode(path: Path) -> bool:
    code_command = shutil.which("code") or shutil.which("code.cmd")
    if code_command is None:
        print(f"VS Code command not found. Open this file in VS Code instead: {path}")
        return False
    try:
        subprocess.Popen(
            [code_command, "-r", str(path)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError as error:
        print(f"Could not open VS Code automatically: {error}")
        print(f"Open this file in VS Code instead: {path}")
        return False
    print(f"Opened in VS Code: {path}")
    return True


def config_from_args(args: argparse.Namespace) -> SolverConfig:
    cfg = SolverConfig()
    updates = {}
    if args.steps is not None:
        updates["n_steps"] = args.steps
    if args.pressure_iterations is not None:
        updates["pressure_iterations"] = args.pressure_iterations
    if args.output is not None:
        updates["output_image"] = args.output
    if args.report is not None:
        updates["validation_report"] = args.report
    return replace(cfg, **updates)


def main() -> None:
    args = parse_args()
    cfg = config_from_args(args)
    result = run_simulation(cfg)
    output_path = plot_results(result.X, result.Y, result.u, result.v, result.p, result.solid, cfg)
    print(f"Saved plot to: {output_path.resolve()}")

    if args.skip_validation:
        if not args.no_vscode:
            open_in_vscode(output_path)
        return

    numerical = compute_numerical_diagnostics(result, cfg)
    render = compute_render_diagnostics(output_path)
    passed, checks = validate_output(numerical, render, cfg)
    report_path = save_validation_report(Path(cfg.validation_report), numerical, render, checks, passed)

    status = "PASSED" if passed else "FAILED"
    print(f"Validation {status}")
    for check in checks:
        marker = "PASS" if check["passed"] else "FAIL"
        print(f"  [{marker}] {check['name']}: value={check['value']} limit={check['limit']}")
    print(f"Saved validation report to: {report_path.resolve()}")
    if not args.no_vscode:
        open_in_vscode(report_path)


if __name__ == "__main__":
    main()
