import argparse
import shutil
import subprocess
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from analysis.rotor_pitch_axis_state_space import PitchModelConfig, rk4_step, state_matrices


OUT_DIR = Path(__file__).resolve().parent
OUTPUT_PATH = OUT_DIR / "helicopter_pitch_phase_portrait_aggressive_damping.png"

# Overdamped, non-oscillatory settling — aggressive damping response.
AGGRESSIVE_ZETA = 1.0


def draw_matrix_table(ax, title, matrix, subtitle=None, fontsize=14):
    ax.axis("off")
    ax.set_title(title, fontsize=fontsize, pad=12, weight="bold")

    table = ax.table(
        cellText=matrix,
        cellLoc="center",
        loc="center",
        bbox=[0.14, 0.25, 0.72, 0.5],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(15)

    for cell in table.get_celld().values():
        cell.set_edgecolor("#334155")
        cell.set_linewidth(1.1)
        cell.set_facecolor("#f8fafc")

    if subtitle:
        ax.text(
            0.5,
            0.08,
            subtitle,
            ha="center",
            va="center",
            fontsize=10,
            color="#475569",
            transform=ax.transAxes,
        )


def simulate_free_response(a, cfg, initial_state):
    time = np.arange(0.0, cfg.time_end + cfg.dt, cfg.dt)
    states = np.zeros((len(time), 2))
    states[0] = initial_state

    b = np.zeros((2, 1))
    for index in range(1, len(time)):
        states[index] = rk4_step(a, b, states[index - 1], 0.0, cfg.dt)
    return time, states


def phase_field(theta_deg, q_deg_s, omega, zeta):
    theta_rad = np.deg2rad(theta_deg)
    q_rad_s = np.deg2rad(q_deg_s)

    dtheta_dt = q_deg_s
    dq_dt = np.rad2deg(-(omega**2) * theta_rad - 2.0 * zeta * omega * q_rad_s)
    speed = np.sqrt(dtheta_dt**2 + dq_dt**2)
    return dtheta_dt, dq_dt, speed


def open_in_vscode(path: Path) -> bool:
    code_command = shutil.which("code") or shutil.which("code.cmd")
    if code_command is None:
        print(f"VS Code command not found. Open this file instead: {path}")
        print(f'Or run: code -r "{path}"')
        return False

    try:
        subprocess.Popen(
            [code_command, "-r", str(path)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError as error:
        print(f"Could not open VS Code automatically: {error}")
        print(f"Open this file instead: {path}")
        return False

    print(f"Opened render in VS Code: {path}")
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render helicopter pitch phase portrait with Jacobian and Hessian panels."
    )
    parser.add_argument(
        "--no-vscode",
        action="store_true",
        help="Do not open the render in VS Code after saving.",
    )
    return parser.parse_args()


def main(open_vscode: bool = True):
    cfg = PitchModelConfig()
    omega = cfg.natural_frequency
    a, _, _, _ = state_matrices(cfg, AGGRESSIVE_ZETA)
    eigenvalues = np.linalg.eigvals(a)

    initial_state = np.array(
        [
            np.deg2rad(cfg.initial_pitch_deg),
            np.deg2rad(cfg.initial_pitch_rate_deg_s),
        ]
    )
    time, states = simulate_free_response(a, cfg, initial_state)
    theta_traj_deg = np.rad2deg(states[:, 0])
    q_traj_deg_s = np.rad2deg(states[:, 1])

    theta = np.linspace(-12.0, 12.0, 19)
    q = np.linspace(-30.0, 30.0, 19)
    theta_grid, q_grid = np.meshgrid(theta, q)
    u, v, speed = phase_field(theta_grid, q_grid, omega, AGGRESSIVE_ZETA)

    fig = plt.figure(figsize=(15, 8.5), dpi=170)
    grid = fig.add_gridspec(3, 2, width_ratios=[1.45, 1.0], hspace=0.45, wspace=0.22)

    ax_field = fig.add_subplot(grid[:, 0])
    ax_field.axhline(0.0, color="#64748b", linewidth=1.1)
    ax_field.axvline(0.0, color="#64748b", linewidth=1.1)
    ax_field.grid(True, linestyle="--", linewidth=0.6, alpha=0.45)

    quiver = ax_field.quiver(
        theta_grid,
        q_grid,
        u,
        v,
        speed,
        cmap="viridis",
        angles="xy",
        scale_units="xy",
        scale=180,
        width=0.0045,
    )
    traj = ax_field.plot(
        theta_traj_deg,
        q_traj_deg_s,
        color="#dc2626",
        linewidth=2.4,
        zorder=4,
        label="transient response",
    )[0]
    ax_field.scatter(
        theta_traj_deg[0],
        q_traj_deg_s[0],
        s=70,
        color="#2563eb",
        edgecolor="white",
        linewidth=0.9,
        zorder=5,
        label="initial state",
    )
    ax_field.scatter(0.0, 0.0, s=55, color="#059669", edgecolor="white", linewidth=0.9, zorder=5, label="trim point")
    ax_field.legend(loc="upper right", fontsize=9)

    fig.colorbar(quiver, ax=ax_field, fraction=0.046, pad=0.04, label=r"$|\dot{x}|$ in state plane")
    ax_field.set_title(
        rf"Pitch Phase Portrait: $\dot{{x}} = Ax$, $\zeta={AGGRESSIVE_ZETA:g}$ (aggressive damping)",
        fontsize=15,
        weight="bold",
    )
    ax_field.set_xlabel(r"pitch angle $\theta$ (deg)")
    ax_field.set_ylabel(r"pitch rate $q$ (deg/s)")
    ax_field.set_xlim(-13.0, 13.0)
    ax_field.set_ylim(-32.0, 32.0)
    ax_field.set_aspect("equal", adjustable="box")

    ax_jacobian = fig.add_subplot(grid[0, 1])
    draw_matrix_table(
        ax_jacobian,
        r"Jacobian $J_F(\theta, q) = A$",
        [
            [r"$0$", r"$1$"],
            [rf"$-{omega**2:.2f}$", rf"$-{2 * AGGRESSIVE_ZETA * omega:.2f}$"],
        ],
        subtitle=r"$\dot{x} = A x$,  $\delta_c = 0$",
    )

    ax_hessian_theta = fig.add_subplot(grid[1, 1])
    draw_matrix_table(
        ax_hessian_theta,
        r"Hessian of $\dot{\theta} = q$",
        [[r"$0$", r"$0$"], [r"$0$", r"$0$"]],
        subtitle=r"linear state equation $\Rightarrow$ zero curvature",
    )

    ax_eigs = fig.add_subplot(grid[2, 1])
    ax_eigs.axis("off")
    ax_eigs.set_title("Aggressive damping transient", fontsize=14, pad=12, weight="bold")
    eig_lines = [
        rf"$\lambda_{index + 1}$ = {value.real:+.3f}{value.imag:+.3f}j"
        for index, value in enumerate(eigenvalues)
    ]
    eig_text = "\n".join(eig_lines)
    summary = (
        rf"$\omega_n = {omega:g}$ rad/s, $\zeta = {AGGRESSIVE_ZETA:g}$"
        "\n"
        rf"$\theta_0 = {cfg.initial_pitch_deg:g}$ deg, $q_0 = {cfg.initial_pitch_rate_deg_s:g}$ deg/s"
        "\n"
        r"$\dot{\theta} = q$"
        "\n"
        rf"$\dot{{q}} = -\omega_n^2 \theta - 2\zeta\omega_n q$"
        "\n\n"
        f"{eig_text}"
        "\n\n"
        "real negative eigenvalues: fast non-oscillatory decay"
    )
    ax_eigs.text(
        0.5,
        0.5,
        summary,
        ha="center",
        va="center",
        fontsize=11,
        color="#0f172a",
        transform=ax_eigs.transAxes,
        bbox=dict(boxstyle="round,pad=0.45", facecolor="#f8fafc", edgecolor="#cbd5e1"),
    )

    inset = ax_field.inset_axes([0.58, 0.08, 0.38, 0.28])
    inset.plot(time, theta_traj_deg, color="#dc2626", linewidth=1.8, label=r"$\theta$")
    inset.plot(time, q_traj_deg_s, color="#7c3aed", linewidth=1.8, label=r"$q$")
    inset.axhline(0.0, color="#64748b", linewidth=0.8)
    inset.set_xlabel("time (s)", fontsize=8)
    inset.set_ylabel("deg / deg/s", fontsize=8)
    inset.tick_params(labelsize=7)
    inset.grid(True, linestyle="--", linewidth=0.5, alpha=0.4)
    inset.set_title("transient decay", fontsize=8.5)
    inset.legend(fontsize=7, loc="upper right")

    fig.suptitle(
        "Helicopter Pitch State-Space Phase Portrait with Aggressive Damping Response",
        fontsize=18,
        weight="bold",
        y=0.98,
    )
    fig.savefig(OUTPUT_PATH, bbox_inches="tight")
    plt.close(fig)

    settle_index = np.where(np.abs(theta_traj_deg) < 0.5)[0]
    settle_time = float(time[settle_index[0]]) if len(settle_index) else float(time[-1])
    print(f"Saved plot to: {OUTPUT_PATH}")
    print(
        f"Aggressive damping transient: theta_0={cfg.initial_pitch_deg:g} deg, "
        f"zeta={AGGRESSIVE_ZETA:g}, settle below 0.5 deg at t={settle_time:.2f} s"
    )

    if open_vscode:
        open_in_vscode(OUTPUT_PATH)


if __name__ == "__main__":
    args = parse_args()
    main(open_vscode=not args.no_vscode)
