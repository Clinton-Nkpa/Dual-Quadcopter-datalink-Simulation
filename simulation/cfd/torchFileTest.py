from pathlib import Path
import argparse
import shutil
import subprocess
import time

import imageio.v2 as imageio
import matplotlib
import numpy as np
import torch
import torch.nn.functional as F


matplotlib.use("Agg")
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.pyplot as plt

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

NX, NY, NZ = 48, 48, 48
DX = 1.0
DT = 0.05
STEPS = 100
VISC = 0.001
PRESSURE_ITERS = 40
DAMPING = 0.995
VORT_EPS = 0.6
SOURCE_FORCE = 3.0
SOURCE_DENSITY = 0.8
PLANE_FRAME_INTERVAL = 4
PLANE_ARROW_STRIDE = 6
DIAGNOSTIC_INTERVAL = 10

OUT_DIR = Path(__file__).resolve().parent


def make_laplacian_kernel(device):
    kernel = torch.zeros((1, 1, 3, 3, 3), device=device)
    kernel[0, 0, 1, 1, 1] = -6
    kernel[0, 0, 0, 1, 1] = 1
    kernel[0, 0, 2, 1, 1] = 1
    kernel[0, 0, 1, 0, 1] = 1
    kernel[0, 0, 1, 2, 1] = 1
    kernel[0, 0, 1, 1, 0] = 1
    kernel[0, 0, 1, 1, 2] = 1
    return kernel


LAPLACIAN_KERNEL = make_laplacian_kernel(DEVICE)
NEIGHBOR_KERNEL = LAPLACIAN_KERNEL.clone()
NEIGHBOR_KERNEL[0, 0, 1, 1, 1] = 0


def source_slices():
    cx, cy = NX // 2, NY // 2
    return (slice(None), slice(None), slice(cx - 2, cx + 2), slice(cy - 2, cy + 2), slice(1, 4))


def add_smoke(density):
    density[source_slices()] += SOURCE_DENSITY
    return density


def add_force(velocity):
    _, _, xs, ys, zs = source_slices()
    velocity[:, 2:3, xs, ys, zs] += SOURCE_FORCE * DT
    return velocity


def laplacian(field):
    channels = field.shape[1]
    kernel = LAPLACIAN_KERNEL.repeat(channels, 1, 1, 1, 1)
    return F.conv3d(field, kernel, padding=1, groups=channels) / (DX * DX)


def divergence(velocity):
    div = torch.zeros_like(velocity[:, 0:1])
    div[:, :, 1:-1, 1:-1, 1:-1] = (
        (velocity[:, 0:1, 2:, 1:-1, 1:-1] - velocity[:, 0:1, :-2, 1:-1, 1:-1])
        + (velocity[:, 1:2, 1:-1, 2:, 1:-1] - velocity[:, 1:2, 1:-1, :-2, 1:-1])
        + (velocity[:, 2:3, 1:-1, 1:-1, 2:] - velocity[:, 2:3, 1:-1, 1:-1, :-2])
    ) / (2 * DX)
    return div


def curl(velocity):
    wx = torch.zeros_like(velocity[:, 0:1])
    wy = torch.zeros_like(velocity[:, 1:2])
    wz = torch.zeros_like(velocity[:, 2:3])

    wx[:, :, 1:-1, 1:-1, 1:-1] = (
        (velocity[:, 2:3, 1:-1, 2:, 1:-1] - velocity[:, 2:3, 1:-1, :-2, 1:-1])
        - (velocity[:, 1:2, 1:-1, 1:-1, 2:] - velocity[:, 1:2, 1:-1, 1:-1, :-2])
    ) / (2 * DX)

    wy[:, :, 1:-1, 1:-1, 1:-1] = (
        (velocity[:, 0:1, 1:-1, 1:-1, 2:] - velocity[:, 0:1, 1:-1, 1:-1, :-2])
        - (velocity[:, 2:3, 2:, 1:-1, 1:-1] - velocity[:, 2:3, :-2, 1:-1, 1:-1])
    ) / (2 * DX)

    wz[:, :, 1:-1, 1:-1, 1:-1] = (
        (velocity[:, 1:2, 2:, 1:-1, 1:-1] - velocity[:, 1:2, :-2, 1:-1, 1:-1])
        - (velocity[:, 0:1, 1:-1, 2:, 1:-1] - velocity[:, 0:1, 1:-1, :-2, 1:-1])
    ) / (2 * DX)

    return torch.cat((wx, wy, wz), dim=1)


def vorticity_confinement(velocity):
    omega = curl(velocity)
    omega_mag = torch.norm(omega, dim=1, keepdim=True)

    nx = torch.zeros_like(omega_mag)
    ny = torch.zeros_like(omega_mag)
    nz = torch.zeros_like(omega_mag)

    nx[:, :, 1:-1, 1:-1, 1:-1] = (omega_mag[:, :, 2:, 1:-1, 1:-1] - omega_mag[:, :, :-2, 1:-1, 1:-1]) / (2 * DX)
    ny[:, :, 1:-1, 1:-1, 1:-1] = (omega_mag[:, :, 1:-1, 2:, 1:-1] - omega_mag[:, :, 1:-1, :-2, 1:-1]) / (2 * DX)
    nz[:, :, 1:-1, 1:-1, 1:-1] = (omega_mag[:, :, 1:-1, 1:-1, 2:] - omega_mag[:, :, 1:-1, 1:-1, :-2]) / (2 * DX)

    direction = torch.cat((nx, ny, nz), dim=1)
    direction = direction / (torch.norm(direction, dim=1, keepdim=True) + 1e-6)
    force = VORT_EPS * torch.cross(direction, omega, dim=1)
    return velocity + DT * force


def pressure_solve(pressure, div):
    for _ in range(PRESSURE_ITERS):
        neighbor_sum = F.conv3d(pressure, NEIGHBOR_KERNEL, padding=1)
        pressure = (neighbor_sum - div * DX * DX) / 6.0
        pressure = apply_pressure_boundary(pressure)
    return pressure


def apply_pressure_boundary(pressure):
    pressure[:, :, 0, :, :] = pressure[:, :, 1, :, :]
    pressure[:, :, -1, :, :] = pressure[:, :, -2, :, :]
    pressure[:, :, :, 0, :] = pressure[:, :, :, 1, :]
    pressure[:, :, :, -1, :] = pressure[:, :, :, -2, :]
    pressure[:, :, :, :, 0] = pressure[:, :, :, :, 1]
    pressure[:, :, :, :, -1] = pressure[:, :, :, :, -2]
    return pressure


def subtract_pressure_gradient(velocity, pressure):
    grad = torch.zeros_like(velocity)
    grad[:, 0:1, 1:-1, 1:-1, 1:-1] = (pressure[:, :, 2:, 1:-1, 1:-1] - pressure[:, :, :-2, 1:-1, 1:-1]) / (2 * DX)
    grad[:, 1:2, 1:-1, 1:-1, 1:-1] = (pressure[:, :, 1:-1, 2:, 1:-1] - pressure[:, :, 1:-1, :-2, 1:-1]) / (2 * DX)
    grad[:, 2:3, 1:-1, 1:-1, 1:-1] = (pressure[:, :, 1:-1, 1:-1, 2:] - pressure[:, :, 1:-1, 1:-1, :-2]) / (2 * DX)
    return velocity - grad


def velocity_grid_displacement(velocity):
    _, _, x_size, y_size, z_size = velocity.shape
    return torch.cat(
        (
            velocity[:, 2:3] * DT * (2 / max(z_size - 1, 1)),
            velocity[:, 1:2] * DT * (2 / max(y_size - 1, 1)),
            velocity[:, 0:1] * DT * (2 / max(x_size - 1, 1)),
        ),
        dim=1,
    ).permute(0, 2, 3, 4, 1)


def base_grid(x_size, y_size, z_size):
    xs = torch.linspace(-1, 1, x_size, device=DEVICE)
    ys = torch.linspace(-1, 1, y_size, device=DEVICE)
    zs = torch.linspace(-1, 1, z_size, device=DEVICE)
    gx, gy, gz = torch.meshgrid(xs, ys, zs, indexing="ij")
    return torch.stack((gz, gy, gx), dim=-1).unsqueeze(0)


def advect(field, velocity, grid):
    _, _, x_size, y_size, z_size = field.shape
    if grid.shape[1:4] != (x_size, y_size, z_size):
        grid = base_grid(x_size, y_size, z_size)
    backtrace_grid = grid - velocity_grid_displacement(velocity)
    backtrace_grid = backtrace_grid.clamp(-1, 1)
    return F.grid_sample(field, backtrace_grid, align_corners=True, padding_mode="border")


def apply_boundary(velocity):
    velocity[:, :, 0, :, :] = 0
    velocity[:, :, -1, :, :] = 0
    velocity[:, :, :, 0, :] = 0
    velocity[:, :, :, -1, :] = 0
    velocity[:, :, :, :, 0] = 0
    velocity[:, :, :, :, -1] = 0
    return velocity


def divergence_metrics(velocity):
    div_abs = divergence(velocity).abs()
    return float(div_abs.mean().item()), float(div_abs.max().item())


def cfl_number(velocity):
    return float((velocity.norm(dim=1).max() * DT / DX).item())


def make_frame(density):
    side_slice = density[0, 0, NX // 2, :, :].detach().cpu().T.numpy()
    max_value = side_slice.max()
    if max_value > 0:
        side_slice = np.power(side_slice / max_value, 0.55)
    rgb = plt.get_cmap("inferno")(side_slice)[:, :, :3]
    return (rgb * 255).astype(np.uint8)


def make_3d_plane_frame(density, velocity, angle):
    plane_x = NX // 2
    density_plane = density[0, 0, plane_x, :, :].detach().cpu().T.numpy()
    max_density = density_plane.max()
    normalized_density = density_plane / max_density if max_density > 0 else density_plane

    y_axis = np.arange(NY)
    z_axis = np.arange(NZ)
    y_grid, z_grid = np.meshgrid(y_axis, z_axis)
    x_grid = np.full_like(y_grid, plane_x)

    colors = plt.get_cmap("inferno")(np.power(normalized_density, 0.55))
    colors[..., 3] = 0.25 + 0.75 * normalized_density

    fig = plt.figure(figsize=(6, 5), dpi=100)
    fig.patch.set_facecolor("#101014")
    canvas = FigureCanvasAgg(fig)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#101014")

    ax.plot_surface(
        x_grid,
        y_grid,
        z_grid,
        facecolors=colors,
        rstride=1,
        cstride=1,
        linewidth=0,
        antialiased=False,
        shade=False,
    )

    stride = PLANE_ARROW_STRIDE
    vx = velocity[0, 0, plane_x, :, :].detach().cpu().T.numpy()
    vy = velocity[0, 1, plane_x, :, :].detach().cpu().T.numpy()
    vz = velocity[0, 2, plane_x, :, :].detach().cpu().T.numpy()
    speed = np.sqrt(vx * vx + vy * vy + vz * vz)

    if speed.max() > 1e-6:
        ax.quiver(
            x_grid[::stride, ::stride],
            y_grid[::stride, ::stride],
            z_grid[::stride, ::stride],
            vx[::stride, ::stride],
            vy[::stride, ::stride],
            vz[::stride, ::stride],
            length=4,
            normalize=True,
            color="#55d6ff",
            linewidth=0.8,
            alpha=0.85,
        )

    ax.set_xlim(0, NX - 1)
    ax.set_ylim(0, NY - 1)
    ax.set_zlim(0, NZ - 1)
    ax.set_xlabel("X", color="white")
    ax.set_ylabel("Y", color="white")
    ax.set_zlabel("Z", color="white")
    ax.set_title("Navier-Stokes smoke flow on a 3D slice plane", color="white", pad=14)
    ax.tick_params(colors="white")
    ax.view_init(elev=24, azim=angle)
    ax.set_box_aspect((1, 1, 1))
    fig.tight_layout(pad=0.6)

    canvas.draw()
    frame = np.asarray(canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return frame


def synchronize():
    if DEVICE.type == "cuda":
        torch.cuda.synchronize()


def open_in_vscode(path):
    code_command = shutil.which("code") or shutil.which("code.cmd")
    if code_command is None:
        print(f"VS Code command not found. Open this file in VS Code instead: {path}")
        print(f"Or run: code -r \"{path}\"")
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

    print(f"Opened animation in VS Code: {path}")
    return True


def parse_args():
    parser = argparse.ArgumentParser(description="Run a tiny 3D smoke simulation and save an animated GIF.")
    parser.add_argument("--steps", type=int, default=STEPS, help="Number of simulation steps.")
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR, help="Directory for generated images and tensors.")
    parser.add_argument(
        "--no-vscode",
        action="store_true",
        help="Save the GIF without opening it in VS Code.",
    )
    parser.add_argument(
        "--no-3d-plane",
        action="store_true",
        help="Skip the 3D plane flow visualization GIF.",
    )
    return parser.parse_args()


def main(open_vscode=True, render_3d_plane=True, steps=STEPS, out_dir=OUT_DIR):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    density = torch.zeros((1, 1, NX, NY, NZ), device=DEVICE)
    velocity = torch.zeros((1, 3, NX, NY, NZ), device=DEVICE)
    pressure = torch.zeros((1, 1, NX, NY, NZ), device=DEVICE)
    advection_grid = base_grid(NX, NY, NZ)
    diagnostics = []
    frames = []
    plane_frames = []

    synchronize()
    start = time.time()

    for step in range(steps):
        should_sample_diagnostics = step % DIAGNOSTIC_INTERVAL == 0 or step == steps - 1

        velocity = add_force(velocity)
        velocity = advect(velocity, velocity, advection_grid)
        velocity = vorticity_confinement(velocity)
        velocity = velocity + VISC * DT * laplacian(velocity)

        div = divergence(velocity)
        div_before_mean = float(div.abs().mean().item()) if should_sample_diagnostics else None
        div_before_max = float(div.abs().max().item()) if should_sample_diagnostics else None
        pressure = pressure_solve(pressure, div)
        velocity = subtract_pressure_gradient(velocity, pressure)
        velocity = apply_boundary(velocity * DAMPING)
        velocity = torch.nan_to_num(velocity, nan=0.0, posinf=0.0, neginf=0.0).clamp(-10, 10)

        if should_sample_diagnostics:
            div_after_mean, div_after_max = divergence_metrics(velocity)
            diagnostics.append(
                {
                    "step": step,
                    "div_before_mean": div_before_mean,
                    "div_before_max": div_before_max,
                    "div_after_mean": div_after_mean,
                    "div_after_max": div_after_max,
                    "cfl": cfl_number(velocity),
                }
            )

        density = add_smoke(density)
        density = advect(density, velocity, advection_grid)
        density = torch.nan_to_num(density, nan=0.0, posinf=10.0, neginf=0.0).clamp(0, 10)

        if step % 2 == 0:
            frames.append(make_frame(density))
        if render_3d_plane and step % PLANE_FRAME_INTERVAL == 0:
            plane_frames.append(make_3d_plane_frame(density, velocity, angle=35 + step * 2))

    synchronize()
    elapsed = time.time() - start

    gif_path = out_dir / "smoke.gif"
    plane_gif_path = out_dir / "smoke_3d_plane.gif"
    slice_tensor_path = out_dir / "smoke_slice.pt"
    slice_png_path = out_dir / "smoke_slice.png"

    imageio.mimsave(gif_path, frames, fps=15)
    if plane_frames:
        imageio.mimsave(plane_gif_path, plane_frames, fps=10)

    final_slice = density[0, 0, NX // 2, :, :].detach().cpu().T
    torch.save(final_slice, slice_tensor_path)

    plt.imshow(final_slice.numpy(), cmap="inferno", origin="lower")
    plt.colorbar()
    plt.title("Smoke side slice")
    plt.savefig(slice_png_path, dpi=150, bbox_inches="tight")
    plt.close()

    speed = velocity.norm(dim=1)
    print(f"Device: {DEVICE}")
    print(f"Saved animation to {gif_path}")
    if plane_frames:
        print(f"Saved 3D plane flow animation to {plane_gif_path}")
    print(f"Saved final slice to {slice_tensor_path}")
    print(f"Saved final slice image to {slice_png_path}")
    print(f"Finished {steps} steps in {elapsed:.3f} s")
    print("Density range:", float(density.min()), float(density.max()))
    print("Velocity magnitude:", float(speed.min()), float(speed.max()))
    print("Projection diagnostics:")
    for sample in diagnostics:
        print(
            f"  step {sample['step']:03d}: "
            f"div mean {sample['div_before_mean']:.6f} -> {sample['div_after_mean']:.6f}, "
            f"div max {sample['div_before_max']:.6f} -> {sample['div_after_max']:.6f}, "
            f"CFL {sample['cfl']:.3f}"
        )

    if open_vscode:
        open_in_vscode(plane_gif_path if plane_frames else gif_path)


if __name__ == "__main__":
    args = parse_args()
    main(
        open_vscode=not args.no_vscode,
        render_3d_plane=not args.no_3d_plane,
        steps=max(1, args.steps),
        out_dir=args.out_dir,
    )
