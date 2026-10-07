from pathlib import Path
import argparse
import json
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

NX, NY, NZ = 56, 44, 64
DX = 1.0
DT = 0.045
WARMUP_STEPS = 70
STEPS = 130
VISC = 0.0015
PRESSURE_ITERS = 45
DAMPING = 0.996
DIAGNOSTIC_INTERVAL = 13

SURFACE_X0 = 12.0
SURFACE_CURVE = 24.0
BOUNDARY_LAYER = 5.0
JET_SPEED = 5.5
JET_TRACER = 0.9
JET_Z_CENTER = 3.0
JET_Z_WIDTH = 2.0
JET_Y_WIDTH = 18.0
COANDA_ATTACH_GAIN = 5.0
COANDA_ALIGN_GAIN = 3.5
SEPARATION_PENALTY = 0.35
MAX_SPEED = 9.0

FRAME_INTERVAL = 4
SIDE_FRAME_INTERVAL = 2
ARROW_STRIDE_Y = 7
ARROW_STRIDE_Z = 7
SLICE_X_WIDTH = 5.0
SLICE_X_OFFSETS = (0.0, 5.0, 10.0, 15.0)

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


def coordinate_fields():
    x = torch.arange(NX, device=DEVICE, dtype=torch.float32).view(1, 1, NX, 1, 1)
    y = torch.arange(NY, device=DEVICE, dtype=torch.float32).view(1, 1, 1, NY, 1)
    z = torch.arange(NZ, device=DEVICE, dtype=torch.float32).view(1, 1, 1, 1, NZ)
    return x, y, z


def curved_surface_fields():
    x, y, z = coordinate_fields()
    s = z / max(NZ - 1, 1)
    surface_x = SURFACE_X0 + SURFACE_CURVE * (1 - torch.cos(torch.pi * s)) / 2
    dx_dz = SURFACE_CURVE * torch.pi * torch.sin(torch.pi * s) / (2 * max(NZ - 1, 1))

    signed_distance = x - surface_x
    fluid_mask = (signed_distance >= 0).float()

    boundary_layer = torch.exp(-0.5 * (signed_distance / BOUNDARY_LAYER) ** 2)
    boundary_layer = boundary_layer * fluid_mask

    tangent_norm = torch.sqrt(1 + dx_dz * dx_dz)
    tangent = torch.cat(
        (
            dx_dz / tangent_norm,
            torch.zeros_like(dx_dz),
            torch.ones_like(dx_dz) / tangent_norm,
        ),
        dim=1,
    )
    normal = torch.cat(
        (
            torch.ones_like(dx_dz) / tangent_norm,
            torch.zeros_like(dx_dz),
            -dx_dz / tangent_norm,
        ),
        dim=1,
    )

    center_y = (NY - 1) / 2
    jet_y = torch.exp(-0.5 * ((y - center_y) / JET_Y_WIDTH) ** 2)
    jet_z = torch.exp(-0.5 * ((z - JET_Z_CENTER) / JET_Z_WIDTH) ** 2)
    jet_x = torch.exp(-0.5 * (signed_distance / BOUNDARY_LAYER) ** 2) * fluid_mask
    jet_mask = jet_x * jet_y * jet_z

    return {
        "signed_distance": signed_distance,
        "fluid_mask": fluid_mask,
        "boundary_layer": boundary_layer,
        "tangent": tangent,
        "normal": normal,
        "jet_mask": jet_mask,
        "surface_x_1d": surface_x.view(NZ).detach().cpu().numpy(),
    }


SURFACE = curved_surface_fields()


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


def base_grid():
    xs = torch.linspace(-1, 1, NX, device=DEVICE)
    ys = torch.linspace(-1, 1, NY, device=DEVICE)
    zs = torch.linspace(-1, 1, NZ, device=DEVICE)
    gx, gy, gz = torch.meshgrid(xs, ys, zs, indexing="ij")
    return torch.stack((gz, gy, gx), dim=-1).unsqueeze(0)


ADVECTION_GRID = base_grid()


def velocity_grid_displacement(velocity):
    return torch.cat(
        (
            velocity[:, 2:3] * DT * (2 / max(NZ - 1, 1)),
            velocity[:, 1:2] * DT * (2 / max(NY - 1, 1)),
            velocity[:, 0:1] * DT * (2 / max(NX - 1, 1)),
        ),
        dim=1,
    ).permute(0, 2, 3, 4, 1)


def advect(field, velocity):
    backtrace_grid = (ADVECTION_GRID - velocity_grid_displacement(velocity)).clamp(-1, 1)
    return F.grid_sample(field, backtrace_grid, align_corners=True, padding_mode="border")


def apply_domain_boundary(velocity):
    velocity[:, :, 0, :, :] = 0
    velocity[:, :, -1, :, :] = 0
    velocity[:, :, :, 0, :] = 0
    velocity[:, :, :, -1, :] = 0
    velocity[:, :, :, :, 0] = velocity[:, :, :, :, 1]
    velocity[:, :, :, :, -1] = velocity[:, :, :, :, -2]
    return velocity


def apply_surface_boundary(velocity, tracer):
    fluid = SURFACE["fluid_mask"]
    velocity = velocity * fluid
    tracer = tracer * fluid
    return velocity, tracer


def inject_jet(velocity, tracer):
    jet = SURFACE["jet_mask"]
    tangent = SURFACE["tangent"]
    target_velocity = JET_SPEED * tangent
    velocity = velocity + DT * 12.0 * jet * (target_velocity - velocity)
    tracer = torch.maximum(tracer, JET_TRACER * jet)
    return velocity, tracer


def apply_coanda_force(velocity):
    signed_distance = SURFACE["signed_distance"]
    boundary_layer = SURFACE["boundary_layer"]
    tangent = SURFACE["tangent"]
    normal = SURFACE["normal"]

    normal_speed = (velocity * normal).sum(dim=1, keepdim=True)
    tangent_velocity = (velocity * tangent).sum(dim=1, keepdim=True) * tangent
    align_force = COANDA_ALIGN_GAIN * boundary_layer * (tangent_velocity - velocity)

    distance_ratio = torch.clamp(signed_distance / BOUNDARY_LAYER, min=0.0, max=2.0)
    attach_force = -COANDA_ATTACH_GAIN * boundary_layer * distance_ratio * normal

    away_from_surface = torch.clamp(normal_speed, min=0.0)
    separation_force = -SEPARATION_PENALTY * boundary_layer * away_from_surface * normal

    return velocity + DT * (align_force + attach_force + separation_force)


def project(velocity, pressure):
    div = divergence(velocity)
    pressure = pressure_solve(pressure, div)
    velocity = subtract_pressure_gradient(velocity, pressure)
    return velocity, pressure, div


def divergence_metrics(velocity):
    div_abs = divergence(velocity).abs()
    return float(div_abs.mean().item()), float(div_abs.max().item())


def cfl_number(velocity):
    return float((velocity.norm(dim=1).max() * DT / DX).item())


def coanda_metrics(tracer, velocity):
    boundary = SURFACE["boundary_layer"]
    tangent = SURFACE["tangent"]
    boundary_volume = boundary * torch.ones_like(tracer)
    tracer_total = tracer.sum() + 1e-6
    attached_fraction = (tracer * boundary).sum() / tracer_total
    tangent_speed = ((velocity * tangent).sum(dim=1, keepdim=True) * boundary).sum() / (boundary_volume.sum() + 1e-6)
    return float(attached_fraction.item()), float(tangent_speed.item())


def surface_sample(field, offset=2.0):
    surface_x = SURFACE["surface_x_1d"]
    z_indices = np.arange(NZ)
    x_indices = np.clip(np.rint(surface_x + offset).astype(np.int64), 0, NX - 1)
    sampled = []
    cpu_field = field.detach().cpu()
    for z_idx, x_idx in zip(z_indices, x_indices):
        sampled.append(cpu_field[0, :, x_idx, :, z_idx])
    return torch.stack(sampled, dim=-1)


def slice_band_sample(field, offset, width=SLICE_X_WIDTH):
    surface_x = SURFACE["surface_x_1d"]
    cpu_field = field.detach().cpu()
    sampled = []

    for z_idx, surface_x_pos in enumerate(surface_x):
        start_x = int(np.clip(np.floor(surface_x_pos + offset), 0, NX - 1))
        end_x = int(np.clip(np.ceil(surface_x_pos + offset + width), start_x + 1, NX))
        sampled.append(cpu_field[0, :, start_x:end_x, :, z_idx].mean(dim=1))

    return torch.stack(sampled, dim=-1)


def make_side_frame(tracer):
    mid_y = NY // 2
    image = tracer[0, 0, :, mid_y, :].detach().cpu().numpy()
    max_value = image.max()
    if max_value > 0:
        image = np.power(image / max_value, 0.6)

    rgb = plt.get_cmap("turbo")(image.T)[:, :, :3]

    surface_x = SURFACE["surface_x_1d"]
    for z_idx, x_pos in enumerate(surface_x.astype(int)):
        if 0 <= x_pos < NX:
            rgb[z_idx, x_pos : min(x_pos + 2, NX), :] = np.array([1.0, 1.0, 1.0])

    return (rgb * 255).astype(np.uint8)


def make_curved_surface_frame(tracer, velocity, angle):
    surface_x = SURFACE["surface_x_1d"]
    y_axis = np.arange(NY)
    z_axis = np.arange(NZ)
    y_grid, z_grid = np.meshgrid(y_axis, z_axis, indexing="ij")

    fig = plt.figure(figsize=(7, 5), dpi=105)
    fig.patch.set_facecolor("#0d1117")
    canvas = FigureCanvasAgg(fig)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#0d1117")

    stride_y = ARROW_STRIDE_Y
    stride_z = ARROW_STRIDE_Z

    for slice_index, offset in enumerate(reversed(SLICE_X_OFFSETS)):
        slice_center = offset + SLICE_X_WIDTH / 2
        x_grid = np.tile((surface_x + slice_center).reshape(1, NZ), (NY, 1))

        tracer_surface = slice_band_sample(tracer, offset=offset)[0].numpy()
        max_tracer = tracer_surface.max()
        normalized = tracer_surface / max_tracer if max_tracer > 0 else tracer_surface
        colors = plt.get_cmap("turbo")(np.power(normalized, 0.55))
        colors[..., 3] = 0.12 + 0.58 * normalized

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

        velocity_surface = slice_band_sample(velocity, offset=offset).numpy()
        vx, vy, vz = velocity_surface[0], velocity_surface[1], velocity_surface[2]
        speed = np.sqrt(vx * vx + vy * vy + vz * vz)
        if speed.max() > 1e-5:
            arrow_color = "#ffffff" if offset == 0 else "#9ee7ff"
            ax.quiver(
                x_grid[::stride_y, ::stride_z],
                y_grid[::stride_y, ::stride_z],
                z_grid[::stride_y, ::stride_z],
                vx[::stride_y, ::stride_z],
                vy[::stride_y, ::stride_z],
                vz[::stride_y, ::stride_z],
                length=4,
                normalize=True,
                color=arrow_color,
                linewidth=0.65,
                alpha=max(0.35, 0.9 - 0.12 * slice_index),
            )

    ax.set_xlim(0, NX - 1)
    ax.set_ylim(0, NY - 1)
    ax.set_zlim(0, NZ - 1)
    ax.set_xlabel("X", color="white")
    ax.set_ylabel("Y", color="white")
    ax.set_zlabel("Z", color="white")
    ax.set_title("Coanda airflow across 5-unit X slices", color="white", pad=12)
    ax.tick_params(colors="white")
    ax.view_init(elev=26, azim=angle)
    ax.set_box_aspect((NX, NY, NZ))
    fig.tight_layout(pad=0.4)

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
    parser = argparse.ArgumentParser(description="Visual Coanda-effect airflow over a curved surface.")
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR, help="Directory for generated GIFs and images.")
    parser.add_argument("--no-vscode", action="store_true", help="Save GIFs without opening the main GIF in VS Code.")
    parser.add_argument("--no-3d", action="store_true", help="Skip the slower 3D curved-surface GIF.")
    parser.add_argument("--warmup-steps", type=int, default=WARMUP_STEPS, help="Simulation steps to run before recording the looping GIF.")
    parser.add_argument("--record-steps", type=int, default=STEPS, help="Simulation steps to record into the looping GIF.")
    return parser.parse_args()


def save_looping_gif(path, frames, fps):
    loop_frames = frames + frames[:1] if len(frames) > 1 else frames
    imageio.mimsave(path, loop_frames, fps=fps, loop=0)


def main(open_vscode=True, render_3d=True, warmup_steps=WARMUP_STEPS, record_steps=STEPS, out_dir=OUT_DIR):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    tracer = torch.zeros((1, 1, NX, NY, NZ), device=DEVICE)
    velocity = torch.zeros((1, 3, NX, NY, NZ), device=DEVICE)
    pressure = torch.zeros((1, 1, NX, NY, NZ), device=DEVICE)

    side_frames = []
    surface_frames = []
    diagnostics = []
    total_steps = max(0, warmup_steps) + max(1, record_steps)
    recorded_steps = max(1, record_steps)
    surface_frame_count = max(1, (recorded_steps + FRAME_INTERVAL - 1) // FRAME_INTERVAL)
    surface_frame_index = 0

    synchronize()
    start = time.time()

    for step in range(total_steps):
        record_step = step - max(0, warmup_steps)
        is_recording = record_step >= 0
        should_sample = is_recording and (record_step % DIAGNOSTIC_INTERVAL == 0 or record_step == recorded_steps - 1)

        velocity, tracer = inject_jet(velocity, tracer)
        velocity = advect(velocity, velocity)
        velocity = apply_coanda_force(velocity)
        velocity = velocity + VISC * DT * laplacian(velocity)
        velocity, tracer = apply_surface_boundary(velocity, tracer)

        velocity, pressure, div_before = project(velocity, pressure)
        velocity = apply_domain_boundary(velocity * DAMPING)
        velocity, tracer = apply_surface_boundary(velocity, tracer)
        velocity = torch.nan_to_num(velocity, nan=0.0, posinf=0.0, neginf=0.0).clamp(-MAX_SPEED, MAX_SPEED)

        tracer = advect(tracer, velocity)
        tracer = 0.998 * torch.nan_to_num(tracer, nan=0.0, posinf=1.0, neginf=0.0).clamp(0, 1)
        velocity, tracer = apply_surface_boundary(velocity, tracer)

        if should_sample:
            div_after_mean, div_after_max = divergence_metrics(velocity)
            attached_fraction, tangent_speed = coanda_metrics(tracer, velocity)
            diagnostics.append(
                {
                    "step": record_step,
                    "div_before_mean": float(div_before.abs().mean().item()),
                    "div_before_max": float(div_before.abs().max().item()),
                    "div_after_mean": div_after_mean,
                    "div_after_max": div_after_max,
                    "cfl": cfl_number(velocity),
                    "attached_fraction": attached_fraction,
                    "tangent_speed": tangent_speed,
                }
            )

        if is_recording and record_step % SIDE_FRAME_INTERVAL == 0:
            side_frames.append(make_side_frame(tracer))
        if render_3d and is_recording and record_step % FRAME_INTERVAL == 0:
            angle = 32 + 360 * surface_frame_index / surface_frame_count
            surface_frames.append(make_curved_surface_frame(tracer, velocity, angle=angle))
            surface_frame_index += 1

    synchronize()
    elapsed = time.time() - start

    side_gif_path = out_dir / "coanda_side.gif"
    surface_gif_path = out_dir / "coanda_curved_surface.gif"
    final_slice_path = out_dir / "coanda_mid_slice.png"

    if not side_frames:
        side_frames.append(make_side_frame(tracer))
    save_looping_gif(side_gif_path, side_frames, fps=15)
    if surface_frames:
        save_looping_gif(surface_gif_path, surface_frames, fps=10)

    final_frame = make_side_frame(tracer)
    imageio.imwrite(final_slice_path, final_frame)

    speed = velocity.norm(dim=1)
    telemetry_path = out_dir / "coanda_telemetry.json"
    telemetry = {
        "device": str(DEVICE),
        "warmup_steps": max(0, warmup_steps),
        "recorded_steps": recorded_steps,
        "total_steps": total_steps,
        "elapsed_seconds": elapsed,
        "tracer_range": [float(tracer.min().item()), float(tracer.max().item())],
        "velocity_magnitude_range": [float(speed.min().item()), float(speed.max().item())],
        "coanda_diagnostics": diagnostics,
        "outputs": {
            "side_animation": str(side_gif_path),
            "curved_surface_animation": str(surface_gif_path) if surface_frames else None,
            "mid_slice_image": str(final_slice_path),
        },
    }
    telemetry_path.write_text(json.dumps(telemetry, indent=2), encoding="utf-8")

    print(f"Device: {DEVICE}")
    print(f"Saved side-view airflow animation to {side_gif_path}")
    if surface_frames:
        print(f"Saved curved-surface Coanda animation to {surface_gif_path}")
    print(f"Saved final mid-slice image to {final_slice_path}")
    print(f"Saved telemetry report to {telemetry_path}")
    print(f"Warmup steps: {max(0, warmup_steps)}")
    print(f"Recorded loop steps: {recorded_steps}")
    print(f"Finished {total_steps} total steps in {elapsed:.3f} s")
    print("Tracer range:", float(tracer.min().item()), float(tracer.max().item()))
    print("Velocity magnitude:", float(speed.min().item()), float(speed.max().item()))
    print("Coanda diagnostics:")
    for sample in diagnostics:
        print(
            f"  step {sample['step']:03d}: "
            f"div mean {sample['div_before_mean']:.6f} -> {sample['div_after_mean']:.6f}, "
            f"div max {sample['div_before_max']:.6f} -> {sample['div_after_max']:.6f}, "
            f"CFL {sample['cfl']:.3f}, "
            f"attached tracer {sample['attached_fraction']:.3f}, "
            f"near-wall tangent speed {sample['tangent_speed']:.3f}"
        )

    if open_vscode:
        open_in_vscode(telemetry_path)


if __name__ == "__main__":
    args = parse_args()
    main(
        open_vscode=not args.no_vscode,
        render_3d=not args.no_3d,
        warmup_steps=args.warmup_steps,
        record_steps=args.record_steps,
        out_dir=args.out_dir,
    )
