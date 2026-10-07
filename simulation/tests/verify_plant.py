#!/usr/bin/env python3
"""Check hover force balance and RK4 response sensitivity to timestep."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulation.quadrotor_6dof import Quadrotor6DOF, QuadrotorConfig, QuadrotorState  # noqa: E402


def run_fixed_commands(dt: float, duration: float = 0.2) -> np.ndarray:
    config = QuadrotorConfig()
    hover_speed = np.sqrt(config.mass_kg * config.gravity_m_s2 / (4.0 * config.thrust_coefficient_n_per_rad_s2))
    state = QuadrotorState(motor_speeds_rad_s=np.full(4, hover_speed))
    plant = Quadrotor6DOF(config, state)
    commands = np.asarray((hover_speed * 0.99, hover_speed * 1.01,
                           hover_speed * 1.005, hover_speed * 0.995))
    for _ in range(round(duration / dt)):
        plant.step(commands, dt)
    return plant.state.pack()


def main() -> int:
    config = QuadrotorConfig()
    hover_speed = np.sqrt(config.mass_kg * config.gravity_m_s2 / (4.0 * config.thrust_coefficient_n_per_rad_s2))
    state = QuadrotorState(motor_speeds_rad_s=np.full(4, hover_speed))
    hover = Quadrotor6DOF(config, state)
    for _ in range(1000):
        hover.step(np.full(4, hover_speed), 0.001)
    if not np.allclose(hover.state.position_ned_m, 0.0, atol=1e-10):
        raise AssertionError(f"level hover force balance drifted: {hover.state.position_ned_m}")
    if not np.allclose(hover.state.velocity_ned_m_s, 0.0, atol=1e-10):
        raise AssertionError(f"level hover force balance developed velocity: {hover.state.velocity_ned_m_s}")

    coarse = run_fixed_commands(0.004)
    medium = run_fixed_commands(0.002)
    fine = run_fixed_commands(0.001)
    coarse_error = float(np.linalg.norm(coarse - fine))
    medium_error = float(np.linalg.norm(medium - fine))
    if not coarse_error > medium_error or not np.all(np.isfinite(fine)):
        raise AssertionError(f"timestep refinement did not reduce state difference: coarse={coarse_error}, medium={medium_error}")

    print("Level hover balance passed for 1.0 simulated second from exact hover rotor speed.")
    print(f"Timestep refinement reduced state difference: dt=0.004 vs 0.001: {coarse_error:.3g}; dt=0.002 vs 0.001: {medium_error:.3g}.")
    print("These are internal model checks, not validation of measured aircraft dynamics.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
