#!/usr/bin/env python3
"""Compare seeded sensor-model sample statistics with configured distributions."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulation.quadrotor_6dof import Quadrotor6DOF, QuadrotorConfig, QuadrotorState  # noqa: E402
from simulation.sensor_models import ImuBarometerModel, SensorModelConfig  # noqa: E402


def main() -> int:
    count = 20_000
    config = QuadrotorConfig()
    hover_speed = np.sqrt(config.mass_kg * config.gravity_m_s2 / (4.0 * config.thrust_coefficient_n_per_rad_s2))
    state = QuadrotorState(
        position_ned_m=np.asarray((0.0, 0.0, -2.0)),
        body_rates_pqr_rad_s=np.asarray((0.1, -0.2, 0.3)),
        motor_speeds_rad_s=np.full(4, hover_speed),
    )
    plant = Quadrotor6DOF(config, state)
    cfg = SensorModelConfig(
        gyro_bias_rad_s=(0.01, -0.02, 0.03),
        gyro_scale=(1.02, 0.98, 1.01),
        gyro_noise_std_rad_s=0.01,
        gyro_quantization_rad_s=0.001,
        accel_bias_m_s2=(0.04, -0.03, 0.06),
        accel_scale=(1.01, 0.99, 1.02),
        accel_noise_std_m_s2=0.08,
        accel_quantization_m_s2=0.01,
        altitude_bias_m=0.03,
        altitude_scale=1.02,
        altitude_noise_std_m=0.025,
        altitude_quantization_m=0.005,
    )
    model = ImuBarometerModel(cfg, seed=20261007)
    samples = [model.sample(plant) for _ in range(count)]
    gyro = np.asarray([s.gyro_pqr_rad_s for s in samples])
    accel = np.asarray([s.accel_specific_force_frd_m_s2 for s in samples])
    altitude = np.asarray([s.altitude_m for s in samples])

    gyro_mean = np.asarray(state.body_rates_pqr_rad_s) * np.asarray(cfg.gyro_scale) + np.asarray(cfg.gyro_bias_rad_s)
    accel_mean = plant.specific_force_body() * np.asarray(cfg.accel_scale) + np.asarray(cfg.accel_bias_m_s2)
    altitude_mean = 2.0 * cfg.altitude_scale + cfg.altitude_bias_m
    checks = [
        ("gyro mean", gyro.mean(axis=0), gyro_mean, cfg.gyro_noise_std_rad_s, cfg.gyro_quantization_rad_s),
        ("accelerometer mean", accel.mean(axis=0), accel_mean, cfg.accel_noise_std_m_s2, cfg.accel_quantization_m_s2),
        ("altitude mean", np.asarray((altitude.mean(),)), np.asarray((altitude_mean,)), cfg.altitude_noise_std_m, cfg.altitude_quantization_m),
    ]
    for name, actual, expected, noise_std, quantum in checks:
        tolerance = max(5.0 * noise_std / np.sqrt(count), quantum * 0.55)
        error = np.max(np.abs(actual - expected))
        if error > tolerance:
            raise AssertionError(f"{name} error {error:.4g} exceeds tolerance {tolerance:.4g}")

    variance_specs = [
        ("gyro", gyro.var(axis=0), cfg.gyro_noise_std_rad_s**2 + cfg.gyro_quantization_rad_s**2 / 12.0),
        ("accelerometer", accel.var(axis=0), cfg.accel_noise_std_m_s2**2 + cfg.accel_quantization_m_s2**2 / 12.0),
        ("altitude", np.asarray((altitude.var(),)), cfg.altitude_noise_std_m**2 + cfg.altitude_quantization_m**2 / 12.0),
    ]
    for name, actual, expected in variance_specs:
        relative_error = np.max(np.abs(actual - expected) / expected)
        if relative_error > 0.08:
            raise AssertionError(f"{name} variance relative error {relative_error:.3%} exceeds 8%")

    print(f"Sensor statistics passed for {count} seeded samples (seed 20261007).")
    print("Checked gyro/accelerometer/barometer mean offsets and noise-plus-quantization variances.")
    print("These configured demonstration distributions are software checks, not sensor characterization.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
