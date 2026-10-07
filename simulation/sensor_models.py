"""Seeded IMU/barometer measurement models for simulation fault experiments."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from simulation.quadrotor_6dof import Quadrotor6DOF


@dataclass(frozen=True)
class SensorModelConfig:
    gyro_bias_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    gyro_noise_std_rad_s: float = 0.0
    gyro_quantization_rad_s: float = 0.0
    accel_bias_m_s2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    accel_noise_std_m_s2: float = 0.0
    accel_quantization_m_s2: float = 0.0
    altitude_bias_m: float = 0.0
    altitude_scale: float = 1.0
    altitude_noise_std_m: float = 0.0
    altitude_quantization_m: float = 0.0
    altitude_drift_m_s: float = 0.0
    delay_steps: int = 0
    dropout_every_n: int = 0

    def __post_init__(self) -> None:
        for name in ("gyro_bias_rad_s", "gyro_scale", "accel_bias_m_s2", "accel_scale"):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.shape != (3,) or not np.all(np.isfinite(values)):
                raise ValueError(f"{name} must contain three finite values")
        for name in ("gyro_noise_std_rad_s", "gyro_quantization_rad_s", "accel_noise_std_m_s2",
                     "accel_quantization_m_s2", "altitude_noise_std_m", "altitude_quantization_m"):
            value = getattr(self, name)
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if not np.isfinite(self.altitude_bias_m) or not np.isfinite(self.altitude_drift_m_s):
            raise ValueError("altitude bias and drift must be finite")
        if not np.isfinite(self.altitude_scale) or self.altitude_scale <= 0.0:
            raise ValueError("altitude_scale must be finite and positive")
        if self.delay_steps < 0 or self.dropout_every_n < 0:
            raise ValueError("delay_steps and dropout_every_n must be non-negative")


@dataclass(frozen=True)
class SensorSample:
    timestamp_s: float
    gyro_pqr_rad_s: np.ndarray
    accel_specific_force_frd_m_s2: np.ndarray
    altitude_m: float
    valid: bool = True


class ImuBarometerModel:
    """Generate deterministic-with-seed gyro, accelerometer, and altitude samples."""

    def __init__(self, config: SensorModelConfig | None = None, seed: int = 0):
        self.config = config or SensorModelConfig()
        self.rng = np.random.default_rng(seed)
        self._sample_index = 0
        self._queue: deque[SensorSample] = deque()
        self._last_sample: SensorSample | None = None

    def sample(self, plant: Quadrotor6DOF) -> SensorSample:
        state = plant.state
        cfg = self.config
        gyro = (
            np.asarray(state.body_rates_pqr_rad_s) * np.asarray(cfg.gyro_scale)
            + np.asarray(cfg.gyro_bias_rad_s)
            + self.rng.normal(0.0, cfg.gyro_noise_std_rad_s, 3)
        )
        accel = (
            plant.specific_force_body() * np.asarray(cfg.accel_scale)
            + np.asarray(cfg.accel_bias_m_s2)
            + self.rng.normal(0.0, cfg.accel_noise_std_m_s2, 3)
        )
        altitude = (
            -float(state.position_ned_m[2]) * cfg.altitude_scale
            + cfg.altitude_bias_m
            + cfg.altitude_drift_m_s * state.time_s
            + float(self.rng.normal(0.0, cfg.altitude_noise_std_m))
        )
        if cfg.gyro_quantization_rad_s > 0.0:
            gyro = np.round(gyro / cfg.gyro_quantization_rad_s) * cfg.gyro_quantization_rad_s
        if cfg.accel_quantization_m_s2 > 0.0:
            accel = np.round(accel / cfg.accel_quantization_m_s2) * cfg.accel_quantization_m_s2
        if cfg.altitude_quantization_m > 0.0:
            altitude = round(altitude / cfg.altitude_quantization_m) * cfg.altitude_quantization_m
        fresh = SensorSample(float(state.time_s), gyro, accel, altitude, True)
        self._queue.append(fresh)
        if len(self._queue) > cfg.delay_steps:
            delayed = self._queue.popleft()
        else:
            delayed = self._queue[0]
        self._sample_index += 1
        dropped = cfg.dropout_every_n > 0 and self._sample_index % cfg.dropout_every_n == 0
        if dropped:
            held = self._last_sample or delayed
            result = SensorSample(
                held.timestamp_s,
                held.gyro_pqr_rad_s.copy(),
                held.accel_specific_force_frd_m_s2.copy(),
                held.altitude_m,
                False,
            )
        else:
            result = delayed
        self._last_sample = result
        return result


def demonstration_fault_profile() -> SensorModelConfig:
    """Illustrative fault levels for software exercising, not sensor specs."""
    return SensorModelConfig(
        gyro_bias_rad_s=(0.008, -0.006, 0.01),
        gyro_noise_std_rad_s=0.003,
        gyro_quantization_rad_s=0.0005,
        accel_bias_m_s2=(0.04, -0.03, 0.06),
        accel_noise_std_m_s2=0.08,
        accel_quantization_m_s2=0.01,
        altitude_bias_m=0.03,
        altitude_noise_std_m=0.025,
        altitude_quantization_m=0.005,
        altitude_drift_m_s=0.001,
        delay_steps=2,
    )
