"""Shared linear pitch-axis model used by the phase-portrait visualizations."""

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class PitchModelConfig:
    """Single-axis pitch model parameters recovered from the project transcript."""

    natural_frequency: float = 1.8
    cyclic_gain: float = 2.6
    time_end: float = 10.0
    dt: float = 0.01
    initial_pitch_deg: float = 10.0
    initial_pitch_rate_deg_s: float = 0.0
    cyclic_step_deg: float = 5.0
    cyclic_step_time: float = 1.0


def state_matrices(
    cfg: PitchModelConfig, zeta: float
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return state-space matrices for x = [theta, q]^T and cyclic input."""
    omega = cfg.natural_frequency
    a = np.array([[0.0, 1.0], [-(omega**2), -2.0 * zeta * omega]])
    b = np.array([[0.0], [cfg.cyclic_gain]])
    c = np.eye(2)
    d = np.zeros((2, 1))
    return a, b, c, d


def rk4_step(
    a: np.ndarray,
    b: np.ndarray,
    state: np.ndarray,
    control: float,
    dt: float,
) -> np.ndarray:
    """Advance the linear state equation by one fourth-order Runge-Kutta step."""
    def rhs(x: np.ndarray) -> np.ndarray:
        return a @ x + b[:, 0] * control

    k1 = rhs(state)
    k2 = rhs(state + 0.5 * dt * k1)
    k3 = rhs(state + 0.5 * dt * k2)
    k4 = rhs(state + dt * k3)
    return state + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
