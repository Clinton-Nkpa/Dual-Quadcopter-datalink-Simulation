"""Simple command-side actuator fault and mismatch model for simulation."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ActuatorFaultConfig:
    motor_speed_scale: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    motor_speed_limit_scale: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    stuck_motor_index: int | None = None
    stuck_motor_speed_rad_s: float = 0.0
    command_delay_steps: int = 0

    def __post_init__(self) -> None:
        for name in ("motor_speed_scale", "motor_speed_limit_scale"):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.shape != (4,) or not np.all(np.isfinite(values)) or np.any(values < 0.0):
                raise ValueError(f"{name} must contain four finite non-negative values")
        if self.stuck_motor_index is not None and self.stuck_motor_index not in range(4):
            raise ValueError("stuck_motor_index must be 0..3 or None")
        if self.stuck_motor_speed_rad_s < 0.0 or self.command_delay_steps < 0:
            raise ValueError("stuck speed and command delay must be non-negative")


class MotorCommandFaultModel:
    def __init__(self, config: ActuatorFaultConfig | None = None):
        self.config = config or ActuatorFaultConfig()
        self._queue: deque[np.ndarray] = deque()

    def apply(self, motor_speed_commands_rad_s: np.ndarray, maximum_speed_rad_s: float) -> tuple[np.ndarray, bool]:
        commands = np.asarray(motor_speed_commands_rad_s, dtype=float)
        if commands.shape != (4,) or not np.all(np.isfinite(commands)):
            raise ValueError("exactly four finite motor speed commands are required")
        if not np.isfinite(maximum_speed_rad_s) or maximum_speed_rad_s <= 0.0:
            raise ValueError("maximum motor speed must be finite and positive")
        self._queue.append(commands.copy())
        if len(self._queue) > self.config.command_delay_steps:
            delayed = self._queue.popleft()
        else:
            delayed = self._queue[0]
        result = delayed * np.asarray(self.config.motor_speed_scale)
        limits = maximum_speed_rad_s * np.asarray(self.config.motor_speed_limit_scale)
        result = np.minimum(np.maximum(result, 0.0), limits)
        if self.config.stuck_motor_index is not None:
            result[self.config.stuck_motor_index] = self.config.stuck_motor_speed_rad_s
        changed = not np.array_equal(result, commands)
        return result, bool(changed)


def demonstration_mismatch_profile() -> ActuatorFaultConfig:
    """Illustrative 2% per-motor command mismatch; not a hardware measurement."""
    return ActuatorFaultConfig(motor_speed_scale=(1.0, 0.98, 1.02, 0.99))
