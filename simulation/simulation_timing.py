"""Deterministic host-side model of controller update jitter and missed ticks."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TimingFaultConfig:
    period_steps: int = 1
    jitter_steps: int = 0
    miss_every_n: int = 0

    def __post_init__(self) -> None:
        if self.period_steps < 1 or self.jitter_steps < 0 or self.miss_every_n < 0:
            raise ValueError("period_steps must be positive; jitter and miss count must be non-negative")


class ControllerUpdateScheduler:
    """Generate controller ticks measured in plant integration steps.

    This models an assumed schedule for sensitivity experiments. It is not a
    measurement or proof of MCU interrupt timing or worst-case execution time.
    """

    def __init__(self, config: TimingFaultConfig | None = None, seed: int = 0):
        self.config = config or TimingFaultConfig()
        self.rng = np.random.default_rng(seed)
        self.next_tick = 0
        self.last_success_step: int | None = None
        self.scheduled_ticks = 0
        self.update_count = 0
        self.missed_count = 0

    def due(self, step_index: int, plant_dt_s: float) -> tuple[bool, float]:
        if step_index < self.next_tick:
            return False, 0.0
        self.scheduled_ticks += 1
        jitter = int(self.rng.integers(-self.config.jitter_steps, self.config.jitter_steps + 1)) if self.config.jitter_steps else 0
        interval = max(1, self.config.period_steps + jitter)
        self.next_tick = step_index + interval
        missed = self.config.miss_every_n > 0 and self.scheduled_ticks % self.config.miss_every_n == 0
        if missed:
            self.missed_count += 1
            return False, 0.0
        elapsed_steps = 1 if self.last_success_step is None else step_index - self.last_success_step
        self.last_success_step = step_index
        self.update_count += 1
        return True, elapsed_steps * plant_dt_s
