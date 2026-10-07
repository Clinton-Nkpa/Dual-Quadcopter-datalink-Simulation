"""Typed controller-facing interfaces, independent of simulator internals."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ControllerState:
    """State values required by the current controller; angles/rates are radians."""

    attitude_rpy_rad: np.ndarray
    body_rates_pqr_rad_s: np.ndarray
    altitude_m: float
    velocity_up_m_s: float

    def __post_init__(self) -> None:
        attitude = np.asarray(self.attitude_rpy_rad, dtype=float).copy()
        rates = np.asarray(self.body_rates_pqr_rad_s, dtype=float).copy()
        if attitude.shape != (3,) or not np.all(np.isfinite(attitude)):
            raise ValueError("attitude_rpy_rad must contain three finite values")
        if rates.shape != (3,) or not np.all(np.isfinite(rates)):
            raise ValueError("body_rates_pqr_rad_s must contain three finite values")
        if not math.isfinite(self.altitude_m) or not math.isfinite(self.velocity_up_m_s):
            raise ValueError("altitude and vertical velocity must be finite")
        attitude.setflags(write=False)
        rates.setflags(write=False)
        object.__setattr__(self, "attitude_rpy_rad", attitude)
        object.__setattr__(self, "body_rates_pqr_rad_s", rates)
        object.__setattr__(self, "altitude_m", float(self.altitude_m))
        object.__setattr__(self, "velocity_up_m_s", float(self.velocity_up_m_s))


@dataclass(frozen=True)
class AttitudeAltitudeSetpoint:
    attitude_rpy_rad: np.ndarray
    altitude_m: float

    def __post_init__(self) -> None:
        attitude = np.asarray(self.attitude_rpy_rad, dtype=float).copy()
        if attitude.shape != (3,) or not np.all(np.isfinite(attitude)) or not math.isfinite(self.altitude_m):
            raise ValueError("setpoint requires three finite angles and a finite altitude")
        attitude.setflags(write=False)
        object.__setattr__(self, "attitude_rpy_rad", attitude)
        object.__setattr__(self, "altitude_m", float(self.altitude_m))


@dataclass(frozen=True)
class MotorCommand:
    motor_speeds_rad_s: np.ndarray
    mixer_saturated: bool

    def __post_init__(self) -> None:
        speeds = np.asarray(self.motor_speeds_rad_s, dtype=float).copy()
        if speeds.shape != (4,) or not np.all(np.isfinite(speeds)) or np.any(speeds < 0.0):
            raise ValueError("MotorCommand requires four finite non-negative motor speeds")
        speeds.setflags(write=False)
        object.__setattr__(self, "motor_speeds_rad_s", speeds)
        object.__setattr__(self, "mixer_saturated", bool(self.mixer_saturated))


def perfect_state_observation(plant_state: object) -> ControllerState:
    """Explicit ideal-sensor adapter for nominal simulation, not an estimator.

    Kept outside the controller so the true-state dependency is visible and can
    be replaced with sensor models plus an estimator without changing control
    equations. Never use this adapter as evidence of sensor-based validation.
    """
    try:
        return ControllerState(
            attitude_rpy_rad=plant_state.attitude_rpy_rad,
            body_rates_pqr_rad_s=plant_state.body_rates_pqr_rad_s,
            altitude_m=-float(plant_state.position_ned_m[2]),
            velocity_up_m_s=-float(plant_state.velocity_ned_m_s[2]),
        )
    except (AttributeError, IndexError, TypeError) as exc:
        raise ValueError("plant_state does not expose the expected simulation fields") from exc
