"""Rigid-body 6-DoF quadrotor plant with first-order motor dynamics.

Frames: world is NED (north, east, down); body is FRD (forward, right,
down). Euler angles are roll, pitch, yaw in radians and use the ZYX rotation
sequence. Body rates p, q, r are radians per second. Positive collective
thrust acts along body -Z, opposing NED gravity when level.

The default coefficients are illustrative placeholders, not measured data for
a particular airframe. Replace them with identified motor, propeller, mass,
and inertia parameters before drawing performance conclusions.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class QuadrotorConfig:
    mass_kg: float = 1.5
    arm_length_m: float = 0.18
    thrust_coefficient_n_per_rad_s2: float = 1.8e-5
    yaw_moment_coefficient_nm_per_rad_s2: float = 2.5e-7
    inertia_diagonal_kg_m2: tuple[float, float, float] = (0.025, 0.025, 0.045)
    motor_time_constant_up_s: float = 0.04
    motor_time_constant_down_s: float = 0.06
    motor_time_constant_up_scale: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    motor_time_constant_down_scale: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    max_motor_speed_rad_s: float = 1000.0
    linear_body_drag_n_per_m_s: float = 0.12
    gravity_m_s2: float = 9.80665

    def __post_init__(self) -> None:
        positive = (
            self.mass_kg,
            self.arm_length_m,
            self.thrust_coefficient_n_per_rad_s2,
            self.yaw_moment_coefficient_nm_per_rad_s2,
            self.motor_time_constant_up_s,
            self.motor_time_constant_down_s,
            self.max_motor_speed_rad_s,
            self.gravity_m_s2,
        )
        if any(not math.isfinite(x) or x <= 0 for x in positive):
            raise ValueError("Mass, geometry, coefficients, motor time constants, speed, and gravity must be positive and finite.")
        if len(self.inertia_diagonal_kg_m2) != 3 or any(
            not math.isfinite(x) or x <= 0 for x in self.inertia_diagonal_kg_m2
        ):
            raise ValueError("The three principal moments of inertia must be positive and finite.")
        for name in ("motor_time_constant_up_scale", "motor_time_constant_down_scale"):
            values = getattr(self, name)
            if len(values) != 4 or any(not math.isfinite(x) or x <= 0.0 for x in values):
                raise ValueError(f"{name} must contain four positive finite motor scales")
        if not math.isfinite(self.linear_body_drag_n_per_m_s) or self.linear_body_drag_n_per_m_s < 0:
            raise ValueError("Body drag must be finite and non-negative.")

    @property
    def inertia_diagonal(self) -> np.ndarray:
        return np.asarray(self.inertia_diagonal_kg_m2, dtype=float)

    @property
    def rotor_positions_body_m(self) -> np.ndarray:
        """Motor centers in the order front-left, front-right, rear-right, rear-left."""
        diagonal = self.arm_length_m / math.sqrt(2.0)
        return np.asarray(
            ((diagonal, -diagonal, 0.0),
             (diagonal, diagonal, 0.0),
             (-diagonal, diagonal, 0.0),
             (-diagonal, -diagonal, 0.0)),
            dtype=float,
        )

    @property
    def rotor_reaction_yaw_signs(self) -> np.ndarray:
        """Signed reaction torque about body +Z for motors 1..4."""
        return np.asarray((1.0, -1.0, 1.0, -1.0), dtype=float)

    @property
    def allocation_matrix(self) -> np.ndarray:
        """Map squared motor speeds to [thrust, tau_x, tau_y, tau_z]."""
        positions = self.rotor_positions_body_m
        kf = self.thrust_coefficient_n_per_rad_s2
        km = self.yaw_moment_coefficient_nm_per_rad_s2
        return np.vstack((
            np.full(4, kf),
            -positions[:, 1] * kf,
            positions[:, 0] * kf,
            self.rotor_reaction_yaw_signs * km,
        ))


@dataclass
class QuadrotorState:
    """Simulation state in NED/FRD coordinates; angles and rates are radians."""

    position_ned_m: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    velocity_ned_m_s: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    attitude_rpy_rad: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    body_rates_pqr_rad_s: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    motor_speeds_rad_s: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
    time_s: float = 0.0

    def copy(self) -> "QuadrotorState":
        return QuadrotorState(
            self.position_ned_m.copy(),
            self.velocity_ned_m_s.copy(),
            self.attitude_rpy_rad.copy(),
            self.body_rates_pqr_rad_s.copy(),
            self.motor_speeds_rad_s.copy(),
            self.time_s,
        )

    def pack(self) -> np.ndarray:
        return np.concatenate((
            self.position_ned_m,
            self.velocity_ned_m_s,
            self.attitude_rpy_rad,
            self.body_rates_pqr_rad_s,
            self.motor_speeds_rad_s,
        )).astype(float, copy=True)

    @classmethod
    def unpack(cls, values: np.ndarray, time_s: float) -> "QuadrotorState":
        return cls(
            position_ned_m=values[0:3].copy(),
            velocity_ned_m_s=values[3:6].copy(),
            attitude_rpy_rad=values[6:9].copy(),
            body_rates_pqr_rad_s=values[9:12].copy(),
            motor_speeds_rad_s=values[12:16].copy(),
            time_s=time_s,
        )


def rotation_body_to_ned(attitude_rpy_rad: Sequence[float]) -> np.ndarray:
    """Return the body-FRD to world-NED direction cosine matrix (ZYX)."""
    roll, pitch, yaw = np.asarray(attitude_rpy_rad, dtype=float)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray((
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    ), dtype=float)


def euler_rate_from_body_rates(attitude_rpy_rad: Sequence[float], pqr_rad_s: Sequence[float]) -> np.ndarray:
    """Convert body angular rates p,q,r to roll, pitch, yaw derivatives."""
    roll, pitch, _ = np.asarray(attitude_rpy_rad, dtype=float)
    p, q, r = np.asarray(pqr_rad_s, dtype=float)
    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) < 1e-5:
        raise ValueError("Euler-angle kinematics are singular near pitch = +/-90 degrees.")
    sin_roll, cos_roll = math.sin(roll), math.cos(roll)
    return np.asarray((
        p + sin_roll * math.tan(pitch) * q + cos_roll * math.tan(pitch) * r,
        cos_roll * q - sin_roll * r,
        sin_roll / cos_pitch * q + cos_roll / cos_pitch * r,
    ), dtype=float)


def allocate_wrench_to_motor_speeds(
    config: QuadrotorConfig,
    collective_thrust_n: float,
    body_torque_nm: Sequence[float],
) -> tuple[np.ndarray, bool]:
    """Allocate a requested wrench to motor speeds, clipping unreachable requests.

    Returns the four commanded angular speeds and whether any motor saturated.
    The wrench is [upward collective thrust, body roll torque, pitch torque,
    yaw torque]. Saturation is independent per motor, so the achieved wrench
    can differ from the requested one when the request is infeasible.
    """
    requested = np.concatenate(((float(collective_thrust_n),), np.asarray(body_torque_nm, dtype=float)))
    if requested.shape != (4,) or not np.all(np.isfinite(requested)):
        raise ValueError("A finite collective thrust and three finite body torques are required.")
    try:
        omega_squared = np.linalg.solve(config.allocation_matrix, requested)
    except np.linalg.LinAlgError as exc:
        raise ValueError("The rotor allocation matrix is singular.") from exc
    maximum_squared = config.max_motor_speed_rad_s ** 2
    saturated = bool(np.any((omega_squared < 0.0) | (omega_squared > maximum_squared)))
    omega_squared = np.clip(omega_squared, 0.0, maximum_squared)
    return np.sqrt(omega_squared), saturated


class Quadrotor6DOF:
    """Nonlinear 6-DoF rigid-body model with four first-order rotor states."""

    def __init__(self, config: QuadrotorConfig | None = None, state: QuadrotorState | None = None):
        self.config = config or QuadrotorConfig()
        self.state = state.copy() if state is not None else QuadrotorState()
        if self.state.position_ned_m.shape != (3,) or self.state.velocity_ned_m_s.shape != (3,):
            raise ValueError("Position and velocity states must each have three components.")
        if self.state.attitude_rpy_rad.shape != (3,) or self.state.body_rates_pqr_rad_s.shape != (3,):
            raise ValueError("Attitude and body-rate states must each have three components.")
        if self.state.motor_speeds_rad_s.shape != (4,):
            raise ValueError("The quadrotor must have four motor-speed states.")

    def rotor_wrench(self, motor_speeds_rad_s: Sequence[float] | None = None) -> tuple[float, np.ndarray]:
        speeds = self.state.motor_speeds_rad_s if motor_speeds_rad_s is None else np.asarray(motor_speeds_rad_s, dtype=float)
        if speeds.shape != (4,):
            raise ValueError("Exactly four motor speeds are required.")
        omega_squared = np.square(np.maximum(speeds, 0.0))
        thrusts = self.config.thrust_coefficient_n_per_rad_s2 * omega_squared
        total_thrust = float(np.sum(thrusts))
        moments = np.zeros(3, dtype=float)
        for position, thrust, yaw_sign in zip(
            self.config.rotor_positions_body_m,
            thrusts,
            self.config.rotor_reaction_yaw_signs,
        ):
            force_body = np.asarray((0.0, 0.0, -thrust))
            moments += np.cross(position, force_body)
            moments[2] += yaw_sign * self.config.yaw_moment_coefficient_nm_per_rad_s2 * (thrust / self.config.thrust_coefficient_n_per_rad_s2)
        return total_thrust, moments

    def specific_force_body(self) -> np.ndarray:
        """Return ideal accelerometer specific force in body FRD axes (m/s^2).

        This is non-gravitational force per unit mass. A level vehicle in
        steady hover therefore reports approximately (0, 0, -g), consistent
        with the simulator's body-down-positive convention.
        """
        rotation = rotation_body_to_ned(self.state.attitude_rpy_rad)
        total_thrust, _ = self.rotor_wrench()
        velocity_body = rotation.T @ self.state.velocity_ned_m_s
        drag_body = -self.config.linear_body_drag_n_per_m_s * velocity_body
        return np.asarray((0.0, 0.0, -total_thrust)) / self.config.mass_kg + drag_body / self.config.mass_kg

    def _derivative(self, packed_state: np.ndarray, motor_commands: np.ndarray) -> np.ndarray:
        state = QuadrotorState.unpack(packed_state, self.state.time_s)
        config = self.config
        rotation = rotation_body_to_ned(state.attitude_rpy_rad)
        total_thrust, torque_body = self.rotor_wrench(state.motor_speeds_rad_s)
        force_body = np.asarray((0.0, 0.0, -total_thrust))

        velocity_body = rotation.T @ state.velocity_ned_m_s
        drag_body = -config.linear_body_drag_n_per_m_s * velocity_body
        acceleration_ned = (
            rotation @ (force_body + drag_body) / config.mass_kg
            + np.asarray((0.0, 0.0, config.gravity_m_s2))
        )

        inertia = config.inertia_diagonal
        angular_momentum = inertia * state.body_rates_pqr_rad_s
        angular_acceleration = (torque_body - np.cross(state.body_rates_pqr_rad_s, angular_momentum)) / inertia
        attitude_rate = euler_rate_from_body_rates(state.attitude_rpy_rad, state.body_rates_pqr_rad_s)

        speed_error = motor_commands - state.motor_speeds_rad_s
        time_constants = np.where(
            speed_error >= 0.0,
            config.motor_time_constant_up_s * np.asarray(config.motor_time_constant_up_scale),
            config.motor_time_constant_down_s * np.asarray(config.motor_time_constant_down_scale),
        )
        motor_acceleration = speed_error / time_constants
        return np.concatenate((
            state.velocity_ned_m_s,
            acceleration_ned,
            attitude_rate,
            angular_acceleration,
            motor_acceleration,
        ))

    def step(self, motor_speed_commands_rad_s: Sequence[float], dt_s: float) -> QuadrotorState:
        """Advance one RK4 step and return a copy of the updated state."""
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be a positive finite value.")
        commands = np.asarray(motor_speed_commands_rad_s, dtype=float)
        if commands.shape != (4,) or not np.all(np.isfinite(commands)):
            raise ValueError("Four finite motor-speed commands are required.")
        commands = np.clip(commands, 0.0, self.config.max_motor_speed_rad_s)
        x = self.state.pack()
        k1 = self._derivative(x, commands)
        k2 = self._derivative(x + 0.5 * dt_s * k1, commands)
        k3 = self._derivative(x + 0.5 * dt_s * k2, commands)
        k4 = self._derivative(x + dt_s * k3, commands)
        updated = x + (dt_s / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        updated[12:16] = np.clip(updated[12:16], 0.0, self.config.max_motor_speed_rad_s)
        self.state = QuadrotorState.unpack(updated, self.state.time_s + dt_s)
        return self.state.copy()
