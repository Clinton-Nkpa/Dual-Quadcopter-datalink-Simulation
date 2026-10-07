#!/usr/bin/env python3
"""Check software-only motor indexing, rotor signs, allocation, and sensor reproducibility."""

from __future__ import annotations

import numpy as np

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from simulation.actuator_faults import ActuatorFaultConfig, MotorCommandFaultModel
from simulation.quadrotor_6dof import Quadrotor6DOF, QuadrotorConfig, allocate_wrench_to_motor_speeds
from simulation.sensor_models import ImuBarometerModel, SensorModelConfig


def main() -> int:
    config = QuadrotorConfig()
    plant = Quadrotor6DOF(config)
    speeds = 400.0
    diagonal = config.arm_length_m / np.sqrt(2.0)
    expected_signs = ((1, 1, 1), (-1, 1, -1), (-1, -1, 1), (1, -1, -1))
    for motor_index, expected in enumerate(expected_signs):
        motor_speeds = np.zeros(4)
        motor_speeds[motor_index] = speeds
        thrust, torque = plant.rotor_wrench(motor_speeds)
        if thrust <= 0.0 or tuple(np.sign(torque)) != expected:
            raise AssertionError(f"motor {motor_index + 1} produced unexpected model wrench signs: {torque}")
        if not np.isclose(abs(torque[0]), diagonal * config.thrust_coefficient_n_per_rad_s2 * speeds**2):
            raise AssertionError(f"motor {motor_index + 1} roll moment magnitude mismatch")
        if not np.isclose(abs(torque[1]), diagonal * config.thrust_coefficient_n_per_rad_s2 * speeds**2):
            raise AssertionError(f"motor {motor_index + 1} pitch moment magnitude mismatch")

    rng = np.random.default_rng(914)
    for case in range(100):
        original = rng.uniform(0.0, config.max_motor_speed_rad_s, 4)
        thrust, torque = plant.rotor_wrench(original)
        recovered, saturated = allocate_wrench_to_motor_speeds(config, thrust, torque)
        if saturated or not np.allclose(recovered, original, rtol=1e-11, atol=1e-8):
            raise AssertionError(f"allocation round-trip failed in case {case}")
    _, saturated = allocate_wrench_to_motor_speeds(config, config.mass_kg * config.gravity_m_s2, (5.0, -5.0, 2.0))
    if not saturated:
        raise AssertionError("infeasible high-torque allocation was not reported as saturated")

    sensor_cfg = SensorModelConfig(
        gyro_bias_rad_s=(0.01, -0.02, 0.03), gyro_noise_std_rad_s=0.005,
        altitude_noise_std_m=0.02, delay_steps=2, dropout_every_n=5,
    )
    first = ImuBarometerModel(sensor_cfg, seed=77)
    second = ImuBarometerModel(sensor_cfg, seed=77)
    for _ in range(20):
        a = first.sample(plant)
        b = second.sample(plant)
        if a.timestamp_s != b.timestamp_s or a.valid != b.valid or a.altitude_m != b.altitude_m:
            raise AssertionError("sensor model is not reproducible for a fixed seed")
        if not np.array_equal(a.gyro_pqr_rad_s, b.gyro_pqr_rad_s):
            raise AssertionError("seeded gyro samples differ")
        if not np.array_equal(a.accel_specific_force_frd_m_s2, b.accel_specific_force_frd_m_s2):
            raise AssertionError("seeded accelerometer samples differ")

    fault = MotorCommandFaultModel(ActuatorFaultConfig(motor_speed_scale=(1.0, 0.9, 1.0, 1.0)))
    input_commands = np.full(4, 500.0)
    applied, changed = fault.apply(input_commands, config.max_motor_speed_rad_s)
    if not changed or not np.allclose(applied, (500.0, 450.0, 500.0, 500.0)):
        raise AssertionError("motor mismatch profile did not apply the expected software command scale")

    print("Software mapping checks passed: 4 single-motor wrench signs, 100 allocator round-trips, and infeasible-wrench saturation detection.")
    print("Seeded sensor noise/delay/dropout and actuator mismatch hooks are deterministic and active.")
    print("These checks verify simulator conventions only; they do not verify physical motor wiring or spin direction.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
