import numpy as np

from cartpole_model import CartPoleParams

from stepper_actuator import (
    StepperActuatorParams,
    build_stepper_command,
)


plant_params = CartPoleParams(
    cart_mass_kg=0.166,
    pendulum_mass_kg=0.077,
    pendulum_length_m=0.305196875,
    center_of_mass_length_m=0.150196875,
    pivot_inertia_kg_m2=0.00236594,
    cart_friction_n_s_m=0.0,
    pivot_friction_n_m_s_rad=0.0,
    gravity_m_s2=9.81,
)


actuator_params = StepperActuatorParams()


state = np.array(
    [
        0.0,
        0.0,
        np.deg2rad(5.0),
        0.0,
    ]
)


requested_force_n = 14.5

current_motor_velocity = 0.0

dt_seconds = 0.001


command = build_stepper_command(
    state=state,

    requested_force_n=requested_force_n,

    current_target_motor_velocity_rad_s=
        current_motor_velocity,

    dt_seconds=dt_seconds,

    plant_params=plant_params,

    actuator_params=actuator_params,
)


print(
    "Meters per motor radian:"
)

print(
    actuator_params.meters_per_motor_rad
)


print(
    "\nMaximum cart velocity:"
)

print(
    actuator_params.max_cart_velocity_m_s
)


print(
    "\nMaximum cart acceleration:"
)

print(
    actuator_params.max_cart_acceleration_m_s2
)


print(
    "\nRequested force:"
)

print(
    command.requested_force_n
)


print(
    "\nIdeal cart acceleration:"
)

print(
    command.ideal_cart_acceleration_m_s2
)


print(
    "\nRequested motor acceleration:"
)

print(
    command.requested_motor_acceleration_rad_s2
)


print(
    "\nApplied motor acceleration:"
)

print(
    command.applied_motor_acceleration_rad_s2
)


print(
    "\nNext target motor velocity:"
)

print(
    command.target_motor_velocity_rad_s
)


print(
    "\nTarget STEP frequency:"
)

print(
    command.target_step_frequency_hz
)