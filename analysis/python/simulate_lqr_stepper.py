import numpy as np
import matplotlib.pyplot as plt

from scipy.linalg import solve_continuous_are

from cartpole_model import (
    CartPoleParams,
    linearized_state_space,
    acceleration_driven_dynamics,
)

from stepper_actuator import (
    StepperActuatorParams,
    build_stepper_command,
)


# =================================================
# Physical model
# =================================================

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


# =================================================
# Stepper actuator model
#
# Change the acceleration limit here when testing.
# =================================================

actuator_params = StepperActuatorParams(
    max_motor_velocity_rad_s=750.0,
    max_motor_acceleration_rad_s2=1500.0,
)


# =================================================
# LQR design
# Same controller from Milestone 12.
# =================================================

A, B = linearized_state_space(
    plant_params
)


Q = np.diag(
    [
        400.0,
        4.0,
        400.0,
        1.0,
    ]
)


R = np.array(
    [
        [0.04]
    ]
)


P = solve_continuous_are(
    A,
    B,
    Q,
    R,
)


K = np.linalg.solve(
    R,
    B.T @ P,
)


print("LQR gain K =")
print(K)


# =================================================
# Simulation settings
#
# 1 ms = 1 kHz controller update rate.
#
# Unlike solve_ivp, this fixed-step structure
# resembles what the Pico will actually do.
# =================================================

DT_SECONDS = 0.001

SIMULATION_TIME_SECONDS = 5.0

NUMBER_OF_STEPS = int(
    SIMULATION_TIME_SECONDS
    /
    DT_SECONDS
) + 1


time = np.arange(
    NUMBER_OF_STEPS
) * DT_SECONDS


# =================================================
# Initial state
#
# [x, x_dot, theta, theta_dot]
# =================================================

state = np.array(
    [
        0.0,
        0.0,
        np.deg2rad(5.0),
        0.0,
    ],
    dtype=float,
)


current_motor_velocity_rad_s = 0.0


# =================================================
# Storage
# =================================================

states = np.zeros(
    (
        NUMBER_OF_STEPS,
        4,
    )
)


requested_forces = np.zeros(
    NUMBER_OF_STEPS
)


ideal_cart_accelerations = np.zeros(
    NUMBER_OF_STEPS
)


requested_motor_accelerations = np.zeros(
    NUMBER_OF_STEPS
)


applied_motor_accelerations = np.zeros(
    NUMBER_OF_STEPS
)


motor_velocities = np.zeros(
    NUMBER_OF_STEPS
)


step_frequencies = np.zeros(
    NUMBER_OF_STEPS
)


actual_cart_accelerations = np.zeros(
    NUMBER_OF_STEPS
)


states[0] = state


# =================================================
# Fixed-step RK4
#
# During each 1 ms controller interval we assume
# the stepper holds one cart acceleration command.
# =================================================

def rk4_step(
    current_state,
    cart_acceleration_m_s2,
    dt_seconds,
):

    def dynamics(local_state):

        return acceleration_driven_dynamics(
            t=0.0,
            state=local_state,
            cart_acceleration_m_s2=
                cart_acceleration_m_s2,
            params=plant_params,
        )


    k1 = dynamics(
        current_state
    )

    k2 = dynamics(
        current_state
        +
        0.5
        * dt_seconds
        * k1
    )

    k3 = dynamics(
        current_state
        +
        0.5
        * dt_seconds
        * k2
    )

    k4 = dynamics(
        current_state
        +
        dt_seconds
        * k3
    )


    return (
        current_state
        +
        (
            dt_seconds
            /
            6.0
        )
        *
        (
            k1
            +
            2.0 * k2
            +
            2.0 * k3
            +
            k4
        )
    )


# =================================================
# Closed-loop simulation
# =================================================

for i in range(
    NUMBER_OF_STEPS - 1
):

    # ---------------------------------------------
    # LQR:
    #
    # u = -Kx
    #
    # Still outputs ideal horizontal force [N].
    # ---------------------------------------------

    requested_force_n = -(
        K @ state
    ).item()


    # ---------------------------------------------
    # Hardware bridge:
    #
    # force
    #   ->
    # ideal cart acceleration
    #   ->
    # motor angular acceleration
    #   ->
    # saturation
    #   ->
    # motor velocity
    # ---------------------------------------------

    command = build_stepper_command(
        state=state,

        requested_force_n=
            requested_force_n,

        current_target_motor_velocity_rad_s=
            current_motor_velocity_rad_s,

        dt_seconds=
            DT_SECONDS,

        plant_params=
            plant_params,

        actuator_params=
            actuator_params,
    )


    # ---------------------------------------------
    # What acceleration can the real cart
    # actually receive?
    #
    # x_ddot = k_x * alpha_motor
    # ---------------------------------------------

    actual_cart_acceleration = (
        actuator_params.meters_per_motor_rad
        *
        command.applied_motor_acceleration_rad_s2
    )


    # ---------------------------------------------
    # Apply THAT acceleration to the nonlinear
    # pendulum model.
    # ---------------------------------------------

    state = rk4_step(
        current_state=state,

        cart_acceleration_m_s2=
            actual_cart_acceleration,

        dt_seconds=
            DT_SECONDS,
    )


    current_motor_velocity_rad_s = (
        command.target_motor_velocity_rad_s
    )


    # ---------------------------------------------
    # Save results
    # ---------------------------------------------

    states[i + 1] = state

    requested_forces[i] = (
        command.requested_force_n
    )

    ideal_cart_accelerations[i] = (
        command.ideal_cart_acceleration_m_s2
    )

    requested_motor_accelerations[i] = (
        command.requested_motor_acceleration_rad_s2
    )

    applied_motor_accelerations[i] = (
        command.applied_motor_acceleration_rad_s2
    )

    motor_velocities[i] = (
        command.target_motor_velocity_rad_s
    )

    step_frequencies[i] = (
        command.target_step_frequency_hz
    )

    actual_cart_accelerations[i] = (
        actual_cart_acceleration
    )


# Fill final samples for prettier plots.
requested_forces[-1] = (
    requested_forces[-2]
)

ideal_cart_accelerations[-1] = (
    ideal_cart_accelerations[-2]
)

requested_motor_accelerations[-1] = (
    requested_motor_accelerations[-2]
)

applied_motor_accelerations[-1] = (
    applied_motor_accelerations[-2]
)

motor_velocities[-1] = (
    motor_velocities[-2]
)

step_frequencies[-1] = (
    step_frequencies[-2]
)

actual_cart_accelerations[-1] = (
    actual_cart_accelerations[-2]
)


# =================================================
# Extract states
# =================================================

cart_position = states[:, 0]

cart_velocity = states[:, 1]

pendulum_angle = states[:, 2]

pendulum_velocity = states[:, 3]


# =================================================
# Diagnostics
# =================================================

print(
    "\nFinal state:"
)

print(
    states[-1]
)


print(
    "\nMaximum absolute cart displacement:"
)

print(
    np.max(
        np.abs(
            cart_position
        )
    )
)


print(
    "\nMaximum absolute pendulum angle [deg]:"
)

print(
    np.rad2deg(
        np.max(
            np.abs(
                pendulum_angle
            )
        )
    )
)


print(
    "\nMaximum requested LQR force [N]:"
)

print(
    np.max(
        np.abs(
            requested_forces
        )
    )
)


print(
    "\nMaximum requested motor acceleration [rad/s^2]:"
)

print(
    np.max(
        np.abs(
            requested_motor_accelerations
        )
    )
)


print(
    "\nMaximum APPLIED motor acceleration [rad/s^2]:"
)

print(
    np.max(
        np.abs(
            applied_motor_accelerations
        )
    )
)


print(
    "\nMaximum motor velocity [rad/s]:"
)

print(
    np.max(
        np.abs(
            motor_velocities
        )
    )
)


print(
    "\nMaximum STEP frequency [Hz]:"
)

print(
    np.max(
        step_frequencies
    )
)


acceleration_saturated = (
    np.abs(
        requested_motor_accelerations
    )
    >
    actuator_params.max_motor_acceleration_rad_s2
)


percent_acceleration_saturated = (
    100.0
    *
    np.mean(
        acceleration_saturated
    )
)


print(
    "\nPercent of control ticks "
    "acceleration-saturated:"
)

print(
    percent_acceleration_saturated
)


# =================================================
# State plots
# =================================================

fig, axes = plt.subplots(
    2,
    2,
    figsize=(11, 7),
)


axes[0, 0].plot(
    time,
    cart_position,
)

axes[0, 0].axhline(
    0.0,
    linewidth=0.8,
)

axes[0, 0].set_title(
    "Cart position"
)

axes[0, 0].set_xlabel(
    "Time [s]"
)

axes[0, 0].set_ylabel(
    "x [m]"
)


axes[0, 1].plot(
    time,
    cart_velocity,
)

axes[0, 1].axhline(
    0.0,
    linewidth=0.8,
)

axes[0, 1].set_title(
    "Cart velocity"
)

axes[0, 1].set_xlabel(
    "Time [s]"
)

axes[0, 1].set_ylabel(
    "x_dot [m/s]"
)


axes[1, 0].plot(
    time,
    np.rad2deg(
        pendulum_angle
    ),
)

axes[1, 0].axhline(
    0.0,
    linewidth=0.8,
)

axes[1, 0].set_title(
    "Pendulum angle"
)

axes[1, 0].set_xlabel(
    "Time [s]"
)

axes[1, 0].set_ylabel(
    "theta [deg]"
)


axes[1, 1].plot(
    time,
    pendulum_velocity,
)

axes[1, 1].axhline(
    0.0,
    linewidth=0.8,
)

axes[1, 1].set_title(
    "Pendulum angular velocity"
)

axes[1, 1].set_xlabel(
    "Time [s]"
)

axes[1, 1].set_ylabel(
    "theta_dot [rad/s]"
)


fig.tight_layout()


# =================================================
# Motor acceleration plot
# =================================================

plt.figure(
    figsize=(10, 5)
)


plt.plot(
    time,
    requested_motor_accelerations,
    label="Requested motor acceleration",
)


plt.plot(
    time,
    applied_motor_accelerations,
    label="Applied motor acceleration",
)


plt.axhline(
    actuator_params.max_motor_acceleration_rad_s2,
    linestyle="--",
)

plt.axhline(
    -actuator_params.max_motor_acceleration_rad_s2,
    linestyle="--",
)


plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Motor acceleration [rad/s^2]"
)

plt.title(
    "LQR demand vs stepper acceleration capability"
)

plt.legend()

plt.tight_layout()


# =================================================
# Motor velocity plot
# =================================================

plt.figure(
    figsize=(10, 5)
)


plt.plot(
    time,
    motor_velocities,
)


plt.axhline(
    actuator_params.max_motor_velocity_rad_s,
    linestyle="--",
)

plt.axhline(
    -actuator_params.max_motor_velocity_rad_s,
    linestyle="--",
)


plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Motor velocity [rad/s]"
)

plt.title(
    "Stepper velocity command"
)

plt.tight_layout()


plt.show()