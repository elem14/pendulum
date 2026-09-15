import numpy as np
import matplotlib.pyplot as plt

from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

from cartpole_model import (
    CartPoleParams,
    nonlinear_dynamics,
    linearized_state_space,
)


# temp params

params = CartPoleParams(
    cart_mass_kg=0.166,
    pendulum_mass_kg=0.077,
    pendulum_length_m=0.305196875,
    center_of_mass_length_m=0.150196875,
    pivot_inertia_kg_m2=0.00236594,
    cart_friction_n_s_m=0.0,
    pivot_friction_n_m_s_rad=0.0,
    gravity_m_s2=9.81,
)


# lin model for lqr design

A, B = linearized_state_space(params)


# Same Q and R from design_lqr

Q = np.diag(
    [
        400.0,   # cart position
        4.0,     # cart velocity
        400.0,   # pendulum angle
        1.0,     # angular velocity
    ]
)

R = np.array(
    [
        [0.04]
    ]
)


# Solve CARE and get K

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


# force saturation
# leave false on first run
# change to true for second run, test temporary
# +/-5 N actuator limit
# True means |u| <= 5 N, false means dont saturate
USE_FORCE_SATURATION = True

MAX_FORCE_N = 5.0


def lqr_control(state: np.ndarray,) -> float:

    # u = -Kx
    force_n = -(K @ state).item()

    if USE_FORCE_SATURATION:

        force_n = np.clip(
            force_n,
            -MAX_FORCE_N,
            MAX_FORCE_N,
        )

    return float(force_n)


# Closed-loop nonlinear dynamics

def closed_loop_dynamics(
    t: float,
    state: np.ndarray,
) -> np.ndarray:

    force_n = lqr_control(state)

    return nonlinear_dynamics(
        t=t,
        state=state,
        params=params,
        input_force_n=force_n,
    )


# initial condition
# pendulum starts 5 degrees away from upright
# everything else starts at rest

initial_state = np.array(
    [
        0.0,                # cart position
        0.0,                # cart velocity
        np.deg2rad(5.0),    # pendulum angle
        0.0,                # angular velocity
    ]
)

#simulate

solution = solve_ivp(
    fun=closed_loop_dynamics,

    t_span=(
        0.0,
        5.0,
    ),

    y0=initial_state,

    method="RK45",

    rtol=1.0e-9,
    atol=1.0e-11,

    max_step=0.001,
)


if not solution.success:
    raise RuntimeError(
        solution.message
    )


time = solution.t

cart_position = solution.y[0]
cart_velocity = solution.y[1]

pendulum_angle = solution.y[2]
pendulum_velocity = solution.y[3]


# reconstruct control force through time

control_force = np.array(
    [
        lqr_control(
            solution.y[:, i]
        )
        for i in range(
            solution.y.shape[1]
        )
    ]
)


# diagnostics

print("\nInitial state:")

print(initial_state)

print("\nFinal state:")

print(solution.y[:, -1])

print(
    "\nMaximum absolute control force: "
    f"{np.max(np.abs(control_force)):.3f} N"
)

print(
    "Maximum absolute cart displacement: "
    f"{np.max(np.abs(cart_position)):.4f} m"
)

print(
    "Maximum absolute pendulum angle: "
    f"{np.rad2deg(np.max(np.abs(pendulum_angle))):.3f} deg"
)


# state plots

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


# Control-force plot

plt.figure(
    figsize=(9, 4)
)

plt.plot(
    time,
    control_force,
)

plt.axhline(
    0.0,
    linewidth=0.8,
)

if USE_FORCE_SATURATION:

    plt.axhline(
        MAX_FORCE_N,
        linestyle="--",
    )

    plt.axhline(
        -MAX_FORCE_N,
        linestyle="--",
    )


plt.title(
    "LQR control force"
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "u [N]"
)

plt.tight_layout()

plt.show()