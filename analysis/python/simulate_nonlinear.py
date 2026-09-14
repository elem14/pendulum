import numpy as np
import matplotlib.pyplot as plt

from scipy.integrate import solve_ivp

from cartpole_model import (
    CartPoleParams,
    nonlinear_dynamics,
    mechanical_energy,
)


# TEMPORARY parameters
#
# still need to measure and replace with real values

params = CartPoleParams(
    cart_mass_kg=0.50,
    pendulum_mass_kg=0.20,
    pendulum_length_m=0.30,

    cart_friction_n_s_m=0.0,
    pivot_friction_n_m_s_rad=0.0,
)


# initial state
#
# theta = 0 is upright
#
# Start 5 degrees away from upright at rest

initial_state = np.array(
    [
        0.0,                # x
        0.0,                # x_dot
        np.pi,              # theta
        0.0,                # theta_dot
    ]
)


# zero input force for now

def dynamics(t, state):

    return nonlinear_dynamics(
        t=t,
        state=state,
        params=params,
        input_force_n=0.0,
    )


# Simulation interval

start_time_s = 0.0
end_time_s = 5.0


solution = solve_ivp(
    fun=dynamics,

    t_span=(
        start_time_s,
        end_time_s,
    ),

    y0=initial_state,

    method="RK45",

    rtol=1.0e-9,
    atol=1.0e-11,

    max_step=0.002,
)


if not solution.success:
    raise RuntimeError(solution.message)


time = solution.t

cart_position = solution.y[0]
cart_velocity = solution.y[1]

pendulum_angle = solution.y[2]
pendulum_velocity = solution.y[3]


# computes mechanical energy thru the trajectory

energies = np.array(
    [
        mechanical_energy(
            solution.y[:, i],
            params,
        )
        for i in range(solution.y.shape[1])
    ]
)


initial_energy = energies[0]

relative_energy_error = (
    (energies - initial_energy) / max(abs(initial_energy), 1.0e-12,)
)

maximum_energy_error = np.max(
    np.abs(relative_energy_error)
)


print("Simulation successful.")

print(f"Integration points: {len(time)}")

print("Maximum relative energy drift: " f"{maximum_energy_error:.3e}")


# State plots

fig, axes = plt.subplots(2, 2, figsize=(11, 7),)

axes[0, 0].plot(time, cart_position,)

axes[0, 0].set_title("Cart position")

axes[0, 0].set_xlabel("Time [s]")

axes[0, 0].set_ylabel("x [m]")

axes[0, 1].plot(time, cart_velocity,)

axes[0, 1].set_title("Cart velocity")

axes[0, 1].set_xlabel("Time [s]")

axes[0, 1].set_ylabel("x_dot [m/s]")

axes[1, 0].plot(time, pendulum_angle,)

axes[1, 0].set_title("Pendulum angle")

axes[1, 0].set_xlabel("Time [s]")

axes[1, 0].set_ylabel("theta [rad]")

axes[1, 1].plot(time, pendulum_velocity,)

axes[1, 1].set_title("Pendulum angular velocity")

axes[1, 1].set_xlabel("Time [s]")

axes[1, 1].set_ylabel("theta_dot [rad/s]")

fig.tight_layout()


# Energy error plot

plt.figure(figsize=(9, 4))

plt.plot(time, relative_energy_error,)

plt.xlabel("Time [s]")

plt.ylabel("Relative energy error")

plt.title("Energy-conservation sanity check")

plt.tight_layout()

plt.show()