import numpy as np

from scipy.linalg import (
    solve_continuous_are,
    eigvals,
)

from cartpole_model import (
    CartPoleParams,
    linearized_state_space,
)


# Temporary parameters
#
# same temps as in check_controllability and simulate_nonlinear

params = CartPoleParams(
    cart_mass_kg=0.50,
    pendulum_mass_kg=0.20,
    pendulum_length_m=0.30,

    cart_friction_n_s_m=0.0,
    pivot_friction_n_m_s_rad=0.0,
)


A, B = linearized_state_space(
    params
)


# Bryson-rule tuning
# LARGEST deviations/control efforts allowable.

max_cart_position_m = 0.05
max_cart_velocity_m_s = 0.50

max_pendulum_angle_rad = 0.05
max_pendulum_angular_velocity_rad_s = 1.00

# Temporary ideal actuator force limit
max_force_n = 5.0


# Construct Q and R

Q = np.diag(
    [
        1.0 / max_cart_position_m**2,
        1.0 / max_cart_velocity_m_s**2,
        1.0 / max_pendulum_angle_rad**2,
        1.0 / max_pendulum_angular_velocity_rad_s**2,
    ]
)


R = np.array(
    [
        [
            1.0 / max_force_n**2
        ]
    ]
)


# continuous time Algebraic Riccati Equation:
# A^T P + P A - P B R^-1 B^T P + Q = 0

P = solve_continuous_are(
    A,
    B,
    Q,
    R,
)


# LQR gain:
# K = R^-1 B^T P

K = np.linalg.solve(
    R, B.T @ P,
)


# Closed loop dynamics:
# x_dot = (A - B K) x

A_closed_loop = (A - B @ K)


closed_loop_eigenvalues = eigvals(A_closed_loop)


# verify Riccati solution numerically

R_inverse = np.linalg.inv(R)
riccati_residual = (A.T @ P + P @ A - P @ B @ R_inverse @ B.T @ P + Q)


riccati_residual_norm = np.linalg.norm(riccati_residual)


# results

np.set_printoptions(
    precision=6,
    suppress=True,
)


print("Q =")
print(Q)
print("\nR =")
print(R)
print("\nP =")
print(P)
print("\nLQR gain K =")
print(K)
print("\nClosed-loop eigenvalues of A - BK =")
print(closed_loop_eigenvalues)

print(
    "\nRiccati residual norm = "
    f"{riccati_residual_norm:.3e}"
)

stable = np.all(
    np.real(closed_loop_eigenvalues) < 0.0
)


if stable:
    print(
        "\nPASS: closed-loop linear system "
        "is asymptotically stable."
    )
else:
    print(
        "\nFAIL: at least one closed-loop "
        "eigenvalue is not stable."
    )