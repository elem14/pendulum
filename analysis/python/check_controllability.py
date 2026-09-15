# controllability matrix
# C = [B, AB, A^2B, A^3B] 
# VERIFY rank(C) = 4
# if true then sys is completely controllable
# rank should be 4, if not something has gone very wrong

import numpy as np

from cartpole_model import (
    CartPoleParams,
    linearized_state_space,
)

#measured params

params = CartPoleParams(
    cart_mass_kg=0.166,
    pendulum_mass_kg=0.077,
    pendulum_length_m=0.305196875,
    center_of_mass_length_m=0.150196875,
    pivot_inertia_kg_m2=0.00236594,
    cart_friction_n_s_m=0.0,
    pivot_friction_n_m_s_rad=0.000332,
    gravity_m_s2=9.81,
)

A, B = linearized_state_space(
    params
)


AB = A @ B
A2B = A @ AB
A3B = A @ A2B


controllability_matrix = np.hstack(
    [
        B,
        AB,
        A2B,
        A3B,
    ]
)

rank = np.linalg.matrix_rank(
    controllability_matrix
)

print("A =")
print(A)

print("\nB =")
print(B)

print("\nControllability matrix C =")
print(controllability_matrix)

print(
    f"\nrank(C) = {rank}"
)


if rank == 4:
    print(
        "PASS: linearized system is controllable."
    )
else:
    print(
        "FAIL: linearized system is NOT fully controllable."
    )