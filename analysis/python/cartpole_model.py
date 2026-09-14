# all derived from LQR latex doc
# physics only doc

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CartPoleParams:
    cart_mass_kg: float
    pendulum_mass_kg: float
    pendulum_length_m: float

    cart_friction_n_s_m: float
    pivot_friction_n_m_s_rad: float

    gravity_m_s2: float = 9.81

    @property
    def center_of_mass_length_m(self) -> float:
        return self.pendulum_length_m / 2.0

    @property
    def pivot_inertia_kg_m2(self) -> float:
        """
        Moment of inertia of a uniform rod about its pivot.

        I_p = (1/3) m_p L^2
        """
        return ((1.0 / 3.0) * self.pendulum_mass_kg * self.pendulum_length_m**2)


def nonlinear_dynamics(
    t: float,
    state: np.ndarray,
    params: CartPoleParams,
    input_force_n: float = 0.0,
) -> np.ndarray:

    x = state[0]
    x_dot = state[1]
    theta = state[2]
    theta_dot = state[3]

    # x doesn't appear in equations but needs to stay in the state vector
    _ = x
    _ = t

    m_c = params.cart_mass_kg
    m_p = params.pendulum_mass_kg

    l_c = params.center_of_mass_length_m
    I_p = params.pivot_inertia_kg_m2

    b_x = params.cart_friction_n_s_m
    b_theta = params.pivot_friction_n_m_s_rad

    g = params.gravity_m_s2

    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)


    # Equation 6:
    # D(theta) = (m_c + m_p) I_p - m_p^2 l_c^2 cos^2(theta)

    denominator = ((m_c + m_p) * I_p - (m_p**2) * (l_c**2) * (cos_theta**2))


    # Common pieces appearing in both equations

    cart_term = (input_force_n - b_x * x_dot + m_p * l_c * sin_theta * theta_dot**2)
    pendulum_term = (m_p * g * l_c * sin_theta - b_theta * theta_dot)


    # Equation 7: cart acceleration

    x_ddot = (I_p * cart_term - m_p * l_c * cos_theta * pendulum_term) / denominator


    # Equation (8): pendulum angular acceleration

    theta_ddot = ((m_c + m_p) * pendulum_term - m_p * l_c * cos_theta * cart_term) / denominator


    # State derivative:
    # d/dt [x, x_dot, theta, theta_dot] = [x_dot, x_ddot, theta_dot, theta_ddot]
    return np.array(
        [
            x_dot,
            x_ddot,
            theta_dot,
            theta_ddot,
        ]
    )


def mechanical_energy(
    state: np.ndarray,
    params: CartPoleParams,
) -> float:

    x_dot = state[1]
    theta = state[2]
    theta_dot = state[3]

    m_c = params.cart_mass_kg
    m_p = params.pendulum_mass_kg

    l_c = params.center_of_mass_length_m
    I_p = params.pivot_inertia_kg_m2

    g = params.gravity_m_s2


    # Kinetic energy from Equation 1
    kinetic_energy = (0.5 * (m_c + m_p) * x_dot**2 + m_p * l_c 
        * np.cos(theta) * x_dot 
        * theta_dot + 0.5 * I_p * theta_dot**2)


    # Potential energy from Equation 2
    potential_energy = (m_p * g * l_c * np.cos(theta))


    return kinetic_energy + potential_energy

# create and return A B matrices
def linearized_state_space(
    params: CartPoleParams,
) -> tuple[np.ndarray, np.ndarray]:

    m_c = params.cart_mass_kg
    m_p = params.pendulum_mass_kg

    l_c = params.center_of_mass_length_m
    I_p = params.pivot_inertia_kg_m2

    b_x = params.cart_friction_n_s_m
    b_theta = params.pivot_friction_n_m_s_rad

    g = params.gravity_m_s2

    D_0 = ((m_c + m_p) * I_p - (m_p**2) * (l_c**2))

    A = np.array(
        [
            [
                0.0,
                1.0,
                0.0,
                0.0,
            ],

            [
                0.0,
                -(I_p * b_x) / D_0,
                -(m_p**2 * g * l_c**2) / D_0,
                (m_p * l_c * b_theta) / D_0,
            ],

            [
                0.0,
                0.0,
                0.0,
                1.0,
            ],

            [
                0.0,
                (m_p * l_c * b_x) / D_0,
                ((m_c + m_p) * m_p * g * l_c) / D_0,
                -((m_c + m_p) * b_theta) / D_0,
            ],
        ],
        dtype=float,
    )


    B = np.aray(
        [
            [0.0],
            [I_p / D_0],
            [0.0],
            [-m_p * l_c / D_0],
        ],
        dtype=float,
    )


    return A, B