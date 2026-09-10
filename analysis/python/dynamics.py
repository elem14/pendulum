#nonlinear EOM
import numpy as np
from params import Params

def D(theta, p: Params) -> float:
    return (p.m_c + p.m_p) * p.I_p - p.m_p ** 2 * p.l_c ** 2 * np.cos(theta) ** 2

def f(state, u, p: Params):
    """state = [x, x_dot, theta, theta_dot]; theta=0 is upright."""
    x, x_dot, theta, theta_dot = state
    s, c = np.sin(theta), np.cos(theta)

    rhs1 = u - p.b_x * x_dot + p.m_p * p.l_c * s * theta_dot ** 2
    rhs2 = p.m_p * p.g * p.l_c * s - p.b_theta * theta_dot

    Dth = D(theta, p)
    x_ddot = (p.I_p * rhs1 - p.m_p * p.l_c * c * rhs2) / Dth
    theta_ddot = ((p.m_c + p.m_p) * rhs2 - p.m_p * p.l_c * c * rhs1) / Dth

    return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

def energy(state, p: Params):
    _, x_dot, theta, theta_dot = state
    s, c = np.sin(theta), np.cos(theta)
    T = (0.5 * (p.m_c + p.m_p) * x_dot ** 2 + p.m_p * p.l_c * c * x_dot * theta_dot + 0.5 * p.I_p * theta_dot ** 2)
    V = p.m_p * p.g * p.l_c * c
    return T, V

