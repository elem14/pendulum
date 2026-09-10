#analytic A B and numeric cross checking
import numpy as np
from params import Params, DEFAULT_PARAMS
from dynamics import f, D                             
    
                                                                                            
def analytic_AB(p: Params):
    D0 = D(0.0, p)
    A = np.array([
        [0, 1, 0, 0],
        [0, -p.I_p * p.b_x / D0, -p.m_p ** 2 * p.g * p.l_c ** 2 / D0,
        p.m_p * p.l_c * p.b_theta / D0],
        [0, 0, 0, 1],
        [0, p.m_p * p.l_c * p.b_x / D0, (p.m_c + p.m_p) * p.m_p * p.g * p.l_c / D0,
        -(p.m_c + p.m_p) * p.b_theta / D0],
    ])                                                
    B = np.array([[0], [p.I_p / D0], [0], [-p.m_p * p.l_c / D0]])
    return A, B
                                                    
    
def numeric_AB(p: Params, eps=1e-6):                                                       
    state_eq, u_eq = np.zeros(4), 0.0
    A = np.zeros((4, 4))
    for j in range(4):
        dx = np.zeros(4); dx[j] = eps
        A[:, j] = (f(state_eq + dx, u_eq, p) - f(state_eq - dx, u_eq, p)) / (2 * eps)
    B = ((f(state_eq, u_eq + eps, p) - f(state_eq, u_eq - eps, p)) / (2 * eps)).reshape(4, 1)
    return A, B


def check_linearization(p: Params = DEFAULT_PARAMS):
    A_an, B_an = analytic_AB(p)                       
    A_num, B_num = numeric_AB(p)
    print("max|A diff| =", np.max(np.abs(A_an - A_num)))                                   
    print("max|B diff| =", np.max(np.abs(B_an - B_num)))
    return A_an, B_an


if __name__ == "__main__":                            
    check_linearization()