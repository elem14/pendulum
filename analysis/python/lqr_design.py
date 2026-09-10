import numpy as np
from scipy.linalg import solve_continuous_are
from params import Params, DEFAULT_PARAMS             
from linearize import analytic_AB

                                                    
def bryson_QR(max_x=0.15, max_theta=np.deg2rad(6), max_xdot=0.5,
            max_thetadot=np.deg2rad(60), max_u=2.0):                                     
    Q = np.diag([1/max_x**2, 1/max_xdot**2, 1/max_theta**2, 1/max_thetadot**2])
    R = np.array([[1/max_u**2]])
    return Q, R


def design_lqr(p: Params = DEFAULT_PARAMS, Q=None, R=None):
    A, B = analytic_AB(p)                             
    if Q is None or R is None:
        Q, R = bryson_QR()                                                                 
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    eigs_cl = np.linalg.eigvals(A - B @ K)
    print("K =", K)
    print("closed-loop eigenvalues:", eigs_cl)
    print("PASS" if np.all(eigs_cl.real < 0) else "FAIL")
    return K, P
    

if __name__ == "__main__":
    design_lqr()