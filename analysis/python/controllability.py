import numpy as np
from params import Params, DEFAULT_PARAMS
from linearize import analytic_AB
                                                    
    
def controllability_matrix(A, B):                                                          
    n = A.shape[0]
    cols = [B]
    for _ in range(1, n):
        cols.append(A @ cols[-1])
    return np.hstack(cols)


def check_controllability(p: Params = DEFAULT_PARAMS):
    A, B = analytic_AB(p)
    C = controllability_matrix(A, B)                                                       
    rank = np.linalg.matrix_rank(C)
    print(f"rank(C) = {rank} (need 4), det(C) = {np.linalg.det(C):.6e}")
    return rank == 4


if __name__ == "__main__":                            
    check_controllability()