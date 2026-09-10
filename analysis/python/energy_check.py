#first validation
import numpy as np
from params import Params, DEFAULT_PARAMS
from dynamics import energy                           
from simulate import run_sim


def frictionless_params(p: Params) -> Params:
    return Params(m_c=p.m_c, m_p=p.m_p, L=p.L, g=p.g, b_x=0.0, b_theta=0.0)
                                                    
    
def check_energy_conservation(p: Params = DEFAULT_PARAMS):                                 
    p0 = frictionless_params(p)
    state0 = np.array([0.0, 0.0, np.deg2rad(30), 0.0])
    res = run_sim(state0, (0, 5), p0, u_func=lambda t, s: 0.0)

    E = np.array([sum(energy(res.y[:, i], p0)) for i in range(res.y.shape[1])])
    rel_drift = np.max(np.abs(E - E[0])) / abs(E[0])
    print(f"max rel drift = {rel_drift:.3e}")
    print("PASS" if rel_drift < 1e-6 else "FAIL")
    return rel_drift


if __name__ == "__main__":                            
    check_energy_conservation()