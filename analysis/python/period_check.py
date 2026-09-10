import numpy as np                                    
from params import Params, DEFAULT_PARAMS
from energy_check import frictionless_params
from simulate import run_sim


def closed_form_period(p: Params) -> float:           
    return 2 * np.pi * np.sqrt(p.I_p / (p.m_p * p.g * p.l_c))


def cart_fixed_params(p: Params) -> Params:
    # the closed form assumes the cart is HELD FIXED; emulate that by
    # making the cart effectively infinitely heavy    
    return Params(m_c=1e8 * p.m_c, m_p=p.m_p, L=p.L, g=p.g,
                b_x=p.b_x, b_theta=p.b_theta)                                            


def measured_period(res, theta_offset=np.pi) -> float:
    centered = res.y[2] - theta_offset
    signs = np.sign(centered)                                                              
    crossings = np.where(np.diff(signs) != 0)[0]
    t = res.t
    cross_times = []                                  
    for i in crossings:
        t0, t1 = t[i], t[i + 1]                                                            
        y0, y1 = centered[i], centered[i + 1]
        cross_times.append(t0 - y0 * (t1 - t0) / (y1 - y0))
    return 2 * np.mean(np.diff(np.array(cross_times)))


def check_period(p: Params = DEFAULT_PARAMS):
    p0 = cart_fixed_params(frictionless_params(p))
    theta0 = np.pi - np.deg2rad(5)                    
    state0 = np.array([0.0, 0.0, theta0, 0.0])

    T_expected = closed_form_period(p0)
    res = run_sim(state0, (0, 6 * T_expected), p0, u_func=lambda t, s: 0.0,
                t_eval=np.linspace(0, 6 * T_expected, 20000))
    T_measured = measured_period(res)
    rel_err = abs(T_measured - T_expected) / T_expected
    print(f"expected={T_expected:.6f}s measured={T_measured:.6f}s rel_err={rel_err:.3e}")
    print("PASS" if rel_err < 1e-3 else "FAIL")
    return rel_err                                    


if __name__ == "__main__":                            
    check_period() 