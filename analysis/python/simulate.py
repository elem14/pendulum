import numpy as np
from scipy.integrate import solve_ivp
from params import Params
from dynamics import f

def run_sim(state0, t_span, p: Params, u_func=lambda t, state: 0.0, t_eval=None, **kwargs):
    def rhs(t, state):
        return f(state, u_func(t, state), p)

    if t_eval is None:
        t_eval = np.linspace(t_span[0], t_span[1], 2000)

    result = solve_ivp(rhs, t_span, state0, t_eval=t_eval, method="RK45", rtol=1e-9, atol=1e-11, **kwargs)

    if not result.success:
        raise RuntimeError(f"Integration failed: {result.message}")
    return result

#u_func(t, state) defaults to zero, can reuse unchanged for unforced validation checks

