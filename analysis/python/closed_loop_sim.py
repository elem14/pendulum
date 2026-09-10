import numpy as np
import matplotlib.pyplot as plt

from params import Params, DEFAULT_PARAMS
from simulate import run_sim
from lqr_design import design_lqr
                                                    
                                                                                        
def closed_loop_test(p: Params = DEFAULT_PARAMS, theta0_deg: float = 10.0,
                    t_final: float = 5.0, plot: bool = True):
    K, _ = design_lqr(p)
    x_eq = np.array([0.0, 0.0, 0.0, 0.0])

    def u_func(t, state):                            
        return float((-K @ (state - x_eq)).item())                                        

    state0 = np.array([0.0, 0.0, np.deg2rad(theta0_deg), 0.0])
    res = run_sim(state0, (0, t_final), p, u_func=u_func)

    theta_final_deg = np.rad2deg(res.y[2, -1])       
    u_vals = np.array([u_func(t, res.y[:, i]) for i, t in enumerate(res.t)])
    max_u = np.max(np.abs(u_vals))
    max_x = np.max(np.abs(res.y[0]))
    print(f"theta0={theta0_deg:5.1f} deg  theta_final={theta_final_deg:9.4f} deg  "
        f"max|u|={max_u:7.2f} N  max|x|={max_x:.3f} m")
    converged = abs(theta_final_deg) < 0.5
    print("PASS -- converged near upright" if converged else "FAIL -- did not converge")

    if plot:
        fig, axes = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
        axes[0].plot(res.t, np.rad2deg(res.y[2]))
        axes[0].set_ylabel("theta [deg]")                                                 
        axes[1].plot(res.t, res.y[0])
        axes[1].set_ylabel("x [m]")
        axes[2].plot(res.t, u_vals)
        axes[2].set_ylabel("u [N]")                                                       
        axes[2].set_xlabel("t [s]")
        fig.suptitle(f"LQR on nonlinear plant, theta0={theta0_deg} deg")
        fig.tight_layout()
        fig.savefig("closed_loop_sim.png", dpi=150)                                       
        print("Saved plot to closed_loop_sim.png")

    return res                                       
                                                                                        

if __name__ == "__main__":
    closed_loop_test(theta0_deg=10.0)