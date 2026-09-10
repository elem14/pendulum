import matplotlib
matplotlib.use("Agg")  # save PNGs instead of trying to pop up a window

from params import DEFAULT_PARAMS
from energy_check import check_energy_conservation
from period_check import check_period
from linearize import check_linearization, analytic_AB
from controllability import check_controllability
from lqr_design import design_lqr
from closed_loop_sim import closed_loop_test


def section(title):
    print("\n" + "=" * 70)
    print(title)
    print("=" * 70)


def main():
    p = DEFAULT_PARAMS

    section("STEP 4: energy conservation check")
    rel_drift = check_energy_conservation(p)

    section("STEP 5: small-oscillation period check")
    rel_err = check_period(p)

    section("STEP 6: linearization (analytic vs numeric Jacobian)")
    A, B = check_linearization(p)

    section("STEP 7: controllability")
    controllable = check_controllability(p)

    section("STEP 8: LQR design")
    K, P = design_lqr(p)

    section("STEP 9: LQR on the nonlinear plant")
    for theta0 in [10, 20, 30]:
        closed_loop_test(p, theta0_deg=theta0, plot=(theta0 == 10))

    section("SUMMARY")
    print(f"Energy conservation:     {'PASS' if rel_drift < 1e-6 else 'FAIL'} "
          f"(rel drift {rel_drift:.2e})")
    print(f"Period check:            {'PASS' if rel_err < 1e-3 else 'FAIL'} "
          f"(rel err {rel_err:.2e})")
    print(f"Controllability:         {'PASS' if controllable else 'FAIL'}")
    print(f"LQR gain K:              {K}")
    print("\nPlots saved: energy_check.png, period_check.png, closed_loop_sim.png")
    print("Edit params.py with real measurements, then just rerun this file.")


if __name__ == "__main__":
    main()
