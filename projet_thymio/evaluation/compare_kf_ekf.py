import numpy as np
from simulation.simulator import simulate_and_filter, circle_trajectory, square_trajectory
from evaluation.metrics import summarize
from filtre.filter_node import FilterNode

def mc(cmds, jac, n=50, **kw):
    res = [summarize(simulate_and_filter(cmds, seed=s,
                     node=FilterNode(use_jacobian=jac), **kw)) for s in range(n)]
    return (np.mean([r["rmse_pos_mm"] for r in res]),
            np.mean([r["mean_nees"] for r in res]),
            np.mean([np.mean(r["coverage_3sigma"]) for r in res]))

scenarios = {
    "carré, caméra 10 Hz":      (square_trajectory(), {}),
    "cercle, caméra 10 Hz":     (circle_trajectory(), {}),
    "cercle, QR caché 5 s":     (circle_trajectory(), dict(hidden=[(10.0, 15.0)])),
    "cercle, odométrie seule":  (circle_trajectory(), dict(hidden=[(0.2, 1e9)])),
}
for name, (cmds, kw) in scenarios.items():
    for jac in (False, True):
        r = mc(cmds, jac, **kw)
        print(f"{name:28s} {'EKF' if jac else 'KF ':4s} RMSE {r[0]:6.2f}  NEES {r[1]:6.2f}  couv {100*r[2]:5.1f}%")