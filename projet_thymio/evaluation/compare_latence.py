import numpy as np
from simulation.simulator import simulate_and_filter, circle_trajectory, square_trajectory
from evaluation.metrics import summarize
from filtre.filter_node import FilterNode

def mc(cmds, delay, comp, n=30):
    runs = [simulate_and_filter(cmds, seed=s, delay_steps=delay,
                                node=FilterNode(use_jacobian=True, compensate=comp))
            for s in range(n)]
    res = [summarize(r) for r in runs]
    return (np.mean([r["rmse_pos_mm"] for r in res]),
            np.mean([r["mean_nees"] for r in res]),
            np.mean([r["kidnapped"].any() for r in runs]))

for name, cmds in {"carré": square_trajectory(speed=80.0),
                   "cercle": circle_trajectory(speed=80.0)}.items():
    for d in (0, 1, 2, 3):
        for comp in (False, True):
            r = mc(cmds, d, comp)
            print(f"{name:7s} retard {d*0.1:.1f} s  {'comp' if comp else 'brut'}  "
                  f"RMSE {r[0]:6.2f}  NEES {r[1]:7.2f}  fausses alarmes {100*r[2]:4.0f}%")