"""Tests de coherence statistique du filtre en simulation (Monte Carlo)."""
import numpy as np
from simulation.simulator import simulate_and_filter, square_trajectory
from evaluation.metrics import summarize
from filtre.filter_node import FilterNode
from filtre import params

N = 20
CMDS = square_trajectory()


def mc(**kw):
    return [summarize(simulate_and_filter(CMDS, seed=s, **kw)) for s in range(N)]


def test_nominal_is_consistent():
    res = mc()
    assert 2.5 < np.mean([r["mean_nees"] for r in res]) < 3.6
    assert 2.5 < np.mean([r["mean_nis"] for r in res]) < 3.6
    assert np.mean([np.mean(r["coverage_3sigma"]) for r in res]) > 0.98


def test_filter_beats_raw_camera_noise():
    # l'estimation doit etre meilleure que la mesure brute (sigma_cam = 2 mm en x et y)
    res = mc()
    assert np.mean([r["rmse_pos_mm"] for r in res]) < 2.0 * np.sqrt(2)


def test_short_dropout_stays_consistent():
    res = mc(hidden=[(10.0, 12.0)])
    assert np.mean([r["mean_nees"] for r in res]) < 4.0


def test_detects_underestimated_noise():
    # un Q 100x trop petit doit etre revele par un NEES tres superieur a 3
    bad = lambda: FilterNode(sigma_v=params.SIGMA_V * 0.1,
                             sigma_w=params.SIGMA_W * 0.1,
                             q_floor=params.Q_FLOOR * 0.01)
    res = [summarize(simulate_and_filter(CMDS, seed=s, node=bad())) for s in range(N)]
    assert np.mean([r["mean_nees"] for r in res]) > 20


def test_runs_are_reproducible():
    a = simulate_and_filter(CMDS, seed=3)
    b = simulate_and_filter(CMDS, seed=3)
    assert np.allclose(a["est"], b["est"])
