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

def test_kidnapping_detected():
    run = simulate_and_filter(CMDS, seed=0, teleport=(10.0, (300.0, 50.0, 2.0)))
    k = run["kidnapped"]
    assert k.sum() == 1                     # détecte le kidnapping une seule fois
    t_det = run["t"][np.argmax(k)]
    assert 10.0 < t_det < 12.1              # détecte le kidnapping après confirmation (environ 2 s)
    err = np.linalg.norm(run["est"][-1][:2] - run["truth"][-1][:2])
    assert err < 10.0                       # l'estimation se recolle rapidement (environ 5 cm)

def test_no_false_kidnapping_nominal():
    for s in range(N):
        assert not simulate_and_filter(CMDS, seed=s)["kidnapped"].any()

def test_dropout_is_not_kidnapping():
    run = simulate_and_filter(CMDS, seed=0, hidden=[(10.0, 15.0)])
    assert not run["kidnapped"].any()

def test_latency_compensation_helps():
    cmds = square_trajectory(speed=80.0)
    def rmse(comp):
        return np.mean([summarize(simulate_and_filter(
            cmds, seed=s, delay_steps=2,
            node=FilterNode(use_jacobian=True, compensate=comp)))["rmse_pos_mm"]
            for s in range(10)])
    assert rmse(True) < 0.7 * rmse(False)

def test_compensation_neutral_without_delay():
    cmds = square_trajectory()
    a = simulate_and_filter(cmds, seed=1, node=FilterNode(compensate=False))
    b = simulate_and_filter(cmds, seed=1, node=FilterNode(compensate=True))
    assert np.allclose(a["est"], b["est"], atol=1e-6)