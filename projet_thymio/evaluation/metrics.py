"""Metriques de validation du filtre (simulation : on connait la verite terrain)."""
import numpy as np
from filtre.angles import wrap


def errors(truth, est):
    """Erreur d'estimation (N,3) avec l'angle normalise."""
    e = np.asarray(est) - np.asarray(truth)
    e[:, 2] = wrap(e[:, 2])
    return e


def nees(err, P):
    """NEES = e^T P^-1 e a chaque pas. Moyenne attendue ~ 3 si le filtre est coherent."""
    return np.array([e @ np.linalg.solve(Pk, e) for e, Pk in zip(err, P)])


def coverage_3sigma(err, P):
    """Proportion des pas ou |erreur| < 3 sigma, pour chaque composante (attendu ~ 99,7 %)."""
    sig = np.sqrt(np.array([np.diag(Pk) for Pk in P]))
    return np.mean(np.abs(err) < 3 * sig, axis=0)


def rmse_position(err):
    return float(np.sqrt(np.mean(err[:, 0] ** 2 + err[:, 1] ** 2)))


def rmse_theta(err):
    return float(np.sqrt(np.mean(err[:, 2] ** 2)))


def summarize(run):
    """Resume d'une simulation (dict renvoye par simulate_and_filter)."""
    err = errors(run["truth"], run["est"])
    n = nees(err, run["P"])
    nis = run["nis"][~np.isnan(run["nis"])]
    return {
        "rmse_pos_mm": rmse_position(err),
        "rmse_theta_deg": np.degrees(rmse_theta(err)),
        "mean_nees": float(np.mean(n)),
        "mean_nis": float(np.mean(nis)) if len(nis) else float("nan"),
        "coverage_3sigma": coverage_3sigma(err, run["P"]),
    }
