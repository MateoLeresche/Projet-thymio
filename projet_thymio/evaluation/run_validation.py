"""Validation du filtre en simulation. Lancer depuis la racine du depot :

    python -m evaluation.run_validation

Affiche un tableau de resultats (Monte Carlo) et sauvegarde des figures dans docs/figures/.
Un filtre est COHERENT si : NEES moyen ~ 3, NIS moyen ~ 3, couverture +-3 sigma ~ 99,7 %.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from simulation.simulator import simulate_and_filter, square_trajectory
from evaluation.metrics import summarize, errors
from filtre.filter_node import FilterNode
from filtre import params

N_RUNS = 50


def monte_carlo(label, make_node=None, **sim_kw):
    cmds = square_trajectory()
    res = [summarize(simulate_and_filter(cmds, seed=s,
                                         node=make_node() if make_node else None,
                                         **sim_kw)) for s in range(N_RUNS)]
    row = {
        "scenario": label,
        "rmse_pos": np.mean([r["rmse_pos_mm"] for r in res]),
        "rmse_th": np.mean([r["rmse_theta_deg"] for r in res]),
        "nees": np.mean([r["mean_nees"] for r in res]),
        "nis": np.nanmean([r["mean_nis"] for r in res]),
        "cov": np.mean([np.mean(r["coverage_3sigma"]) for r in res]),
    }
    return row


def main():
    rows = [
        monte_carlo("Camera 10 Hz (reglage nominal)"),
        monte_carlo("Camera 2 Hz", cam_every=5),
        monte_carlo("QR cache 2 s (t=10 a 12 s)", hidden=[(10.0, 12.0)]),
        monte_carlo("QR cache 5 s (t=10 a 15 s)", hidden=[(10.0, 15.0)]),
        monte_carlo("Odometrie seule (QR cache des 0,2 s)", hidden=[(0.2, 1e9)]),
        monte_carlo("Q trop petit (x0,01)",
                    make_node=lambda: FilterNode(sigma_v=params.SIGMA_V * 0.1,
                                                 sigma_w=params.SIGMA_W * 0.1,
                                                 q_floor=params.Q_FLOOR * 0.01)),
        monte_carlo("R trop petit (x0,01)",
                    make_node=lambda: FilterNode(R=params.R * 0.01)),
    ]
    print(f"{'Scenario':42s} {'RMSE pos':>9s} {'RMSE th':>8s} {'NEES':>7s} {'NIS':>6s} {'Couv.3s':>8s}")
    print(f"{'':42s} {'(mm)':>9s} {'(deg)':>8s} {'(~3)':>7s} {'(~3)':>6s} {'(~99.7%)':>8s}")
    for r in rows:
        print(f"{r['scenario']:42s} {r['rmse_pos']:9.2f} {r['rmse_th']:8.2f} "
              f"{r['nees']:7.2f} {r['nis']:6.2f} {100 * r['cov']:7.1f}%")

    # ---- figure 1 : un run nominal ----
    run = simulate_and_filter(square_trajectory(), seed=1)
    err = errors(run["truth"], run["est"])
    sig = np.sqrt(np.array([np.diag(P) for P in run["P"]]))
    t = run["t"]
    fig, ax = plt.subplots(2, 3, figsize=(14, 7))
    a = ax[0, 0]
    a.plot(run["truth"][:, 0], run["truth"][:, 1], "k-", label="verite")
    a.plot(run["est"][:, 0], run["est"][:, 1], "r--", label="estimation")
    a.plot(run["z"][:, 0], run["z"][:, 1], ".", ms=2, alpha=0.4, label="camera")
    a.set_aspect("equal"); a.set_title("Trajectoire (mm)"); a.legend(fontsize=7)
    for i, (name, unit) in enumerate([("x", "mm"), ("y", "mm"), ("theta", "rad")]):
        a = ax[(i + 1) // 3, (i + 1) % 3]
        a.plot(t, err[:, i], lw=0.8)
        a.fill_between(t, -3 * sig[:, i], 3 * sig[:, i], alpha=0.2, label="+-3 sigma (de P)")
        a.set_title(f"Erreur sur {name} ({unit})"); a.set_xlabel("t (s)"); a.legend(fontsize=7)
    from evaluation.metrics import nees
    a = ax[1, 1]
    a.plot(t, nees(err, run["P"]), lw=0.8)
    a.axhline(3, color="g", ls=":", label="valeur attendue (3)")
    a.set_title("NEES (chi2, 3 ddl)"); a.set_xlabel("t (s)"); a.legend(fontsize=7)
    a = ax[1, 2]
    nis = run["nis"]
    a.plot(t, nis, ".", ms=3)
    a.axhline(11.34, color="r", ls="--", label="seuil 99 % (11,34)")
    a.axhline(3, color="g", ls=":", label="valeur attendue (3)")
    a.set_title("NIS (chi2, 3 ddl)"); a.set_xlabel("t (s)"); a.legend(fontsize=7)
    plt.tight_layout()
    plt.savefig("docs/figures/validation_nominal.png", dpi=110)
    plt.close()

    # ---- figure 2 : QR cache 5 s ----
    run = simulate_and_filter(square_trajectory(), seed=1, hidden=[(10.0, 15.0)])
    err = errors(run["truth"], run["est"])
    sig = np.sqrt(np.array([np.diag(P) for P in run["P"]]))
    fig, ax = plt.subplots(1, 2, figsize=(12, 4))
    ax[0].plot(run["t"], np.hypot(err[:, 0], err[:, 1]), label="erreur de position")
    ax[0].plot(run["t"], 3 * np.hypot(sig[:, 0], sig[:, 1]) / np.sqrt(2), "--",
               label="~3 sigma (de P)")
    ax[0].axvspan(10, 15, alpha=0.15, color="gray", label="QR cache")
    ax[0].set_xlabel("t (s)"); ax[0].set_ylabel("mm"); ax[0].legend(); ax[0].set_title("Position")
    ax[1].plot(run["t"], np.degrees(np.abs(err[:, 2])), label="erreur d'angle")
    ax[1].plot(run["t"], 3 * np.degrees(sig[:, 2]), "--", label="3 sigma (de P)")
    ax[1].axvspan(10, 15, alpha=0.15, color="gray")
    ax[1].set_xlabel("t (s)"); ax[1].set_ylabel("deg"); ax[1].legend(); ax[1].set_title("Angle")
    plt.tight_layout()
    plt.savefig("docs/figures/validation_qr_cache.png", dpi=110)
    plt.close()
    print("\nFigures sauvegardees dans docs/figures/")


if __name__ == "__main__":
    main()
