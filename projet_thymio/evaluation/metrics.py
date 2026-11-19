"""Banc d'essai : verite terrain + commandes/mesures bruitees, branche sur FilterNode.

Le filtre ne voit que des ControlInput et des PoseMeasurement : il ne fait pas
la difference avec P1 (camera) et P4 (controle). C'est l'interet du contrat.

Hypotheses du simulateur (coherentes avec le modele du filtre) :
- le robot execute la commande + un bruit sur (v, omega) de ecart-types (sigma_v, sigma_w)
  + un petit bruit de modele Q_floor ;
- la camera mesure la verite + un bruit gaussien de ecart-types sigma_cam.
"""
import numpy as np
from interfaces import ControlInput, PoseMeasurement
from filtre.angles import wrap
from filtre.filter_node import FilterNode
from filtre import params


def build_commands(segments, ts):
    """Construit la liste des commandes (v, omega) pas par pas.

    segments : liste de tuples
        ("move", distance_mm, vitesse_mm_s)
        ("turn", angle_rad, vitesse_rad_s)      (angle signe, + = anti-horaire)
        ("wait", duree_s)
    Le robot fait soit une translation pure, soit une rotation sur place.
    """
    cmds = []
    for seg in segments:
        kind = seg[0]
        if kind == "move":
            _, dist, speed = seg
            n = int(round(abs(dist) / speed / ts))
            cmds += [(np.sign(dist) * speed, 0.0)] * n
        elif kind == "turn":
            _, ang, rate = seg
            n = int(round(abs(ang) / rate / ts))
            cmds += [(0.0, np.sign(ang) * rate)] * n
        elif kind == "wait":
            n = int(round(seg[1] / ts))
            cmds += [(0.0, 0.0)] * n
        else:
            raise ValueError(f"segment inconnu : {kind}")
    return np.array(cmds, dtype=float)


def square_trajectory(side_mm=400.0, speed=50.0, rate=0.5, ts=params.TS):
    """Carre parcouru dans le sens anti-horaire : 4 x (translation + rotation 90 deg)."""
    segs = []
    for _ in range(4):
        segs += [("move", side_mm, speed), ("turn", np.pi / 2, rate)]
    return build_commands(segs, ts)


def simulate_and_filter(commands, x0=(100.0, 100.0, 0.0), ts=params.TS,
                        sigma_cam=params.SIGMA_CAM, sigma_v=params.SIGMA_V,
                        sigma_w=params.SIGMA_W, q_floor=params.Q_FLOOR,
                        cam_every=1, hidden=(), seed=0, node=None):
    """Simule le robot, genere les messages et fait tourner le filtre.

    cam_every : une mesure camera tous les `cam_every` pas.
    hidden    : intervalles (t_debut, t_fin) en s pendant lesquels le QR est cache.
    node      : FilterNode a tester (par defaut : parametres de params.py).
    Retourne un dict de tableaux (une ligne par pas).
    """
    rng = np.random.default_rng(seed)
    node = node if node is not None else FilterNode()
    Su = np.array([sigma_v, sigma_w])
    q_std = np.sqrt(np.diag(q_floor))

    truth = np.array(x0, dtype=float)
    out = {k: [] for k in ("t", "truth", "u", "z", "visible", "est", "P", "nis", "used")}

    def cam_visible(t, k):
        if k % cam_every != 0:
            return False
        return not any(a <= t < b for a, b in hidden)

    def measure(t, k):
        if not cam_visible(t, k):
            return PoseMeasurement(t=t, visible=False)
        z = truth + rng.normal(0.0, sigma_cam)
        return PoseMeasurement(t=t, visible=True, x=z[0], y=z[1], theta=wrap(z[2]))

    # pas 0 : initialisation (la premiere mesure doit etre visible)
    m = PoseMeasurement(t=0.0, visible=True,
                        **dict(zip(("x", "y", "theta"), truth + rng.normal(0.0, sigma_cam))))
    m.theta = wrap(m.theta)
    node.on_measurement(m)
    est = node.step(0.0)
    _log(out, 0.0, truth, (0.0, 0.0), m, est, node)

    for k, (v, w) in enumerate(commands, start=1):
        t = k * ts
        # commande appliquee entre t-ts et t, annoncee au filtre
        node.on_control(ControlInput(t=t - ts, v=float(v), omega=float(w)))
        # le vrai robot execute la commande + bruit
        th = truth[2]
        B = np.array([[np.cos(th) * ts, 0.0],
                      [np.sin(th) * ts, 0.0],
                      [0.0,             ts]])
        noisy_u = np.array([v, w]) + rng.normal(0.0, Su)
        truth = truth + B @ noisy_u + rng.normal(0.0, q_std)
        truth[2] = wrap(truth[2])
        m = measure(t, k)
        node.on_measurement(m)
        est = node.step(t)
        _log(out, t, truth, (v, w), m, est, node)

    return {k: np.array(v) for k, v in out.items()}


def _log(out, t, truth, u, m, est, node):
    out["t"].append(t)
    out["truth"].append(truth.copy())
    out["u"].append(u)
    out["z"].append([m.x, m.y, m.theta] if m.visible else [np.nan] * 3)
    out["visible"].append(m.visible)
    out["est"].append([est.x, est.y, est.theta])
    out["P"].append(est.P)
    out["nis"].append(np.nan if node.last_nis is None else node.last_nis)
    out["used"].append(est.measurement_used)
