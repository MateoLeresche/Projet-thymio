"""Passage entre les structures de interfaces.py et le reste du module controle.

Le module controle travaille en interne avec des tuples (x, y, theta) et des
listes de points ; ce fichier est le seul endroit où on lit ou fabrique les
messages des autres modules :

    Filtre -> Controle       : PoseEstimate  -> pose_tuple(est), must_stop(est)
    Planification -> Controle: Path          -> path_points(path)
    Controle -> Filtre       : ControlInput  <- control_input(t, v_left, v_right)
"""

import math

from interfaces import ControlInput, Path, PoseEstimate
from controle import motion_model

# Arrêt de sécurité si le filtre est trop incertain (contrat, section 4).
POSITION_STD_MAX_MM = 50.0   # [À CONFIRMER] écart-type de position au-delà duquel on s'arrête


def pose_tuple(estimate: PoseEstimate):
    """(x, y, theta) en mm et rad, le format de controller.PathFollower.update."""
    return (estimate.x, estimate.y, estimate.theta)


def path_points(path: Path):
    """Liste de points [(x, y), ...] en mm, ou None si la planification n'a pas trouvé de chemin."""
    if path is None or not path.valid:
        return None
    return [tuple(p) for p in path.waypoints]


def position_std(estimate: PoseEstimate):
    """Écart-type de position (mm) : racine de la plus grande des deux variances x, y."""
    return math.sqrt(max(estimate.P[0][0], estimate.P[1][1]))


def must_stop(estimate: PoseEstimate):
    """True si le robot doit s'arrêter : kidnapping, ou filtre trop incertain (QR caché trop longtemps)."""
    return estimate.kidnapped or position_std(estimate) > POSITION_STD_MAX_MM


def control_input(t, v_left, v_right):
    """ControlInput pour le filtre, à partir des vitesses des roues en mm/s.

    Passer les vitesses MESURÉES (Thymio.read()), pas les consignes : pendant un
    évitement ou un démarrage, c'est ce que le robot fait vraiment.
    """
    v, omega = motion_model.wheels_to_v_omega(v_left, v_right)
    return ControlInput(t=t, v=v, omega=omega)
