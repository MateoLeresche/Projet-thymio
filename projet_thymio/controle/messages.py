"""Le lien entre mon code et les structures de interfaces.py (le contrat de Matéo).

Dans controle/ on travaille avec des tuples (x, y, theta) et des listes de points.
Ce fichier c'est le seul endroit où on lit ou fabrique les messages des autres :

    filtre vers moi : PoseEstimate, on en tire la pose avec pose_tuple et on regarde must_stop
    Robin vers moi : Path, on en tire la liste de points avec path_points
    moi vers le filtre : ControlInput, fabriqué par control_input
"""

import math

from interfaces import ControlInput, Path, PoseEstimate
from controle import motion_model

# si le filtre est trop incertain (QR caché trop longtemps) on arrête le robot
POSITION_STD_MAX_MM = 50.0   # à confirmer avec Matéo : 5 cm d'incertitude max


def pose_tuple(estimate: PoseEstimate):
    """Transforme le PoseEstimate du filtre en (x, y, theta)."""
    return (estimate.x, estimate.y, estimate.theta)


def path_points(path: Path):
    """Transforme le Path de Robin en liste de points, ou None s'il n'a pas trouvé de chemin."""
    if path is None or not path.valid:
        return None
    return [tuple(p) for p in path.waypoints]


def position_std(estimate: PoseEstimate):
    """Incertitude sur la position en mm (on prend la pire entre x et y dans la matrice P)."""
    return math.sqrt(max(estimate.P[0][0], estimate.P[1][1]))


def must_stop(estimate: PoseEstimate):
    """Vrai s'il faut arrêter le robot : kidnapping, ou le filtre ne sait plus trop où il est."""
    return estimate.kidnapped or position_std(estimate) > POSITION_STD_MAX_MM


def control_input(t, v_left, v_right):
    """Fabrique le ControlInput pour le filtre à partir de la vitesse des roues (mm/s).

    On donne les vitesses MESURÉES et pas les consignes, parce que pendant un
    évitement ou au démarrage c'est ce que le robot fait vraiment.
    """
    v, omega = motion_model.wheels_to_v_omega(v_left, v_right)
    return ControlInput(t=t, v=v, omega=omega)
