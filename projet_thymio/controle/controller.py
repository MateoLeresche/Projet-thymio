"""Suivi du chemin : calcule la vitesse des roues pour aller d'un point au suivant.

On utilise la loi "fluide" du cours :
    leçon 1 : tourner sur place puis aller tout droit c'est simple mais pas fluide,
    Astolfi fait avancer et tourner en même temps.
    leçon 5 : rotation proportionnelle à l'angle alpha, avance proportionnelle à cos(alpha).

    rho   = distance jusqu'au point visé
    alpha = de combien il faut tourner pour regarder le point (entre -pi et pi)

    omega = K_ALPHA * alpha               on tourne d'autant plus vite que l'angle est grand
    v     = V_MAX * max(0, cos(alpha))    on avance d'autant plus vite qu'on regarde bien le point
    v     = min(v, K_RHO * rho + V_MIN)   seulement au dernier point, pour ralentir en arrivant

Ce qu'on a changé par rapport à Astolfi (le prof n'aime pas Astolfi copié tel quel) :
    pas de terme beta, on ne veut pas arriver avec un angle précis, juste passer par le point
    pas de v = K_RHO * rho partout, sinon le robot s'arrête à chaque point
    pas de marche arrière, si le point est derrière cos(alpha) < 0 donc v = 0 et il tourne d'abord

Pourquoi il arrive toujours : la distance baisse à la vitesse V_MAX * cos(alpha)**2,
qui n'est jamais négative. Donc la distance ne peut que baisser, le robot ne peut
pas tourner en rond autour du point.

Entrées : la pose (x, y, theta) donnée par le filtre, le chemin [(x, y), ...] donné par Robin.
Sortie : (v_left, v_right) en mm/s pour Thymio.set_motors.
"""

import math

from controle import motion_model

# Réglages, testés sur le vrai robot (voir mesures_reelles.md)
V_MAX = 80.0                  # vitesse max quand le robot regarde bien le point, en mm/s
K_ALPHA = 3.0                 # force de rotation (2 virages un peu larges, 3 le mieux depuis qu'on reçoit les mesures toutes les 50 ms)
OMEGA_MAX = 2.5               # rotation max en rad/s
K_RHO = 1.0                   # pour ralentir en arrivant au but
V_MIN = 15.0                  # vitesse mini en arrivant, sinon il rampe pendant des heures
WAYPOINT_TOLERANCE = 20.0     # à 2 cm d'un point de passage on considère qu'il est atteint
GOAL_TOLERANCE = 15.0         # à 1,5 cm du but on considère qu'on est arrivé

# Ancienne loi (tourner sur place puis avancer), gardée pour comparer dans le rapport :
# mettre CONTROL_LAW = "turn_then_go"
CONTROL_LAW = "smooth"
TURN_IN_PLACE_ANGLE = math.radians(30)


def heading_error(pose, target):
    """Distance rho (mm) et angle alpha (rad) entre le robot et le point visé."""
    x, y, theta = pose
    dx, dy = target[0] - x, target[1] - y
    rho = math.hypot(dx, dy)
    alpha = motion_model.wrap_angle(math.atan2(dy, dx) - theta)  # direction du point moins direction du robot
    return rho, alpha


def go_to_point(pose, target, is_final=False):
    """Vitesses des roues (mm/s) pour aller vers target."""
    rho, alpha = heading_error(pose, target)
    omega = max(-OMEGA_MAX, min(OMEGA_MAX, K_ALPHA * alpha))  # on tourne vers le point, sans dépasser OMEGA_MAX

    if CONTROL_LAW == "turn_then_go":
        # ancienne loi : si l'angle est trop grand on tourne sur place, sinon tout droit
        v = 0.0 if abs(alpha) > TURN_IN_PLACE_ANGLE else V_MAX
    else:
        v = V_MAX * max(0.0, math.cos(alpha))   # bien orienté : vite, de travers : lentement
        if is_final:
            v = min(v, K_RHO * rho + V_MIN)     # on ralentit en arrivant au but

    v_left, v_right = motion_model.v_omega_to_wheels(v, omega)
    return motion_model.limit_wheels(v_left, v_right)


class PathFollower:
    """Suit les points du chemin un par un.

        follower = PathFollower(chemin)
        à chaque pas : v_left, v_right, fini = follower.update(pose)
        nouveau chemin (après un kidnapping) : follower.set_path(nouveau_chemin)
        après un évitement on garde le même chemin (le robot le reprend quand il retombe dessus)
    """

    def __init__(self, path=()):
        self.set_path(path)

    def set_path(self, path):
        self.path = [tuple(p) for p in path]
        self.index = 0   # numéro du point qu'on vise

    @property
    def target(self):
        """Point visé en ce moment, None si on a fini le chemin."""
        return self.path[self.index] if self.index < len(self.path) else None

    @property
    def is_final(self):
        return self.index == len(self.path) - 1   # on vise le dernier point, donc le but

    @property
    def done(self):
        return self.index >= len(self.path)

    def segment(self, pose):
        """Segment du chemin qu'on suit en ce moment : du point d'avant au point visé.
        Sert à l'évitement pour savoir quand on retombe sur le chemin."""
        a = self.path[self.index - 1] if self.index > 0 else (pose[0], pose[1])
        return a, self.target

    def update(self, pose):
        # on saute les points déjà atteints (le premier c'est souvent là où le robot est déjà)
        while not self.done:
            rho, _ = heading_error(pose, self.target)
            tolerance = GOAL_TOLERANCE if self.is_final else WAYPOINT_TOLERANCE
            if rho > tolerance:
                break
            self.index += 1
        if self.done:
            return 0.0, 0.0, True   # arrivé, on s'arrête
        v_left, v_right = go_to_point(pose, self.target, self.is_final)
        return v_left, v_right, False
