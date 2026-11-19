"""Suivi du chemin : calcule les vitesses des roues pour aller de point en point.

Loi fluide (utilisée par défaut), d'après le cours :
    - leçon 1 : « rotations sur place + lignes droites » est simple mais pas
      fluide ; le contrôleur d'Astolfi module en douceur avance et rotation ;
    - leçon 5 (champs de potentiel, robot différentiel) : rotation
      proportionnelle à l'angle alpha, avance proportionnelle à cos(alpha).

    rho   = distance au point visé
    alpha = angle dont il faut tourner pour regarder le point (entre -pi et pi)

    omega = K_ALPHA * alpha                       (comme Astolfi, sans le terme beta)
    v     = V_MAX * max(0, cos(alpha))            (avance d'autant plus vite qu'on est bien orienté)
    v     = min(v, K_RHO * rho + V_MIN)           (dernier point seulement : ralentir en arrivant, comme Astolfi)

Adaptation d'Astolfi à un chemin (le prof pénalise Astolfi « appliqué tel quel ») :
    - pas de terme beta : on ne demande pas d'orientation précise aux points
      de passage, seulement d'y passer ;
    - pas de v = K_RHO * rho partout : le robot s'arrêterait à chaque point ;
      on ne ralentit qu'à l'arrivée ;
    - pas de marche arrière : si le point est derrière, cos(alpha) < 0, donc
      v = 0 et le robot tourne d'abord sur lui-même.

Pourquoi le robot arrive toujours : la distance diminue à la vitesse
v * cos(alpha) = V_MAX * cos(alpha)**2, jamais négative. Elle ne peut donc
que baisser, sauf si alpha = ±90°, mais alors omega est maximal et alpha
quitte aussitôt cette valeur. Le robot ne peut pas tourner en rond autour du point.

Entrées : pose (x, y, theta) en mm et rad, donnée par le filtre ;
          chemin = liste de points (x, y) en mm, donnée par la planification.
Sortie  : (v_left, v_right) en mm/s, à passer à Thymio.set_motors.
"""

import math

from controle import motion_model

# --- Réglages (À RÉGLER par essais, noter les valeurs testées pour le rapport) ---
V_MAX = 80.0                  # mm/s, vitesse quand le robot est bien orienté
K_ALPHA = 2.0                 # 1/s, gain de rotation, choisi sur le vrai robot : 3 oscille après les virages (voir mesures_reelles.md)
OMEGA_MAX = 2.5               # rad/s, rotation maximale
K_RHO = 1.0                   # 1/s, ralentissement à l'arrivée
V_MIN = 15.0                  # mm/s, pour ne pas ramper indéfiniment juste avant le but
WAYPOINT_TOLERANCE = 20.0     # mm, point de passage atteint (plus grand = coupe davantage les virages)
GOAL_TOLERANCE = 15.0         # mm, arrivée

# Ancienne loi, gardée pour comparer dans le rapport : CONTROL_LAW = "turn_then_go".
CONTROL_LAW = "smooth"
TURN_IN_PLACE_ANGLE = math.radians(30)


def heading_error(pose, target):
    """Distance rho (mm) et erreur d'angle alpha (rad) entre la pose et la cible."""
    x, y, theta = pose
    dx, dy = target[0] - x, target[1] - y
    rho = math.hypot(dx, dy)
    alpha = motion_model.wrap_angle(math.atan2(dy, dx) - theta)
    return rho, alpha


def go_to_point(pose, target, is_final=False):
    """Vitesses (v_left, v_right) en mm/s pour aller vers `target` depuis `pose`."""
    rho, alpha = heading_error(pose, target)
    omega = max(-OMEGA_MAX, min(OMEGA_MAX, K_ALPHA * alpha))

    if CONTROL_LAW == "turn_then_go":
        v = 0.0 if abs(alpha) > TURN_IN_PLACE_ANGLE else V_MAX
    else:
        v = V_MAX * max(0.0, math.cos(alpha))
        if is_final:
            v = min(v, K_RHO * rho + V_MIN)

    v_left, v_right = motion_model.v_omega_to_wheels(v, omega)
    return motion_model.limit_wheels(v_left, v_right)


class PathFollower:
    """Suit une liste de points de passage, l'un après l'autre.

        follower = PathFollower(chemin)
        à chaque pas :  v_left, v_right, fini = follower.update(pose)
        nouveau chemin (après évitement, kidnapping...) : follower.set_path(nouveau_chemin)
    """

    def __init__(self, path=()):
        self.set_path(path)

    def set_path(self, path):
        self.path = [tuple(p) for p in path]
        self.index = 0

    @property
    def target(self):
        """Point visé en ce moment, ou None si le chemin est terminé."""
        return self.path[self.index] if self.index < len(self.path) else None

    @property
    def is_final(self):
        return self.index == len(self.path) - 1

    @property
    def done(self):
        return self.index >= len(self.path)

    def update(self, pose):
        # Passer les points déjà atteints (le premier est souvent la position de départ).
        while not self.done:
            rho, _ = heading_error(pose, self.target)
            tolerance = GOAL_TOLERANCE if self.is_final else WAYPOINT_TOLERANCE
            if rho > tolerance:
                break
            self.index += 1
        if self.done:
            return 0.0, 0.0, True
        v_left, v_right = go_to_point(pose, self.target, self.is_final)
        return v_left, v_right, False
