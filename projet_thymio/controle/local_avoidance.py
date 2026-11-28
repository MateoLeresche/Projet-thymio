"""Évitement local : contourner un obstacle qu'on n'avait pas prévu (Bug 2 du cours).

Les obstacles ajoutés pendant la démo ne sont pas sur la carte (la carte est
faite une seule fois au début), donc seuls les capteurs infrarouges avant les voient.

Comment ça marche :
    1. Un des 5 capteurs avant dépasse PROX_ENTER : on passe en contournement.
       On choisit le côté où on garde l'obstacle (celui où il est le plus visible) et on n'en change plus.
    2. On longe l'obstacle en le gardant de ce côté :
         obstacle devant (capteur du milieu ou celui d'à côté)  -> on tourne sur place pour s'écarter
         un capteur de ce côté le voit encore                   -> on s'écarte en avançant, d'autant plus qu'il voit fort
         plus aucun capteur ne le voit                          -> on tourne doucement vers lui pour le retrouver
       Comme ça le robot fait le tour de l'objet.
    3. Bug 2 : pendant qu'on contourne, si le robot retombe sur le segment du chemin qu'il suivait,
       plus loin que l'endroit où il l'a quitté, et qu'il n'y a plus rien devant, on reprend le chemin.
    4. Si on tourne autour trop longtemps sans retrouver le chemin, on abandonne et main.py
       redemande un chemin à Robin.

Capteurs : prox[0] avant gauche (bord), prox[1], prox[2] milieu, prox[3], prox[4] avant droit (bord).
"""

import math

from controle import motion_model

# Réglages (PROX_ENTER et PROX_EXIT mesurés sur notre robot, voir mesures_reelles.md)
PROX_ENTER = 1000        # on commence à contourner dès qu'un capteur voit nettement quelque chose
                         # (l'obstacle de la démo ne dépasse jamais 2300, même collé : 1239 à 6 cm, 0 à 8 cm)
PROX_EXIT = 300          # en dessous le capteur ne voit plus rien (il renvoie 0 au delà de 13 cm environ)
PROX_FRONT = 2000        # obstacle devant plus fort que ça : on tourne sur place
V_AVOID = 60.0           # vitesse d'avance pendant le contournement, en mm/s
OMEGA_TURN = 1.5         # rotation sur place quand l'obstacle est devant, en rad/s
K_FOLLOW = 0.001         # rad/s par unité de capteur : plus le capteur voit fort, plus on s'écarte
R_ARC = 160.0            # quand on ne voit plus l'obstacle, on tourne vers lui sur un cercle de 16 cm de rayon
# Réglages choisis en simulation avec 2 courbes de capteur (boîte blanche et obstacle de la démo)
# et 13 obstacles de 2 à 4,5 cm de rayon, sur et à côté du chemin : aucun contact. À vérifier en vrai.
LINE_TOLERANCE = 20.0    # à moins de 2 cm du segment du chemin on considère qu'on l'a retrouvé
MIN_PROGRESS = 50.0      # il faut avoir avancé d'au moins 5 cm le long du chemin (sinon on repart avant l'obstacle)
MAX_CONTOUR_MM = 1500.0  # après 1,5 m de contournement sans retrouver le chemin on abandonne


def obstacle_detected(prox):
    """Vrai si un des 5 capteurs avant voit un obstacle trop proche."""
    return max(prox[:5]) > PROX_ENTER


def segment_position(pose, a, b):
    """Où est le robot par rapport au segment a -> b du chemin.

    Renvoie (along, distance) : along = combien on a avancé le long du segment depuis a,
    distance = distance du robot au segment.
    """
    x, y = pose[0], pose[1]
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    if length < 1.0:
        return 0.0, math.hypot(x - a[0], y - a[1])
    along = ((x - a[0]) * (b[0] - a[0]) + (y - a[1]) * (b[1] - a[1])) / length   # produit scalaire
    k = min(max(along / length, 0.0), 1.0)                                         # point le plus proche sur le segment
    px, py = a[0] + k * (b[0] - a[0]), a[1] + k * (b[1] - a[1])
    return along, math.hypot(x - px, y - py)


class LocalAvoidance:
    """Utilisation :
        avoid = LocalAvoidance()
        si obstacle_detected(prox) : avoid.start(prox, pose, follower.segment(pose))
        puis à chaque pas : v_left, v_right, fini = avoid.step(prox, pose)
        quand fini vaut True : on reprend le chemin, sauf si avoid.gave_up (alors nouveau chemin)
    """

    def __init__(self):
        self.side = 1.0          # +1 obstacle gardé à gauche (on tourne à droite), -1 gardé à droite
        self.segment = None      # segment du chemin qu'on suivait (a, b)
        self.leave_along = 0.0   # où on a quitté le chemin (le long du segment)
        self.last_xy = None
        self.travelled = 0.0     # distance faite depuis le début du contournement
        self.gave_up = False

    def start(self, prox, pose, segment):
        # on garde l'obstacle du côté où il est le plus visible
        left, right = prox[0] + prox[1], prox[3] + prox[4]
        self.side = 1.0 if left >= right else -1.0
        self.segment = segment
        self.leave_along, _ = segment_position(pose, *segment)
        self.last_xy = (pose[0], pose[1])
        self.travelled = 0.0
        self.gave_up = False

    def step(self, prox, pose):
        self.travelled += math.hypot(pose[0] - self.last_xy[0], pose[1] - self.last_xy[1])
        self.last_xy = (pose[0], pose[1])

        # capteurs vus depuis le côté de l'obstacle : edge = celui du bord, inner = celui d'à côté
        if self.side > 0:
            edge, inner = prox[0], prox[1]
        else:
            edge, inner = prox[4], prox[3]
        center = prox[2]
        away = -self.side        # omega > 0 tourne à gauche, donc obstacle à gauche -> omega < 0

        # 3. Bug 2 : est-ce qu'on est retombé sur le chemin, plus loin, avec rien devant ?
        along, distance = segment_position(pose, *self.segment)
        nothing_ahead = max(prox[1:4]) < PROX_EXIT
        if along > self.leave_along + MIN_PROGRESS and distance < LINE_TOLERANCE and nothing_ahead:
            return 0.0, 0.0, True

        # 4. on tourne autour depuis trop longtemps
        if self.travelled > MAX_CONTOUR_MM:
            self.gave_up = True
            return 0.0, 0.0, True

        # 2. on longe l'obstacle
        if max(center, inner) > PROX_FRONT:
            v, omega = 0.0, away * OMEGA_TURN                          # obstacle devant : on pivote
        elif max(edge, inner, center) > PROX_EXIT:
            seen = max(edge, inner, center)
            v, omega = V_AVOID, away * K_FOLLOW * seen                 # on le voit encore : on s'écarte
        else:
            v, omega = V_AVOID, -away * V_AVOID / R_ARC                # on ne le voit plus : on tourne vers lui
        v_left, v_right = motion_model.v_omega_to_wheels(v, omega)
        return (*motion_model.limit_wheels(v_left, v_right), False)
