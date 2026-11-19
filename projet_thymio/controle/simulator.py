"""Faux Thymio pour tester sans le robot.

SimThymio a les mêmes fonctions que thymio.Thymio (read, set_motors, set_leds,
sleep, close), donc on peut mettre l'un ou l'autre sans changer le reste du code.

Ce qu'il fait (en gros) :
    la roue gauche va un peu moins vite et les vitesses mesurées ont du bruit,
    pour que l'odométrie se trompe un peu comme en vrai
    les 5 capteurs avant voient des obstacles ronds (avec la courbe mesurée sur notre robot)
    les commandes arrivent avec un peu de retard, comme en vrai
    le kidnapping : lift() soulève le robot, put_down(pose) le repose ailleurs
Ça ne remplace pas les tests sur le vrai robot.
"""

import math
import random

from controle import motion_model
from controle.thymio import Reading

LEFT_WHEEL_BIAS = 0.97        # la roue gauche va 3 % moins vite que ce qu'on demande
WHEEL_NOISE_STD_MMS = 3.0     # bruit sur la vitesse mesurée des roues
GROUND_ON_PAPER = 900         # capteur de sol posé sur la feuille
GROUND_LIFTED = 20            # capteur de sol quand on soulève le robot (on a mesuré 15 à 63)
COMMAND_DELAY_STEPS = 2       # la commande agit 2 pas plus tard (0,2 s), avec ça le simulateur dépasse dans les virages comme le vrai robot

PROX_ANGLES_RAD = (0.64, 0.32, 0.0, -0.32, -0.64)   # direction de chaque capteur avant, le 0 c'est à gauche
PROX_FRONT_OFFSET_MM = 55.0   # distance entre le milieu des roues et l'avant du robot
# Courbe qu'on a mesurée sur notre Thymio avec la boîte blanche (voir mesures_reelles.md) :
# (distance à l'avant du robot en mm, valeur du capteur). Plus loin que PROX_RANGE_MM ça donne 0.
PROX_CURVE = ((0.0, 4500), (20.0, 4290), (40.0, 3628), (60.0, 2845), (80.0, 2416),
              (100.0, 2051), (120.0, 1702))
PROX_RANGE_MM = 130.0         # le capteur voyait la boîte à 12 cm mais plus à 14 cm


def prox_value(gap):
    """Valeur du capteur pour un obstacle à gap mm de l'avant du robot (on lit la courbe mesurée)."""
    if gap >= PROX_RANGE_MM:
        return 0
    gap = max(gap, 0.0)
    for (d0, v0), (d1, v1) in zip(PROX_CURVE, PROX_CURVE[1:]):
        if gap <= d1:
            return int(v0 + (v1 - v0) * (gap - d0) / (d1 - d0))
    return PROX_CURVE[-1][1]


class SimThymio:
    def __init__(self, start_pose=(0.0, 0.0, 0.0), obstacles=(), seed=0):
        """obstacles : liste de (x, y, rayon) en mm."""
        self.true_pose = tuple(start_pose)   # la vraie position (le vrai robot, lui, ne la connaît pas)
        self.obstacles = list(obstacles)
        self.t = 0.0
        self._cmd = (0.0, 0.0)
        self._pending = []
        self._measured = (0.0, 0.0)
        self._rng = random.Random(seed)
        self.lifted = False

    def read(self):
        return Reading(
            v_left=self._measured[0],
            v_right=self._measured[1],
            prox=self._prox_values(),
            ground=[GROUND_LIFTED] * 2 if self.lifted else [GROUND_ON_PAPER] * 2,
        )

    def set_motors(self, v_left, v_right):
        # mêmes arrondis et mêmes limites que sur le vrai robot
        units_l = motion_model.mms_to_thymio(v_left, is_left=True)
        units_r = motion_model.mms_to_thymio(v_right)
        self._pending.append((motion_model.thymio_to_mms(units_l), motion_model.thymio_to_mms(units_r)))
        if len(self._pending) > COMMAND_DELAY_STEPS:
            self._cmd = self._pending.pop(0)

    def set_leds(self, r, g, b):
        pass

    def sleep(self, dt):
        """Fait avancer le temps de dt secondes."""
        real_left = self._cmd[0] * LEFT_WHEEL_BIAS
        real_right = self._cmd[1]
        if not self.lifted:                    # en l'air, les roues tournent dans le vide
            x, y, th = self.true_pose
            self.true_pose = motion_model.odometry_step(x, y, th, real_left, real_right, dt)
        # le robot mesure la vitesse de ses roues avec un peu de bruit
        self._measured = (
            real_left + self._rng.gauss(0.0, WHEEL_NOISE_STD_MMS),
            real_right + self._rng.gauss(0.0, WHEEL_NOISE_STD_MMS),
        )
        self.t += dt

    def close(self):
        self._cmd = (0.0, 0.0)

    def lift(self):
        """Kidnapping : on soulève le robot."""
        self.lifted = True

    def put_down(self, pose):
        """Kidnapping : on repose le robot à `pose` (x, y, theta)."""
        self.true_pose = tuple(pose)
        self.lifted = False

    def collides(self):
        """Vrai si le robot (environ 6 cm de rayon) touche un obstacle."""
        x, y, _ = self.true_pose
        return any(math.hypot(ox - x, oy - y) < r + 60.0 for ox, oy, r in self.obstacles)

    def _prox_values(self):
        values = [0] * 7   # les capteurs arrière (5, 6) ne sont pas simulés
        x, y, th = self.true_pose
        for i, angle in enumerate(PROX_ANGLES_RAD):
            dx, dy = math.cos(th + angle), math.sin(th + angle)
            nearest = math.inf
            for ox, oy, r in self.obstacles:
                cx, cy = ox - x, oy - y
                along = cx * dx + cy * dy
                off2 = cx * cx + cy * cy - along * along
                if along > 0.0 and off2 < r * r:
                    nearest = min(nearest, along - math.sqrt(r * r - off2))
            gap = nearest - PROX_FRONT_OFFSET_MM
            values[i] = prox_value(gap)
        return values
