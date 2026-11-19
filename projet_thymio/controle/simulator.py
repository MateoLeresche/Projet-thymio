"""Faux Thymio pour tester le contrôle et l'évitement sans robot.

SimThymio a les mêmes méthodes que thymio.Thymio (read, set_motors, set_leds,
sleep, close) : on remplace l'un par l'autre sans toucher au reste du code.

Ce qu'il simule (de façon grossière) :
    - la roue gauche un peu plus lente et du bruit sur les vitesses mesurées,
      pour que l'odométrie dérive comme sur le vrai robot ;
    - les 5 capteurs avant, face à des obstacles ronds ;
    - le kidnapping : lift() soulève le robot (capteurs de sol à ~20, il ne bouge
      plus), put_down(pose) le repose ailleurs.
Il ne remplace pas les essais sur le vrai robot.
"""

import math
import random

from controle import motion_model
from controle.thymio import Reading

LEFT_WHEEL_BIAS = 0.97        # la roue gauche va 3 % moins vite que la consigne
WHEEL_NOISE_STD_MMS = 3.0
GROUND_ON_PAPER = 900
GROUND_LIFTED = 20            # mesuré : 15 à 63 selon la hauteur
COMMAND_DELAY_STEPS = 3       # la consigne agit 3 pas plus tard (~0,3 s : communication + moteurs) ; reproduit le dépassement mesuré dans les virages

PROX_ANGLES_RAD = (0.64, 0.32, 0.0, -0.32, -0.64)   # capteur 0 = avant gauche
PROX_FRONT_OFFSET_MM = 55.0   # du milieu de l'axe des roues à l'avant du robot
# Courbe mesurée sur le vrai Thymio (capteur du milieu, boîte blanche, voir mesures_reelles.md) :
# (distance au bord avant en mm, valeur). Au-delà de PROX_RANGE_MM, le capteur renvoie 0.
PROX_CURVE = ((0.0, 4500), (20.0, 4290), (40.0, 3628), (60.0, 2845), (80.0, 2416),
              (100.0, 2051), (120.0, 1702))
PROX_RANGE_MM = 130.0         # vu à 12 cm, plus rien à 14 cm


def prox_value(gap):
    """Valeur du capteur pour un obstacle à `gap` mm du bord avant (interpolation de la courbe mesurée)."""
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
        self.true_pose = tuple(start_pose)   # vraie position (inconnue du robot réel)
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
        # Même arrondi et mêmes limites que le vrai robot.
        units_l = motion_model.mms_to_thymio(v_left, is_left=True)
        units_r = motion_model.mms_to_thymio(v_right)
        self._pending.append((motion_model.thymio_to_mms(units_l), motion_model.thymio_to_mms(units_r)))
        if len(self._pending) > COMMAND_DELAY_STEPS:
            self._cmd = self._pending.pop(0)

    def set_leds(self, r, g, b):
        pass

    def sleep(self, dt):
        """Fait avancer le monde de dt secondes."""
        real_left = self._cmd[0] * LEFT_WHEEL_BIAS
        real_right = self._cmd[1]
        if not self.lifted:                    # en l'air, les roues tournent dans le vide
            x, y, th = self.true_pose
            self.true_pose = motion_model.odometry_step(x, y, th, real_left, real_right, dt)
        # Le robot mesure ses vitesses avec du bruit.
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
        """Vrai si le robot (rayon environ 60 mm) touche un obstacle."""
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
