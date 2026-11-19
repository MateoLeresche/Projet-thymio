"""Fausse caméra et faux planificateur, pour tester main.py en attendant ceux de Sujith et Robin.

Ils renvoient exactement les mêmes structures que interfaces.py, donc le jour où
les vrais sont prêts on les met à la place dans main.py et le reste ne change pas.

    vision(t) donne un PoseMeasurement (comme fera la caméra de Sujith)
    plan(start, goal) donne un Path (comme fera le planificateur de Robin)
"""

import random

from interfaces import Path, PoseMeasurement
from controle import motion_model

CAMERA_STD = (2.0, 2.0, 0.02)   # bruit de la fausse caméra : 2 mm, 2 mm et 0,02 rad


class FakeCamera:
    """Caméra pour le simulateur : on prend la vraie position du robot et on ajoute du bruit.

    hidden : périodes (début, fin) en secondes où le QR code est caché.
    Si le robot est soulevé (kidnapping), la caméra ne le voit pas non plus.
    """

    def __init__(self, sim, hidden=(), seed=0):
        self.sim = sim
        self.hidden = list(hidden)
        self._rng = random.Random(seed)

    def __call__(self, t):
        if self.sim.lifted or any(t0 <= t < t1 for t0, t1 in self.hidden):
            return PoseMeasurement(t=t, visible=False)   # QR pas visible
        x, y, theta = self.sim.true_pose
        sx, sy, st = CAMERA_STD
        return PoseMeasurement(
            t=t, visible=True,
            x=x + self._rng.gauss(0.0, sx),
            y=y + self._rng.gauss(0.0, sy),
            theta=motion_model.wrap_angle(theta + self._rng.gauss(0.0, st)),
        )


class StartOnlyCamera:
    """Pour le vrai robot sans caméra : donne la position de départ une fois, puis plus rien.

    Le filtre ne fait alors que prédire avec la vitesse des roues (de l'odométrie).
    """

    def __init__(self, start_pose):
        self.start_pose = start_pose
        self.sent = False

    def __call__(self, t):
        if self.sent:
            return PoseMeasurement(t=t, visible=False)
        self.sent = True
        x, y, theta = self.start_pose
        return PoseMeasurement(t=t, visible=True, x=x, y=y, theta=theta)


def straight_line_plan(start, goal):
    """Faux planificateur : ligne droite du robot jusqu'au but (comme si la carte était vide)."""
    return Path(t=0.0, waypoints=[tuple(start[:2]), tuple(goal)], valid=True)
