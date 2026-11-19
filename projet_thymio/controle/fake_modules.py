"""Faux modules caméra et planification, pour tester la boucle principale
avant que ceux de Sujith et Robin soient prêts.

Ils renvoient exactement les structures de interfaces.py : le jour où les vrais
modules sont prêts, on les branche à la place dans main.py sans rien changer d'autre.

    vision(t)          -> PoseMeasurement   (comme le fera le module de Sujith)
    plan(start, goal)  -> Path              (comme le fera le module de Robin)
"""

import random

from interfaces import Path, PoseMeasurement
from controle import motion_model

CAMERA_STD = (2.0, 2.0, 0.02)   # mm, mm, rad : bruit de la fausse caméra (même ordre que filtre/params.py)


class FakeCamera:
    """Caméra simulée : lit la vraie position du robot dans le simulateur et ajoute du bruit.

    hidden : liste d'intervalles (t_début, t_fin) en s pendant lesquels le QR code est caché.
    Robot soulevé (kidnapping) : la caméra ne le voit pas non plus.
    """

    def __init__(self, sim, hidden=(), seed=0):
        self.sim = sim
        self.hidden = list(hidden)
        self._rng = random.Random(seed)

    def __call__(self, t):
        if self.sim.lifted or any(t0 <= t < t1 for t0, t1 in self.hidden):
            return PoseMeasurement(t=t, visible=False)
        x, y, theta = self.sim.true_pose
        sx, sy, st = CAMERA_STD
        return PoseMeasurement(
            t=t, visible=True,
            x=x + self._rng.gauss(0.0, sx),
            y=y + self._rng.gauss(0.0, sy),
            theta=motion_model.wrap_angle(theta + self._rng.gauss(0.0, st)),
        )


class StartOnlyCamera:
    """Pour le vrai robot sans caméra : donne la position de départ une seule fois,
    puis « QR caché » pour toujours. Le filtre ne fait alors que prédire (odométrie)."""

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
    """Faux planificateur : ligne droite du départ au but (carte sans obstacle connu)."""
    return Path(t=0.0, waypoints=[tuple(start[:2]), tuple(goal)], valid=True)
