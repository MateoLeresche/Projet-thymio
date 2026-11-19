"""Évitement local : contourner un obstacle qu'on n'avait pas prévu.

Les obstacles ajoutés pendant la démo ne sont pas sur la carte (la carte est
faite une seule fois au début), donc seuls les capteurs infrarouges avant les voient.

Comment ça marche :
    1. Un des 5 capteurs avant dépasse PROX_ENTER : on passe en évitement.
    2. On tourne du côté opposé à l'obstacle, comme Braitenberg dans les labs 3 et 4.
       Si l'obstacle est pile en face (capteur du milieu), on choisit un côté au début
       et on le garde, sinon le robot hésite devant l'obstacle.
    3. Quand les capteurs ne voient plus rien pendant 5 pas, on avance encore 10 cm
       tout droit pour bien dépasser l'obstacle.
    4. Après, main.py redemande un chemin depuis là où est le robot.

Capteurs : prox[0] avant gauche, prox[2] milieu, prox[4] avant droit.
"""

import math

from controle import motion_model

# Réglages, tous testés sur le vrai robot (voir mesures_reelles.md)
PROX_ENTER = 2400        # valeur du capteur quand l'obstacle est à environ 8 cm : on commence à éviter
PROX_EXIT = 300          # en dessous le capteur ne voit plus rien (il renvoie 0 au delà de 13 cm environ)
CLEAR_STEPS = 5          # il faut 5 pas de suite sans rien voir (0,5 s) pour dire que c'est dégagé
CLEAR_DISTANCE_MM = 100.0  # puis 10 cm tout droit (avec 20 cm le robot partait trop loin sur le côté)
V_AVOID = 60.0           # vitesse d'avance pendant l'évitement, en mm/s
K_PROX = 0.015           # force du virage (avec 0,03 le vrai robot pivotait presque à l'envers)
WEIGHTS = (1.0, 2.0, 0.0, -2.0, -1.0)   # obstacle à gauche on tourne à droite, obstacle à droite on tourne à gauche
CENTER_WEIGHT = 2.0                      # le capteur du milieu fait tourner du côté choisi au début


def obstacle_detected(prox):
    """Vrai si un des 5 capteurs avant voit un obstacle trop proche."""
    return max(prox[:5]) > PROX_ENTER


class LocalAvoidance:
    """Utilisation :
        avoid = LocalAvoidance()
        si obstacle_detected(prox) : avoid.start(prox, pose)
        puis à chaque pas : v_left, v_right, fini = avoid.step(prox, pose)
        quand fini vaut True on redemande un chemin.
    """

    def __init__(self):
        self.side = 1.0          # +1 on tourne à droite, -1 à gauche
        self.clear_steps = 0     # nombre de pas de suite où on ne voit plus rien
        self.clear_start = None  # endroit où on a arrêté de voir l'obstacle

    def start(self, prox, pose):
        # on regarde de quel côté l'obstacle est le plus visible et on part de l'autre côté
        left, right = prox[0] + prox[1], prox[3] + prox[4]
        self.side = 1.0 if left >= right else -1.0
        self.clear_steps = 0
        self.clear_start = None

    def step(self, prox, pose):
        front = [float(p) for p in prox[:5]]

        if max(front) < PROX_EXIT:
            # plus rien devant : on va tout droit pour dépasser l'obstacle
            self.clear_steps += 1
            if self.clear_start is None:
                self.clear_start = (pose[0], pose[1])
            travelled = math.hypot(pose[0] - self.clear_start[0], pose[1] - self.clear_start[1])
            finished = self.clear_steps >= CLEAR_STEPS and travelled >= CLEAR_DISTANCE_MM
            return V_AVOID, V_AVOID, finished

        # on voit encore l'obstacle : Braitenberg, chaque capteur pousse le virage d'un côté
        self.clear_steps = 0
        self.clear_start = None
        turn = K_PROX * (sum(w * p for w, p in zip(WEIGHTS, front)) + CENTER_WEIGHT * front[2] * self.side)
        # si turn > 0 la roue gauche va plus vite donc le robot tourne à droite
        v_left, v_right = V_AVOID + turn, V_AVOID - turn
        return (*motion_model.limit_wheels(v_left, v_right), False)
