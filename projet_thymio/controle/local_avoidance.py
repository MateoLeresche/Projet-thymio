"""Évitement local : contourner un obstacle imprévu vu par les capteurs infrarouges.

Les obstacles imprévus ne sont pas sur la carte (la carte est faite une seule
fois au démarrage), donc seuls les capteurs de proximité les voient.

Fonctionnement :
    1. Déclenchement : un des 5 capteurs avant dépasse PROX_ENTER.
    2. Contournement façon Braitenberg (labs 3 et 4) : on tourne à l'opposé
       des capteurs qui voient l'obstacle. Pour le capteur du milieu (obstacle
       pile en face), le côté est choisi au début et gardé jusqu'à la fin,
       pour que le robot n'hésite pas devant l'obstacle.
    3. Sortie : tous les capteurs sous PROX_EXIT pendant CLEAR_STEPS pas,
       puis encore CLEAR_DISTANCE_MM tout droit pour dépasser l'obstacle.
    4. Ensuite, la boucle principale redemande un chemin depuis la position
       actuelle (sinon le point visé peut être juste derrière l'obstacle).

Capteurs : prox[0] = avant gauche ... prox[2] = milieu ... prox[4] = avant droit.
"""

import math

from controle import motion_model

# --- Réglages (MESURER les seuils : valeur des capteurs en fonction de la distance) ---
PROX_ENTER = 2400        # mesuré : ~8 cm devant le capteur du milieu (boîte blanche) -> évitement
PROX_EXIT = 300          # mesuré : sous ~1500 le capteur renvoie 0, donc < 300 = rien vu (au-delà de ~13 cm)
CLEAR_STEPS = 5          # pas de suite sans obstacle (5 x 0,1 s = 0,5 s)
CLEAR_DISTANCE_MM = 100.0  # tout droit après l'obstacle ; 200 envoyait le robot ~23 cm sur le côté (essai réel), 100 : ~15 cm, même marge en simulation
V_AVOID = 60.0           # mm/s, vitesse d'avance pendant l'évitement
K_PROX = 0.015           # (mm/s) par unité de capteur : force du virage ; 0,03 faisait pivoter le vrai robot de 90 à 128° (essais réels)
WEIGHTS = (1.0, 2.0, 0.0, -2.0, -1.0)   # gauche -> tourner à droite, droite -> tourner à gauche
CENTER_WEIGHT = 2.0                      # le capteur du milieu pousse du côté choisi


def obstacle_detected(prox):
    return max(prox[:5]) > PROX_ENTER


class LocalAvoidance:
    """
        avoid = LocalAvoidance()
        si obstacle_detected(prox) : avoid.start(prox, pose)
        puis à chaque pas :          v_left, v_right, fini = avoid.step(prox, pose)
        quand fini : redemander un chemin depuis la pose actuelle.
    """

    def __init__(self):
        self.side = 1.0
        self.clear_steps = 0
        self.clear_start = None

    def start(self, prox, pose):
        # +1 : on tourne à droite (obstacle plutôt à gauche), -1 : à gauche.
        left, right = prox[0] + prox[1], prox[3] + prox[4]
        self.side = 1.0 if left >= right else -1.0
        self.clear_steps = 0
        self.clear_start = None

    def step(self, prox, pose):
        front = [float(p) for p in prox[:5]]

        if max(front) < PROX_EXIT:
            # Plus rien devant : on avance tout droit pour dépasser l'obstacle.
            self.clear_steps += 1
            if self.clear_start is None:
                self.clear_start = (pose[0], pose[1])
            travelled = math.hypot(pose[0] - self.clear_start[0], pose[1] - self.clear_start[1])
            finished = self.clear_steps >= CLEAR_STEPS and travelled >= CLEAR_DISTANCE_MM
            return V_AVOID, V_AVOID, finished

        # Obstacle encore visible : Braitenberg.
        self.clear_steps = 0
        self.clear_start = None
        turn = K_PROX * (sum(w * p for w, p in zip(WEIGHTS, front)) + CENTER_WEIGHT * front[2] * self.side)
        # turn > 0 : roue gauche plus rapide -> le robot tourne à droite.
        v_left, v_right = V_AVOID + turn, V_AVOID - turn
        return (*motion_model.limit_wheels(v_left, v_right), False)
