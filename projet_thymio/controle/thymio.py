"""Communication avec le Thymio via tdmclient (voir le lab « Control your Thymio in Python »).

C'est le SEUL fichier qui parle au robot. Tout le reste du code travaille en mm/s ;
la conversion en unités Thymio se fait ici, avec motion_model.

Utilisation :
    robot = Thymio()
    r = robot.read()               # r.v_left, r.v_right (mm/s), r.prox (7 valeurs), r.ground (2)
    robot.set_motors(50.0, 50.0)   # mm/s
    robot.sleep(0.1)               # attendre ET recevoir les nouvelles valeurs (envoyées par le robot toutes les 50 ms)
    robot.close()                  # arrête les moteurs et libère le robot

Un seul programme peut contrôler le robot à la fois : fermer les notebooks qui
l'ont verrouillé (Thymio Suite, Jupyter) avant de lancer un script.
"""

from dataclasses import dataclass
from typing import List

from controle import motion_model

GROUND_LIFTED_THRESHOLD = 100   # mesuré : soulevé de 5 cm -> 63 et 42, posé sur la table -> 914 et 588

# Petit programme Aseba chargé sur le Thymio : toutes les 50 ms, il envoie lui-même
# ses mesures à l'ordinateur (événement « data »). Sans lui, le Thymio Device Manager
# ne transmet les variables qu'environ toutes les 0,2 s (mesuré), ce qui retardait
# les réactions du robot (virages qui dépassent, évitement trop tardif).
_PROGRAM = """
var data[11]
timer.period[0] = 50
onevent timer0
    data[0] = motor.left.speed
    data[1] = motor.right.speed
    data[2:8] = prox.horizontal
    data[9:10] = prox.ground.reflected
    emit data data
"""


@dataclass
class Reading:
    v_left: float       # vitesse mesurée de la roue gauche, mm/s
    v_right: float      # vitesse mesurée de la roue droite, mm/s
    prox: List[int]     # 7 capteurs horizontaux : 0 à 4 devant (de gauche à droite), 5 et 6 derrière
    ground: List[int]   # 2 capteurs de sol (lumière réfléchie)


def is_lifted(reading):
    """Vrai si les deux capteurs de sol ne voient plus rien : le robot est soulevé.

    Attention : sur du papier noir, la valeur baisse aussi. Le kidnapping se
    confirme en vérifiant pendant plusieurs pas de suite (et avec la caméra).
    """
    return max(reading.ground) < GROUND_LIFTED_THRESHOLD


class Thymio:
    def __init__(self):
        from tdmclient import ClientAsync, aw

        self._aw = aw
        self._client = ClientAsync()
        self._node = aw(self._client.wait_for_node())
        aw(self._node.lock())
        self._data = None
        aw(self._node.register_events([("data", 11)]))
        error = aw(self._node.compile(_PROGRAM))
        if error is not None:
            raise RuntimeError(f"programme Aseba refusé : {error}")
        self._node.add_events_received_listener(self._on_events)
        aw(self._node.watch(events=True))
        aw(self._node.run())
        while self._data is None:          # attendre les premières mesures
            aw(self._client.sleep(0.05))
        self._leds = None

    def _on_events(self, node, events):
        """Appelé par tdmclient à chaque événement reçu : on garde la dernière mesure."""
        if "data" in events:
            self._data = list(events["data"])

    def read(self):
        """Dernière mesure envoyée par le robot (au plus 50 ms d'âge)."""
        d = self._data
        return Reading(
            v_left=motion_model.thymio_to_mms(d[0]),
            v_right=motion_model.thymio_to_mms(d[1]),
            prox=d[2:9],
            ground=d[9:11],
        )

    def set_motors(self, v_left, v_right):
        """Consignes des roues en mm/s.

        Envoi sans attendre la confirmation du robot : attendre prend environ
        100 ms par appel (mesuré), ce qui doublait la durée de la boucle.
        """
        self._node.send_set_variables({
            "motor.left.target": [motion_model.mms_to_thymio(v_left, is_left=True)],
            "motor.right.target": [motion_model.mms_to_thymio(v_right)],
        })

    def set_motor_units(self, left_units, right_units):
        """Consignes brutes en unités Thymio (seulement pour la calibration)."""
        self._aw(self._node.set_variables({
            "motor.left.target": [int(left_units)],
            "motor.right.target": [int(right_units)],
        }))

    def set_leds(self, r, g, b):
        """LEDs du dessus, chaque couleur de 0 à 32. N'envoie rien si la couleur ne change pas."""
        color = [int(r), int(g), int(b)]
        if color != self._leds:
            self._node.send_set_variables({"leds.top": color})
            self._leds = color

    def sleep(self, dt):
        """Attendre dt secondes. tdmclient a besoin de cet appel pour recevoir les valeurs."""
        self._aw(self._client.sleep(max(dt, 0.001)))

    def close(self):
        # Ici on attend la confirmation : le robot doit être arrêté avant de le libérer.
        self._aw(self._node.set_variables({"motor.left.target": [0], "motor.right.target": [0]}))
        self._aw(self._node.stop())
        self._aw(self._node.unlock())
