"""Tout ce qui parle au Thymio passe par ici (avec tdmclient, comme dans le lab).

Le reste du code travaille en mm/s, la conversion en unités Thymio se fait ici.

Utilisation :
    robot = Thymio()
    r = robot.read()               # r.v_left, r.v_right en mm/s, r.prox (7 valeurs), r.ground (2)
    robot.set_motors(50.0, 50.0)   # vitesse des roues en mm/s
    robot.sleep(0.1)               # attendre (et recevoir les nouvelles mesures)
    robot.close()                  # arrête les moteurs et libère le robot

Un seul programme peut contrôler le robot à la fois : fermer Aseba Studio,
VPL ou les notebooks avant de lancer un script.
"""

from dataclasses import dataclass
from typing import List

from controle import motion_model

GROUND_LIFTED_THRESHOLD = 100   # capteurs de sol : soulevé de 5 cm ça donne 63 et 42, posé sur la table 914 et 588

# Petit programme Aseba qu'on met sur le Thymio. Toutes les 50 ms il envoie lui même
# ses mesures à l'ordinateur (un événement qui s'appelle "data").
# Sans ça, Thymio Suite ne nous envoie les valeurs que toutes les 0,2 s et le robot
# réagissait en retard (virages qui dépassent, évitement trop tard).
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
    v_left: float       # vitesse mesurée de la roue gauche en mm/s
    v_right: float      # vitesse mesurée de la roue droite en mm/s
    prox: List[int]     # 7 capteurs : 0 à 4 devant (de gauche à droite), 5 et 6 derrière
    ground: List[int]   # 2 capteurs de sol (lumière qui revient du sol)


def is_lifted(reading):
    """Vrai si les 2 capteurs de sol ne voient plus le sol, donc le robot est soulevé.

    Attention sur du noir ça baisse aussi, c'est pour ça que la caméra confirme.
    """
    return max(reading.ground) < GROUND_LIFTED_THRESHOLD


class Thymio:
    def __init__(self):
        from tdmclient import ClientAsync, aw

        self._aw = aw
        self._client = ClientAsync()
        self._node = aw(self._client.wait_for_node())   # on attend de trouver le robot
        aw(self._node.lock())                           # on le prend pour nous
        self._data = None
        # on charge le petit programme qui envoie les mesures toutes les 50 ms
        aw(self._node.register_events([("data", 11)]))
        error = aw(self._node.compile(_PROGRAM))
        if error is not None:
            raise RuntimeError(f"programme Aseba refusé : {error}")
        self._node.add_events_received_listener(self._on_events)   # _on_events sera appelé à chaque envoi
        aw(self._node.watch(events=True))
        aw(self._node.run())
        while self._data is None:          # on attend la première mesure avant de continuer
            aw(self._client.sleep(0.05))
        self._leds = None

    def _on_events(self, node, events):
        """Appelé tout seul par tdmclient quand le robot envoie ses mesures : on garde la dernière."""
        if "data" in events:
            self._data = list(events["data"])

    def read(self):
        """Dernière mesure reçue (elle a au plus 50 ms)."""
        d = self._data
        return Reading(
            v_left=motion_model.thymio_to_mms(d[0]),
            v_right=motion_model.thymio_to_mms(d[1]),
            prox=d[2:9],
            ground=d[9:11],
        )

    def set_motors(self, v_left, v_right):
        """Vitesse des roues en mm/s.

        On envoie sans attendre que le robot réponde : attendre prenait 100 ms
        à chaque fois et ça faisait une boucle 2 fois trop lente.
        """
        self._node.send_set_variables({
            "motor.left.target": [motion_model.mms_to_thymio(v_left, is_left=True)],
            "motor.right.target": [motion_model.mms_to_thymio(v_right)],
        })

    def set_motor_units(self, left_units, right_units):
        """Vitesse directement en unités Thymio (seulement pour calibration.py)."""
        self._aw(self._node.set_variables({
            "motor.left.target": [int(left_units)],
            "motor.right.target": [int(right_units)],
        }))

    def set_leds(self, r, g, b):
        """Couleur des LEDs du dessus (0 à 32 pour chaque couleur). On n'envoie que si ça change."""
        color = [int(r), int(g), int(b)]
        if color != self._leds:
            self._node.send_set_variables({"leds.top": color})
            self._leds = color

    def sleep(self, dt):
        """Attendre dt secondes. Pendant ce temps tdmclient reçoit les mesures."""
        self._aw(self._client.sleep(max(dt, 0.001)))

    def close(self):
        # là on attend la réponse du robot, pour être sûr qu'il est arrêté avant de le lâcher
        self._aw(self._node.set_variables({"motor.left.target": [0], "motor.right.target": [0]}))
        self._aw(self._node.stop())
        self._aw(self._node.unlock())
