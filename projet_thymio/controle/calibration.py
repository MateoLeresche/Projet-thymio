"""Les mesures à faire sur le vrai robot pour régler motion_model.py, thymio.py et local_avoidance.py.

    python -m controle.calibration vitesse    # SPEED_FACTOR, LEFT_CORRECTION, WHEEL_SPEED_STD_MMS
    python -m controle.calibration rotation   # WHEEL_BASE_MM
    python -m controle.calibration prox       # PROX_ENTER, PROX_EXIT
    python -m controle.calibration sol        # GROUND_LIFTED_THRESHOLD

À lancer depuis le dossier projet_thymio/.

Faire chaque mesure plusieurs fois et garder la moyenne. Noter les résultats
dans mesures_reelles.md, ils vont dans le rapport (le prof veut des valeurs mesurées).

La vitesse et la rotation se mesurent à la vitesse où le robot roule vraiment
(environ 80 mm/s, soit 227 unités) : on a vu que le facteur change avec la vitesse.
"""

import math
import statistics
import sys

from controle import motion_model
from controle.thymio import Thymio

DT = 0.1


def measure_speed(robot):
    units, duration = 227, 4.5   # 227 unités = environ 80 mm/s, ça fait environ 35 cm
    input(f"Marquer la position de départ du robot (avant de la coque), puis Entrée. "
          f"Il va avancer {duration:.1f} s à la consigne {units}.")
    left, right = [], []
    robot.set_motor_units(units, units)
    for _ in range(int(duration / DT)):
        robot.sleep(DT)
        r = robot.read()
        left.append(r.v_left / motion_model.SPEED_FACTOR)    # on remet en unités Thymio
        right.append(r.v_right / motion_model.SPEED_FACTOR)
    robot.set_motor_units(0, 0)

    distance = float(input("Distance parcourue en mm ? "))
    mean_units = statistics.mean(left[5:] + right[5:])        # on enlève le démarrage (les 5 premières mesures)
    factor = distance / duration / mean_units
    std_units = statistics.mean([statistics.stdev(left[5:]), statistics.stdev(right[5:])])
    print(f"\nvitesse mesurée moyenne : {mean_units:.1f} unités pour une consigne de {units}")
    print(f"SPEED_FACTOR        = {factor:.4f}   (mm/s par unité)")
    print(f"WHEEL_SPEED_STD_MMS = {std_units * factor:.2f}   (écart-type d'une roue, pour le filtre)")
    print("Si le robot a dévié vers la GAUCHE : augmenter LEFT_CORRECTION (ex. 1.02) et recommencer.")
    print("S'il a dévié vers la DROITE : la diminuer (ex. 0.98).")


def measure_rotation(robot):
    units, duration = 227, 8.0   # environ 2 tours
    input(f"Marquer l'orientation du robot (trait au sol), puis Entrée. "
          f"Il va tourner sur place {duration:.0f} s.")
    theta = 0.0   # angle calculé avec l'empattement actuel (on ne le remet pas entre -pi et pi pour compter les tours)
    robot.set_motor_units(-units, units)
    for _ in range(int(duration / DT)):
        robot.sleep(DT)
        r = robot.read()
        _, omega = motion_model.wheels_to_v_omega(r.v_left, r.v_right)
        theta += omega * DT
    robot.set_motor_units(0, 0)

    computed = math.degrees(theta)
    real = float(input(f"Angle calculé : {computed:.0f}°. Angle réellement tourné en degrés "
                       f"(compter les tours : 2 tours et demi = 900) ? "))
    wheel_base = motion_model.WHEEL_BASE_MM * computed / real
    print(f"\nWHEEL_BASE_MM = {wheel_base:.1f}   (avant : {motion_model.WHEEL_BASE_MM})")


def show_prox(robot):
    print("Placer un obstacle (de la couleur de ceux de la démo) à 1, 2, 4, 6, 8, 10 cm devant")
    print("chaque capteur et noter la valeur. Ctrl+C pour arrêter.")
    print("PROX_ENTER = valeur à la distance où on veut commencer à éviter (1000 avec l'obstacle de la démo, vu vers 6 cm),")
    print("PROX_EXIT  = valeur quand l'obstacle ne gêne plus (environ 10 cm).\n")
    try:
        while True:
            robot.sleep(0.2)
            r = robot.read()
            print("avant :", " ".join(f"{p:5d}" for p in r.prox[:5]), "  arrière :", r.prox[5:])
    except KeyboardInterrupt:
        pass


def show_ground(robot):
    print("Noter les valeurs : robot sur la feuille blanche, sur une forme noire, puis soulevé.")
    print("GROUND_LIFTED_THRESHOLD = entre la valeur 'soulevé' et la plus petite valeur posé.")
    print("Ctrl+C pour arrêter.\n")
    try:
        while True:
            robot.sleep(0.2)
            print("sol :", robot.read().ground)
    except KeyboardInterrupt:
        pass


MODES = {"vitesse": measure_speed, "rotation": measure_rotation, "prox": show_prox, "sol": show_ground}


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in MODES:
        print(__doc__)
        return
    robot = Thymio()
    try:
        MODES[sys.argv[1]](robot)
    finally:
        robot.close()


if __name__ == "__main__":
    main()
