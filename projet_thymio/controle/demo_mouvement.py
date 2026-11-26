"""Test du mouvement tout seul : le robot fait un carré de 40 cm, avec l'évitement.

Pas besoin de caméra ni de filtre, la position vient juste de l'odométrie
(motion_model.odometry_step). C'est ce script qu'on a utilisé pour régler les
virages et l'évitement. La vraie boucle du projet c'est main.py.

    (depuis le dossier projet_thymio/)
    python -m controle.demo_mouvement --sim              # faux robot avec un obstacle rond
    python -m controle.demo_mouvement --sim --no-obstacle
    python -m controle.demo_mouvement --sim --plot data/trajectoire.png
    python -m controle.demo_mouvement                    # vrai Thymio (posé à l'origine, regardant vers +x)

Sur le vrai robot l'odométrie toute seule se trompe un peu : quelques cm à la
fin du carré c'est normal, c'est ce que la caméra et le filtre vont corriger.
"""

import argparse
import math
import time

from controle import controller
from controle import local_avoidance
from controle import motion_model

DT = 0.1                      # un tour de boucle toutes les 0,1 s
MAX_TIME = 60.0               # au bout de 60 s on arrête, au cas où

# carré de 40 cm, on tourne à gauche à chaque coin et on revient au départ
PATH = [(0.0, 0.0), (400.0, 0.0), (400.0, 400.0), (0.0, 400.0), (0.0, 0.0)]
# obstacle rond (x, y, rayon) sur le premier côté, pour la simulation
SIM_OBSTACLE = (200.0, 10.0, 30.0)

LED_FOLLOW = (0, 0, 32)       # bleu : suit le chemin
LED_AVOID = (32, 12, 0)       # orange : évite un obstacle
LED_DONE = (0, 32, 0)         # vert : arrivé


def run(robot, path, sim=None):
    pose = path[0] + (0.0,)   # on suppose que le robot part du premier point, vers +x
    follower = controller.PathFollower(path)
    avoider = local_avoidance.LocalAvoidance()
    avoiding = False
    trajectory = []
    t = 0.0
    collided = False

    last = time.monotonic()
    while t < MAX_TIME:
        start = time.monotonic()
        # on prend le vrai temps écoulé depuis le dernier tour, pas 0,1 s pile :
        # au début un tour prenait 0,21 s et le robot comptait 2 fois trop peu de distance
        dt = DT if sim is not None else start - last
        last = start

        # 1. on lit le robot et on met à jour sa position avec l'odométrie
        reading = robot.read()
        pose = motion_model.odometry_step(*pose, reading.v_left, reading.v_right, dt)

        # 2. on décide : le suivi du chemin tourne toujours (comme ça il saute les points
        #    déjà atteints), mais si un obstacle est là c'est l'évitement qui commande
        v_left, v_right, done = follower.update(pose)
        if done:
            robot.set_motors(0.0, 0.0)
            robot.set_leds(*LED_DONE)
            print(f"t={t:5.1f}s  arrivé")
            break

        if not avoiding and local_avoidance.obstacle_detected(reading.prox):
            avoiding = True
            avoider.start(reading.prox, pose, follower.segment(pose))
            print(f"t={t:5.1f}s  obstacle -> contournement")

        led = LED_FOLLOW
        if avoiding:
            avoid_left, avoid_right, finished = avoider.step(reading.prox, pose)
            if not finished:
                v_left, v_right = avoid_left, avoid_right
                led = LED_AVOID
            else:
                avoiding = False         # retombé sur le carré (ou abandon) : on reprend le carré
                print(f"t={t:5.1f}s  fin du contournement -> retour au chemin")

        # 3. on envoie les vitesses au robot
        robot.set_motors(v_left, v_right)
        robot.set_leds(*led)

        if sim is not None:
            trajectory.append((pose, sim.true_pose))
            collided = collided or sim.collides()
        else:
            trajectory.append((pose, None))

        # 4. on attend la fin du tour de 0,1 s (le simulateur avance de 0,1 s d'un coup)
        robot.sleep(DT if sim is not None else max(DT - (time.monotonic() - start), 0.001))
        t += dt

    return trajectory, collided, pose


def save_plot(trajectory, path, filename, obstacle=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib absent : pas de figure)")
        return
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot([p[0] for p in path], [p[1] for p in path], "o--", color="purple", label="chemin")
    ax.plot([e[0][0] for e in trajectory], [e[0][1] for e in trajectory], color="green", label="odométrie")
    if trajectory and trajectory[0][1] is not None:
        ax.plot([e[1][0] for e in trajectory], [e[1][1] for e in trajectory], color="black",
                label="vraie position (simulation)")
    if obstacle is not None:
        ax.add_patch(plt.Circle(obstacle[:2], obstacle[2], color="orange", label="obstacle"))
    ax.set_aspect("equal")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.legend()
    fig.savefig(filename, dpi=120, bbox_inches="tight")
    print(f"figure enregistrée : {filename}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", help="faux robot au lieu du vrai Thymio")
    parser.add_argument("--no-obstacle", action="store_true", help="simulation sans obstacle")
    parser.add_argument("--plot", default=None, help="enregistrer la figure (ex. data/trajectoire.png)")
    args = parser.parse_args()

    obstacle = None
    if args.sim:
        from controle.simulator import SimThymio
        obstacle = None if args.no_obstacle else SIM_OBSTACLE
        robot = SimThymio(start_pose=(0.0, 0.0, 0.0), obstacles=[obstacle] if obstacle else [])
        sim = robot
    else:
        from controle.thymio import Thymio
        robot = Thymio()
        sim = None

    try:
        trajectory, collided, end_odo = run(robot, PATH, sim)
    finally:
        robot.close()

    goal = PATH[-1]
    print(f"erreur finale (odométrie) : {math.hypot(end_odo[0] - goal[0], end_odo[1] - goal[1]):.1f} mm")
    if sim is not None:
        true = sim.true_pose
        print(f"erreur finale (vraie position) : {math.hypot(true[0] - goal[0], true[1] - goal[1]):.1f} mm")
        print("collision !" if collided else "aucune collision")
    if args.plot:
        save_plot(trajectory, PATH, args.plot, obstacle)


if __name__ == "__main__":
    main()
