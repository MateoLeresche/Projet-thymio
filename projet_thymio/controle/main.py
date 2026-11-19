"""Boucle principale du projet : caméra -> filtre -> chemin -> suivi + évitement -> moteurs.

À chaque pas (0,1 s) :
    1. lire le robot ; s'il est soulevé (kidnapping), l'arrêter, et une fois reposé
       repartir de zéro (nouveau filtre, nouveau chemin) ;
    2. filtre de Kalman (Matéo) : commande des roues + mesure caméra -> pose estimée ;
       arrêt si le filtre n'a pas encore de position ou devient trop incertain ;
    3. demander un chemin (Robin) au départ, après un évitement ou un kidnapping ;
    4. suivre le chemin, ou éviter un obstacle vu par les capteurs infrarouges ;
    5. envoyer les vitesses aux moteurs.

Les modules ne s'échangent que les structures de interfaces.py :
    vision(t) -> PoseMeasurement        plan(start, goal) -> Path
Tant que la caméra et la planification ne sont pas prêtes, on utilise les faux
modules de fake_modules.py. Pour brancher les vrais : changer `vision` et `plan` dans main().

    (depuis le dossier projet_thymio/)
    python -m controle.main --sim                       # tout simulé : obstacle, QR caché, kidnapping
    python -m controle.main --sim --plot data/main.png
    python -m controle.main --reel --goal 400 0         # vrai robot sans caméra (filtre = odométrie)
"""

import argparse
import math
import time

from filtre.filter_node import FilterNode
from controle import controller, local_avoidance, messages, thymio
from controle.fake_modules import FakeCamera, StartOnlyCamera, straight_line_plan

DT = 0.1            # s
MAX_TIME = 90.0     # s, sécurité

LED_FOLLOW = (0, 0, 32)      # bleu : suit le chemin
LED_AVOID = (32, 12, 0)      # orange : évite un obstacle
LED_STOP = (32, 0, 0)        # rouge : arrêté (soulevé, pas de position, filtre trop incertain, pas de chemin)
LED_DONE = (0, 32, 0)        # vert : arrivé


def run(robot, vision, plan, goal, clock, sim=None, scenario=None):
    """Boucle principale. Renvoie le journal [(t, pose estimée ou None, vraie pose ou None), ...]."""
    kalman = FilterNode()
    follower = controller.PathFollower()
    avoider = local_avoidance.LocalAvoidance()
    avoiding = False
    need_path = True
    lifted = False
    log = []
    t0 = clock()

    while clock() - t0 < MAX_TIME:
        step_start = time.monotonic()
        now = clock()
        if scenario is not None:
            scenario(now - t0, sim)
        reading = robot.read()

        def stop():
            robot.set_motors(0.0, 0.0)
            robot.set_leds(*LED_STOP)
            log.append((now - t0, None, sim.true_pose if sim else None))
            wait(robot, step_start, sim)

        # 1. Kidnapping vu par les capteurs de sol.
        if thymio.is_lifted(reading):
            if not lifted:
                print(f"t={now - t0:5.1f}s  robot soulevé -> arrêt")
            lifted = True
            stop()
            continue
        if lifted:
            print(f"t={now - t0:5.1f}s  robot reposé -> nouveau filtre (repart de la caméra) et nouveau chemin")
            lifted = False
            kalman = FilterNode()        # s'initialise sur la prochaine mesure caméra visible
            avoiding = False
            need_path = True

        # 2. Filtre : commande réellement appliquée (vitesses mesurées) + mesure caméra.
        kalman.on_control(messages.control_input(now, reading.v_left, reading.v_right))
        kalman.on_measurement(vision(now))
        estimate = kalman.step(now)
        if estimate is None or messages.must_stop(estimate):
            if estimate is not None and estimate.kidnapped:
                need_path = True
            stop()
            continue
        pose = messages.pose_tuple(estimate)

        # 3. Chemin : au départ, après un évitement, après un kidnapping.
        if need_path:
            points = messages.path_points(plan(pose, goal))
            if points is None:
                print(f"t={now - t0:5.1f}s  pas de chemin -> arrêt")
                stop()
                continue
            follower.set_path(points)
            need_path = False
            print(f"t={now - t0:5.1f}s  nouveau chemin : {len(points)} points")

        # 4. Suivi du chemin, remplacé par l'évitement quand un obstacle est trop près.
        v_left, v_right, done = follower.update(pose)
        if done:
            robot.set_motors(0.0, 0.0)
            robot.set_leds(*LED_DONE)
            log.append((now - t0, pose, sim.true_pose if sim else None))
            print(f"t={now - t0:5.1f}s  arrivé")
            break

        if not avoiding and local_avoidance.obstacle_detected(reading.prox):
            avoiding = True
            avoider.start(reading.prox, pose)
            print(f"t={now - t0:5.1f}s  obstacle -> évitement")

        led = LED_FOLLOW
        if avoiding:
            v_left, v_right, finished = avoider.step(reading.prox, pose)
            led = LED_AVOID
            if finished:
                avoiding = False
                need_path = True         # le point visé peut être juste derrière l'obstacle
                print(f"t={now - t0:5.1f}s  obstacle dépassé -> nouveau chemin")

        # 5. Agir.
        robot.set_motors(v_left, v_right)
        robot.set_leds(*led)
        log.append((now - t0, pose, sim.true_pose if sim else None))
        wait(robot, step_start, sim)

    return log


def wait(robot, step_start, sim):
    """Attendre la fin du pas (le simulateur avance de DT d'un coup)."""
    robot.sleep(DT if sim is not None else max(DT - (time.monotonic() - step_start), 0.001))


# --- Scénario de simulation ---
SIM_START = (100.0, 100.0, 0.0)
SIM_GOAL = (700.0, 500.0)
SIM_OBSTACLE = (340.0, 265.0, 20.0)      # gobelet sur la ligne droite, pas sur la carte
SIM_HIDDEN = [(3.0, 6.0)]                # QR caché pendant 3 s (pendant l'évitement)
SIM_KIDNAP = (8.0, 10.0, (150.0, 450.0, 0.0))   # soulevé à 8 s, reposé à 10 s à cette pose


def sim_scenario(t, sim):
    lift_t, down_t, new_pose = SIM_KIDNAP
    if lift_t <= t < down_t and not sim.lifted:
        sim.lift()
    if t >= down_t and sim.lifted:
        sim.put_down(new_pose)


def save_plot(log, filename, goal, obstacle=None, kidnap_pose=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib absent : pas de figure)")
        return
    fig, ax = plt.subplots(figsize=(7, 5))
    est = [e[1] for e in log if e[1] is not None]
    true = [e[2] for e in log if e[2] is not None]
    if true:
        ax.plot([p[0] for p in true], [p[1] for p in true], ".", ms=2, color="black", label="vraie position")
    ax.plot([p[0] for p in est], [p[1] for p in est], ".", ms=3, color="tab:blue", label="estimée par le filtre")
    ax.plot(*goal, "*", ms=14, color="green", label="but")
    if obstacle is not None:
        ax.add_patch(plt.Circle(obstacle[:2], obstacle[2], color="orange", label="obstacle imprévu"))
    if kidnap_pose is not None:
        ax.plot(*kidnap_pose[:2], "x", ms=10, color="red", label="reposé ici (kidnapping)")
    ax.set_aspect("equal")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.legend(fontsize=8)
    fig.savefig(filename, dpi=120, bbox_inches="tight")
    print(f"figure enregistrée : {filename}")


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="tout simulé")
    mode.add_argument("--reel", action="store_true", help="vrai robot, sans caméra")
    parser.add_argument("--goal", nargs=2, type=float, default=None, help="but x y en mm")
    parser.add_argument("--start", nargs=3, type=float, default=(0.0, 0.0, 0.0),
                        help="vrai robot : pose de départ x y theta (mm, mm, rad)")
    parser.add_argument("--plot", default=None, help="enregistrer la figure (ex. data/main.png)")
    args = parser.parse_args()

    if args.sim:
        from controle.simulator import SimThymio
        robot = SimThymio(start_pose=SIM_START, obstacles=[SIM_OBSTACLE], seed=1)
        sim, clock, scenario = robot, (lambda: robot.t), sim_scenario
        vision = FakeCamera(robot, hidden=SIM_HIDDEN)            # plus tard : la caméra de Sujith
        goal = tuple(args.goal) if args.goal else SIM_GOAL
    else:
        robot = thymio.Thymio()
        sim, clock, scenario = None, time.monotonic, None
        vision = StartOnlyCamera(tuple(args.start))               # plus tard : la caméra de Sujith
        goal = tuple(args.goal) if args.goal else (400.0, 0.0)
    plan = straight_line_plan                                     # plus tard : le planificateur de Robin

    try:
        log = run(robot, vision, plan, goal, clock, sim, scenario)
    finally:
        robot.close()

    if sim is not None:
        x, y, _ = sim.true_pose
        print(f"vraie position finale : {math.hypot(x - goal[0], y - goal[1]):.0f} mm du but")
        print("collision !" if sim.collides() else "aucune collision")
    if args.plot:
        save_plot(log, args.plot, goal, SIM_OBSTACLE if sim else None, SIM_KIDNAP[2] if sim else None)


if __name__ == "__main__":
    main()
