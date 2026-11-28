"""Boucle principale du projet : caméra, filtre, chemin, suivi + évitement, moteurs.

À chaque pas (toutes les 0,1 s) :
    1. on lit le robot. S'il est soulevé (kidnapping) on l'arrête, et quand il est
       reposé on repart de zéro (nouveau filtre, nouveau chemin)
    2. filtre de Kalman de Matéo : vitesse des roues + mesure caméra, ça donne la position.
       Si le filtre n'a pas encore de position ou n'est plus sûr de lui, on s'arrête
    3. on demande un chemin à Robin au départ et après un kidnapping
    4. on suit le chemin, ou on contourne un obstacle vu par les capteurs infrarouges
       jusqu'à retomber sur le chemin (Bug 2)
    5. on envoie les vitesses aux moteurs

Les modules ne s'échangent que les structures de interfaces.py :
    vision(t) donne un PoseMeasurement, plan(estimation, goal) donne un Path
    (vision peut redonner la dernière image : on regarde son t pour savoir si elle est nouvelle)
Le planificateur est celui de Robin (planification/global_navigation.py), la caméra et la
carte (un Environment) sont celles de Sujith (vision/realtime.py). En simulation on utilise
la fausse caméra et la fausse carte de fake_modules.py. Sur le vrai robot sans caméra
(--odometrie) on n'a pas de carte, donc on va en ligne droite.

    (à lancer depuis le dossier projet_thymio/)
    python -m controle.main --sim                       # tout simulé : obstacle, QR caché, kidnapping
    python -m controle.main --sim --plot data/main.png
    python -m controle.main --camera --goal 400 100     # caméra seule, sans Thymio
    python -m controle.main --reel --goal 400 100       # vrai robot avec caméra
    python -m controle.main --odometrie --goal 400 0    # vrai robot sans caméra (le filtre fait juste de l'odométrie)
"""

import argparse
import math
import time

from filtre.filter_node import FilterNode
from controle import controller, local_avoidance, messages, thymio
from controle.fake_modules import FakeCamera, StartOnlyCamera, fake_map, straight_line_plan
from planification.global_navigation import GlobalPlanner

DT = 0.1            # durée d'un pas en secondes

LED_FOLLOW = (0, 0, 32)      # bleu : suit le chemin
LED_AVOID = (32, 12, 0)      # orange : évite un obstacle
LED_STOP = (32, 0, 0)        # rouge : arrêté (soulevé, pas de position, filtre trop incertain, pas de chemin)
LED_DONE = (0, 32, 0)        # vert : arrivé

CAMERA_DEFAULT_GOAL = (400.0, 100.0)


def run(robot, vision, plan, goal, clock, sim=None, scenario=None, max_duration=None):
    """La boucle. Renvoie un journal [(t, pose estimée ou None, vraie pose ou None), ...] pour les figures."""
    kalman = FilterNode()
    follower = controller.PathFollower()
    avoider = local_avoidance.LocalAvoidance()
    avoiding = False
    need_path = True
    current_path = None
    lifted = False
    last_image_t = None   # instant de la dernière image donnée au filtre
    last_stop_reason = None
    goal_reached = False
    log = []
    step_times = []       # vraie durée de chaque pas, pour voir si la caméra ralentit la boucle
    max_front = 0         # plus grande valeur vue par les 5 capteurs avant pendant l'essai
    t0 = clock()

    while max_duration is None or clock() - t0 < max_duration:
        step_start = time.monotonic()
        if log:
            step_times.append(step_start - last_step_start)
        last_step_start = step_start
        now = clock()
        if scenario is not None:
            scenario(now - t0, sim)
        reading = robot.read()
        max_front = max(max_front, max(reading.prox[:5]))

        def stop():
            # on arrête le robot et on passe au pas suivant
            robot.set_motors(0.0, 0.0)
            robot.set_leds(*LED_STOP)
            log.append((now - t0, None, sim.true_pose if sim else None))
            wait(robot, step_start, sim)

        # 1. Kidnapping : les capteurs de sol ne voient plus le sol
        if thymio.is_lifted(reading):
            if not lifted:
                print(f"t={now - t0:5.1f}s  robot soulevé -> arrêt")
            lifted = True
            vision(now)  # garder la capture active sans intégrer une pose pendant le kidnapping
            if sim is None and hasattr(vision, "show_interface"):
                key = vision.show_interface(None, current_path, goal)
                if key == ord("q"):
                    robot.set_motors(0.0, 0.0)
                    robot.set_leds(*LED_STOP)
                    print("arrêt demandé")
                    break
            stop()
            continue
        if lifted:
            print(f"t={now - t0:5.1f}s  robot reposé -> nouveau filtre (repart de la caméra) et nouveau chemin")
            lifted = False
            kalman = FilterNode()        # nouveau filtre, il prendra la position de la prochaine image de la caméra
            avoiding = False
            need_path = True
            goal_reached = False

        # 2. Filtre : la vitesse mesurée des roues à chaque pas, et la caméra seulement si c'est une
        #    nouvelle image (si on redonne la même, le filtre la compte plusieurs fois et se croit trop sûr)
        kalman.on_control(messages.control_input(now, reading.v_left, reading.v_right))
        image = vision(now)
        if image is not None and image.t != last_image_t:
            kalman.on_measurement(image)
            last_image_t = image.t
        estimate = kalman.step(now)      # None tant que le filtre n'a pas vu le QR une première fois
        if estimate is None:
            reason = "aucune première mesure caméra"
            if reason != last_stop_reason:
                print(f"t={now - t0:5.1f}s  arrêt : {reason}")
                last_stop_reason = reason
            stop()
            continue
        if messages.must_stop(estimate):
            std_x = estimate.P[0][0] ** 0.5
            std_y = estimate.P[1][1] ** 0.5
            # on n'écrit le message qu'une fois par raison d'arrêt (sinon une ligne tous les 0,1 s)
            reason = "kidnapping vu par le filtre" if estimate.kidnapped else "filtre trop incertain"
            if reason != last_stop_reason:
                print(f"t={now - t0:5.1f}s  arrêt : {reason} (écart-type x={std_x:.0f} mm, y={std_y:.0f} mm)")
                last_stop_reason = reason
            if estimate.kidnapped:
                need_path = True
                goal_reached = False
            stop()
            continue
        last_stop_reason = None
        pose = messages.pose_tuple(estimate)

        # 3. Chemin : on en demande un au départ et après un kidnapping
        if need_path:
            current_path = plan(estimate, goal)
            points = messages.path_points(current_path)
            if points is None:
                print(f"t={now - t0:5.1f}s  pas de chemin -> arrêt")
                stop()
                continue
            follower.set_path(points)
            need_path = False
            print(f"t={now - t0:5.1f}s  nouveau chemin : {len(points)} points")

        if sim is None and hasattr(vision, "show_interface"):
            key = vision.show_interface(estimate, current_path, goal)
            if key == ord("q"):
                robot.set_motors(0.0, 0.0)
                robot.set_leds(*LED_STOP)
                print("arrêt demandé")
                break

        if goal_reached:
            robot.set_motors(0.0, 0.0)
            robot.set_leds(*LED_DONE)
            log.append((now - t0, pose, sim.true_pose if sim else None))
            wait(robot, step_start, sim)
            continue

        # 4. On suit le chemin, sauf si un obstacle est trop près : là c'est l'évitement qui commande
        v_left, v_right, done = follower.update(pose)
        if done:
            goal_reached = True
            robot.set_motors(0.0, 0.0)
            robot.set_leds(*LED_DONE)
            log.append((now - t0, pose, sim.true_pose if sim else None))
            print(f"t={now - t0:5.1f}s  arrivé")
            wait(robot, step_start, sim)
            continue

        if not avoiding and local_avoidance.obstacle_detected(reading.prox):
            avoiding = True
            avoider.start(reading.prox, pose, follower.segment(pose))
            print(f"t={now - t0:5.1f}s  obstacle -> contournement (capteurs avant : {list(reading.prox[:5])})")

        led = LED_FOLLOW
        if avoiding:
            avoid_left, avoid_right, finished = avoider.step(reading.prox, pose)
            if not finished:
                v_left, v_right = avoid_left, avoid_right   # pendant le contournement c'est l'évitement qui commande
                led = LED_AVOID
            elif avoider.gave_up:
                avoiding = False
                need_path = True         # on a tourné autour sans retrouver le chemin : on en redemande un
                print(f"t={now - t0:5.1f}s  chemin pas retrouvé -> nouveau chemin")
            else:
                avoiding = False         # on est retombé sur le chemin : on le reprend
                print(f"t={now - t0:5.1f}s  chemin retrouvé -> on le reprend")

        # 5. On envoie tout au robot
        robot.set_motors(v_left, v_right)
        robot.set_leds(*led)
        log.append((now - t0, pose, sim.true_pose if sim else None))
        wait(robot, step_start, sim)

    if max_duration is not None and clock() - t0 >= max_duration:
        robot.set_motors(0.0, 0.0)
        robot.set_leds(*LED_STOP)
    if sim is None:
        # si le max reste sous le seuil alors que le robot a touché l'obstacle, il faut baisser PROX_ENTER
        print(f"capteurs avant : max {max_front} pendant l'essai (seuil d'évitement {local_avoidance.PROX_ENTER})")
    if sim is None and step_times:
        # si les pas durent plus de 0,1 s le robot réagit en retard (obstacles vus trop tard, virages trop larges)
        mean_ms = 1000 * sum(step_times) / len(step_times)
        print(f"durée d'un pas : moyenne {mean_ms:.0f} ms, max {1000 * max(step_times):.0f} ms (il faudrait 100 ms)")
    return log


def wait(robot, step_start, sim):
    """Attendre la fin du pas de 0,1 s (le simulateur, lui, avance de 0,1 s d'un coup)."""
    robot.sleep(DT if sim is not None else max(DT - (time.monotonic() - step_start), 0.001))


# Scénario pour tester en simulation
SIM_START = (100.0, 100.0, 0.0)
SIM_GOAL = (700.0, 500.0)
SIM_ARENA = (1000.0, 700.0)              # taille de la zone (dans le cadre noir) en mm
SIM_MAP_OBSTACLE = (400.0, 560.0, 50.0)  # une boîte connue, qui est sur la carte
SIM_OBSTACLE = (340.0, 265.0, 20.0)      # un gobelet sur la ligne droite, qui n'est pas sur la carte
SIM_HIDDEN = [(3.0, 6.0)]                # QR caché entre 3 et 6 s (en plein contournement)
SIM_KIDNAP = (11.0, 13.0, (150.0, 450.0, 0.0))  # soulevé à 11 s (après le contournement), reposé à 13 s à cet endroit


def sim_scenario(t, sim):
    # on soulève le robot puis on le repose ailleurs, comme pendant la démo
    lift_t, down_t, new_pose = SIM_KIDNAP
    if lift_t <= t < down_t and not sim.lifted:
        sim.lift()
    if t >= down_t and sim.lifted:
        sim.put_down(new_pose)


def save_plot(log, filename, goal, obstacle=None, kidnap_pose=None, map_obstacle=None):
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
    if map_obstacle is not None:
        x, y, r = map_obstacle
        ax.add_patch(plt.Rectangle((x - r, y - r), 2 * r, 2 * r, color="gray", label="obstacle sur la carte"))
    if kidnap_pose is not None:
        ax.plot(*kidnap_pose[:2], "x", ms=10, color="red", label="reposé ici (kidnapping)")
    ax.set_aspect("equal")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.legend(fontsize=8, loc="lower right")
    fig.savefig(filename, dpi=120, bbox_inches="tight")
    print(f"figure enregistrée : {filename}")


def run_camera_test(vision, goal):
    """Teste la caméra, le filtre, l'environnement et le planificateur sans moteur."""
    kalman = FilterNode()
    planner = GlobalPlanner(margin_mm=100)
    environment = vision.environment(goal)
    if environment.goal is not None:
        goal = environment.goal
    planner.set_environment(environment)
    last_image_t = None
    last_visible = None
    last_heartbeat = time.monotonic()
    print("Mode caméra uniquement : aucun moteur ne sera envoyé")
    print("Recherche du cercle rouge plein...")

    while True:
        try:
            now = time.monotonic()
            image = vision(now)
        except KeyboardInterrupt:
            print("Arrêt demandé par Ctrl+C")
            break

        if image is not None and image.t != last_image_t:
            kalman.on_measurement(image)
            last_image_t = image.t

            if last_visible != image.visible:
                if image.visible:
                    print("QR ArUco détecté")
                else:
                    print("QR ArUco caché : prédiction seule")
                last_visible = image.visible

        kalman.on_control(messages.control_input(now, 0.0, 0.0))
        estimate = kalman.step(now)

        if estimate is not None:
            path = planner.plan(estimate, goal)
            key = vision.show_interface(estimate, path, goal)
            if key == ord("q"):
                print("Arrêt demandé par la touche q")
                break
            if now - last_heartbeat >= 2.0:
                print(
                    f"pose=({estimate.x:.1f}, {estimate.y:.1f}, {estimate.theta:.2f}) "
                    f"chemin={'valide' if path.valid else 'invalide'} "
                    f"camera={'mesure' if estimate.measurement_used else 'perdue'}"
                )
                last_heartbeat = now
        time.sleep(DT)


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="tout simulé")
    mode.add_argument("--camera", action="store_true", help="test de la caméra, du filtre et du planificateur sans Thymio")
    mode.add_argument("--reel", action="store_true", help="vrai robot avec caméra")
    mode.add_argument("--odometrie", action="store_true", help="vrai robot sans caméra, position de départ donnée par --start")
    parser.add_argument("--goal", nargs=2, type=float, default=None, help="but x y en mm")
    parser.add_argument("--start", nargs=3, type=float, default=(0.0, 0.0, 0.0),
                        help="--odometrie : pose de départ x y theta (mm, mm, rad)")
    parser.add_argument("--plot", default=None, help="enregistrer la figure (ex. data/main.png)")
    parser.add_argument("--prox-enter", type=int, default=None,
                        help="seuil d'évitement pour un autre Thymio ou obstacle (voir calibration prox), 1000 par défaut")
    args = parser.parse_args()
    if args.prox_enter is not None:
        local_avoidance.PROX_ENTER = args.prox_enter   # les capteurs changent d'un robot à l'autre

    if args.camera:
        goal = tuple(args.goal) if args.goal else CAMERA_DEFAULT_GOAL
        from vision.realtime import RealCamera
        vision = RealCamera()
        try:
            run_camera_test(vision, goal)
        finally:
            vision.close()
        return

    if args.sim:
        from controle.simulator import SimThymio
        robot = SimThymio(start_pose=SIM_START, obstacles=[SIM_MAP_OBSTACLE, SIM_OBSTACLE], seed=1)
        sim, clock, scenario = robot, (lambda: robot.t), sim_scenario
        vision = FakeCamera(robot, hidden=SIM_HIDDEN)            # à remplacer par la caméra de Sujith
        goal = tuple(args.goal) if args.goal else SIM_GOAL
        planner = GlobalPlanner(margin_mm=100)                                 # le planificateur de Robin
        planner.set_environment(fake_map(SIM_ARENA, [SIM_MAP_OBSTACLE], goal))   # carte : à remplacer par celle de Sujith
        plan = planner.plan
    elif args.odometrie:
        # vrai robot sans caméra : on donne la position de départ une fois, ensuite le filtre
        # ne fait que de l'odométrie. Pas de carte, donc ligne droite jusqu'au but
        sim, clock, scenario = None, time.monotonic, None
        robot = thymio.Thymio()
        vision = StartOnlyCamera(tuple(args.start))
        goal = tuple(args.goal) if args.goal else (400.0, 0.0)
        plan = straight_line_plan
        # sans caméra l'incertitude ne fait que grandir : on ne s'arrête pas pour ça pendant les essais
        messages.POSITION_STD_MAX_MM = float("inf")
    else:
        sim, clock, scenario = None, time.monotonic, None
        goal = tuple(args.goal) if args.goal else CAMERA_DEFAULT_GOAL
        from vision.realtime import RealCamera
        vision = RealCamera()
        robot = thymio.Thymio()

        planner = GlobalPlanner(margin_mm=100)
        environment = vision.environment(goal)
        if environment.goal is not None:
            goal = environment.goal
        planner.set_environment(environment)
        plan = planner.plan

    try:
        log = run(robot, vision, plan, goal, clock, sim, scenario)
    finally:
        robot.close()
        if hasattr(vision, "close"):     # seulement la vraie caméra
            vision.close()

    if sim is not None:
        x, y, _ = sim.true_pose
        print(f"vraie position finale : {math.hypot(x - goal[0], y - goal[1]):.0f} mm du but")
        print("collision !" if sim.collides() else "aucune collision")
    if args.plot:
        save_plot(log, args.plot, goal, SIM_OBSTACLE if sim else None, SIM_KIDNAP[2] if sim else None,
                  SIM_MAP_OBSTACLE if sim else None)


if __name__ == "__main__":
    main()
