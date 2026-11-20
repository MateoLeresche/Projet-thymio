"""Tests du module controle (Matthieu), sans robot : simulateur controle/simulator.py."""
import math

import numpy as np

from interfaces import Path, PoseEstimate
from controle import controller, local_avoidance, messages, motion_model
from controle.demo_mouvement import run
from controle.simulator import SimThymio

SQUARE = [(0.0, 0.0), (400.0, 0.0), (400.0, 400.0), (0.0, 400.0), (0.0, 0.0)]


def test_wheels_v_omega_round_trip():
    v, omega = 50.0, 0.8
    assert np.allclose(motion_model.wheels_to_v_omega(*motion_model.v_omega_to_wheels(v, omega)), (v, omega))


def test_odometry_straight_and_rotation():
    assert np.allclose(motion_model.odometry_step(0, 0, 0, 100, 100, 1.0), (100, 0, 0))
    x, y, th = motion_model.odometry_step(50, 50, 0, -10, 10, 1.0)
    assert np.allclose((x, y), (50, 50)) and math.isclose(th, 20 / motion_model.WHEEL_BASE_MM)


def test_thymio_units_are_clipped():
    assert abs(motion_model.mms_to_thymio(10_000)) <= motion_model.MAX_THYMIO_UNITS


def test_target_behind_turns_in_place():
    v_left, v_right = controller.go_to_point((0, 0, 0), (-100, 0))
    assert math.isclose(v_left, -v_right, abs_tol=1e-9)


def test_square_without_obstacle():
    robot = SimThymio(start_pose=(0.0, 0.0, 0.0), obstacles=[], seed=1)
    _, collided, _ = run(robot, SQUARE, sim=robot)
    x, y, _ = robot.true_pose
    assert not collided and math.hypot(x, y) < 40.0


def test_obstacle_is_avoided():
    obstacle = (200.0, 10.0, 30.0)
    robot = SimThymio(start_pose=(0.0, 0.0, 0.0), obstacles=[obstacle], seed=1)
    _, collided, _ = run(robot, SQUARE, sim=robot)
    x, y, _ = robot.true_pose
    assert not collided and math.hypot(x, y) < 40.0


def test_avoidance_turns_away_from_left_obstacle():
    prox = [3000, 2000, 0, 0, 0, 0, 0]
    avoid = local_avoidance.LocalAvoidance()
    avoid.start(prox, (0, 0, 0))
    v_left, v_right, _ = avoid.step(prox, (0, 0, 0))
    assert v_left > v_right        # obstacle à gauche -> tourne à droite


def test_messages():
    est = PoseEstimate(t=0.0, x=10.0, y=20.0, theta=0.5, P=np.eye(3), measurement_used=True)
    assert messages.pose_tuple(est) == (10.0, 20.0, 0.5)
    assert not messages.must_stop(est)
    est.kidnapped = True
    assert messages.must_stop(est)
    assert messages.path_points(Path(t=0.0, waypoints=[(0, 0), (1, 2)], valid=True)) == [(0, 0), (1, 2)]
    assert messages.path_points(Path(t=0.0, waypoints=[], valid=False)) is None
    u = messages.control_input(1.0, 40.0, 60.0)
    assert math.isclose(u.v, 50.0) and math.isclose(u.omega, 20.0 / motion_model.WHEEL_BASE_MM)


def test_main_loop_simulated():
    """Boucle complète en simulation : obstacle imprévu, QR caché 3 s, kidnapping."""
    from controle import main
    robot = SimThymio(start_pose=main.SIM_START, obstacles=[main.SIM_OBSTACLE], seed=1)
    vision = main.FakeCamera(robot, hidden=main.SIM_HIDDEN)
    log = main.run(robot, vision, main.straight_line_plan, main.SIM_GOAL, lambda: robot.t, robot, main.sim_scenario)
    x, y, _ = robot.true_pose
    assert math.hypot(x - main.SIM_GOAL[0], y - main.SIM_GOAL[1]) < 40.0
    assert not robot.collides()
    assert any(pose is None for _, pose, _ in log)          # arrêté pendant le kidnapping


def test_same_image_given_once_to_filter(monkeypatch):
    """La caméra prend une image toutes les 0,3 s mais on l'appelle toutes les 0,1 s :
    le filtre ne doit recevoir chaque image qu'une fois."""
    from controle import main
    from filtre.filter_node import FilterNode
    received = []

    class CountingFilter(FilterNode):
        def on_measurement(self, m):
            received.append(m.t)
            super().on_measurement(m)

    monkeypatch.setattr(main, "FilterNode", CountingFilter)
    robot = SimThymio(start_pose=(0.0, 0.0, 0.0), obstacles=[], seed=1)
    camera = main.FakeCamera(robot)
    images = {}

    def slow_camera(t):
        t_image = round(math.floor(t / 0.3 + 1e-6) * 0.3, 3)   # instant de la dernière image prise
        if t_image not in images:
            images[t_image] = camera(t_image)
        return images[t_image]

    main.run(robot, slow_camera, main.straight_line_plan, (300.0, 0.0), lambda: robot.t, robot)
    assert len(received) > 5
    assert len(received) == len(set(received))      # jamais 2 fois la même image
