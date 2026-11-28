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
    avoid.start(prox, (0, 0, 0), ((0.0, 0.0), (1000.0, 0.0)))
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
    """Boucle complète en simulation avec le planificateur de Robin : boîte sur la carte,
    obstacle imprévu, QR caché 3 s, kidnapping."""
    from controle import main
    robot = SimThymio(start_pose=main.SIM_START, obstacles=[main.SIM_MAP_OBSTACLE, main.SIM_OBSTACLE], seed=1)
    camera = main.FakeCamera(robot, hidden=main.SIM_HIDDEN)
    lifted_camera_calls = []

    def vision(t):
        if robot.lifted:
            lifted_camera_calls.append(t)
        return camera(t)

    planner = main.GlobalPlanner()
    planner.set_environment(main.fake_map(main.SIM_ARENA, [main.SIM_MAP_OBSTACLE], main.SIM_GOAL))
    log = main.run(robot, vision, planner.plan, main.SIM_GOAL, lambda: robot.t, robot, main.sim_scenario,
                   max_duration=90.0)
    x, y, _ = robot.true_pose
    assert math.hypot(x - main.SIM_GOAL[0], y - main.SIM_GOAL[1]) < 40.0
    assert not robot.collides()
    assert any(pose is None for _, pose, _ in log)          # arrêté pendant le kidnapping
    assert lifted_camera_calls                            # la caméra reste appelée pendant le kidnapping
    assert log[-1][0] >= 89.9                               # la boucle reste active après l'arrivée


def test_robot_resumes_after_kidnap_at_goal():
    from controle import main

    goal = (0.0, 0.0)
    robot = SimThymio(start_pose=(0.0, 0.0, 0.0), obstacles=[], seed=1)
    camera = main.FakeCamera(robot)
    plan_calls = []

    def plan(estimate, target):
        plan_calls.append((estimate.x, estimate.y))
        return main.straight_line_plan(estimate, target)

    def scenario(t, sim):
        if 0.5 <= t < 1.0 and not sim.lifted:
            sim.lift()
        elif t >= 1.0 and sim.lifted:
            sim.put_down((100.0, 0.0, 0.0))

    main.run(robot, camera, plan, goal, lambda: robot.t, robot, scenario, max_duration=8.0)

    assert len(plan_calls) >= 2                  # un nouveau chemin est calculé après le kidnapping
    assert math.hypot(robot.true_pose[0], robot.true_pose[1]) < 40.0


def test_prediction_remains_visible_when_camera_is_hidden_and_filter_stops(monkeypatch):
    from controle import main

    robot = SimThymio(start_pose=(0.0, 0.0, 0.0), obstacles=[], seed=1)
    camera = main.FakeCamera(robot)
    displayed = []

    class Vision:
        def __call__(self, t):
            if t > 0.0:
                from interfaces import PoseMeasurement
                return PoseMeasurement(t=t, visible=False)
            return camera(t)

        def show_interface(self, estimate, path, goal, robot_detected=None, prediction_active=False):
            displayed.append((estimate, robot_detected, prediction_active))
            return -1

    monkeypatch.setattr(messages, "POSITION_STD_MAX_MM", 0.0)
    main.run(robot, Vision(), main.straight_line_plan, (300.0, 0.0),
             lambda: robot.t, max_duration=2.5)

    assert displayed
    assert all(estimate is not None for estimate, _, _ in displayed)
    assert any(not detected for _, detected, _ in displayed)
    assert any(not estimate.measurement_used for estimate, _, _ in displayed)
    prediction_frames = [i for i, (_, _, active) in enumerate(displayed) if active]
    assert prediction_frames and prediction_frames[0] >= main.PREDICTION_AFTER_MISSED_FRAMES
    assert all(not active for _, _, active in displayed[:main.PREDICTION_AFTER_MISSED_FRAMES - 1])


def test_repeated_camera_frame_does_not_advance_prediction_threshold(monkeypatch):
    from controle import main
    from interfaces import PoseMeasurement

    robot = SimThymio(start_pose=(0.0, 0.0, 0.0), obstacles=[], seed=1)
    camera = main.FakeCamera(robot)
    displayed = []

    class Vision:
        def __call__(self, t):
            if t == 0.0:
                return camera(t)
            return PoseMeasurement(t=0.1, visible=False)

        def show_interface(self, estimate, path, goal, robot_detected=None, prediction_active=False):
            displayed.append(prediction_active)
            return -1

    monkeypatch.setattr(messages, "POSITION_STD_MAX_MM", 0.0)
    main.run(robot, Vision(), main.straight_line_plan, (300.0, 0.0),
             lambda: robot.t, max_duration=1.0)

    assert displayed
    assert not any(displayed)


def test_prediction_status_requires_confirmed_recovery_frames():
    from controle import main
    from interfaces import PoseMeasurement

    missed = 0
    active = False
    recovery = 0
    hidden = PoseMeasurement(t=0.0, visible=False)
    visible = PoseMeasurement(t=1.0, visible=True)

    for frame_index in range(main.PREDICTION_AFTER_MISSED_FRAMES):
        missed, active, recovery = main.update_prediction_status(
            hidden, True, missed, active, recovery,
        )
    assert active

    missed, active, recovery = main.update_prediction_status(visible, True, missed, active, recovery)
    assert active
    missed, active, recovery = main.update_prediction_status(visible, False, missed, active, recovery)
    assert active

    for frame_index in range(main.CAMERA_RECOVERY_FRAMES - 1):
        visible = PoseMeasurement(t=2.0 + frame_index, visible=True)
        missed, active, recovery = main.update_prediction_status(
            visible, True, missed, active, recovery,
        )
    assert not active


def test_prediction_uncertainty_ellipse_uses_position_covariance():
    from interfaces import PoseEstimate
    from vision.realtime import _position_uncertainty_ellipse

    estimate = PoseEstimate(
        t=0.0, x=0.0, y=0.0, theta=0.0,
        P=np.diag([100.0, 25.0, 0.01]),
        measurement_used=False,
    )

    axes, angle = _position_uncertainty_ellipse(estimate, pixels_per_mm=0.1)

    assert axes == (2, 1)
    assert math.isclose(angle, 0.0, abs_tol=1e-9)


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

    main.run(robot, slow_camera, main.straight_line_plan, (300.0, 0.0), lambda: robot.t, robot,
             max_duration=2.0)
    assert len(received) > 5
    assert len(received) == len(set(received))      # jamais 2 fois la même image



def test_contour_leaves_when_back_on_path():
    """Bug 2 : on reprend le chemin quand on retombe dessus, plus loin, avec rien devant."""
    avoid = local_avoidance.LocalAvoidance()
    nothing = [0, 0, 0, 0, 0, 0, 0]
    avoid.start([3000, 2000, 0, 0, 0, 0, 0], (100.0, 0.0, 0.0), ((0.0, 0.0), (1000.0, 0.0)))
    assert avoid.side > 0                                   # obstacle à gauche, on le garde à gauche
    assert not avoid.step(nothing, (120.0, -60.0, 0.0))[2]  # écarté du chemin : on continue
    assert avoid.step(nothing, (300.0, 5.0, 0.0))[2]        # retombé sur le chemin plus loin : fini
    assert not avoid.gave_up
