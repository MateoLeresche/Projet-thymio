"""
Unit tests for the global planner (global_navigation.py). Tests only, no plots.

To plot a scenario instead, see plot_path.py.
"""
import math
import sys
from pathlib import Path as FsPath
from planification.global_navigation import (GlobalPlanner, inflate_polygon,
                                            point_strictly_inside, segment_is_free,
                                            _dist_point_segment)

# Make `interfaces.py` (project root) and the planner importable from any
# working directory: add this folder, the project root, and one level above.
HERE = FsPath(__file__).resolve().parent
for p in (HERE, HERE.parent, HERE.parent.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import numpy as np

from interfaces import Environment, PoseEstimate

MARGIN = 80.0


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def make_env(obstacles, goal=(900, 450), w=1000, h=800) -> Environment:
    return Environment(t=0.0, width_mm=w, height_mm=h,
                       obstacles=obstacles, goal=goal)


def make_pose(x, y, t=0.0) -> PoseEstimate:
    return PoseEstimate(t=t, x=x, y=y, theta=0.0, P=np.eye(3),
                        measurement_used=True)


def make_planner(env, margin=MARGIN) -> GlobalPlanner:
    planner = GlobalPlanner(margin_mm=margin)
    planner.set_environment(env)
    return planner


# Reference scene: a box and a triangle
SCENE = [
    [(300, 200), (500, 200), (500, 500), (300, 500)],
    [(650, 400), (800, 400), (725, 650)],
]


# --------------------------------------------------------------------------
# Tests (pytest-compatible, also run by __main__)
# --------------------------------------------------------------------------
def test_straight_line_when_free():
    env = make_env([], goal=(900, 700))
    path = make_planner(env).plan(make_pose(100, 100))
    assert path.valid
    assert path.waypoints == [(100, 100), (900, 700)]


def test_detour_around_obstacles():
    env = make_env(SCENE)
    planner = make_planner(env)
    path = planner.plan(make_pose(100, 350))
    assert path.valid
    assert path.waypoints[0] == (100, 350)
    assert path.waypoints[-1] == (900, 450)
    assert len(path.waypoints) > 2               # direct line is blocked
    # every segment stays out of the inflated obstacles
    for a, b in zip(path.waypoints, path.waypoints[1:]):
        assert segment_is_free(a, b, planner.polys)


def test_path_stays_inside_arena_margin():
    env = make_env(SCENE)
    path = make_planner(env).plan(make_pose(100, 350))
    for x, y in path.waypoints:
        assert MARGIN - 1e-6 <= x <= 1000 - MARGIN + 1e-6 or (x, y) in [(100, 350), (900, 450)]
        assert MARGIN - 1e-6 <= y <= 800 - MARGIN + 1e-6


def test_no_aligned_waypoints():
    env = make_env(SCENE)
    wps = make_planner(env).plan(make_pose(100, 350)).waypoints
    for a, b, c in zip(wps, wps[1:], wps[2:]):
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        assert abs(cross) > 1e-3


def test_path_is_shortest_among_two_detours():
    # symmetric box centred on the start-goal line: both detours have equal length,
    # moving the goal slightly off-centre must pick the shorter side.
    box = [[(400, 300), (600, 300), (600, 500), (400, 500)]]
    env = make_env(box, goal=(900, 450))
    path = make_planner(env).plan(make_pose(100, 400))
    assert path.valid
    # goal is above start -> the upper detour (through y > 500 + margin) is shorter
    assert any(y > 500 for _, y in path.waypoints[1:-1])


def test_inflation_never_closer_than_margin_nor_much_farther():
    tri = [(650, 400), (800, 400), (725, 650)]
    inf = inflate_polygon(tri, MARGIN, 45.0)
    for q in inf:                                   # every node at >= margin, <= margin/cos(22.5 deg)
        d = min(_dist_point_segment(q, tri[i], tri[(i + 1) % 3]) for i in range(3))
        assert MARGIN - 1e-6 <= d <= MARGIN / math.cos(math.radians(22.5)) + 1e-6
    assert max(y for _, y in inf) < 650 + 1.1 * MARGIN   # no spike above the apex


def test_goal_inside_obstacle_is_invalid():
    env = make_env(SCENE, goal=(400, 350))        # inside the box
    path = make_planner(env).plan(make_pose(100, 350))
    assert not path.valid


def test_goal_inside_inflated_zone_is_invalid():
    env = make_env(SCENE, goal=(530, 350))        # 30 mm from the box < margin
    path = make_planner(env).plan(make_pose(100, 350))
    assert not path.valid


def test_goal_outside_arena_is_invalid():
    env = make_env(SCENE, goal=(1200, 400))
    path = make_planner(env).plan(make_pose(100, 350))
    assert not path.valid


def test_no_goal_is_invalid():
    env = make_env(SCENE, goal=None)
    path = make_planner(env).plan(make_pose(100, 350))
    assert not path.valid


def test_unreachable_goal_is_invalid():
    wall = [[(480, -100), (520, -100), (520, 900), (480, 900)]]   # spans the arena
    env = make_env(wall, goal=(900, 400))
    path = make_planner(env).plan(make_pose(100, 400))
    assert not path.valid


def test_plan_before_set_environment_is_invalid():
    path = GlobalPlanner(MARGIN).plan(make_pose(0, 0))
    assert not path.valid


def test_start_inside_inflated_zone_still_plans():
    env = make_env(SCENE)
    planner = make_planner(env)
    start = (270, 350)                                  # 30 mm from the box
    assert any(point_strictly_inside(start, p) for p in planner.polys)
    path = planner.plan(make_pose(*start))
    assert path.valid
    # must never cross the REAL obstacles (only the inflated margin may be relaxed)
    for a, b in zip(path.waypoints, path.waypoints[1:]):
        assert segment_is_free(a, b, env.obstacles)


def test_path_timestamp_follows_pose():
    env = make_env(SCENE)
    path = make_planner(env).plan(make_pose(100, 350, t=12.34))
    assert path.t == 12.34


def test_replan_after_kidnapping_uses_new_pose():
    env = make_env(SCENE)
    planner = make_planner(env)
    p1 = planner.plan(make_pose(100, 350))
    p2 = planner.plan(make_pose(900, 100))              # kidnapped elsewhere
    assert p1.valid and p2.valid
    assert p2.waypoints[0] == (900, 100)
    assert p2.waypoints != p1.waypoints