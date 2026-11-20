"""
Tests + visualisation for global_navigation.py.

Usage:
    python test_global_navigation.py            # runs the checks, saves + shows plots
    python test_global_navigation.py --no-show  # same, only saves PNGs (headless)
    pytest test_global_navigation.py            # tests only (plots are skipped)

Place this file next to global_navigation.py. The small sys.path block below
makes `interfaces.py` (project root) importable wherever the file is run from.
"""
import math
import sys
from pathlib import Path as FsPath

HERE = FsPath(__file__).resolve().parent
for p in (HERE, HERE.parent):          # this folder + project root
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import numpy as np

from interfaces import Environment, PoseEstimate
from global_navigation import (GlobalPlanner, point_strictly_inside,
                               segment_is_free)

MARGIN = 60.0


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


def path_length(wps):
    return sum(math.dist(a, b) for a, b in zip(wps, wps[1:]))


# Reference scene: a box and a triangle
SCENE = [
    [(300, 200), (500, 200), (500, 500), (300, 500)],
    [(650, 400), (800, 400), (725, 650)],
]


# --------------------------------------------------------------------------
# Tests (pytest-compatible, also run by __main__)
# --------------------------------------------------------------------------
def test_straight_line_when_free():
    env = make_env([], goal=(900, 450))
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


# --------------------------------------------------------------------------
# Visualisation
# --------------------------------------------------------------------------
def plot_plan(planner: GlobalPlanner, env: Environment, pose: PoseEstimate,
              path, title="Global plan", ax=None, show_graph=True):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon, Rectangle

    own_fig = ax is None
    if own_fig:
        _, ax = plt.subplots(figsize=(9, 7.2))

    # arena boundary (black limits) and usable area after margin
    ax.add_patch(Rectangle((0, 0), env.width_mm, env.height_mm, fill=False,
                           edgecolor="black", linewidth=2.5, label="Boundary"))
    xmin, ymin, xmax, ymax = planner.bounds
    ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, fill=False,
                           edgecolor="gray", linestyle=":", linewidth=1,
                           label="Robot-centre limit"))

    # obstacles: raw (solid) and inflated (dashed)
    for i, poly in enumerate(env.obstacles):
        ax.add_patch(MplPolygon(poly, closed=True, facecolor="#444",
                                edgecolor="black", alpha=0.85,
                                label="Obstacle" if i == 0 else None))
    for i, poly in enumerate(planner.polys):
        ax.add_patch(MplPolygon(poly, closed=True, fill=False,
                                edgecolor="tab:red", linestyle="--",
                                label="Inflated" if i == 0 else None))

    # visibility graph + nodes
    if show_graph:
        for i, nbrs in planner.adj.items():
            for j, _ in nbrs:
                if j > i:
                    a, b = planner.nodes[i], planner.nodes[j]
                    ax.plot([a[0], b[0]], [a[1], b[1]], color="tab:blue",
                            alpha=0.15, linewidth=0.8)
    if planner.nodes:
        xs, ys = zip(*planner.nodes)
        ax.scatter(xs, ys, s=18, color="tab:blue", zorder=3, label="Corners")

    # path
    if path.valid:
        xs, ys = zip(*path.waypoints)
        ax.plot(xs, ys, "-o", color="tab:green", linewidth=2.5, markersize=6,
                zorder=4, label=f"Path ({path_length(path.waypoints):.0f} mm, "
                                f"{len(path.waypoints)} pts)")
    else:
        ax.text(0.5, 0.5, "NO PATH (valid=False)", transform=ax.transAxes,
                ha="center", va="center", fontsize=18, color="tab:red",
                bbox=dict(facecolor="white", alpha=0.8))

    # start (with heading) and goal
    ax.plot(pose.x, pose.y, "s", color="orange", markersize=10, zorder=5, label="Start")
    ax.arrow(pose.x, pose.y, 40 * math.cos(pose.theta), 40 * math.sin(pose.theta),
             head_width=15, color="orange", zorder=5)
    if env.goal is not None:
        ax.plot(*env.goal, "*", color="gold", markeredgecolor="black",
                markersize=16, zorder=5, label="Goal")

    ax.set_xlim(-50, env.width_mm + 50)
    ax.set_ylim(-50, env.height_mm + 50)
    ax.set_aspect("equal")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")          # y up, origin bottom-left (world frame)
    ax.set_title(title)
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    ax.grid(alpha=0.2)
    return ax


def run_visual_scenarios(show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    scenarios = [
        ("Reference scene", SCENE, (900, 450), (100, 350)),
        ("Symmetric box (shorter side chosen)",
         [[(400, 300), (600, 300), (600, 500), (400, 500)]], (900, 450), (100, 400)),
        ("Unreachable goal (full-width wall)",
         [[(480, -100), (520, -100), (520, 900), (480, 900)]], (900, 450), (100, 400)),
        ("Start inside inflated zone",
         SCENE, (900, 450), (270, 350)),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    for ax, (name, obs, goal, start) in zip(axes.ravel(), scenarios):
        env = make_env(obs, goal=goal)
        planner = make_planner(env)
        pose = make_pose(*start)
        plot_plan(planner, env, pose, planner.plan(pose), title=name, ax=ax)
    fig.tight_layout()
    out = HERE / "plan_scenarios.png"
    fig.savefig(out, dpi=120)
    print(f"Saved {out}")
    if show:
        plt.show()


# --------------------------------------------------------------------------
if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  {name}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {name}  {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} tests passed")
    run_visual_scenarios(show="--no-show" not in sys.argv)
    sys.exit(1 if failed else 0)
