"""
Plot the planned path for a hand-defined scene.

World frame (as in the contract): origin = bottom-left corner of the black
boundary, x to the right, y up, everything in mm.
"""
import math
import sys
from pathlib import Path as FsPath

HERE = FsPath(__file__).resolve().parent
for p in (HERE, HERE.parent, HERE.parent.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import numpy as np

from interfaces import Environment, PoseEstimate
from planification.global_navigation import GlobalPlanner

# ======================================================================
# CONFIG - edit freely
# ======================================================================
AREA_WIDTH_MM = 900          # usable area inside the black boundary
AREA_HEIGHT_MM = 900

# Each obstacle = list of (x, y) corners in mm, in order (clockwise or not).
OBSTACLES = [
    [(200, 350), (450, 350), (450, 700), (200, 700)],      # rectangle
    [(650, 400), (800, 400), (725, 650)],                  # triangle
]

START = (100, 100)            # robot position (mm)
START_THETA = 0.0             # heading (rad), only used for the arrow
GOAL = (800, 800)             # goal (mm)

MARGIN_MM = 80                # robot radius + safety
ARC_STEP_DEG = 45             # corner roundness (smaller = rounder, more nodes)
SHOW_GRAPH = True             # draw the visibility graph (faint blue lines)
OUTPUT_PNG = HERE / "plot_path.png"
# ======================================================================


def path_length(wps):
    return sum(math.dist(a, b) for a, b in zip(wps, wps[1:]))


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


def main(show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    env = Environment(t=0.0, width_mm=AREA_WIDTH_MM, height_mm=AREA_HEIGHT_MM,
                      obstacles=OBSTACLES, goal=GOAL)
    planner = GlobalPlanner(margin_mm=MARGIN_MM, arc_step_deg=ARC_STEP_DEG)
    planner.set_environment(env)
    pose = PoseEstimate(t=0.0, x=START[0], y=START[1], theta=START_THETA,
                        P=np.eye(3), measurement_used=True)
    path = planner.plan(pose)

    print("valid:", path.valid)
    if path.valid:
        for i, (x, y) in enumerate(path.waypoints):
            print(f"  {i}: ({x:.1f}, {y:.1f})")
        print(f"length: {path_length(path.waypoints):.0f} mm")

    plot_plan(planner, env, pose, path, title="Global plan", show_graph=SHOW_GRAPH)
    plt.tight_layout()
    plt.savefig(OUTPUT_PNG, dpi=130)
    print(f"Saved {OUTPUT_PNG}")
    if show:
        plt.show()


if __name__ == "__main__":
    main(show="--no-show" not in sys.argv)