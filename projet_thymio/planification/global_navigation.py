"""
Global navigation (Planification) - Projet Thymio.

Visibility graph built on the corners of the inflated obstacles + Dijkstra.

Contract (CONTRAT_INTERFACES.md v0.1):
  - Input : Environment (Vision), goal (Vision), PoseEstimate (Filtre)
  - Output: Path(waypoints in mm, valid)
  - Obstacles arrive in mm WITHOUT margin -> inflation is done here.
  - The black limits are treated as obstacles -> arena rectangle is shrunk by the margin.
  - Aligned waypoints are removed.
  - Kidnapping: call plan() again when PoseEstimate.kidnapped is True.

Frame: world origin = bottom-left corner of the black boundary, so the free
arena is [0, width_mm] x [0, height_mm]. The goal comes from Environment.goal.
"""
from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]
Polygon = List[Point]

EPS = 1e-9

# --------------------------------------------------------------------------
# Interfaces (shared contract, see interfaces.py)
# --------------------------------------------------------------------------
from interfaces import Environment, Path, PoseEstimate


def _obstacles_of(env: Environment) -> List[Polygon]:
    return [[(float(x), float(y)) for x, y in poly] for poly in env.obstacles]


def _bounds_of(env: Environment) -> Tuple[float, float, float, float]:
    # World origin = bottom-left corner of the black boundary
    return (0.0, 0.0, float(env.width_mm), float(env.height_mm))


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------
def _cross(o: Point, a: Point, b: Point) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _signed_area(poly: Polygon) -> float:
    return 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1]
                     - poly[(i + 1) % len(poly)][0] * poly[i][1]
                     for i in range(len(poly)))


def _dist_point_segment(p: Point, a: Point, b: Point) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    if L2 < EPS:
        return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.dist(p, (a[0] + t * dx, a[1] + t * dy))


def point_strictly_inside(p: Point, poly: Polygon, tol: float = 1e-6) -> bool:
    """True if p is inside poly and NOT on its boundary (within tol)."""
    n = len(poly)
    for i in range(n):
        if _dist_point_segment(p, poly[i], poly[(i + 1) % n]) < tol:
            return False
    inside = False
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if (a[1] > p[1]) != (b[1] > p[1]):
            x_int = a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if x_int > p[0]:
                inside = not inside
    return inside


def inflate_polygon(poly: Polygon, margin: float, arc_step_deg: float = 45.0) -> Polygon:
    """Offset a polygon outward by `margin` (round buffer).

    - Convex corners become circular arcs, approximated by a circumscribed
      polygon (edges tangent to the circle), so the distance to the obstacle is
      never below `margin` and the overshoot is at most margin/cos(step/2).
    - Concave corners use the intersection of the two offset edges (mitre).
    """
    if _signed_area(poly) < 0:      # force CCW
        poly = poly[::-1]
    n = len(poly)
    step = math.radians(max(arc_step_deg, 1.0))

    def normal(a: Point, b: Point) -> Point:       # outward normal of CCW edge
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy) or 1.0
        return (dy / L, -dx / L)

    out: Polygon = []
    for i in range(n):
        p_prev, p, p_next = poly[i - 1], poly[i], poly[(i + 1) % n]
        n1, n2 = normal(p_prev, p), normal(p, p_next)
        turn = _cross(p_prev, p, p_next)           # >0: convex corner (CCW polygon)
        a1, a2 = math.atan2(n1[1], n1[0]), math.atan2(n2[1], n2[0])
        delta = (a2 - a1) % (2 * math.pi)
        if delta > math.pi:
            delta -= 2 * math.pi

        if turn > EPS and delta > 1e-6:            # convex: arc
            k = max(1, math.ceil(delta / step))
            R = margin / math.cos(delta / (2 * k))
            for j in range(k):
                ang = a1 + (j + 0.5) * delta / k
                out.append((p[0] + R * math.cos(ang), p[1] + R * math.sin(ang)))
        else:                                       # concave / flat: mitre
            bx, by = n1[0] + n2[0], n1[1] + n2[1]
            bl = math.hypot(bx, by)
            if bl < EPS:
                out.append((p[0] + n1[0] * margin, p[1] + n1[1] * margin))
                continue
            bx, by = bx / bl, by / bl
            cos_half = max(bx * n1[0] + by * n1[1], EPS)
            out.append((p[0] + bx * margin / cos_half, p[1] + by * margin / cos_half))
    return out


def _seg_intersection_params(a: Point, b: Point, c: Point, d: Point) -> List[float]:
    """Parameters t in [0,1] along a->b where it touches segment c->d."""
    r = (b[0] - a[0], b[1] - a[1])
    s = (d[0] - c[0], d[1] - c[1])
    denom = r[0] * s[1] - r[1] * s[0]
    qp = (c[0] - a[0], c[1] - a[1])
    if abs(denom) < EPS:             # parallel / collinear: use overlap endpoints
        ts = []
        L2 = r[0] ** 2 + r[1] ** 2
        if L2 < EPS or abs(qp[0] * r[1] - qp[1] * r[0]) > 1e-6 * math.sqrt(L2):
            return ts
        for q in (c, d):
            t = ((q[0] - a[0]) * r[0] + (q[1] - a[1]) * r[1]) / L2
            if 0.0 <= t <= 1.0:
                ts.append(t)
        return ts
    t = (qp[0] * s[1] - qp[1] * s[0]) / denom
    u = (qp[0] * r[1] - qp[1] * r[0]) / denom
    if -1e-9 <= t <= 1 + 1e-9 and -1e-9 <= u <= 1 + 1e-9:
        return [min(1.0, max(0.0, t))]
    return []


def segment_is_free(a: Point, b: Point, polys: Sequence[Polygon],
                    ignore: Sequence[int] = ()) -> bool:
    """True if segment a-b never enters the interior of any polygon.
    Touching / sliding along an edge is allowed (nodes lie on inflated borders)."""
    for k, poly in enumerate(polys):
        if k in ignore:
            continue
        ts = [0.0, 1.0]
        n = len(poly)
        for i in range(n):
            ts += _seg_intersection_params(a, b, poly[i], poly[(i + 1) % n])
        ts = sorted(ts)
        for t0, t1 in zip(ts, ts[1:]):
            if t1 - t0 < 1e-9:
                continue
            tm = 0.5 * (t0 + t1)
            mid = (a[0] + tm * (b[0] - a[0]), a[1] + tm * (b[1] - a[1]))
            if point_strictly_inside(mid, poly):
                return False
    return True


def simplify_collinear(pts: List[Point], tol_mm: float = 1.0, free=None) -> List[Point]:
    """Remove waypoints aligned with their neighbours (distance to line < tol).
    If `free(a, b)` is given, a point is removed only if the shortcut a->b is
    also collision-free (so simplification never shaves into an obstacle margin)."""
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        a, b = out[-1], pts[i + 1]
        aligned = _dist_point_segment(pts[i], a, b) <= tol_mm
        if aligned and (free is None or free(a, b)):
            continue
        out.append(pts[i])
    out.append(pts[-1])
    return out


# --------------------------------------------------------------------------
# Planner
# --------------------------------------------------------------------------
class GlobalPlanner:
    """Visibility graph + Dijkstra.

    margin_mm : robot radius + safety.
    The static graph (corner <-> corner) is rebuilt only when set_environment()
    is called, so replanning after a kidnapping only costs start/goal edges.
    """

    def __init__(self, margin_mm: float = 80.0, arc_step_deg: float = 45.0):
        self.margin = margin_mm
        self.arc_step_deg = arc_step_deg   # smaller = rounder corners, more nodes
        self.polys: List[Polygon] = []        # inflated
        self.raw_polys: List[Polygon] = []    # as given by Vision
        self.bounds: Optional[Tuple[float, float, float, float]] = None
        self.goal: Optional[Point] = None
        self.nodes: List[Point] = []
        self.adj: Dict[int, List[Tuple[int, float]]] = {}

    # ---- environment ----------------------------------------------------
    def set_environment(self, env: Environment) -> None:
        m = self.margin
        self.goal = tuple(env.goal) if env.goal is not None else None
        self.raw_polys = _obstacles_of(env)
        self.polys = [inflate_polygon(p, m, self.arc_step_deg) for p in self.raw_polys]
        xmin, ymin, xmax, ymax = _bounds_of(env)
        # black limits = obstacles: free space is the arena shrunk by the margin
        self.bounds = (xmin + m, ymin + m, xmax - m, ymax - m)
        # candidate nodes = inflated corners that are inside the free arena
        self.nodes = [c for poly in self.polys for c in poly if self._in_bounds(c)]
        self._build_static_graph()

    def _in_bounds(self, p: Point) -> bool:
        xmin, ymin, xmax, ymax = self.bounds  # type: ignore[misc]
        return xmin - 1e-6 <= p[0] <= xmax + 1e-6 and ymin - 1e-6 <= p[1] <= ymax + 1e-6

    def _build_static_graph(self) -> None:
        n = len(self.nodes)
        self.adj = {i: [] for i in range(n)}
        for i in range(n):
            for j in range(i + 1, n):
                if segment_is_free(self.nodes[i], self.nodes[j], self.polys):
                    d = math.dist(self.nodes[i], self.nodes[j])
                    self.adj[i].append((j, d))
                    self.adj[j].append((i, d))

    # ---- planning -------------------------------------------------------
    def plan(self, pose: PoseEstimate, goal: Optional[Point] = None) -> Path:
        """Returns Path(valid=False) if no path exists (P4 then does not move).
        `goal` defaults to the one given by Vision in Environment.goal."""
        t = pose.t
        invalid = Path(t=t, waypoints=[], valid=False)
        goal = goal if goal is not None else self.goal
        if self.bounds is None or goal is None:
            return invalid
        start: Point = (pose.x, pose.y)

        if not self._in_bounds(goal) or any(point_strictly_inside(goal, p) for p in self.polys):
            return invalid                                   # goal unreachable

        # If the robot is already inside an inflated zone (close to an obstacle),
        # relax those zones for the start edges only so it can get out.
        start_ignore = [k for k, p in enumerate(self.polys) if point_strictly_inside(start, p)]

        n = len(self.nodes)
        S, G = n, n + 1
        extra: Dict[int, List[Tuple[int, float]]] = {S: [], G: []}

        def start_edge_free(q: Point) -> bool:
            # Inflated zones containing the start are relaxed (so the robot can leave
            # them) but the real obstacle inside them is still forbidden.
            return (segment_is_free(start, q, self.polys, ignore=start_ignore)
                    and segment_is_free(start, q, [self.raw_polys[k] for k in start_ignore]))

        if start_edge_free(goal) and segment_is_free(start, goal, self.polys, ignore=start_ignore):
            return Path(t=t, waypoints=[start, goal], valid=True)

        for i, c in enumerate(self.nodes):
            if start_edge_free(c):
                extra[S].append((i, math.dist(start, c)))
            if segment_is_free(goal, c, self.polys):
                extra[G].append((i, math.dist(goal, c)))

        pts = self.nodes + [start, goal]
        path_idx = self._dijkstra(S, G, extra)
        if path_idx is None:
            return invalid

        def shortcut_free(a: Point, b: Point) -> bool:
            if a == start:
                return start_edge_free(b)
            return segment_is_free(a, b, self.polys)

        waypoints = simplify_collinear([pts[i] for i in path_idx], free=shortcut_free)
        return Path(t=t, waypoints=waypoints, valid=True)

    def _dijkstra(self, S: int, G: int,
                  extra: Dict[int, List[Tuple[int, float]]]) -> Optional[List[int]]:
        def neighbours(u: int):
            if u in extra:
                yield from extra[u]
            else:
                yield from self.adj[u]
                if any(v == u for v, _ in extra[G]):          # edge u -> goal
                    yield (G, dict(extra[G])[u])

        dist = {S: 0.0}
        prev: Dict[int, int] = {}
        heap = [(0.0, S)]
        done = set()
        while heap:
            d, u = heapq.heappop(heap)
            if u in done:
                continue
            done.add(u)
            if u == G:
                path = [G]
                while path[-1] != S:
                    path.append(prev[path[-1]])
                return path[::-1]
            for v, w in neighbours(u):
                nd = d + w
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))
        return None