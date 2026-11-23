"""Demo de la visualisation du filtre, sans camera ni robot (simulation avec verite terrain).

    (depuis la racine projet_thymio/)
    python -m evaluation.demo_viz                       # fenetre en direct, QR cache de 10 a 15 s
    python -m evaluation.demo_viz --hidden 8 14         # autre fenetre de QR cache
    python -m evaluation.demo_viz --save data/viz       # sans fenetre : data/viz_cache.png et data/viz_fin.png
    q pour quitter.

On voit : l'estimation (bleu), les mesures camera brutes (orange), l'odometrie seule (gris),
la vraie position de la simulation (noir), et l'ellipse de covariance qui grossit quand le QR
est cache puis se resserre quand la camera le retrouve.
"""
import argparse
import math

import numpy as np
import cv2

from simulation.simulator import simulate_and_filter, square_trajectory
from filtre import params
from filtre.viz import FilterViz
from interfaces import PoseEstimate, PoseMeasurement

ARENA = 900.0          # mm
SCALE = 0.8            # pixels par mm
MARGIN = 30
PANEL_W = 340


def world_to_px(x, y):
    return int(round(MARGIN + x * SCALE)), int(round(MARGIN + (ARENA - y) * SCALE))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hidden", nargs=2, type=float, default=(10.0, 15.0), help="QR cache de t1 a t2 (s)")
    ap.add_argument("--mag", type=float, default=5.0, help="agrandissement dessine de l'ellipse")
    ap.add_argument("--speed", type=float, default=1.0, help="vitesse de lecture (1 = temps reel)")
    ap.add_argument("--save", default=None, help="prefixe de fichier : pas de fenetre, 2 images")
    ap.add_argument("--scale", type=float, default=0.6, help="pixels par mm (0.6 = fenetre plus petite)")
    args = ap.parse_args()
    global SCALE
    SCALE = args.scale

    run = simulate_and_filter(square_trajectory(speed=80.0), seed=1, hidden=[tuple(args.hidden)])
    ts = params.TS

    # odometrie seule : on integre les commandes nominales depuis la premiere estimation
    ox, oy, oth = run["est"][0]
    odom = [(ox, oy)]
    for v, w in run["u"][1:]:
        ox += v * ts * math.cos(oth)
        oy += v * ts * math.sin(oth)
        oth += w * ts
        odom.append((ox, oy))

    h = int(MARGIN * 2 + ARENA * SCALE)
    w = int(MARGIN * 2 + ARENA * SCALE) + PANEL_W
    viz = FilterViz(world_to_px, ellipse_mag=args.mag)
    arena_px = int(ARENA * SCALE)
    t_snap = args.hidden[1] - 0.1
    saved_hidden = False
    if not args.save:
        cv2.namedWindow("Filtre - visualisation", cv2.WINDOW_NORMAL)

    for k in range(len(run["t"])):
        zk = run["z"][k]
        meas = PoseMeasurement(t=run["t"][k], visible=bool(run["visible"][k]),
                               x=None if np.isnan(zk[0]) else float(zk[0]),
                               y=None if np.isnan(zk[1]) else float(zk[1]),
                               theta=None if np.isnan(zk[2]) else float(zk[2]))
        ex, ey, eth = run["est"][k]
        est = PoseEstimate(t=run["t"][k], x=float(ex), y=float(ey), theta=float(eth), P=run["P"][k],
                           measurement_used=bool(run["used"][k]), kidnapped=bool(run["kidnapped"][k]))
        nis = None if np.isnan(run["nis"][k]) else float(run["nis"][k])
        viz.update(est, meas, nis, odom=odom[k])

        img = np.full((h, w, 3), 255, dtype=np.uint8)
        cv2.rectangle(img, (MARGIN, MARGIN), (MARGIN + arena_px, MARGIN + arena_px), (30, 30, 30), 3)
        tx, ty, _ = run["truth"][k]
        cv2.circle(img, world_to_px(tx, ty), 4, (0, 0, 0), -1)          # verite (simulation seulement)
        viz.draw(img)
        viz.draw_panel(img, MARGIN * 2 + arena_px, MARGIN, PANEL_W - 20, h - 2 * MARGIN)
        cv2.putText(img, f"t = {run['t'][k]:5.1f} s", (MARGIN + 10, MARGIN + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 30, 30), 2)
        cv2.putText(img, "bleu: estime | orange: camera | gris: odometrie | noir: verite (sim)",
                    (MARGIN + 10, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1)

        if args.save:
            if not saved_hidden and run["t"][k] >= t_snap:
                cv2.imwrite(args.save + "_cache.png", img)
                saved_hidden = True
            if k == len(run["t"]) - 1:
                cv2.imwrite(args.save + "_fin.png", img)
        else:
            cv2.imshow("Filtre - visualisation", img)
            if cv2.waitKey(max(int(ts * 1000 / args.speed), 1)) & 0xFF == ord("q"):
                break

    if not args.save:
        cv2.destroyAllWindows()
    else:
        print(f"images enregistrees : {args.save}_cache.png, {args.save}_fin.png")


if __name__ == "__main__":
    main()