"""Visualisation temps reel du filtre (ellipse de covariance, mesure vs estimation, NIS).

Ce fichier est OPTIONNEL : ni kalman.py ni filter_node.py ne l'importent, donc la regle
"le filtre n'importe pas OpenCV" reste vraie pour le coeur du filtre. OpenCV n'est utilise
ici que pour dessiner (comme dans vision/realtime.py).

Utilisation (dans show_interface de vision/realtime.py) :

    viz = FilterViz(world_to_px)                  # une fois, world_to_px(x_mm, y_mm) -> (px, py)
    ...
    viz.update(estimate, measurement, nis, odom=(x, y))   # a chaque pas du filtre
    viz.draw(model_img)                           # dessine sur la vue 2D du terrain
    viz.draw_panel(img, x0, y0, w, h)             # texte + courbe du NIS dans une zone libre

Le texte est en ASCII : cv2.putText ne sait pas afficher les accents.
"""
import math
from collections import deque

import numpy as np
import cv2

from . import params

# Couleurs BGR
BLUE = (200, 90, 0)
ORANGE = (0, 140, 255)
GREEN = (0, 170, 0)
RED = (0, 0, 220)
GREY = (150, 150, 150)
PURPLE = (180, 0, 180)
BLACK = (30, 30, 30)


class FilterViz:
    def __init__(self, world_to_px, sigma_k=3.0, ellipse_mag=1.0, trail_len=600, nis_len=120):
        """
        world_to_px : fonction (x_mm, y_mm) -> (px, py) de la vue 2D
        sigma_k     : l'ellipse represente k sigma (3 = 99 % des cas)
        ellipse_mag : agrandissement DESSINE de l'ellipse. Avec la camera, sigma ~ 2 mm :
                      a 1 l'ellipse est invisible tant que le QR n'est pas cache. Si != 1,
                      c'est ecrit dans le panneau pour ne pas tromper le prof.
        """
        self.w2p = world_to_px
        p0, p1 = world_to_px(0.0, 0.0), world_to_px(1.0, 0.0)
        self.px_per_mm = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) or 1.0
        self.k = sigma_k
        self.mag = ellipse_mag
        self.trail = deque(maxlen=trail_len)       # trajectoire estimee
        self.raw = deque(maxlen=trail_len)         # mesures camera brutes
        self.odom = deque(maxlen=trail_len)        # odometrie seule (optionnel)
        self.nis_hist = deque(maxlen=nis_len)
        self.est = None
        self.meas = None
        self.nis = None

    # ---------------------------------------------------------------- donnees
    def update(self, estimate, measurement=None, nis=None, odom=None):
        """estimate : PoseEstimate ; measurement : PoseMeasurement ou None ;
        nis : node.last_nis ou None ; odom : (x, y) de l'odometrie seule ou None."""
        self.est, self.meas, self.nis = estimate, measurement, nis
        self.trail.append((estimate.x, estimate.y))
        if measurement is not None and measurement.visible:
            self.raw.append((measurement.x, measurement.y))
        if odom is not None:
            self.odom.append(tuple(odom))
        self.nis_hist.append(float("nan") if nis is None else nis)

    # ---------------------------------------------------------------- vue 2D
    def _polyline(self, img, pts, color, thick=2):
        if len(pts) >= 2:
            arr = np.array([self.w2p(x, y) for x, y in pts], dtype=np.int32)
            cv2.polylines(img, [arr], False, color, thick)

    def _covariance_ellipse(self, img, est):
        vals, vecs = np.linalg.eigh(np.asarray(est.P)[:2, :2])
        vals = np.clip(vals, 0.0, None)
        major = vecs[:, 1]                                   # eigh : valeurs croissantes
        angle = math.degrees(math.atan2(major[1], major[0]))
        scale = self.k * self.px_per_mm * self.mag
        axes = (max(int(math.sqrt(vals[1]) * scale), 1), max(int(math.sqrt(vals[0]) * scale), 1))
        # l'axe y de l'image est inverse par rapport au monde -> angle oppose
        cv2.ellipse(img, self.w2p(est.x, est.y), axes, -angle, 0, 360, PURPLE, 2)

    def draw(self, img):
        """Dessine sur la vue 2D du terrain (pas sur l'image camera brute)."""
        if self.est is None:
            return
        est = self.est
        self._polyline(img, self.odom, GREY, 1)
        for x, y in self.raw:
            cv2.circle(img, self.w2p(x, y), 2, ORANGE, -1)
        self._polyline(img, self.trail, BLUE, 2)
        self._covariance_ellipse(img, est)

        centre = self.w2p(est.x, est.y)
        cv2.circle(img, centre, 8, PURPLE, -1)
        tip = (int(centre[0] + 30 * math.cos(est.theta)), int(centre[1] - 30 * math.sin(est.theta)))
        cv2.arrowedLine(img, centre, tip, PURPLE, 3)

        m = self.meas
        if m is not None and m.visible:
            mp = self.w2p(m.x, m.y)
            cv2.drawMarker(img, mp, ORANGE, cv2.MARKER_CROSS, 16, 2)
            cv2.line(img, mp, centre, ORANGE, 1)             # ecart mesure -> estimation

    # ---------------------------------------------------------------- panneau
    def state_text(self):
        est, m = self.est, self.meas
        if est is None:
            return "ATTENTE DE MESURE", ORANGE
        if est.kidnapped:
            return "KIDNAPPING DETECTE", RED
        if est.measurement_used:
            return "MESURE UTILISEE", GREEN
        if m is not None and m.visible:
            return "MESURE REJETEE (NIS > seuil)", RED
        return "PREDICTION SEULE (QR cache)", ORANGE

    def draw_panel(self, img, x0, y0, w, h):
        """Texte (etat, sigma, NIS) + courbe du NIS dans le rectangle (x0, y0, w, h)."""
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (250, 250, 250), -1)
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), BLACK, 1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        text, color = self.state_text()
        cv2.putText(img, text, (x0 + 10, y0 + 25), font, 0.6, color, 2)
        if self.est is not None:
            P = np.asarray(self.est.P)
            sx, sy = math.sqrt(P[0, 0]), math.sqrt(P[1, 1])
            sth = math.degrees(math.sqrt(P[2, 2]))
            lines = [f"sigma x = {sx:6.1f} mm", f"sigma y = {sy:6.1f} mm", f"sigma theta = {sth:5.2f} deg"]
            if self.meas is not None and self.meas.visible:
                d = math.hypot(self.meas.x - self.est.x, self.meas.y - self.est.y)
                lines.append(f"ecart camera-estimation = {d:5.1f} mm")
            else:
                lines.append("ecart camera-estimation = -")
            lines.append("NIS = " + ("-" if self.nis is None else f"{self.nis:5.2f}") + " (attendu ~3)")
            for i, s in enumerate(lines):
                cv2.putText(img, s, (x0 + 10, y0 + 55 + 22 * i), font, 0.5, BLACK, 1)
            if self.mag != 1.0:
                cv2.putText(img, f"ellipse {self.k:.0f} sigma, dessinee x{self.mag:g}",
                            (x0 + 10, y0 + 55 + 22 * len(lines)), font, 0.45, PURPLE, 1)

        # courbe du NIS (0..20) avec seuil de rejet et valeur attendue
        gx, gy, gw, gh = x0 + 10, y0 + h - 130, w - 20, 110
        cv2.rectangle(img, (gx, gy), (gx + gw, gy + gh), GREY, 1)
        ymax = 20.0

        def ypx(v):
            return int(gy + gh - min(max(v, 0.0), ymax) / ymax * gh)

        cv2.line(img, (gx, ypx(params.NIS_GATE)), (gx + gw, ypx(params.NIS_GATE)), RED, 1)
        cv2.line(img, (gx, ypx(3.0)), (gx + gw, ypx(3.0)), GREEN, 1)
        cv2.putText(img, f"seuil {params.NIS_GATE}", (gx + 4, ypx(params.NIS_GATE) - 3), font, 0.4, RED, 1)
        cv2.putText(img, "NIS", (gx + 4, gy + gh - 4), font, 0.4, BLACK, 1)
        n = len(self.nis_hist)
        step = gw / max(self.nis_hist.maxlen - 1, 1)
        prev = None
        for i, v in enumerate(self.nis_hist):
            if math.isnan(v):
                prev = None                                  # pas de mesure : trou dans la courbe
                continue
            p = (int(gx + i * step), ypx(v))
            if prev is not None:
                cv2.line(img, prev, p, BLUE, 1)
            prev = p