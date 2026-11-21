"""Boucle temporelle du filtre : seul fichier qui parle le langage de interfaces.py.

Les entrees arrivent de facon asynchrone (on_control, on_measurement) et ne
font que memoriser. Le calcul se fait dans step(), appele toutes les Ts.
Ordre a chaque pas : prediction d'abord, correction ensuite.
"""
import numpy as np
from interfaces import PoseMeasurement, ControlInput, PoseEstimate
from .kalman import KalmanFilter
from . import params


class FilterNode:
    def __init__(self, R=None, sigma_v=None, sigma_w=None, q_floor=None, use_jacobian=True, compensate=True):
        """Les parametres par defaut viennent de params.py ; on peut les
        surcharger (utile pour tester un filtre mal regle en simulation)."""
        self.R = params.R if R is None else R
        self.sigma_v = params.SIGMA_V if sigma_v is None else sigma_v
        self.sigma_w = params.SIGMA_W if sigma_w is None else sigma_w
        self.q_floor = params.Q_FLOOR if q_floor is None else q_floor
        self.kf = None                                   # cree a la 1re mesure visible
        self.last_nis = None                             # NIS du dernier pas corrige
        self.last_u = ControlInput(t=0.0, v=0.0, omega=0.0)
        self.last_t = None
        self.pending_z = None
        self.nis_gate = params.NIS_GATE
        self.n_kidnap = params.N_KIDNAP
        self.n_reject = 0
        self.use_jacobian = use_jacobian
        self.compensate = compensate
        self.hist = []

    # --- entrees (appelees par les autres modules) ---
    def on_control(self, u: ControlInput):
        self.last_u = u

    def on_measurement(self, m: PoseMeasurement):
        self.pending_z = m

    # --- un pas du filtre ---
    def step(self, t_now: float):
        # 1. initialisation sur la premiere mesure visible
        if self.kf is None:
            m = self.pending_z
            if m is None or not m.visible:
                return None
            self.kf = KalmanFilter([m.x, m.y, m.theta], self.R,
                                   self.sigma_v, self.sigma_w, self.q_floor, use_jacobian=self.use_jacobian)
            self.last_t, self.pending_z = t_now, None
            return self._publish(t_now, used=True)

        # 2. prediction avec la derniere commande recue
        dt = t_now - self.last_t
        self.kf.predict(self.last_u.v, self.last_u.omega, dt)
        self.last_t = t_now

        self.hist.append([t_now, self.kf.x.copy(), self.kf.P.copy(),
                        self.last_u.v, self.last_u.omega])
        while len(self.hist) > 2 and self.hist[0][0] < t_now - 2.0:
            self.hist.pop(0)                             # on garde 2 s d'historique

        # 3. correction si une mesure visible est arrivee (sinon : prediction seule)
        used, kidnapped = False, False
        self.last_nis = None
        m, self.pending_z = self.pending_z, None
        if m is not None and m.visible:
            z = np.array([m.x, m.y, m.theta])
            nis, ok = self._correct(z, m.t)
            self.last_nis = nis
            if ok:
                used, self.n_reject = True, 0
            else:
                self.n_reject += 1
                if self.n_reject >= self.n_kidnap:
                    self.kf = KalmanFilter(z, self.R, self.sigma_v, self.sigma_w,
                                   self.q_floor, use_jacobian=self.use_jacobian)
                    self.hist = []
                    self.n_reject = 0
                    used, kidnapped = True, True
        if self.hist:
            self.hist[-1][1], self.hist[-1][2] = self.kf.x.copy(), self.kf.P.copy()
        return self._publish(t_now, used, kidnapped)
               
    def _index_at(self, t):
        """Entrée d'historique la plus proche de l'instant t, ou None si trop ancienne."""
        if not self.hist:
            return None
        i = min(range(len(self.hist)), key=lambda j: abs(self.hist[j][0] - t))
        return i if abs(self.hist[i][0] - t) <= params.TS else None

    def _correct(self, z, t_meas):
        """Applique la mesure (avec rejeu si besoin). Retourne (nis, acceptée)."""
        i = self._index_at(t_meas) if self.compensate else None
        if i is None or i == len(self.hist) - 1:
            _, _, nis = self.kf.innovation(z)
            if nis <= self.nis_gate:
                self.kf.update(z)
                return nis, True
            return nis, False
        x_now, P_now = self.kf.x.copy(), self.kf.P.copy()
        self.kf.x, self.kf.P = self.hist[i][1].copy(), self.hist[i][2].copy()
        _, _, nis = self.kf.innovation(z)           # NIS à l'instant de capture
        if nis > self.nis_gate:
            self.kf.x, self.kf.P = x_now, P_now     # rejet : on ne touche à rien
            return nis, False
        self.kf.update(z)
        self.hist[i][1], self.hist[i][2] = self.kf.x.copy(), self.kf.P.copy()
        for j in range(i + 1, len(self.hist)):      # rejeu jusqu'à maintenant
            t, _, _, v, w = self.hist[j]
            self.kf.predict(v, w, t - self.hist[j - 1][0])
            self.hist[j][1], self.hist[j][2] = self.kf.x.copy(), self.kf.P.copy()
        return nis, True

    def _publish(self, t, used, kidnapped=False):
        x, y, th = self.kf.x
        return PoseEstimate(t=t, x=float(x), y=float(y), theta=float(th),
                            P=self.kf.P.copy(), measurement_used=used, kidnapped=kidnapped)
