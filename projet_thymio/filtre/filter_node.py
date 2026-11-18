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
    def __init__(self):
        self.kf = None                                   # cree a la 1re mesure visible
        self.last_u = ControlInput(t=0.0, v=0.0, omega=0.0)
        self.last_t = None
        self.pending_z = None

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
            self.kf = KalmanFilter([m.x, m.y, m.theta], params.R,
                                   params.SIGMA_V, params.SIGMA_W, params.Q_FLOOR)
            self.last_t, self.pending_z = t_now, None
            return self._publish(t_now, used=True)

        # 2. prediction avec la derniere commande recue
        dt = t_now - self.last_t
        self.kf.predict(self.last_u.v, self.last_u.omega, dt)
        self.last_t = t_now

        # 3. correction si une mesure visible est arrivee (sinon : prediction seule)
        used = False
        m, self.pending_z = self.pending_z, None
        if m is not None and m.visible:
            self.kf.update(np.array([m.x, m.y, m.theta]))
            used = True

        return self._publish(t_now, used)

    def _publish(self, t, used):
        x, y, th = self.kf.x
        return PoseEstimate(t=t, x=float(x), y=float(y), theta=float(th),
                            P=self.kf.P.copy(), measurement_used=used)
