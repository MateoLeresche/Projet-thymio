"""Filtre de Kalman pour la pose (x, y, theta) du Thymio.

Module de maths pures : il ne connait ni la camera, ni tdmclient, ni les
dataclasses de interfaces.py. Il ne manipule que des tableaux numpy.

Modele (lineaire par morceaux : le robot fait soit une translation pure,
soit une rotation sur place) :
    x_{k+1} = x_k + B(theta_k) u_k + w_k      u = [v, omega], w ~ N(0, Q)
    z_k     = x_k + v_k                       v ~ N(0, R)
"""
import numpy as np
from .angles import wrap


class KalmanFilter:
    def __init__(self, z0, R, sigma_v, sigma_w, q_floor):
        """
        z0      : premiere mesure [x, y, theta]
        R       : covariance de mesure camera (3x3)
        sigma_v : ecart-type sur la vitesse lineaire commandee (mm/s)
        sigma_w : ecart-type sur la vitesse angulaire commandee (rad/s)
        q_floor : bruit de modele non explique (3x3, diagonale)
        """
        self.x = np.array(z0, dtype=float)
        self.x[2] = wrap(self.x[2])
        self.P = np.array(R, dtype=float).copy()   # P0 = R
        self.R = np.array(R, dtype=float)
        self.Su = np.diag([sigma_v ** 2, sigma_w ** 2])
        self.Qf = np.array(q_floor, dtype=float)
        self.H = np.eye(3)

    def predict(self, v, w, dt):
        """Etape de prediction : P augmente."""
        th = self.x[2]
        B = np.array([[np.cos(th) * dt, 0.0], # v influence x, w n'influence pas x
                      [np.sin(th) * dt, 0.0], # v influence y, w n'influence pas y
                      [0.0,             dt]]) # v n'influence pas theta, w influence theta
        self.x = self.x + B @ np.array([v, w])     # x_nouveau = A x_ancien + B u, avec A = I
        self.x[2] = wrap(self.x[2])
        Q = B @ self.Su @ B.T + self.Qf
        self.P = self.P + Q                        # A P A^T + Q avec A = I

    def update(self, z):
        """Etape de correction : P diminue. Retourne (innovation, S, NIS)."""
        innov = np.asarray(z, dtype=float) - self.H @ self.x
        innov[2] = wrap(innov[2])                  # angle : innovation normalisee
        S = self.H @ self.P @ self.H.T + self.R
        S_inv = np.linalg.inv(S)
        K = self.P @ self.H.T @ S_inv
        self.x = self.x + K @ innov
        self.x[2] = wrap(self.x[2])
        IKH = np.eye(3) - K @ self.H               # forme de Joseph
        self.P = IKH @ self.P @ IKH.T + K @ self.R @ K.T
        nis = float(innov @ S_inv @ innov)
        return innov, S, nis
