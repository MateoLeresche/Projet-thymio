"""Parametres du filtre = journal de calibration cote P2.

IMPORTANT : les valeurs ci-dessous sont des PLACEHOLDERS pour demarrer.
Les remplacer par des valeurs MESUREES et noter la methode + la date.
"""
import numpy as np

TS = 0.1                                   # s [Mesuré à 101 à 105 ms par pas sur le thymio, le 19/11/2026]

# Bruit camera (a fournir par P1 : ecart-type robot immobile, ~200 images)
SIGMA_CAM = np.array([2.0, 2.0, 0.02])     # mm, mm, rad   [PLACEHOLDER]
R = np.diag(SIGMA_CAM ** 2)

# Bruit sur la commande (a mesurer : allers de longueur connue, rotations de 90 deg)
SIGMA_V = 3.6                              # mm/s          [vitesse mesurée à 227 unités, environ 5 mm/s par roue en ligne droite et en rotation, le 19/11/2026]
SIGMA_W = 0.075                            # rad/s         [SIGMA_V = SIGMA_ROUE/sqrt(2), SIGMA_W = SIGMA_ROUE * sqrt(2)/L, avec L = 96.5 mm, le 19/11/2026]

# Bruit de modele non explique (glissement, jeu mecanique)
Q_FLOOR = np.diag([0.1, 0.1, 1e-4])        # [PLACEHOLDER]

# Seuil NIS (chi2, 3 ddl, 99 %) pour rejeter une mesure aberrante (etape 4b)
NIS_GATE = 11.34
N_KIDNAP = 3                               # rejets consecutifs pour declencher un "kidnap" (0,3 s à 10 Hz)
