"""Parametres du filtre = journal de calibration cote P2.

IMPORTANT : les valeurs ci-dessous sont des PLACEHOLDERS pour demarrer.
Les remplacer par des valeurs MESUREES et noter la methode + la date.
"""
import numpy as np

TS = 0.1                                   # s [A CONFIRMER avec le groupe]

# Bruit camera (a fournir par P1 : ecart-type robot immobile, ~200 images)
SIGMA_CAM = np.array([2.0, 2.0, 0.02])     # mm, mm, rad   [PLACEHOLDER]
R = np.diag(SIGMA_CAM ** 2)

# Bruit sur la commande (a mesurer : allers de longueur connue, rotations de 90 deg)
SIGMA_V = 5.0                              # mm/s          [PLACEHOLDER]
SIGMA_W = 0.05                             # rad/s         [PLACEHOLDER]

# Bruit de modele non explique (glissement, jeu mecanique)
Q_FLOOR = np.diag([0.1, 0.1, 1e-4])        # [PLACEHOLDER]

# Seuil NIS (chi2, 3 ddl, 99 %) pour rejeter une mesure aberrante (etape 4b)
NIS_GATE = 11.34
