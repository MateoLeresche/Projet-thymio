"""Contrat d'interfaces entre les modules (version 0.1).

Ce fichier est PARTAGE par les 4 modules. Personne ne le modifie sans
prevenir le groupe (voir README.md et docs/CONTRAT_INTERFACES.md).

Conventions :
- Repere monde : origine = coin inferieur gauche du carre noir [A CONFIRMER],
  x vers la droite de l'image, y vers le haut, theta positif anti-horaire.
- Unites : mm, rad (dans [-pi, pi]), mm/s, rad/s, s.
- Temps : time.monotonic(), instant de CAPTURE de la mesure.
"""
from dataclasses import dataclass, field
from typing import Optional
import numpy as np


@dataclass
class PoseMeasurement:
    """Vision (P1) -> Filtre (P2). Une mesure de pose du Thymio par la camera."""
    t: float                       # s, instant de capture
    visible: bool                  # False si le QR code n'est pas detecte
    x: Optional[float] = None      # mm (None si visible == False)
    y: Optional[float] = None      # mm
    theta: Optional[float] = None  # rad, dans [-pi, pi]


@dataclass
class ControlInput:
    """Controle (P4) -> Filtre (P2). Commande reellement appliquee au robot."""
    t: float        # s
    v: float        # mm/s, vitesse lineaire
    omega: float    # rad/s, vitesse angulaire (+ = anti-horaire)


@dataclass
class PoseEstimate:
    """Filtre (P2) -> Planification (P3), Controle (P4), Visualisation."""
    t: float                  # s
    x: float                  # mm
    y: float                  # mm
    theta: float              # rad, [-pi, pi]
    P: np.ndarray             # covariance 3x3 (x, y, theta)
    measurement_used: bool    # True si corrige par la camera a ce pas
    kidnapped: bool = False   # True si un kidnapping a ete detecte


@dataclass
class Environment:
    """Vision (P1) -> Planification (P3)."""
    t: float
    width_mm: float                                # largeur de la zone utile
    height_mm: float                               # hauteur de la zone utile
    obstacles: list = field(default_factory=list)  # polygones [(x, y), ...] en mm
    goal: Optional[tuple] = None                   # (x, y) en mm, si detecte


@dataclass
class Path:
    """Planification (P3) -> Controle (P4)."""
    t: float
    waypoints: list   # [(x, y), ...] en mm, du depart vers le but
    valid: bool       # False si aucun chemin trouve
