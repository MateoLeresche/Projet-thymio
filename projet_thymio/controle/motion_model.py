"""Modèle cinématique du Thymio (robot à deux roues différentielles).

Ce fichier est partagé avec le filtre de Kalman : la prédiction du filtre
utilise `odometry_step` et `odometry_jacobians` d'ici, pour que tout le
groupe utilise le même modèle et les mêmes valeurs mesurées.

Conventions du groupe :
    - unités : mm, rad, s ; vitesses des roues en mm/s ;
    - repère de la carte : origine en bas à gauche, x vers la droite, y vers le haut ;
    - theta = 0 quand le robot regarde vers +x, positif dans le sens trigonométrique ;
    - la position (x, y) est celle du milieu de l'axe des roues.

Les valeurs marquées MESURER sont des valeurs de départ : on les remplace par
celles obtenues avec `calibration.py`.
"""

import math

# --- Valeurs du robot (MESURER) -------------------------------------------------

WHEEL_BASE_MM = 96.5        # mesuré : rotation rapide sur place (±227 unités), 715° calculés avec 97,2 mm pour 720° réels
SPEED_FACTOR = 0.337        # mesuré à la vitesse de croisière (80 mm/s) ; 0,353 à 35 mm/s : le facteur dépend de la vitesse, voir mesures_reelles.md
LEFT_CORRECTION = 1.0       # mesuré : aucune déviation sur 35 cm en ligne droite
WHEEL_SPEED_STD_MMS = 2.6   # mesuré : écart-type de la vitesse mesurée d'une roue, 7,5 unités à la consigne 100 (pour le filtre)

MAX_THYMIO_UNITS = 500      # vitesse maximale acceptée par les moteurs du Thymio
MAX_WHEEL_SPEED_MMS = 150.0 # limite qu'on s'impose (en dessous du max du robot)


def wrap_angle(angle):
    """Ramène un angle entre -pi et pi."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# --- Conversion unités Thymio <-> mm/s ------------------------------------------

def thymio_to_mms(units):
    """Vitesse lue sur le Thymio (motor.x.speed) -> mm/s."""
    return units * SPEED_FACTOR


def mms_to_thymio(speed_mms, is_left=False):
    """Vitesse voulue en mm/s -> consigne pour motor.x.target (entier)."""
    speed_mms = max(-MAX_WHEEL_SPEED_MMS, min(MAX_WHEEL_SPEED_MMS, speed_mms))
    units = speed_mms / SPEED_FACTOR
    if is_left:
        units *= LEFT_CORRECTION
    return int(round(max(-MAX_THYMIO_UNITS, min(MAX_THYMIO_UNITS, units))))


# --- Cinématique ----------------------------------------------------------------

def wheels_to_v_omega(v_left, v_right):
    """Vitesses des roues (mm/s) -> vitesse d'avance v (mm/s) et de rotation omega (rad/s)."""
    v = 0.5 * (v_right + v_left)
    omega = (v_right - v_left) / WHEEL_BASE_MM
    return v, omega


def v_omega_to_wheels(v, omega):
    """Vitesse d'avance v (mm/s) et de rotation omega (rad/s) -> vitesses des roues (mm/s)."""
    v_left = v - omega * WHEEL_BASE_MM / 2.0
    v_right = v + omega * WHEEL_BASE_MM / 2.0
    return v_left, v_right


def limit_wheels(v_left, v_right, max_speed=MAX_WHEEL_SPEED_MMS):
    """Si une roue dépasse max_speed, réduit les DEUX dans la même proportion.

    On garde ainsi le même rapport entre les roues, donc la même courbe :
    le robot va moins vite mais dans la même direction.
    """
    biggest = max(abs(v_left), abs(v_right))
    if biggest > max_speed:
        scale = max_speed / biggest
        v_left, v_right = v_left * scale, v_right * scale
    return v_left, v_right


# --- Odométrie (= prédiction du filtre) ------------------------------------------

def odometry_step(x, y, theta, v_left, v_right, dt):
    """Nouvelle pose après dt secondes avec les vitesses de roues mesurées.

    x += v*dt*cos(theta) ; y += v*dt*sin(theta) ; theta += omega*dt
    """
    v, omega = wheels_to_v_omega(v_left, v_right)
    x_new = x + v * dt * math.cos(theta)
    y_new = y + v * dt * math.sin(theta)
    theta_new = wrap_angle(theta + omega * dt)
    return x_new, y_new, theta_new


def odometry_jacobians(theta, v_left, v_right, dt):
    """Dérivées de odometry_step, pour le filtre de Kalman étendu.

    F (3x3) : par rapport à l'état (x, y, theta).
    G (3x2) : par rapport aux vitesses des roues (v_left, v_right).
    Bruit de prédiction : Q = G @ diag(WHEEL_SPEED_STD_MMS**2, WHEEL_SPEED_STD_MMS**2) @ G.T
    Renvoie des listes de listes (à passer dans np.array).
    """
    v, _ = wheels_to_v_omega(v_left, v_right)
    c, s = math.cos(theta), math.sin(theta)
    F = [
        [1.0, 0.0, -v * dt * s],
        [0.0, 1.0, v * dt * c],
        [0.0, 0.0, 1.0],
    ]
    G = [
        [0.5 * dt * c, 0.5 * dt * c],
        [0.5 * dt * s, 0.5 * dt * s],
        [-dt / WHEEL_BASE_MM, dt / WHEEL_BASE_MM],
    ]
    return F, G
