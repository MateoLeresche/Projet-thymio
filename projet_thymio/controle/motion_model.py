"""Modèle du Thymio : un robot avec deux roues (une à gauche, une à droite).

Ici il y a les valeurs mesurées du robot, la conversion entre les unités du
Thymio et les mm/s, et l'odométrie (calculer où est le robot à partir de la
vitesse de ses roues).

Conventions du groupe :
    unités en mm, rad et s, vitesses des roues en mm/s
    repère de la carte : origine en bas à gauche, x vers la droite, y vers le haut
    theta = 0 quand le robot regarde vers +x, et theta augmente quand il tourne à gauche
    la position (x, y) c'est le milieu entre les deux roues
"""

import math

# Valeurs mesurées sur notre Thymio (tout est expliqué dans mesures_reelles.md)
WHEEL_BASE_MM = 96.5        # distance entre les 2 roues, mesurée en faisant tourner le robot sur place (2 tours)
SPEED_FACTOR = 0.337        # 1 unité Thymio = 0,337 mm/s, mesuré à 80 mm/s (à 35 mm/s on trouvait 0,353, ça change avec la vitesse)
LEFT_CORRECTION = 1.0       # pas besoin de corriger la roue gauche, le robot va droit sur 35 cm
WHEEL_SPEED_STD_MMS = 2.6   # bruit sur la vitesse mesurée d'une roue à 35 mm/s (environ 5 mm/s à 80 mm/s), utile pour le filtre

MAX_THYMIO_UNITS = 500      # le Thymio n'accepte pas plus que 500
MAX_WHEEL_SPEED_MMS = 150.0 # on se limite à 150 mm/s pour rester tranquille


def wrap_angle(angle):
    """Remet un angle entre -pi et pi (par exemple 270° devient -90°)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def thymio_to_mms(units):
    """Vitesse lue sur le Thymio (en unités) convertie en mm/s."""
    return units * SPEED_FACTOR


def mms_to_thymio(speed_mms, is_left=False):
    """Vitesse voulue en mm/s convertie en consigne pour le moteur (un entier)."""
    # on bloque la vitesse entre -150 et 150 mm/s
    speed_mms = max(-MAX_WHEEL_SPEED_MMS, min(MAX_WHEEL_SPEED_MMS, speed_mms))
    units = speed_mms / SPEED_FACTOR
    if is_left:
        units *= LEFT_CORRECTION  # correction de la roue gauche si un jour elle va moins vite
    # le moteur veut un entier entre -500 et 500
    return int(round(max(-MAX_THYMIO_UNITS, min(MAX_THYMIO_UNITS, units))))


def wheels_to_v_omega(v_left, v_right):
    """Vitesses des 2 roues en vitesse d'avance v (mm/s) et vitesse de rotation omega (rad/s)."""
    v = 0.5 * (v_right + v_left)                 # le robot avance à la moyenne des 2 roues
    omega = (v_right - v_left) / WHEEL_BASE_MM   # il tourne si une roue va plus vite que l'autre
    return v, omega


def v_omega_to_wheels(v, omega):
    """L'inverse : on veut avancer à v et tourner à omega, quelle vitesse pour chaque roue ?"""
    v_left = v - omega * WHEEL_BASE_MM / 2.0
    v_right = v + omega * WHEEL_BASE_MM / 2.0
    return v_left, v_right


def limit_wheels(v_left, v_right, max_speed=MAX_WHEEL_SPEED_MMS):
    """Si une roue va trop vite, on ralentit les DEUX roues pareil.

    Comme ça le rapport entre les roues reste le même, donc le robot fait la
    même courbe, juste moins vite.
    """
    biggest = max(abs(v_left), abs(v_right))
    if biggest > max_speed:
        scale = max_speed / biggest
        v_left, v_right = v_left * scale, v_right * scale
    return v_left, v_right


def odometry_step(x, y, theta, v_left, v_right, dt):
    """Odométrie : où est le robot après dt secondes, avec la vitesse de ses roues.

    C'est la même formule que la prédiction du filtre de Matéo.
    """
    v, omega = wheels_to_v_omega(v_left, v_right)
    x_new = x + v * dt * math.cos(theta)   # il avance dans la direction où il regarde
    y_new = y + v * dt * math.sin(theta)
    theta_new = wrap_angle(theta + omega * dt)
    return x_new, y_new, theta_new


def odometry_jacobians(theta, v_left, v_right, dt):
    """Dérivées de odometry_step, au cas où le filtre en a besoin (filtre de Kalman étendu).

    F : dérivée par rapport à (x, y, theta)
    G : dérivée par rapport aux vitesses des roues (v_left, v_right)
    Bruit de prédiction : Q = G @ diag(std**2, std**2) @ G.T
    Ça renvoie des listes de listes, il faut les passer dans np.array.
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
