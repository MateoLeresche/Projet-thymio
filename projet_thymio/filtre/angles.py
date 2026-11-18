import numpy as np


def wrap(a):
    """Ramene un angle (rad) dans [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi
