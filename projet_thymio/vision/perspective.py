
import cv2
import numpy as np

from .detect_arena import detect_arena
from .calibration import ARENA_MARGIN_PX, CORRECTED_SIZE_PX


CORRECTED_SIZE = (CORRECTED_SIZE_PX, CORRECTED_SIZE_PX)


def order_points(points):

    # 1. Convertir les quatre coins en tableau NumPy
    points = np.array(points, dtype=np.float32)

    # 2. Préparer les coordonnées ordonnées
    ordered = np.zeros((4, 2), dtype=np.float32)

    # 3. Trier les coins par somme et différence des coordonnées
    s = points.sum(axis=1)
    diff = np.diff(points, axis=1)

    # 4. Associer chaque coin à un angle du terrain
    ordered[0] = points[np.argmin(s)]
    ordered[1] = points[np.argmin(diff)]
    ordered[2] = points[np.argmax(s)]
    ordered[3] = points[np.argmax(diff)]

    return ordered


def calculate_perspective(corners):
    # 1. Vérifier que les quatre coins sont disponibles
    if corners is None or len(corners) != 4:
        return None

    # 2. Ordonner les coins dans le même sens
    corners = order_points(corners)

    width, height = CORRECTED_SIZE
    margin = ARENA_MARGIN_PX

    # 3. Définir les quatre coins de l'image corrigée
    destination = np.array([
        [margin, margin],
        [width - margin, margin],
        [width - margin, height - margin],
        [margin, height - margin]
    ], dtype=np.float32)

    # 4. Calculer la matrice de transformation perspective
    return cv2.getPerspectiveTransform(corners, destination)


def correct_perspective(img, M):
    # 1. Vérifier que la matrice de perspective existe
    if M is None:
        return None

    # 2. Transformer l'image caméra en vue de dessus
    return cv2.warpPerspective(img, M, CORRECTED_SIZE)


if __name__ == "__main__":

    cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)

    if not cap.isOpened():
        print("Impossible d'ouvrir la webcam")
        exit()

    # 1. Attendre la détection des 4 coins du terrain
    print("Détection du terrain...")

    M = None

    while M is None:
        ret, frame = cap.read()

        if not ret:
            continue

        result, corners = detect_arena(frame)

        if corners is not None:
            M = calculate_perspective(corners)
            print("Terrain détecté !")

        cv2.imshow("Camera originale", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            exit()

    # 2. Réutiliser la même matrice pour chaque image
    while True:
        ret, frame = cap.read()

        if not ret:
            break

        corrected = correct_perspective(frame, M)

        cv2.imshow("Vue de dessus", corrected)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()