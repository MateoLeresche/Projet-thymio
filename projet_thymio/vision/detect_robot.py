
import cv2
import numpy as np
import math
import time

from interfaces import PoseMeasurement
from .calibration import pixel_to_mm

from .detect_arena import detect_arena
from .perspective import calculate_perspective, correct_perspective


# --------------------------------------------------
# 1. DETECTEUR ARUCO
# --------------------------------------------------

def create_aruco_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_5X5_50
    )

    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    return cv2.aruco.ArucoDetector(aruco_dict, parameters)


# --------------------------------------------------
# 2. DETECTION DU ROBOT
# --------------------------------------------------

def detect_robot(img, detector, t=None):
    t = time.monotonic() if t is None else t

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    result = img.copy()
    measurement = PoseMeasurement(t=t, visible=False)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(result, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == 0:
                points = corners[i][0]

                # Position du centre du marqueur
                center_x = np.mean(points[:, 0])
                center_y = np.mean(points[:, 1])

                x, y = pixel_to_mm(center_x, center_y)

                # Orientation du robot
                edge_center = np.mean(points[0:2], axis=0)
                dx = edge_center[0] - center_x
                dy = edge_center[1] - center_y

                theta = -math.atan2(dy, dx)
                theta = math.atan2(math.sin(theta), math.cos(theta))

                # Mesure envoyée au filtre
                measurement = PoseMeasurement(
                    t=t,
                    visible=True,
                    x=float(x),
                    y=float(y),
                    theta=theta
                )

                # Centre du robot
                center = (int(center_x), int(center_y))
                cv2.circle(result, center, 6, (0, 0, 255), -1)

                # Flèche bleue indiquant la direction
                length = 50
                end_x = int(center_x + length * math.cos(-theta))
                end_y = int(center_y + length * math.sin(-theta))

                cv2.arrowedLine(
                    result,
                    center,
                    (end_x, end_y),
                    (255, 0, 0),
                    3
                )

                # Affichage des coordonnées
                cv2.putText(
                    result,
                    f"x={x:.1f} mm, y={y:.1f} mm",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    result,
                    f"theta={theta:.2f} rad",
                    (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                break

    return result, measurement


# --------------------------------------------------
# 3. PROGRAMME PRINCIPAL
# --------------------------------------------------

if __name__ == "__main__":

    cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)

    if not cap.isOpened():
        print("Impossible d'ouvrir la webcam")
        exit()

    # Détection du terrain
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

    # Création du détecteur
    detector = create_aruco_detector()

    print("Détection du Thymio...")

    # Boucle principale
    while True:
        ret, frame = cap.read()

        if not ret:
            break

        # Correction de perspective
        corrected = correct_perspective(frame, M)

        # Détection du robot
        result, measurement = detect_robot(corrected, detector)

        # Afficher l'image
        cv2.imshow("Vue de dessus - Thymio", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
