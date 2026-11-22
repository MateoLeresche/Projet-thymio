
import cv2
import numpy as np
import time

from .detect_arena import detect_arena
from .perspective import calculate_perspective, correct_perspective

from interfaces import Environment
from .calibration import ARENA_WIDTH_MM, ARENA_HEIGHT_MM, pixel_to_mm


MIN_OBSTACLE_AREA = 500
OBSTACLE_GRAY_THRESHOLD = 80


def create_aruco_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_5X5_50
    )
    parameters = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(aruco_dict, parameters)


def contours_to_polygons(contours):
    polygons = []

    # Parcourir chaque contour détecté
    for contour in contours:

        # Simplifier le contour pour obtenir un polygone
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        polygon = []

        # Convertir chaque sommet en coordonnées millimétriques
        for point in approx:
            px, py = point[0]

            # Conversion pixels -> millimètres
            x, y = pixel_to_mm(px, py)

            polygon.append((round(float(x), 1), round(float(y), 1)))

        polygons.append(polygon)

    return polygons


def detect_obstacles(img, detector):

    # 1. Convertir l'image en niveaux de gris
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 2. Détecter le marqueur du Thymio pour pouvoir le masquer
    corners, ids, _ = detector.detectMarkers(gray)

    # 3. Seuiller les zones sombres correspondant aux obstacles
    _, binary = cv2.threshold(
        gray, OBSTACLE_GRAY_THRESHOLD, 255, cv2.THRESH_BINARY_INV   #pour findContours, on veut les zones sombres en blanc
    )

    # 4. Masquer le marqueur ID 0 afin de ne pas le compter comme obstacle
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):

            if marker_id == 0:
                points = corners[i][0].astype(np.int32)

                # Créer un masque légèrement plus grand
                center = np.mean(points, axis=0)

                expanded_points = (
                    center + (points - center) * 1.5
                ).astype(np.int32)

                cv2.fillConvexPoly(
                    binary,
                    expanded_points,
                    0
                )

    # 5. Détecter les contours noirs restants
    contours, _ = cv2.findContours(
        binary,
        cv2.RETR_LIST,
        cv2.CHAIN_APPROX_SIMPLE
    )

    # 6. Éliminer les petits contours parasites
    # et les contours correspondant aux bordures du terrain
    height, width = binary.shape

    filtered_contours = []

    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)

        # Ignorer les contours qui couvrent presque toute l'image
        is_border = (
            w > 0.8 * width and
            h > 0.8 * height
        )

        if area > MIN_OBSTACLE_AREA and not is_border:
            filtered_contours.append(contour)

    contours = filtered_contours

    # 7. Dessiner les obstacles détectés
    result = img.copy()

    cv2.drawContours(
        result, contours, -1, (0, 0, 255), 2
    )

    return result, contours


if __name__ == "__main__":

    cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)

    if not cap.isOpened():
        print("Impossible d'ouvrir la webcam")
        exit()

    # Détecter le terrain une seule fois
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

    # Créer la vue de dessus
    detector = create_aruco_detector()

    obstacles_detected = False

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        corrected = correct_perspective(frame, M)

        if not obstacles_detected:

            # Détecter les obstacles
            result, contours = detect_obstacles(corrected, detector)

            # Convertir les contours en polygones (mm)
            polygons = contours_to_polygons(contours)

            print("DETECTION EFFECTUEE")
            print("Obstacles :", polygons)

            # Créer l'environnement
            environment = Environment(
                t=time.monotonic(),
                width_mm=ARENA_WIDTH_MM,
                height_mm=ARENA_HEIGHT_MM,
                obstacles=polygons,
                goal=None,
            )

            print("ENVIRONNEMENT CREE")
            print(environment)

            obstacles_detected = True

        cv2.imshow("Vue de dessus", corrected)
        cv2.imshow("Contours detectes", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
