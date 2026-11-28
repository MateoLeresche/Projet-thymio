import time
import math

import cv2
import numpy as np

from interfaces import Environment, PoseMeasurement
from .detect_arena import detect_arena
from .detect_obstacles import detect_obstacles, contours_to_polygons
from .detect_robot import create_aruco_detector, detect_robot
from .perspective import calculate_perspective, correct_perspective
from .detect_goal import detect_red_goal
from .calibration import ARENA_WIDTH_MM, ARENA_HEIGHT_MM, mm_to_pixel, pixel_to_mm


class RealCamera:
    """Caméra réelle utilisée par la boucle de contrôle."""

    def __init__(self, camera_index=0):
        # 1. Ouvrir la webcam avec l'API AVFoundation sur macOS
        self.capture = cv2.VideoCapture(camera_index, cv2.CAP_AVFOUNDATION)
        if not self.capture.isOpened():
            raise RuntimeError("Impossible d'ouvrir la webcam")

        # 2. Initialiser les données de vision
        self.perspective = None
        self.detector = create_aruco_detector()
        self.frame = None
        self.obstacle_contours = []
        self.goal = None
        self.goal_pixel = None
        self.environment_obstacles = []

        # 3. Calibrer une seule fois la vue de dessus
        self._calibrate()

    def _calibrate(self):
        # 1. Attendre la détection des quatre coins du terrain
        print("Détection du terrain...")
        while self.perspective is None:
            ret, frame = self.capture.read()
            if not ret:
                time.sleep(0.01)
                continue

            # 2. Tester si les quatre coins sont actuellement détectés
            result, corners = detect_arena(frame)
            if corners is not None:
                self.perspective = calculate_perspective(corners)
                print("Terrain détecté")

            # 3. Afficher l'image originale pendant la calibration
            cv2.imshow("Camera originale", result)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.close()
                raise KeyboardInterrupt

    def __call__(self, t):
        # 1. Lire une nouvelle image
        ret, frame = self.capture.read()
        if not ret:
            return PoseMeasurement(t=t, visible=False)

        # 2. Noter l'instant de capture pour le filtre
        t_capture = time.monotonic()

        # 3. Transformer l'image en vue de dessus
        corrected = correct_perspective(frame, self.perspective)

        # 4. Détecter la position du Thymio
        result, measurement = detect_robot(corrected, self.detector, t=t_capture)

        # 5. Détecter le but une seule fois
        if self.goal is None:
            result, detected_goal, detected_goal_pixel = detect_red_goal(result)
            if detected_goal is not None:
                self.goal = detected_goal
                self.goal_pixel = detected_goal_pixel
        else:
            # 6. Afficher la position mémorisée du but
            detected_goal_pixel = self.goal_pixel
            if detected_goal_pixel is not None:
                px, py, radius = detected_goal_pixel
                cv2.circle(result, (int(px), int(py)), int(radius), (0, 255, 255), 2)
                cv2.drawMarker(result, (int(px), int(py)), (0, 255, 0),
                               cv2.MARKER_CROSS, 24, 3)

        # 7. Conserver l'image annotée pour l'interface
        self.frame = result
        return measurement

    def environment(self, goal=None):
        # 1. Lire une image pour construire l'environnement
        ret, frame = self.capture.read()
        if not ret:
            return Environment(t=time.monotonic(), width_mm=ARENA_WIDTH_MM, height_mm=ARENA_HEIGHT_MM,
                               obstacles=[], goal=tuple(goal) if goal is not None else None)

        # 2. Transformer l'image en vue de dessus
        t_capture = time.monotonic()
        corrected = correct_perspective(frame, self.perspective)

        # 3. Détecter les obstacles
        _, contours = detect_obstacles(corrected, self.detector)

        # 4. Détecter le but une seule fois
        if self.goal is None:
            _, detected_goal, detected_goal_pixel = detect_red_goal(corrected)
            if detected_goal is not None:
                self.goal = detected_goal
                self.goal_pixel = detected_goal_pixel

        # 5. Retirer du contour des obstacles le cercle rouge détecté
        if self.goal_pixel is not None:
            gx, gy, gradius = self.goal_pixel
            filtered_contours = []
            for contour in contours:
                moments = cv2.moments(contour)
                if moments["m00"] > 0:
                    cx = moments["m10"] / moments["m00"]
                    cy = moments["m01"] / moments["m00"]
                    if np.hypot(cx - gx, cy - gy) < gradius * 1.5:
                        continue
                filtered_contours.append(contour)
            contours = filtered_contours

        # 6. Conserver les obstacles pour l'affichage
        self.obstacle_contours = contours

        # 7. Construire l'environnement envoyé au planificateur
        selected_goal = self.goal if self.goal is not None else goal
        polygons = contours_to_polygons(contours)
        self.environment_obstacles = polygons
        return Environment(
            t=t_capture,
            width_mm=ARENA_WIDTH_MM,
            height_mm=ARENA_HEIGHT_MM,
            obstacles=polygons,
            goal=tuple(selected_goal) if selected_goal is not None else None,
        )

    def mm_to_pixel(self, x, y):
        # 1. Appliquer la conversion inverse des millimetres en pixels
        return mm_to_pixel(x, y)

    def show_interface(self, estimate, path, goal):
        # Dimensions de la fenetre : zones gauche, droite et panneau inferieur
        main_width = 1400
        main_height = 800
        panel_height = 120
        arena_size = main_height - panel_height
        left_width = main_width // 2
        right_width = main_width - left_width
        panel_width = left_width

        interface = np.full((main_height, main_width, 3), 245, dtype=np.uint8)

        # 1. Afficher la camera en direct dans la zone gauche
        camera = self.frame.copy() if self.frame is not None else np.zeros((arena_size, arena_size, 3), dtype=np.uint8)
        camera = cv2.resize(camera, (arena_size, arena_size))
        interface[0:arena_size, 0:arena_size] = camera
        cv2.putText(interface, "CAMERA LIVE", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # 2. Creer la modelisation 2D dans la zone droite
        model = np.full((arena_size, right_width, 3), 255, dtype=np.uint8)
        origin_x = 30
        origin_y = 30
        arena_px = arena_size - 60
        scale = arena_px / ARENA_WIDTH_MM

        cv2.rectangle(model,
                      (origin_x, origin_y),
                      (origin_x + arena_px, origin_y + arena_px),
                      (30, 30, 30), 3)

        def world_to_model(x, y):
            px = int(round(origin_x + x * scale))
            py = int(round(origin_y + (ARENA_HEIGHT_MM - y) * scale))
            return px, py

        # Dessiner les obstacles detectes
        for polygon in getattr(self, "environment_obstacles", []):
            points = np.array([world_to_model(x, y) for x, y in polygon], dtype=np.int32)
            cv2.fillPoly(model, [points], (80, 80, 80))
            cv2.polylines(model, [points], True, (20, 20, 20), 2)

        # Dessiner les contours bruts detectes par la camera
        for contour in self.obstacle_contours:
            model_contour = []
            for px, py in contour[:, 0]:
                x, y = pixel_to_mm(px, py)
                model_contour.append(world_to_model(x, y))
            if len(model_contour) >= 3:
                cv2.polylines(model, [np.array(model_contour, dtype=np.int32)],
                              True, (0, 0, 180), 2)

        # 3. Dessiner le chemin calcule
        if path is not None and path.valid:
            pixels = [world_to_model(x, y) for x, y in path.waypoints]
            for start, end in zip(pixels, pixels[1:]):
                cv2.line(model, start, end, (0, 220, 220), 5)
            for point in pixels:
                cv2.circle(model, point, 7, (0, 220, 220), -1)

        # 4. Dessiner le but
        goal_px = world_to_model(*goal)
        cv2.circle(model, goal_px, 18, (0, 0, 220), -1)
        cv2.circle(model, goal_px, 21, (0, 0, 100), 2)

        # 5. Dessiner le Thymio estime
        if estimate is not None:
            pose_px = world_to_model(estimate.x, estimate.y)
            cv2.circle(model, pose_px, 16, (180, 0, 180), -1)
            arrow_length = 38
            arrow_end = (
                int(pose_px[0] + arrow_length * math.cos(estimate.theta)),
                int(pose_px[1] - arrow_length * math.sin(estimate.theta)),
            )
            cv2.arrowedLine(model, pose_px, arrow_end, (255, 0, 255), 4)

        interface[0:arena_size, panel_width:main_width] = model
        cv2.putText(interface, "MODELISATION 2D", (panel_width + 20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (30, 30, 30), 2)

        # 6. Afficher l'etat du robot en dessous
        status_y = arena_size
        cv2.rectangle(interface,
                      (0, status_y),
                      (main_width, main_height),
                      (25, 25, 25), -1)

        if estimate is None:
            status = "ROBOT : ATTENTE DE MESURE"
            position = "Position : non disponible"
            theta_text = "Theta : non disponible"
            measurement_text = "Camera : aucune mesure"
            path_text = "Chemin : non calcule"
            status_color = (0, 180, 255)
        else:
            status = "ROBOT : CAMERA PERDUE" if not estimate.measurement_used else "ROBOT : ACTIF"
            position = f"Position : x={estimate.x:.1f} mm, y={estimate.y:.1f} mm"
            theta_text = f"Theta : {estimate.theta:.3f} rad"
            measurement_text = "Camera : mesure utilisee" if estimate.measurement_used else "Camera : prediction seule"
            path_text = "Chemin : non calcule" if path is None else (
                "Chemin : valide" if path.valid else "Chemin : invalide"
            )
            status_color = (0, 200, 0) if estimate.measurement_used else (0, 180, 255)
            if estimate.kidnapped:
                status = "ROBOT : KIDNAPPING DETECTE"
                status_color = (0, 0, 255)

        cv2.putText(interface, status, (30, status_y + 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2)
        cv2.putText(interface, position, (30, status_y + 78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(interface, theta_text, (500, status_y + 78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(interface, measurement_text, (760, status_y + 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(interface, path_text, (760, status_y + 78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(interface, "q : quitter", (1200, status_y + 78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180, 180, 180), 2)

        cv2.imshow("Interface robot", interface)
        return cv2.waitKey(1) & 0xFF

    def close(self):
        self.capture.release()
        cv2.destroyAllWindows()
