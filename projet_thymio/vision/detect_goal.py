import cv2
import numpy as np
from .calibration import pixel_to_mm

MIN_GOAL_AREA = 300


def detect_red_goal(img):
    """Détecte le cercle rouge plein et retourne son centre en pixels/mm."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red_1 = np.array([0, 100, 100], dtype=np.uint8)
    upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_red_2 = np.array([170, 100, 100], dtype=np.uint8)
    upper_red_2 = np.array([179, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask |= cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            np.ones((7, 7), dtype=np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((5, 5), dtype=np.uint8))

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    result = img.copy()
    best = None

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_GOAL_AREA:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue

        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if circularity < 0.65:
            continue

        (px, py), radius = cv2.minEnclosingCircle(contour)
        candidate = (area, (float(px), float(py)), float(radius), contour)
        if best is None or candidate[0] > best[0]:
            best = candidate

    if best is None:
        cv2.putText(
            result,
            "Goal rouge non detecte",
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
        )
        return result, None, None

    _, center, radius, contour = best
    px, py = center
    x, y = pixel_to_mm(px, py)

    cv2.drawContours(result, [contour], -1, (0, 255, 255), 3)
    cv2.circle(result, (int(px), int(py)), int(radius), (0, 255, 255), 2)
    cv2.drawMarker(
        result,
        (int(px), int(py)),
        (0, 255, 0),
        cv2.MARKER_CROSS,
        24,
        3,
    )
    cv2.putText(
        result,
        f"Goal: {x:.1f}, {y:.1f} mm",
        (20, 100),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
    )

    return result, (x, y), (px, py, radius)
