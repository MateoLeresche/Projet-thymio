
import cv2


def detect_arena(frame):

    # 1. Convertir en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 2. Appliquer un filtre gaussien
    gaussian = cv2.GaussianBlur(gray, (9, 9), 0)

    # 3. Appliquer le seuillage d'Otsu
    _, otsu = cv2.threshold(
        gaussian, 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    # 4. Détecter les contours
    contours, _ = cv2.findContours(
        otsu,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) == 0:
        return None, None

    # 5. Garder le plus grand contour
    largest_contour = max(contours, key=cv2.contourArea)

    # 6. Approximer le contour par un polygone
    perimeter = cv2.arcLength(largest_contour, True)
    epsilon = 0.02 * perimeter

    approx = cv2.approxPolyDP(
        largest_contour,
        epsilon,
        True
    )

    # 7. Dessiner le résultat
    result = frame.copy()

    cv2.drawContours(
        result,
        [largest_contour],
        -1,
        (0, 255, 0),
        3
    )

    # 8. Vérifier si on a trouvé quatre coins
    if len(approx) == 4:

        corners = approx.reshape(4, 2)

        for point in corners:
            x, y = point
            cv2.circle(result, (int(x), int(y)), 8, (0, 0, 255), -1)

        cv2.putText(
            result,
            "Terrain detecte",
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        return result, corners

    return result, None


if __name__ == "__main__":

    cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)

    if not cap.isOpened():
        print("Impossible d'ouvrir la webcam")
        exit()

    while True:

        ret, frame = cap.read()

        if not ret:
            print("Impossible de recevoir une image")
            break

        result, corners = detect_arena(frame)

        cv2.imshow("Detection du terrain - Thymio", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
