import time
import random
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2

BUTTON_PIN = 17

# Definir rango HSV (ajstalo segn la luz si es necesario)
LOWER_HSV = np.array([120, 90, 119])
UPPER_HSV = np.array([140, 170, 199])

def on_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_value = frame[y, x]
        lower_bound = np.array([max(0, hsv_value[0] - 10), max(0, hsv_value[1] - 40), max(0, hsv_value[2] - 40)])
        upper_bound = np.array([min(179, hsv_value[0] + 10), min(255, hsv_value[1] + 40), min(255, hsv_value[2] + 40)])
        print(f"HSV seleccionado: {hsv_value}")
        print(f"Rango HSV: Inferior {lower_bound}, Superior {upper_bound}")

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

picam2 = Picamera2()
resolutions = [(640, 480), (800, 600), (1024, 768), (1280, 720), (1920, 1080)]

picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
picam2.start()

cv2.namedWindow("Camera Preview (HSV)")
cv2.setMouseCallback("Camera Preview (HSV)", on_mouse_click)

try:
    while True:
        frame = picam2.capture_array()
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

        # Suavizado y cierre para juntar puntos cercanos
        kernel = np.ones((5, 5), np.uint8)
        mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filtrar contornos pequos (omitimos manchas grandes y puntos ruidosos)
        valid_contours = [cnt for cnt in contours if 5 < cv2.contourArea(cnt) < 300]

        if valid_contours:
            # Unir todos los contornos en uno solo
            all_points = np.vstack(valid_contours)
            x, y, w, h = cv2.boundingRect(all_points)

            # Dibujar el recuadro
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            # Calcular el centro del rectangulo
            center_x = x + w // 2
            center_y = y + h // 2
            radius = min(w, h) // 64 # Radio opcional, un cuarto del ancho o alto menor
            cv2.circle(frame, (center_x, center_y), radius, (255, 0, 0), 2)
            cv2.putText(frame, "Matriz detectada", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Mostrar mcara y frame final
        cv2.imshow("Camera Preview (HSV)", frame)

        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            new_res = random.choice(resolutions)
            print(f"Cambiando resolucion a: {new_res}")
            picam2.stop()
            picam2.configure(picam2.create_preview_configuration(main={"size": new_res, "format": "RGB888"}))
            picam2.start()
            time.sleep(0.5)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

GPIO.cleanup()
picam2.stop()
cv2.destroyAllWindows()

