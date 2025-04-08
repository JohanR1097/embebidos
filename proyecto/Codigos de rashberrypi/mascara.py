import time
import random
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2

BUTTON_PIN = 17

# Definir el rango HSV fijo basado en el valor seleccionado
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
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)  # Convertir a HSV
        
        # Aplicar la ma scara HSV
        mask = cv2.inRange(frame, LOWER_HSV, UPPER_HSV)
        masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
        
        cv2.imshow("Camera Preview (HSV)", masked_frame)

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
