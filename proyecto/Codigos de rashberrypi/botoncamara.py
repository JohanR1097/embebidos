import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2

BUTTON_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

picam2 = Picamera2()
config = picam2.create_still_configuration()
picam2.configure(config)
picam2.start()

try:
    while True:
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            print("Tomando foto...")
            picam2.capture_file("captura.jpg")
            print("Imagen guardada como 'captura.jpg'.")
            time.sleep(0.5) 
except KeyboardInterrupt:
    GPIO.cleanup()
    picam2.stop()
