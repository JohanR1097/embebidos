from machine import Pin
from neopixel import NeoPixel
import time

class MatrizLED:
    def __init__(self, pin=13, num_leds=64):
        self.np = NeoPixel(Pin(pin), num_leds)
        self.NUM_OF_LED = num_leds

    def encender_azul(self):
        for i in range(self.NUM_OF_LED):
            self.np[i] = (0, 0, 50)
        self.np.write()

    def apagar(self):
        for i in range(self.NUM_OF_LED):
            self.np[i] = (0, 0, 0)
        self.np.write()

    def mostrar_patron(self, matriz_leds, color):
        """matriz_leds: lista 8x8, color: [R,G,B]"""
        # Suponiendo matriz de 8x8, mapeo secuencial (puedes adaptar si tu wiring es diferente)
        for fila in range(8):
            for col in range(8):
                idx = fila * 8 + col  # Cambia este mapeo si tu matriz tiene otro orden
                if matriz_leds[fila][col]:
                    self.np[idx] = tuple(color)
                else:
                    self.np[idx] = (0, 0, 0)
        self.np.write()
