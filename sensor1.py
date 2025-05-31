from machine import Pin, I2C
from vl530x import VL53L0X
import time

class SensoresLaser:
    def __init__(self):
        # Configuración de pines XSHUT para cada sensor
        self.XSHUT_PINS = [33, 32]  # Pines XSHUT para el sensor frontal
        self.SENSOR_COUNT = len(self.XSHUT_PINS)

        # Inicialización del bus I2C
        self.i2c = I2C(0, sda=Pin(21), scl=Pin(22))

        # Lista para almacenar los objetos de los sensores
        self.sensors = []

        # Configuración inicial de los sensores
        for i in range(self.SENSOR_COUNT):
            address = 0x30 + i
            sensor = self.setup_sensor(self.XSHUT_PINS[i], address)
            self.sensors.append(sensor)

    def setup_sensor(self, xshut_pin, address):
        # Apagar el sensor
        xshut = Pin(xshut_pin, Pin.OUT)
        xshut.value(0)
        time.sleep_ms(10)

        # Encender el sensor
        xshut.value(1)
        time.sleep_ms(10)

        # Inicializar el sensor con la dirección predeterminada (0x29)
        sensor = VL53L0X(self.i2c, address=0x29)
        sensor.start()

        # Cambiar la dirección I2C del sensor
        sensor.change_address(address)
        return sensor

    def leer_distancias(self):
        # Leer las distancias de los sensores
        distancia_izquierda = self.sensors[0].read()
        distancia_frontal = self.sensors[1].read() 
        return distancia_frontal, distancia_izquierda

