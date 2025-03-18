from machine import Pin, I2C
from vl53l0x import VL53L0X
import time

# Definición de pines I2C
I2C_SDA = 21  # Pin SDA
I2C_SCL = 22  # Pin SCL

# Inicializar I2C
i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL))

# Inicializar el sensor VL53L0X
sensor = VL53L0X(i2c)

# Iniciar medición continua
sensor.start_continuous()

print("Sensor VL53L0X listo!")

while True:
    distancia = sensor.read()
    print("Distancia:", distancia, "mm")
    time.sleep(0.5)
