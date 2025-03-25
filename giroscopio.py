from machine import I2C, Pin
from mpu6050 import MPU6050
import time

# Inicializar I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21))  # Ajusta los pines según tu configuración
mpu = MPU6050(i2c)

print("\nPrueba de Giroscopio HW-123 (MPU6050)")
print("--------------------------------------")

try:
    mpu.get_accel_data()
    print("MPU6050 listo!")
except:
    print("Error al inicializar el MPU6050")
    while True:
        pass

while True:
    gyro = mpu.get_gyro_data()
    accel = mpu.get_accel_data()
    
    print(f"Giroscopio -> X: {gyro['x']:.2f} | Y: {gyro['y']:.2f} | Z: {gyro['z']:.2f}")
    print(f"Acelerómetro -> X: {accel['x']:.2f} | Y: {accel['y']:.2f} | Z: {accel['z']:.2f}")
    
    time.sleep(0.5)
