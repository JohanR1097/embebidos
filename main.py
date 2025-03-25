from machine import Pin, PWM, I2C
import time
from vl53l0x import VL53L0X

# Definir pines del motor
ENA = Pin(5, Pin.OUT)
IN1 = Pin(18, Pin.OUT)
IN2 = Pin(19, Pin.OUT)
ENB = Pin(25, Pin.OUT)
IN3 = Pin(4, Pin.OUT)
IN4 = Pin(23, Pin.OUT)

# Configurar PWM para controlar la velocidad
pwm_A = PWM(ENA, freq=1000, duty=0)
pwm_B = PWM(ENB, freq=1000, duty=0)

# Pines de control de encendido de los sensores
XSHUT_FRONTAL = Pin(32, Pin.OUT)
XSHUT_LATERAL = Pin(33, Pin.OUT)

# Configurar I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21))

# Apagar sensores antes de inicializar
XSHUT_FRONTAL.off()
XSHUT_LATERAL.off()
time.sleep(0.01)

# Encender e inicializar sensores con diferentes direcciones
XSHUT_FRONTAL.on()
time.sleep(0.01)
sensor_frontal = VL53L0X(i2c)
sensor_frontal.set_address(0x30)

time.sleep(0.01)
XSHUT_LATERAL.on()
time.sleep(0.01)
sensor_lateral = VL53L0X(i2c)
sensor_lateral.set_address(0x31)

def medir_distancia(sensor):
    return sensor.read() if sensor.read() > 0 else 9999  # 9999 indica fuera de rango

def adelante():
    pwm_A.duty(500)
    IN1.on()
    IN2.off()
    pwm_B.duty(500)
    IN3.on()
    IN4.off()

def atras():
    pwm_A.duty(500)
    IN1.off()
    IN2.on()
    pwm_B.duty(500)
    IN3.off()
    IN4.on()

def izquierda():
    pwm_A.duty(500)
    IN1.on()
    IN2.off()
    pwm_B.duty(500)
    IN3.off()
    IN4.on()

def derecha():
    pwm_A.duty(500)
    IN1.off()
    IN2.on()
    pwm_B.duty(500)
    IN3.on()
    IN4.off()

def detener():
    pwm_A.duty(0)
    IN1.off()
    IN2.off()
    pwm_B.duty(0)
    IN3.off()
    IN4.off()

while True:
    distancia_frontal = medir_distancia(sensor_frontal)
    distancia_lateral = medir_distancia(sensor_lateral)
    
    print(f"Distancia Frontal: {distancia_frontal} mm")
    print(f"Distancia Lateral: {distancia_lateral} mm")
    
    if distancia_frontal > 100:
        adelante()
        print("Avanzando")
    else:
        detener()
        time.sleep(0.05)
        
        if distancia_frontal <= 100 and distancia_lateral <= 100:
            izquierda()
            print("Girando a la izquierda")
        elif distancia_lateral > 100:
            derecha()
            print("Girando a la derecha")
        else:
            izquierda()
            print("Girando a la izquierda")
    
    time.sleep(0.05)
