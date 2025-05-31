from machine import Pin, PWM
import time

class Motores:
    def __init__(self):
        # Pines de control para L298N
        self.IN1 = Pin(18, Pin.OUT)
        self.IN2 = Pin(19, Pin.OUT)
        self.IN3 = Pin(4, Pin.OUT)
        self.IN4 = Pin(23, Pin.OUT)
        self.ENA = PWM(Pin(5), freq=1000)
        self.ENB = PWM(Pin(25), freq=1000)

        # Estado inicial
        self.estado_actual = "detenido"
        self.detener_motores()
        time.sleep_ms(20)

    def mover_adelante(self, velocidad):
        velocidad = max(0, min(1023, int(velocidad)))
        self.IN1.value(0)
        self.IN2.value(1)
        self.IN3.value(0)
        self.IN4.value(1)
        self.ENA.duty(velocidad)
        self.ENB.duty(velocidad)
        self.estado_actual = "andando"

    def girar_derecha(self, velocidad):
        velocidad = max(0, min(1023, int(velocidad)))
        self.IN1.value(0)
        self.IN2.value(1)
        self.IN3.value(1)
        self.IN4.value(0)
        self.ENA.duty(velocidad)
        self.ENB.duty(velocidad)
        self.estado_actual = "girando"

    def girar_izquierda(self, velocidad):
        velocidad = max(0, min(1023, int(velocidad)))
        self.IN1.value(1)
        self.IN2.value(0)
        self.IN3.value(0)
        self.IN4.value(1)
        self.ENA.duty(velocidad)
        self.ENB.duty(velocidad)
        self.estado_actual = "girando"

    def mover_ruedas(self, velocidad_izquierda, velocidad_derecha):
        velocidad_izquierda = max(0, min(1023, int(velocidad_izquierda)))
        velocidad_derecha = max(0, min(1023, int(velocidad_derecha)))
        self.IN1.value(0)
        self.IN2.value(1)
        self.IN3.value(0)
        self.IN4.value(1)
        self.ENA.duty(velocidad_izquierda)
        self.ENB.duty(velocidad_derecha)
        self.estado_actual = "andando"

    def detener_motores(self):
        self.ENA.duty(0)
        self.ENB.duty(0)
        self.IN1.value(0)
        self.IN2.value(0)
        self.IN3.value(0)
        self.IN4.value(0)
        self.estado_actual = "detenido"

    def en_movimiento(self):
        return self.estado_actual != "detenido"
