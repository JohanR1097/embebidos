from machine import Pin, PWM
import time

# Definición de pines
MOTOR_DERECHO_PIN1 = 12
MOTOR_DERECHO_PIN2 = 14
MOTOR_IZQUIERDO_PIN1 = 27
MOTOR_IZQUIERDO_PIN2 = 26

# Configuración de PWM
FREQ = 5000
RESOLUTION = 8

class ControlMotores:
    def __init__(self):
        self.motor_derecho_pwm = PWM(Pin(MOTOR_DERECHO_PIN1), freq=FREQ, duty=0)
        self.motor_izquierdo_pwm = PWM(Pin(MOTOR_IZQUIERDO_PIN1), freq=FREQ, duty=0)
        self.motor_derecho_dir = Pin(MOTOR_DERECHO_PIN2, Pin.OUT)
        self.motor_izquierdo_dir = Pin(MOTOR_IZQUIERDO_PIN2, Pin.OUT)
        self.detener()
    
    def avanzar(self):
        self.motor_derecho_dir.off()
        self.motor_izquierdo_dir.off()
        self.motor_derecho_pwm.duty(255)
        self.motor_izquierdo_pwm.duty(255)
        print("Movimiento: AVANZAR")
    
    def girar_derecha(self):
        self.motor_derecho_dir.off()
        self.motor_izquierdo_dir.off()
        self.motor_derecho_pwm.duty(128)
        self.motor_izquierdo_pwm.duty(255)
        print("Movimiento: GIRO DERECHA")
    
    def girar_izquierda(self):
        self.motor_derecho_dir.off()
        self.motor_izquierdo_dir.off()
        self.motor_derecho_pwm.duty(255)
        self.motor_izquierdo_pwm.duty(128)
        print("Movimiento: GIRO IZQUIERDA")
    
    def detener(self):
        self.motor_derecho_pwm.duty(0)
        self.motor_izquierdo_pwm.duty(0)
        print("Movimiento: DETENER")

motores = ControlMotores()

def main():
    print("\nPrueba de Motores ESP32")
    print("-----------------------------")
    time.sleep(1)
    
    motores.avanzar()
    time.sleep(2)
    motores.girar_derecha()
    time.sleep(1)
    motores.girar_izquierda()
    time.sleep(1)
    motores.detener()
    time.sleep(2)

if __name__ == "__main__":
    while True:
        main()
