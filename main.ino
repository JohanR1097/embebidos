#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensorFrente;
VL53L0X sensorIzquierdo;
VL53L0X sensorDerecho;

#define XSHUT_FRENTE 32
#define XSHUT_IZQ 33
#define XSHUT_DER 25

#define MOTOR_DERECHO_PIN1 12
#define MOTOR_DERECHO_PIN2 14
#define MOTOR_IZQUIERDO_PIN1 27
#define MOTOR_IZQUIERDO_PIN2 26
#define ENABLE_PIN1 13
#define ENABLE_PIN2 17

#define UMBRAL_SEGURIDAD 300
#define TIEMPO_GIRO 450
#define VELOCIDAD_LECTURA 50

class ControlMotores {
public:
    ControlMotores() {
        pinMode(ENABLE_PIN1, OUTPUT);
        pinMode(ENABLE_PIN2, OUTPUT);
        detener(); // Asegurar que los motores están detenidos al inicio
    }

    void avanzar() {
        controlMotor(MOTOR_DERECHO_PIN1, MOTOR_DERECHO_PIN2, MOTOR_IZQUIERDO_PIN1, MOTOR_IZQUIERDO_PIN2, 255, 255);
        Serial.println("Movimiento: AVANZAR");
    }

    void girarDerecha() {
        controlMotor(MOTOR_DERECHO_PIN1, MOTOR_DERECHO_PIN2, MOTOR_IZQUIERDO_PIN1, MOTOR_IZQUIERDO_PIN2, 128, 255);
        Serial.println("Movimiento: GIRO DERECHA");
    }

    void girarIzquierda() {
        controlMotor(MOTOR_DERECHO_PIN1, MOTOR_DERECHO_PIN2, MOTOR_IZQUIERDO_PIN1, MOTOR_IZQUIERDO_PIN2, 255, 128);
        Serial.println("Movimiento: GIRO IZQUIERDA");
    }

    void detener() {
        controlMotor(MOTOR_DERECHO_PIN1, MOTOR_DERECHO_PIN2, MOTOR_IZQUIERDO_PIN1, MOTOR_IZQUIERDO_PIN2, 0, 0);
        Serial.println("Movimiento: DETENER");
    }

private:
    void controlMotor(int pin1, int pin2, int pin3, int pin4, int duty1, int duty2) {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        digitalWrite(pin3, HIGH);
        digitalWrite(pin4, LOW);
        ledcWrite(0, duty1);
        ledcWrite(1, duty2);
    }
};

ControlMotores motores;

void configurarSensores() {
    pinMode(XSHUT_FRENTE, OUTPUT);
    pinMode(XSHUT_IZQ, OUTPUT);
    pinMode(XSHUT_DER, OUTPUT);
    digitalWrite(XSHUT_FRENTE, LOW);
    digitalWrite(XSHUT_IZQ, LOW);
    digitalWrite(XSHUT_DER, LOW);
    delay(10);
    digitalWrite(XSHUT_FRENTE, HIGH);
    delay(10);
    sensorFrente.init();
    sensorFrente.setAddress(0x30);
    sensorFrente.startContinuous();
    digitalWrite(XSHUT_IZQ, HIGH);
    delay(10);
    sensorIzquierdo.init();
    sensorIzquierdo.setAddress(0x31);
    sensorIzquierdo.startContinuous();
    digitalWrite(XSHUT_DER, HIGH);
    delay(10);
    sensorDerecho.init();
    sensorDerecho.setAddress(0x32);
    sensorDerecho.startContinuous();
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(MOTOR_DERECHO_PIN1, OUTPUT);
    pinMode(MOTOR_DERECHO_PIN2, OUTPUT);
    pinMode(MOTOR_IZQUIERDO_PIN1, OUTPUT);
    pinMode(MOTOR_IZQUIERDO_PIN2, OUTPUT);

    configurarSensores();
    motores.detener();  // Ya no necesitamos llamar a setupMotorController()

    Serial.println("\nSistema de Navegación Autónoma");
    Serial.println("-----------------------------");
    delay(1000);
}

void loop() {
    uint16_t distanciaFrente = sensorFrente.readRangeContinuousMillimeters();
    uint16_t distanciaIzq = sensorIzquierdo.readRangeContinuousMillimeters();
    uint16_t distanciaDer = sensorDerecho.readRangeContinuousMillimeters();

    Serial.print("Frente: ");
    Serial.print(distanciaFrente);
    Serial.print("mm | Izq: ");
    Serial.print(distanciaIzq);
    Serial.print("mm | Der: ");
    Serial.println(distanciaDer);

    if (distanciaFrente < UMBRAL_SEGURIDAD || distanciaIzq < UMBRAL_SEGURIDAD || distanciaDer < UMBRAL_SEGURIDAD) {
        motores.detener();
        delay(100);
        if (distanciaFrente < UMBRAL_SEGURIDAD) {
            (distanciaDer > distanciaIzq) ? motores.girarDerecha() : motores.girarIzquierda();
        } else if (distanciaIzq < UMBRAL_SEGURIDAD) {
            motores.girarDerecha();
        } else {
            motores.girarIzquierda();
        }
        delay(TIEMPO_GIRO);
    } else {
        motores.avanzar();
    }
    delay(VELOCIDAD_LECTURA);
}
