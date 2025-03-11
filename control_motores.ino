#define MOTOR_DERECHO_PIN1 12  // PWM
#define MOTOR_DERECHO_PIN2 14  // PWM
#define MOTOR_IZQUIERDO_PIN1 27  // PWM
#define MOTOR_IZQUIERDO_PIN2 26  // PWM
#define ENABLE_PIN1 13
#define ENABLE_PIN2 17

class ControlMotores {
public:
    ControlMotores() {
        ledcSetup(0, 5000, 8);
        ledcSetup(1, 5000, 8);
        ledcAttachPin(MOTOR_DERECHO_PIN1, 0);
        ledcAttachPin(MOTOR_IZQUIERDO_PIN1, 1);
        detener();
    }

    void avanzar() {
        digitalWrite(MOTOR_DERECHO_PIN2, LOW);
        digitalWrite(MOTOR_IZQUIERDO_PIN2, LOW);
        ledcWrite(0, 255);
        ledcWrite(1, 255);
        Serial.println("Movimiento: AVANZAR");
    }

    void girarDerecha() {
        digitalWrite(MOTOR_DERECHO_PIN2, LOW);
        digitalWrite(MOTOR_IZQUIERDO_PIN2, LOW);
        ledcWrite(0, 128);
        ledcWrite(1, 255);
        Serial.println("Movimiento: GIRO DERECHA");
    }

    void girarIzquierda() {
        digitalWrite(MOTOR_DERECHO_PIN2, LOW);
        digitalWrite(MOTOR_IZQUIERDO_PIN2, LOW);
        ledcWrite(0, 255);
        ledcWrite(1, 128);
        Serial.println("Movimiento: GIRO IZQUIERDA");
    }

    void detener() {
        ledcWrite(0, 0);
        ledcWrite(1, 0);
        Serial.println("Movimiento: DETENER");
    }
};

ControlMotores motores;

void setup() {
    Serial.begin(115200);
    pinMode(MOTOR_DERECHO_PIN2, OUTPUT);
    pinMode(MOTOR_IZQUIERDO_PIN2, OUTPUT);
    Serial.println("\nPrueba de Motores ESP32");
    Serial.println("-----------------------------");
    delay(1000);
}

void loop() {
    motores.avanzar();
    delay(2000);
    motores.girarDerecha();
    delay(1000);
    motores.girarIzquierda();
    delay(1000);
    motores.detener();
    delay(2000);
}
