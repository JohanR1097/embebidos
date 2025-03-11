#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensorFrente;
VL53L0X sensorIzquierdo;
VL53L0X sensorDerecho;

#define XSHUT_FRENTE 32
#define XSHUT_IZQ 33
#define XSHUT_DER 25

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
    configurarSensores();
    Serial.println("\nPrueba de Sensores VL53L0X");
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
    
    delay(500);
}
