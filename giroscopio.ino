#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    Serial.println("\nPrueba de Giroscopio HW-123 (MPU6050)");
    Serial.println("--------------------------------------");
    
    if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
        Serial.println("Error al inicializar el MPU6050");
        while (1);
    }
    
    Serial.println("MPU6050 listo!");
}

void loop() {
    Vector gyro = mpu.readRawGyro();
    Vector accel = mpu.readRawAccel();
    
    Serial.print("Giroscopio -> X: ");
    Serial.print(gyro.XAxis);
    Serial.print(" | Y: ");
    Serial.print(gyro.YAxis);
    Serial.print(" | Z: ");
    Serial.println(gyro.ZAxis);
    
    Serial.print("Acelerómetro -> X: ");
    Serial.print(accel.XAxis);
    Serial.print(" | Y: ");
    Serial.print(accel.YAxis);
    Serial.print(" | Z: ");
    Serial.println(accel.ZAxis);
    
    delay(500);
}
