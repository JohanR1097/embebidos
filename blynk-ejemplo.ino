#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char auth[] = "TU_AUTH_TOKEN";  // Reemplaza con tu token de Blynk
char ssid[] = "TU_SSID";        // Reemplaza con el nombre de tu WiFi
char pass[] = "TU_PASSWORD";    // Reemplaza con la contraseña de tu WiFi

void setup() {
    Serial.begin(115200);
    Blynk.begin(auth, ssid, pass);
    pinMode(2, OUTPUT);  // Configura el LED en el pin 2
}

BLYNK_WRITE(V1) {  // Virtual Pin V1 para controlar el LED
    int estado = param.asInt();
    digitalWrite(2, estado);
}

void loop() {
    Blynk.run();
}
