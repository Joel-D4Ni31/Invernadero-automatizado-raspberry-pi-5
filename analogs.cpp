#include <ArduinoJson.h>

/* ──────────────── CONFIGURACIÓN DE PINES ──────────────── */

// FC-28 (sensores resistivos de humedad)
const int FC28_1_PIN = A0;
const int FC28_2_PIN = A1;

// MQ-135 (calidad de aire, analógico)
const int MQ135_PIN  = A2;

// Sensores capacitivos de humedad
const int HUMCAP_1_PIN = A3;
const int HUMCAP_2_PIN = A4;

// FC-37 (sensor de lluvia/nieve, analógico)
const int FC37_PIN   = A5;

/* ──────────────── CONSTANTES DE CALIBRACIÓN ──────────────── */
const int DRY = 1023;  // valor en seco
const int WET = 300;   // valor en agua

/* ──────────────── VARIABLES DE CONTROL ──────────────── */
unsigned long lastSend = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  unsigned long current = millis();
  if (current - lastSend >= 2000) { // cada 2 segundos
    lastSend = current;

    // Crear JSON
    StaticJsonDocument<256> doc;

    /* --- FC-28 --- */
    int fc28_1_val = analogRead(FC28_1_PIN);
    int fc28_1_hum = map(fc28_1_val, DRY, WET, 0, 100);
    doc["fc28_1"] = constrain(fc28_1_hum, 0, 100);

    int fc28_2_val = analogRead(FC28_2_PIN);
    int fc28_2_hum = map(fc28_2_val, DRY, WET, 0, 100);
    doc["fc28_2"] = constrain(fc28_2_hum, 0, 100);

    /* --- MQ-135 --- */
    int mq135_val = analogRead(MQ135_PIN);
    doc["mq135"] = mq135_val;

    /* --- Humedad Capacitivos --- */
    int humcap_1_val = analogRead(HUMCAP_1_PIN);
    int humcap_1_hum = map(humcap_1_val, DRY, WET, 0, 100);
    doc["humcap_1"] = constrain(humcap_1_hum, 0, 100);

    int humcap_2_val = analogRead(HUMCAP_2_PIN);
    int humcap_2_hum = map(humcap_2_val, DRY, WET, 0, 100);
    doc["humcap_2"] = constrain(humcap_2_hum, 0, 100);

    /* --- FC-37 (lluvia) --- */
    int fc37_val = analogRead(FC37_PIN);
    int fc37_porc = map(fc37_val, 1023, 0, 0, 100); 
    doc["fc37"] = constrain(fc37_porc, 0, 100);

    // Enviar JSON por Serial
    serializeJson(doc, Serial);
    Serial.println();
  }
}