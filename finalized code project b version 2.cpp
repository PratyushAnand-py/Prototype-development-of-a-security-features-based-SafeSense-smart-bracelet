// finalized code version 3- with MQTT 

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// ========== WiFi ==========
const char* ssid = "OnePlus 7";
const char* password = "12345678";

// ========== ThingSpeak ==========
const char* apiKey = "YOUR_THINGSPEAK_WRITE_API_KEY";
const char* serverName = "https://thingspeak.mathworks.com/channels/3004261/private_show";

// ========== MAX30105 ==========
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

// ========== GSR ==========
const int GSR_PIN = 36;
const int GSR_SAMPLE_SIZE = 20;
int gsrSamples[GSR_SAMPLE_SIZE];
int gsrIndex = 0;
long gsrSum = 0;

// ========== GPS ==========
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 9600
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// ========== GSR Helpers ==========
float smoothGSR() {
  gsrSum -= gsrSamples[gsrIndex];
  delay(10);
  gsrSamples[gsrIndex] = analogRead(GSR_PIN);
  gsrSum += gsrSamples[gsrIndex];
  gsrIndex = (gsrIndex + 1) % GSR_SAMPLE_SIZE;
  return (float)gsrSum / GSR_SAMPLE_SIZE;
}

float rawToConductance(int raw) {
  if (raw <= 1) return 0;
  return (1.0 / ((4095.0 / raw - 1) * 10000.0)) * 1000000;
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nConnected. IP: " + WiFi.localIP().toString());

  // MAX30105
  Serial.println("Initializing MAX30105...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found!");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  // GSR
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (int i = 0; i < GSR_SAMPLE_SIZE; i++) {
    gsrSamples[i] = analogRead(GSR_PIN);
    gsrSum += gsrSamples[i];
  }

  // GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

// ========== Loop ==========
void loop() {
  // Heartbeat
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute > 20 && beatsPerMinute < 255) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
    Serial.printf("[HEART] BPM: %.1f, Avg: %d\n", beatsPerMinute, beatAvg);
  }

  // GSR
  float smoothedGSR = smoothGSR();
  float conductance = rawToConductance(smoothedGSR);
  Serial.printf("[GSR] Raw: %.1f, Conductance: %.2f µS\n", smoothedGSR, conductance);

  // GPS
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  float lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  float lng = gps.location.isValid() ? gps.location.lng() : 0.0;
  float spd = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  float alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  Serial.printf("[GPS] Lat: %.6f, Lng: %.6f, Speed: %.2f, Alt: %.2f\n", lat, lng, spd, alt);

  // Send to ThingSpeak
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(serverName) +
                 "?api_key=" + apiKey +
                 "&field1=" + String(beatsPerMinute, 1) +
                 "&field2=" + String(beatAvg) +
                 "&field3=" + String(smoothedGSR, 1) +
                 "&field4=" + String(conductance, 2) +
                 "&field5=" + String(lat, 6) +
                 "&field6=" + String(lng, 6) +
                 "&field7=" + String(spd, 2) +
                 "&field8=" + String(alt, 2);

    http.begin(url);
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      Serial.println("[ThingSpeak] Data sent. Code: " + String(httpResponseCode));
    } else {
      Serial.println("[ThingSpeak] Failed. Error: " + String(http.errorToString(httpResponseCode)));
    }
    http.end();
  }

  delay(15000); // ThingSpeak requires 15 sec minimum interval
}
