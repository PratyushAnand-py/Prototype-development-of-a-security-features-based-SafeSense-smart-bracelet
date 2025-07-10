// finalized code version 2- with TCP/IP Stack

#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>

// ========== WiFi ==========
const char* ssid = "OnePlus 7";
const char* password = "12345678";
WebServer server(80);

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

// ========== Helper Functions ==========
float smoothGSR() {
  gsrSum -= gsrSamples[gsrIndex];
  delay(10);
  gsrSamples[gsrIndex] = analogRead(GSR_PIN);
  gsrSum += gsrSamples[gsrIndex];
  gsrIndex = (gsrIndex + 1) % GSR_SAMPLE_SIZE;
  return (float)gsrSum / GSR_SAMPLE_SIZE;
}

float rawToConductance(int raw) {
  if(raw <= 1) return 0;
  return (1.0 / ((4095.0 / raw - 1) * 10000.0)) * 1000000;
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());

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
  Serial.println("Initializing GSR...");
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(GSR_PIN, ADC_11db);
  for (int i = 0; i < GSR_SAMPLE_SIZE; i++) {
    gsrSamples[i] = analogRead(GSR_PIN);
    gsrSum += gsrSamples[i];
  }

  // GPS
  Serial.println("Initializing GPS...");
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Web Server
  server.on("/", []() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta http-equiv='refresh' content='1'>"
                  "<title>ESP32 Sensor Monitor</title></head><body style='font-family:Arial;background:#f9f9f9;padding:20px;'>";
    html += "<h2>ESP32 Multi-Sensor Monitor</h2><hr>";

    // Heart Rate Sensor
    long irValue = particleSensor.getIR();
    html += "<h3>Heart Rate Sensor</h3>";
    html += "IR Value: " + String(irValue) + "<br>";
    html += "Current BPM: " + String(beatsPerMinute, 1) + "<br>";
    html += "Average BPM: " + String(beatAvg) + "<br>";
    if (irValue < 50000) html += "<span style='color:red;'>WARNING: Finger not detected</span><br>";

    // GSR
    float smoothedGSR = smoothGSR();
    float conductance = rawToConductance(smoothedGSR);
    html += "<h3>GSR Sensor</h3>";
    html += "Raw Value: " + String(smoothedGSR, 1) + "<br>";
    html += "Conductance: " + String(conductance, 2) + " µS<br>";

    // GPS
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
    html += "<h3>GPS Data</h3>";
    if (gps.location.isValid()) {
      html += "Latitude: " + String(gps.location.lat(), 6) + "<br>";
      html += "Longitude: " + String(gps.location.lng(), 6) + "<br>";
      html += "Speed: " + String(gps.speed.kmph(), 2) + " km/h<br>";
      html += "Altitude: " + String(gps.altitude.meters(), 2) + " m<br>";
      html += "Satellites: " + String(gps.satellites.value()) + "<br>";
    } else {
      html += "No GPS fix available<br>";
    }

    html += "</body></html>";
    server.send(200, "text/html", html);
  });

  server.begin();
  Serial.println("Web server started.");
}

// ========== Loop ==========
void loop() {
  server.handleClient();

  // Heartbeat detection
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

    Serial.printf("[HEARTBEAT] BPM: %.1f, Avg: %d\n", beatsPerMinute, beatAvg);
  }

  // GSR
  float smoothedGSR = smoothGSR();
  float conductance = rawToConductance(smoothedGSR);
  Serial.printf("[GSR] Raw: %.1f, Conductance: %.2f µS\n", smoothedGSR, conductance);

  // GPS
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (gps.location.isValid()) {
    Serial.printf("[GPS] Lat: %.6f, Lng: %.6f, Speed: %.2f km/h, Alt: %.2f m, Sats: %d\n",
      gps.location.lat(),
      gps.location.lng(),
      gps.speed.kmph(),
      gps.altitude.meters(),
      gps.satellites.value());
  } else {
    Serial.println("[GPS] No GPS fix");
  }

  delay(1000); // Match 1-second refresh rate
}
