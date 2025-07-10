//Grove GSR sensor 

#include <Arduino.h>
#include <BioData.h>

const int GSR_PIN = 36;       // GPIO36 (Analog-capable pin)
const int SAMPLE_SIZE = 20;   // Number of samples for smoothing
const int DELAY_MS = 20;      // Delay between samples (ms)

// Moving average smoothing function
float smoothGSR() {
  static int samples[SAMPLE_SIZE];
  static int index = 0;
  static long sum = 0;
  
  // Remove oldest sample from sum
  sum -= samples[index];
  
  // Take new reading (with 10ms stabilization delay)
  delay(10);
  samples[index] = analogRead(GSR_PIN);
  
  // Add new sample to sum
  sum += samples[index];
  
  // Update index
  index = (index + 1) % SAMPLE_SIZE;
  
  // Return smoothed average
  return (float)sum / SAMPLE_SIZE;
}

// Convert raw ADC to conductance (µS)
float rawToConductance(int raw) {
  if(raw <= 1) return 0;  // Prevent divide-by-zero
  
  // Scientific conversion formula:
  // Resistance = (4095/raw - 1) * 10kΩ
  // Conductance (µS) = (1/Resistance) * 1,000,000
  return (1.0 / ((4095.0 / raw - 1) * 10000.0)) * 1000000;
}

void setup() {
  Serial.begin(115200);
  
  // Configure ESP32 ADC for optimal GSR readings
  analogReadResolution(12);       // 12-bit resolution (0-4095)
  analogSetAttenuation(ADC_11db); // 0-3.3V range
  analogSetPinAttenuation(GSR_PIN, ADC_11db);
  
  Serial.println("GSR Sensor Stabilizing...");
  delay(2000);  // Allow sensor to stabilize
}

void loop() {
  float smoothedRaw = smoothGSR();
  float conductance = rawToConductance(smoothedRaw);
  
  // Print with scientific formatting
  Serial.print("Smoothed Raw: ");
  Serial.print(smoothedRaw, 1);
  Serial.print("\tConductance: ");
  Serial.print(conductance, 2);
  Serial.println(" µS");
  
  delay(DELAY_MS);
}