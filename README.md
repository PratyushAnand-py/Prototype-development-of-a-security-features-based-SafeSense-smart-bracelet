Project Overview

SafeSense is a prototype smart bracelet designed to enhance personal security by monitoring biometric and environmental data in real-time. This multi-sensor system detects potential danger situations through physiological stress indicators (heart rate and galvanic skin response) combined with movement and location tracking.

Key features:

- Real-time heart rate monitoring using MAX30105 sensor
- Stress detection via GSR (Galvanic Skin Response) sensor
- Location tracking with NEO-6M GPS module
- WiFi-enabled data transmission and web interface
- IoT integration with ThingSpeak platform

Hardware Components

- ESP32-WROVER Development Board (Microcontroller)
- MAX30105 (Heart rate and SpO₂ sensor)
- Grove GSR Sensor (Skin conductance for stress detection)
- NEO-6M GPS Module (Location tracking)
- Breadboard and jumper wires

Software Architecture

- Arduino-based firmware (C++)
- Web server interface (Embedded HTML)
- ThingSpeak IoT integration
- TinyGPS++ library for GPS data processing
- MAX3010x library for heart rate monitoring

Key Functionality

- Biometric Monitoring:
- Continuous heart rate measurement (BPM)
- Stress detection through skin conductance (µS)

Location Tracking:

- Real-time GPS coordinates
- Movement speed and altitude

Alert System:

- Combined biometric thresholds trigger alerts
- Location data included in alerts

Data Visualization:

- Local web interface (TCP/IP)
- Cloud monitoring via ThingSpeak

SafeSense/
├── src/                         # Main source code
│   ├── finalized_code_project_b_version_1.cpp  # Primary firmware
│   ├── MAX30105_BPM_Sensor.cpp  # Heart rate sensor module
│   ├── Grove_GSR_Sensor.cpp     # GSR sensor module
│   └── neo6m.cpp                # GPS module
├── docs/                        # Documentation
│   └── Project_B_Report.docx    # Complete project report
└── platformio.ini               # Build configuration

Installation & Setup
Prerequisites

- Arduino IDE or PlatformIO (VS Code extension)
- ESP32 board support package

Required libraries:

- TinyGPS++
- MAX3010x
- Adafruit BusIO

Usage

- Upload the firmware to ESP32
- Connect to the device's WiFi access point
- Access the web interface at http://[ESP32_IP]
- View real-time sensor data:
  * Heart rate (BPM)
  * GSR readings (µS)
  * GPS coordinates
  * Movement data
 
Future Enhancements

- Miniaturization to wearable form factor
- Machine learning for false positive reduction
- Additional sensors (temperature, accelerometer)
- Bluetooth Low Energy (BLE) connectivity
- Mobile app integration

Documentation

Complete project documentation is available in Project_B_Report.docx, including:
- Theoretical background
- Component specifications
- Implementation details
- Results analysis
- Future scope

License

This project is open-source under the MIT License.

Acknowledgements
- Prof. Dr.-Ing. Christian Karnutsch (Project Supervisor)
- Research teams at FAU Erlangen-Nuremberg for biomechanical stress research
- Open-source library developers (TinyGPS++, MAX3010x, etc.)

For questions or contributions, please contact the project author.
