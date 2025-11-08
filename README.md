# 🛫 Real-Time Avionics Primary Flight Display (PFD) Using ESP32 & Processing  
### Boeing-Style Glass Cockpit • Real-Time Sensor Fusion • Embedded Avionics System

This project implements a fully functional **Primary Flight Display (PFD)** similar to those found in commercial aircraft
An **ESP32** collects and fuses sensor data (attitude, altitude, heading), and a **Processing 4.x** application renders a real-time **Boeing-style PFD**.

---

## ✈️ Overview

This system measures and visualizes:

- ✅ Roll (Bank angle)  
- ✅ Pitch (Attitude)  
- ✅ Heading (Gyro-based integration)  
- ✅ Altitude (BMP280 barometric pressure)  
- ✅ VSI – Vertical Speed Indicator (altitude rate)  
- ✅ IAS (optional 6th field from ESP32)  
- ✅ IMU failover using ADXL345  

The Processing application displays:

- Boeing-style artificial horizon  
- Pitch ladder (5°/10° markers)  
- Bank-angle scale (10°/20°/30°/45°/60°)  
- Flight Director (FD) bars (magenta)  
- IAS tape (left)  
- Altitude tape (right)  
- VSI tape (right slim)  
- Heading tape (bottom)  
- Aircraft symbol  
- Real-time smooth animation (filtering included)

---
    +-----------------------------+
    |       MPU6050 (IMU)         |
    |  Gyro + Accelerometer Data  |
    +-------------+---------------+
                  |
    +-------------v---------------+
    |        ADXL345 (Backup)     |
    |   Secondary Attitude Source |
    +-------------+---------------+
                  |
    +-------------v---------------+
    |         ESP32 Dev Kit       |
    |  - Sensor Fusion (Roll/Pitch)
    |  - Gyro-based Heading
    |  - BMP280 Altitude/VSI
    |  - Failover Logic (ADXL)
    |  - CSV Serial Stream
    +-------------+---------------+
                  |
          USB Serial @115200
                  |
    +-------------v---------------+
    |       Processing 4.x        |
    |  Boeing-Style PFD Renderer  |
    |  Graphics + FD Bars + Tapes |
    +-----------------------------+

---

## ✅ Features

### ESP32 Firmware
- Complementary filter for roll/pitch  
- Gyro-Z integration for heading  
- ADXL345-based failover  
- BMP280 altitude + VSI  
- Optional IAS field  
- OLED debug support  
- Drift suppression + NaN protections  

### Processing PFD
- Boeing-style horizon & pitch ladder  
- Bank scale with precise ticks  
- FD guidance bars  
- Airspeed/Altitude/VSI/Heading tapes  
- Digital readouts  
- 1600×900 optimized layout    

---

## 📂 Repository Structure
Real-Time-Avionics-PFD-ESP32/
│
├── firmware/
│ └── pfd_esp32.ino
│
├── processing/
│ └── BoeingPFD.pde
│
├── schematics/
│ └── wiring_diagram.png
│
├── media/
│ ├── screenshots/
│ │ └── pfd_running.png
│ └── demo/
│ └── pfd_demo.mp4
│
└── README.md

---

## 🔧 Hardware Used

| Component | Purpose |
|----------|---------|
| ESP32 (38-pin) | Main controller + sensor fusion |
| MPU6050 | Primary accelerometer + gyro |
| ADXL345 | Backup accelerometer (failover) |
| BMP280 | Altitude + pressure |
| OLED 128×64 | Optional debug display |
| USB Cable | Serial link to Processing |

---

## 🛠️ Software Requirements

- Arduino IDE 2.x  
- Processing 4.x (Java Mode)  
- Adafruit Sensor Libraries  
- ESP32 Board Support Package  

---

## 🚀 How to Run

### 1. Flash ESP32
Upload:
firmware/pfd_esp32.ino

ESP32 outputs clean CSV lines:

roll,pitch,alt_m,vsi_mps,hdg_deg[,ias_knots]

### 2. Run Processing PFD
Open:

processing/BoeingPFD.pde

Select correct COM port:

java
int PORT_INDEX = 0;
Run the sketch → PFD appears & updates in real time.


media/screenshots/pfd_running.png
media/demo/pfd_demo.mp4

---

🧩 Future Enhancements

Synthetic Vision Terrain (SVS)

Navigation Display (ND)

Modern G1000-style UI

GPS integration (track & groundspeed)

Black-box flight data recorder

Engine/system pages

Pitot-based IAS simulation

---

📁 License

MIT License — free to use for education and research.

👨‍💻 Author

Vinay Sharma
Embedded Systems • Avionics Simulation • ECE

