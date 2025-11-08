# 🛫 Real-Time Avionics Primary Flight Display (PFD) Using ESP32 & Processing  
### Boeing-Style Glass Cockpit • Real-Time Sensor Fusion • Embedded Avionics System

This project implements a **real-time Primary Flight Display (PFD)** similar to those used in commercial aircraft (e.g., **Boeing 737/787**).  
It uses an **ESP32** to read flight-critical data from an IMU and barometric sensors, performs **sensor fusion**, and streams live data to a **Processing-based Boeing PFD**.

---

## ✈️ Overview

This system measures and displays:

- ✅ **Roll** (Bank angle)
- ✅ **Pitch** (Attitude)
- ✅ **Heading** (Gyro-based)
- ✅ **Altitude** (BMP280 barometric pressure)
- ✅ **VSI** – Vertical Speed Indicator (derived from altitude rate)
- ✅ **IAS** – Indicated Airspeed (optional 6th data field)
- ✅ **IMU Failover** using ADXL345

The Processing application renders a **full Boeing-style PFD** including:

- Artificial horizon  
- Pitch ladder  
- Bank-angle scale  
- Flight-director bars  
- IAS tape (left)  
- Altitude tape (right)  
- VSI scale  
- Heading tape (bottom)  
- Aircraft symbol  
- Smooth movement using filters  

---

## 🧠 System Architecture

