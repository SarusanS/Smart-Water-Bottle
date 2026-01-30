# Project Highlight | Product Design – Smart Water Bottle

This project presents a **standalone Smart Water Bottle** developed as part of a **Product Design course**, focused on accurate, motion-aware water consumption monitoring under real-world usage conditions such as **movement, refilling, and power interruptions**.

The system is **fully embedded** and operates independently **without requiring a mobile application**, prioritizing **reliability, usability, and data integrity**.

---

## System Functionality

Water consumption is monitored by tracking changes in liquid level using a **VL53L0X Time-of-Flight (ToF) distance sensor**. Instead of relying on the absolute bottle capacity, the system calculates consumed volume based on **successive stable distance measurements**, allowing cumulative tracking across multiple refill cycles.

Even if the physical bottle capacity is limited (e.g., **1 litre**), users can configure daily intake goals of **5 L, 10 L, or 15 L**. Each refill is automatically detected as an **increase in water level**, while subsequent decreases are accumulated to compute **total consumption over time**.  
This approach enables **continuous and scalable intake monitoring**, independent of bottle size.

---

## Motion-Aware Validation

To ensure measurement accuracy, the system integrates a **BMI270 Inertial Measurement Unit (IMU)** combined with a **Madgwick sensor fusion filter**.

- Distance measurements are accepted only when the bottle is **upright and stable**
- **Tilt, motion, shaking, lifting, and refilling events** are detected and filtered out
- A **stability verification delay** ensures readings are taken only after motion has fully stopped

This motion-aware logic prevents **false volume calculations** and significantly improves measurement reliability during **daily use**.

---

## Floating Reflective Marker for Accurate Measurement

To overcome the challenges of measuring water level in transparent containers, a **floating reflective marker** is used:

- A small **floating rubber disc** is placed directly on the water surface
- The ToF sensor reflects off the **rubber surface instead of the water itself**
- This avoids errors caused by **ripples, bubbles, refraction, or transparent bottle walls**

As the marker naturally follows the water surface, the system achieves **consistent and repeatable distance measurements**, which are converted into volume through **firmware-level calculations**.

---

## Operating Modes

### Mode 1 – Goal-Based Tracking
- User-configurable **daily intake goal**
- Tracks **daily water consumption**
- Automatically **resets every 24 hours**
- Stores **previous day’s consumption** for reference

### Mode 2 – Continuous Tracking
- No predefined intake goal
- Tracks **total consumption continuously**
- **Manual reset** via touch-based confirmation
- Suitable for **long-term or unrestricted monitoring**

---

## User Interface and Interaction

- Touch-based input supports **display wake-up, goal setting, mode switching, and reset confirmation**
- **OLED display** presents real-time information including:
  - Water consumption
  - Daily goal status
  - Operating mode
  - Battery voltage
  - Time and date
- **Automatic display sleep** minimizes power consumption

---

## Touch Gesture Instructions

The system uses a **capacitive touch interface** to provide intuitive, button-less user interaction. Different tap patterns are interpreted by firmware to trigger specific actions.

| Gesture | Action |
|-------|--------|
| Single Tap | Increment value during goal or setting configuration |
| Double Tap | Wake the display or confirm a selection |
| Triple Tap | Switch between operating modes (Goal-Based / Continuous) |
| Long Inactivity | Automatically puts the display into sleep mode |

Touch input is **debounced and time-window validated** in firmware to prevent false triggers and ensure reliable gesture detection.

---

## Power and Battery Management

- Battery voltage is monitored in real time using the **ESP32 ADC**
- Status is displayed directly on the **OLED interface**
- Safe charging is handled externally via a **dedicated battery charging circuit**
- **Persistent storage** ensures data integrity across power cycles, supporting portable operation

---

## Hardware and PCB Design

- Custom **SMD-based PCB** designed for compact, reliable, and production-ready assembly
- Optimized component placement for:
  - Distance sensor
  - IMU
  - OLED display
  - Power management circuitry
- Designed to support **robust, portable, and scalable product deployment**

---

## Data Integrity and Reliability

- All critical parameters and usage data are stored in **non-volatile flash memory (ESP32 NVS)**
- Seamless recovery after **unexpected power loss**
- **Noise-filtered distance averaging and outlier rejection** improve measurement accuracy

---

## Technology Stack

- **Microcontroller:** ESP32  
- **Sensors:** VL53L0X ToF Sensor, BMI270 IMU  
- **Sensor Fusion:** Madgwick Filter  
- **Display:** SSD1306 OLED  
- **Input:** Touch Interface  
- **Storage:** ESP32 Non-Volatile Storage (NVS)  
- **Hardware:** Custom SMD PCB Design  

---

## Learning Outcomes

This project significantly strengthened my skills in:

- Embedded firmware development  
- Sensor fusion and motion-aware measurement  
- Robust signal processing for real-world conditions  
- Power-aware system design  
- SMD PCB design and hardware integration  
- User-centric embedded product development  
