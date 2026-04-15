# 🔥FLUXION — Smart Reflow Oven (ESP32)

FLUXION is a low-cost smart reflow oven built using an ESP32 and a modified toaster oven.  
It implements closed-loop PID control with a rule-based adaptive system and a real-time web dashboard.

---

## Features

- Closed-loop PID temperature control
- Rule-based adaptive correction (non-ML)
- Real-time temperature monitoring (Web Serial)
- Interactive web dashboard (Chart.js)
- Multiple reflow profiles:
  - SAC305 (Lead-Free)
  - Sn63Pb37 (Leaded)
  - Low-temp (Bismuth)
  - Custom profile
- CSV export of run data

---

## Tech Stack

- ESP32 (Arduino framework)
- HTML, CSS, JavaScript
- Web Serial API

---

## Hardware

- ESP32
- Solid State Relay (SSR)
- K-type thermocouple (MAX31855)
- Toaster oven (modified)

---

## Project Structure

FLUXION/
├── dashboard.html
├── reflow_oven.ino
└── README.md

---

## How It Works

1. Temperature is measured using a thermocouple  
2. ESP32 runs a PID control loop  
3. Adaptive rules adjust heating behavior in real time  
4. Data is sent via serial  
5. Dashboard displays live graph and system stats  

---

## Usage

### 1. Upload Firmware
- Open `reflow_oven.ino` in Arduino IDE
- Select ESP32 board
- Upload code

### 2. Run Dashboard
- Open `FLUXION.html` in Chrome/Edge
- Click **CONNECT**
- Select ESP32 serial port

---

## Output

- Real-time temperature vs time graph
- PID parameters and SSR output
- Phase tracking (Preheat → Soak → Peak → Cooling)
- Exportable CSV logs

---

## Future Improvements

- Active cooling (fan control)
- WiFi-based monitoring
- Mobile app interface

---

## Author

Rishu Raj  
Electrical Engineering
