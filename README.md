# STM32F407VGT + LIS3DSH SPI Polling Demo

[![YouTube Demo](https://img.shields.io/badge/YouTube-Demo-red?logo=youtube)](https://youtu.be/sTjlXTumpVE)  
[![GitHub Repo](https://img.shields.io/badge/GitHub-Repo-black?logo=github)](https://github.com/nikhilsagar/stm32f407-lis3dsh-demo.git)  
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

This project demonstrates using the **STM32F4DISCOVERY board** (STM32F407VGT MCU) with its onboard **LIS3DSH 3-axis accelerometer** via **SPI (polling mode)**.  
The firmware is written in **bare-metal C**, using custom drivers (GPIO, SPI, LIS3DSH) to avoid HAL overhead and keep the code transparent for learning and professional development.

---

## ✨ Features
- Custom bare-metal GPIO and SPI drivers
- LIS3DSH accelerometer driver
- **Signal processing pipeline**:
  - Moving Average (MA) filter – reduces random noise spikes
  - First-order Low-Pass Filter (LPF) – smooths motion data
- Real-time tilt angle calculation (roll/pitch) in degrees
- **LED orientation indicator** (see below)
- Cortex-M4 **FPU enabled** for fast floating-point math

---

## 🎥 Demo
📺 Watch the live demo here: [YouTube Link](https://youtu.be/sTjlXTumpVE)

---

## ⚡ Hardware Setup
- **Board:** STM32F4DISCOVERY (STM32F407VGT MCU):contentReference[oaicite:0]{index=0}
- **Accelerometer:** LIS3DSH MEMS (onboard sensor):contentReference[oaicite:1]{index=1}
- **Interface:** SPI1 (Mode 3: CPOL=1, CPHA=1)
- **GPIO LEDs:** Onboard Discovery LEDs (PD12–PD15):contentReference[oaicite:2]{index=2}

| LED Color | Pin  | Meaning (Tilt) |
|-----------|------|----------------|
| Green     | PD12 | **LEFT**  (X < –15°) |
| Red       | PD14 | **RIGHT** (X > +15°) |
| Orange    | PD13 | **UP**    (Y > +15°) |
| Blue      | PD15 | **DOWN**  (Y < –15°) |

---

## 🧠 How It Works
1. Raw accelerometer data is read from LIS3DSH over SPI.
2. A **Moving Average filter** smooths short-term spikes.
3. A **Low-Pass Filter** provides stable orientation estimation.
4. Tilt angles (roll, pitch) are computed from filtered acceleration.
5. **LED indicators**:
   - Tilt board right → **Red LED glows**  
   - Tilt left → **Green LED glows**  
   - Tilt forward (up) → **Orange LED glows**  
   - Tilt backward (down) → **Blue LED glows**

This makes the Discovery board act as a **tilt sensor with visual LED feedback**.

---

## 📂 Repository Structure
