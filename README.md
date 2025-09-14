# STM32F407VGT + LIS3DSH SPI Polling Demo  

[![GitHub Repo](https://img.shields.io/badge/GitHub-Repo-black?logo=github)](https://github.com/nikhilsagar/stm32f407-lis3dsh-demo.git)  


---

## 🎥 Demo  
[![STM32F407 + LIS3DSH SPI Demo](https://img.youtube.com/vi/sTjlXTumpVE/0.jpg)](https://youtu.be/sTjlXTumpVE)  
*Click the thumbnail to watch the live demo on YouTube.*  

---

## ✨ Features  
- ⚡ Custom bare-metal **GPIO** and **SPI** drivers  
- 📡 **LIS3DSH accelerometer** driver  
- 🎚️ Signal Processing Pipeline:  
  - 🟢 **Moving Average Filter (MA)** – reduces noise spikes  
  - 🔵 **Low-Pass Filter (LPF)** – smooths motion data  
- 📐 Real-time tilt angle calculation (roll/pitch)  
- 💡 **LED orientation indicators** using onboard LEDs  
- 🧮 Cortex-M4 **FPU enabled** for fast floating-point math  

---

## 📖 Background  
The **STM32F4DISCOVERY** board includes a built-in **LIS3DSH 3-axis accelerometer**.  
This project demonstrates:  
- **Direct SPI communication** (polling mode, no HAL/LL overhead).  
- **Bare-metal driver development** for learning transparency & control.  
- **Embedded signal processing** for motion sensing applications.  

Polling mode was chosen for simplicity and clarity, making it easier to debug and understand low-level data transfers before moving on to **interrupts** or **DMA**.  

---

## ⚡ Hardware Setup  
- **Board:** STM32F4DISCOVERY (STM32F407VGT MCU)  
- **Accelerometer:** LIS3DSH MEMS (onboard sensor)  
- **Interface:** SPI1 (Mode 3: CPOL=1, CPHA=1)  
- **GPIO LEDs:** Onboard Discovery LEDs (PD12–PD15)  

| LED Color | Pin  | Meaning (Tilt) |
|-----------|------|----------------|
| 🟢 Green  | PD12 | **LEFT**  (X < –15°) |
| 🔴 Red    | PD14 | **RIGHT** (X > +15°) |
| 🟠 Orange | PD13 | **UP**    (Y > +15°) |
| 🔵 Blue   | PD15 | **DOWN**  (Y < –15°) |  

---

## 🧠 How It Works  
1. Raw accelerometer data is read via SPI (polling).  
2. Data passes through filters:  
   - **Moving Average filter** → smooths short-term spikes  
   - **Low-Pass Filter** → stabilizes long-term orientation  
3. Tilt angles (roll, pitch) are computed in **degrees**.  
4. Onboard LEDs indicate orientation in real-time.  

Effectively, the board acts as a **tilt sensor with visual LED feedback**.  

---

## 🧮 Filters & Signal Processing  
- **Moving Average (MA):**  
  Computes the average of the last *N* samples to reduce high-frequency noise.  
- **First-order Low-Pass Filter (LPF):**  
  Allows slow-changing signals (motion/orientation) while attenuating fast changes.  

📖 For detailed derivations, formulas, and design trade-offs, see the [`docs/`](./docs) folder.  

---

## 🛠️ Build & Run  
1. Clone the repository:  
   ```bash
   git clone https://github.com/nikhilsagar/stm32f407-lis3dsh-demo.git
   cd stm32f407-lis3dsh-demo
