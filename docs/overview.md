# Overview

This project runs a **bare-metal SPI polling demo** on the STM32F407VGT (STM32F4DISCOVERY) with the onboard **LIS3DSH** accelerometer.

## What it does
- Brings up GPIO (LEDs), SPI1 (PA5/6/7), and manual CS (PE3).
- Initializes LIS3DSH (WHO_AM_I, axes enable, ODR/BW/FS).
- Reads XYZ data in a loop (polling, not interrupts).
- Applies a **Moving Average** and a **1st-order Low-Pass Filter**.
- Converts to g, computes simple **axis tilt** angles.
- Lights LEDs based on orientation, prints angles via `printf`.

## Why bare-metal?
- Transparent control of registers (good for learning and interviews).
- No HAL overhead; custom drivers show your embedded fundamentals.

## Key files
- `Src/main.c` — enables the **FPU** and runs the demo loop.
- `Src/lis3dsh_spi_polling_demo.c` — full application pipeline.
- `Drivers/Src/*.c` — custom GPIO, SPI, and LIS3DSH drivers.
