# Code Structure

## `main.c`
- **FPU_Enable()**: Enables CP10/CP11 (hardware FPU) using `SCB_CPACR`.
- **main()**:
  - Calls `FPU_Enable()` very early (before any float executes).
  - Enters `while(1)`; calls `lis3dsh_spi_polling_demo_main()`.

### Why enable FPU?
- The build uses hard-float (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`).
- Without enabling CP10/CP11, first FP instruction triggers a **UsageFault**.

---

## `lis3dsh_spi_polling_demo.c`

### Bring-up
- **Init_LEDs()**
  - GPIOD PD12–PD15 outputs (GREEN/ORANGE/RED/BLUE).
- **Init_SPI1_GPIO_CS()**
  - PA5=SCK, PA6=MISO, PA7=MOSI (AF5).
  - PE3 as manual **chip-select** (idle HIGH).
- **Config_SPI1()**
  - Master, 8-bit, **Mode 0** (CPOL=0, CPHA=1) to start.
  - Slow prescaler first; can switch to **Mode 3** if reads look wrong.

### LIS3DSH init
- `LIS3DSH_Init(...)`:
  - Reads WHO_AM_I, configures axes, BDU, ODR/BW/FS.
  - Returns error → blink RED LED forever.

### Data path (loop)
1. **Poll data-ready** (timeout guard).
2. **Burst read** 16-bit XYZ.
3. **Moving Average** (integer domain, window N=8).
4. **Low-Pass Filter** (1st-order IIR with α from `fc` and `fs`).
5. **Convert** LSB→g (FS=±2g → ~60 µg/LSB).
6. **Clamp** to [-1, +1] for `asin()`.
7. **Angles** (axis-only tilt, degrees).
8. **LED logic**:
   - X > +15° → RED (right), X < −15° → GREEN (left)
   - Y > +15° → ORANGE (up), Y < −15° → BLUE (down)

### Timing
- ODR = 100 Hz, LPF cutoff `fc = 3 Hz`, sample rate `fs = 100 Hz`.

---

## Drivers
- **gpio_driver**: mode, type, pull, speed, write helpers.
- **spi_driver**: blocking TX/RX, master config, SSM/SSI handling.
- **lis3dsh**: WHO_AM_I, config registers, burst XYZ read, status DRDY.

---

## Typical control flow
