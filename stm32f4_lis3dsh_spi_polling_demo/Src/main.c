/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Nikhil Sagar Dadem
 * @brief          : Entry point for STM32F407VGT + LIS3DSH polling demo
 ******************************************************************************
 * @attention
 * This project demonstrates basic SPI-based bring-up of the LIS3DSH
 * accelerometer on the STM32F4DISCOVERY board. The code enables the
 * Cortex-M4 FPU, then runs a continuous demo loop defined in
 * lis3dsh_spi_polling_demo.c.
 *
 * Board: STM32F4DISCOVERY (STM32F407VGT MCU):contentReference[oaicite:0]{index=0}
 * Sensor: LIS3DSH MEMS 3-axis accelerometer (16-bit, ±2g/±4g/±6g/±8g/±16g):contentReference[oaicite:1]{index=1}
 *
 * Copyright (c) 2025 Nikhil Sagar Dadem.
 * Provided AS-IS for educational and demonstration purposes.
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include "STM32f407VGT.h"               // Bare-metal register & peripheral defs
#include "lis3dsh_spi_polling_demo.h"   // Demo application function prototype

/* ==========================================================================
 *                           FPU INITIALIZATION
 * ==========================================================================
 */

/**
 * @brief Enable hardware FPU (CP10/CP11).
 *
 * By default, the STM32F407 boots with the FPU disabled. Any attempt to
 * execute floating-point instructions without enabling it will cause a
 * UsageFault:contentReference[oaicite:2]{index=2}.
 *
 * Writing 0b11 to CP10 and CP11 fields in SCB->CPACR (bits 20–23) grants
 * full access to the FPU for both privileged and unprivileged code.
 */
static inline void FPU_Enable(void) {
    SCB_CPACR |= (0xF << 20);   // CP10=11 (full), CP11=11 (full)
    __asm volatile ("dsb");     // Data Synchronization Barrier
    __asm volatile ("isb");     // Instruction Synchronization Barrier
}

/* ==========================================================================
 *                               MAIN PROGRAM
 * ==========================================================================
 */

int main(void) {
    /* ---- Core bring-up ---- */
    FPU_Enable();   // Enable floating-point unit before any FP math is used

    /* ---- Application loop ----
     * The LIS3DSH demo runs inside its own while(1) loop, handling:
     *   - SPI communication with LIS3DSH
     *   - Moving average + LPF filtering
     *   - Tilt angle calculation (roll/pitch from accelerometer data)
     *   - LED direction indicators on the Discovery board
     */
    while (1) {
        lis3dsh_spi_polling_demo_main();
    }

    // Not reached
    return 0;
}
