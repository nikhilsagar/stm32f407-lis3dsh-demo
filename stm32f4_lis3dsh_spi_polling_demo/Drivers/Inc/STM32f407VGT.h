/*
 * STM32f407VGT.h
 *
 *  Created on: Jul 25, 2025
 *      Author: Nikhil Sagar
 */

#ifndef INC_STM32F407VGT_H_
#define INC_STM32F407VGT_H_

#include "stdint.h"

// ========================== System and Core Interrupts ==========================

#define WWDG_IRQ_NUM               0   /*!< Window Watchdog interrupt */
#define PVD_IRQ_NUM                1   /*!< PVD through EXTI Line detection */
#define TAMP_STAMP_IRQ_NUM         2   /*!< Tamper and TimeStamp interrupts */
#define RTC_WKUP_IRQ_NUM           3   /*!< RTC Wakeup interrupt */
#define FLASH_IRQ_NUM              4   /*!< Flash global interrupt */
#define RCC_IRQ_NUM                5   /*!< RCC global interrupt */

// ============================== External Interrupts ==============================

#define EXTI0_IRQ_NUM              6   /*!< EXTI Line0 interrupt */
#define EXTI1_IRQ_NUM              7   /*!< EXTI Line1 interrupt */
#define EXTI2_IRQ_NUM              8   /*!< EXTI Line2 interrupt */
#define EXTI3_IRQ_NUM              9   /*!< EXTI Line3 interrupt */
#define EXTI4_IRQ_NUM             10   /*!< EXTI Line4 interrupt */
#define EXTI9_5_IRQ_NUM           23   /*!< EXTI Line[9:5] interrupts */
#define EXTI15_10_IRQ_NUM         40   /*!< EXTI Line[15:10] interrupts */

// ================================ DMA Interrupts =================================

#define DMA1_Stream0_IRQ_NUM      11
#define DMA1_Stream1_IRQ_NUM      12
#define DMA1_Stream2_IRQ_NUM      13
#define DMA1_Stream3_IRQ_NUM      14
#define DMA1_Stream4_IRQ_NUM      15
#define DMA1_Stream5_IRQ_NUM      16
#define DMA1_Stream6_IRQ_NUM      17
#define DMA1_Stream7_IRQ_NUM      47

#define DMA2_Stream0_IRQ_NUM      56
#define DMA2_Stream1_IRQ_NUM      57
#define DMA2_Stream2_IRQ_NUM      58
#define DMA2_Stream3_IRQ_NUM      59
#define DMA2_Stream4_IRQ_NUM      60
#define DMA2_Stream5_IRQ_NUM      68
#define DMA2_Stream6_IRQ_NUM      69
#define DMA2_Stream7_IRQ_NUM      70

// ================================ ADC / DAC / RNG ================================

#define ADC_IRQ_NUM               18   /*!< ADC1, ADC2, ADC3 global interrupt */
#define TIM6_DAC_IRQ_NUM          54   /*!< TIM6 and DAC underrun interrupt */
#define HASH_RNG_IRQ_NUM          80   /*!< HASH and RNG global interrupt */

// ================================ CAN Bus Interrupts =============================

#define CAN1_TX_IRQ_NUM           19
#define CAN1_RX0_IRQ_NUM          20
#define CAN1_RX1_IRQ_NUM          21
#define CAN1_SCE_IRQ_NUM          22

#define CAN2_TX_IRQ_NUM           63
#define CAN2_RX0_IRQ_NUM          64
#define CAN2_RX1_IRQ_NUM          65
#define CAN2_SCE_IRQ_NUM          66

// =============================== Timer Interrupts ================================

#define TIM1_BRK_TIM9_IRQ_NUM     24
#define TIM1_UP_TIM10_IRQ_NUM     25
#define TIM1_TRG_COM_TIM11_IRQ_NUM 26
#define TIM1_CC_IRQ_NUM           27

#define TIM2_IRQ_NUM              28
#define TIM3_IRQ_NUM              29
#define TIM4_IRQ_NUM              30
#define TIM5_IRQ_NUM              50

#define TIM8_BRK_TIM12_IRQ_NUM    43
#define TIM8_UP_TIM13_IRQ_NUM     44
#define TIM8_TRG_COM_TIM14_IRQ_NUM 45
#define TIM8_CC_IRQ_NUM           46

#define TIM7_IRQ_NUM              55

// =========================== SPI / I2C / USART / UART ============================

#define SPI1_IRQ_NUM              35
#define SPI2_IRQ_NUM              36
#define SPI3_IRQ_NUM              51

#define I2C1_EV_IRQ_NUM           31
#define I2C1_ER_IRQ_NUM           32
#define I2C2_EV_IRQ_NUM           33
#define I2C2_ER_IRQ_NUM           34
#define I2C3_EV_IRQ_NUM           72
#define I2C3_ER_IRQ_NUM           73

#define USART1_IRQ_NUM           37
#define USART2_IRQ_NUM           38
#define USART3_IRQ_NUM           39
#define USART6_IRQ_NUM           71
#define UART4_IRQ_NUM            52
#define UART5_IRQ_NUM            53

// ============================== USB / OTG / SDIO ===============================

#define SDIO_IRQ_NUM              49   /*!< SDIO global interrupt */
#define OTG_FS_IRQ_NUM            67   /*!< USB OTG FS global interrupt */
#define OTG_FS_WKUP_IRQ_NUM       42   /*!< USB OTG FS Wakeup */
#define OTG_HS_EP1_OUT_IRQ_NUM    74
#define OTG_HS_EP1_IN_IRQ_NUM     75
#define OTG_HS_WKUP_IRQ_NUM       76
#define OTG_HS_IRQ_NUM            77

// =============================== Ethernet / DCMI ===============================

#define ETH_IRQ_NUM               61   /*!< Ethernet global interrupt */
#define ETH_WKUP_IRQ_NUM          62   /*!< Ethernet Wakeup interrupt */
#define DCMI_IRQ_NUM              78   /*!< Digital Camera Interface interrupt */

// =============================== Crypto / FPU ================================

#define CRYP_IRQ_NUM              79   /*!< Cryptographic processor interrupt */
#define FPU_IRQ_NUM               81   /*!< Floating Point Unit interrupt */

// ================================================================================

#define __vo volatile
#define HIGH 1
#define LOW 0


// ====================== CORE MEMORY REGIONS =========================
#define BOOT_ALIAS_BASE         0x00000000U   // Aliased to Flash/System/SRAM based on BOOT config
#define FLASH_BASE              0x08000000U   // Main program memory (1MB max)
#define CCMRAM_BASE             0x10000000U   // Core Coupled Memory (64KB, CPU-only)
#define SYSTEM_MEMORY_BASE      0x1FFF0000U   // System bootloader ROM
#define OPTION_BYTES_BASE       0x1FFFC000U   // Option bytes area (16 bytes)
#define SRAM1_BASE              0x20000000U   // SRAM1 (112KB)
#define SRAM2_BASE              0x2001C000U   // SRAM2 (16KB)

// ====================== PERIPHERAL REGIONS ==========================

// Base address for all peripheral regions (starts at 0x40000000)
#define PERIPH_BASE             0x40000000U

// Base address for APB1 (Advanced Peripheral Bus 1)
// Includes: TIM2–TIM7, USART2–5, I2C1–3, SPI2/3, CAN1/2, etc.
#define APB1PERIPH_BASE         PERIPH_BASE  // 0x40000000

// Base address for APB2 (Advanced Peripheral Bus 2)
// Includes: TIM1, USART1/6, ADC1–3, SPI1, SYSCFG, EXTI, etc.
#define APB2PERIPH_BASE         0x40010000U  // 64 KB above APB1

// Base address for AHB1 (Advanced High-performance Bus 1)
// Includes: GPIOA–GPIOI, CRC, RCC, Flash interface, DMA1/2, etc.
#define AHB1PERIPH_BASE         0x40020000U  // 128 KB above APB1

// Base address for AHB2 (Advanced High-performance Bus 2)
// Limited usage in STM32F407 (e.g., USB OTG FS)
#define AHB2PERIPH_BASE         0x50000000U

// Base address for AHB3 (Advanced High-performance Bus 3)
// Used for external memory interface (FSMC) in STM32F407
#define AHB3PERIPH_BASE         0x60000000U

// ====================== APB1 PERIPHERALS ===========================
// Base: 0x40000000 – APB1 max frequency: 42 MHz

#define TIM2_BASE               0x40000000U  // General-purpose timer 2 (32-bit)
#define TIM3_BASE               0x40000400U  // General-purpose timer 3 (16-bit)
#define TIM4_BASE               0x40000800U  // General-purpose timer 4 (16-bit)
#define TIM5_BASE               0x40000C00U  // General-purpose timer 5 (32-bit)
#define TIM6_BASE               0x40001000U  // Basic timer 6
#define TIM7_BASE               0x40001400U  // Basic timer 7
#define TIM12_BASE              0x40001800U  // General-purpose timer 12 (16-bit)
#define TIM13_BASE              0x40001C00U  // General-purpose timer 13 (16-bit)
#define TIM14_BASE              0x40002000U  // General-purpose timer 14 (16-bit)

#define RTC_BKP_BASE            0x40002800U  // RTC and Backup Registers
#define WWDG_BASE               0x40002C00U  // Window watchdog
#define IWDG_BASE               0x40003000U  // Independent watchdog

#define I2S2ext_BASE            0x40003400U  // Extended I2S2 interface
#define SPI2_BASE               0x40003800U  // SPI2 / I2S2
#define SPI3_BASE               0x40003C00U  // SPI3 / I2S3
#define I2S3ext_BASE            0x40004000U  // Extended I2S3 interface

#define USART2_BASE             0x40004400U  // USART2
#define USART3_BASE             0x40004800U  // USART3
#define UART4_BASE              0x40004C00U  // UART4
#define UART5_BASE              0x40005000U  // UART5

#define I2C1_BASE               0x40005400U  // I2C1
#define I2C2_BASE               0x40005800U  // I2C2
#define I2C3_BASE               0x40005C00U  // I2C3

#define CAN1_BASE               0x40006400U  // CAN1
#define CAN2_BASE               0x40006800U  // CAN2

#define PWR_BASE                0x40007000U  // Power control interface
#define DAC_BASE                0x40007400U  // Digital-to-Analog Converter
// ====================== APB1 PERIPHERALS END ===========================

// ====================== APB2 PERIPHERALS ===========================
// Base: 0x40010000 – APB2 max frequency: 84 MHz
// STM32F407VGT6 on STM32F4DISCOVERY board

#define TIM1_BASE               0x40010000U  // Advanced-control Timer 1 (PWM, input capture, etc.)
#define TIM8_BASE               0x40010400U  // Advanced-control Timer 8

#define USART1_BASE             0x40011000U  // USART1 (High-speed serial communication)
#define USART6_BASE             0x40011400U  // USART6

#define ADC_BASE                0x40012000U  // Base address for ADC block
#define ADC1_BASE               (ADC_BASE + 0x000U)  // Analog-to-Digital Converter 1
#define ADC2_BASE               (ADC_BASE + 0x100U)  // ADC2
#define ADC3_BASE               (ADC_BASE + 0x200U)  // ADC3
#define ADC_COMMON_BASE_REG     (ADC_BASE + 0x300U)  // ADC Common control and status registers

#define SDIO_BASE               0x40012C00U  // SDIO (Secure Digital Input/Output for SD cards)

#define SPI1_BASE               0x40013000U  // SPI1 (Serial Peripheral Interface)
#define SPI4_BASE               0x40013400U  // SPI4 (on larger packages)

#define SYSCFG_BASE             0x40013800U  // System Configuration Controller
#define EXTI_BASE               0x40013C00U  // External Interrupt/Event Controller

#define TIM9_BASE               0x40014000U  // General-purpose Timer 9
#define TIM10_BASE              0x40014400U  // Timer 10
#define TIM11_BASE              0x40014800U  // Timer 11

#define SPI5_BASE               0x40015000U  // SPI5 (on larger packages)
#define SPI6_BASE               0x40015400U  // SPI6 (on larger packages)

// ====================== APB2 PERIPHERALS END ===========================

// ====================== AHB1 PERIPHERALS ===========================
// Base: 0x40020000 – AHB1 max frequency: 168 MHz
// STM32F407VGT6 on STM32F4DISCOVERY board

#define GPIOA_BASE              0x40020000U  // General-purpose I/O port A
#define GPIOB_BASE              0x40020400U  // GPIO port B
#define GPIOC_BASE              0x40020800U  // GPIO port C
#define GPIOD_BASE              0x40020C00U  // GPIO port D
#define GPIOE_BASE              0x40021000U  // GPIO port E
//NO GPIO F and GPIO G
#define GPIOH_BASE              0x40021C00U  // GPIO port H

#define CRC_BASE                0x40023000U  // CRC calculation unit
#define RCC_BASE                0x40023800U  // Reset and Clock Control (RCC)

#define FLASH_INTERFACE_BASE    0x40023C00U  // Flash interface registers
#define BKPSRAM_BASE            0x40024000U  // Backup SRAM (4KB, powered by VBAT)

#define DMA1_BASE               0x40026000U  // DMA1 controller
#define DMA2_BASE               0x40026400U  // DMA2 controller

#define ETH_BASE                0x40028000U  // Base address of Ethernet peripheral block
#define ETH_MAC_BASE            (ETH_BASE + 0x000U)  // Ethernet MAC control/status registers
#define ETH_MMC_BASE            (ETH_BASE + 0x100U)  // Ethernet MMC (Media Management Counters)
#define ETH_PTP_BASE            (ETH_BASE + 0x700U)  // Ethernet PTP (Precision Time Protocol)
#define ETH_DMA_BASE            (ETH_BASE + 0x1000U) // Ethernet DMA control/status registers

#define OTG_HS_BASE             0x40040000U  // Base address for USB OTG High-Speed core
#define OTG_HS_HOST_BASE        (OTG_HS_BASE + 0x0200U)  // Host-mode registers
#define OTG_HS_DEVICE_BASE      (OTG_HS_BASE + 0x0800U)  // Device-mode registers
#define OTG_HS_PWRCLK_BASE      (OTG_HS_BASE + 0x0E00U)  // Power and clock gating registers
// ====================== AHB1 PERIPHERALS END ===========================

// ====================== AHB2 PERIPHERALS ===========================
// Base: 0x50000000 – AHB2 bus
// STM32F407VGT6 includes limited AHB2 peripherals (e.g., USB OTG FS)

#define AHB2PERIPH_BASE         0x50000000U  // AHB2 peripheral base address

#define OTG_FS_BASE             (AHB2PERIPH_BASE + 0x0000U)  // USB OTG Full-Speed core base address
#define OTG_FS_GLOBAL_BASE      OTG_FS_BASE                 // Alias for global registers
#define OTG_FS_HOST_BASE        (OTG_FS_BASE + 0x0400U)     // USB OTG FS host-mode registers
#define OTG_FS_DEVICE_BASE      (OTG_FS_BASE + 0x0800U)     // USB OTG FS device-mode registers
#define OTG_FS_PWRCLK_BASE      (OTG_FS_BASE + 0x0E00U)     // USB OTG FS power and clock gating registers
// ====================== AHB2 PERIPHERALS END ===========================

// ====================== AHB3 PERIPHERALS ===========================
// Base: 0x60000000 – AHB3 bus
// STM32F407 includes only one AHB3 peripheral: FSMC

#define AHB3PERIPH_BASE         0x60000000U  // AHB3 peripheral base
#define FSMC_R_BASE             (AHB3PERIPH_BASE + 0x00000000U)  // Flexible Static Memory Controller (FSMC)
// ====================== AHB3 PERIPHERALS END===========================

// ====================== RCC REGISTER STRUCTURE ===========================
typedef struct {
	__vo uint32_t CR;            // 0x00: Clock control register
	__vo uint32_t PLLCFGR;       // 0x04: PLL configuration register
	__vo uint32_t CFGR;          // 0x08: Clock configuration register
	__vo uint32_t CIR;           // 0x0C: Clock interrupt register
	__vo uint32_t AHB1RSTR;      // 0x10: AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;      // 0x14: AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;      // 0x18: AHB3 peripheral reset register
	uint32_t RESERVED0;     // 0x1C: Reserved
	__vo uint32_t APB1RSTR;      // 0x20: APB1 peripheral reset register
	__vo uint32_t APB2RSTR;      // 0x24: APB2 peripheral reset register
	uint32_t RESERVED1[2];  // 0x28–0x2C: Reserved
	__vo uint32_t AHB1ENR;       // 0x30: AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;       // 0x34: AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;       // 0x38: AHB3 peripheral clock enable register
	uint32_t RESERVED2;     // 0x3C: Reserved
	__vo uint32_t APB1ENR;       // 0x40: APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;       // 0x44: APB2 peripheral clock enable register
	uint32_t RESERVED3[2];  // 0x48–0x4C: Reserved
	__vo uint32_t AHB1LPENR; // 0x50: AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR; // 0x54: AHB2 peripheral clock enable in low power mode register
	__vo uint32_t AHB3LPENR; // 0x58: AHB3 peripheral clock enable in low power mode register
	uint32_t RESERVED4;     // 0x5C: Reserved
	__vo uint32_t APB1LPENR; // 0x60: APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR; // 0x64: APB2 peripheral clock enable in low power mode register
	uint32_t RESERVED5[2];  // 0x68–0x6C: Reserved
	__vo uint32_t BDCR;          // 0x70: Backup domain control register
	__vo uint32_t CSR;           // 0x74: Control/status register
	uint32_t RESERVED6[2];  // 0x78–0x7C: Reserved
	__vo uint32_t SSCGR;      // 0x80: Spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;    // 0x84: PLLI2S configuration register
	__vo uint32_t PLLSAICFGR; // 0x88: PLLSAI configuration register (not used in STM32F407)
	__vo uint32_t DCKCFGR;      // 0x8C: Dedicated clocks configuration register
} RCC_RegDef_t;
// ====================== RCC REGISTER STRUCTURE END =======================
// ====================== RCC REGISTER DEFINITIONS =======================
#define RCC ((RCC_RegDef_t *) RCC_BASE)
// ====================== RCC REGISTER DEFINITIONS END=======================

// ====================== SYSCFG_BASE REGISTER STRUCTURE ===========================
typedef struct {
	__vo uint32_t MEMRMP; /*!< 0x00 */
	__vo uint32_t PMC; /*!< 0x04 */
	__vo uint32_t EXTICR[4]; /*!< 0x08 – 0x14: EXTICR1–4 combined as array */
	uint32_t RESERVED1[2]; /*!< 0x18–0x1C: Reserved */
	__vo uint32_t CMPCR; /*!< 0x20 */
} SYSCFG_RegDef_t;

// ====================== RCC REGISTER STRUCTURE END =======================

// ====================== SYSCFG REGISTER DEFINITIONS =======================
#define SYSCFG ((SYSCFG_RegDef_t *) SYSCFG_BASE) /*!< Pointer to SYSCFG register structure */
// ====================== SYSCFG REGISTER DEFINITIONS END=======================

// ====================== EXTI REGISTER STRUCTURE ===========================
typedef struct {
	__vo uint32_t IMR; /*!< Interrupt mask register: Mask/unmask EXTI lines          (Offset: 0x00) */
	__vo uint32_t EMR; /*!< Event mask register: Mask/unmask events (not interrupts) (Offset: 0x04) */
	__vo uint32_t RTSR; /*!< Rising trigger selection register                        (Offset: 0x08) */
	__vo uint32_t FTSR; /*!< Falling trigger selection register                       (Offset: 0x0C) */
	__vo uint32_t SWIER; /*!< Software interrupt event register                        (Offset: 0x10) */
	__vo uint32_t PR; /*!< Pending register: write 1 to clear                       (Offset: 0x14) */
} EXTI_RegDef_t;
// ====================== EXTI REGISTER STRUCTURE END =======================

// ====================== EXTI REGISTER DEFINITIONS =======================
#define EXTI ((EXTI_RegDef_t *) EXTI_BASE) /*!< Pointer to EXTI peripheral base address */

// ====================== EXTI REGISTER DEFINITIONS END=======================

// ====================== CORE SCS BASE ADDRESSES ===========================
// System Control Space (SCS) region: core peripherals (SysTick, NVIC, SCB, FPU)
#define SCS_BASE        (0xE000E000UL)
#define SCB_BASE        (SCS_BASE + 0x0D00UL)   /* 0xE000ED00: System Control Block */
#define NVIC_BASE       (SCS_BASE + 0x0100UL)   /* 0xE000E100: NVIC registers */
#define SYSTICK_BASE    (SCS_BASE + 0x0010UL)   /* 0xE000E010: SysTick timer */
#define FPU_REG_BASE    (SCS_BASE + 0x0F30UL)   /* 0xE000EF30: FPU control/status block */
// ====================== CORE SCS BASE ADDRESSES END =======================



// ====================== SCB REGISTER DEFINITIONS ==========================
// SCB: CPU/core control (vector table, reset/priority, sleep, faults, FPU access)
#define SCB_CPUID       (*(volatile uint32_t *)(SCB_BASE + 0x000))  /* CPU ID */
#define SCB_ICSR        (*(volatile uint32_t *)(SCB_BASE + 0x004))  /* Int Ctrl/State */
#define SCB_VTOR        (*(volatile uint32_t *)(SCB_BASE + 0x008))  /* Vector Table base */
#define SCB_AIRCR       (*(volatile uint32_t *)(SCB_BASE + 0x00C))  /* App Int/Reset Ctrl */
#define SCB_SCR         (*(volatile uint32_t *)(SCB_BASE + 0x010))  /* System Control (sleep) */
#define SCB_CCR         (*(volatile uint32_t *)(SCB_BASE + 0x014))  /* Config/Control */
#define SCB_SHCSR       (*(volatile uint32_t *)(SCB_BASE + 0x024))  /* Sys Handler Ctrl/State */
#define SCB_CFSR        (*(volatile uint32_t *)(SCB_BASE + 0x028))  /* Configurable Fault Status */
#define SCB_HFSR        (*(volatile uint32_t *)(SCB_BASE + 0x02C))  /* HardFault Status */
#define SCB_MMFAR       (*(volatile uint32_t *)(SCB_BASE + 0x034))  /* MemManage Fault Addr */
#define SCB_BFAR        (*(volatile uint32_t *)(SCB_BASE + 0x038))  /* BusFault Addr */
#define SCB_CPACR       (*(volatile uint32_t *)(SCB_BASE + 0x088))  /* Coprocessor Access (FPU) */
// ====================== SCB REGISTER DEFINITIONS END ======================



// ====================== FPU REGISTER DEFINITIONS ==========================
// FPU block: context/behavior (lazy stacking), not the enable switch.
// Enable FPU via SCB_CPACR (CP10/CP11) first; these tune behavior.
#define FPU_FPCCR       (*(volatile uint32_t *)(0xE000EF34UL))  /* FP Context Control (ASPEN/LSPEN) */
#define FPU_FPCAR       (*(volatile uint32_t *)(0xE000EF38UL))  /* FP Context Addr */
#define FPU_FPDSCR      (*(volatile uint32_t *)(0xE000EF3CUL))  /* Default FP Status/Control */
#define FPU_MVFR0       (*(volatile uint32_t *)(0xE000EF40UL))  /* Media/FP Feature 0 */
#define FPU_MVFR1       (*(volatile uint32_t *)(0xE000EF44UL))  /* Media/FP Feature 1 */
#define FPU_MVFR2       (*(volatile uint32_t *)(0xE000EF48UL))  /* Media/FP Feature 2 */
// ====================== FPU REGISTER DEFINITIONS END ======================



// ====================== NVIC REGISTER BLOCK BASES =========================
// NVIC: IRQ enable/disable, pending/active, priority, software trigger
#define NVIC_ISER_BASE  (NVIC_BASE + 0x000)    /* 0xE000E100: Set-Enable  */
#define NVIC_ICER_BASE  (NVIC_BASE + 0x080)    /* 0xE000E180: Clear-Enable */
#define NVIC_ISPR_BASE  (NVIC_BASE + 0x100)    /* 0xE000E200: Set-Pending */
#define NVIC_ICPR_BASE  (NVIC_BASE + 0x180)    /* 0xE000E280: Clear-Pending */
#define NVIC_IABR_BASE  (NVIC_BASE + 0x200)    /* 0xE000E300: Active Bit */
#define NVIC_IPR_BASE   (NVIC_BASE + 0x300)    /* 0xE000E400: Priority (byte-addressed) */
#define NVIC_STIR       (*(volatile uint32_t *)(0xE000EF00UL))  /* Software Trigger Interrupt */
// ====================== NVIC REGISTER BLOCK BASES END =====================



// ====================== NVIC POINTER ARRAYS ===============================
// Convenient typed views for indexed access (e.g., ISER[n], IPR[irq_num])
#define ISER  ((volatile uint32_t *)(NVIC_ISER_BASE))  /* Enable IRQs   */
#define ICER  ((volatile uint32_t *)(NVIC_ICER_BASE))  /* Disable IRQs  */
#define ISPR  ((volatile uint32_t *)(NVIC_ISPR_BASE))  /* Pend IRQs     */
#define ICPR  ((volatile uint32_t *)(NVIC_ICPR_BASE))  /* Unpend IRQs   ) */
#define IABR  ((volatile uint32_t *)(NVIC_IABR_BASE))  /* Active status */
#define IPR   ((volatile uint8_t  *)(NVIC_IPR_BASE))   /* Priority bytes (1B/IRQ) */
// ====================== NVIC POINTER ARRAYS END ===========================


#endif /* INC_STM32F407VGT_H_ */
