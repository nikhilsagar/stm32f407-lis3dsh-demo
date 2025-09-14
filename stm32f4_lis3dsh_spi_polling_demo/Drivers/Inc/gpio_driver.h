/**
 * @file    gpio_driver.h
 * @author  Nikhil Sagar
 * @brief   GPIO driver interface for STM32F407
 *
 * This header provides the interface and data structures for configuring and
 * using the GPIO peripheral on STM32F4 microcontrollers. It supports initialization,
 * pin read/write operations, pin toggling, alternate function setup, clock control,
 * and EXTI-based interrupt configuration.
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "STM32f407VGT.h"
#include <stdbool.h>

// ====================== GPIO REGISTER STRUCT ===========================
typedef struct {
    __vo uint32_t MODER;    /*!< 0x00: GPIO port mode register */
    __vo uint32_t OTYPER;   /*!< 0x04: GPIO port output type register */
    __vo uint32_t OSPEEDR;  /*!< 0x08: GPIO port output speed register */
    __vo uint32_t PUPDR;    /*!< 0x0C: GPIO port pull-up/pull-down register */
    __vo uint32_t IDR;      /*!< 0x10: GPIO port input data register */
    __vo uint32_t ODR;      /*!< 0x14: GPIO port output data register */
    __vo uint32_t BSRR;     /*!< 0x18: GPIO port bit set/reset register */
    __vo uint32_t LCKR;     /*!< 0x1C: GPIO port configuration lock register */
    __vo uint32_t AFR[2];   /*!< 0x20: AFR[0] = low (0-7), AFR[1] = high (8-15) */
} GPIO_RegDef_t;
// ====================== GPIO REGISTER STRUCT END ===========================

// ====================== GPIO REGISTER DEFINITIONS =======================
#define GPIOA   ((GPIO_RegDef_t *) GPIOA_BASE)
#define GPIOB   ((GPIO_RegDef_t *) GPIOB_BASE)
#define GPIOC   ((GPIO_RegDef_t *) GPIOC_BASE)
#define GPIOD   ((GPIO_RegDef_t *) GPIOD_BASE)
#define GPIOE   ((GPIO_RegDef_t *) GPIOE_BASE)
#define GPIOH   ((GPIO_RegDef_t *) GPIOH_BASE)
// ====================== GPIO REGISTER DEFINITIONS END=======================

// ========================== ENUMERATIONS ===============================

/**
 * @brief GPIO port index used for RCC clock control
 */
typedef enum {
    GPIO_PORT_A = 0U,
    GPIO_PORT_B = 1U,
    GPIO_PORT_C = 2U,
    GPIO_PORT_D = 3U,
    GPIO_PORT_E = 4U,
    GPIO_PORT_H = 7U
} gpio_port_t;

/**
 * @brief Enable or disable a peripheral
 */
typedef enum {
    ENABLE  = 1U,
    DISABLE = 0U
} en_state_t;

/**
 * @brief GPIO pin modes
 */
typedef enum {
    GPIO_MODE_INPUT     = 0x00U,
    GPIO_MODE_OUTPUT    = 0x01U,
    GPIO_MODE_ALT       = 0x02U,
    GPIO_MODE_ANALOG    = 0x03U
} GPIO_Mode_t;

/**
 * @brief EXTI edge trigger types
 */
typedef enum {
    GPIO_TRIGGER_RISING,
    GPIO_TRIGGER_FALLING,
    GPIO_TRIGGER_BOTH
} edge_t;

/**
 * @brief GPIO output types
 */
typedef enum {
    GPIO_OTYPE_PP = 0x00U,
    GPIO_OTYPE_OD = 0x01U
} GPIO_OType_t;

/**
 * @brief GPIO output speed levels
 */
typedef enum {
    GPIO_SPEED_LOW     = 0x00U,
    GPIO_SPEED_MEDIUM  = 0x01U,
    GPIO_SPEED_FAST    = 0x02U,
    GPIO_SPEED_HIGH    = 0x03U
} GPIO_Speed_t;

/**
 * @brief GPIO pull-up/pull-down settings
 */
typedef enum {
    GPIO_NOPULL   = 0x00U,
    GPIO_PULLUP   = 0x01U,
    GPIO_PULLDOWN = 0x02U
} GPIO_PuPd_t;
// ======================================================================

// ========================== CONFIG STRUCT =============================

/**
 * @brief GPIO pin configuration structure
 */
typedef struct {
    GPIO_Mode_t    Mode;         /*!< Pin mode */
    GPIO_OType_t   OutputType;   /*!< Output type (used in output/alt mode) */
    GPIO_PuPd_t    PuPd;         /*!< Pull-up/pull-down setting */
    uint8_t        AltFunction;  /*!< Alternate function number (0–15) */
} GPIO_Config_t;
// ======================================================================

// =========================== DRIVER API ===============================

/**
 * @brief Initializes a GPIO pin.
 * @param GPIOx       GPIO port base address
 * @param PinNumber   Pin number (0 to 15)
 * @param config      Pointer to configuration structure
 */
void GPIO_Init(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, GPIO_Config_t *config);

/**
 * @brief Writes a logic level to a GPIO pin.
 * @param GPIOx       GPIO port base address
 * @param PinNumber   Pin number (0 to 15)
 * @param value       Logic level (0 = LOW, 1 = HIGH)
 */
void GPIO_WritePin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, uint8_t value);

/**
 * @brief Writes a 16-bit value to the GPIO output register.
 * @param GPIOx       GPIO port base address
 * @param value       16-bit value (only lower 16 bits used)
 */
void GPIO_WritePort(GPIO_RegDef_t *GPIOx, uint16_t value);

/**
 * @brief Reads logic level from a GPIO input pin.
 * @param GPIOx       GPIO port base address
 * @param PinNumber   Pin number (0 to 15)
 * @return Logic level (0 or 1)
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);

/**
 * @brief Reads the entire GPIO input register.
 * @param GPIOx       GPIO port base address
 * @return 16-bit input value (IDR)
 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t *GPIOx);

/**
 * @brief Toggles a GPIO output pin.
 * @param GPIOx       GPIO port base address
 * @param PinNumber   Pin number (0 to 15)
 */
void GPIO_TogglePin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);

/**
 * @brief Enables or disables the AHB1 peripheral clock for a GPIO port.
 * @param Port        GPIO port index (GPIO_PORT_A to GPIO_PORT_H)
 * @param State       ENABLE or DISABLE
 */
void GPIO_EnableClock(gpio_port_t Port, en_state_t State);

/**
 * @brief Performs a software reset of a GPIO port.
 * @param Port        GPIO port index
 */
void GPIO_Port_RST(gpio_port_t Port);

/**
 * @brief Configures output speed for a GPIO pin.
 * @param GPIOx       GPIO port base address
 * @param PinNumber   Pin number (0 to 15)
 * @param Speed       Output speed setting
 */
void GPIO_OSpeed(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, GPIO_Speed_t Speed);

/**
 * @brief Configures a GPIO pin as an external interrupt source.
 * @param State         ENABLE or DISABLE
 * @param PinNumber     Pin number (0 to 15)
 * @param Port          GPIO port index (A=0, B=1, etc.)
 * @param Trigger_type  Edge trigger type (rising, falling, both)
 */
void GPIO_InterruptConfig(en_state_t State, uint8_t PinNumber, uint8_t Port, edge_t Trigger_type);

/**
 * @brief Enables or disables the NVIC interrupt for a given IRQ number and sets its priority.
 * @param IRQNumber     The interrupt request number (corresponds to vector table position).
 * @param IRQPriority   The priority level to assign (0 = highest, 15 = lowest).
 * @param State         ENABLE to enable the interrupt, DISABLE to disable it.
 */
void NVIC_Config(uint8_t IRQNumber, uint8_t IRQPriority, en_state_t State);

/**
 * @brief Handles an interrupt event on a GPIO pin and clears its EXTI pending flag.
 * @param PinNumber     GPIO pin number (0 to 15) associated with the EXTI line.
 * @return true if the interrupt was pending and is now cleared, false otherwise.
 */
bool GPIO_IRQHandling(uint8_t PinNumber);

// ======================================================================

#endif /* INC_GPIO_DRIVER_H_ */
