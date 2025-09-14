/**
 * @file    gpio_driver.c
 * @author  Nikhil Sagar
 * @brief   GPIO driver implementation for STM32F407
 *
 * This file provides the implementation for GPIO initialization,
 * configuration, read/write operations, speed control, and EXTI-based
 * interrupt configuration.
 */

#include "STM32f407VGT.h"
#include "gpio_driver.h"
#include <stdbool.h>

/**
 * @brief Initializes a GPIO pin based on the provided configuration structure.
 * @note Assumes GPIO clock is already enabled.
 */
void GPIO_Init(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, GPIO_Config_t *config)
{
    uint8_t mode  = config->Mode;

    if (PinNumber > 15) return; // Ensure valid pin number

    // 1. Configure pin mode
    GPIOx->MODER &= ~(0x3U << (2 * PinNumber));
    GPIOx->MODER |= ((mode & 0x3U) << (2 * PinNumber));

    // 2. Configure output type
    if (mode == GPIO_MODE_OUTPUT || mode == GPIO_MODE_ALT)
    {
        GPIOx->OTYPER &= ~(1U << PinNumber);
        GPIOx->OTYPER |= ((config->OutputType & 0x1U) << PinNumber);
    }

    // 3. Configure pull-up/pull-down
    GPIOx->PUPDR &= ~(0x3U << (2 * PinNumber));
    GPIOx->PUPDR |= ((config->PuPd & 0x3U) << (2 * PinNumber));

    // 4. Configure alternate function if needed
    if (mode == GPIO_MODE_ALT)
    {
        uint8_t afr_index = PinNumber / 8;
        uint8_t afr_shift = (PinNumber % 8) * 4;
        GPIOx->AFR[afr_index] &= ~(0xFU << afr_shift);
        GPIOx->AFR[afr_index] |= ((config->AltFunction & 0xFU) << afr_shift);
    }
}

/**
 * @brief Configures the output speed of a GPIO pin.
 */
void GPIO_OSpeed(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, GPIO_Speed_t Speed)
{
    GPIOx->OSPEEDR &= ~(0x3U << (2 * PinNumber));
    GPIOx->OSPEEDR |= ((Speed & 0x3U) << (2 * PinNumber));
}

/**
 * @brief Writes a value to a specific GPIO pin using BSRR register.
 */
void GPIO_WritePin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, uint8_t value)
{
    if (PinNumber < 16 && (value == 0 || value == 1)) {
        GPIOx->BSRR = (value == 1) ? (1U << PinNumber) : (1U << (PinNumber + 16));
    }
}

/**
 * @brief Writes a value to the entire GPIO port.
 */
void GPIO_WritePort(GPIO_RegDef_t *GPIOx, uint16_t value)
{
    GPIOx->ODR = value & 0xFFFFU;
}

/**
 * @brief Reads the logic level of a specific GPIO pin.
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber)
{
    return (uint8_t)((GPIOx->IDR >> PinNumber) & 0x01U);
}

/**
 * @brief Reads the logic level of all 16 GPIO pins.
 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t *GPIOx)
{
    return (uint16_t)(GPIOx->IDR & 0xFFFF);
}

/**
 * @brief Toggles the logic level of a GPIO output pin.
 */
void GPIO_TogglePin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber)
{
    if (PinNumber > 15) return;
    GPIOx->ODR ^= (1U << PinNumber);
}

/**
 * @brief Enables or disables the AHB1 clock for a GPIO port.
 */
void GPIO_EnableClock(gpio_port_t Port, en_state_t State)
{
    if (Port <= GPIO_PORT_E || Port == GPIO_PORT_H)
    {
        RCC->AHB1ENR = (State == ENABLE)
                     ? (RCC->AHB1ENR | (1U << Port))
                     : (RCC->AHB1ENR & ~(1U << Port));
    }
}

/**
 * @brief Performs a software reset of the specified GPIO port.
 */
void GPIO_Port_RST(gpio_port_t Port)
{
    RCC->AHB1RSTR |= (1U << Port);
    RCC->AHB1RSTR &= ~(1U << Port);
}

/**
 * @brief Configures a GPIO pin as an external interrupt source.
 */
void GPIO_InterruptConfig(en_state_t State, uint8_t PinNumber, uint8_t Port, edge_t Trigger_type)
{
    if (State == ENABLE)
    {
        uint8_t exti_index = PinNumber / 4;
        uint8_t shift = (PinNumber % 4) * 4;

        // 1. Enable SYSCFG clock
        RCC->APB2ENR |= (1 << 14);

        // 2. Configure EXTI line source
        SYSCFG->EXTICR[exti_index] &= ~(0xF << shift);
        SYSCFG->EXTICR[exti_index] |= (Port << shift);

        // 3. Unmask EXTI line
        EXTI->IMR |= (1U << PinNumber);

        // 4. Configure edge trigger
        switch (Trigger_type)
        {
            case GPIO_TRIGGER_RISING:
                EXTI->RTSR |= (1U << PinNumber);
                EXTI->FTSR &= ~(1U << PinNumber);
                break;

            case GPIO_TRIGGER_FALLING:
                EXTI->FTSR |= (1U << PinNumber);
                EXTI->RTSR &= ~(1U << PinNumber);
                break;

            case GPIO_TRIGGER_BOTH:
                EXTI->RTSR |= (1U << PinNumber);
                EXTI->FTSR |= (1U << PinNumber);
                break;
        }

        // 5. Enable NVIC IRQ (optional, left for application to decide)
        // NVIC_EnableIRQ(EXTI0_IRQn + PinNumber);
    }
    else // DISABLE
    {
        EXTI->IMR &= ~(1U << PinNumber);
        // NVIC_DisableIRQ(EXTI0_IRQn + PinNumber);
    }
}

void NVIC_Config(uint8_t IRQNumber, uint8_t IRQPriority, en_state_t State)
{
    if (State == ENABLE)
    {
        // 1. Enable IRQ
        ISER[IRQNumber / 32] = (1U << (IRQNumber % 32));

        // 2. Set IRQ priority (4 MSBs used in STM32F4)
        IPR[IRQNumber] = (IRQPriority & 0xF) << 4;
    }
    else
    {
        // Disable IRQ
        ICER[IRQNumber / 32] = (1U << (IRQNumber % 32));
    }
}

bool GPIO_IRQHandling(uint8_t PinNumber)
{
    if (EXTI->PR & (1U << PinNumber))
    {
        EXTI->PR |= (1U << PinNumber);  // Clear interrupt
        return true;  // Interrupt was pending
    }
    return false;     // No interrupt
}

