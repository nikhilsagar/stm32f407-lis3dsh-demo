/*
 ******************************************************************************
 * @file    main.c
 * @author  Auto-generated
 * @brief   Minimal SPI1 half-duplex TX-only demo with manual CS
 *
 * - SPI1 master, Mode 0, 8-bit, BR=/256 (slowest)
 * - SCK  = PA5 (AF5), MOSI = PA7 (AF5)  [Half-duplex TX uses MOSI pin]
 * - CS   = PA4 (software-controlled)
 * - MISO = PA6 unused
 * - Sends text repeatedly; CS is toggled around each transfer
 ******************************************************************************
 */

#include "STM32F407VGT.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include <string.h>
#include <stdio.h>

/* ------------ Pin map (STM32F4DISC) ------------ */
#define LED_PORT         GPIOD
#define LED_PIN_GREEN    12U
#define LED_PIN_RED      14U

/* ------------ Globals ------------ */
static SPI_Handle_t hspi1, hspi2;

/* ------------ Prototypes ------------ */
static void Init_LEDs(void);
static void Init_Buttons(void);
static void Init_SPI1_GPIO(void);
static void Init_SPI1_Master_TXOnly(void);
static void Init_SPI2_GPIO(void);
static void Init_SPI2_Master_TXRX(void);
static void DelayCycles(volatile uint32_t cycles);

/* ================== Application ================================================================== */
int main(void)
{
    /* Board peripherals */
    Init_LEDs();
    Init_Buttons();
    // Init_SPI1_CS_Soft();

    RCC->APB2ENR |= (1U << 12);   /* Enable SPI1 clock  */
    Init_SPI1_GPIO();
    Init_SPI1_Master_TXOnly();

    RCC->APB1ENR |= (1U << 14);   /* Enable SPI2 clock  */
    Init_SPI2_GPIO();
    Init_SPI2_Master_TXRX();

    const char msg[] = "my name is Nikhil";

    /* SPI2 full-duplex loopback test */
    const uint8_t tx2[] = "ok got your name1";
    uint8_t rx2[sizeof tx2] = {0};

    /* Run loop */
    while (1)
    {
        SPI_PeripheralControl(&hspi1, ENABLE);
        SPI_SendData(&hspi1, (uint8_t *)msg, (uint32_t)strlen(msg));
        SPI_PeripheralControl(&hspi1, DISABLE);

        SPI_PeripheralControl(&hspi2, ENABLE);
        SPI_TransmitReceive(&hspi2, tx2, rx2, (uint32_t)strlen((const char *)tx2));
        SPI_PeripheralControl(&hspi2, DISABLE);

        printf("%s\r\n", (char *)rx2);   /* ✅ prints "SPI2 loopback" */

        if (memcmp(tx2, rx2, strlen((const char *)tx2)) == 0)
        {
            GPIO_TogglePin(LED_PORT, LED_PIN_GREEN);   /* pass */
        }
        else
        {
            GPIO_TogglePin(LED_PORT, LED_PIN_RED);     /* fail */
        }

        DelayCycles(500000);
    }
}

/* ================== Init helpers ================== */
static void Init_LEDs(void)
{
    GPIO_EnableClock(GPIO_PORT_D, ENABLE);

    GPIO_Config_t led = {0};
    led.Mode       = GPIO_MODE_OUTPUT;
    led.OutputType = GPIO_OTYPE_PP;
    led.PuPd       = GPIO_NOPULL;

    GPIO_Init(LED_PORT, LED_PIN_GREEN, &led);
    GPIO_Init(LED_PORT, LED_PIN_RED, &led);

    GPIO_OSpeed(LED_PORT, LED_PIN_GREEN, GPIO_SPEED_LOW);
    GPIO_OSpeed(LED_PORT, LED_PIN_RED,   GPIO_SPEED_LOW);
}

static void Init_Buttons(void)
{
    GPIO_EnableClock(GPIO_PORT_A, ENABLE);
    GPIO_EnableClock(GPIO_PORT_E, ENABLE);

    GPIO_Config_t btn = {0};
    btn.Mode = GPIO_MODE_INPUT;
    btn.PuPd = GPIO_PULLDOWN;

    GPIO_Init(GPIOA, 0U, &btn);   /* User button on PA0 */
    GPIO_Init(GPIOE, 3U, &btn);   /* MEMS CS pin on PE3 */

    /* Optional: EXTI setup */
    GPIO_InterruptConfig(ENABLE, 0, GPIO_PORT_A, GPIO_TRIGGER_FALLING);
    NVIC_Config(EXTI0_IRQ_NUM, 10, ENABLE);

    GPIO_InterruptConfig(ENABLE, 3, GPIO_PORT_E, GPIO_TRIGGER_FALLING);
    NVIC_Config(EXTI3_IRQ_NUM, 10, ENABLE);
}

static void Init_SPI1_GPIO(void)
{
    GPIO_EnableClock(GPIO_PORT_A, ENABLE);

    /* Common AF5 config */
    GPIO_Config_t af = {0};
    af.Mode        = GPIO_MODE_ALT;
    af.OutputType  = GPIO_OTYPE_PP;
    af.PuPd        = GPIO_NOPULL;     /* keep NOPULL for SCK/MOSI */
    af.AltFunction = 5;               /* AF5 = SPI1 */

    /* SCK (PA5) and MOSI (PA7) */
    GPIO_OSpeed(GPIOA, 5U, GPIO_SPEED_LOW);
    GPIO_OSpeed(GPIOA, 7U, GPIO_SPEED_LOW);
    GPIO_OSpeed(GPIOA, 6U, GPIO_SPEED_LOW);
    GPIO_Init(GPIOA, 7U, &af);        /* PA7 = MOSI */
    GPIO_Init(GPIOA, 6U, &af);        /* PA6 = MISO */

    af.PuPd = GPIO_PULLDOWN;
    GPIO_Init(GPIOA, 5U, &af);        /* PA5 = SCK */

    /* NSS (PA4) for hardware NSS: small pull-up helps keep line high when SPI disabled */
    GPIO_OSpeed(GPIOA, 4U, GPIO_SPEED_LOW);
    af.PuPd = GPIO_PULLUP;
    GPIO_Init(GPIOA, 4U, &af);        /* PA4 = NSS (hardware) */
}

static void Init_SPI1_Master_TXOnly(void)
{
    hspi1.SPIx             = SPI1;
    hspi1.config.deviceMode = SPI_MODE_MASTER;
    hspi1.config.busConfig  = SPI_BUS_HALF_DUPLEX;
    hspi1.config.dff        = SPI_DFF_8BITS;
    hspi1.config.cpol       = SPI_CPOL_LOW;    /* Mode 0 */
    hspi1.config.cpha       = SPI_CPHA_FIRST;
    hspi1.config.ssm        = SPI_SSM_DISABLE;
    hspi1.config.clkSpeed   = 7;               /* BR=/256 */

    SPI_Init(&hspi1);

    /* Enable SSOE so NSS is driven high when SPI is enabled */
    SPI_SSOEConfig(SPI1, ENABLE);
}

static void Init_SPI2_GPIO(void)
{
    /* SPI2 pins live on GPIOB */
    GPIO_EnableClock(GPIO_PORT_B, ENABLE);

    GPIO_Config_t af = {0};
    af.Mode        = GPIO_MODE_ALT;
    af.OutputType  = GPIO_OTYPE_PP;
    af.AltFunction = 5;                 /* AF5 = SPI1/SPI2 on STM32F4 */

    /* PB13 = SCK  (pulldown helps keep line quiet when disabled) */
    af.PuPd = GPIO_PULLDOWN;
    GPIO_OSpeed(GPIOB, 13U, GPIO_SPEED_HIGH);
    GPIO_Init(GPIOB, 13U, &af);

    /* PB14 = MISO (no pull) */
    af.PuPd = GPIO_NOPULL;
    GPIO_OSpeed(GPIOB, 14U, GPIO_SPEED_HIGH);
    GPIO_Init(GPIOB, 14U, &af);

    /* PB15 = MOSI (no pull) */
    GPIO_OSpeed(GPIOB, 15U, GPIO_SPEED_HIGH);
    GPIO_Init(GPIOB, 15U, &af);

    /* PB12 = NSS (hardware CS) — pull-up idle high */
    af.PuPd = GPIO_PULLUP;
    GPIO_OSpeed(GPIOB, 12U, GPIO_SPEED_HIGH);
    GPIO_Init(GPIOB, 12U, &af);
}

static void Init_SPI2_Master_TXRX(void)
{
    hspi2.SPIx              = SPI2;
    hspi2.config.deviceMode = SPI_MODE_MASTER;
    hspi2.config.busConfig  = SPI_BUS_FULL_DUPLEX;
    hspi2.config.dff        = SPI_DFF_8BITS;
    hspi2.config.cpol       = SPI_CPOL_LOW;     /* Mode 0 */
    hspi2.config.cpha       = SPI_CPHA_FIRST;
    hspi2.config.ssm        = SPI_SSM_DISABLE;  /* Hardware NSS */
    hspi2.config.clkSpeed   = 7;                /* BR=/256 */

    SPI_Init(&hspi2);

    /* Enable SSOE so NSS is driven LOW while SPE=1 (and HIGH when SPE=0) */
    SPI_SSOEConfig(SPI2, ENABLE);
}

/* ================== ISRs ================== */
void EXTI0_IRQHandler(void)
{
    if (GPIO_IRQHandling(0))
    {
        GPIO_TogglePin(LED_PORT, LED_PIN_GREEN);
    }
}

void EXTI3_IRQHandler(void)
{
    if (GPIO_IRQHandling(3))
    {
        GPIO_TogglePin(LED_PORT, LED_PIN_RED);
    }
}

/* ================== Delay ================== */
static void DelayCycles(volatile uint32_t cycles)
{
    while (cycles--)
    {
        __asm volatile ("nop");
    }
}
