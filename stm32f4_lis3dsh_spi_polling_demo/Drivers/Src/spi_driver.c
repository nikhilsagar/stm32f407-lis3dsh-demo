/*
 * @file    spi_driver.c
 * @author  Nikhil Sagar
 * @brief   STM32F407 SPI driver (Phase‑1: polling/blocking, helper‑free)
 *
 * @details
 * This module provides a minimal, robust SPI driver for STM32F4 in
 * polling/blocking mode:
 *  - Initialization from a user-specified configuration
 *  - Explicit enable/disable (SPE)
 *  - Blocking transmit / receive / transmit-receive
 *  - Exact-width writes/reads to DR (8/16-bit) matching DFF
 *  - Tail flush (TXE then BSY=0) and defensive OVR clearing inline
 *
 * Design notes:
 *  - SPI_Init() programs CR1 but does NOT enable SPE; use SPI_PeripheralControl().
 *  - No local/static helpers are used; loops are kept inline for clarity.
 *  - Bit positions follow the STM32F4 reference manual mapping and are written
 *    directly (no new macros introduced in this file).
 */

#include "spi_driver.h"

/* ========================================================================== */
/*                                Public API                                   */
/* ========================================================================== */

/**
 * @brief  Initialize the SPI peripheral with the configuration in @p hspi.
 *
 * @param  hspi  Pointer to SPI handle (contains SPIx and user config)
 *
 * @note   Programs CR1 according to the configuration:
 *         - MSTR (bit 2) from deviceMode
 *         - Bus config (BIDIMODE/BIDIOE/RXONLY)
 *         - BR[2:0] (bits 5:3) from clkSpeed (/2..../256)
 *         - DFF (bit 11), CPOL (bit 1), CPHA (bit 0)
 *         - SSM (bit 9); if SSM=1 then SSI (bit 8)=1
 *         The function does NOT set SPE; call SPI_PeripheralControl() to enable.
 */
void SPI_Init(SPI_Handle_t *hspi)
{
    uint32_t cr1 = 0U;

    /* 1) Master/slave (CR1.MSTR bit 2) */
    cr1 |= ((uint32_t)hspi->config.deviceMode << 2);

    /* 2) Bus configuration */
    if (hspi->config.busConfig == SPI_BUS_FULL_DUPLEX)
    {
        cr1 &= ~(1U << 15);                 /* BIDIMODE=0 */
    }
    else if (hspi->config.busConfig == SPI_BUS_HALF_DUPLEX)
    {
        cr1 |=  (1U << 15);                 /* BIDIMODE=1 (1-line) */
        cr1 |=  (1U << 14);                 /* BIDIOE=1 (output enabled) */
    }
    else /* SPI_BUS_SIMPLEX_RXONLY */
    {
        cr1 &= ~(1U << 15);                 /* BIDIMODE=0 */
        cr1 |=  (1U << 10);                 /* RXONLY=1  */
    }

    /* 3) Baud rate (CR1.BR[2:0] = bits [5:3]) */
    cr1 |= ((uint32_t)hspi->config.clkSpeed << 3);

    /* 4) Frame size (CR1.DFF bit 11) */
    cr1 |= ((uint32_t)hspi->config.dff << 11);

    /* 5) Polarity/phase (CR1.CPOL bit 1, CR1.CPHA bit 0) */
    cr1 |= ((uint32_t)hspi->config.cpol << 1);
    cr1 |= ((uint32_t)hspi->config.cpha << 0);

    /* 6) Software slave management (CR1.SSM bit 9) */
    cr1 |= ((uint32_t)hspi->config.ssm << 9);

    /* 7) Internal NSS when SSM=1 (CR1.SSI bit 8) */
    if (hspi->config.ssm == SPI_SSM_ENABLE)
    {
        cr1 |= (1U << 8);                   /* keep NSS high internally (SSI = 1)*/
    }

    /* 8) Program CR1 */
    hspi->SPIx->CR1 = cr1;
}

/**
 * @brief  Reset the SPI peripheral registers to their reset state.
 * @param  hspi  Pointer to SPI handle
 * @note   Uses the corresponding APB reset bits per instance.
 */
void SPI_DeInit(SPI_Handle_t *hspi)
{
    if (hspi->SPIx == SPI1)
    {
        RCC->APB2RSTR |=  (1U << 12);
        RCC->APB2RSTR &= ~(1U << 12);
    }
    else if (hspi->SPIx == SPI2)
    {
        RCC->APB1RSTR |=  (1U << 14);
        RCC->APB1RSTR &= ~(1U << 14);
    }
    else if (hspi->SPIx == SPI3)
    {
        RCC->APB1RSTR |=  (1U << 15);
        RCC->APB1RSTR &= ~(1U << 15);
    }
}

/**
 * @brief  Enable or disable the SPI peripheral (CR1.SPE).
 * @param  hspi    Pointer to SPI handle
 * @param  enable  ENABLE(1) or DISABLE(0)
 * @note   Do not toggle SPE during an active frame.
 */
void SPI_PeripheralControl(SPI_Handle_t *hspi, uint8_t enable)
{
    if (enable) { hspi->SPIx->CR1 |=  (1U << 6); }  /* SPE=1 */
    else        { hspi->SPIx->CR1 &= ~(1U << 6); }  /* SPE=0 */
}

/**
 * @brief  Configure SSOE (CR2.SSOE) to let hardware drive NSS in master mode.
 * @param  SPIx    SPI instance pointer
 * @param  enable  ENABLE(1) or DISABLE(0)
 * @note   With SSOE=1, hardware drives NSS low during transfers and high when idle.
 */
void SPI_SSOEConfig(SPI_RegDef_t *SPIx, uint8_t enable)
{
    if (enable) { SPIx->CR2 |=  (1U << 2); }   /* SSOE=1 */
    else        { SPIx->CR2 &= ~(1U << 2); }
}

/**
 * @brief  Configure SSI (CR1.SSI) when SSM is enabled (software NSS).
 * @param  SPIx    SPI instance pointer
 * @param  enable  ENABLE(1) or DISABLE(0)
 * @note   In master mode with SSM=1, set SSI=1 to avoid mode fault (MODF).
 */
void SPI_SSIConfig(SPI_RegDef_t *SPIx, uint8_t enable)
{
    if (enable) { SPIx->CR1 |=  (1U << 8); }   /* SSI=1 */
    else        { SPIx->CR1 &= ~(1U << 8); }
}

/**
 * @brief  Transmit bytes via SPI in blocking mode (polling).
 *
 * @param  hspi       Pointer to SPI handle
 * @param  pTxBuffer  Pointer to transmit buffer
 * @param  len        Number of bytes to transmit
 *
 * @note   - Uses exact-width DR accesses based on DFF (8/16-bit).
 *         - Waits TXE before each write.
 *         - Tail flush at the end: wait TXE=1 then BSY=0.
 * @warning In 16-bit mode (DFF=1), @p len must be even. The buffer is expected
 *          MSB-first per word: p[i]=high byte, p[i+1]=low byte.
 */
void SPI_SendData(SPI_Handle_t *hspi, const uint8_t *pTxBuffer, uint32_t len)
{
    if (len == 0U) return;

    uint8_t is16bit = (hspi->SPIx->CR1 & (1U << 11)) ? 1U : 0U; /* DFF */

    if (is16bit)
    {
        if (len & 1U) return;  /* 2 bytes per 16-bit frame */

        for (uint32_t i = 0; i < len; i += 2)
        {
            /* Wait TXE=1 */
            while (!(hspi->SPIx->SR & (1U << 1))) { /* TXE */ }

            /* Pack MSB-first and write exact 16 bits */
            uint16_t t = ((uint16_t)pTxBuffer[i] << 8) | (uint16_t)pTxBuffer[i + 1U];
            *(__vo uint16_t *)&hspi->SPIx->DR = t;
        }
    }
    else
    {
        for (uint32_t i = 0; i < len; ++i)
        {
            /* Wait TXE=1 */
            while (!(hspi->SPIx->SR & (1U << 1))) { /* TXE */ }

            /* Exact 8-bit write */
            *(__vo uint8_t *)&hspi->SPIx->DR = pTxBuffer[i];
        }
    }

    /* Tail flush: ensure the last frame completed */
    while (!(hspi->SPIx->SR & (1U << 1))) { /* TXE */ }
    while  ( hspi->SPIx->SR & (1U << 7))  { /* BSY */ }
}

/**
 * @brief  Receive bytes via SPI in blocking mode (polling).
 *
 * @param  hspi       Pointer to SPI handle
 * @param  pRxBuffer  Destination buffer
 * @param  len        Number of bytes to receive
 *
 * @note   - In master mode, dummy data (0xFF/0xFFFF) is written to generate SCK.
 *         - Uses exact-width DR accesses based on DFF (8/16-bit).
 *         - Waits RXNE before each read.
 *         - Tail flush at the end and defensive OVR clear.
 * @warning In 16-bit mode (DFF=1), @p len must be even. Received bytes are stored
 *          MSB-first per word: p[i]=high byte, p[i+1]=low byte.
 */
void SPI_ReceiveData(SPI_Handle_t *hspi, uint8_t *pRxBuffer, uint32_t len)
{
    if (len == 0U) return;

    uint8_t is16bit  = (hspi->SPIx->CR1 & (1U << 11)) ? 1U : 0U; /* DFF */
    uint8_t isMaster = (hspi->SPIx->CR1 & (1U <<  2)) ? 1U : 0U; /* MSTR */

    if (is16bit)
    {
        if (len & 1U) return;

        for (uint32_t i = 0; i < len; i += 2)
        {
            /* Master must push a dummy frame to clock SCK */
            if (isMaster)
            {
                while (!(hspi->SPIx->SR & (1U << 1))) { /* TXE */ }
                *(__vo uint16_t *)&hspi->SPIx->DR = 0xFFFFU;
            }

            /* Wait RXNE=1 and read exact 16 bits */
            while (!(hspi->SPIx->SR & (1U << 0))) { /* RXNE */ }
            uint16_t r = *(__vo uint16_t *)&hspi->SPIx->DR;

            /* Store MSB then LSB */
            pRxBuffer[i]   = (uint8_t)(r >> 8);
            pRxBuffer[i+1] = (uint8_t)(r & 0xFFU);
        }
    }
    else
    {
        for (uint32_t i = 0; i < len; ++i)
        {
            if (isMaster)
            {
                while (!(hspi->SPIx->SR & (1U << 1))) { /* TXE */ }
                *(__vo uint8_t *)&hspi->SPIx->DR = 0xFFU;
            }

            while (!(hspi->SPIx->SR & (1U << 0))) { /* RXNE */ }
            pRxBuffer[i] = *(__vo uint8_t *)&hspi->SPIx->DR;
        }
    }

    /* Tail flush & defensive OVR clear */
    while (!(hspi->SPIx->SR & (1U << 1))) { /* TXE */ }
    while  ( hspi->SPIx->SR & (1U << 7))  { /* BSY */ }

    if (hspi->SPIx->SR & (1U << 6))        /* OVR */
    {
        volatile uint32_t tmp;
        tmp = *(__vo uint16_t *)&hspi->SPIx->DR; /* read DR first */
        tmp = hspi->SPIx->SR;                     /* then SR to clear OVR */
        (void)tmp;
    }
}

/**
 * @brief  Full‑duplex transmit/receive (blocking).
 *
 * @param  hspi  Pointer to SPI handle
 * @param  pTx   Transmit buffer (NULL → send dummy 0xFF/0xFFFF)
 * @param  pRx   Receive buffer (NULL → read & discard to avoid OVR)
 * @param  len   Number of bytes to transfer
 *
 * @retval >=0   Number of BYTES transferred
 * @retval -1    Invalid args (e.g., odd @p len in 16-bit mode)
 *
 * @note   - Uses exact-width DR accesses based on DFF (8/16-bit).
 *         - Waits TXE before write, RXNE before read, per frame.
 *         - Tail flush at the end and defensive OVR clear.
 */
int SPI_TransmitReceive(SPI_Handle_t *hspi,
                        const uint8_t *pTx,
                        uint8_t       *pRx,
                        uint32_t       len)
{
    if (len == 0U) return 0;

    if (hspi->config.dff == SPI_DFF_16BITS)
    {
        if (len & 1U) return -1; /* two bytes per frame */

        for (uint32_t i = 0; i < len; i += 2)
        {
            /* Write 16-bit (dummy if pTx==NULL) */
            while (!(hspi->SPIx->SR & (1U << 1))) { /* TXE */ }
            uint16_t t = pTx ? (uint16_t)(((uint16_t)pTx[i] << 8) | pTx[i+1U])
                             : (uint16_t)0xFFFFU;
            *(__vo uint16_t *)&hspi->SPIx->DR = t;

            /* Read 16-bit */
            while (!(hspi->SPIx->SR & (1U << 0))) { /* RXNE */ }
            uint16_t r = *(__vo uint16_t *)&hspi->SPIx->DR;

            if (pRx)
            {
                pRx[i]   = (uint8_t)(r >> 8);
                pRx[i+1] = (uint8_t)(r & 0xFFU);
            }
        }
    }
    else /* 8-bit frames */
    {
        for (uint32_t i = 0; i < len; ++i)
        {
            /* Write 8-bit (dummy if pTx==NULL) */
            while (!(hspi->SPIx->SR & (1U << 1))) { /* TXE */ }
            uint8_t t = pTx ? pTx[i] : 0xFFU;
            *(__vo uint8_t *)&hspi->SPIx->DR = t;

            /* Read 8-bit */
            while (!(hspi->SPIx->SR & (1U << 0))) { /* RXNE */ }
            uint8_t r = *(__vo uint8_t *)&hspi->SPIx->DR;

            if (pRx) { pRx[i] = r; }
        }
    }

    /* Tail flush & defensive OVR clear */
    while (!(hspi->SPIx->SR & (1U << 1))) { /* TXE */ }
    while  ( hspi->SPIx->SR & (1U << 7))  { /* BSY */ }

    if (hspi->SPIx->SR & (1U << 6))        /* OVR */
    {
        (void)*(__vo uint16_t *)&hspi->SPIx->DR;
        (void)hspi->SPIx->SR;
    }

    return (int)len;
}
