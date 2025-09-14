/**
 * @file    spi_driver.h
 * @author  Nikhil Sagar
 * @brief   SPI driver interface for STM32F407
 *
 * @details
 * This header defines the API, data structures, and register mappings
 * for the Serial Peripheral Interface (SPI) on STM32F4 microcontrollers.
 * It provides configuration options for:
 *  - Master/slave selection
 *  - Full-/half-duplex and simplex RX-only communication
 *  - Clock polarity and phase (CPOL/CPHA)
 *  - Data frame size (8-bit or 16-bit)
 *  - Software and hardware slave management (SSM/NSS)
 *  - Polling-based blocking send and receive operations
 *
 * @note The implementation targets the STM32F407xx MCU family.
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include "STM32f407VGT.h"

// ============================================================================
//                           BASE ADDRESSES
// ============================================================================

/** @brief SPI1 base address on APB2 */
#define SPI1_BASEADDR   (APB2PERIPH_BASE + 0x3000U)
/** @brief SPI2 base address on APB1 */
#define SPI2_BASEADDR   (APB1PERIPH_BASE + 0x3800U)
/** @brief SPI3 base address on APB1 */
#define SPI3_BASEADDR   (APB1PERIPH_BASE + 0x3C00U)

/** @brief SPI1 peripheral definition */
#define SPI1    ((SPI_RegDef_t *) SPI1_BASEADDR)
/** @brief SPI2 peripheral definition */
#define SPI2    ((SPI_RegDef_t *) SPI2_BASEADDR)
/** @brief SPI3 peripheral definition */
#define SPI3    ((SPI_RegDef_t *) SPI3_BASEADDR)

// ============================================================================
//                           ENUMERATIONS
// ============================================================================

/**
 * @enum  SPI_DeviceMode_t
 * @brief Selects SPI operating mode.
 */
typedef enum {
    SPI_MODE_SLAVE,   /**< Configure SPI as Slave */
    SPI_MODE_MASTER   /**< Configure SPI as Master */
} SPI_DeviceMode_t;

/**
 * @enum  SPI_BusConfig_t
 * @brief Selects SPI bus configuration.
 */
typedef enum {
    SPI_BUS_FULL_DUPLEX,     /**< Full duplex: MISO + MOSI */
    SPI_BUS_HALF_DUPLEX,     /**< Half duplex: single bi-directional line */
    SPI_BUS_SIMPLEX_RXONLY   /**< Simplex: receive-only mode */
} SPI_BusConfig_t;

/**
 * @enum  SPI_DFF_t
 * @brief Selects SPI data frame format.
 */
typedef enum {
    SPI_DFF_8BITS,   /**< 8-bit data frame */
    SPI_DFF_16BITS   /**< 16-bit data frame */
} SPI_DFF_t;

/**
 * @enum  SPI_CPOL_t
 * @brief SPI clock polarity options.
 */
typedef enum {
    SPI_CPOL_LOW,    /**< Clock idles low */
    SPI_CPOL_HIGH    /**< Clock idles high */
} SPI_CPOL_t;

/**
 * @enum  SPI_CPHA_t
 * @brief SPI clock phase options.
 */
typedef enum {
    SPI_CPHA_FIRST,   /**< Sample on first clock edge */
    SPI_CPHA_SECOND   /**< Sample on second clock edge */
} SPI_CPHA_t;

/**
 * @enum  SPI_SSM_t
 * @brief Software slave management options.
 */
typedef enum {
    SPI_SSM_DISABLE,   /**< Use hardware NSS pin */
    SPI_SSM_ENABLE     /**< Use software-controlled NSS */
} SPI_SSM_t;

// ============================================================================
//                        REGISTER DEFINITION STRUCTURE
// ============================================================================

/**
 * @struct SPI_RegDef_t
 * @brief  Register definition for an SPI peripheral.
 */
typedef struct {
    __vo uint32_t CR1;       /**< Control register 1, offset: 0x00 */
    __vo uint32_t CR2;       /**< Control register 2, offset: 0x04 */
    __vo uint32_t SR;        /**< Status register, offset: 0x08 */
    __vo uint32_t DR;        /**< Data register, offset: 0x0C */
    __vo uint32_t CRCPR;     /**< CRC polynomial register, offset: 0x10 */
    __vo uint32_t RXCRCR;    /**< RX CRC register, offset: 0x14 */
    __vo uint32_t TXCRCR;    /**< TX CRC register, offset: 0x18 */
    __vo uint32_t I2SCFGR;   /**< I2S configuration register, offset: 0x1C */
    __vo uint32_t I2SPR;     /**< I2S prescaler register, offset: 0x20 */
} SPI_RegDef_t;

// ============================================================================
//                          CONFIGURATION STRUCTURES
// ============================================================================

/**
 * @struct SPI_Config_t
 * @brief  Holds SPI configuration parameters.
 */
typedef struct {
    SPI_DeviceMode_t deviceMode;   /**< Master or Slave mode */
    SPI_BusConfig_t  busConfig;    /**< Full-duplex, Half-duplex, or Simplex RX */
    SPI_DFF_t        dff;          /**< Data frame format (8/16 bits) */
    SPI_CPOL_t       cpol;         /**< Clock polarity */
    SPI_CPHA_t       cpha;         /**< Clock phase */
    SPI_SSM_t        ssm;          /**< Software slave management */
    uint8_t          clkSpeed;     /**< Clock prescaler: 0–7 => /2 to /256 */
} SPI_Config_t;

/**
 * @struct SPI_Handle_t
 * @brief  SPI handle structure: combines registers and config.
 */
typedef struct {
    SPI_RegDef_t *SPIx;     /**< Pointer to SPI peripheral base */
    SPI_Config_t  config;   /**< User-specified configuration */
} SPI_Handle_t;

// ============================================================================
//                               DRIVER API
// ============================================================================

/**
 * @brief  Initialize the SPI peripheral with given configuration.
 * @param  hspi Pointer to SPI handle structure
 */
void SPI_Init(SPI_Handle_t *hspi);

/**
 * @brief  Reset and disable the SPI peripheral.
 * @param  hspi Pointer to SPI handle
 */
void SPI_DeInit(SPI_Handle_t *hspi);

/**
 * @brief  Enable or disable the SPI peripheral.
 * @param  hspi   Pointer to SPI handle
 * @param  enable ENABLE (1) or DISABLE (0)
 */
void SPI_PeripheralControl(SPI_Handle_t *hspi, uint8_t enable);

/**
 * @brief  Transmit data via SPI in blocking (polling) mode.
 * @param  hspi       Pointer to SPI handle
 * @param  pTxBuffer  Pointer to transmit buffer
 * @param  len        Number of bytes to transmit
 */
void SPI_SendData(SPI_Handle_t *hspi, const uint8_t *pTxBuffer, uint32_t len);

/**
 * @brief  Receive data via SPI in blocking (polling) mode.
 * @param  hspi       Pointer to SPI handle
 * @param  pRxBuffer  Pointer to receive buffer
 * @param  len        Number of bytes to receive
 */
void SPI_ReceiveData(SPI_Handle_t *hspi, uint8_t *pRxBuffer, uint32_t len);

/**
 * @brief  Configure the SSI bit (software slave management).
 * @param  SPIx    SPI base pointer
 * @param  enable  ENABLE (1) or DISABLE (0)
 */
void SPI_SSIConfig(SPI_RegDef_t *SPIx, uint8_t enable);

/**
 * @brief  Configure SSOE (slave select output enable).
 * @param  SPIx    SPI base pointer
 * @param  enable  ENABLE (1) or DISABLE (0)
 */
void SPI_SSOEConfig(SPI_RegDef_t *SPIx, uint8_t enable);

/**
 * @brief  Perform full-duplex transmit and receive in blocking mode.
 * @param  hspi   Pointer to SPI handle
 * @param  pTx    Pointer to transmit buffer (NULL for dummy writes)
 * @param  pRx    Pointer to receive buffer
 * @param  len    Number of bytes to transfer
 * @retval Number of bytes successfully transferred
 */
int SPI_TransmitReceive(SPI_Handle_t *hspi,
                        const uint8_t *pTx,
                        uint8_t *pRx,
                        uint32_t len);

#endif /* INC_SPI_DRIVER_H_ */
