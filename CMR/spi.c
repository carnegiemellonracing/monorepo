/**
 * @file spi.c
 * @brief Serial Peripheral Interface port wrapper implementation.
 *
 * The `HAL_SPI_...Callback()` functions override weak implementations in the
 * HAL SPI driver (`stm32f4xx_hal_spi.c`).
 *
 * @warning All callbacks are called from interrupt handlers!
 *
 * @author Carnegie Mellon Racing
 */

#include "spi.h"    // Interface to implement

#ifdef HAL_SPI_MODULE_ENABLED
#ifdef HAL_DMA_MODULE_ENABLED

#include "rcc.h"    // cmr_rccSPIClockEnable(), cmr_rccGPIOClockEnable()
#include "panic.h"  // cmr_panic()

/**
 * @brief Default transaction completion callback.
 *
 * @param spi The SPI port.
 */
static void cmr_spiDoneCallbackDefault(cmr_spi_t *spi) {
    (void) spi;
}

/**
 * @brief HAL SPI error handler.
 *
 * @param handle The HAL SPI handle.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *handle) {
    (void) handle;
    cmr_panic("HAL SPI error callback reached!");
}

/**
 * @brief HAL SPI transmit completion handler.
 *
 * @param handle The HAL SPI handle.
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *handle) {
    cmr_spi_t *spi = (cmr_spi_t *) handle;
    spi->doneCallback(spi);
}

/**
 * @brief HAL SPI receive completion handler.
 *
 * @param handle The HAL SPI handle.
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *handle) {
    cmr_spi_t *spi = (cmr_spi_t *) handle;
    spi->doneCallback(spi);
}

/**
 * @brief HAL SPI transaction completion handler.
 *
 * @param handle The HAL SPI handle.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *handle) {
    cmr_spi_t *spi = (cmr_spi_t *) handle;
    spi->doneCallback(spi);
}

/**
 * @brief Initializes the SPI port.
 *
 * @warning It is undefined behavior to initialize the same HAL SPI or DMA
 * stream instances more than once!
 *
 * @param spi The SPI port to initialize.
 * @param instance The HAL SPI instance (`SPIx` from `stm32f413xx.h`).
 * @param init The HAL SPI configuration (see `stm32f4xx_hal_spi.h`).
 * @param pins Pin configuration.
 * @param rxDMA HAL DMA stream instance for receiving data (`DMAx_StreamN` from
 * `stm32f413xx.h`).
 * @param rxDMAChannel HAL DMA channel for receiving data (`DMA_CHANNEL_x` from
 * `stm32f4xx_hal_dma.h`).
 * @param txDMA HAL DMA stream instance for transmitting data.
 * @param txDMAChannel HAL DMA channel for transmitting data.
 * @param doneCallback Transaction completion callback (or `NULL` to ignore).
 */
void cmr_spiInit(
    cmr_spi_t *spi, SPI_TypeDef *instance, const SPI_InitTypeDef *init,
    const cmr_spiPinConfig_t *pins,
    DMA_Stream_TypeDef *rxDMA, uint32_t rxDMAChannel,
    DMA_Stream_TypeDef *txDMA, uint32_t txDMAChannel,
    cmr_spiCallback_t doneCallback
) {
    if (doneCallback == NULL) {
        doneCallback = cmr_spiDoneCallbackDefault;
    }

    *spi = (cmr_spi_t) {
        .handle = {
            .Instance = instance,
            .Init = *init
        },
        .rxDMA = {
            .Instance = rxDMA,
            .Init = {
                .Channel = rxDMAChannel,
                .Direction = DMA_PERIPH_TO_MEMORY,
                .PeriphInc = DMA_PINC_DISABLE,
                .MemInc = DMA_MINC_ENABLE,
                .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
                .MemDataAlignment = DMA_MDATAALIGN_BYTE,
                .Mode = DMA_NORMAL,
                .Priority = DMA_PRIORITY_LOW,
                .FIFOMode = DMA_FIFOMODE_DISABLE
            }
        },
        .txDMA = {
            .Instance = txDMA,
            .Init = {
                .Channel = txDMAChannel,
                .Direction = DMA_MEMORY_TO_PERIPH,
                .PeriphInc = DMA_PINC_DISABLE,
                .MemInc = DMA_MINC_ENABLE,
                .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
                .MemDataAlignment = DMA_MDATAALIGN_BYTE,
                .Mode = DMA_NORMAL,
                .Priority = DMA_PRIORITY_LOW,
                .FIFOMode = DMA_FIFOMODE_DISABLE
            }
        },
        .doneCallback = doneCallback
    };

    cmr_rccSPIClockEnable(instance);

    // Configure pins.
    for (size_t i = 0; i < sizeof(*pins) / sizeof(cmr_spiPin_t); i++) {
        const cmr_spiPin_t *pin = ((const cmr_spiPin_t *) pins) + i;

        GPIO_InitTypeDef pinConfig = {
            .Pin = pin->pin,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI1,     // All SPI ports on AF5.
        };
        HAL_GPIO_Init(pin->port, &pinConfig);
    }

    // Configure DMA.
    cmr_dmaInit(&spi->rxDMA);
    cmr_dmaInit(&spi->txDMA);
    __HAL_LINKDMA(&spi->handle, hdmarx, &spi->rxDMA);
    __HAL_LINKDMA(&spi->handle, hdmatx, &spi->txDMA);

    if (HAL_SPI_Init(&spi->handle) != HAL_OK) {
        cmr_panic("HAL_SPI_Init() failed!");
    }
}

/**
 * @brief Initiates a transaction over the given SPI port.
 *
 * @note This function does NOT block. Upon success, the registered callbacks
 * will be called via an ISR.
 *
 * @warning This function is NOT synchronized!
 *
 * @param spi The SPI port.
 * @param txData The data to send, or `NULL` to not send any data.
 * @param rxData Buffer for data received, or `NULL` to not receive any data.
 * @param len The number of bytes to receive/transmit.
 *
 * @return 0 on success, or a negative error code if the port is busy.
 */
int cmr_spiTXRX(
    cmr_spi_t *spi, void *txData, const void *rxData, size_t len
) {
    HAL_StatusTypeDef status;
    if (txData == NULL) {
        if (rxData == NULL) {
            return 0;   // Nothing to do.
        }

        status = HAL_SPI_Receive_DMA(&spi->handle, rxData, len);
    } else if (rxData == NULL) {
        status = HAL_SPI_Transmit_DMA(&spi->handle, txData, len);
    } else {
        status = HAL_SPI_TransmitReceive_DMA(&spi->handle, txData, rxData, len);
    }

    switch (status) {
        case HAL_OK:
            return 0;
        case HAL_BUSY:
            return -1;
        default:
            cmr_panic("HAL SPI transaction failed!");
    }
}

#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_SPI_MODULE_ENABLED */

