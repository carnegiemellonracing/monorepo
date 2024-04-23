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
#include "dma.h"    // cmr_dmaInit()
#include "panic.h"  // cmr_panic()

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
 * @brief Handles SPI completion for the given port.
 *
 * @warning The handle must have been configured through this library!
 *
 * @param handle The HAL SPI handle.
 */
static void cmr_spiDoneCallback(SPI_HandleTypeDef *handle) {
    char *addr = (void *) handle;
    cmr_spi_t *spi = (void *) (addr - offsetof(cmr_spi_t, handle));

    // Disable slave select.
    HAL_GPIO_WritePin(spi->nssPin.port, spi->nssPin.pin, GPIO_PIN_SET);

    // Indicate completion.
    BaseType_t higherWoken;
    if (xSemaphoreGiveFromISR(spi->doneSem, &higherWoken) != pdTRUE) {
        cmr_panic("SPI done semaphore released more than once!");
    }
    portYIELD_FROM_ISR(higherWoken);
}

/**
 * @brief HAL SPI transmit completion handler.
 *
 * @param handle The HAL SPI handle.
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *handle) {
    cmr_spiDoneCallback(handle);
}

/**
 * @brief HAL SPI receive completion handler.
 *
 * @param handle The HAL SPI handle.
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *handle) {
    cmr_spiDoneCallback(handle);
}

/**
 * @brief HAL SPI transaction completion handler.
 *
 * @param handle The HAL SPI handle.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *handle) {
    cmr_spiDoneCallback(handle);
}

/**
 * @brief Initializes the SPI port.
 *
 * The SPI will always be configured to use software NSS, with the pin specified
 * set as a GPIO output and managed automatically during transmit/receive.
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
 */
void cmr_spiInit(
    cmr_spi_t *spi, SPI_TypeDef *instance, const SPI_InitTypeDef *init,
    const cmr_spiPinConfig_t *pins,
    DMA_Stream_TypeDef *rxDMA, uint32_t rxDMAChannel,
    DMA_Stream_TypeDef *txDMA, uint32_t txDMAChannel
) {
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
        .nssPin = pins->nss,
    };

    // Always use software NSS.
    spi->handle.Init.NSS = SPI_NSS_SOFT;

    spi->doneSem = xSemaphoreCreateBinaryStatic(&spi->doneSemBuf);
    configASSERT(spi->doneSem != NULL);

    cmr_rccSPIClockEnable(instance);

    // Configure pins.
    GPIO_InitTypeDef pinConfig = {
        .Pin = pins->mosi.pin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF5_SPI1,     // All SPI ports on AF5.
    };
    cmr_rccGPIOClockEnable(pins->mosi.port);
    HAL_GPIO_Init(pins->mosi.port, &pinConfig);

    pinConfig.Pin = pins->miso.pin;
    cmr_rccGPIOClockEnable(pins->miso.port);
    HAL_GPIO_Init(pins->miso.port, &pinConfig);

    pinConfig.Pin = pins->sck.pin;
    pinConfig.Pull = GPIO_PULLUP;
    cmr_rccGPIOClockEnable(pins->sck.port);
    HAL_GPIO_Init(pins->sck.port, &pinConfig);

    pinConfig = (GPIO_InitTypeDef) {
        .Pin = spi->nssPin.pin,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH
    };
    cmr_rccGPIOClockEnable(spi->nssPin.port);
    HAL_GPIO_Init(spi->nssPin.port, &pinConfig);
    HAL_GPIO_WritePin(spi->nssPin.port, spi->nssPin.pin, GPIO_PIN_SET);

    // Configure DMA.
    // cmr_dmaInit(&spi->rxDMA);
    // cmr_dmaInit(&spi->txDMA);
    // __HAL_LINKDMA(&spi->handle, hdmarx, spi->rxDMA);
    // __HAL_LINKDMA(&spi->handle, hdmatx, spi->txDMA);

    if (HAL_SPI_Init(&spi->handle) != HAL_OK) {
        cmr_panic("HAL_SPI_Init() failed!");
    }
}

/**
 * @brief Performs a transaction over the given SPI port.
 *
 * @note Blocks until the transaction actually completes.
 * @note Does NOT block if the port is currently busy.
 *
 * @param spi The SPI port.
 * @param txData The data to send, or `NULL` to not send any data.
 * @param rxData Buffer for data received, or `NULL` to not receive any data.
 * @param len The number of bytes to receive/transmit.
 *
 * @return 0 on success, or a negative error code if the port is busy.
 */
int cmr_spiTXRX(
    cmr_spi_t *spi, const void *txData, void *rxData, size_t len
) {
    HAL_StatusTypeDef status;
    if (txData == NULL && rxData == NULL) {
        return 0;   // Nothing to do.
    }

    // Enable slave select.
    HAL_GPIO_WritePin(spi->nssPin.port, spi->nssPin.pin, GPIO_PIN_RESET);

    if (txData == NULL) {
        status = HAL_SPI_Receive(&spi->handle, rxData, len, 1);
    } else if (rxData == NULL) {
        status = HAL_SPI_Transmit(&spi->handle, (void *) txData, len, 1);
    } else {
        status = HAL_SPI_TransmitReceive(
            &spi->handle, (void *) txData, rxData, len, 1
        );
    }

    // Enable slave select.
    HAL_GPIO_WritePin(spi->nssPin.port, spi->nssPin.pin, GPIO_PIN_SET);

    switch (status) {
        case HAL_OK:
            break;
//        case HAL_TIMEOUT:
        case HAL_BUSY:
            return -1;
//        default:
            //cmr_panic("HAL SPI transaction failed!");
    }

    // Wait for transaction to complete.
//    if (xSemaphoreTake(spi->doneSem, portMAX_DELAY) != pdTRUE) {
//        cmr_panic("Acquiring SPI port done semaphore timed out!");
//    }

    return 0;
}

#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_SPI_MODULE_ENABLED */

