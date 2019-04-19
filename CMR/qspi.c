/**
 * @file qspi.c
 * @brief QuadSPI implementation.
 *
 * The `HAL_QSPI_...Callback()` functions override weak implementations in the
 * HAL SPI driver (`stm32f4xx_hal_spi.c`).
 *
 * @warning All callbacks are called from interrupt handlers!
 *
 * @author Carnegie Mellon Racing
 */

#include "qspi.h"   // Interface to implement
#include "dma.h"    // DMA interface
#include "rcc.h"    // cmr_rccQSPIClockEnable(), cmr_rccGPIOClockEnable()

/** @brief QuadSPI device configuration. */
typedef struct {
    /** @brief The associated handle, or `NULL` if not configured. */
    QSPI_HandleTypeDef *handle;
} cmr_qspiDevice_t;

/**
 * @brief All QuadSPI device configurations, indexed by interface.
 *
 * There is only 1 QuadSPI interface on the STM32F413.
 */
static cmr_qspiDevice_t cmr_qspiDevices[1];

/**
 * @brief QuadSPI interrupt handler.
 */
void QUADSPI_IRQHandler(void) {
    HAL_QSPI_IRQHandler(cmr_qspiDevices[0].handle);
}

/**
 * @brief Handles QuadSPI completion for the given port.
 *
 * @warning The handle must have been configured through this library!
 *
 * @param handle The HAL QuadSPI handle.
 */
static void cmr_qspiDoneCallback(QSPI_HandleTypeDef *handle) {
    char *addr = (void *) handle;
    cmr_qspi_t *qspi = (void *) (addr - offsetof(cmr_qspi_t, handle));

    // Indicate completion.
    BaseType_t higherWoken;
    if (xSemaphoreGiveFromISR(qspi->doneSem, &higherWoken) != pdTRUE) {
        cmr_panic("QuadSPI done semaphore released more than once!");
    }
    portYIELD_FROM_ISR(higherWoken);
}

/**
 * @brief HAL QuadSPI command completion handler.
 *
 * @param handle The HAL UART handle.
 */
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *handle) {
    cmr_qspiDoneCallback(handle);
}

/**
 * @brief HAL QuadSPI transmit completion handler.
 *
 * @param handle The HAL UART handle.
 */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *handle) {
    cmr_qspiDoneCallback(handle);
}

/**
 * @brief HAL QuadSPI receive completion handler.
 *
 * @param handle The HAL UART handle.
 */
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *handle) {
    cmr_qspiDoneCallback(handle);
}

/**
 * @brief Initializes QuadSPI.
 *
 * @warning It is undefined behavior to initialize the same HAL QSPI or DMA
 * stream instances more than once!
 *
 * @param qspi The QuadSPI port to initialize.
 * @param instance The HAL QUADSPI instance (`QUADSPI` from `stm32f413xx.h`).
 * @param init The HAL QSPI configuration (see `stm32f4xx_hal_spi.h`).
 * @param pins Pin configuration.
 * @param dma HAL DMA stream instance (`DMAx_StreamN` from `stm32f413xx.h`).
 * @param dmaChannel HAL DMA channel (`DMA_CHANNEL_x` from
 * `stm32f4xx_hal_dma.h`).
 */
void cmr_qspiInit(
    cmr_qspi_t *qspi, QUADSPI_TypeDef *instance, const QSPI_InitTypeDef *init,
    const cmr_qspiPinConfig_t *pins,
    DMA_Stream_TypeDef *dma, uint32_t dmaChannel
) {
    *qspi = (cmr_qspi_t) {
        .handle = {
            .Instance = instance,
            .Init = *init
        },
        .dma = {
            .Instance = dma,
            .Init = {
                .Channel = dmaChannel,
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
    };

    qspi->doneSem = xSemaphoreCreateBinaryStatic(&qspi->doneSemBuf);
    configASSERT(qspi->doneSem != NULL);

    cmr_rccQSPIClockEnable(instance);

    // Configure pins.
    GPIO_InitTypeDef pinConfig = {
        .Pin = pins->sck.pin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF9_QSPI
    };
    cmr_rccGPIOClockEnable(pins->sck.port);
    HAL_GPIO_Init(pins->sck.port, &pinConfig);

    pinConfig.Pin = pins->nss.pin;
    cmr_rccGPIOClockEnable(pins->nss.port);
    HAL_GPIO_Init(pins->nss.port, &pinConfig);

    // All I/O pins are AF10.
    pinConfig.Alternate = GPIO_AF10_OTG_FS;
    for (size_t i = 0; i < sizeof(pins->io) / sizeof(pins->io[0]); i++) {
        const cmr_qspiPin_t *pin = pins->io + i;
        pinConfig.Pin = pin->pin;
        cmr_rccGPIOClockEnable(pin->port);
        HAL_GPIO_Init(pin->port, &pinConfig);
    }

    // Configure DMA.
    cmr_dmaInit(&qspi->dma);
    __HAL_LINKDMA(&qspi->handle, hdma, qspi->dma);

    // Configure device.
    cmr_qspiDevices[0].handle = &qspi->handle;
    HAL_NVIC_SetPriority(QUADSPI_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(QUADSPI_IRQn);

    if (HAL_QSPI_Init(&qspi->handle) != HAL_OK) {
        cmr_panic("HAL_QSPI_Init() failed!");
    }
}

/**
 * @brief Sends a command that has no data.
 *
 * @note Blocks until the transaction actually completes.
 * @note Does NOT block if the port is currently busy.
 *
 * @warning `cmd->DataMode` MUST be `QSPI_DATA_NONE`!
 *
 * @param qspi The QuadSPI port.
 * @param cmd The command's configuration.
 */
void cmr_qspiCmd(cmr_qspi_t *qspi, const QSPI_CommandTypeDef *cmd) {
    configASSERT(cmd->DataMode == QSPI_DATA_NONE);

    // HAL does not declare the command pointer `const`, even though it doesn't
    // modify it. Oh well.
    if (
        HAL_QSPI_Command_IT(&qspi->handle, (QSPI_CommandTypeDef *) cmd) !=
        HAL_OK
    ) {
        cmr_panic("HAL_QSPI_Command_IT() failed!");
    }

    // Wait for transaction to complete.
    if (xSemaphoreTake(qspi->doneSem, portMAX_DELAY) != pdTRUE) {
        cmr_panic("Acquiring QuadSPI port done semaphore timed out!");
    }
}

/**
 * @brief Sends data.
 *
 * @note Blocks until the transaction actually completes.
 * @note Does NOT block if the port is currently busy.
 *
 * @warning `cmd->DataMode` MUST NOT be `QSPI_DATA_NONE`!
 * @warning `data` MUST have at least `cmd->NbData` bytes!
 *
 * @param qspi The QuadSPI port.
 * @param cmd The command's configuration.
 */
void cmr_qspiTX(
    cmr_qspi_t *qspi, const QSPI_CommandTypeDef *cmd, const void *data
) {
    configASSERT(cmd->DataMode != QSPI_DATA_NONE);

    // HAL does not declare the command pointer `const`, even though it doesn't
    // modify it. Oh well.
    if (
        HAL_QSPI_Command(&qspi->handle, (QSPI_CommandTypeDef *) cmd, 0) !=
        HAL_OK
    ) {
        cmr_panic("HAL_QSPI_Command() failed!");
    }

    // HAL does not declare the data pointer `const`, even though it doesn't
    // modify it. Oh well.
    if (HAL_QSPI_Transmit_DMA(&qspi->handle, (void *) data) != HAL_OK) {
        cmr_panic("HAL_QSPI_Transmit_DMA() failed!");
    }

    // Wait for transaction to complete.
    if (xSemaphoreTake(qspi->doneSem, portMAX_DELAY) != pdTRUE) {
        cmr_panic("Acquiring QuadSPI port done semaphore timed out!");
    }
}

/**
 * @brief Receives data.
 *
 * @note Blocks until the transaction actually completes.
 * @note Does NOT block if the port is currently busy.
 *
 * @warning `cmd->DataMode` MUST NOT be `QSPI_DATA_NONE`!
 * @warning `data` MUST have at least `cmd->NbData` bytes!
 *
 * @param qspi The QuadSPI port.
 * @param cmd The command's configuration.
 */
void cmr_qspiRX(
    cmr_qspi_t *qspi, const QSPI_CommandTypeDef *cmd, void *data
) {
    configASSERT(cmd->DataMode != QSPI_DATA_NONE);

    // HAL does not declare the command pointer `const`, even though it doesn't
    // modify it. Oh well.
    if (
        HAL_QSPI_Command(&qspi->handle, (QSPI_CommandTypeDef *) cmd, 0) !=
        HAL_OK
    ) {
        cmr_panic("HAL_QSPI_Command() failed!");
    }

    if (HAL_QSPI_Receive_DMA(&qspi->handle, data) != HAL_OK) {
        cmr_panic("HAL_QSPI_Receive_DMA() failed!");
    }

    // Wait for transaction to complete.
    if (xSemaphoreTake(qspi->doneSem, portMAX_DELAY) != pdTRUE) {
        cmr_panic("Acquiring QuadSPI port done semaphore timed out!");
    }
}

