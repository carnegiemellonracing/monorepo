/**
 * @file uart.c
 * @brief Universal Asynchronous Receiver/Transmitter interface wrapper
 * implementation.
 *
 * The `HAL_UART_...Callback()` functions override weak implementations in the
 * HAL UART driver (`stm32f4xx_hal_uart.c`).
 *
 * @warning All callbacks are called from interrupt handlers!
 *
 * @author Carnegie Mellon Racing
 */

#include "uart.h"    // Interface to implement

#ifdef HAL_USART_MODULE_ENABLED
#ifdef HAL_UART_MODULE_ENABLED
#ifdef HAL_DMA_MODULE_ENABLED

#include "rcc.h"    // cmr_rccUSARTClockEnable(), cmr_rccGPIOClockEnable()
#include "dma.h"    // cmr_dmaInit()

/** @brief U(S)ART device configuration. */
typedef struct {
    /** @brief The associated handle, or `NULL` if not configured. */
    UART_HandleTypeDef *handle;
} cmr_uartDevice_t;

/**
 * @brief All U(S)ART device configurations, indexed by interface.
 *
 * There are 10 U(S)ART interfaces on the STM32F413.
 *
 * @note This array maps USART1 to index 0, USART2 to index 1, etc.
 */
static cmr_uartDevice_t cmr_uartDevices[10];

/**
 * @brief Instantiates the macro for each U(S)ART interface.
 *
 * @param f The macro to instantiate.
 */
#define UART_FOREACH(f) \
    f(1, USART, AF7) \
    f(2, USART, AF7) \
    f(3, USART, AF7) \
    f(4, UART, AF11) \
    f(5, UART, AF8) \
    f(6, USART, AF8) \
    f(7, UART, AF8) \
    f(8, UART, AF8) \
    f(9, UART, AF11) \
    f(10, UART, AF11)

/**
 * @brief Initializes the device corresponding to the given instance.
 *
 * @param instance The HAL U(S)ART instance (`U(S)ARTx` from `stm32f413xx.h`).
 * @param handle The handle to use.
 * @param[out] pinAlternate The GPIO alternate function.
 *
 * @return The corresponding device.
 */
static void cmr_uartDeviceInit(
    USART_TypeDef *instance,
    UART_HandleTypeDef *handle,
    uint8_t *pinAlternate
) {
    cmr_uartDevice_t *device = NULL;
    IRQn_Type irqNum;

    switch ((uintptr_t) instance) {
/**
 * @brief Initializes the given UART device.
 *
 * @param uart The UART interface number.
 * @param pref The UART/USART prefix.
 * @param af The alternate function.
 */
#define UART_DEVICE(uart, pref, af) \
        case pref ## uart ## _BASE: \
            device = cmr_uartDevices + (uart - 1); \
            device->handle = handle; \
            *pinAlternate = GPIO_ ## af ## _ ## pref ## uart; \
            irqNum = pref ## uart ## _IRQn; \
            break;
UART_FOREACH(UART_DEVICE)
#undef UART_DEVICE
        default:
            cmr_panic("Unknown USART instance!", instance);
    }

    HAL_NVIC_SetPriority(irqNum, 7, 0);
    HAL_NVIC_EnableIRQ(irqNum);
}

/**
 * @brief Defines interrupt handlers for each U(S)ART interface.
 *
 * @param uart The UART interface number.
 * @param pref The UART/USART prefix.
 * @param ... Ignored.
 */
#define UART_IRQ_HANDLERS(uart, pref, ...) \
    void pref ## uart ## _IRQHandler(void) { \
        UART_HandleTypeDef *handle = cmr_uartDevices[uart - 1].handle; \
        HAL_UART_IRQHandler(handle); \
        \
        /* Check if line became idle. */ \
        if (__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE)) { \
            /* Disable idle interrupt, clear flag, and abort receive. */ \
            __HAL_UART_DISABLE_IT(handle, UART_IT_IDLE); \
            __HAL_UART_CLEAR_IDLEFLAG(handle); \
            HAL_StatusTypeDef status = HAL_UART_AbortReceive_IT(handle); \
            configASSERT(status == HAL_OK); \
        } \
    }
UART_FOREACH(UART_IRQ_HANDLERS)
#undef UART_IRQ_HANDLERS

/**
 * @brief Gets the corresponding UART interface from the HAL handle.
 *
 * @warning The handle must have been configured through this library!
 *
 * @param handle The handle.
 *
 * @return The interface.
 */
static cmr_uart_t *cmr_uartFromHandle(UART_HandleTypeDef *handle) {
    char *addr = (void *) handle;
    return (void *) (addr - offsetof(cmr_uart_t, handle));
}

/**
 * @brief HAL UART transmit completion handler.
 *
 * @warning Called from an interrupt handler!
 * @warning The handle must have been configured through this library!
 *
 * @param handle The HAL UART handle.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle) {
    cmr_uart_t *uart = cmr_uartFromHandle(handle);

    // Indicate completion.
    BaseType_t higherWoken;
    if (xSemaphoreGiveFromISR(uart->tx.doneSem, &higherWoken) != pdTRUE) {
        cmr_panic("UART TX done semaphore released more than once!");
    }
    portYIELD_FROM_ISR(higherWoken);
}

/**
 * @brief HAL UART receive completion handler.
 *
 * @warning Called from an interrupt handler!
 * @warning The handle must have been configured through this library!
 *
 * @param handle The HAL UART handle.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle) {
    cmr_uart_t *uart = cmr_uartFromHandle(handle);

    // Record remaining bytes in DMA transfer.
    uart->rx.remLen = __HAL_DMA_GET_COUNTER(&uart->rx.dma);

    // Indicate completion.
    BaseType_t higherWoken;
    if (xSemaphoreGiveFromISR(uart->rx.doneSem, &higherWoken) != pdTRUE) {
        cmr_panic("UART RX done semaphore released more than once!");
    }
    portYIELD_FROM_ISR(higherWoken);
}

/**
 * @brief HAL UART error handler.
 *
 * @warning Called from an interrupt handler!
 * @warning The handle must have been configured through this library!
 *
 * @param handle The HAL UART handle.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle) {
    // Treat errors as receive completion, since errors in DMA mode will
    // terminate receive DMA.
    HAL_UART_RxCpltCallback(handle);
}

/**
 * @brief Initializes the UART interface.
 *
 * @warning It is undefined behavior to initialize the same HAL UART or DMA
 * stream instances more than once!
 *
 * @param uart The UART interface to initialize.
 * @param instance The HAL U(S)ART instance (`U(S)ARTx` from `stm32f413xx.h`).
 * @param init The HAL UART configuration (see `stm32f4xx_hal_uart.h`).
 * @param rxPort Receiving GPIO port (`GPIOx` from `stm32f413xx.h`).
 * @param rxPin Receiving GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`).
 * @param txPort Transmitting GPIO port.
 * @param txPin Transmitting GPIO pin.
 * @param rxDMA HAL DMA stream instance for receiving data (`DMAx_StreamN` from
 * `stm32f413xx.h`).
 * @param rxDMAChannel HAL DMA channel for receiving data (`DMA_CHANNEL_x` from
 * `stm32f4xx_hal_dma.h`).
 * @param txDMA HAL DMA stream instance for transmitting data.
 * @param txDMAChannel HAL DMA channel for transmitting data.
 */
void cmr_uartInit(
    cmr_uart_t *uart, USART_TypeDef *instance, const UART_InitTypeDef *init,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin,
    DMA_Stream_TypeDef *rxDMA, uint32_t rxDMAChannel,
    DMA_Stream_TypeDef *txDMA, uint32_t txDMAChannel
) {
    *uart = (cmr_uart_t) {
        .handle = {
            .Instance = instance,
            .Init = *init
        },
        .rx = {
            .dma = {
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
            }
        },
        .tx = {
            .dma = {
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
            }
        }
    };

    uart->rx.doneSem = xSemaphoreCreateBinaryStatic(&uart->rx.doneSemBuf);
    configASSERT(uart->rx.doneSem != NULL);

    uart->tx.doneSem = xSemaphoreCreateBinaryStatic(&uart->tx.doneSemBuf);
    configASSERT(uart->tx.doneSem != NULL);

    cmr_rccUSARTClockEnable(instance);

    // Configure UART device.
    uint8_t pinAlternate;
    cmr_uartDeviceInit(instance, &uart->handle, &pinAlternate);

    // Configure pins.
    GPIO_InitTypeDef pinConfig = {
        .Pin = rxPin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = pinAlternate
    };
    HAL_GPIO_Init(rxPort, &pinConfig);
    cmr_rccGPIOClockEnable(rxPort);

    pinConfig.Pin = txPin;
    HAL_GPIO_Init(txPort, &pinConfig);
    cmr_rccGPIOClockEnable(txPort);

    // Configure DMA.
    cmr_dmaInit(&uart->rx.dma);
    cmr_dmaInit(&uart->tx.dma);
    __HAL_LINKDMA(&uart->handle, hdmarx, uart->rx.dma);
    __HAL_LINKDMA(&uart->handle, hdmatx, uart->tx.dma);

    if (HAL_UART_Init(&uart->handle) != HAL_OK) {
        cmr_panic("HAL_UART_Init() failed!");
    }
}

/**
 * @brief Transmits data over the given UART interface.
 *
 * @note Blocks until the transmission actually completes.
 * @note Does NOT block if the port is currently busy.
 *
 * @param uart The UART interface.
 * @param data The data to send, or `NULL` to not send any data.
 * @param len The number of bytes to transmit.
 *
 * @return 0 on success, or a negative error code if the port is busy.
 */
int cmr_uartTX(cmr_uart_t *uart, const void *data, size_t len) {
    if (data == NULL || len == 0) {
        return 0;   // Nothing to do.
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
        &uart->handle, (void *) data, len
    );
    switch (status) {
        case HAL_OK:
            break;
        case HAL_BUSY:
            return -1;
        default:
            cmr_panic("HAL UART TX failed!");
    }

    // Wait for transmission to complete.
    if (xSemaphoreTake(uart->tx.doneSem, portMAX_DELAY) != pdTRUE) {
        cmr_panic("Acquiring UART port done semaphore timed out!");
    }

    return 0;
}

/**
 * @brief Receives data over the given UART interface.
 *
 * @note Blocks until the reception actually completes.
 * @note Does NOT block if the port is currently busy.
 *
 * @param uart The UART interface.
 * @param data The buffer for receiving, or `NULL` to not receive any data.
 * @param lenp Points to the receive buffer's size in bytes. Upon successful
 * return, the number of bytes actually received is placed here.
 *
 * @return 0 on success, or a negative error code if the port is busy.
 */
int cmr_uartRX(cmr_uart_t *uart, void *data, size_t *lenp) {
    size_t bufLen = *lenp;
    if (data == NULL || bufLen == 0) {
        return 0;   // Nothing to do.
    }

    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(
        &uart->handle, data, bufLen
    );
    switch (status) {
        case HAL_OK:
            break;
        case HAL_BUSY:
            return -1;
        default:
            cmr_panic("HAL UART RX failed!");
    }

    // Enable idle line detection.
    __HAL_UART_ENABLE_IT(&uart->handle, UART_IT_IDLE);

    // Wait for transmission to complete.
    if (xSemaphoreTake(uart->rx.doneSem, portMAX_DELAY) != pdTRUE) {
        cmr_panic("Acquiring UART port done semaphore timed out!");
    }

    // Calculate number of bytes actually received.
    size_t remLen = uart->rx.remLen;
    configASSERT(remLen <= bufLen);
    *lenp = bufLen - remLen;

    return 0;
}

#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_UART_MODULE_ENABLED */
#endif /* HAL_USART_MODULE_ENABLED */

