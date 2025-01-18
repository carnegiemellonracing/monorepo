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
#include "panic.h"

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



#ifdef H725

/**
 * @brief Instantiates the macro for each U(S)ART interface.
 *
 * @param f The macro to instantiate.
 */
#define UART_FOREACH(f) \
    f(1, USART, AF7) \
    f(2, USART, AF7) \
    f(3, USART, AF7) \
    f(4, UART, AF6) \
    f(5, UART, AF8) \
    f(6, USART, AF7) \
    f(7, UART, AF11) \
    f(8, UART, AF8) \
    f(9, UART, AF11) \
    f(10, USART, AF11)



/**
 * @brief Instantiates the macro for each CAN interface.
 *
 * @param f The macro to instantiate.
 */

uint32_t DMA_UART_Request_From_Instance(USART_TypeDef *instance, bool Tx) {
	switch((uintptr_t) instance) {
#define DMA_HANDLE_CONFIG(num, uart, af) \
        case ## uart ## num: \
            if(Tx) { \
				return DMA_REQUEST_ ## uart ## num ## _TX ; \
			} \
			else { \
				return DMA_REQUEST_ ## uart ## num ## _RX ; \
			} \
UART_FOREACH(DMA_HANDLE_CONFIG)
#undef DMA_HANDLE_CONFIG
	default:
		cmr_panic("Invalid UART Handle!");
	}
}

#else

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
    f(5, UART, AF11) \
    f(6, USART, AF8) \
    f(7, UART, AF8) \
    f(8, UART, AF8) \
    f(9, UART, AF11) \
    f(10, UART, AF11)
#endif
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
        \
        /* Handle idle interrupt, if present. */ \
        if (__HAL_UART_GET_IT_SOURCE(handle, UART_IT_IDLE)) { \
            /* Disable idle interrupt, clear flag, and abort receive. */ \
            __HAL_UART_DISABLE_IT(handle, UART_IT_IDLE); \
            __HAL_UART_CLEAR_IDLEFLAG(handle); \
            HAL_StatusTypeDef status = HAL_UART_AbortReceive_IT(handle); \
            configASSERT(status == HAL_OK); \
        } \
        HAL_UART_IRQHandler(handle); \
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

    BaseType_t higherWoken;
    cmr_uartMsg_t *msg;
    if (xQueueReceiveFromISR(uart->tx.q, &msg, &higherWoken) != pdTRUE) {
        cmr_panic("HAL UART TX completion handler called with empty queue!");
    }
    portYIELD_FROM_ISR(higherWoken);

    // Signal message as done.
    if (xSemaphoreGiveFromISR(msg->doneSem, &higherWoken) != pdTRUE) {
        cmr_panic("HAL UART TX completion handler failed to signal message!");
    }
    portYIELD_FROM_ISR(higherWoken);

    if (xQueuePeekFromISR(uart->tx.q, &msg) != pdTRUE) {
        // No more messages pending; release DMA semaphore.
        if (xSemaphoreGiveFromISR(uart->tx.dmaSem, &higherWoken) != pdTRUE) {
            cmr_panic("HAL UART TX completion handler failed to release DMA!");
        }
        portYIELD_FROM_ISR(higherWoken);
        return;
    }

    // Message pending; start DMA.
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
        &uart->handle, msg->data, msg->len
    );
    if (status != HAL_OK) {
        cmr_panic("HAL UART TX failed!");
    }
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

    BaseType_t higherWoken;
    cmr_uartMsg_t *msg;
    if (xQueueReceiveFromISR(uart->rx.q, &msg, &higherWoken) != pdTRUE) {
        cmr_panic("HAL UART RX completion handler called with empty queue!");
    }
    portYIELD_FROM_ISR(higherWoken);

    // Record actual number of bytes received.
    size_t remLen = __HAL_DMA_GET_COUNTER(&uart->rx.dma);
    configASSERT(remLen <= msg->len);
    msg->len -= remLen;

    // Signal message as done.
    if (xSemaphoreGiveFromISR(msg->doneSem, &higherWoken) != pdTRUE) {
        cmr_panic("HAL UART RX completion handler failed to signal message!");
    }
    portYIELD_FROM_ISR(higherWoken);

    if (xQueuePeekFromISR(uart->rx.q, &msg) != pdTRUE) {
        // No more messages pending; release DMA semaphore.
        if (xSemaphoreGiveFromISR(uart->rx.dmaSem, &higherWoken) != pdTRUE) {
            cmr_panic("HAL UART RX completion handler failed to release DMA!");
        }
        portYIELD_FROM_ISR(higherWoken);
        return;
    }

    // Message pending; start DMA.
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(
        &uart->handle, msg->data, msg->len
    );
    if (status != HAL_OK) {
        cmr_panic("HAL UART RX failed!");
    }

    if (msg->opts & CMR_UART_RXOPTS_IDLEABORT) {
        // Enable idle line detection.
        __HAL_UART_ENABLE_IT(&uart->handle, UART_IT_IDLE);
    }
}

/**
 * @brief HAL UART receive abort completion handler.
 *
 * @warning Called from an interrupt handler!
 * @warning The handle must have been configured through this library!
 *
 * @param handle The HAL UART handle.
 */
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *handle) {
    // Receive aborted; treat as completion.
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
#ifdef H725
            	.Request = DMA_UART_Request_From_Instance(instance, false),
#else
                .Channel = rxDMAChannel,
#endif
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
#ifdef H725
            	.Request = DMA_UART_Request_From_Instance(instance, true),
#else
                .Channel = rxDMAChannel,
#endif
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

    uart->rx.dmaSem = xSemaphoreCreateBinaryStatic(&uart->rx.dmaSemBuf);
    configASSERT(uart->rx.dmaSem != NULL);
    if (xSemaphoreGive(uart->rx.dmaSem) != pdTRUE) {
        cmr_panic("UART RX DMA semaphore initialization failed!");
    }

    uart->rx.q = xQueueCreateStatic(
        sizeof(uart->rx.qItemBuf) / sizeof(uart->rx.qItemBuf[0]),
        sizeof(uart->rx.qItemBuf[0]),
        (void *) uart->rx.qItemBuf,
        &uart->rx.qBuf
    );

    uart->tx.dmaSem = xSemaphoreCreateBinaryStatic(&uart->tx.dmaSemBuf);
    configASSERT(uart->tx.dmaSem != NULL);
    if (xSemaphoreGive(uart->tx.dmaSem) != pdTRUE) {
        cmr_panic("UART TX DMA semaphore initialization failed!");
    }

    uart->tx.q = xQueueCreateStatic(
        sizeof(uart->tx.qItemBuf) / sizeof(uart->tx.qItemBuf[0]),
        sizeof(uart->tx.qItemBuf[0]),
        (void *) uart->tx.qItemBuf,
        &uart->tx.qBuf
    );

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
    cmr_rccGPIOClockEnable(rxPort);
    HAL_GPIO_Init(rxPort, &pinConfig);

    pinConfig.Pin = txPin;
    cmr_rccGPIOClockEnable(txPort);
    HAL_GPIO_Init(txPort, &pinConfig);

    // Configure DMA.
    cmr_dmaInit(&uart->rx.dma);
    cmr_dmaInit(&uart->tx.dma);
    __HAL_LINKDMA(&uart->handle, hdmarx, uart->rx.dma);
    __HAL_LINKDMA(&uart->handle, hdmatx, uart->tx.dma);

    if (HAL_UART_Init(&uart->handle) != HAL_OK) {
        cmr_panic("HAL_UART_Init() failed!");
    }
    
    // Disable idle interrupt and clear the flag.
    __HAL_UART_DISABLE_IT(&uart->handle, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(&uart->handle);
}

/**
 * @brief Enqueues message for transmission over the given UART interface.
 *
 * @note Does not block; use `cmr_uartMsgWait()` on the provided message to wait
 * for the transmission to complete.
 *
 * @param uart The UART interface.
 * @param msg The message to use.
 * @param data The data to send, or `NULL` to not send any data.
 * @param len The number of bytes to transmit.
 */
void cmr_uartTX(
    cmr_uart_t *uart, cmr_uartMsg_t *msg, const void *data, size_t len
) {
    if (data == NULL || len == 0) {
        return;     // Nothing to do.
    }

    msg->data = (void *) data;
    msg->len = len;

    // Mark message as not done yet.
    // Ignore return value; pdFALSE just means the message was already marked.
    xSemaphoreTake(msg->doneSem, 0);

    if (xQueueSend(uart->tx.q, &msg, portMAX_DELAY) != pdTRUE) {
        cmr_panic("UART TX queue send timed out!");
    }

    if (xSemaphoreTake(uart->tx.dmaSem, 0) != pdTRUE) {
        // DMA already active; the completion handler will handle our request.
        return;
    }

    // Start DMA.
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
        &uart->handle, msg->data, msg->len
    );
    if (status != HAL_OK) {
        cmr_panic("HAL UART TX failed!");
    }
}

/**
 * @brief Receives data over the given UART interface.
 *
 * @note Does not block; use `cmr_uartMsgWait()` on the provided message to wait
 * for the transmission to complete.
 *
 * @param uart The UART interface.
 * @param msg The message to use.
 * @param data The buffer for receiving, or `NULL` to not receive any data.
 * @param len The number of bytes to transmit.
 * @param opts Receive options.
 */
void cmr_uartRX(
    cmr_uart_t *uart, cmr_uartMsg_t *msg,
    void *data, size_t len, cmr_uartRXOpts_t opts
) {
    if (data == NULL || len == 0) {
        return;     // Nothing to do.
    }

    msg->opts = opts;
    msg->data = (void *) data;
    msg->len = len;

    // Mark message as not done yet.
    // Ignore return value; pdFALSE just means the message was already marked.
    xSemaphoreTake(msg->doneSem, 0);

    if (xQueueSend(uart->rx.q, &msg, portMAX_DELAY) != pdTRUE) {
        cmr_panic("UART RX queue send timed out!");
    }

    if (xSemaphoreTake(uart->rx.dmaSem, 0) != pdTRUE) {
        // DMA already active; the completion handler will handle our request.
        return;
    }

    // Start DMA.
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(
        &uart->handle, msg->data, msg->len
    );
    if (status != HAL_OK) {
        cmr_panic("HAL UART RX failed!");
    }

    if (msg->opts & CMR_UART_RXOPTS_IDLEABORT) {
        // Enable idle line detection.
        __HAL_UART_ENABLE_IT(&uart->handle, UART_IT_IDLE);
    }

    return;
}

/**
 * @brief Initializes a UART message.
 *
 * @param msg The message to initialize.
 */
void cmr_uartMsgInit(cmr_uartMsg_t *msg) {
    msg->data = NULL;
    msg->len = 0;

    msg->doneSem = xSemaphoreCreateBinaryStatic(&msg->doneSemBuf);
    configASSERT(msg->doneSem != NULL);
    if (xSemaphoreGive(msg->doneSem) != pdTRUE) {
        cmr_panic("UART message done semaphore initialization failed!");
    }
}

/**
 * @brief Destroys a UART message.
 *
 * @param msg The message to destroy.
 */
void cmr_uartMsgDestroy(cmr_uartMsg_t *msg) {
    vSemaphoreDelete(msg->doneSem);
}

/**
 * @brief Blocks until the message transmit/receive completes.
 *
 * @warning The message MUST have been enqueued for transmit/receive via
 * `cmr_uartTX()`/`cmr_uartRX()`, or freshly initialized.
 *
 * @param msg The message.
 *
 * @returns The message's length. For transmit, this is the number of bytes
 * transmitted; for receive, this is the number of bytes received.
 */
size_t cmr_uartMsgWait(cmr_uartMsg_t *msg) {
    if (xSemaphoreTake(msg->doneSem, portMAX_DELAY) != pdTRUE) {
        cmr_panic("UART message wait timed out!");
    }

    return msg->len;
}


/**
 * @brief initializes polling UART
 *
 * @param
 */
void cmr_uart_polling_init(cmr_uart_t *uart, USART_TypeDef *instance, const UART_InitTypeDef *init,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin) {

    *uart = (cmr_uart_t) {
        .handle = {
            .Instance = instance,
            .Init = *init
        }
    };

    // Enable UART Clock
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
    cmr_rccGPIOClockEnable(rxPort);
    HAL_GPIO_Init(rxPort, &pinConfig);

    pinConfig.Pin = txPin;
    cmr_rccGPIOClockEnable(txPort);
    HAL_GPIO_Init(txPort, &pinConfig);

    if (HAL_UART_Init(&uart->handle) != HAL_OK) {
        cmr_panic("HAL_UART_Init() failed!");
    }

    return;
}

/**
* @brief transmits a byte over UART
*
* @param
*/
cmr_uart_result_t cmr_uart_pollingTX(cmr_uart_t *uart, uint8_t *data, uint16_t length) {
//	TickType_t lastWakeTime = xTaskGetTickCount();

    if (uart == NULL || data == NULL) {
        return UART_FAILURE;
    }

    int timeout = CMR_UART_DEFAULT_TIMEOUT;
    HAL_StatusTypeDef status = HAL_ERROR;

    while (timeout > 0 && status != HAL_OK) {
        // May still hang within task critical sections

    	status = HAL_UART_Transmit(
            &(uart->handle), data, length, 1);
        timeout--;
    }

//    vTaskDelayUntil(&lastWakeTime, 1);


    if (status != HAL_OK) {
        return UART_FAILURE;
    }
    return UART_SUCCESS;
}

/**
* @brief receives a byte over UART
*/
cmr_uart_result_t cmr_uart_pollingRX(cmr_uart_t *uart, uint8_t *data, uint16_t length) {
    if (uart == NULL || data == NULL) {
        return UART_FAILURE;
    }

    int timeout = CMR_UART_DEFAULT_TIMEOUT;
    HAL_StatusTypeDef status = HAL_ERROR;

    while (timeout > 0 && status != HAL_OK) {
        // May still hang within task critical sections
        status = HAL_UART_Receive(
            &(uart->handle), data, length, 1);
        timeout--;
    }

    if (status != HAL_OK) {
        return UART_FAILURE;
    }
    return UART_SUCCESS;
}

#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_UART_MODULE_ENABLED */
#endif /* HAL_USART_MODULE_ENABLED */
