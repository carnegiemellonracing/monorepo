/**
 * @file can.c
 * @brief Controller Area Network wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "CMR/can.h"    // Interface to implement

#ifdef HAL_CAN_MODULE_ENABLED

#include <string.h> // memcpy()
#include <stdbool.h> // bool

#include "CMR/rcc.h"    // cmr_rccCANClockEnable(), cmr_rccGPIOClockEnable()
#include "CMR/panic.h"  // cmr_panic()

/**
 * @brief Gets the corresponding CAN interface from the HAL handle.
 *
 * @warning The handle must have been configured through this library!
 *
 * @param handle The handle.
 *
 * @return The interface.
 */
static cmr_can_t *cmr_canFromHandle(CAN_HandleTypeDef *handle) {
    char *addr = (void *) handle;
    return (void *) (addr - offsetof(cmr_can_t, handle));
}

/**
 * @brief Callback for CAN transmit mailbox completion.
 *
 * @warning Called from an interrupt handler!
 * @warning The handle must have been configured through this library!
 *
 * @param handle The HAL CAN handle.
 * @param mailbox The completed mailbox.
 */
static void cmr_canTXCpltCallback(CAN_HandleTypeDef *handle, size_t mailbox) {
    (void) mailbox;     // Placate compiler.
    cmr_can_t *can = cmr_canFromHandle(handle);

    // Indicate completion.
    BaseType_t higherWoken;
    if (xSemaphoreGiveFromISR(can->txSem, &higherWoken) != pdTRUE) {
        cmr_panic("TX semaphore released too many times!");
    }
    portYIELD_FROM_ISR(higherWoken);
}

/**
 * @brief Defines callbacks for the given CAN TX mailbox.
 *
 * @param mailbox The mailbox number.
 */
#define CAN_TX_MAILBOX_CALLBACK(mailbox) \
    void HAL_CAN_TxMailbox ## mailbox ## CompleteCallback( \
        CAN_HandleTypeDef *handle \
    ) { \
        cmr_canTXCpltCallback(handle, mailbox); \
    } \
    \
    void HAL_CAN_TxMailbox ## mailbox ## AbortCallback( \
        CAN_HandleTypeDef *handle \
    ) { \
        /* Treat abort as complete. */ \
        cmr_canTXCpltCallback(handle, mailbox); \
    }
CAN_TX_MAILBOX_CALLBACK(0)
CAN_TX_MAILBOX_CALLBACK(1)
CAN_TX_MAILBOX_CALLBACK(2)
#undef CAN_TX_MAILBOX_CALLBACK

/**
 * @brief HAL CAN error callback.
 *
 * @warning Called from an interrupt handler!
 * @warning The handle must have been configured through this library!
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *handle) {
    cmr_can_t *can = cmr_canFromHandle(handle);

    uint32_t error = handle->ErrorCode;
    if (error & (
            HAL_CAN_ERROR_TX_ALST0 |
            HAL_CAN_ERROR_TX_ALST1 |
            HAL_CAN_ERROR_TX_ALST2 |
            HAL_CAN_ERROR_TX_TERR0 |
            HAL_CAN_ERROR_TX_TERR1 |
            HAL_CAN_ERROR_TX_TERR2
    )) {
        // Transmit error; drop semaphore.
        BaseType_t higherWoken;
        if (xSemaphoreGiveFromISR(can->txSem, &higherWoken) != pdTRUE) {
            cmr_panic("TX semaphore released too many times!");
        }
        portYIELD_FROM_ISR(higherWoken);
    }

    // Clear errors.
    handle->ErrorCode = 0;
}

/**
 * @brief Checks if a timeout has occurred.
 *
 * @param lastReceived_ms Last receive timestamp, in milliseconds.
 * @param threshold_ms Threshold period, in milliseconds.
 * @param now_ms Current timestamp, in milliseconds.
 *
 * @return A negative value if a timeout has occurred; otherwise 0.
 */
static int cmr_canTimeout(
    TickType_t lastReceived_ms, TickType_t threshold_ms, TickType_t now_ms
) {
    TickType_t release_ms = lastReceived_ms + threshold_ms;

    if (now_ms < lastReceived_ms && release_ms <= lastReceived_ms) {
        // Current time overflowed; release did not. Timeout!
        return -1;
    }

    if (lastReceived_ms <= now_ms && release_ms < lastReceived_ms) {
        // Current time did not overflow; release time did. No timeout.
        return 0;
    }

    // Neither current nor release overflowed, or both have.
    // In either case, release less than current indicates timeout.
    if (release_ms < now_ms) {
        return -1;
    }

    return 0;
}

/**
 * @brief Checks if the given periodic message has a timeout warning.
 *
 * @param meta The periodic message's reception metadata.
 * @param now_ms Current timestamp, in milliseconds.
 *
 * @return A negative value if a timeout warning has occurred; otherwise 0.
 */
int cmr_canRXMetaTimeoutWarn(const cmr_canRXMeta_t *meta, TickType_t now_ms) {
    return cmr_canTimeout(
        meta->lastReceived_ms, meta->timeoutWarn_ms, now_ms
    );
}

/**
 * @brief Checks if the given periodic message has a timeout error.
 *
 * @param meta The periodic message's reception metadata.
 * @param now_ms Current timestamp, in milliseconds.
 *
 * @return A negative value if a timeout error has occurred; otherwise 0.
 */
int cmr_canRXMetaTimeoutError(const cmr_canRXMeta_t *meta, TickType_t now_ms) {
    return cmr_canTimeout(
        meta->lastReceived_ms, meta->timeoutError_ms, now_ms
    );
}

/**
 * @brief Searches for the receive metadata associated with the given CAN ID.
 *
 * @warning May be called from an interrupt handler!
 *
 * @param can The interface.
 * @param canID The CAN ID to search for.
 *
 * @return The associated receive metadata, or `NULL` if not found.
 */
static cmr_canRXMeta_t *cmr_canRXMetaFind(cmr_can_t *can, uint16_t canID) {
    for (size_t i = 0; i < can->rxMetaLen; i++) {
        cmr_canRXMeta_t *meta = can->rxMeta + i;
        if (meta->canID == canID) {
            return meta;
        }
    }

    return NULL;    // No matching metadata.
}

/**
 * @brief Callback for reading CAN data into structs.
 *
 * @warning Called from an interrupt handler!
 *
 * @param can The interface.
 * @param canID The received message's CAN ID.
 * @param data The received data.
 * @param dataLen The received data's length.
 */
static void cmr_canRXData(
    cmr_can_t *can, uint16_t canID, const void *data, size_t dataLen
) {
    cmr_canRXMeta_t *meta = cmr_canRXMetaFind(can, canID);
    if (meta == NULL) {
        // Not a configured message; attempt to use the callback.
        if (can->rxCallback) {
            can->rxCallback(can, canID, data, dataLen);
        }

        return;
    }

    memcpy((void *) meta->payload, data, dataLen);
    meta->lastReceived_ms = xTaskGetTickCountFromISR();
}

/**
 * @brief Callback for CAN receive FIFO message pending.
 *
 * @warning Called from an interrupt handler!
 * @warning The handle must have been configured through this library!
 *
 * @param handle The HAL CAN handle.
 * @param fifo The pending FIFO.
 */
static void cmr_canRXPendingCallback(CAN_HandleTypeDef *handle, uint32_t fifo) {
    CAN_RxHeaderTypeDef msg;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(handle, fifo, &msg, data) != HAL_OK) {
        return;
    }

    cmr_can_t *can = cmr_canFromHandle(handle);
    cmr_canRXData(can, msg.StdId, data, msg.DLC);
}

/**
 * @brief Defines the message pending callback for the given receive FIFO.
 *
 * @param fifo The fifo number.
 */
#define CAN_RX_FIFO_PENDING(fifo) \
    void HAL_CAN_RxFifo ## fifo ## MsgPendingCallback( \
        CAN_HandleTypeDef *handle \
    ) { \
        cmr_canRXPendingCallback(handle, CAN_RX_FIFO ## fifo); \
    }
CAN_RX_FIFO_PENDING(0)
CAN_RX_FIFO_PENDING(1)
#undef CAN_RX_FIFO_PENDING

/**
 * @brief Initializes a CAN interface.
 *
 * @warning It is undefined behavior to initialize the same HAL CAN instance
 * more than once!
 * @warning This driver assumes a 48 MHz APB1 peripheral clock frequency!
 *
 * @param can The interface to initialize.
 * @param instance The HAL CAN instance (`CANx` from `stm32f413xx.h`).
 * @param bitRate The CAN bit rate to use.
 * @param rxMeta Metadata for periodic messages to receive.
 * @param rxMetaLen Number of periodic receive messages.
 * @param rxCallback Callback for other messages received, or `NULL` to ignore.
 * @param rxPort Receiving GPIO port (`GPIOx` from `stm32f413xx.h`).
 * @param rxPin Receiving GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`).
 * @param txPort Transmitting GPIO port.
 * @param txPin Transmitting GPIO pin.
 */
void cmr_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
    cmr_canBitRate_t bitRate,
    cmr_canRXMeta_t *rxMeta, size_t rxMetaLen,
    cmr_canRXCallback_t rxCallback,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin
) {
    /* Do any platform-specific initialization */
    _platform_canInit(
        can, instance,
        bitRate,
        rxMeta, rxMetaLen,
        rxCallback,
        rxPort, rxPin,
        txPort, txPin
    );

    cmr_rccCANClockEnable(instance);
    cmr_rccGPIOClockEnable(rxPort);
    cmr_rccGPIOClockEnable(txPort);

    // Configure CAN RX pin.
    GPIO_InitTypeDef pinConfig = {
        .Pin = rxPin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = cmr_canGPIOAF(instance, rxPort)
    };
    HAL_GPIO_Init(rxPort, &pinConfig);

    // Configure CAN TX pin.
    pinConfig.Pin = txPin;
    pinConfig.Alternate = cmr_canGPIOAF(instance, txPort);
    HAL_GPIO_Init(txPort, &pinConfig);

    if (HAL_CAN_Init(&can->handle) != HAL_OK) {
        cmr_panic("HAL_CAN_Init() failed!");
    }

    if (HAL_CAN_Start(&can->handle) != HAL_OK) {
        cmr_panic("HAL_CAN_Start() failed!");
    }

    if (HAL_CAN_ActivateNotification(
            &can->handle,
            CAN_IT_TX_MAILBOX_EMPTY |
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_RX_FIFO1_MSG_PENDING |
            CAN_IT_ERROR_WARNING |
            CAN_IT_ERROR_PASSIVE |
            CAN_IT_BUSOFF |
            CAN_IT_LAST_ERROR_CODE |
            CAN_IT_ERROR
    )) {
        cmr_panic("HAL_CAN_ActivateNotification() failed!");
    }
}

/**
 * @brief Queues a CAN message for transmission.
 *
 * @param can The CAN interface to send on.
 * @param id The message's CAN ID.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 * @param timeout The timeout.
 *
 * @return 0 on success, or a negative error code on timeout.
 */
int cmr_canTX(
    cmr_can_t *can,
    uint16_t id, const void *data, uint8_t len,
    TickType_t timeout
) {
    CAN_TxHeaderTypeDef txHeader = {
        .StdId = id,
        .ExtId = 0,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = len,
        .TransmitGlobalTime = DISABLE
    };

    // Attempt to reserve a mailbox.
    BaseType_t result = xSemaphoreTake(can->txSem, timeout);
    if (result != pdTRUE) {
        return -1;
    }

    // Even though the interface for HAL_CAN_AddTxMessage() does not specify the
    // data as `const`, it does not touch the data. Oh well.
    uint32_t txMailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
        &can->handle, &txHeader, (void *) data, &txMailbox
    );
    if (status != HAL_OK) {
        cmr_panic("Semaphore was available, but no mailboxes were found!");
    }

    return 0;
}

/**
 * @brief Enables (sets) bit(s) in a CAN field.
 *
 * @note This is used for multi-byte bitfields that may need to be misaligned in
 * a CAN message struct.
 *
 * @param field The field to modify.
 * @param value The value; each byte will be bitwise-OR'd with the corresponding
 * byte in the field.
 * @param len The length of the field/value, in bytes.
 */
void cmr_canFieldEnable(uint8_t *field, const void *value, size_t len) {
    for (size_t i = 0; i < len; i++) {
        field[i] |= ((const uint8_t *) value)[i];
    }
}

/**
 * @brief Disables (clears) bit(s) in a CAN field.
 *
 * @note This is used for multi-byte bitfields that may need to be misaligned in
 * a CAN message struct.
 *
 * @param field The field to modify.
 * @param value The value; each byte will be bitwise-negated, then bitwise-AND'd
 * with the corresponding byte in the field.
 * @param len The length of the field/value, in bytes.
 */
void cmr_canFieldDisable(uint8_t *field, const void *value, size_t len) {
    for (size_t i = 0; i < len; i++) {
        field[i] &= ~((const uint8_t *) value)[i];
    }
}

/**
 * @brief Configures a filter bank with 4 CAN IDs to filter.
 *
 * @param can The CAN interface to configure.
 * @param filters The filter configuration(s).
 * @param filtersLen The number of filters. Must be less than
 * `CMR_CAN_FILTERBANKS`.
 */
void cmr_canFilter(
    cmr_can_t *can, const cmr_canFilter_t *filters, size_t filtersLen
) {
    _platform_canFilter(can, filters, filtersLen);
}

/**
 * @brief Determines the GPIO alternate function for the given CAN interface.
 *
 * @param can The CAN interface.
 * @param port The GPIO port.
 *
 * @return The GPIO alternate function.
 */
uint32_t cmr_canGPIOAF(CAN_TypeDef *instance, GPIO_TypeDef *port) {
    return _platform_canGPIOAF(instance, port);
}

#endif /* HAL_CAN_MODULE_ENABLED */

