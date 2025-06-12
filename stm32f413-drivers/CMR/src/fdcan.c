/**
 * @file can.c
 * @brief Controller Area Network wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/fdcan.h>    // Interface to implement

#ifdef HAL_FDCAN_MODULE_ENABLED

#include <string.h> // memcpy()
#include <stdbool.h> // bool

#include <CMR/rcc.h>   // cmr_rccCANClockEnable(), cmr_rccGPIOClockEnable()
#include <CMR/panic.h>  // cmr_panic()
#include <CMR/h725.h> // cmr_


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

    // If no message has been received yet, default to throwing a timeout
    if (lastReceived_ms == 0) {
    	return -1;
    }

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
    //
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
    // shifted right because of macro definitions
    uint8_t len = dataLen >> 16;
    memcpy((void *) meta->payload, data, len);
    meta->lastReceived_ms = xTaskGetTickCountFromISR();
}




#ifdef H725

/**
 * @brief Gets the corresponding CAN interface from the HAL handle.
 *
 * @warning The handle must have been configured through this library!
 *
 * @param handle The handle.
 *
 * @return The interface.
 */
static cmr_can_t *cmr_canFromHandle(FDCAN_HandleTypeDef *handle) {
    char *addr = (void *) handle;
    return (void *) (addr - offsetof(cmr_can_t, handle));
}



void cmr_FDcanInit(
    cmr_can_t *can, FDCAN_GlobalTypeDef *instance,
    cmr_canBitRate_t bitRate,
    cmr_canRXMeta_t *rxMeta, size_t rxMetaLen,
    cmr_canRXCallback_t rxCallback,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin
) {
    /* Do any platform-specific initialization */
	_platform_FDCANInit(
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

    if (HAL_FDCAN_Init(&can->handle) != HAL_OK) {
        cmr_panic("HAL_FDCAN_Init() failed!");
    }

    if (HAL_FDCAN_Start(&can->handle) != HAL_OK) {
        cmr_panic("HAL_FDCAN_Start() failed!");
    }

    if (HAL_FDCAN_ActivateNotification(
            &can->handle,
			FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
			FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
			FDCAN_IT_ERROR_WARNING |
			FDCAN_IT_ERROR_PASSIVE |
			FDCAN_IT_BUS_OFF,
			0
    )) {
        cmr_panic("HAL_FDCAN_ActivateNotification() failed!");
    }
}


static void cmr_canRXPendingCallback(FDCAN_HandleTypeDef *handle, uint32_t fifo) {
	FDCAN_RxHeaderTypeDef msg;
    uint8_t data[8];
    if (HAL_FDCAN_GetRxMessage(handle, fifo, &msg, data) != HAL_OK) {
        return;
    }

    cmr_can_t *can = cmr_canFromHandle(handle);

    // shift by 16 because datalength lives in upper 16 bits of msg.DataLength
    cmr_canRXData(can, msg.Identifier, data, msg.DataLength);
}

#define CAN_RX_FIFO_PENDING(fifo) \
    void HAL_FDCAN_RxFifo ## fifo ## Callback( \
        FDCAN_HandleTypeDef *handle, uint32_t RxFifo ## fifo ## ITs \
    ) { \
        cmr_canRXPendingCallback(handle, FDCAN_RX_FIFO ## fifo); \
    }
CAN_RX_FIFO_PENDING(0)
CAN_RX_FIFO_PENDING(1)
#undef CAN_RX_FIFO_PENDING


int cmr_canTX(
    cmr_can_t *can,
    uint16_t id, const void *data, uint8_t len,
    TickType_t timeout
) {
	FDCAN_TxHeaderTypeDef txHeader = {
        .Identifier = id,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = len, // Doesn't get shifted by 16
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = FDCAN_BRS_OFF,
		.FDFormat = FDCAN_CLASSIC_CAN,
		.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
		.MessageMarker = 0
    };

    BaseType_t result = xSemaphoreTake(can->txSem, timeout);
    if (result != pdTRUE) {
        return -1;
    }
    if(HAL_FDCAN_GetTxFifoFreeLevel(&can->handle) == 0) {
    	FDCAN_ProtocolStatusTypeDef ProtocolStatus;
    	HAL_FDCAN_GetProtocolStatus(&can->handle, &ProtocolStatus);

    	result = xSemaphoreGive(can->txSem);
    	return -1;
    }

    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(
        &can->handle, &txHeader, (void *) data
    );

	FDCAN_ProtocolStatusTypeDef ProtocolStatus;
	HAL_FDCAN_GetProtocolStatus(&can->handle, &ProtocolStatus);

    if (status != HAL_OK) {
    	FDCAN_ProtocolStatusTypeDef ProtocolStatus;
    	HAL_FDCAN_GetProtocolStatus(&can->handle, &ProtocolStatus);
        cmr_panic("FDCAN Tx Failed!!");
    }
    result = xSemaphoreGive(can->txSem);
    if (result != pdTRUE) {
    	return -1;
    }

    return 0;
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *handle) {
	uint32_t error = handle->ErrorCode;

	if (error & (
			HAL_FDCAN_ERROR_RAM_ACCESS
	    )) {


	    }
	//clear error lol
	//handle->ErrorCode = 0;

}

/**
 * @brief Determines the GPIO alternate function for the given CAN interface.
 *
 * @param can The CAN interface.
 * @param port The GPIO port.
 *
 * @return The GPIO alternate function.
 */
uint32_t cmr_canGPIOAF(FDCAN_GlobalTypeDef *instance, GPIO_TypeDef *port) {
    return _platform_FDcanGPIOAF(instance, port);
}



#else

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
    uint16_t id, const void *data, size_t len,
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

    BaseType_t result = xSemaphoreTake(can->txSem, timeout);
	if (result != pdTRUE) {
		return -1;
	}

	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(
		&can->handle, &txHeader, (void *) data
	);
	if (status != HAL_OK) {
		cmr_panic("FDCAN Tx Failed!!");
	}
	result = xSemaphoreGive(can->txSem);
	if (result != pdTRUE) {
		return -1;
	}

	return 0;
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



#endif

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




#endif /* HAL_CAN_MODULE_ENABLED */