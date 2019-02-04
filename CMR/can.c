/**
 * @file can.c
 * @brief Controller Area Network wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "can.h"    // Interface to implement

#ifdef HAL_CAN_MODULE_ENABLED

#include <task.h>       // Task interface

#include "rcc.h"    // cmr_rccCANClockEnable(), cmr_rccGPIOClockEnable()
#include "panic.h"  // cmr_panic()

/** @brief Number of CAN filter banks allocated for each interface. */
static const uint32_t CMR_CAN_FILTERBANKS = 14;

/** @brief The metadata for possible structs to receive over CAN. */
static cmr_rxMetadata_t *cmr_rxMetadataArray;

/** @brief The number of possible structs to receive over CAN. */
static size_t cmr_rxMetadataArrayLength;

/** @brief The index representing no payload corresponding to the provided CAN ID. */
static const size_t NO_PAYLOAD = SIZE_MAX;

/** @brief CAN RX priority. */
static const uint32_t cmr_canRXPriority  = 7;

/** @brief CAN RX period (milliseconds). */
static const TickType_t cmr_canRXPeriod_ms = 1;

/**
 * @brief Task for receiving CAN messages (polling RX FIFOs).
 *
 * @param pvParameters The associated CAN interface.
 *
 * @return Does not return.
 */
static void cmr_canRX_task(void *pvParameters) {
    cmr_can_t *can = pvParameters;
    CAN_HandleTypeDef *canHandle = &can->handle;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        CAN_RxHeaderTypeDef rxHeader = { 0 };
        uint8_t rxData[8];

        // Receive on FIFO 0.
        while (HAL_CAN_GetRxFifoFillLevel(canHandle, CAN_RX_FIFO0) > 0) {
            HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &rxHeader, rxData);
            cmr_readRXData((uint16_t) rxHeader.StdId, rxData, rxHeader.DLC);
        }

        // Receive on FIFO 1.
        while (HAL_CAN_GetRxFifoFillLevel(canHandle, CAN_RX_FIFO1) > 0) {
            HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO1, &rxHeader, rxData);
            cmr_readRXData((uint16_t) rxHeader.StdId, rxData, rxHeader.DLC);
        }

        vTaskDelayUntil(&lastWakeTime, cmr_canRXPeriod_ms);
    }
}

/**
 * @brief Callback for reading CAN data into structs.
 *
 * @param id The received message's CAN ID.
 * @param data The received data.
 * @param len The received data's length.
 */
static void cmr_readRXData(uint16_t id, const void *data, size_t len) {
    size_t payloadIndex = cmr_getPayloadIndex(id);

    if (payloadIndex == NO_PAYLOAD) {
        // No payload found to receive this message, toss it
        return;
    }

    // Payload was found for this message ID, process it
    void *payloadAddress = cmr_getPayloadStructAddress(payloadIndex);
    size_t payloadSize = cmr_getPayloadStructSize(payloadIndex);

    if (payloadSize != len) {
        // Packet data length did not match expected payload size
        return;
    }

    // Copy packet data into payload struct
    memcpy(payloadAddress, data, payloadSize);

    // Reset the timeout for this payload
    cmr_resetTimeout(payloadIndex);
}

/**
 * @brief Initializes a CAN interface.
 *
 * @warning It is undefined behavior to initialize the same HAL CAN instance
 * more than once!
 *
 * @param can The interface to initialize.
 * @param instance The HAL CAN instance (`CANx` from `stm32f413xx.h`).
 * @param rxTaskName Receive task name for this interface.
 * @param rxMetadataArray The metadata for structs received over CAN.
 * @param rxMetadataArrayLength The number of possible structs to receive over CAN
 * @param rxPort Receiving GPIO port (`GPIOx` from `stm32f413xx.h`).
 * @param rxPin Receiving GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`).
 * @param txPort Transmitting GPIO port.
 * @param txPin Transmitting GPIO pin.
 */
void cmr_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
    const char *rxTaskName,
	const cmr_rxMetadata_t *rxMetadataArray,
	size_t rxMetadataArrayLength,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin
) {
	cmr_rxMetadataArray = rxMetadataArray;
	cmr_rxMetadataArrayLength = rxMetadataArrayLength;
    *can = (cmr_can_t) {
        .handle = {
            .Instance = instance,
            .Init = {
                .Prescaler = 12,
                .Mode = CAN_MODE_NORMAL,
                .SyncJumpWidth = CAN_SJW_2TQ,
                .TimeSeg1 = CAN_BS1_6TQ,
                .TimeSeg2 = CAN_BS2_1TQ,
                .TimeTriggeredMode = DISABLE,
                .AutoBusOff = DISABLE,
                .AutoWakeUp = DISABLE,
                .AutoRetransmission = DISABLE,
                .ReceiveFifoLocked = DISABLE,
                .TransmitFifoPriority = DISABLE
            }
        }
    };

    can->txMutex = xSemaphoreCreateMutex();
    configASSERT(can->txMutex != NULL);

    cmr_rccCANClockEnable(instance);
    cmr_rccGPIOClockEnable(rxPort);
    cmr_rccGPIOClockEnable(txPort);

    // Configure CAN RX pin.
    GPIO_InitTypeDef pinConfig = {
        .Pin = rxPin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF9_CAN2,
    };
    HAL_GPIO_Init(rxPort, &pinConfig);

    // Configure CAN TX pin.
    pinConfig.Pin = txPin;
    HAL_GPIO_Init(txPort, &pinConfig);

    if (HAL_CAN_Init(&can->handle) != HAL_OK) {
        cmr_panic("HAL_CAN_Init() failed!");
    }

    if (HAL_CAN_Start(&can->handle) != HAL_OK) {
        cmr_panic("HAL_CAN_Start() failed!");
    }

    // Create receive task.
    xTaskCreate(
        cmr_canRX_task, rxTaskName,
        configMINIMAL_STACK_SIZE, can,
        cmr_canRXPriority, NULL
    );
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
    if (filtersLen >= CMR_CAN_FILTERBANKS) {
        cmr_panic("Too many filter banks!");
    }

    CAN_TypeDef *instance = can->handle.Instance;

    for (size_t i = 0; i < filtersLen; i++) {
        const cmr_canFilter_t *filter = filters + i;

        uint32_t bank = i;
        if (instance == CAN2) {
            // CAN2 uses banks 14-27.
            bank += CMR_CAN_FILTERBANKS;
        }

        // In 16 bit ID list mode, FilterIdHigh, FilterIdLow, FilterMaskIdHigh,
        // and FilterMaskIdLow all serve as a whitelist of left-aligned 11-bit
        // CAN IDs.
        // See RM0430 32.7.4 Fig. 387.
        const uint16_t CMR_CAN_ID_FILTER_SHIFT = 5;
        CAN_FilterTypeDef config = {
            .FilterIdHigh           = filter->ids[0] << CMR_CAN_ID_FILTER_SHIFT,
            .FilterIdLow            = filter->ids[1] << CMR_CAN_ID_FILTER_SHIFT,
            .FilterMaskIdHigh       = filter->ids[2] << CMR_CAN_ID_FILTER_SHIFT,
            .FilterMaskIdLow        = filter->ids[3] << CMR_CAN_ID_FILTER_SHIFT,
            .FilterFIFOAssignment   = filter->rxFIFO,
            .FilterBank             = bank,
            .FilterMode             = CAN_FILTERMODE_IDLIST,
            .FilterScale            = CAN_FILTERSCALE_16BIT,
            .FilterActivation       = ENABLE,
            .SlaveStartFilterBank   = CMR_CAN_FILTERBANKS
        };

        if (HAL_CAN_ConfigFilter(&can->handle, &config) != HAL_OK) {
            cmr_panic("HAL_CAN_ConfigFilter() failed!");
        }
    }
}

/**
 * @brief Queues a CAN message for transmission.
 *
 * @param can The CAN interface to send on.
 * @param id The message's CAN ID.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 *
 * @return 0 on success, or a negative error code.
 */
int cmr_canTX(
    cmr_can_t *can, uint16_t id, const void *data, size_t len
) {
    CAN_TxHeaderTypeDef txHeader = {
        .StdId = id,
        .ExtId = 0,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = len,
        .TransmitGlobalTime = DISABLE
    };

    BaseType_t result = xSemaphoreTake(can->txMutex, portMAX_DELAY);
    if (result != pdTRUE) {
        cmr_panic("cmr_canTX() xSemaphoreTake() timed out!");
    }

    // Even though the interface for HAL_CAN_AddTxMessage() does not specify the
    // data as `const`, it does not touch the data. Oh well.
    uint32_t txMailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
        &can->handle, &txHeader, (void *) data, &txMailbox
    );
    if (status != HAL_OK) {
        return -1;
    }

    xSemaphoreGive(can->txMutex);

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
 * @brief Returns the index in cmr_rxMetadataArray corresponding to the given CAN ID.
 *
 * @param canID The CAN ID to return the corresponding index for.
 * @return The index of the given CAN ID in cmr_rxMetadataArray if it exists, else NO_PAYLOAD.
 */
static size_t cmr_getPayloadIndex(uint16_t canID) {

    // Search for a payload with the desired CAN ID
    for (size_t i = 0; i < cmr_rxMetadataArrayLength; i++) {
        if (cmr_rxMetadataArray[i].canID == canID) {
            return i;
        }
    }

    // No matching payload found
    return NO_PAYLOAD;
}

/**
 * @brief Returns the payload struct of the given payload index.
 *
 * @param payloadIndex the payload index to get the struct address of.
 * @return The pointer to the struct for the given payloadIndex.
 */
void *cmr_getPayloadStructAddress(size_t payloadIndex) {
    if (payloadIndex >= cmr_rxMetadataArrayLength) {
        return NULL;
    }

    return cmr_rxMetadataArray[payloadIndex].payloadStruct;
}

/**
 * @brief Returns the payload struct size of the given payload index.
 *
 * @param payloadIndex The payload index to get the struct size of.
 * @return The size of the struct for the given payloadIndex.
 */
size_t cmr_getPayloadStructSize(size_t payloadIndex) {
    if (payloadIndex >= cmr_rxMetadataArrayLength) {
        return NO_PAYLOAD;
    }

    return cmr_rxMetadataArray[payloadIndex].payloadSize;
}

/**
 * @brief Resets the timeout of the given payload index.
 *
 * @param payloadIndex The payload index to reset the timeout for.
 * @return The status of the payload of the payload index, or
 * CAN_PAYLOAD_STATUS_NO_PAYLOAD if the payloadIndex is not valid.
 */
cmr_canPayloadStatus_t cmr_resetTimeout(size_t payloadIndex) {

    if (payloadIndex >= cmr_rxMetadataArrayLength) {
        return CAN_PAYLOAD_STATUS_NO_PAYLOAD;
    }

    // Set message timestamp to current time
    cmr_rxMetadataArray[payloadIndex].lastReceived_ms = xTaskGetTickCount();

    return CAN_PAYLOAD_STATUS_OK;
}

/**
 * @brief Check whether or not a specific CAN message has reached the
 * timeout warning threshold.
 * @param payloadIndex The payload index to check the timeout warn status of.
 * @return CAN_PAYLOAD_STATUS_NO_TIMEOUT if the message has not reached its
 * timeout warn threshold, CAN_PAYLOAD_STATUS_TIMEOUT if it has, or
 * CAN_PAYLOAD_STATUS_NO_PAYLOAD if the payloadIndex is not valid.
 */
cmr_canPayloadStatus_t cmr_checkMessageTimeoutWarn(size_t payloadIndex) {
    if (payloadIndex >= cmr_rxMetadataArrayLength) {
        return CAN_PAYLOAD_STATUS_NO_PAYLOAD;
    }

    return checkTimeoutExpiry(cmr_rxMetadataArray[payloadIndex].lastReceived_ms,
    						 cmr_rxMetadataArray[payloadIndex].timeoutWarnThreshold_ms);
}

/**
 * @brief Check whether or not a specific CAN message has reached its timeout
 * error threshold.
 * @param payloadIndex The payload index to check the timeout error status of.
 * @return CAN_PAYLOAD_STATUS_NO_TIMEOUT if the message has not reached its
 * timeout error threshold, CAN_PAYLOAD_STATUS_TIMEOUT if it has, or
 * CAN_PAYLOAD_STATUS_NO_PAYLOAD if the payloadIndex is not valid.
 */
cmr_canPayloadStatus_t cmr_checkMessageTimeoutError(size_t payloadIndex) {
    if (payloadIndex >= cmr_rxMetadataArrayLength) {
        return CAN_PAYLOAD_STATUS_NO_PAYLOAD;
    }

    return checkTimeoutExpiry(cmr_rxMetadataArray[payloadIndex].lastReceived_ms,
    						 cmr_rxMetadataArray[payloadIndex].timeoutErrorThreshold_ms);
}

/**
 * @brief Check whether messages above or equal to a certain severity have
 * passed their error time out thresholds.
 * @param minTimeoutSeverity The minimum timeout severity to check for.
 * @return CAN_PAYLOAD_STATUS_NO_TIMEOUT to indicate no timeout,
 * CAN_PAYLOAD_STATUS_TIMEOUT if a timeout has occurred, or
 * CAN_PAYLOAD_STATUS_NO_PAYLOAD if the payloadIndex is not valid.
 */
cmr_canPayloadStatus_t cmr_checkSeverityTimeouts(uint8_t minTimeoutSeverity) {
    cmr_canPayloadStatus_t retv;
    // Search all payloads for timeouts above or equal to the specified severity
    for (size_t i = 0; i < cmr_rxMetadataArrayLength; i++) {
        if (cmr_rxMetadataArray[i].timeoutSeverity >= minTimeoutSeverity) {
            retv = canPayloads_checkMessageTimeout(i);
            if (retv == CAN_PAYLOAD_STATUS_TIMEOUT) {
                return CAN_PAYLOAD_STATUS_TIMEOUT;
            }
        }
    }

    return CAN_PAYLOAD_STATUS_NO_TIMEOUT;
}

/**
 * @brief Check to see if the timeout threshold has been passed.
 * @note Assumes multiple over flows will not occur since last run time.
 * @note Adapted from xTaskCheckForTimeOut but does not modify inputs.
 * @param lastReceived_ms The last time a message reset the timeout
 * @param timeoutThreshold_ms The threshold for a timeout.
 * @return CAN_PAYLOAD_STATUS_NO_TIMEOUT to indicate no timeout threshold
 * has been passed or CAN_PAYLOAD_STATUS_TIMEOUT otherwise.
 */
static cmr_canPayloadStatus_t cmr_checkTimeoutExpiry(TickType_t lastReceived_ms, TickType_t timeoutThreshold_ms) {
    cmr_canPayloadStatus_t retv;

    TickType_t currentTime_ms = xTaskGetTickCount();
    TickType_t releaseTime_ms = lastReceived_ms + timeoutThreshold_ms;

    if((currentTime_ms < lastReceived_ms) && !(releaseTime_ms < lastReceived_ms)) {
        // Current time has overflown but release time has not
        // Current time has thus exceed release time and timeout threshold has been passed.
        retv = CAN_PAYLOAD_STATUS_TIMEOUT;
    } else if(!(currentTime_ms < lastReceived_ms) &&  (releaseTime_ms < lastReceived_ms)) {
        // Current time has not overflown but release time has
        // Release time then still exceeds current time and no timeout has occurred
        retv = CAN_PAYLOAD_STATUS_NO_TIMEOUT;
    } else if (currentTime_ms > releaseTime_ms) {
        // Neither time has overflow or both have
        // In either case, current being greater than release indicates timeout threshold has been passed.
        retv = CAN_PAYLOAD_STATUS_TIMEOUT;
    } else {
        // Neither time has overflow or both have
        // In either case, current being less than release indicates no timeout threshold has been passed.
        retv = CAN_PAYLOAD_STATUS_NO_TIMEOUT;
    }

    return retv;
}

/**
 * @brief Sets the timeout severity of the given payload index.
 * @param payloadIndex The payload index to set the timeout severity of.
 * @param timeoutSeverity The new timeoutSeverity.
 */
void cmr_setTimeoutSeverity(size_t payloadIndex, cmr_timeoutSeverity_t timeoutSeverity) {
    if (payloadIndex >= cmr_rxMetadataArrayLength) {
        return;
    }

	cmr_rxMetadataArray[payloadIndex].timeoutSeverity = timeoutSeverity;
}

/**
 * @brief Sets the timeout warn threshold of the given payload index.
 * @param payloadIndex The payload index to set the warn threshold of.
 * @param warnThreshold_ms The new warnThreshold.
 */
void cmr_setTimeoutWarnThreshold(size_t payloadIndex, TickType_t warnThreshold_ms) {
    if (payloadIndex >= cmr_rxMetadataArrayLength) {
        return;
    }

	cmr_rxMetadataArray[payloadIndex].timeoutWarnThreshold_ms = warnThreshold_ms;
}

/**
 * @brief Sets the timeout error threshold of the given payload index.
 * @param payloadIndex The payload index to set the error threshold of.
 * @param errorThreshold_ms The new errorThreshold.
 */
void cmr_setTimeoutErrorThreshold(size_t payloadIndex, TickType_t errorThreshold_ms) {
    if (payloadIndex >= cmr_rxMetadataArrayLength) {
        return;
    }

	cmr_rxMetadataArray[payloadIndex].timeoutErrorThreshold_ms = errorThreshold_ms;
}

#endif /* HAL_CAN_MODULE_ENABLED */

