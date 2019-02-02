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
static void cmr_canRXTask(void *pvParameters) {
    cmr_can_t *can = pvParameters;
    CAN_HandleTypeDef *canHandle = &can->handle;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        CAN_RxHeaderTypeDef rxHeader = { 0 };
        uint8_t rxData[8];

        // Receive on FIFO 0.
        while (HAL_CAN_GetRxFifoFillLevel(canHandle, CAN_RX_FIFO0) > 0) {
            HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &rxHeader, rxData);
            can->rxCallback((uint16_t) rxHeader.StdId, rxData, rxHeader.DLC);
        }

        // Receive on FIFO 1.
        while (HAL_CAN_GetRxFifoFillLevel(canHandle, CAN_RX_FIFO1) > 0) {
            HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO1, &rxHeader, rxData);
            can->rxCallback((uint16_t) rxHeader.StdId, rxData, rxHeader.DLC);
        }

        vTaskDelayUntil(&lastWakeTime, cmr_canRXPeriod_ms);
    }
}

/**
 * @brief Initializes a CAN interface.
 *
 * @warning It is undefined behavior to initialize the same HAL CAN instance
 * more than once!
 *
 * @param can The interface to initialize.
 * @param instance The HAL CAN instance (`CANx` from `stm32f413xx.h`).
 * @param rxCallback Callback for received messages on this interface.
 * @param rxTaskName Receive task name for this interface.
 * @param rxPort Receiving GPIO port (`GPIOx` from `stm32f413xx.h`).
 * @param rxPin Receiving GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`).
 * @param txPort Transmitting GPIO port.
 * @param txPin Transmitting GPIO pin.
 */
void cmr_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
    cmr_canRXCallback_t rxCallback, const char *rxTaskName,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin
) {
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
        },

        .rxCallback = rxCallback
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
        cmr_canRXTask, rxTaskName,
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

#endif /* HAL_CAN_MODULE_ENABLED */

