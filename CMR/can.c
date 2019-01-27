/**
 * @file can.c
 * @brief Controller Area Network wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "can.h"    // Interface to implement
#include "panic.h"  // cmr_panic()

#ifdef HAL_CAN_MODULE_ENABLED

/** @brief Number of CAN filter banks allocated for each interface. */
static const uint32_t CMR_CAN_FILTERBANKS = 14;

/**
 * @brief Initializes a CAN interface.
 *
 * @warning It is undefined behavior to initialize the same HAL CAN instance
 * more than once!
 *
 * @param can The interface to initialize.
 * @param instance The HAL CAN instance (`CANx` from `stm32f413xx.h`).
 * @param rxPort Receiving GPIO port (`GPIOx` from `stm32f413xx.h`).
 * @param rxPin Receiving GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`).
 * @param txPort Transmitting GPIO port.
 * @param txPin Transmitting GPIO pin.
 */
void cmr_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
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
        }
    };

    if (HAL_CAN_Init(&can->handle) != HAL_OK) {
        cmr_panic("HAL_CAN_Init() failed!");
    }

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

    if (HAL_CAN_Start(&can->handle) != HAL_OK) {
        cmr_panic("HAL_CAN_Start() failed!");
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
    if (filtersLen >= CMR_CAN_FILTERBANKS) {
        panic("Too many filter banks!");
    }

    uint32_t bank = 0;

    switch (can->handle.Instance) {
        case CAN2:
            // CAN2 uses banks 14-27.
            bank += CMR_CAN_FILTERBANKS;
            break;
    }

    while (bank < filtersLen) {
        const cmr_canFilter_t *filter = filters[bank];

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
            .FilterFIFOAssignment   = fifo,
            .FilterBank             = bank,
            .FilterMode             = CAN_FILTERMODE_IDLIST,
            .FilterScale            = CAN_FILTERSCALE_16BIT,
            .FilterActivation       = ENABLE,
            .SlaveStartFilterBank   = CMR_CAN_FILTERBANKS
        };

        if (HAL_CAN_ConfigFilter(&can->handle, &config) != HAL_OK) {
            panic("HAL_CAN_ConfigFilter() failed!");
        }
    }
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

