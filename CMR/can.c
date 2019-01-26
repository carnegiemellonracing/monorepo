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
void cmr_canInit(cmr_can_t *can, CAN_TypeDef *instance,
                 GPIO_TypeDef *rxPort, uint16_t rxPin,
                 GPIO_TypeDef *txPort, uint16_t txPin) {
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
 * @param filterBank The filter bank number to configure. Must be less than
 * `CMR_CAN_FILTERBANKS`.
 * @param rxFIFO The CAN receive FIFO to configure (one of `CAN_RX_FIFOx` from
 * `stm32f4xx_hal_can.h`).
 * @param canID1 Filter ID #1.
 * @param canID2 Filter ID #2.
 * @param canID3 Filter ID #3.
 * @param canID4 Filter ID #4.
 */
void cmr_canFilter(cmr_can_t *can, uint32_t filterBank, uint32_t rxFIFO,
                   uint16_t canID1, uint16_t canID2,
                   uint16_t canID3, uint16_t canID4) {
    if (filterBank >= CMR_CAN_FILTERBANKS) {
        panic("Invalid filter bank!");
    }

    switch (can->handle.Instance) {
        case CAN2:
            // CAN2 uses banks 14-27.
            bank += CMR_CAN_FILTERBANKS;
            break;
    }

    // In 16 bit ID list mode, FilterIdHigh, FilterIdLow, FilterMaskIdHigh, and
    // FilterMaskIdLow all serve as a whitelist of left-aligned 11 bit CAN IDs.
    // See RM0430 32.7.4 Fig. 387.
    const uint16_t CMR_CAN_ID_FILTER_SHIFT = 5;
    CAN_FilterTypeDef filter = {
        .FilterIdHigh           = canID1 << CMR_CAN_ID_FILTER_SHIFT,
        .FilterIdLow            = canID2 << CMR_CAN_ID_FILTER_SHIFT,
        .FilterMaskIdHigh       = canID3 << CMR_CAN_ID_FILTER_SHIFT,
        .FilterMaskIdLow        = canID4 << CMR_CAN_ID_FILTER_SHIFT,
        .FilterFIFOAssignment   = fifo,
        .FilterBank             = filterBank,
        .FilterMode             = CAN_FILTERMODE_IDLIST,
        .FilterScale            = CAN_FILTERSCALE_16BIT,
        .FilterActivation       = ENABLE,
        .SlaveStartFilterBank   = can2StartFilterBank,
    };

    if (HAL_CAN_ConfigFilter(&can2Handle, &filter) != HAL_OK) {
        panic("HAL_CAN_ConfigFilter() failed!");
    }
}

#endif /* HAL_CAN_MODULE_ENABLED */

