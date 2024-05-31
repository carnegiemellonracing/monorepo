/**
 * @file can.c
 * @brief Board-specific CAN implementation.
 *
 * Adding a new periodic message struct:
 *
 * 1. Add the corresponding index to the `canRX_t` enum in `can.h`.
 * 2. Add a configuration entry in `canRXMeta` at that index.
 * 3. Access the message using `canRXMeta[index]`.
 *
 * @author Carnegie Mellon Racing
 */

#include <string.h>     // memcpy()

#include <CMR/tasks.h>  // Task interface

#include "can.h"        // Interface to implement
#include "parser.h"     // parser ingestation

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[CANRX_LEN] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_VSM_SENSORS] = {
        .canID = CMR_CANID_VSM_STATUS,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_HVC_VOLTAGE] = {
        .canID = CMR_CANID_HVC_PACK_VOLTAGE,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_HVC_TEMPS] = {
        .canID = CMR_CANID_HVC_MINMAX_CELL_TEMPS,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_SBG_VELOCITY] = {
        .canID = 0,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_SBG_ACCELERATION] = {
        .canID = 0,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_SBG_GPS_POS] = {
        .canID = 0,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
    [CANRX_PTC_TEMPS] = {
        .canID = CMR_CANID_PTC_LOOP_TEMPS_A,
        .timeoutError_ms = -1,
        .timeoutWarn_ms = -1
    },
};

/** @brief CAN interfaces */
static cmr_can_t can[CMR_CAN_BUS_NUM];

static void canRX(
    cmr_can_t *canb_rx, uint16_t canID, const void *data, size_t dataLen
) {
	if(canID == CMR_CANID_AMK_1_SETPOINTS) {
		int x = 0;
	}
    size_t iface_idx = (canb_rx - can);
    configASSERT(iface_idx < CMR_CAN_BUS_NUM);
    int ret = parseData((uint32_t) iface_idx, canID, data, dataLen);
    configASSERT(ret == 0);
    (void) ret;
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // VEH-CAN (CAN3) initialization.
    cmr_canInit(
        &can[CMR_CAN_BUS_VEH], CAN3,
        CMR_CAN_BITRATE_500K,
        NULL, 0,
        canRX,
        GPIOA, GPIO_PIN_8,     // CAN3 RX port/pin.
        GPIOB, GPIO_PIN_4      // CAN3 TX port/pin.
    );

    // DAQ-CAN (CAN2) initialization.
    cmr_canInit(
        &can[CMR_CAN_BUS_DAQ], CAN2,
        CMR_CAN_BITRATE_500K,
        NULL, 0,
        canRX,
        GPIOB, GPIO_PIN_12,    // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_13     // CAN2 TX port/pin.
    );
    // Trac-CAN (CAN1) initialization.
	cmr_canInit(
		&can[CMR_CAN_BUS_TRAC], CAN1,
		CMR_CAN_BITRATE_500K,
		NULL, 0,
		canRX,
		GPIOB, GPIO_PIN_8,    // CAN1 RX port/pin.
		GPIOB, GPIO_PIN_9     // CAN1 TX port/pin.
	);

    // filters.
    const cmr_canFilter_t canFilters[] = {
        {
            .isMask = true,
            .rxFIFO = CAN_RX_FIFO0,

            // Match all even IDs (bottom bit 0, all others don't care).
            .ids = {
                0x000, 0x000,
                0x001, 0x001
            }
        }, {
            .isMask = true,
            .rxFIFO = CAN_RX_FIFO1,

            // Match all odd IDs (bottom bit 1, all others don't care).
            .ids = {
                0x001, 0x001,
                0x001, 0x001
            }
        }
    };

    cmr_canFilter(
        &can[CMR_CAN_BUS_VEH], canFilters, sizeof(canFilters) / sizeof(canFilters[0])
    );

    cmr_canFilter(
        &can[CMR_CAN_BUS_DAQ], canFilters, sizeof(canFilters) / sizeof(canFilters[0])
    );
    cmr_canFilter(
		&can[CMR_CAN_BUS_TRAC], canFilters, sizeof(canFilters) / sizeof(canFilters[0])
	);
}

/**
 * @brief Sends a CAN message with the given ID.
 *
 * @param id The bus to send on.
 * @param id The ID for the message.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 * @param timeout_ms The timeout, in ms.
 *
 * @return 0 on success, or a negative error code on timeout.
 */
int canTX(
    cmr_canBusID_t bus_id, cmr_canID_t id,
    const void *data, size_t len,
    TickType_t timeout_ms
) {
    configASSERT(bus_id < CMR_CAN_BUS_NUM);
    return cmr_canTX(&can[bus_id], id, data, len, timeout_ms);
}

/**
 * @brief Gets a pointer to the payload of a received CAN message.
 *
 * @param rxMsg The message to get the payload of.
 *
 * @return Pointer to payload, or NULL if rxMsg is invalid.
 */
void *getPayload(canRX_t rxMsg) {
    configASSERT(rxMsg < CANRX_LEN);

    cmr_canRXMeta_t *rxMeta = &(canRXMeta[rxMsg]);

    return (void *)(&rxMeta->payload);
}
