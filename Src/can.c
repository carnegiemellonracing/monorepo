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

#include "can.h"        // Interface to implement
#include "parser.h"     // parser ingestation


/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */

/** @brief Metadata for vehicle CAN message reception. */
cmr_canRXMeta_t canVehicleRXMeta[CANRX_VEH_LEN] = {
    [CANRX_VEH_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_VSM_TIMEOUT,
        .warnFlag = CMR_CAN_WARN_VSM_TIMEOUT
    },
    [CANRX_VEH_DATA_FSM] = {
        .canID = CMR_CANID_FSM_DATA,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_VEH_REQUEST_DIM] = {
        .canID = CMR_CANID_DIM_REQUEST,
        .timeoutError_ms = UINT32_MAX,
        .timeoutWarn_ms = UINT32_MAX,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_VEH_VOLTAGE_HVC] = {
        .canID = CMR_CANID_HVC_PACK_VOLTAGE,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_VEH_CURRENT_HVC] = {
        .canID = CMR_CANID_HVC_PACK_CURRENT,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_VEH_DIM_ACTION_BUTTON] = {
        .canID = CMR_CANID_DIM_ACTIONS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 50,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_VEH_PACK_CELL_VOLTAGE] = {
        .canID = CMR_CANID_HVC_MINMAX_CELL_VOLTAGE,
        // TODO: Check timeout period
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 50,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_VEH_PACK_CELL_TEMP] = {
        .canID = CMR_CANID_HVC_MINMAX_CELL_TEMPS,
        // TODO: Check timeout period
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 50,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_VEH_EMD_MEASURE] = {
        .canID = CMR_CANID_EMD_MEASUREMENT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 25
    },
    [CANRX_VEH_VSM_SENSORS] = {
        .canID = CMR_CANID_VSM_SENSORS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 50,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_RTC_SET] = {
        .canID = CMR_CANID_CDC_RTC_DATA_IN,
	    .timeoutError_ms = 1500,
	    .timeoutWarn_ms = 50,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    }
};

/** @brief Metadata for tractive CAN message reception. */
cmr_canRXMeta_t canTractiveRXMeta[CANRX_TRAC_LEN] = {
    [CANRX_TRAC_INV_FL_ACT1] = {
        .canID = CMR_CANID_AMK_1_ACT_1,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_AMK_FL | CMR_CAN_WARN_CDC_AMK_TIMEOUT
    },
    [CANRX_TRAC_INV_FL_ACT2] = {
        .canID = CMR_CANID_AMK_1_ACT_2,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_AMK_FL | CMR_CAN_WARN_CDC_AMK_TIMEOUT
    },
    [CANRX_TRAC_INV_FR_ACT1] = {
        .canID = CMR_CANID_AMK_3_ACT_1,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_AMK_FR | CMR_CAN_WARN_CDC_AMK_TIMEOUT
    },
    [CANRX_TRAC_INV_FR_ACT2] = {
        .canID = CMR_CANID_AMK_3_ACT_2,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_AMK_FR | CMR_CAN_WARN_CDC_AMK_TIMEOUT
    },
    [CANRX_TRAC_INV_RL_ACT1] = {
        .canID = CMR_CANID_AMK_4_ACT_1,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_AMK_RL | CMR_CAN_WARN_CDC_AMK_TIMEOUT
    },
    [CANRX_TRAC_INV_RL_ACT2] = {
        .canID = CMR_CANID_AMK_4_ACT_2,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_AMK_RL | CMR_CAN_WARN_CDC_AMK_TIMEOUT
    },
    [CANRX_TRAC_INV_RR_ACT1] = {
        .canID = CMR_CANID_AMK_2_ACT_1,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_AMK_RR | CMR_CAN_WARN_CDC_AMK_TIMEOUT
    },
    [CANRX_TRAC_INV_RR_ACT2] = {
        .canID = CMR_CANID_AMK_2_ACT_2,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_AMK_RR | CMR_CAN_WARN_CDC_AMK_TIMEOUT
    },
    [CANRX_TRAC_HVI_SENSE] = {
        .canID = CMR_CANID_HEARTBEAT_HVI,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75
    }
};

cmr_canRXMeta_t canDaqRXMeta[CANRX_DAQ_LEN] = {
    [CANRX_DAQ_SBG_STATUS_3] = {
        .canID = CMR_CANID_SBG_STATUS_3,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    [CANRX_DAQ_SBG_POS] = {
        .canID = CMR_CANID_SBG_EKF_POS,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    [CANRX_DAQ_SBG_VEL] = {
        .canID = CMR_CANID_SBG_EKF_VEL,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    [CANRX_DAQ_SBG_BODY_VEL] = {
        .canID = CMR_CANID_SBG_BODY_VEL,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    [CANRX_DAQ_SBG_ORIENT] = {
        .canID = CMR_CANID_SBG_EKF_ORIENT,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    [CANRX_DAQ_SBG_IMU_ACCEL] = {
        .canID = CMR_CANID_SBG_IMU_ACCEL,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    [CANRX_DAQ_SBG_IMU_GYRO] = {
        .canID = CMR_CANID_SBG_IMU_GYRO,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    [CANRX_DAQ_EMD_MEASURE] = {
        .canID = CMR_CANID_EMD_MEASUREMENT,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250
    },
    [CANRX_DAQ_SBG_SLIPANGLE] = {
	    .canID  = CMR_CANID_SBG_AUTOMOTIVE,
        .timeoutError_ms = 1000,
        .timeoutWarn_ms = 750,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
	},
	[CANRX_DAQ_LOAD_FL] = {
        .canID  = CMR_CANID_DAQ_3_LOADCELL,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
	},
	[CANRX_DAQ_LOAD_FR] = {
        .canID  = CMR_CANID_DAQ_0_LOADCELL,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
	},
	[CANRX_DAQ_LOAD_RL] = {
        .canID  = CMR_CANID_DAQ_2_LOADCELL,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
	},
	[CANRX_DAQ_LOAD_RR] = {
        .canID  = CMR_CANID_DAQ_1_LOADCELL,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
	},
    [CANRX_DAQ_LINPOTS_RIGHTS] = {
        .canID  = CMR_CANID_DAQ_0_THERMISTOR,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
	},
    [CANRX_DAQ_LINPOTS_LEFTS] = {
        .canID  = CMR_CANID_DAQ_3_THERMISTOR,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
	},
    [CANRX_DAQ_MEMORATOR_BROADCAST] = {
        .canID = CMR_CANID_HEARTBEAT_MEMORATOR,
        .timeoutError_ms = 5000,
        .timeoutWarn_ms = 3000
    }
};

/** @brief CAN interfaces */
static cmr_can_t can[CMR_CAN_BUS_NUM];

static void canRX(
    cmr_can_t *canb_rx, uint16_t canID, const void *data, size_t dataLen
) {
	size_t iface_idx = (canb_rx - can);
	configASSERT(iface_idx < CMR_CAN_BUS_NUM);


	int ret = parseData((uint32_t) iface_idx, canID, data, dataLen);
	configASSERT(ret == 0);

	// Update the RX Meta array
	cmr_canRXMeta_t *rxMetaArray = NULL;
	uint32_t rxMetaArrayLen = 0;
	if (iface_idx == CMR_CAN_BUS_VEH) {
		rxMetaArray = canVehicleRXMeta;
		rxMetaArrayLen = CANRX_VEH_LEN;
	} else if (iface_idx == CMR_CAN_BUS_DAQ) {
		rxMetaArray = canDaqRXMeta;
		rxMetaArrayLen = CANRX_DAQ_LEN;
	} else if (iface_idx == CMR_CAN_BUS_TRAC) {
		rxMetaArray = canTractiveRXMeta;
		rxMetaArrayLen = CANRX_TRAC_LEN;
	}
	if (rxMetaArray != NULL) {
		for (uint32_t i = 0; i < rxMetaArrayLen; i++) {
			if (rxMetaArray[i].canID == canID) {
				memcpy((void *) rxMetaArray[i].payload, data, dataLen);
				rxMetaArray[i].lastReceived_ms = xTaskGetTickCountFromISR();
				break;
			}
		}
	}
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

