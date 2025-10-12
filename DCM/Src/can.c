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

#include <cmr_error.h>  // getErrors(), getWarnings()
#include <string.h>     // memcpy()
#include <math.h>

#include <CMR/tasks.h>      // Task interface
#include <CMR/can_types.h>      // Task interface
#include <CMR/config_screen_helper.h>

#include "can.h"    // Interface to implement
#include "adc.h"    // adcVSense, adcISense
#include "motors.h" // cmr_canDTISetpoints_t
#include "daq.h"
#include "i2c.h"
#include "drs_controls.h"
#include "controls_helper.h"
#include "controls.h"
#include "sensors.h"
#include "movella.h"
#include "safety_filter.h"

extern volatile uint8_t currentParameters[MAX_MENU_ITEMS];
volatile uint8_t parametersFromDIM[MAX_MENU_ITEMS];
volatile cmr_driver_profile_t currentDriver = Default;
volatile bool framWrite_flag = false;

extern volatile cmr_can_rtc_data_t time;
extern volatile float odometer_km;
extern bool isTorqueMode;

/** @brief Fan/Pump channel states. */
uint16_t fan_1_State;
uint16_t fan_2_State;
uint16_t pump_1_State;
uint16_t pump_2_State;

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

/** @brief Metadata for vehicle CAN message reception. */
cmr_canRXMeta_t canVehicleRXMeta[CANRX_VEH_LEN] = {
    [CANRX_VEH_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_VSM_TIMEOUT,
        .warnFlag = CMR_CAN_WARN_VSM_TIMEOUT
    },
    [CANRX_VSM_STATUS] = {
        .canID = CMR_CANID_VSM_STATUS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_VSM_TIMEOUT,
        .warnFlag = CMR_CAN_WARN_VSM_TIMEOUT,
    },
    [CANRX_VEH_DATA_FSM] = {
        .canID = CMR_CANID_FSM_DATA,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_VEH_SWANGLE_FSM] = {
        .canID = CMR_CANID_FSM_SWANGLE,
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
    },
	[CANRX_HVI_SENSE] = {
	        .canID = CMR_CANID_HEARTBEAT_HVI,
	        .timeoutError_ms = 100,
	        .timeoutWarn_ms = 75
	},
    // Temporary.
    [CANRX_VEH_MOVELLA_STATUS] = {
        .canID = CMR_CANID_MOVELLA_STATUS,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    // Temporary.
    [CANRX_VEH_MOVELLA_QUATERNION] = {
        .canID = CMR_CANID_MOVELLA_QUATERNION,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    // Temporary.
    [CANRX_VEH_MOVELLA_IMU_GYRO] = {
        .canID = CMR_CANID_MOVELLA_IMU_GYRO,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    // Temporary.
    [CANRX_VEH_MOVELLA_IMU_ACCEL] = {
        .canID = CMR_CANID_MOVELLA_IMU_ACCEL,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    // Temporary.
    [CANRX_VEH_MOVELLA_VELOCITY] = {
        .canID = CMR_CANID_MOVELLA_VELOCITY,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
};

/** @brief Metadata for tractive CAN message reception. */
cmr_canRXMeta_t canTractiveRXMeta[CANRX_TRAC_LEN] = {
    /* Front Left Inverter (Node ID 0x00) */
    [CANRX_TRAC_FL_CONTROL_STATUS] = {
        .canID = CMR_CANID_DTI_FL_CONTROL_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FL_ERPM] = {
        .canID = CMR_CANID_DTI_FL_ERPM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FL_CURRENT] = {
        .canID = CMR_CANID_DTI_FL_CURRENT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FL_TEMPFAULT] = {
        .canID = CMR_CANID_DTI_FL_TEMPFAULT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FL_IDIQ] = {
        .canID = CMR_CANID_DTI_FL_IDIQ,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FL_IO_STATUS] = {
        .canID = CMR_CANID_DTI_FL_IO_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FL_ACLIMS] = {
        .canID = CMR_CANID_DTI_FL_ACLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FL_DCLIMS] = {
        .canID = CMR_CANID_DTI_FL_DCLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },

    /* Front Right Inverter (Node ID 0x01) */
    [CANRX_TRAC_FR_CONTROL_STATUS] = {
        .canID = CMR_CANID_DTI_FR_CONTROL_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FR_ERPM] = {
        .canID = CMR_CANID_DTI_FR_ERPM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FR_CURRENT] = {
        .canID = CMR_CANID_DTI_FR_CURRENT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FR_TEMPFAULT] = {
        .canID = CMR_CANID_DTI_FR_TEMPFAULT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FR_IDIQ] = {
        .canID = CMR_CANID_DTI_FR_IDIQ,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FR_IO_STATUS] = {
        .canID = CMR_CANID_DTI_FR_IO_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FR_ACLIMS] = {
        .canID = CMR_CANID_DTI_FR_ACLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_FR_DCLIMS] = {
        .canID = CMR_CANID_DTI_FR_DCLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },

    /* Rear Right Inverter (Node ID 0x02) */
    [CANRX_TRAC_RR_CONTROL_STATUS] = {
        .canID = CMR_CANID_DTI_RR_CONTROL_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RR_ERPM] = {
        .canID = CMR_CANID_DTI_RR_ERPM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RR_CURRENT] = {
        .canID = CMR_CANID_DTI_RR_CURRENT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RR_TEMPFAULT] = {
        .canID = CMR_CANID_DTI_RR_TEMPFAULT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RR_IDIQ] = {
        .canID = CMR_CANID_DTI_RR_IDIQ,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RR_IO_STATUS] = {
        .canID = CMR_CANID_DTI_RR_IO_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RR_ACLIMS] = {
        .canID = CMR_CANID_DTI_RR_ACLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RR_DCLIMS] = {
        .canID = CMR_CANID_DTI_RR_DCLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },

    /* Rear Left Inverter (Node ID 0x03) */
    [CANRX_TRAC_RL_CONTROL_STATUS] = {
        .canID = CMR_CANID_DTI_RL_CONTROL_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RL_ERPM] = {
        .canID = CMR_CANID_DTI_RL_ERPM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RL_CURRENT] = {
        .canID = CMR_CANID_DTI_RL_CURRENT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RL_TEMPFAULT] = {
        .canID = CMR_CANID_DTI_RL_TEMPFAULT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RL_IDIQ] = {
        .canID = CMR_CANID_DTI_RL_IDIQ,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RL_IO_STATUS] = {
        .canID = CMR_CANID_DTI_RL_IO_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RL_ACLIMS] = {
        .canID = CMR_CANID_DTI_RL_ACLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_TRAC_RL_DCLIMS] = {
        .canID = CMR_CANID_DTI_RL_DCLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    }
};

cmr_canRXMeta_t canDaqRXMeta[CANRX_DAQ_LEN] = {
    [CANRX_DAQ_MOVELLA_STATUS] = {
        .canID = CMR_CANID_MOVELLA_STATUS,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    [CANRX_DAQ_MOVELLA_QUATERNION] = {
        .canID = CMR_CANID_MOVELLA_QUATERNION,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    [CANRX_DAQ_MOVELLA_IMU_GYRO] = {
        .canID = CMR_CANID_MOVELLA_IMU_GYRO,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    [CANRX_DAQ_MOVELLA_IMU_ACCEL] = {
        .canID = CMR_CANID_MOVELLA_IMU_ACCEL,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
    [CANRX_DAQ_MOVELLA_VELOCITY] = {
        .canID = CMR_CANID_MOVELLA_VELOCITY,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1000
    },
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
    [CANRX_DAQ_SBG_SLIPANGLE] = {
	    .canID  = CMR_CANID_SBG_AUTOMOTIVE,
        .timeoutError_ms = 1000,
        .timeoutWarn_ms = 750,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
	},
	[CANRX_DAQ_LOAD_FL] = {
        .canID  = CMR_CANID_LOADCELL_FL,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
	},
	[CANRX_DAQ_LOAD_FR] = {
        .canID  = CMR_CANID_LOADCELL_FR,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
	},
	[CANRX_DAQ_LOAD_RL] = {
        .canID  = CMR_CANID_LOADCELL_RL,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
	},
	[CANRX_DAQ_LOAD_RR] = {
        .canID  = CMR_CANID_LOADCELL_RR,
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

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_VSM_TIMEOUT,
        .warnFlag = CMR_CAN_WARN_VSM_TIMEOUT
    },
    [CANRX_VSM_STATUS] = {
        .canID = CMR_CANID_VSM_STATUS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25,
        .errorFlag = CMR_CAN_ERROR_VSM_TIMEOUT,
        .warnFlag = CMR_CAN_WARN_VSM_TIMEOUT,
    },
    [CANRX_VSM_SENSORS] = {
        .canID = CMR_CANID_VSM_SENSORS,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
        .errorFlag = CMR_CAN_ERROR_VSM_TIMEOUT,
        .warnFlag = CMR_CAN_WARN_VSM_TIMEOUT
    },
    [CANRX_FSM_DATA] = {
        .canID = CMR_CANID_FSM_DATA,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 50,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_FSM_SWANGLE] = {
        .canID = CMR_CANID_FSM_SWANGLE,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 50,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    },
    [CANRX_HVC_MINMAX_TEMPS] = {
        .canID = CMR_CANID_HVC_MINMAX_CELL_TEMPS,
        .timeoutError_ms = 5000,
        .timeoutWarn_ms = 2500,
        .errorFlag = CMR_CAN_ERROR_NONE,
        .warnFlag = CMR_CAN_WARN_NONE
    }
};

/** @brief CAN interfaces - Vehicle, DAQ, and Tractive */
static cmr_can_t can[CMR_CAN_BUS_NUM];

static void transmitCDC_DIMconfigMessages();

/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10Hz_priority = 3;
/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10Hz_period_ms = 100;
/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX10Hz_task;

/**
 * @brief Task for sending CAN messages at 10 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX10Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();

    cmr_canCDCWheelVelocity_t speedFeedback;
    cmr_canCDCWheelTorque_t torqueFeedback;
    cmr_canCDCWheelVelocity_t speedSetpoint;
    cmr_canCDCWheelTorque_t torqueSetpoint;

    cmr_canCDCPosePosition_t posePos;
    cmr_canCDCPoseOrientation_t poseOrient;
    cmr_canCDCPoseVelocity_t poseVel;

    cmr_canPowerSense_t powerSense;

    while (1) {
        daqWheelSpeedFeedback(&speedFeedback);
        daqWheelTorqueFeedback(&torqueFeedback);
        daqWheelSpeedSetpoints(&speedSetpoint);
        daqWheelTorqueSetpoints(&torqueSetpoint);
        daqPosePosition(&posePos);
        daqPoseOrientation(&poseOrient);
        daqPoseVelocity(&poseVel);

        powerSense.packCurrent_dA = getCurrent();
        powerSense.packVoltage_cV = getVoltage();
        //powersense is dead, voltage * HVI current
        powerSense.packPower_W = getPackVoltage() * getPackCurrent();

        
        cmr_canDAQTherm_t therms;
        therms.therm_1 = adcRead(ADC_THERM1);
        therms.therm_2 = adcRead(ADC_THERM2);

        canTX(CMR_CAN_BUS_VEH, 0x659, &therms, sizeof(cmr_canDAQTherm_t), canTX10Hz_period_ms);

        // Is data valid? Set it in the orientation/velocity messages
//        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_WHEEL_SPEED_FEEDBACK, &speedFeedback, sizeof(speedFeedback), canTX10Hz_period_ms);
//        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_WHEEL_TORQUE_FEEDBACK, &torqueFeedback, sizeof(torqueFeedback), canTX10Hz_period_ms);
//        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_WHEEL_SPEED_SETPOINT, &speedSetpoint, sizeof(speedSetpoint), canTX10Hz_period_ms);
//        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_WHEEL_TORQUE_SETPOINT, &torqueSetpoint, sizeof(torqueSetpoint), canTX10Hz_period_ms);
        //canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_POSE_POSITION, &posePos, sizeof(posePos), canTX10Hz_period_ms);

        //TODO: Fix error with padding (manual size 7)
        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_POSE_ORIENTATION, &poseOrient, sizeof(poseOrient), canTX10Hz_period_ms);
        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_POSE_VELOCITY, &poseVel, sizeof(poseVel), canTX10Hz_period_ms);

        canTX(CMR_CAN_BUS_VEH, CMR_CANID_FRONT_SLIP_RATIOS, &frontSlipRatios, sizeof(frontSlipRatios), canTX10Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_REAR_SLIP_RATIOS, &rearSlipRatios, sizeof(rearSlipRatios), canTX10Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_FRONT_WHL_SETPOINTS, &frontWhlSetpoints, sizeof(frontSlipRatios), canTX10Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_REAR_WHL_SETPOINTS, &rearWhlSetpoints, sizeof(rearWhlSetpoints), canTX10Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_FRONT_WHL_VELS, &frontWhlVelocities, sizeof(frontWhlVelocities), canTX10Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_REAR_WHL_VELS, &rearWhlVelocities, sizeof(rearWhlVelocities), canTX10Hz_period_ms);

        //powersense is dead, it's voltage * HVI
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CDC_POWER_SENSE, &powerSense, sizeof(powerSense), canTX10Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CDC_COULOMB_COUNTING, &coulombCounting, sizeof(cmr_canCDCKiloCoulombs_t), canTX10Hz_period_ms);

        vTaskDelayUntil(&lastWakeTime, canTX10Hz_period_ms);
    }
}

/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100Hz_priority = 5;
/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;
/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;

/**
 * @brief Task for sending CAN messages at 100 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX100Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    volatile cmr_canHeartbeat_t *heartbeatVSM = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_canHeartbeat_t heartbeat = {
            .state = heartbeatVSM->state
        };


        updateErrorsWarnings(&heartbeat, lastWakeTime);

        if (heartbeat.error[0] != 0 || heartbeat.error[1] != 0) {
            heartbeat.state = CMR_CAN_ERROR;
        }

        cmr_canDAQTherm_t linpots;
        linpots.therm_1 = adcRead(ADC_LINPOT1);
        linpots.therm_2 = adcRead(ADC_LINPOT2);

        canTX(CMR_CAN_BUS_VEH, 0x658, &linpots, sizeof(cmr_canDAQTherm_t), canTX100Hz_period_ms);

        // Solver
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CONTROLS_SOLVER_INPUTS, &solver_inputs, sizeof(cmr_can_solver_inputs_t), canTX100Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CONTROLS_SOLVER_AUX, &solver_aux, sizeof(cmr_can_solver_aux_t), canTX100Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CONTROLS_SOLVER_OUTPUTS, &solver_torques, sizeof(solver_torques), canTX100Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CONTROLS_SOLVER_SETTINGS, &solver_settings, sizeof(cmr_can_solver_settings_t), canTX100Hz_period_ms);

		// SF
		const cmr_canCDCSafetyFilterStates_t *sfStatesInfo = getSafetyFilterInfo();
		cmr_canCDCMotorPower_t *motorPowerInfo = getMotorPowerInfo();
		motorPowerInfo->motor_power_FL = (HAL_FDCAN_GetTxFifoFreeLevel(&(can[CMR_CAN_BUS_TRAC].handle)) >> 16) & 0xFFFF;
		motorPowerInfo->motor_power_FR = HAL_FDCAN_GetTxFifoFreeLevel(&(can[CMR_CAN_BUS_TRAC].handle));

		motorPowerInfo->motor_power_RL = (HAL_FDCAN_GetTxFifoFreeLevel(&(can[CMR_CAN_BUS_VEH].handle)) >> 16) & 0xFFFF;
		motorPowerInfo->motor_power_RR = HAL_FDCAN_GetTxFifoFreeLevel(&(can[CMR_CAN_BUS_VEH].handle));

		canTX(CMR_CAN_BUS_VEH, CMR_CANID_SF_STATE, sfStatesInfo, sizeof(*sfStatesInfo), canTX100Hz_period_ms); //safety filter
		//canTX(CMR_CAN_BUS_VEH, CMR_CANID_MOTORPOWER_STATE, motorPowerInfo, sizeof(*motorPowerInfo), canTX200Hz_period_ms); //motor power
		canTX(CMR_CAN_BUS_DAQ, CMR_CANID_MOTORPOWER_STATE, motorPowerInfo, sizeof(*motorPowerInfo), canTX100Hz_period_ms); //motor power
		//canTX(CMR_CAN_BUS_TRAC, CMR_CANID_MOTORPOWER_STATE, motorPowerInfo, sizeof(*motorPowerInfo), canTX200Hz_period_ms); //motor power


        //debug code for sending rxmeta receive to current time difference
//        uint16_t arr[2];
//        arr[0] = lastWakeTime - canVehicleRXMeta[CANRX_VEH_HEARTBEAT_VSM].lastReceived_ms;
//        canTX(
//                    CMR_CAN_BUS_VEH,
//                    0x108,
//                    &arr,
//                    2,
//                    canTX100Hz_period_ms
//                );

        // Send heartbeat
        canTX(
            CMR_CAN_BUS_VEH,
            CMR_CANID_HEARTBEAT_CDC,
            &heartbeat,
            sizeof(heartbeat),
            canTX100Hz_period_ms
        );
        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}

/** @brief CAN 200 Hz TX priority. */
static const uint32_t canTX200Hz_priority = 6;
/** @brief CAN 200 Hz TX period (milliseconds). */
static const TickType_t canTX200Hz_period_ms = 5;
/** @brief CAN 200 Hz TX task. */
static cmr_task_t canTX200Hz_task;

/**
 * @brief Task for sending CAN messages at 200 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX200Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    volatile cmr_canHeartbeat_t *heartbeatVSM = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);

    const cmr_canDTISetpoints_t *dtiSetpointsFL = getDTISetpoints(MOTOR_FL);
    const cmr_canDTISetpoints_t *dtiSetpointsFR = getDTISetpoints(MOTOR_FR);
    const cmr_canDTISetpoints_t *dtiSetpointsRL = getDTISetpoints(MOTOR_RL);
    const cmr_canDTISetpoints_t *dtiSetpointsRR = getDTISetpoints(MOTOR_RR);

    const cmr_canDTI_TX_TempFault_t *dtiTempFaultFL = getDTITempFault(MOTOR_FL);
    const cmr_canDTI_TX_TempFault_t *dtiTempFaultFR = getDTITempFault(MOTOR_FR);
    const cmr_canDTI_TX_TempFault_t *dtiTempFaultRL = getDTITempFault(MOTOR_RL);
    const cmr_canDTI_TX_TempFault_t *dtiTempFaultRR = getDTITempFault(MOTOR_RR);

    // only have torque in the modifiable struct, have variables in can.c itself that can convert it and then send it
    int16_t currFL = torqueToCurrent
    int16_t currFR = 
    int16_t currRL = 
    int16_t currRL = 

    cmr_canCDCWheelVelocity_t speedFeedback;
    cmr_canCDCWheelTorque_t torqueFeedback;
    cmr_canCDCWheelVelocity_t speedSetpoint;
    cmr_canCDCWheelTorque_t torqueSetpoint;

    cmr_canCDCPosePosition_t posePos;
    cmr_canCDCPoseOrientation_t poseOrient;
    cmr_canCDCPoseVelocity_t poseVel;

    cmr_canCOGVelocity_t cog_velocity;
    cmr_canFrontWheelVelocity_t front_velocity;
    cmr_canRearWheelVelocity_t rear_velocity;

    uint8_t drive_enable = 1;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        if (heartbeatVSM->state == CMR_CAN_HV_EN || 
            heartbeatVSM->state == CMR_CAN_RTD || 
            heartbeatVSM->state == CMR_CAN_AS_READY ||
            heartbeatVSM->state == CMR_CAN_AS_DRIVING){
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FL_DRIVE_EN, &drive_enable, sizeof(uint8_t), canTX200Hz_period_ms);
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FR_DRIVE_EN, &drive_enable, sizeof(uint8_t), canTX200Hz_period_ms);
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RR_DRIVE_EN, &drive_enable, sizeof(uint8_t), canTX200Hz_period_ms);
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RL_DRIVE_EN, &drive_enable, sizeof(uint8_t), canTX200Hz_period_ms);
        }

        canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FL_TORLIMPOS, &(dtiSetpointsFL->torqueLimPos_mNm), sizeof(int16_t), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FL_TORLIMNEG, &(dtiSetpointsFL->torqueLimNeg_mNm), sizeof(int16_t), canTX200Hz_period_ms);

        canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FR_TORLIMPOS, &(dtiSetpointsFR->torqueLimPos_mNm), sizeof(int16_t), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FR_TORLIMNEG, &(dtiSetpointsFR->torqueLimNeg_mNm), sizeof(int16_t), canTX200Hz_period_ms);

        canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RR_TORLIMPOS, &(dtiSetpointsRR->torqueLimPos_mNm), sizeof(int16_t), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RR_TORLIMNEG, &(dtiSetpointsRR->torqueLimNeg_mNm), sizeof(int16_t), canTX200Hz_period_ms);

        canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RL_TORLIMPOS, &(dtiSetpointsRL->torqueLimPos_mNm), sizeof(int16_t), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RL_TORLIMNEG, &(dtiSetpointsRL->torqueLimNeg_mNm), sizeof(int16_t), canTX200Hz_period_ms);

        if (isTorqueMode){
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FL_CURRENT, &(dtiSetpointsFL->ac_current), sizeof(int16_t), canTX200Hz_period_ms);
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FR_CURRENT, &(dtiSetpointsFR->ac_current), sizeof(int16_t), canTX200Hz_period_ms);
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RL_CURRENT, &(dtiSetpointsRL->ac_current), sizeof(int16_t), canTX200Hz_period_ms);
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RR_CURRENT, &(dtiSetpointsRR->ac_current), sizeof(int16_t), canTX200Hz_period_ms);
        } else {
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FL_VELOCITY, &(dtiSetpointsFL->velocity_rpm), sizeof(int16_t), canTX200Hz_period_ms);
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FR_VELOCITY, &(dtiSetpointsFR->velocity_rpm), sizeof(int16_t), canTX200Hz_period_ms);
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RL_VELOCITY, &(dtiSetpointsRL->velocity_rpm), sizeof(int16_t), canTX200Hz_period_ms);
            canTX(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RR_VELOCITY, &(dtiSetpointsRR->velocity_rpm), sizeof(int16_t), canTX200Hz_period_ms);
        }
        
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_DTI_FL_TEMPFAULT, &(dtiTempFaultFL->fault_code), sizeof(uint16_t), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_DTI_FR_TEMPFAULT, &(dtiTempFaultFR->fault_code), sizeof(uint16_t), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_DTI_RL_TEMPFAULT, &(dtiTempFaultRL->fault_code), sizeof(uint16_t), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_DTI_RR_TEMPFAULT, &(dtiTempFaultRR->fault_code), sizeof(uint16_t), canTX200Hz_period_ms);

        daqWheelSpeedFeedback(&speedFeedback);
        daqWheelTorqueFeedback(&torqueFeedback);
        daqWheelSpeedSetpoints(&speedSetpoint);
        daqWheelTorqueSetpoints(&torqueSetpoint);

        daqPosePosition(&posePos);
        //daqPoseOrientation(&poseOrient);
        daqPoseVelocity(&poseVel);
        
        cog_velocity.cog_x = car_state.velocity.x * 100.0f;
        cog_velocity.cog_y = car_state.velocity.y * 100.0f;
        cog_velocity.slip_angle = car_state.slip_angle.body;

        front_velocity.fl_x = car_state.fl_velocity.x * 100.0f;
        front_velocity.fl_y = car_state.fl_velocity.y * 100.0f;
        front_velocity.fr_x = car_state.fr_velocity.x * 100.0f;
        front_velocity.fr_y = car_state.fr_velocity.y * 100.0f;

        rear_velocity.rl_x = car_state.rl_velocity.x * 100.0f;
        rear_velocity.rl_y = car_state.rl_velocity.y * 100.0f;
        rear_velocity.rr_x = car_state.rr_velocity.x * 100.0f;
        rear_velocity.rr_y = car_state.rr_velocity.y * 100.0f;

        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CDC_COG_VELOCITY, &cog_velocity, sizeof(cog_velocity), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CDC_FRONT_VELOCITY, &front_velocity, sizeof(front_velocity), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CDC_REAR_VELOCITY, &rear_velocity, sizeof(rear_velocity), canTX200Hz_period_ms);

        // Is data valid? Set it in the orientation/velocity messages
        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_WHEEL_SPEED_FEEDBACK, &speedFeedback, sizeof(speedFeedback), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_WHEEL_TORQUE_FEEDBACK, &torqueFeedback, sizeof(torqueFeedback), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_WHEEL_SPEED_SETPOINT, &speedSetpoint, sizeof(speedSetpoint), canTX200Hz_period_ms);
        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_WHEEL_TORQUE_SETPOINT, &torqueSetpoint, sizeof(torqueSetpoint), canTX200Hz_period_ms);


        // Forward DTI messages to vehicle CAN at 200Hz.
        for (size_t i = 0; i <= CANRX_TRAC_LEN; i++) {
            // Do not transmit if we haven't received that message lately
            if (cmr_canRXMetaTimeoutError(&canTractiveRXMeta[i], xTaskGetTickCountFromISR()) < 0) continue;

            canTX(
                CMR_CAN_BUS_VEH,
                canTractiveRXMeta[i].canID,
                (void *) &(canTractiveRXMeta[i].payload),
                sizeof(canTractiveRXMeta[i].payload),
                canTX200Hz_period_ms
            );
        }

        // Send setpoints to vehicle CAN at 200Hz as well.
        // canTX(CMR_CAN_BUS_VEH, CMR_CANID_AMK_FL_SETPOINTS, amkSetpointsFL, sizeof(*amkSetpointsFL), canTX200Hz_period_ms);
        // canTX(CMR_CAN_BUS_VEH, CMR_CANID_AMK_FR_SETPOINTS, amkSetpointsFR, sizeof(*amkSetpointsFR), canTX200Hz_period_ms);
        // canTX(CMR_CAN_BUS_VEH, CMR_CANID_AMK_RL_SETPOINTS, amkSetpointsRL, sizeof(*amkSetpointsRL), canTX200Hz_period_ms);
        // canTX(CMR_CAN_BUS_VEH, CMR_CANID_AMK_RR_SETPOINTS, amkSetpointsRR, sizeof(*amkSetpointsRR), canTX200Hz_period_ms);

        // canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_POSE_POSITION, &posePos, sizeof(posePos), canTX200Hz_period_ms);
        //TODO: Fix error with padding (manual size 7)
        //canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_POSE_ORIENTATION, &poseOrient, sizeof(poseOrient), canTX200Hz_period_ms);
        //canTX(CMR_CAN_BUS_DAQ, CMR_CANID_CDC_POSE_VELOCITY, &poseVel, sizeof(poseVel), canTX200Hz_period_ms);

        // YRC
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CONTROLS_PID_IO, &yrcDebug, sizeof(yrcDebug), canTX200Hz_period_ms);

        //Forward HVI Sense to vehic/le CAN. Do not transmit if we haven't received that message lately
        if (cmr_canRXMetaTimeoutError(&canTractiveRXMeta[CANRX_TRAC_HVI_SENSE], xTaskGetTickCountFromISR()) == 0) {
            canTX(
                CMR_CAN_BUS_VEH,
                canTractiveRXMeta[CANRX_TRAC_HVI_SENSE].canID,
                (void *) &(canTractiveRXMeta[CANRX_TRAC_HVI_SENSE].payload),
                sizeof(cmr_canHVIHeartbeat_t),
                canTX200Hz_period_ms
            );
        }

        vTaskDelayUntil(&lastWakeTime, canTX200Hz_period_ms);
    }
}

/** @brief CAN 5 Hz TX priority. */
static const uint32_t canTX5Hz_priority = 2;
/** @brief CAN 5 Hz TX period (milliseconds). */
static const TickType_t canTX5Hz_period_ms = 100;
/** @brief CAN 5 Hz TX task. */
static cmr_task_t canTX5Hz_task;

/**
 * @brief Task for sending CAN messages at 5 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX5Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {

        // Forward SBG Systems messages to vehicle CAN at lower 5Hz rate
        for (size_t i = 0; i < CANRX_DAQ_LEN; i++) {

            uint16_t canID = canDaqRXMeta[i].canID;
            if (canID == CMR_CANID_EMD_MEASUREMENT || i == CANRX_DAQ_MEMORATOR_BROADCAST) {
                continue;
                canID = CMR_CANID_EMD_MEASUREMENT_RETX;
            }

            // Do not transmit if we haven't received that message lately
            if (cmr_canRXMetaTimeoutError(&canDaqRXMeta[i], xTaskGetTickCountFromISR()) < 0) continue;

            // 7 messages in RX stuct are 6 bytes long except position message and EMD message
            size_t message_size = (
                (i == CANRX_DAQ_SBG_POS || (i >= CANRX_DAQ_LOAD_FL && i <= CANRX_DAQ_LOAD_RR) || (i >= CANRX_DAQ_LINPOTS_LEFTS && i <= CANRX_DAQ_LINPOTS_RIGHTS)) ?
                8 : ((i == CANRX_DAQ_SBG_SLIPANGLE) ? 7 : 6)
            );

            if (canID == CMR_CANID_EMD_MEASUREMENT) {
                canID = CMR_CANID_EMD_MEASUREMENT_RETX;
            }

//            canTX(
//                CMR_CAN_BUS_VEH,
//                canID,
//				(void *) &(canDaqRXMeta[i].payload),
//                message_size,
//                canTX5Hz_period_ms
//            );
        }

        // Send DRS state and debug data
        const cmr_canCDCDRSStates_t *drsStatesInfo = getDRSInfo();
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_DRS_STATE, drsStatesInfo, sizeof(*drsStatesInfo), canTX5Hz_period_ms);

        transmitCDC_DIMconfigMessages();

        vTaskDelayUntil(&lastWakeTime, canTX5Hz_period_ms);

        // forward DTI messages to Vehicle CAN

        // Forward DTI messages to vehicle CAN at 200Hz.
        for (size_t i = 0; i <= CANRX_TRAC_LEN; i++) {
            // Do not transmit if we haven't received that message lately
            if (cmr_canRXMetaTimeoutError(&canTractiveRXMeta[i], xTaskGetTickCountFromISR()) < 0) continue;

            canTX(
                CMR_CAN_BUS_VEH,
                canTractiveRXMeta[i].canID,
                (void *) &(canTractiveRXMeta[i].payload),
                sizeof(canTractiveRXMeta[i].payload),
                canTX5Hz_period_ms
            );
        }
    }

}

/** @brief CAN 1 Hz TX priority. */
static const uint32_t canTX1Hz_priority = 2;
/** @brief CAN 1 Hz TX period (milliseconds). */
static const TickType_t canTX1Hz_period_ms = 1000;
/** @brief CAN 1 Hz TX task. */
static cmr_task_t canTX1Hz_task;

/**
 * @brief Task for sending CAN messages at 1 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX1Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {

        cmr_canDAQTest_t daqTest = getDAQTest();
        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_TEST_ID, &daqTest, sizeof(daqTest), canTX1Hz_period_ms);

        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CDC_RTC_DATA_OUT, &time, sizeof(time), canTX1Hz_period_ms);
        cmr_canCDCOdometer_t odometer = (cmr_canCDCOdometer_t) {
            .odometer_km = odometer_km
        };
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CDC_ODOMETER, &odometer, sizeof(odometer), canTX1Hz_period_ms);

        cmr_canCDCControlsStatus_t *controlsStatus = getControlsStatus();
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CDC_CONTROLS_STATUS, controlsStatus, sizeof(cmr_canCDCControlsStatus_t), canTX1Hz_period_ms);

        cmr_canCDCPowerLimitLog_t power_limit = {
            // If you don't #include "safety_filter.h",
            // getPowerLimit_W() is 0!!!!!!!!!!! (╯°□°)╯ノ彡┻━┻
            .power_limit_W = getPowerLimit_W(),
        };
        canTX(CMR_CAN_BUS_VEH, CMR_CANID_CDC_POWER_LOG, &power_limit, sizeof(power_limit), canTX1Hz_period_ms);

        // TODO: constantly send current parameters
        vTaskDelayUntil(&lastWakeTime, canTX1Hz_period_ms);
    }
}

void *canGetPayload(canRX_t rxMsg) {
    configASSERT(rxMsg < CANRX_LEN);

    cmr_canRXMeta_t *rxMeta = &(canRXMeta[rxMsg]);

    return (void *)(&rxMeta->payload);
}

/**
 * @brief Get the Received Driver profile number
 *
 * @param canID
 * @param packet_number
 * @return driver returns -1 if not found
 */
int getReceivedDriver(uint16_t canID, int *packet_number) {
    // Return quickly so that callback doesn't take a long time
    if (canID < CMR_CANID_DIM_CONFIG0_DRV0 || canID > CMR_CANID_DIM_CONFIG3_DRV3) {
        return -1;
    }
    // TODO: condense this logic
    if (canID >= CMR_CANID_DIM_CONFIG0_DRV0 && canID <= CMR_CANID_DIM_CONFIG3_DRV0) {
        *packet_number = canID - CMR_CANID_DIM_CONFIG0_DRV0;
        return 0;
    }
    if (canID >= CMR_CANID_DIM_CONFIG0_DRV1 && canID <= CMR_CANID_DIM_CONFIG3_DRV1) {
        *packet_number = canID - CMR_CANID_DIM_CONFIG0_DRV1;
        return 1;
    }
    if (canID >= CMR_CANID_DIM_CONFIG0_DRV2 && canID <= CMR_CANID_DIM_CONFIG3_DRV2) {
        *packet_number = canID - CMR_CANID_DIM_CONFIG0_DRV2;
        return 2;
    }
    if (canID >= CMR_CANID_DIM_CONFIG0_DRV3 && canID <= CMR_CANID_DIM_CONFIG3_DRV3) {
        *packet_number = canID - CMR_CANID_DIM_CONFIG0_DRV3;
        return 3;
    }
    // If nothing matches, return -1
    return -1;
}

/** @brief All callback for recieving config msgs from DIM
 */
void dim_params_callback (cmr_can_t *canb_rx, uint16_t canID, const void *data, size_t dataLen) {
    // basic filter for wrong canids
    if(canID < CMR_CANID_DIM_CONFIG0_DRV0 || canID > CMR_CANID_CDC_CONFIG3_DRV3) return;

    static bool gotten_packet[NUM_CONFIG_PACKETS] = {0};
    static TickType_t lastDriverChangeTime = 0;
    TickType_t currentTime = xTaskGetTickCount();

    // exit if not glv or hv-enable
    volatile cmr_canHeartbeat_t *vsm_heartbeat = (cmr_canHeartbeat_t *) canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);
    cmr_canVSMState_t vsm_state = vsm_heartbeat->state;
    // if (!(vsm_state == CMR_CAN_VSM_STATE_GLV_ON || vsm_state == CMR_CAN_VSM_STATE_HV_EN)) return;

    // calculate what config packet this message is
    int packet_number = (canID - CMR_CANID_DIM_CONFIG0_DRV0) % NUM_CONFIG_PACKETS;

    // calculate what config packet this message is and the driver
    cmr_driver_profile_t recievedDriver = getReceivedDriver(canID, &packet_number);

    // Exit quickly if not found
    if (recievedDriver == -1) return;

    /* Actual logic starts here */

    // exit if just changed driver and let DIM stop sending data
    if (currentTime - lastDriverChangeTime < 10000) return;

    // cast the data to the appropriate format
    cmr_canDIMCDCconfig_t *dim_data = (cmr_canDIMCDCconfig_t *) data;
    // cast the data to an array for easy indexing
    uint8_t *dim_data_arr = (uint8_t*) dim_data;

    // Copy received data to parametersFromDIM - add 1 because sending Driver is encoded in CAN ID
    int current_copy_index = packet_number*sizeof(cmr_canDIMCDCconfig_t) + 1;
    // Deal with Driver (set to index 0)
    parametersFromDIM[0] = recievedDriver;
    // note, the following only works bc each element is a byte
    int size_to_copy = min( (MAX_MENU_ITEMS - current_copy_index), sizeof(cmr_canDIMCDCconfig_t) );
    memcpy((void *) &(parametersFromDIM[current_copy_index]), dim_data_arr, size_to_copy);
    gotten_packet[packet_number] = true;

    // check if all config messages have been received
    bool all_packets_recieved = true;
    for(uint8_t i = 0; i < NUM_CONFIG_PACKETS; i++){
        all_packets_recieved &= gotten_packet[i];
    }

    // only proceed with the following logic to trigger an FRAM update if all the packets have been received
    if (all_packets_recieved == false) return;

    // Reset received flags
    for(uint8_t i = 0; i < NUM_CONFIG_PACKETS; i++){
        gotten_packet[i] = false;
    }


    // If RX Driver is same as current driver, save local copy and ensure all packets are recieved before writing to FRAM
    if (recievedDriver == currentDriver) {
        framWrite_flag = true;

        // Update the config_menu_main_array
        for (int i = 0; i < MAX_MENU_ITEMS; i++) {
            config_menu_main_array[i].value.value = parametersFromDIM[i];
        }

    }
}

void conditionalCallback(cmr_can_t *canb_rx, uint16_t canID, const void *data, size_t dataLen) {
	uint32_t au32_initial_ticks = DWT->CYCCNT;

	size_t iface_idx = (canb_rx - can);
    configASSERT(iface_idx < CMR_CAN_BUS_NUM);

    // If DIM config message, handle it
    if(CMR_CANID_CDC_CONFIG3_DRV3 >= canID && canID >= CMR_CANID_DIM_CONFIG0_DRV0) {
        dim_params_callback(canb_rx, canID, data, dataLen);
    }

    if(canID == CMR_CANID_CDC_POWER_UPDATE) {
    	cmr_canCDCPowerLimit_t *limit = (cmr_canCDCPowerLimit_t*) data;
    	setPowerLimit_kW(limit->powerLimit_kW);
    }

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

    volatile void* payload = NULL;
    if (rxMetaArray != NULL) {
        for (uint32_t i = 0; i < rxMetaArrayLen; i++) {
            if (rxMetaArray[i].canID == canID) {
                payload = (void *) rxMetaArray[i].payload;
                memcpy(payload, data, dataLen);
                rxMetaArray[i].lastReceived_ms = xTaskGetTickCountFromISR();
                break;
            }
        }
    }

    uint16_t temp = canID & 0x770;
    (void) temp;
    if(temp == 0x770) {
        movella_parse(canID, payload);
    }

    uint32_t total_ticks = DWT->CYCCNT - au32_initial_ticks;
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // Vehicle CAN initialization - CAN1
    cmr_FDcanInit(&can[CMR_CAN_BUS_VEH], FDCAN2, CMR_CAN_BITRATE_500K, NULL,
                  0, &conditionalCallback, GPIOB,
                  GPIO_PIN_12,        // CAN1 RX port/pin.
                  GPIOB, GPIO_PIN_13  // CAN1 TX port/pin.
    );

    // Tractive CAN initialization. - CAN3
    cmr_FDcanInit(&(can[CMR_CAN_BUS_DAQ]), FDCAN3, CMR_CAN_BITRATE_500K, NULL,
                  0, &conditionalCallback, GPIOD,
                  GPIO_PIN_12,        // CAN3 RX port/pin.
                  GPIOD, GPIO_PIN_13  // CAN3 TX port/pin.
    );

    // DAQ CAN init. - CAN2
    cmr_FDcanInit(&can[CMR_CAN_BUS_TRAC], FDCAN1, CMR_CAN_BITRATE_500K, NULL,
                  0, &conditionalCallback, GPIOA,
                  GPIO_PIN_11,        // CAN2 RX port/pin.
                  GPIOA, GPIO_PIN_12  // CAN2 TX port/pin.
    );

    // Vehicle CAN filters.
    const cmr_canFilter_t canVehicleFilters[] = {
        {
            .isMask = true,
            .rxFIFO = FDCAN_RX_FIFO1,

            // Match all even IDs (bottom bit 0, all others don't care).
            .ids = {0x000,0x000}
        },

        {
            .isMask = false,
            .rxFIFO = FDCAN_RX_FIFO0,
            .ids = {CMR_CANID_CDC_RTC_DATA_IN,
                    CMR_CANID_VSM_SENSORS}
        }
    };

    cmr_canFilter(&(can[CMR_CAN_BUS_VEH]), canVehicleFilters,
                  sizeof(canVehicleFilters) / sizeof(canVehicleFilters[0]));

    // Tractive CAN filters.
    const cmr_canFilter_t canTractiveFilters[] = {
        // FL CAN IDs
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_FL_CONTROL_STATUS, CMR_CANID_DTI_FL_ERPM,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_FL_CURRENT, CMR_CANID_DTI_FL_TEMPFAULT,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_FL_IDIQ, CMR_CANID_DTI_FL_IO_STATUS,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_FL_ACLIMS, CMR_CANID_DTI_FL_DCLIMS,}
        },

        // FR CAN IDs
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_FR_CONTROL_STATUS, CMR_CANID_DTI_FR_ERPM,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_FR_CURRENT, CMR_CANID_DTI_FR_TEMPFAULT,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_FR_IDIQ, CMR_CANID_DTI_FR_IO_STATUS,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_FR_ACLIMS, CMR_CANID_DTI_FR_DCLIMS,}
        },

        // RL CAN IDs
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_RL_CONTROL_STATUS, CMR_CANID_DTI_RL_ERPM,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_RL_CURRENT, CMR_CANID_DTI_RL_TEMPFAULT,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_RL_IDIQ, CMR_CANID_DTI_RL_IO_STATUS,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_RL_ACLIMS, CMR_CANID_DTI_RL_DCLIMS,}
        },

        // RR CAN IDs
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_RR_CONTROL_STATUS, CMR_CANID_DTI_RR_ERPM,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_RR_CURRENT, CMR_CANID_DTI_RR_TEMPFAULT,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_RR_IDIQ, CMR_CANID_DTI_RR_IO_STATUS,}
        },
        {.isMask = false,
         .rxFIFO = FDCAN_RX_FIFO0,
         .ids = {CMR_CANID_DTI_RR_ACLIMS, CMR_CANID_DTI_RR_DCLIMS,}
        }
    };

    cmr_canFilter(&(can[CMR_CAN_BUS_TRAC]), canTractiveFilters,
                  sizeof(canTractiveFilters) / sizeof(canTractiveFilters[0]));

    // DAQ CAN filters.
    const cmr_canFilter_t canDaqFilters[] = {
        {.isMask = true,
         .rxFIFO = FDCAN_RX_FIFO0,

         // Match all even IDs (bottom bit 0, all others don't care).
         .ids = {0x000, 0x001}
        },
        {.isMask = true,
         .rxFIFO = FDCAN_RX_FIFO1,

         // Match all odd IDs (bottom bit 1, all others don't care).
         .ids = {0x001, 0x001}
        }
    };

    cmr_canFilter(&(can[CMR_CAN_BUS_DAQ]), canDaqFilters,
                  sizeof(canDaqFilters) / sizeof(canDaqFilters[0]));

    // Task initialization.

    cmr_taskInit(
        &canTX1Hz_task,
        "CAN TX 1Hz",
        canTX1Hz_priority,
        canTX1Hz,
        NULL
    );

    cmr_taskInit(
        &canTX5Hz_task,
        "CAN TX 5Hz",
        canTX5Hz_priority,
        canTX5Hz,
        NULL
    );
    cmr_taskInit(
        &canTX10Hz_task,
        "CAN TX 10Hz",
        canTX10Hz_priority,
        canTX10Hz,
        NULL
    );
    cmr_taskInit(
        &canTX100Hz_task,
        "CAN TX 100Hz",
        canTX100Hz_priority,
        canTX100Hz,
        NULL
    );
    cmr_taskInit(
        &canTX200Hz_task,
        "CAN TX 200Hz",
        canTX200Hz_priority,
        canTX200Hz,
        NULL
    );
}

/**
 * @brief Sends a CAN message with the given ID.
 *
 * @param bus The CAN bus to transmit over.
 * @param id The ID for the message.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 * @param timeout The timeout, in ticks.
 *
 * @return 0 on success, or a negative error code on timeout.
 */
int ABC = 0;
int canTX(cmr_canBusID_t bus, cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    configASSERT(bus < CMR_CAN_BUS_NUM);

    return cmr_canTX(&(can[bus]), id, data, len, timeout);
}

/**
 * @brief Gets a pointer to a vehicle CAN payload.
 *
 * @param msg The desired vehicle CAN message.
 *
 * @return Pointer to desired payload.
 */
volatile void *canVehicleGetPayload(canVehicleRX_t msg) {
    return &(canVehicleRXMeta[msg].payload);
}
/**

 * @brief Gets a pointer to a tractive CAN payload.
 *
 * @param msg The desired tractive CAN message.
 *
 * @return Pointer to desired payload.
 */
volatile void *canTractiveGetPayload(canTractiveRX_t msg) {
    return &(canTractiveRXMeta[msg].payload);
}

/**
 * @brief Gets a pointer to a DAQ CAN payload.
 *
 * @param msg The desired DAQ CAN message.
 *
 * @return Pointer to desired payload.
 */
volatile void *canDAQGetPayload(canDaqRX_t msg) {
    return &(canDaqRXMeta[msg].payload);
}

/**
 * @brief Gets a pointer to a vehicle CAN RX metadata entry.
 *
 * @param msg The desired vehicle CAN message.
 *
 * @return Pointer to desired metadata.
 */
cmr_canRXMeta_t *canVehicleGetMeta(canVehicleRX_t msg) {
    return &(canVehicleRXMeta[msg]);
}

/**
 * @brief Gets a pointer to a tractive CAN RX metadata entry.
 *
 * @param msg The desired tractive CAN message.
 *
 * @return Pointer to desired metadata.
 */
cmr_canRXMeta_t *canTractiveGetMeta(canTractiveRX_t msg) {
    return &(canTractiveRXMeta[msg]);
}

/**
 * @brief Gets a pointer to a DAQ CAN RX metadata entry.
 *
 * @param msg The desired DAQ CAN message.
 *
 * @return Pointer to desired metadata.
 */
cmr_canRXMeta_t *canDAQGetMeta(canDaqRX_t msg) {
    return &(canDaqRXMeta[msg]);
}

/**
 * @brief Return the HV voltage as measured by the EMD.
 *
 * @return HV voltage.
 */
float canEmdHvVoltage() {
//    static const float div = powf(2.0f, 16.0f);
//
//    volatile cmr_canEMDMeasurements_t *meas = canVehicleGetPayload(CANRX_VEH_EMD_MEASURE);
//    int32_t converted = (int32_t) __builtin_bswap32((uint32_t) meas->voltage);
//    return ((float) converted) / div;
	return 0;
}

/**
 * @brief Return the HV current as measured by the EMD.
 *
 * @return HV current.
 */
float canEmdHvCurrent() {
//    static const float div = powf(2.0f, 16.0f);
//
//    volatile cmr_canEMDMeasurements_t *meas = canVehicleGetPayload(CANRX_VEH_EMD_MEASURE);
//    int32_t converted = (int32_t) __builtin_bswap32((uint32_t) meas->current);
//    return ((float) converted) / div;
	return 0;
}

static void transmitCDC_DIMconfigMessages(){
    /* pack struct message for config */
    cmr_canDIMCDCconfig_t config0 = {
        .config_val_1 = config_menu_main_array[1].value.value,
        .config_val_2 = config_menu_main_array[2].value.value,
        .config_val_3 = config_menu_main_array[3].value.value,
        .config_val_4 = config_menu_main_array[4].value.value,
    };
    cmr_canDIMCDCconfig_t config1 = {
        .config_val_1 = config_menu_main_array[5].value.value,
        .config_val_2 = config_menu_main_array[6].value.value,
        .config_val_3 = config_menu_main_array[7].value.value,
        .config_val_4 = config_menu_main_array[8].value.value,
    };
    cmr_canDIMCDCconfig_t config2 = {
        .config_val_1 = config_menu_main_array[9].value.value,
        .config_val_2 = config_menu_main_array[10].value.value,
        .config_val_3 = config_menu_main_array[11].value.value,
        .config_val_4 = config_menu_main_array[12].value.value,
    };
    cmr_canDIMCDCconfig_t config3 = {
        .config_val_1 = config_menu_main_array[13].value.value,
        .config_val_2 = config_menu_main_array[14].value.value,
        .config_val_3 = config_menu_main_array[15].value.value,
        .config_val_4 = config_menu_main_array[16].value.value,
    };

    cmr_canDIMCDCconfig_t config_message_array[NUM_CONFIG_PACKETS] = {
        config0,
        config1,
        config2,
        config3
    };

    // calculate the correct CAN ID based on the current driver
    uint32_t can_ids_config_driver[NUM_CONFIG_PACKETS];
    // uint8_t requested_driver = config_menu_main_array[DRIVER_PROFILE_INDEX].value.value;
    uint32_t base_driver_canid = CMR_CANID_CDC_CONFIG0_DRV0 + (2 * currentDriver * NUM_CONFIG_PACKETS);
    for(int i = 0; i < NUM_CONFIG_PACKETS; i++){
        can_ids_config_driver[i] = base_driver_canid + i;
    }

    /* Transmit new messages to DIM */
    for(int i = 0; i < NUM_CONFIG_PACKETS; i++){
        canTX(
            CMR_CAN_BUS_VEH,
            can_ids_config_driver[i],
            &config_message_array[i],
            sizeof(config_message_array[i]),
            canTX5Hz_period_ms
        );
    }


}

