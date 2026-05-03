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

#include "can.h"  // Interface to implement

#include <CMR/panic.h>  // Panic interface
#include <CMR/tasks.h>  // Task interface
#include <string.h>     // memcpy()
#include <CMR/config_screen_helper.h>
#include <CMR/can_types.h>

#include "adc.h"        // adcVSense, adcISense
#include "gpio.h"       // For actionButtonPressed status

#include "tftDL.h"      // For RAM buffer indices
#include "state.h"	// For new state machine

// Config Screen update requested
bool volatile flush_config_screen_to_cdc = false;

volatile bool update_voltages = true;
volatile bool update_temps = true;
volatile bool update_sensors_odom = true;

// bool on if waiting for cdc to confirm config screen update
bool volatile config_screen_update_confirmed = false;

// recieved initial config screen values
bool volatile config_screen_values_received_on_boot = false;

// letting the rx callback to know to pay attention to the cdc messages
bool volatile waiting_for_cdc_to_confirm_config = false;

// letting the DIM know that it has received all the config screen values for a new driver
bool volatile config_screen_values_received_for_new_driver = false;

bool paddle_is_pressed = false;

// Size of text buffer from RAM
#define RAMBUFLEN 64

/** @brief Text buffer from RAM - used to display messages to driver */
char RAMBUF[RAMBUFLEN];

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    [CANRX_HEARTBEAT_VSM] =       { .canID = CMR_CANID_HEARTBEAT_VSM,.timeoutError_ms = 50,.timeoutWarn_ms = 25 },
    [CANRX_HVC_PACK_VOLTAGE] =    { .canID = CMR_CANID_HVBMS_PACK_VOLTAGE, .timeoutError_ms = 50, .timeoutWarn_ms = 25 },
    [CANRX_DTI_FL_CONTROL_STATUS] = {
        .canID = CMR_CANID_DTI_FL_CONTROL_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FL_ERPM] = {
        .canID = CMR_CANID_DTI_FL_ERPM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FL_CURRENT] = {
        .canID = CMR_CANID_DTI_FL_CURRENT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FL_TEMPFAULT] = {
        .canID = CMR_CANID_DTI_FL_TEMPFAULT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FL_IDIQ] = {
        .canID = CMR_CANID_DTI_FL_IDIQ,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FL_IO_STATUS] = {
        .canID = CMR_CANID_DTI_FL_IO_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FL_ACLIMS] = {
        .canID = CMR_CANID_DTI_FL_ACLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FL_DCLIMS] = {
        .canID = CMR_CANID_DTI_FL_DCLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },

    /* Front Right Inverter (Node ID 0x01) */
    [CANRX_DTI_FR_CONTROL_STATUS] = {
        .canID = CMR_CANID_DTI_FR_CONTROL_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FR_ERPM] = {
        .canID = CMR_CANID_DTI_FR_ERPM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FR_CURRENT] = {
        .canID = CMR_CANID_DTI_FR_CURRENT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FR_TEMPFAULT] = {
        .canID = CMR_CANID_DTI_FR_TEMPFAULT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FR_IDIQ] = {
        .canID = CMR_CANID_DTI_FR_IDIQ,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FR_IO_STATUS] = {
        .canID = CMR_CANID_DTI_FR_IO_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FR_ACLIMS] = {
        .canID = CMR_CANID_DTI_FR_ACLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_FR_DCLIMS] = {
        .canID = CMR_CANID_DTI_FR_DCLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_FR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },

    /* Rear Right Inverter (Node ID 0x02) */
    [CANRX_DTI_RR_CONTROL_STATUS] = {
        .canID = CMR_CANID_DTI_RR_CONTROL_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RR_ERPM] = {
        .canID = CMR_CANID_DTI_RR_ERPM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RR_CURRENT] = {
        .canID = CMR_CANID_DTI_RR_CURRENT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RR_TEMPFAULT] = {
        .canID = CMR_CANID_DTI_RR_TEMPFAULT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RR_IDIQ] = {
        .canID = CMR_CANID_DTI_RR_IDIQ,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RR_IO_STATUS] = {
        .canID = CMR_CANID_DTI_RR_IO_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RR_ACLIMS] = {
        .canID = CMR_CANID_DTI_RR_ACLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RR_DCLIMS] = {
        .canID = CMR_CANID_DTI_RR_DCLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RR | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },

    /* Rear Left Inverter (Node ID 0x03) */
    [CANRX_DTI_RL_CONTROL_STATUS] = {
        .canID = CMR_CANID_DTI_RL_CONTROL_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RL_ERPM] = {
        .canID = CMR_CANID_DTI_RL_ERPM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RL_CURRENT] = {
        .canID = CMR_CANID_DTI_RL_CURRENT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RL_TEMPFAULT] = {
        .canID = CMR_CANID_DTI_RL_TEMPFAULT,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RL_IDIQ] = {
        .canID = CMR_CANID_DTI_RL_IDIQ,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RL_IO_STATUS] = {
        .canID = CMR_CANID_DTI_RL_IO_STATUS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RL_ACLIMS] = {
        .canID = CMR_CANID_DTI_RL_ACLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_DTI_RL_DCLIMS] = {
        .canID = CMR_CANID_DTI_RL_DCLIMS,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 75,
        .warnFlag = CMR_CAN_WARN_CDC_DTI_RL | CMR_CAN_WARN_CDC_DTI_TIMEOUT,
    },
    [CANRX_SENSORIC_VEL_ANG] =    { .canID = CMR_CANID_SENSORIC_VEL_ANG, .timeoutError_ms = 50, .timeoutWarn_ms = 25},
    [CANRX_SENSORIC_DIST] =       { .canID = CMR_CANID_SENSORIC_DIST, .timeoutError_ms = 50, .timeoutWarn_ms = 25},
    [CANRX_HVC_PACK_TEMPS] =      { .canID = CMR_CANID_HVBMS_MIN_MAX_CELL_TEMPERATURE, .timeoutError_ms = 50, .timeoutWarn_ms = 25 },
    [CANRX_VSM_STATUS] =          { .canID = CMR_CANID_VSM_STATUS, .timeoutError_ms = 50, .timeoutWarn_ms = 25 },
    [CANRX_PTCf_LOOP_A_TEMPS] =   { .canID = CMR_CANID_PTC_LOOP_TEMPS_A, .timeoutError_ms = 50, .timeoutWarn_ms = 25 },
    [CANRX_PTC_LOOP_A_TEMPS] =    { .canID = CMR_CANID_PTC_LOOP_TEMPS_A, .timeoutError_ms = 50, .timeoutWarn_ms = 25 },
    [CANRX_PTC_LOOP_B_TEMPS] =    { .canID = CMR_CANID_PTC_LOOP_TEMPS_B, .timeoutError_ms = 50, .timeoutWarn_ms = 25 },
    [CANRX_PTC_LOOP_C_TEMPS] =    { .canID = CMR_CANID_PTC_LOOP_TEMPS_C, .timeoutError_ms = 50, .timeoutWarn_ms = 25 },
    [CANRX_HVC_HEARTBEAT] =       { .canID = CMR_CANID_HEARTBEAT_HVC, .timeoutError_ms = 50, .timeoutWarn_ms = 25 },
    [CANRX_HVC_BMB_STATUS] =      { .canID = CMR_CANID_HVC_BMB_STATUS_ERRORS, .timeoutError_ms = 50, .timeoutWarn_ms = 25 },
    [CANRX_MEMORATOR_BROADCAST] = { .canID = CMR_CANID_HEARTBEAT_MEMORATOR, .timeoutError_ms = 2000, .timeoutWarn_ms = 1500 },
    [CANRX_MOVELLA_STATUS] =        { .canID = CMR_CANID_MOVELLA_STATUS, .timeoutError_ms = 4000, .timeoutWarn_ms = 2000 },
    [CANRX_EMD_VALUES] =          { .canID = CMR_CANID_EMD_MEASUREMENT, .timeoutError_ms = 4000, .timeoutWarn_ms = 2000 },
    [CANRX_VSM_SENSORS] =         { .canID = CMR_CANID_VSM_SENSORS, .timeoutError_ms = 50, .timeoutWarn_ms = 25 },
    [CANRX_HVC_LOW_VOLTAGE] =     { .canID = CMR_CANID_HVC_LOW_VOLTAGE, .timeoutError_ms = 4000, .timeoutWarn_ms = 2000 },
    [CANRX_DRS_STATE] =           { .canID = CMR_CANID_DRS_STATE, .timeoutError_ms = 4000, .timeoutWarn_ms = 2000 },
    [CANRX_CDC_ODOMETER] =        { .canID = CMR_CANID_CDC_ODOMETER, .timeoutError_ms = 4000, .timeoutWarn_ms = 2000 },
    [CANRX_CDC_CONTROLS_STATUS] = { .canID = CMR_CANID_CDC_CONTROLS_STATUS, .timeoutError_ms = 4000, .timeoutWarn_ms = 2000 },
    [CANRX_CDC_HEARTBEAT] =       { .canID = CMR_CANID_HEARTBEAT_CDC, .timeoutError_ms = 4000, .timeoutWarn_ms = 2000 },
    [CANRX_PACK_CELL_VOLTAGES] =  { .canID = CMR_CANID_HVC_MINMAX_CELL_VOLTAGE, .timeoutError_ms = 4000, .timeoutWarn_ms = 2000 },
    [CANRX_EAB_STATUS] =          { .canID = CMR_CANID_EAB_STATUS, .timeoutError_ms = 100, .timeoutWarn_ms = 50 },
    [CANRX_VSM_POWER_DIAGNOSTICS] = {.canID = CMR_CANID_VSM_POWER_DIAGNOSTICS} 
};

/** @brief Primary CAN interface. */
static cmr_can_t can;

/**
 * @brief Callback for the RAM messages to the DIM to write to the screen
 *
 * For displaying from the RAM buffer, character indices 0-20 inclusive are for the
 * message at the top. This corresponds to text->address 0, 1, 2, 3, and 4.
 * Character indices 40-52 inclusive are for the first note on the right side of the screen.
 * This corresponds to text->address 0x0A, 0x0B, and 0x0C.
 * Character indices 60-72 inclusive are for the second note on the right side of the screen.
 * This corresponds to text->address 0x0F, 0x10, and 0x11.
 */

bool verifyData(uint8_t dim_config_data_array_starting_idx, int items_per_struct,  uint8_t *cdc_config_data_arr){
    for (uint8_t i = 0; i < items_per_struct; i++) {
        if(config_menu_main_array[dim_config_data_array_starting_idx + i].value.value != cdc_config_data_arr[i]){
            return false;
        }
    }
    return true;
}

void setConfigValues(uint8_t dim_config_data_array_starting_idx,
                     int items_per_struct, uint8_t *cdc_config_data_arr) {
    for (uint8_t i = 0; i < items_per_struct; i++) {
        config_menu_main_array[dim_config_data_array_starting_idx + i].value.value =
            cdc_config_data_arr[i];
    }
}

bool correctDriverCanID(uint32_t canID) {
    // calulcate the base canID for the requested driver from the CDC side
    uint8_t requested_driver = config_menu_main_array[DRIVER_PROFILE_INDEX].value.value;
    uint32_t base_driver_canid = CMR_CANID_CDC_CONFIG0_DRV0 + (2 * requested_driver * NUM_CONFIG_PACKETS);
    return (canID >= base_driver_canid && canID < base_driver_canid + NUM_CONFIG_PACKETS);
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    for (uint16_t i = 0; i < RAMBUFLEN; i++) {
        if (i == PREV_TIME_INDEX + TIMEDISPLAYLEN - 1 ||
            i == TARGET_TIME_INDEX + TIMEDISPLAYLEN - 1 ||
            i == MESSAGE_INDEX + MESSAGEDISPLAYLEN - 1) {
            RAMBUF[i] = '\0';
        } else {
            RAMBUF[i] = ' ';
        }
    }
}

/**
 * @brief Sends a CAN message with the given ID.
 *
 * @param id The ID for the message.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 * @param timeout The timeout, in ticks.
 *
 * @return 0 on success, or a negative error code on timeout.
 */
int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&can, id, data, len, timeout);
}

// /**
//  * @brief Return the HV voltage as measured by the EMD.
//  *
//  * @return HV voltage.
//  */
// float canEmdHvVoltage(cmr_canEMDMeasurements_t emd_vals) {
//     static const float div = 65536.0f;

//     int32_t converted = (int32_t)__builtin_bswap32((uint32_t)emd_vals.voltage);
//     return ((float)converted) / div;
// }

// /**
//  * @brief Return the HV current as measured by the EMD.
//  *
//  * @return HV current.
//  */
// float canEmdHvCurrent(cmr_canEMDMeasurements_t emd_vals) {
//     static const float div = 65536.0f;

//     int32_t converted = (int32_t)__builtin_bswap32((uint32_t)emd_vals.current);
//     return ((float)converted) / div;
// }

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

int32_t getDTIERPM(canRX_t rxMsg) {
    cmr_canDTI_TX_Erpm_t *dtiERPM = getPayload(rxMsg);
    return big_endian_to_int32(&(dtiERPM->erpm));
}

int16_t getDTIACCurrent_dA(canRX_t rxMsg) {
    cmr_canDTI_TX_Current_t *dtiCurrent = getPayload(rxMsg);
    return parse_int16(&(dtiCurrent->ac_current_dA));
}

int16_t getDTIDCCurrent_dA(canRX_t rxMsg) {
    cmr_canDTI_TX_Current_t *dtiCurrent = getPayload(rxMsg);
    return parse_int16(&(dtiCurrent->dc_current_dA));
}

int16_t getDTICtlrTemp_dC(canRX_t rxMsg) {
    cmr_canDTI_TX_TempFault_t *dtiTempFault = getPayload(rxMsg);
    return parse_int16(&(dtiTempFault->ctlr_temp));
}

int16_t getDTIMotorTemp_dC(canRX_t rxMsg) {
    cmr_canDTI_TX_TempFault_t *dtiTempFault = getPayload(rxMsg);
    return parse_int16(&(dtiTempFault->motor_temp));
}


cmr_canBMSMinMaxCellVoltage_t* getPackVoltages(void){
    cmr_canBMSMinMaxCellVoltage_t* packVoltagesStruct = getPayload(CANRX_PACK_CELL_VOLTAGES);
    return packVoltagesStruct;
}
