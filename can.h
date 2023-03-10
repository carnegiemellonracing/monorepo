/**
 * @file can.h
 * @brief Board-specific CAN interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CAN_H
#define CAN_H

#include <CMR/can.h>        // CMR CAN interface
#include <CMR/can_types.h>  // CMR CAN types
#include <CMR/can_ids.h>    // CMR CAN IDs
#include <CMR/config_screen_helper.h> // for config_screen_data tx
#include "state.h"
#include "sensors.h"        // sensors interface


/** @brief Text buffer from RAM - used to display messages to driver */
extern char RAMBUF[];

/** @note the state transition of the dim-cdc handshake is described as follows
 * 
 * 1. The CDC is always broadcasting its values. On boot, config_screen_update_confirmed
 *    will be set to true once it's recieved the initial set of params from the CDC.
 *    Only then will the DIM be able to show the config screen.
 * 2. Once the values are modified, the flush_config_to_cdc variable is set to true.
 * 3. The CAN 5hz function will transmit these config messages and set 
 *    waiting_for_cdc_to_confim_config to true. It will continue to transmit till 
 *    config_screen_update_confirmed is set to true.
 * 4. The RX filter will listen while waiting_for_cdc_to_confirm_config is set true.
 *    Once it gets all the data, it'll set this to false and set the config_screen_update_confirmed
 *    to true. The screen will now exit and go back to the RTD screen per normal. and it'll reset
 *    the config_screen_update_confirmed back to false for another iteration. 
 */

// Config Screen update requested
extern volatile bool flush_config_screen_to_cdc;// = false;

// bool on if waiting for cdc to confirm config screen update
extern volatile bool config_screen_update_confirmed;// = false;

// recieved initial config screen values
extern volatile bool config_screen_values_received_on_boot;// = false;

// letting the rx callback to know to pay attention to the cdc messages
extern volatile bool waiting_for_cdc_to_confirm_config;// = false;

// letting the DIM know that new driver params are available
extern volatile bool waiting_for_cdc_new_driver_config;

extern volatile bool exit_config_request;

/** @brief Checks to see if the screen needs to be redrawn after getting new driver profiles */
extern volatile bool redraw_new_driver_profiles;


/**
 * @brief CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_LEN`.
 */
typedef enum {
    CANRX_HEARTBEAT_VSM = 0,    /**< @brief VSM heartbeat. */
    CANRX_HVC_PACK_VOLTAGE,     /**< @brief BMS pack voltage. */
    CANRX_AMK_FL_ACT_1,         /**< @brief AMK FL status*/
    CANRX_AMK_FR_ACT_1,         /**< @brief AMK FR status*/
    CANRX_AMK_RL_ACT_1,         /**< @brief AMK BL status*/
    CANRX_AMK_RR_ACT_1,         /**< @brief AMK BR status*/
    CANRX_AMK_FL_ACT_2,         /**< @brief AMK FL Temps*/
    CANRX_AMK_FR_ACT_2,         /**< @brief AMK FR Temps*/
    CANRX_AMK_RL_ACT_2,         /**< @brief AMK BL Temps*/
    CANRX_AMK_RR_ACT_2,         /**< @brief AMK BR Temps*/
    CANRX_HVC_PACK_TEMPS,       /**< @brief HVC cell temps. */
    CANRX_VSM_STATUS,           /**< @brief VSM status */
    CANRX_PTCf_LOOP_A_TEMPS,    /**< @brief PTCf Loop A temps */
    CANRX_PTCf_LOOP_B_TEMPS,    /**< @brief PTCf Loop B temps */
    CANRX_PTC_LOOP_A_TEMPS,    /**< @brief PTC Loop A temps */
    CANRX_PTC_LOOP_B_TEMPS,    /**< @brief PTC Loop B temps */
    CANRX_PTC_LOOP_C_TEMPS,    /**< @brief PTC Loop C temps */
    CANRX_HVC_HEARTBEAT,        /**< @brief HVC Error. */
    CANRX_CDC_MOTOR_FAULTS,     /**< @brief CDC Motor Faults */
    CANRX_CDL_BROADCAST,        /**< @brief CDL broadcast. */
    CANRX_SBG_STATUS_3,            /**< @brief INS Status 3 */
	CANRX_EMD_VALUES,			/**< @brief EMD Values for HV voltages and current */
    CANRX_VSM_SENSORS,
    CANRX_HVC_LOW_VOLTAGE,      /**< @brief HVC Low Voltage for Safety Circuit Status*/
    CANRX_DRS_STATE,
    CANRX_LEN     /**< @brief Number of periodic CAN messages. */
} canRX_t;

extern cmr_canRXMeta_t canRXMeta[];

void canInit(void);

int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);

void ramCallback (cmr_can_t *can, uint16_t canID, const void *data, size_t dataLen);
void *getPayload(canRX_t rxMsg);
uint8_t throttleGetPos(void);

float canEmdHvVoltage(cmr_canEMDMeasurements_t emd_vals);
float canEmdHvCurrent(cmr_canEMDMeasurements_t emd_vals);

void transmit_cdc_config_request();

#endif /* CAN_H */

