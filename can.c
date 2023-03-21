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
#include <math.h>       // powf()

#include <CMR/tasks.h>  // Task interface

#include "can.h"    // Interface to implement
#include "adc.h"    // adcVSense, adcISense
#include "state.h"  // State interface
#include "tftDL.h"  // For RAM buffer indices
#include "gpio.h"   // For actionButtonPressed status
#include "expanders.h" // For Paddles

// Config Screen update requested
bool volatile flush_config_screen_to_cdc = false;

// bool on if waiting for cdc to confirm config screen update
bool volatile config_screen_update_confirmed= false;

// recieved initial config screen values
bool volatile config_screen_values_received_on_boot = false;

// letting the rx callback to know to pay attention to the cdc messages
bool volatile waiting_for_cdc_to_confirm_config = false;

// letting the DIM know that it has received all the config screen values for a new driver
bool volatile config_screen_values_received_for_new_driver = false;


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
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_PACK_VOLTAGE] = {
        .canID = CMR_CANID_HVC_PACK_VOLTAGE,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_FL_ACT_1] = {// TODO: Change CAN ID order based on inverter CAN IDs
        .canID = CMR_CANID_AMK_1_ACT_1,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_FR_ACT_1] = {
        .canID = CMR_CANID_AMK_2_ACT_1,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_RL_ACT_1] = {
        .canID = CMR_CANID_AMK_3_ACT_1,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_RR_ACT_1] = {
        .canID = CMR_CANID_AMK_4_ACT_1,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_FL_ACT_2] = {
        .canID = CMR_CANID_AMK_1_ACT_2,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_FR_ACT_2] = {
        .canID = CMR_CANID_AMK_2_ACT_2,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_RL_ACT_2] = {
        .canID = CMR_CANID_AMK_3_ACT_2,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_RR_ACT_2] = {
        .canID = CMR_CANID_AMK_4_ACT_2,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_PACK_TEMPS] = {
        .canID = CMR_CANID_HVC_MINMAX_CELL_TEMPS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_VSM_STATUS] = {
        .canID = CMR_CANID_VSM_STATUS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_PTCf_LOOP_A_TEMPS] = {
        .canID = CMR_CANID_PTC_LOOP_TEMPS_A,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_PTC_LOOP_A_TEMPS] = {
        .canID = CMR_CANID_PTC_LOOP_TEMPS_A,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_PTC_LOOP_B_TEMPS] = {
        .canID = CMR_CANID_PTC_LOOP_TEMPS_B,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_PTC_LOOP_C_TEMPS] = {
        .canID = CMR_CANID_PTC_LOOP_TEMPS_C,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_HEARTBEAT] = {
        .canID = CMR_CANID_HEARTBEAT_HVC,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_BMB_STATUS] = {
        .canID = CMR_CANID_HVC_BMB_STATUS_ERRORS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_CDL_BROADCAST] = {
        .canID = CMR_CANID_CDL_BROADCAST,
        .timeoutError_ms = 4000,
        .timeoutWarn_ms = 2000
    },
    [CANRX_SBG_STATUS_3] = {
        .canID = CMR_CANID_SBG_STATUS_3,
        .timeoutError_ms = 4000,
        .timeoutWarn_ms = 2000
    },
    [CANRX_EMD_VALUES] = {
		.canID = CMR_CANID_EMD_MEASUREMENT,
		.timeoutError_ms = 4000,
		.timeoutWarn_ms = 2000
    },
    [CANRX_VSM_SENSORS] = {
        .canID = CMR_CANID_VSM_SENSORS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_LOW_VOLTAGE] = {
        .canID = CMR_CANID_HVC_LOW_VOLTAGE,
        .timeoutError_ms = 4000,
		.timeoutWarn_ms = 2000
    }, 
    [CANRX_DRS_STATE] = {
        .canID = CMR_CANID_DRS_STATE,
        .timeoutError_ms = 4000,
		.timeoutWarn_ms = 2000
    },
    [CANRX_CDC_ODOMETER] {
        .canID = CMR_CANID_CDC_ODOMETER,
        .timeoutError_ms = 4000,
		.timeoutWarn_ms = 2000
    }
};

/** @brief Primary CAN interface. */
static cmr_can_t can;

/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10Hz_priority = 3;

/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10Hz_period_ms = 100;

/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX10Hz_task;

// Forward declarations
static void sendHeartbeat(TickType_t lastWakeTime);
static void sendFSMData(void);
static cmr_canWarn_t genSafetyCircuitMessage(void);
static void sendFSMPedalsADC(void);
static void sendFSMSensorsADC(void);
static void sendPowerDiagnostics(void);

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
    cmr_driver_profile_t previousDriverReq = Default;  // Request Default
    while (1) {
        /* if DIM is requesting a state/gear change
         * send this request to VSM */
        cmr_canState_t stateVSM = stateGetVSM();
        cmr_canState_t stateVSMReq = stateGetVSMReq();
        cmr_canGear_t gear = stateGetGear();
        cmr_canGear_t gearReq = stateGetGearReq();
        cmr_canDRSMode_t drsMode = stateGetDrs();
        cmr_canDRSMode_t drsReq = stateGetDrsReq();

        if (
            (stateVSM != stateVSMReq) ||
            (gear != gearReq) ||
            (drsMode != drsReq) ||
            (config_menu_main_array[DRIVER_PROFILE_INDEX].value.value != previousDriverReq)
        ) {
            cmr_canDIMRequest_t request = {
                .requestedState = stateVSMReq,
                .requestedGear = gearReq,
                .requestedDrsMode = drsReq,
                .requestedDriver = (uint8_t) config_menu_main_array[DRIVER_PROFILE_INDEX].value.value
            };
            canTX(
                CMR_CANID_DIM_REQUEST,
                &request, sizeof(request),
                canTX10Hz_period_ms
            );

            previousDriverReq = config_menu_main_array[DRIVER_PROFILE_INDEX].value.value;
            stateGearUpdate();
            stateDrsUpdate();
        }
        sendPowerDiagnostics();

        // TODO These should probably only be sent when in some "calibration"
        // mode that we enter upon receiving some "begin calibration" message
        // from PCAN Explorer
        sendFSMPedalsADC();
        sendFSMSensorsADC();

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

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        sendHeartbeat(lastWakeTime);
        sendFSMData();

        // Calculate integer regenPercent from regenStep
        uint8_t regenPercent = (uint8_t) (REGEN_MIN + REGEN_STEP * regenStep);

        /* Transmit action button status */
        cmr_canDIMActions_t actions = {
            .drsButtonPressed = drsButtonPressed,
            .action1ButtonPressed = action1ButtonPressed,
            .action2ButtonPressed = action2ButtonPressed,
            .regenPercent = regenPercent,
            .paddleLeft = getLeftPaddleState(),
            .paddleRight = getRightPaddleState(),
        };
        canTX(
            CMR_CANID_DIM_ACTIONS,
            &actions, sizeof(actions),
            canTX100Hz_period_ms
        );

        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}

/** @brief CAN 1 Hz TX priority. */
static const uint32_t canTX1Hz_priority = 3;

/** @brief CAN 1 Hz TX period (milliseconds). */
static const TickType_t canTX1Hz_period_ms = 100;

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
        if (flush_config_screen_to_cdc){
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

            cmr_canDIMCDCconfig_t config_message_array[num_config_packets] = {
                config0,
                config1,
                config2,
                config3
            };

            // calculate the correct CAN ID based on the current driver
            uint32_t can_ids_config_driver[num_config_packets];
            uint8_t requested_driver = config_menu_main_array[DRIVER_PROFILE_INDEX].value.value;
            uint32_t base_driver_canid = CMR_CANID_DIM_CONFIG0_DRV0 + (2 * requested_driver * num_config_packets);
            for(int i = 0; i < num_config_packets; i++){
                can_ids_config_driver[i] = base_driver_canid + i;
            }

            /* Transmit new messages to cdc */
            for(int i = 0; i < num_config_packets; i++){
                canTX(
                    can_ids_config_driver[i],
                    &config_message_array[i],
                    sizeof(config_message_array[i]),
                    canTX1Hz_period_ms
                );
            } 

            /* Set waiting for cdc to be true. RX will wipe this and flush_config_screen
               only if this was true */
        }

        vTaskDelayUntil(&lastWakeTime, canTX1Hz_period_ms);
    }
}

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
void ramRxCallback (cmr_can_t *can, uint16_t canID, const void *data, size_t dataLen) {
    if (canID == CMR_CANID_DIM_TEXT_WRITE) {
        cmr_canDIMTextWrite_t *text = (cmr_canDIMTextWrite_t *)data;
        if (dataLen == sizeof(cmr_canDIMTextWrite_t)) {
            uint16_t index = ((uint16_t)text->address) << 2;
            if (index < RAMBUFLEN) {
                memcpy(RAMBUF + index, &(text->data), 4);
                
                /* 
                 * Replace all null terminators with spaces - otherwise screen
                 * doesn't display any characters after a null terminator is found
                 * clearing is so that if found null terminator within in section of text,
                 * all characters after null terminator will also be cleared
                 */
                bool clearing = false;
                for(uint16_t i = 0; i < RAMBUFLEN; i++) {
                    if (i == TARGET_TIME_INDEX || i == MESSAGE_INDEX) {
                        clearing = false;
                    }
                    if (RAMBUF[i] == '\0' || clearing) {
                        clearing = true;
                        RAMBUF[i] = ' ';
                    }
                }
            }
        }
    }
}

void cdcRXCallback(cmr_can_t *can, uint16_t canID, const void *data, size_t dataLen){
    // the gotten packet array keeps track of which of the config packets we've gotten
    // since they can be received out of order.
    static bool gotten_packet[num_config_packets] = {0};
    static bool initialized = false;
    static int items_per_struct = 4; // number of items in the struct. Probably not good to leave hardcoded

    // calculate what config packet this message is
    int packet_number = (canID - CMR_CANID_CDC_CONFIG0_DRV0) % num_config_packets;
    // cast the data to the appropriate format
    cmr_canDIMCDCconfig_t *cdc_config_data = (cmr_canDIMCDCconfig_t*) data;
    // cast the data to an array for easy indexing. Sly i know :P
    uint8_t *cdc_config_data_arr = (uint8_t*) cdc_config_data;

    // find the appropriate values to modify in the local copy of our data 
    // note that there are 4 values per config struct hence the *4
    // Increment i by 1 because offset for driver index
    uint8_t dim_config_data_array_starting_idx = packet_number * items_per_struct + 1; 

    if (packet_number >= num_config_packets){
        // time to shit yo pants bc some wack shit has happened.
        // TODO: Add throwing an error here
        while(1);
    }

    /**** Cold Boot, await default parameters-- blindly read data if it's the first driver and set that to done ****/
    if (!initialized){
        config_menu_main_array[DRIVER_PROFILE_INDEX].value.value = 0; // Initialize to driver 0
        if (((uint32_t)canID - CMR_CANID_CDC_CONFIG0_DRV0) < num_config_packets){
        	int local_can_data_index = 0;
            // copy data over to local memory!
            for(uint8_t i = dim_config_data_array_starting_idx; i < dim_config_data_array_starting_idx + items_per_struct; i++){
                config_menu_main_array[i].value.value = cdc_config_data_arr[local_can_data_index++];
            } 
        }
        // mark the appropriate packet as recieved
        gotten_packet[packet_number] = true;
    }

    /**** Update driver config. Same driver, new data ****/
    // if waiting to save -- make sure all data is same and then reset that
    else if(waiting_for_cdc_to_confirm_config){
        uint8_t requested_driver = config_menu_main_array[DRIVER_PROFILE_INDEX].value.value;
        // calulcate the base canID for the requested driver from the CDC side
        uint32_t requested_driver_cdc_canid = CMR_CANID_CDC_CONFIG0_DRV0 + (2 * requested_driver * num_config_packets);

        // filter for only the right driver can ID
        if (((uint32_t)canID - requested_driver_cdc_canid) < num_config_packets && ((uint32_t) canID) >= requested_driver_cdc_canid){
            bool all_data_matches = true;

            // get the data and check if all the data is the same
            int local_index = 0;
            for(uint8_t i = dim_config_data_array_starting_idx; i < dim_config_data_array_starting_idx + items_per_struct; i++){
                all_data_matches &= (config_menu_main_array[i].value.value == cdc_config_data_arr[local_index++]);
            }
            // set appropriate config message rx flag if data matches
            gotten_packet[packet_number] = all_data_matches;
        }
    }

    /**** Get New Driver Parameters. Blindly read them since only CDC knows the right values for new driver ****/
    else if(waiting_for_cdc_new_driver_config){
        // new requested driver is already in the config_main_menu
        uint8_t requested_driver = config_menu_main_array[DRIVER_PROFILE_INDEX].value.value;
        // calulcate the base canID for the requested driver from the CDC side
        uint32_t requested_driver_cdc_canid = CMR_CANID_CDC_CONFIG0_DRV0 + (2 * requested_driver * num_config_packets);

        // filter for only the right driver can ID based on the new requested driver
        if (((uint32_t)canID - requested_driver_cdc_canid) < num_config_packets && ((uint32_t) canID) >= requested_driver_cdc_canid){
            // get the data and flush it to local memory
        	int local_index = 0;
            for(uint8_t i = dim_config_data_array_starting_idx; i < dim_config_data_array_starting_idx + items_per_struct; i++){
                config_menu_main_array[i].value.value = cdc_config_data_arr[local_index++];
            } 
            // set appropriate config message rx flag if data matches
            gotten_packet[packet_number] = true;
        }
    }

    // check if all config messages have been received
    bool all_packets_recieved = true;
    for(uint8_t i = 0; i < num_config_packets; i++){
        all_packets_recieved &= gotten_packet[i];
    }

    // if all data is recieved
    if(all_packets_recieved){
        // no need to ever re-init data
        initialized = true;
        config_screen_values_received_on_boot = true;

        // if statements put here to prevent out of order execution/ context switching
        // in case they are set false here before they are requested to be true elsewhere.
        if (waiting_for_cdc_to_confirm_config) {
        	waiting_for_cdc_to_confirm_config = false;
            if (exit_config_request) {
        	    exitConfigScreen();
            }
        }
        if (flush_config_screen_to_cdc) flush_config_screen_to_cdc = false; 
        if (waiting_for_cdc_new_driver_config){
        	waiting_for_cdc_new_driver_config = false;
        	// redraw all values
            redraw_new_driver_profiles = true;
        }


        // reset all packets rx for the next run
        for(uint8_t i = 0; i < num_config_packets; i++){
            gotten_packet[i] = false;
        }

        config_screen_update_confirmed = true;
    }
}

void canRXCallback(cmr_can_t *can, uint16_t canID, const void *data, size_t dataLen){
    if (canID == CMR_CANID_DIM_TEXT_WRITE) {
        ramRxCallback(can, canID, data, dataLen);
    }
    // make sure its a valid can id for config.
    if (canID >= CMR_CANID_CDC_CONFIG0_DRV0 && 
        canID <= CMR_CANID_CDC_CONFIG3_DRV3){
    	cdcRXCallback(can, canID, data, dataLen);
    }
}


/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &can, CAN1, CMR_CAN_BITRATE_500K,
        canRXMeta, sizeof(canRXMeta) / sizeof(canRXMeta[0]),
        &canRXCallback,
        GPIOB, GPIO_PIN_8,     // CAN1 RX port/pin.
        GPIOB, GPIO_PIN_9      // CAN1 TX port/pin.
    );

    // Clear RAM Buf - Set all to Spaces
    for(uint16_t i = 0; i < RAMBUFLEN; i++) {
        RAMBUF[i] = ' ';
    }

    // CAN2 filters.
    const cmr_canFilter_t canFilters[] = {
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_HVC_PACK_VOLTAGE,
                CMR_CANID_HVC_BMB_STATUS_ERRORS,
                CMR_CANID_HEARTBEAT_HVC
            }
        },
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_SBG_STATUS_3,
                CMR_CANID_CDC_ODOMETER,
                CMR_CANID_DIM_TEXT_WRITE,
                CMR_CANID_DIM_TEXT_WRITE
            }
        },
        {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_AFC1_DRIVER_TEMPS,
                CMR_CANID_HVC_MINMAX_CELL_TEMPS,
                CMR_CANID_VSM_STATUS,
                CMR_CANID_CDL_BROADCAST
            }
        },
        {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_PTC_LOOP_TEMPS_A,
                CMR_CANID_PTC_LOOP_TEMPS_B,
                CMR_CANID_PTC_LOOP_TEMPS_C,
                CMR_CANID_PTC_LOOP_TEMPS_B
            }
        },
        {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_AMK_1_ACT_1,
                CMR_CANID_AMK_2_ACT_1,
                CMR_CANID_AMK_3_ACT_1,
                CMR_CANID_AMK_4_ACT_1
            }
        },
        {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_AMK_1_ACT_2,
                CMR_CANID_AMK_2_ACT_2,
                CMR_CANID_AMK_3_ACT_2,
                CMR_CANID_AMK_4_ACT_2
            }
        },
        {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_CDC_CONFIG0_DRV0,
                CMR_CANID_CDC_CONFIG1_DRV0,
                CMR_CANID_CDC_CONFIG2_DRV0,
                CMR_CANID_CDC_CONFIG3_DRV0
            }
        },
        {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_CDC_CONFIG0_DRV1,
                CMR_CANID_CDC_CONFIG1_DRV1,
                CMR_CANID_CDC_CONFIG2_DRV1,
                CMR_CANID_CDC_CONFIG3_DRV1
            }
        },
        {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_CDC_CONFIG0_DRV2,
                CMR_CANID_CDC_CONFIG1_DRV2,
                CMR_CANID_CDC_CONFIG2_DRV2,
                CMR_CANID_CDC_CONFIG3_DRV2
            }
        },
        {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_CDC_CONFIG0_DRV3,
                CMR_CANID_CDC_CONFIG1_DRV3,
                CMR_CANID_CDC_CONFIG2_DRV3,
                CMR_CANID_CDC_CONFIG3_DRV3
            }
        },
            {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_EMD_MEASUREMENT,
                CMR_CANID_EMD_MEASUREMENT,
                CMR_CANID_EMD_MEASUREMENT,
                CMR_CANID_EMD_MEASUREMENT
            }
        },
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO1,
            .ids = {
                CMR_CANID_VSM_SENSORS,
                CMR_CANID_VSM_SENSORS,
                CMR_CANID_HVC_LOW_VOLTAGE,
                CMR_CANID_DRS_STATE
            }
        }
    };

    cmr_canFilter(
        &can, canFilters, sizeof(canFilters) / sizeof(canFilters[0])
    );

    // Task initialization.
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
        &canTX1Hz_task,
        "CAN TX 1Hz",
        canTX1Hz_priority,
        canTX1Hz,
        NULL
    );
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

/**
 * @brief Return the HV voltage as measured by the EMD.
 *
 * @return HV voltage.
 */
float canEmdHvVoltage(cmr_canEMDMeasurements_t emd_vals) {
    static const float div = powf(2.0f, 16.0f);

    int32_t converted = (int32_t) __builtin_bswap32((uint32_t) emd_vals.voltage);
    return ((float) converted) / div;
}

/**
 * @brief Return the HV current as measured by the EMD.
 *
 * @return HV current.
 */
float canEmdHvCurrent(cmr_canEMDMeasurements_t emd_vals) {
    static const float div = powf(2.0f, 16.0f);

    int32_t converted = (int32_t) __builtin_bswap32((uint32_t) emd_vals.current);
    return ((float) converted) / div;
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

/**
 * @brief Sets up FSM CAN heartbeat by checking errors and sends it.
 *
 * @param lastWakeTime Pass in from canTX100Hz. Used to determine pedal implausibility
 * according to FSAE rule T.6.2.3.
 */
static void sendHeartbeat(TickType_t lastWakeTime) {
    cmr_canRXMeta_t *heartbeatVSMMeta = &(canRXMeta[CANRX_HEARTBEAT_VSM]);

    cmr_canState_t vsmState = stateGetVSM();
    cmr_canHeartbeat_t heartbeat = {
        .state = vsmState
    };

    cmr_canWarn_t warning = CMR_CAN_WARN_NONE;
    cmr_canError_t error = CMR_CAN_ERROR_NONE;

    cmr_sensorListGetFlags(&sensorList, &warning, &error);

    if (cmr_sensorListGetValue(&sensorList, SENSOR_CH_BPP_IMPLAUS) != 0) {
        warning |= CMR_CAN_WARN_FSM_BPP;
    }
    if (cmr_sensorListGetValue(&sensorList, SENSOR_CH_TPOS_IMPLAUS) != 0) {
        warning |= CMR_CAN_WARN_FSM_TPOS_IMPLAUSIBLE;
    }

    if (cmr_canRXMetaTimeoutError(heartbeatVSMMeta, lastWakeTime) < 0) {
        error |= CMR_CAN_ERROR_VSM_TIMEOUT;
    }

    // TODO: are these the correct bits?
    warning |= genSafetyCircuitMessage(); // add safety circuit message to heartbeat

    if (error != CMR_CAN_ERROR_NONE) {
        heartbeat.state = CMR_CAN_ERROR;
    }

    memcpy(&heartbeat.error, &error, sizeof(heartbeat.error));

    if (cmr_canRXMetaTimeoutWarn(heartbeatVSMMeta, lastWakeTime) < 0) {
        warning |= CMR_CAN_WARN_VSM_TIMEOUT;
    }

    memcpy(&heartbeat.warning, &warning, sizeof(heartbeat.warning));

    canTX(
        CMR_CANID_HEARTBEAT_FSM,
        &heartbeat,
        sizeof(heartbeat),
        canTX100Hz_period_ms
    );
    canTX(
        CMR_CANID_HEARTBEAT_DIM,
        &heartbeat,
        sizeof(heartbeat),
        canTX100Hz_period_ms
    );

}

/**
 * @brief Sends FSM data message.
 */
static void sendFSMData(void) {
    cmr_canRXMeta_t *heartbeatVSMMeta = &(canRXMeta[CANRX_HEARTBEAT_VSM]);
    volatile cmr_canHeartbeat_t *heartbeatVSM = (void *) heartbeatVSMMeta->payload;

    uint8_t throttlePosition = throttleGetPos();
    uint8_t torqueRequested = 0;

    if (heartbeatVSM->state == CMR_CAN_RTD &&
        cmr_sensorListGetValue(&sensorList, SENSOR_CH_BPP_IMPLAUS) == 0 // Temp comment to remove implausability
    ) {
        torqueRequested = throttlePosition;
    }

    uint8_t brakePressureFront_PSI = (uint8_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BPRES_PSI);
    int16_t steeringWheelAngle_deg = (int16_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_SWANGLE_DEG);
    uint8_t brakePedalPosition = (uint8_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BPOS_U8);

    cmr_canFSMData_t msg = {
        .torqueRequested         = torqueRequested,
        .throttlePosition        = throttlePosition,
        .brakePressureFront_PSI  = brakePressureFront_PSI,
        .brakePedalPosition      = brakePedalPosition,
        .steeringWheelAngle_deg  = steeringWheelAngle_deg
    };

    canTX(CMR_CANID_FSM_DATA, &msg, sizeof(msg), canTX100Hz_period_ms);
}

/**
 * @brief Sends safety circuit message.
 */
static cmr_canWarn_t genSafetyCircuitMessage(void) {
    sensorChannel_t safetyCircuitStart = SENSOR_CH_SS_MODULE; // first safety circuit in sensor list
    sensorChannel_t safetyCircuitEnd = SENSOR_CH_SS_BOTS; // last safety circuit in sensor list
    const cmr_canWarn_t safetyCircuitWarns[] = { // in same order as in sensors.h to map them
        CMR_CAN_WARN_FSM_SS_MODULE,
        CMR_CAN_WARN_FSM_SS_COCKPIT,
        CMR_CAN_WARN_FSM_SS_FRHUB,
        CMR_CAN_WARN_FSM_SS_INERTIA,
        CMR_CAN_WARN_FSM_SS_FLHUB,
        CMR_CAN_WARN_FSM_SS_BOTS
    };

    cmr_canWarn_t warn = CMR_CAN_WARN_NONE;

    for (sensorChannel_t sensor = safetyCircuitStart; sensor <= safetyCircuitEnd; sensor++) {
        if (!cmr_sensorListGetValue(&sensorList, sensor)) {
            warn |= safetyCircuitWarns[sensor-safetyCircuitStart];
            break;
        }
    }
    return warn;
}

/**
 * @brief Sends raw pedal position sensor ADC values.
 *
 * @note This is only useful for calibration and should not be sent constantly.
 */
static void sendFSMPedalsADC(void) {
    cmr_canFSMPedalsADC_t msg = {
        .throttleLeftADC  = adcRead(sensorsADCChannels[SENSOR_CH_TPOS_L_U8]),
        .throttleRightADC = adcRead(sensorsADCChannels[SENSOR_CH_TPOS_R_U8]),
        .brakePedalADC    = adcRead(sensorsADCChannels[SENSOR_CH_BPOS_U8])
    };

    canTX(CMR_CANID_FSM_PEDALS_ADC, &msg, sizeof(msg), canTX10Hz_period_ms);
}

/**
 * @brief Sends raw sensor ADC values.
 *
 * @note This is only useful for calibration and should not be sent constantly.
 */
static void sendFSMSensorsADC(void) {
    cmr_canFSMSensorsADC_t msg = {
        .brakePressureFrontADC = adcRead(sensorsADCChannels[SENSOR_CH_BPRES_PSI]),
        .steeringWheelAngleADC = adcRead(sensorsADCChannels[SENSOR_CH_SWANGLE_DEG])
    };

    canTX(CMR_CANID_FSM_SENSORS_ADC, &msg, sizeof(msg), canTX10Hz_period_ms);
}

/**
 * @brief Sends latest bus voltage and current draw measurements.
 */
static void sendPowerDiagnostics(void) {
    // value * 0.8 (mV per bit) * 11 (1:11 voltage divider)
    uint32_t busVoltage_mV = cmr_sensorListGetValue(&sensorList, SENSOR_CH_VOLTAGE_MV);
    uint32_t busCurrent_mA = cmr_sensorListGetValue(&sensorList, SENSOR_CH_AVG_CURRENT_MA);

    cmr_canDIMPowerDiagnostics_t powerDiagnosticsDIM = {
        .busVoltage_mV = busVoltage_mV,
        .busCurrent_mA = busCurrent_mA
    };
    cmr_canFSMPowerDiagnostics_t powerDiagnosticsFSM = {
        .busVoltage_mV = busVoltage_mV,
        .busCurrent_mA = busCurrent_mA
    };

    canTX(
            CMR_CANID_DIM_POWER_DIAGNOSTICS,
            &powerDiagnosticsDIM, sizeof(powerDiagnosticsDIM),
            canTX10Hz_period_ms
        );
    canTX(
            CMR_CANID_FSM_POWER_DIAGNOSTICS,
            &powerDiagnosticsFSM, sizeof(powerDiagnosticsFSM),
            canTX10Hz_period_ms
        );
}
