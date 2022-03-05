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

#include "can.h"    // Interface to implement
#include "adc.h"    // adcVSense, adcISense
#include "state.h"  // State interface
#include "tftDL.h"  // For RAM buffer indices
#include "gpio.h"   // For actionButtonPressed status

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
    [CANRX_AMK_FL_ACT_1] = {
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
        .canID = CMR_CANID_PTCf_LOOP_TEMPS_A,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_PTCf_LOOP_B_TEMPS] = {
        .canID = CMR_CANID_PTCf_LOOP_TEMPS_B,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_PTCp_LOOP_A_TEMPS] = {
        .canID = CMR_CANID_PTCp_LOOP_TEMPS_A,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_PTCp_LOOP_B_TEMPS] = {
        .canID = CMR_CANID_PTCp_LOOP_TEMPS_B,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_HEARTBEAT] = {
        .canID = CMR_CANID_HEARTBEAT_HVC,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_CDC_MOTOR_FAULTS] = {
        .canID = CMR_CANID_CDC_MOTOR_FAULTS,
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
    while (1) {
        /* Transmit Power Diagnostics */
        cmr_canDIMPowerDiagnostics_t powerDiagnostics = {
            .busVoltage_mV = adcRead(ADC_VSENSE) * 8 * 11 / 10, // TODO: figure out where 8, 10 come from
            .busCurrent_mA = adcRead(ADC_ISENSE) * 8 / 20 / 10 // TODO: figure out where 8, 10 come from
        };

        canTX(
            CMR_CANID_DIM_POWER_DIAGNOSTICS,
            &powerDiagnostics, sizeof(powerDiagnostics),
            canTX10Hz_period_ms
        );

        /* if DIM is requesting a state/gear change
         * send this request to VSM */
        cmr_canState_t stateVSM = stateGetVSM();
        cmr_canState_t stateVSMReq = stateGetVSMReq();
        cmr_canGear_t gear = stateGetGear();
        cmr_canGear_t gearReq = stateGetGearReq();

        if (
            (stateVSM != stateVSMReq) ||
            (gear != gearReq)
        ) {
            cmr_canDIMRequest_t request = {
                .requestedState = stateVSMReq,
                .requestedGear = gearReq
            };
            canTX(
                CMR_CANID_DIM_REQUEST,
                &request, sizeof(request),
                canTX10Hz_period_ms
            );

            stateGearUpdate();
        }

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

    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        /* Transmit DIM heartbeat */
        cmr_canState_t vsmState = stateGetVSM();
        cmr_canHeartbeat_t heartbeat = {
            .state = vsmState
        };

        uint16_t error = CMR_CAN_ERROR_NONE;
        if (cmr_canRXMetaTimeoutError(heartbeatVSMMeta, lastWakeTime) < 0) {
            error |= CMR_CAN_ERROR_VSM_TIMEOUT;
        }
        memcpy(&heartbeat.error, &error, sizeof(error));

        uint16_t warning = CMR_CAN_WARN_NONE;
        if (cmr_canRXMetaTimeoutWarn(heartbeatVSMMeta, lastWakeTime) < 0) {
            warning |= CMR_CAN_WARN_VSM_TIMEOUT;
        }
        memcpy(&heartbeat.warning, &warning, sizeof(warning));

        canTX(
            CMR_CANID_HEARTBEAT_DIM,
            &heartbeat,
            sizeof(heartbeat),
            canTX100Hz_period_ms
        );

        /* Transmit action button status */
        cmr_canDIMActionButton_t actionButton = {
            .actionButtonPressed = actionButtonPressed
        };
        canTX(
            CMR_CANID_DIM_ACTION_BUTTON,
            &actionButton, sizeof(actionButton),
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

    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        if (flush_config_screen_to_cdc){
            /* pack struct message for config */
            cmr_canDIMCDCconfig_t config0 = {
                .config_val_1 = config_menu_main_array[0].value.value,
                .config_val_2 = config_menu_main_array[1].value.value,
                .config_val_3 = config_menu_main_array[2].value.value,
                .config_val_4 = config_menu_main_array[3].value.value,
            };
            cmr_canDIMCDCconfig_t config1 = {
                .config_val_1 = config_menu_main_array[4].value.value,
                .config_val_2 = config_menu_main_array[5].value.value,
                .config_val_3 = config_menu_main_array[6].value.value,
                .config_val_4 = config_menu_main_array[7].value.value,
            };
            cmr_canDIMCDCconfig_t config2 = {
                .config_val_1 = config_menu_main_array[8].value.value,
                .config_val_2 = config_menu_main_array[9].value.value,
                .config_val_3 = config_menu_main_array[10].value.value,
                .config_val_4 = config_menu_main_array[11].value.value,
            };
            cmr_canDIMCDCconfig_t config3 = {
                .config_val_1 = config_menu_main_array[12].value.value,
                .config_val_2 = config_menu_main_array[13].value.value,
                .config_val_3 = config_menu_main_array[14].value.value,
                .config_val_4 = config_menu_main_array[15].value.value,
            };
            cmr_canDIMCDCconfig_t config4 = {
                .config_val_1 = config_menu_main_array[16].value.value,
                .config_val_2 = 0,
                .config_val_3 = 0,
                .config_val_4 = 0,
            };

            cmr_canDIMCDCconfig_t config_message_array[num_config_packets] = {
                config0,
                config1,
                config2,
                config3,
                config4
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
               waiting_for_cdc_to_confirm_config = true;
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
 * Character indices 80-92 inclusive are for the third note on the right side of the screen.
 * This corresponds to text->address 0x14, 0x15, and 0x16.
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
                    if (i == NOTE1_INDEX || i == NOTE2_INDEX || i == NOTE3_INDEX) {
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
    int packet_number = (canID - CMR_CANID_DIM_CONFIG0_DRV0) % num_config_packets;
    // cast the data to the appropriate format
    cmr_canDIMCDCconfig_t *cdc_config_data = (cmr_canDIMCDCconfig_t*) data;
    // cast the data to an array for easy indexing. Sly i know :P
    uint8_t *cdc_config_data_arr = (uint8_t*) cdc_config_data;

    // find the appropriate values to modify in the local copy of our data 
    // note that there are 4 values per config struct hence the *4
    uint8_t dim_config_data_array_starting_idx = packet_number * items_per_struct; 

    if (packet_number >= num_config_packets){
        // time to shit yo pants bc some wack shit has happened.
        // TODO: Add throwing an error here
        while(1);
    }

    /**** Cold Boot, await default parameters-- blindly read data if it's the first driver and set that to done ****/
    if (!initialized){
        if (((uint32_t)canID - CMR_CANID_CDC_CONFIG0_DRV0) < num_config_packets){
            // copy data over to local memory!
            for(uint8_t i = dim_config_data_array_starting_idx; i < dim_config_data_array_starting_idx + 4; i++){
                config_menu_main_array[i].value.value = cdc_config_data_arr[i];
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
        if (((uint32_t)canID - requested_driver_cdc_canid) < num_config_packets){
            bool all_data_matches = true;

            // get the data and check if all the data is the same
            for(uint8_t i = dim_config_data_array_starting_idx; i < dim_config_data_array_starting_idx + 4; i++){
                all_data_matches &= config_menu_main_array[i].value.value == cdc_config_data_arr[i];
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
        if (((uint32_t)canID - requested_driver_cdc_canid) < num_config_packets){
            // get the data and flush it to local memory
            for(uint8_t i = dim_config_data_array_starting_idx; i < dim_config_data_array_starting_idx + 4; i++){
                config_menu_main_array[i].value.value = cdc_config_data_arr[i];
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
        if (waiting_for_cdc_to_confirm_config) waiting_for_cdc_to_confirm_config = false;
        if (flush_config_screen_to_cdc) flush_config_screen_to_cdc = false; 
        if (waiting_for_cdc_new_driver_config) waiting_for_cdc_new_driver_config = false;


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
        canID <= CMR_CANID_CDC_CONFIG4_DRV4){ 
    	cdcRXCallback(can, canID, data, dataLen);
    }
}


/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &can, CAN2, CMR_CAN_BITRATE_500K,
        canRXMeta, sizeof(canRXMeta) / sizeof(canRXMeta[0]),
        &canRXCallback,
        GPIOB, GPIO_PIN_12,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_13      // CAN2 TX port/pin.
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
                CMR_CANID_CDC_MOTOR_FAULTS,
                CMR_CANID_HEARTBEAT_HVC
            }
        },
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_SBG_STATUS_3,
                CMR_CANID_SBG_STATUS_3,
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
                CMR_CANID_PTCf_LOOP_TEMPS_A,
                CMR_CANID_PTCf_LOOP_TEMPS_B,
                CMR_CANID_PTCp_LOOP_TEMPS_A,
                CMR_CANID_PTCp_LOOP_TEMPS_B
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
                CMR_CANID_CDC_CONFIG4_DRV0,
                CMR_CANID_CDC_CONFIG4_DRV1,
                CMR_CANID_CDC_CONFIG4_DRV2,
                CMR_CANID_CDC_CONFIG4_DRV3
            }
        },
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
