/**
 * @file state.c
 * @brief DIM state implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "state.h"  // interface to implement

#include <FreeRTOS.h>  // FreeRTOS interface
#include <semphr.h>    // Semaphore interface
#include <stdbool.h>

#include "CMR/config.h"  // Board-specific flash interface
#include "can.h"         // can interface
#include "gpio.h"  // GPIO interface
#include "stdlib.h"

#define min(a, b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

/** @brief declaration of config screen variables */
extern volatile bool flush_config_screen_to_cdc;
/** @brief declaration of config screen variables */
volatile bool config_increment_up_requested = false;
/** @brief declaration of config screen variables */
volatile bool config_increment_down_requested = false;
/** @brief declaration of config screen variables */
volatile uint8_t config_paddle_left_request = 0;
/** @brief declaration of config screen variables */
volatile uint8_t config_paddle_right_request = 0;
/** @brief declaration of config screen variables */
volatile int8_t config_move_request = 0;
/** @brief declaration of what screen mode one is in */
volatile bool in_config_screen = false;
/** @brief declaration of what screen mode one is in */
volatile bool in_racing_screen = false;
/** @brief declaration of if the DIM is waiting for a new driver config */
volatile bool waiting_for_cdc_new_driver_config;
/** @brief declaration of if the DIM is waiting for a new driver config */
volatile bool exit_config_request = false;
/** @brief Checks to see if the screen has been setup before and if not will appropriately draw it */
volatile bool dim_first_time_config_screen;
/** @brief Checks to see if the screen needs to be redrawn after getting new driver profiles */
volatile bool redraw_new_driver_profiles;

/** @brief Push To Talk button value */
bool pttButtonPressed;
/** @brief Action button value */
bool actionButtonPressed;
/** @brief Acknowledge button value */
bool ackButtonPressed;
/** @brief Screen button value */
bool screenButtonPressed;
/** @brief Up button value */
bool actionUpButtonPressed;
/** @brief Down button value*/
bool actionDownButtonPressed;
/** @brief Left button value */
bool actionLeftButtonPressed;
/** @brief Right button value */
bool actionRightButtonPressed;
/** @brief rotatory values {ROT_2,ROT_1,ROT_0} */
uint8_t rotaryPos;
uint8_t switchValues;
uint8_t prevRotary = 0;
uint8_t prevSwitch = 0;
// Forward declarations
void stateVSMUp(void);
void stateVSMDown(void);
void enterConfigScreen(void);

static uint32_t test_message_id = 0;

uint32_t get_test_message_id() {
    return test_message_id;
}

void new_test_message_id() {
	srand(HAL_GetTick());
    test_message_id = (rand() % 0xFFFFFFFFu) & 0x0FFFFFFF;
}

void actionButton(bool pressed) {
    actionButtonPressed = pressed;  // True
    if (!pressed) {
        return;
    }

    if (inConfigScreen()) {
        config_increment_up_requested = true;
    }
}

/**
 * @brief handles Acknowledge Button button press on steering wheel
 *
 * @param pressed `true` if button is currently pressed.
 */
void ackButton(bool pressed) {
    ackButtonPressed = pressed;
    if (!pressed) {
        ackButtonPressed = false;
        return;
    }
    if (inConfigScreen()) {
        config_increment_down_requested = true;
        return;
    }

	if (!ackButtonPressed) {
		sendAcknowledgement();
	}
}

/**
 * @brief handles Screen (SCR) button press on steering wheel
 *
 * @param pressed `true` if button is currently pressed.
 */
void screenButton(bool pressed) {
    screenButtonPressed = pressed;
    if (!pressed) {
        return;
    }
    if (inConfigScreen()) {
        exit_config_request = true;
        exitConfigScreen();
        return;
    }
    in_racing_screen = false;
    // disabling racing screen bc its not working (comp change)
    // in_racing_screen = !inRacingScreen();
}

/**
 * @brief handles PTT button press on steering wheel
 *
 * @param pressed `true` if button is currently pressed.
 */
void pttButton(bool pressed) {
    pttButtonPressed = pressed;
    
}

/**
 * @brief handles UP button press on D-Pad
 *
 * @param pressed `true` if button is currently pressed.
 */

void upButton(bool pressed) {
    actionUpButtonPressed = pressed;
    if (!pressed) {
        return;
    }

    if (inConfigScreen()) {
        config_move_request = -CONFIG_SCREEN_NUM_COLS;
    } else {
        stateVSMUp();
    }
}

/**
 * @brief handles DOWN button press on D-Pad
 *
 * @param pressed `true` if button is currently pressed.
 */
void downButton(bool pressed) {
    actionDownButtonPressed = pressed;
    if (!pressed) {
        return;
    }

    if (inConfigScreen()) {
        config_move_request = CONFIG_SCREEN_NUM_COLS;
    } else {
        stateVSMDown();
    }
}

/**
 * @brief handles LEFT button press on D-Pad
 *
 * @param pressed `true` if button is currently pressed.
 */
void leftButton(bool pressed) {
    actionLeftButtonPressed = pressed;
    if (!pressed) {
        return;
    }

    if (inConfigScreen()) {
        config_move_request = -1;
    } else {
        // Enter config screen function does necesarry state checks
        enterConfigScreen();
    }
}

/**
 * @brief handles RIGHT button press on D-Pad
 *
 * @param pressed `true` if button is currently pressed.
 */
void rightButton(bool pressed) {
    actionRightButtonPressed = pressed;
    if (!pressed) {
        return;
    }

    if (inConfigScreen()) {
        config_move_request = 1;
    } else {
        new_test_message_id();
        // TODO: do stuff
    }
}

void rotaries(bool select, uint8_t pos) {
    // stabilizes the rotary switch inputs
    if (select) {
        if (prevSwitch == pos) {
            switchValues = pos;
        }
        prevSwitch = pos;
    } else {
        if (prevRotary == pos) {
            rotaryPos = pos;
        }
        prevRotary = pos;
    }
}

void exitConfigScreen() {
    // the first time the user presses the exit button, it'll flush the memory to the cdc
    // the second time it'll exit the config screen because it'll be dependent having
    // recieved the message from CDC
    if (!flush_config_screen_to_cdc) {
        flush_config_screen_to_cdc = true;
        waiting_for_cdc_to_confirm_config = true;
        return;
    }
    if (!waiting_for_cdc_to_confirm_config) {
        in_config_screen = false;
        exit_config_request = false;
        return;
    }
}

void enterConfigScreen() {
    // make sure you've booted and you can enter by seeing if
    // waiting for cdc

    if (in_config_screen == false &&
        config_screen_values_received_on_boot &&
        ((stateGetVSM() == CMR_CAN_GLV_ON && stateGetVSMReq() == CMR_CAN_GLV_ON) ||
         (stateGetVSM() == CMR_CAN_HV_EN && stateGetVSMReq() == CMR_CAN_HV_EN))) {
        in_config_screen = true;
        dim_first_time_config_screen = true;
    }
}

bool inConfigScreen() {
    return in_config_screen;
}

bool inRacingScreen() {
    return in_racing_screen;
}

/** @brief DIM state. */
static volatile struct {
    cmr_canState_t vsmReq;    /**< @brief Requested VSM state. */
    cmr_canGear_t gear;       /**< @brief Current gear. */
    cmr_canGear_t gearReq;    /**< @brief Requested gear. */
    cmr_canDrsMode_t drsMode; /**< @brief Current DRS Mode. */
    cmr_canDrsMode_t drsReq;  /**< @brief Requested DRS Mode. */
} state = {
    .vsmReq = CMR_CAN_GLV_ON,
    .gear = CMR_CAN_GEAR_SLOW,
    .gearReq = CMR_CAN_GEAR_SLOW,
    .drsMode = CMR_CAN_DRSM_CLOSED,
    .drsReq = CMR_CAN_DRSM_CLOSED
};

/**
 * @brief Gets the VSM state.
 *
 * @note VSM state is maintained in the received CAN heartbeat.
 *
 * @return The VSM state.
 */
cmr_canState_t stateGetVSM(void) {
    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
    volatile cmr_canHeartbeat_t *heartbeatVSM =
        (void *)heartbeatVSMMeta->payload;

    return heartbeatVSM->state;
}

/**
 * @brief Gets the requested VSM state.
 *
 * @return The requested VSM state.
 */
cmr_canState_t stateGetVSMReq(void) {
    return state.vsmReq;
}

/**
 * @brief Gets the current gear.
 *
 * @return The current gear.
 */
cmr_canGear_t stateGetGear(void) {
    return state.gear;
}

/**
 * @brief Gets the requested gear.
 *
 * @return The requested gear.
 */
cmr_canGear_t stateGetGearReq(void) {
    return state.gearReq;
}

cmr_canDrsMode_t stateGetDrs(void) {
    return state.drsMode;
}

cmr_canDrsMode_t stateGetDrsReq(void) {
    return state.drsReq;
}

bool getAcknowledgeButton(void) {
    return ackButtonPressed;
}

/**
 * @brief Gets the average wheel speed reported by the inverters.
 *
 * @param none
 *
 * @return average wheel speed
 */
int32_t getAverageWheelRPM(void) {
    /* Get CAN data */
    // Front Left
    cmr_canRXMeta_t *metaAMK_FL_Act1 = canRXMeta + CANRX_AMK_FL_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_FL_Act1 =
        (void *)metaAMK_FL_Act1->payload;
    // Front Right
    cmr_canRXMeta_t *metaAMK_FR_Act1 = canRXMeta + CANRX_AMK_FR_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_FR_Act1 =
        (void *)metaAMK_FR_Act1->payload;
    // Rear Left
    cmr_canRXMeta_t *metaAMK_RL_Act1 = canRXMeta + CANRX_AMK_RL_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_RL_Act1 =
        (void *)metaAMK_RL_Act1->payload;
    // Rear Right
    cmr_canRXMeta_t *metaAMK_RR_Act1 = canRXMeta + CANRX_AMK_RR_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_RR_Act1 =
        (void *)metaAMK_RR_Act1->payload;

    /* Extract wheel speeds */
    int32_t frontLeftRPM = (canAMK_FL_Act1->velocity_rpm);  // Motor direction reversed on left side
    int32_t frontRightRPM = canAMK_FR_Act1->velocity_rpm;
    int32_t rearLeftRPM = (canAMK_RL_Act1->velocity_rpm);  // Motor direction reversed on left side
    int32_t rearRightRPM = canAMK_RR_Act1->velocity_rpm;

    /* Compute average */
    int32_t average = (frontLeftRPM + frontRightRPM + rearLeftRPM + rearRightRPM) / 4;

    return average;
}

/**
 * @brief Checks if vehicle is slow enough to request state down during RTD.
 *        This intends to stop accidental state down requests during driving.
 *
 * @param none
 *
 * @return If the car is slow enough to state down from rtd
 */
bool slowEnough(void) {
    int32_t avgWheelRPM = getAverageWheelRPM();

    uint16_t cutoffMph = 3;

    /* Wheel Speed to Vehicle Speed Conversion
     *      (x rotations / 1min) * (18" * PI) * (1' / 12") *
     *      (60min / 1hr) * (1 mi / 5280')
     *      = x * 0.0535                                   */
    uint16_t vehicleSpeed = avgWheelRPM * 0.0535;

    return vehicleSpeed < cutoffMph;
}

/**
 * @brief Checks if the requested VSM state is allowed.
 *
 * @param vsm The current VSM state.
 * @param vsmReq The requested VSM state.
 */
bool stateVSMReqIsValid(cmr_canState_t vsm, cmr_canState_t vsmReq) {
    switch (vsm) {
        case CMR_CAN_UNKNOWN:
            return (vsmReq == CMR_CAN_GLV_ON);
        case CMR_CAN_GLV_ON:
            return (vsmReq == CMR_CAN_GLV_ON) ||
                   (vsmReq == CMR_CAN_HV_EN);
        case CMR_CAN_HV_EN:
            return (vsmReq == CMR_CAN_GLV_ON) ||
                   (vsmReq == CMR_CAN_HV_EN) ||
                   (vsmReq == CMR_CAN_RTD);
        case CMR_CAN_RTD:
            return ((vsmReq == CMR_CAN_HV_EN)) ||
                   (vsmReq == CMR_CAN_RTD);
        case CMR_CAN_ERROR:
            return (vsmReq == CMR_CAN_GLV_ON);
        case CMR_CAN_CLEAR_ERROR:
            return (vsmReq == CMR_CAN_GLV_ON);
        default:
            break;
    }

    return false;
}

/**
 * @brief Handles VSM state up.
 */
void stateVSMUp() {
    cmr_canState_t vsmState = stateGetVSM();
    if (state.vsmReq < vsmState) {
        // Cancel state-down request.
        state.vsmReq = vsmState;
        return;
    }

    cmr_canState_t vsmReq = ((vsmState == CMR_CAN_UNKNOWN) || (vsmState == CMR_CAN_ERROR))
                                ? (CMR_CAN_GLV_ON)  // Unknown state; request GLV_ON.
                                : (vsmState + 1);   // Increment state.
    if (!stateVSMReqIsValid(vsmState, vsmReq)) {
        return;  // Invalid requested state.
    }

    state.vsmReq = vsmReq;
}

/**
 * @brief Handles VSM state down request.
 */
void stateVSMDown() {
    cmr_canState_t vsmState = stateGetVSM();
    if (state.vsmReq > vsmState) {
        // Cancel state-up request.
        state.vsmReq = vsmState;
        return;
    }

    if (
        state.vsmReq == CMR_CAN_RTD &&
        getAverageWheelRPM() > 5) {
        // Only exit RTD when motor is basically stopped.
        return;
    }

    cmr_canState_t vsmReq = vsmState - 1;  // Decrement state.
    // Valid State
    if (stateVSMReqIsValid(vsmState, vsmReq)) {
        state.vsmReq = vsmReq;
        return;
    }
}





/**
 * @brief Updates state request to be consistent with VSM state.
 */
void updateReq(void) {
    state.vsmReq = stateGetVSM();
}

/**
 * @brief Updates the gear to be the requested gear.
 */
void stateGearUpdate(void) {
    state.gear = state.gearReq;
}

void stateDrsUpdate(void) {
    state.drsMode = state.drsReq;
}




/**
 * @brief Returns the current car's speed in km/h
 */
float getSpeedKmh() {
    int32_t avgWheelRPM = getAverageWheelRPM();

    /* Wheel Speed to Vehicle Speed Conversion
     *      (x rotations / 1min) * (18" * PI) *  (2.54*10^-5km/inch)
     *      (60min / 1hr) * (1/15.1 gear ratio)
     *      = x * 0.0057072960048526627892388896218624717297547517194475432371                                 */
    float vehicleSpeed = (float)avgWheelRPM * 0.0057073;

    return vehicleSpeed;
}

/**
 * @brief Returns the current car's odometry in km
 */
float getOdometer() {
    volatile cmr_canCDCOdometer_t *odometer = (volatile cmr_canCDCOdometer_t *)getPayload(CANRX_CDC_ODOMETER);
    return odometer->odometer_km;
}

/**
 * @brief Struct for voltage SoC lookup table
 */
typedef struct {
    float voltage;
    uint8_t SoC;
} voltage_SoC_t;

#define LV_LIFEPO_LUT_NUM_ITEMS 11
#define LV_LIPO_LUT_NUM_ITEMS 6

/**
 * @brief Look up table for 24v lifepo4 battery State of Charge
 *
 * Must be sorted in descending order
 */
static voltage_SoC_t LV_LiFePo_SoC_lookup[LV_LIFEPO_LUT_NUM_ITEMS] = {
    { 27.2f, 100 },
    { 26.8f, 90 },
    { 26.6f, 80 },
    { 26.1f, 70 },
    { 26.4f, 60 },
    { 26.1f, 50 },
    { 26.0f, 40 },
    { 25.8f, 30 },
    { 25.6f, 20 },
    { 24.0f, 10 },
    { 20.0f, 0 }
};

static voltage_SoC_t LV_LiPo_SoC_lookup[LV_LIPO_LUT_NUM_ITEMS] = {
    { 25.2f, 100 },
    { 24.5f, 90 },
    { 23.0f, 80 },
    { 21.0f, 20 },
    { 20.0f, 10 },
    { 18.0f, 0 }
};

/**
 * @brief Function for getting Low Voltage SoC
 *
 * @param voltage the current LV Voltage.
 *
 * @return the state of charge % between 0 and 99.
 */
uint8_t getLVSoC(float voltage, lv_battery_type_t battery_type) {
    voltage_SoC_t *lut;
    size_t num_items;

    if (battery_type == LV_LIFEPO) {
        lut = LV_LiFePo_SoC_lookup;
        num_items = LV_LIFEPO_LUT_NUM_ITEMS;
    } else if (battery_type == LV_LIPO) {
        lut = LV_LiPo_SoC_lookup;
        num_items = LV_LIPO_LUT_NUM_ITEMS;
    } else {
        // unknown battery type - return 0%
        return 0;
    }

    for (size_t i = 0; i < num_items; i++) {
        if (lut[i].voltage == voltage) {
            // if voltage equals voltage from lut, return soc
            return lut[i].SoC;
        } else if (lut[i].voltage < voltage) {
            // if voltage > voltage from lut, we have passed correct value
            if (i == 0) {
                // if i == 0, then it must be higher than highest voltage
                return 99;
            } else {
                // otherwise we do some linear extrapolation!
                float result = (float)lut[i].SoC + ((voltage - lut[i].voltage) / (lut[i - 1].voltage - lut[i].voltage)) * ((float)(lut[i - 1].SoC - lut[i].SoC));
                return min(99, ((uint8_t)result));
            }
        }
    }
    // if we get to end of loop, voltage is less than lowest voltage in lut
    return 0;
}
