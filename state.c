/**
 * @file state.c
 * @brief DIM state implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "state.h"  // interface to implement
#include "gpio.h"   // GPIO interface
#include "can.h"    // can interface
#include "stdlib.h"
#include "expanders.h"
#include <stdbool.h>

#include "CMR/config.h"     // Board-specific flash interface
#include <FreeRTOS.h>       // FreeRTOS interface
#include <semphr.h>         // Semaphore interface

/** @brief Update Odometer task. */
static cmr_task_t odometer_task;
/** @brief Odometer update priority. */
static const uint32_t odometer_priority = 2;
/** @brief declaration of odometer update task */
static const TickType_t odometer_period_ms = 100;

/**@brief Some string to distinguish first-time-boots */
#define CANARY "\xbd\xa6\x27\x59\xe2\xcf\x25\x88\x95\x24\xed\xee"

/** @brief Odometer variable */
typedef struct {
    float odometer_km;
    uint8_t canary[12];
} odometer_t;
static odometer_t odometer;

/**@brief flash driver wrapping the settings */
static cmr_config_t cfg;

/**@brief Index of the sector to use */
static const uint32_t sector_id = FLASH_SECTOR_15;
static SemaphoreHandle_t cfg_lock;
static StaticSemaphore_t _cfg_lock_buf;

/** @brief declaration of config screen variables */
extern volatile bool flush_config_screen_to_cdc;
/** @brief declaration of config screen variables */
volatile bool config_increment_up_requested = false;
/** @brief declaration of config screen variables */
volatile bool config_increment_down_requested = false;
/** @brief declaration of config screen variables */
volatile uint8_t config_paddle_left_request = 0;
volatile uint8_t config_paddle_right_request = 0;
/** @brief declaration of config screen variables */
volatile int8_t config_move_request = 0;
/** @brief declaration of what screen mode one is in */
volatile bool in_config_screen = false;
/** @brief decleration of if the DIM is waiting for a new driver config */
volatile bool waiting_for_cdc_new_driver_config;
/** @brief decleration of if the DIM is waiting for a new driver config */
volatile bool exit_config_request = false;
/** @brief Checks to see if the screen has been setup before and if not will appropriately draw it */
volatile bool dim_first_time_config_screen;
/** @brief Checks to see if the screen needs to be redrawn after getting new driver profiles */
volatile bool redraw_new_driver_profiles;

/** @brief AE/DRS button value */
bool drsButtonPressed;
/** @brief Action 1 button value */
bool action1ButtonPressed;
/** @brief Action 2 button value */
bool action2ButtonPressed;

// Forward declarations
void stateVSMUp(void);
void stateVSMDown(void);
void enterConfigScreen(void);


void actionOneButton(bool pressed) {
    if (!pressed) {
        action1ButtonPressed = false;
        return;
    } else {
        if (inConfigScreen()) {
		    config_increment_down_requested = true;
        } else {
            // only set can message to true if we're not in config screen
            action1ButtonPressed = pressed;
        }
    }
    
}

void actionTwoButton(bool pressed) {
    if (!pressed) {
        action2ButtonPressed = false;
        return;
    } else {
        if (inConfigScreen()) {
            exit_config_request = true;
            exitConfigScreen();
            return;
        } else {
            // only set can message to true if we're not in config screen
            action2ButtonPressed = pressed;
        }
    }
}

void drsButton(bool pressed) {
    if (!pressed) {
        drsButtonPressed = false;
        return;
    } else {
        if (inConfigScreen()) {
            config_increment_up_requested = true;
        } else {
            // only set can message to true if we're not in config screen
            drsButtonPressed = pressed;
        }
    }
}

/**
 * @brief handles UP button press on D-Pad
 * 
 * @param pressed `true` if button is currently pressed.
*/

void upButton(bool pressed) {
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
    if (!pressed) {
        return;
    }

    if (inConfigScreen()) {
        config_move_request = 1;
    } else {
        // TODO: do stuff
    }
}

void exitConfigScreen(){
    // the first time the user presses the exit button, it'll flush the memory to the cdc
    // the second time it'll exit the config screen because it'll be dependent having 
    // recieved the message from CDC
    if (flush_config_screen_to_cdc == false){
        flush_config_screen_to_cdc = true;
        waiting_for_cdc_to_confirm_config = true;
        return;
    }
    if (waiting_for_cdc_to_confirm_config == false){
        in_config_screen = false;
        exit_config_request = false;
    }
}

void enterConfigScreen(){
    // make sure you've booted and you can enter by seeing if
    // waiting for cdc


    if (in_config_screen == false && 
    config_screen_values_received_on_boot && 
    ((stateGetVSM() == CMR_CAN_GLV_ON && stateGetVSMReq() == CMR_CAN_GLV_ON) ||
    (stateGetVSM() == CMR_CAN_HV_EN && stateGetVSMReq() == CMR_CAN_HV_EN))){
        in_config_screen = true;
        dim_first_time_config_screen = true;
    }
}

bool inConfigScreen() {
    return in_config_screen;
}

/** @brief DIM state. */
static volatile struct {
    cmr_canState_t vsmReq;      /**< @brief Requested VSM state. */
    cmr_canGear_t gear;         /**< @brief Current gear. */
    cmr_canGear_t gearReq;      /**< @brief Requested gear. */
    cmr_canDrsMode_t drsMode;   /**< @brief Current DRS Mode. */
    cmr_canDrsMode_t drsReq;    /**< @brief Requested DRS Mode. */
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
        (void *) heartbeatVSMMeta->payload;

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

cmr_canDRSMode_t stateGetDrs(void) {
    return state.drsMode;
}

cmr_canDRSMode_t stateGetDrsReq(void) {
    return state.drsReq;
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
        (void *) metaAMK_FL_Act1->payload;
    // Front Right
    cmr_canRXMeta_t *metaAMK_FR_Act1 = canRXMeta + CANRX_AMK_FR_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_FR_Act1 =
        (void *) metaAMK_FR_Act1->payload;
    // Rear Left
    cmr_canRXMeta_t *metaAMK_RL_Act1 = canRXMeta + CANRX_AMK_RL_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_RL_Act1 =
        (void *) metaAMK_RL_Act1->payload;
    // Rear Right
    cmr_canRXMeta_t *metaAMK_RR_Act1 = canRXMeta + CANRX_AMK_RR_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_RR_Act1 =
        (void *) metaAMK_RR_Act1->payload;

    /* Extract wheel speeds */
    int32_t frontLeftRPM = -1*(canAMK_FL_Act1->velocity_rpm); // Motor direction reversed on left side
    int32_t frontRightRPM = canAMK_FR_Act1->velocity_rpm;
    int32_t rearLeftRPM = -1*(canAMK_RL_Act1->velocity_rpm); // Motor direction reversed on left side
    int32_t rearRightRPM = canAMK_RR_Act1->velocity_rpm;

    /* Compute average */
    int32_t average = (frontLeftRPM + frontRightRPM + rearLeftRPM + rearRightRPM)/4;

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

    uint16_t cutoff = 3; //mph

    /* Wheel Speed to Vehicle Speed Conversion
     *      (x rotations / 1min) * (18" * PI) * (1' / 12") * 
     *      (60min / 1hr) * (1 mi / 5280')
     *      = x * 0.0535                                   */
    uint16_t vehicleSpeed = avgWheelRPM * 0.0535;

    return abs(vehicleSpeed) < cutoff;
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
        : (vsmState + 1);        // Increment state.
    if (!stateVSMReqIsValid(vsmState, vsmReq)) {
        return;     // Invalid requested state.
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
        getAverageWheelRPM() > 5
    ) {
        // Only exit RTD when motor is basically stopped.
        return;
    }

    cmr_canState_t vsmReq = vsmState - 1;   // Decrement state.
    if (!stateVSMReqIsValid(vsmState, vsmReq)) {
        return;     // Invalid requested state.
    }

    state.vsmReq = vsmReq;
}

void stateGearSwitch(expanderRotaryPosition_t position) {
    if ((stateGetVSM() != CMR_CAN_HV_EN) && (stateGetVSM() != CMR_CAN_GLV_ON)) {
        return;     // Can only change gears in HV_EN and GLV_ON.
    }
    cmr_canGear_t gearReq;
    if (position == ROTARY_POS_INVALID) {
        gearReq = CMR_CAN_GEAR_SLOW;
    } else {
        gearReq = (cmr_canGear_t)((size_t) position + 1);
        if (gearReq >= CMR_CAN_GEAR_LEN) {
            gearReq = CMR_CAN_GEAR_SLOW;
        }
    }

    state.gearReq = gearReq;
}

void stateDrsModeSwitch(expanderRotaryPosition_t position) {
    // we can change DRS in any state

    if (position == ROTARY_POS_INVALID) {
        state.drsReq = CMR_CAN_DRSM_UNKNOWN;
    } else if ((cmr_canDrsMode_t) position >= CMR_CAN_DRSM_LEN) { 
        // set drs mode to closed if dial pos > drs modes
        state.drsReq = CMR_CAN_DRSM_CLOSED;
    } else {
        state.drsReq = position;
    }
}

void stateRotary2Switch(expanderRotaryPosition_t position) {
    // Not implemented
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

// Called by CAN 100 Hz
uint8_t getLeftPaddleState() {
    uint8_t pos = (uint8_t) (((float) expanderGetClutch(EXP_CLUTCH_1)) / ((float) (0xFFF)) * ((float) UINT8_MAX));
    if (inConfigScreen()) {
        if (pos > MIN_PADDLE_VAL) {
            config_paddle_left_request = pos - MIN_PADDLE_VAL;
        } else {
            config_paddle_left_request= 0;
        }
        // Return 0 to CAN because we are in config screen
        return 0;
    } else {
        // only send paddle state if we're not in config screen
        return pos;
    }
}

// Called by CAN 100 Hz
uint8_t getRightPaddleState() {
    uint8_t pos = (uint8_t) (((float) expanderGetClutch(EXP_CLUTCH_2)) / ((float) (0xFFF)) * ((float) UINT8_MAX));
    if (inConfigScreen()) {
        if (pos > MIN_PADDLE_VAL) {
            config_paddle_right_request = pos - MIN_PADDLE_VAL;
        } else {
            config_paddle_right_request= 0;
        }
        // Return 0 to CAN because we are in config screen
        return 0;
    } else {
        // only send paddle state if we're not in config screen
        return pos;
    }
}

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
    return odometer.odometer_km;
}

/**
 * @brief Writes the current odometer to flash
*/
static void commitOdometer() {
    // xSemaphoreTake(cfg_lock, portMAX_DELAY);
    memcpy(odometer.canary, CANARY, sizeof(odometer.canary));
    taskENTER_CRITICAL();
    cmr_configCommit(&cfg);
    taskEXIT_CRITICAL();
    // xSemaphoreGive(cfg_lock);
}

static void updateOdometer(void *pvParameters) {
    (void) pvParameters;
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (true) {
        float speed_kmph = getSpeedKmh();
        // dist = x kmph * (1 hr/3600sec) * (20ms/ (1000ms/sec))
        odometer.odometer_km += (speed_kmph / (60.0f * 60.0f)) * (((float) odometer_period_ms) / 1000.0f);
        commitOdometer();
        vTaskDelayUntil(&lastWakeTime, odometer_period_ms);
    }
}

/**
 * @brief Task for updating odometer.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
void odometerInit(void) {
    /** Initialize flash cache, pull in current settings,
     * and perform first-boot-time initialization.
     */
    cmr_configInit(
        &cfg,
        (volatile uint32_t *) &odometer,
        sizeof(odometer) / sizeof(uint32_t),    /* Flash driver expects
                                                         * a size in words,
                                                         * sadly */
        sector_id
    );

    cfg_lock = xSemaphoreCreateBinaryStatic(&_cfg_lock_buf);
    xSemaphoreGive(cfg_lock);
    configASSERT(cfg_lock);

    /* Do a vaguely sketchy uninitialized flash check to
     * set defaults on the first boot */
    if (memcmp(odometer.canary, CANARY, sizeof(odometer.canary))) {
        odometer.odometer_km = 0.0f;
        commitOdometer();
    }

    cmr_taskInit(
        &odometer_task,
        "odometer",
        odometer_priority,
        updateOdometer,
        NULL
    );
}


typedef struct {
    float voltage;
    uint8_t SoC;
} voltage_SoC_t;

#define LV_LUT_NUM_ITEMS 11

// look up table must be sorted in descending order
static const voltage_SoC_t LV_SoC_lookup[LV_LUT_NUM_ITEMS] = {
    {27.2, 100},
    {26.8, 90},
    {26.6, 80},
    {26.1, 70},
    {26.4, 60},
    {26.1, 50},
    {26.0, 40},
    {25.8, 30},
    {25.6, 20},
    {24.0, 10},
    {10.0, 0}
};

uint8_t getLVSoC(float voltage) {
    for (size_t i = 0; i < LV_LUT_NUM_ITEMS; i++) {
        if (LV_SoC_lookup[i].voltage == voltage) {
            // if voltage equals voltage from lut, return soc
            return LV_SoC_lookup[i].SoC;
        } else if (LV_SoC_lookup[i].voltage > voltage) {
            // if voltage > voltage from lut, we have passed correct value
            if (i == 0) {
                // if i == 0, then it must be higher than highest voltage
                return 100;
            } else {
                // otherwise we do some linear extrapolation! 
                return LV_SoC_lookup[i].SoC + 
                       ((voltage - LV_SoC_lookup[i].voltage) / (LV_SoC_lookup[i-1].voltage - LV_SoC_lookup[i].voltage)) * 
                       (LV_SoC_lookup[i-1].SoC - LV_SoC_lookup[i].SoC);
            }
        }
    }
    // if we get to end of loop, voltage is less than lowest voltage in lut
    return 0;

}