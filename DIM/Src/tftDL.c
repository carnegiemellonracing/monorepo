/**
 * @file tftDL.c
 * @brief TFT display list implementation.
 *
 * @author Carnegie Mellon Racing
 */



#include <stdio.h>   // snprintf
#include <string.h>  // memcpy()

#include "adc.h"         // GLV voltage
#include "can.h"         // Board-specific CAN interface
#include "gpio.h"        // Board-specific CAN interface
#include "state.h"       // State interface
#include "tftContent.h"  // Content interface
#include "tftDL.h"  // Interface to implement
#include "tft.h"

// used to calculate increment frequency:
// max paddle val (255) / max increment speed (20Hz)
static const float paddle_time_scale_factor = 255.0f / 20.0f;

/** @brief Represents a display list. */
struct tftDL {
    size_t len;           /**< @brief Length of the display list, in bytes. */
    const uint32_t *data; /**< @brief The display list data. */

    size_t contentLen;            /**< @brief Number of content items. */
    const tftContent_t **content; /**< @brief Associated content. */
};

/** @brief Raw startup screen */
static uint32_t tftDL_startupData[] = {
#include <DIM-ESE/startup.rawh>
};

/** @brief Packets to send to the DL on startup.
 * See datasheet */
static const tftContent_t *tftDL_startupContent[] = {
    &tftContent_startup_lut,
    &tftContent_startup
};

/** @brief Complete data required to draw the startup screen.
 * Exposed to interface consumers. */
const tftDL_t tftDL_startup = {
    .len = sizeof(tftDL_startupData),
    .data = tftDL_startupData,

    .contentLen = sizeof(tftDL_startupContent) / sizeof(tftDL_startupContent[0]),
    .content = tftDL_startupContent
};

/** @brief GLV Screen */
static uint32_t tftDL_errorData[] = {
#include <DIM-ESE/errors.rawh>
};

/** @brief Complete data required to draw the error screen.
 * Exposed to interface consumers. */
const tftDL_t tftDL_error = {
    .len = sizeof(tftDL_errorData),
    .data = tftDL_errorData,

    .contentLen = 0,
    .content = NULL
};

/** @brief Config Screen */
static uint32_t tftDL_configData[] = {
#include <DIM-ESE/config.rawh>
};

/** @brief Complete data required to draw the error screen.
 * Exposed to interface consumers. */
const tftDL_t tftDL_config = {
    .len = sizeof(tftDL_configData),
    .data = tftDL_configData,
    .contentLen = 0,
    .content = NULL
};

/** @brief RTD Screen
 *
 *  #include is effectiveley copy and paste. Creates a well formed array of uint32_t
 */
static uint32_t tftDL_RTDData[] = {
#include <DIM-ESE/RTD.rawh>
};

/** @brief Complete data required to draw the
 * Ready-to-drive screen.
 * Exposed to interface consumers. */
const tftDL_t tftDL_RTD = {
    .len = sizeof(tftDL_RTDData),
    .data = tftDL_RTDData,

    .contentLen = 0,
    .content = NULL
};

/** @brief Racing Screen */
static uint32_t tftDL_racingData[] = {
#include <DIM-ESE/racing-screen.rawh>
};
/** @brief Complete data required to draw the
 * Racing Screen
 * Exposed to interface consumers. */
const tftDL_t tftDL_racing_screen = {
    .len = sizeof(tftDL_racingData),
    .data = tftDL_racingData,

    .contentLen = 0,
    .content = NULL
};

static uint32_t tftDl_safetyData[] = {
#include <DIM-ESE/safety-circuit.rawh>
};

const tftDL_t tftDL_safety_screen = {
    .len = sizeof(tftDl_safetyData),
    .data = tftDl_safetyData,

    .contentLen = 0,
    .content = NULL
};

/** @brief Bitposition of Y-coordinate byte in vertices */
#define TFT_DL_VERTEX_Y_BIT 15

/** @brief How to draw a single bar dynamically. */
typedef struct {
    uint32_t *addr;  /**< @brief Top-left vertex addr */
    uint32_t topY;   /**< @brief Top edge Y coord. */
    uint32_t botY;   /**< @brief Bot edge Y coord. */
    uint32_t maxVal; /**< @brief Logical value
                      * corresponding to bottom edge */
    uint32_t minVal; /**< @brief Logical value
                      * corresponding to top edge */
} tftDL_vert_bar_t;

typedef struct {
    uint32_t *addr;  /**< @brief Top-left vertex addr */
    uint32_t leftX;  /**< @brief Top edge Y coord. */
    uint32_t rightX; /**< @brief Bot edge Y coord. */
    uint32_t maxVal; /**< @brief Logical value
                      * corresponding to bottom edge */
    uint32_t minVal; /**< @brief Logical value
                      * corresponding to top edge */
} tftDL_horiz_bar_t;

static const tftDL_vert_bar_t hvSoc_bar = {
    .addr = tftDL_RTDData + ESE_HV_BOX_VAL,
    .topY = 1920,
    .botY = 6120,
    .maxVal = 99,
    .minVal = 0
};

static const tftDL_vert_bar_t glvSoc_bar = {
    .addr = tftDL_RTDData + ESE_GLV_BOX_VAL,
    .topY = 1920,
    .botY = 6120,
    .maxVal = 99,
    .minVal = 0
};

static const tftDL_horiz_bar_t hvHorizSoc_bar = {
    .addr = tftDL_racingData + ESE_RS_HV_BOX_VAL,
    .leftX = 3728,
    .rightX = 10032,
    .maxVal = 99,
    .minVal = 0
};

/**
 * @brief Reflect logical value into bar plot for drawing.
 *
 * @param bar The bar to update.
 * @param val The logical value to draw.
 */
static void tftDL_barSetY(const tftDL_vert_bar_t *bar, uint32_t val) {
    uint32_t y;
    if (val < bar->minVal) {
        y = bar->botY;
    } else if (val > bar->maxVal) {
        y = bar->topY;
    } else {
        uint32_t len = ((val - bar->minVal) * (uint32_t)(bar->botY - bar->topY)) / (bar->maxVal - bar->minVal);
        y = bar->botY - len;
    }

    uint32_t vertex = *bar->addr;
    // Create mask with 0s for y, and 1s for x
    uint32_t mask = ((~((uint32_t)0)) << TFT_DL_VERTEX_Y_BIT);
    // vertex and mask to remove current y coord
    vertex &= mask;
    // replace zeros with new y coord (already in correct position)
    vertex |= y;
    *bar->addr = vertex;
}

/**
 * @brief Reflect logical value into bar plot for drawing.
 *
 * @param bar The bar to update.
 * @param val The logical value to draw.
 */
static void tftDL_barSetX(const tftDL_horiz_bar_t *bar, uint32_t val) {
    uint32_t x;
    if (val < bar->minVal) {
        x = bar->leftX;
    } else if (val > bar->maxVal) {
        x = bar->rightX;
    } else {
        uint32_t len = ((val - bar->minVal) * ((uint32_t)(bar->rightX - bar->leftX))) / (bar->maxVal - bar->minVal);
        x = bar->leftX + len;
    }

    uint32_t vertex = *bar->addr;
    // Create mask with 1s for y, and 0s for x
    uint32_t mask = (~((~((uint32_t)0)) << TFT_DL_VERTEX_Y_BIT)) ^ ((1 << 31) >> 1);
    // vertex and mask to remove current x coord
    vertex &= mask;
    // replace zeros with new x coord (shifted to correct position)
    vertex |= (x << TFT_DL_VERTEX_Y_BIT);
    *bar->addr = vertex;
}

/** @brief Color green to be displayed */
#define green 0x0400FF00

/** @brief Color red to be displayed*/
#define red 0x04FF0000

/** @brief Color blue to be displayed*/
#define blue 0x040000FF

/** @brief Color dark red to be displayed*/
#define dark_red 0x04C80000

/** @brief Color black to be displayed*/
#define black 0x04000000

/** @brief Color black to be displayed*/
#define white 0x04FFFFFF

/** @brief Color yellow to be displayed*/
#define yellow 0x04FFFF00

/** @brief Color grey to be displayed*/
#define grey 0x04787878

/*
 * @brief Writes an int via some format string to a location in RTD.
 *
 * @param location the location to write to in RTD.
 * @param length the maximum number of chars to write.
 * @param formatString the format string to write the int using.
 * @param number the number to write.
 */
static void tftDL_RTDwriteInt(uint32_t *file_addr, uint32_t location, uint32_t length, char *formatString, uint32_t number) {
    char *print_location = (void *)(file_addr + location);
    snprintf(print_location, length, formatString, number);
}

/*
 * @brief Writes the current VSM state and gear to the RTD screen.
 */
static void tftDL_showStates(uint32_t *file_addr, uint32_t state_addr, uint32_t state_col_addr,
                             uint32_t gear_addr, uint32_t drs_addr, bool centered) {
    /** @brief Characters for each state. */

    size_t stateCharsLen = 6;
    static const char *stateChars[] = {
        [CMR_CAN_UNKNOWN] = "????",
        [CMR_CAN_GLV_ON] = " GLV",
        [CMR_CAN_HV_EN] = "HVEN",
        [CMR_CAN_RTD] = " RTD",
        [CMR_CAN_ERROR] = " ERR",
        [CMR_CAN_CLEAR_ERROR] = " CLR",
    };

    /** @brief Characters for each gear. */
    static const char *gearChars[CMR_CAN_GEAR_LEN] = {
        [CMR_CAN_GEAR_UNKNOWN] = "  ?????? ",
        [CMR_CAN_GEAR_REVERSE] = " REVERSE ",
        [CMR_CAN_GEAR_SLOW] = "  SLOW   ",
        [CMR_CAN_GEAR_FAST] = "  FAST   ",
        [CMR_CAN_GEAR_ENDURANCE] = "ENDURANCE",
        [CMR_CAN_GEAR_AUTOX] = "AUTOCROSS",
        [CMR_CAN_GEAR_SKIDPAD] = " SKIDPAD ",
        [CMR_CAN_GEAR_ACCEL] = "  ACCEL  ",
        [CMR_CAN_GEAR_TEST] = "   TEST  "
    };

    size_t drsCharsLen = 6;
    static const char *drsChars[] = {
        [CMR_CAN_DRSM_UNKNOWN] = "??????",
        [CMR_CAN_DRSM_QUIET] = "QUIET\0",
        [CMR_CAN_DRSM_CLOSED] = "CLOSED",
        [CMR_CAN_DRSM_OPEN] = " OPEN ",
        [CMR_CAN_DRSM_TOGGLE] = "TOGGLE",
        [CMR_CAN_DRSM_HOLD] = " HOLD ",
        [CMR_CAN_DRSM_AUTO] = " AUTO ",
    };

    cmr_canState_t stateVSM = stateGetVSM();
    cmr_canState_t stateVSMReq = stateGetVSMReq();
    cmr_canGear_t gear = stateGetGear();
    cmr_canDrsMode_t drsMode = stateGetDrs();
    uint32_t *state_color = (void *)(file_addr + state_col_addr);

    char stateChar[12];
    // TODO make this name better I dont want it to be stateCharLen since it very similar to other var
    size_t bufLen = sizeof(stateChars);
    if (stateVSM == stateVSMReq) {
        if (stateVSM < stateCharsLen) {
            if (centered) {
                strcpy(stateChar, "    ");
                strlcat(stateChar, stateChars[stateVSM], bufLen);
                strlcat(stateChar, "    ", stateCharsLen);
            } else {
                strcpy(stateChar, "        ");
                strlcat(stateChar, stateChars[stateVSM], bufLen);
            }
        }
        *state_color = white;
    } else {
        if (stateVSM < stateCharsLen && stateVSMReq < stateCharsLen) {
            strcpy(stateChar, stateChars[stateVSM]);
            strlcat(stateChar, " -> ", bufLen);
            strlcat(stateChar, stateChars[stateVSMReq], bufLen);
        } else if (stateVSM < stateCharsLen) {
            strcpy(stateChar, stateChars[stateVSM]);
            strlcat(stateChar, " -> ", stateCharsLen);
            strlcat(stateChar, stateChars[CMR_CAN_UNKNOWN], bufLen);
        } else {
            strcpy(stateChar, stateChars[CMR_CAN_UNKNOWN]);
            strlcat(stateChar, " -> ", bufLen);
            strlcat(stateChar, stateChars[stateVSMReq], bufLen);
        }
        *state_color = grey;
    }

    const char *gearChar = (gear < CMR_CAN_GEAR_LEN)
                               ? gearChars[gear]
                               : gearChars[CMR_CAN_GEAR_UNKNOWN];

    const char *drsChar = (drsMode < drsCharsLen)
                              ? drsChars[drsMode]
                              : drsChars[CMR_CAN_DRSM_UNKNOWN];

    struct {
        char buf[STATEDISPLAYLEN];
    } *vsmState_str = (void *)(file_addr + state_addr);

    memcpy((void *)vsmState_str->buf, (void *)stateChar, STATEDISPLAYLEN);

    struct {
        char buf[GEARDISPLAYLEN];
    } *gearState_str = (void *)(file_addr + gear_addr);

    memcpy((void *)gearState_str->buf, (void *)gearChar, GEARDISPLAYLEN);

    struct {
        char buf[DRSDISPLAYLEN];
    } *drsState_str = (void *)(file_addr + drs_addr);

    memcpy((void *)drsState_str->buf, (void *)drsChar, DRSDISPLAYLEN);
}

/**
 * @brief sets the display message from the RAM
 * Sets at top and 2 notes on right side
 */
void tftDL_showRAMMsg(uint32_t *file_addr, uint32_t prev_lap_loc, uint32_t targ_lap_loc, uint32_t msg_loc) {
    // struct {
    //     char buf[TIMEDISPLAYLEN];
    // } *prev_time_str = (void *)(file_addr + prev_lap_loc);
    // // Removed get_test_message_id()
    // // sprintf(prev_time_str->buf, "%07x", get_test_message_id());

    // sprintf(prev_time_str->buf, (void *)&(RAMBUF[PREV_TIME_INDEX]), TIMEDISPLAYLEN);
    // //memcpy((void *)prev_time_str->buf, (void *)&(RAMBUF[PREV_TIME_INDEX]), TIMEDISPLAYLEN);
    // struct {
    //     char buf[TIMEDISPLAYLEN];
    // } *target_time_str = (void *)(file_addr + targ_lap_loc);

    // memcpy((void *)target_time_str->buf, (void *)&(RAMBUF[TARGET_TIME_INDEX]), TIMEDISPLAYLEN);
    struct {
        char buf[MESSAGEDISPLAYLEN];
    } *ramMsg_str = (void *)(file_addr + msg_loc);
    memcpy((void *)ramMsg_str->buf, (void *)&(RAMBUF[MESSAGE_INDEX]), MESSAGEDISPLAYLEN);
}

/**
 * @brief sets the color in tftDL_RTDData corresponding to index
 *
 * @param background_index Index into the tftDL_RTDData to add the background color
 * @param text_index Index into the tftDL_RTDData to add the text color
 * @param temp_yellow if the temperature is slightly too high
 * @param temp_red If the temperature is too high
 *
 */
void setTempColor(uint32_t background_index, uint32_t text_index, bool temp_yellow, bool temp_red) {
    uint32_t *background_p = (void *)(tftDL_RTDData + background_index);
    *background_p = temp_red ? dark_red : (temp_yellow ? yellow : black);
    uint32_t *text_p = (void *)(tftDL_RTDData + text_index);
    *text_p = (temp_yellow && !temp_red) ? black : white;
}

/**
 * @brief Updates the ready-to-drive screen.
 *
 * @param memoratorPresent Memorator present (based on heartbeat)
 * @param movellaStatus Movella Status
 * @param speed_mph Speed (from CDC)
 * @param hvVoltage_mV Pack Voltage (from HVC)
 * @param power_kW Electrical power dissipation
 * (inferred from CDC)
 * @param dcdcTemp_C DCDC Thermistor temp.
 * Unused.
 * @param motorTemp_C Motor termperature.
 * Referred from RMS via CDC. Deg. C.
 * @param acTemp_C AC temp (from HVC).
 * Currently Max cell temp. Deg. C.
 * @param mcTemp_C MC internal temp.
 * Referred from RMS via CDC. Deg. C.
 * @param glvVoltage Voltage from GLV
 */
void tftDL_RTDUpdate(
    memorator_status_t memoratorStatus,
    cmr_canMovellaStatus_t movellaStatus,
    int32_t hvVoltage_mV,
    int32_t power_kW,
    uint32_t speed_kmh,
    bool motorTemp_yellow,
    bool motorTemp_red,
    bool acTemp_yellow,
    bool acTemp_red,
    bool mcTemp_yellow,
    bool mcTemp_red,
    int32_t motorTemp_C,
    int32_t acTemp_C,
    int32_t mcTemp_C,
    int32_t glvVoltage_V,
    uint8_t glvSoC,
    uint8_t hvSoC,
    bool yrcOn,
    bool tcOn,
    bool ssOn,
    float odometer_km,
    bool drsOpen,
    cornerId_t hottest_motor) {

     // temps probs arent 0 - just display nothing
    tftDL_RTDwriteInt(tftDL_RTDData, ESE_MOTOR_TEMP_STR, 4, "%3ld", motorTemp_C);

    tftDL_RTDwriteInt(tftDL_RTDData, ESE_MC_TEMP_STR, 4, "%3ld", mcTemp_C);

    tftDL_RTDwriteInt(tftDL_RTDData, ESE_HV_VOLTAGE_VAL, 4, "%3ld", hvVoltage_mV / 1000);
    tftDL_RTDwriteInt(tftDL_RTDData, ESE_AC_TEMP_STR, 4, "%3ld", acTemp_C);
    tftDL_RTDwriteInt(tftDL_RTDData, ESE_GLV_VOLTAGE_VAL, 3, "%2d", glvVoltage_V);
    tftDL_RTDwriteInt(tftDL_RTDData, ESE_POWER_VAL, 3, "%2ld", power_kW);
    tftDL_RTDwriteInt(tftDL_RTDData, ESE_SPEED_VAL, 4, "%3ld", (int32_t)speed_kmh);
    tftDL_RTDwriteInt(tftDL_RTDData, ESE_HV_SOC_VAL, 3, "%2ld", (int32_t)hvSoC);
    tftDL_RTDwriteInt(tftDL_RTDData, ESE_GLV_SOC_VAL, 3, "%2ld", (int32_t)glvSoC);

// Doing this jank buffer because snprintf doesnt work for floats on embedded
// TODO check if we can use "Use float with printf from newlib-nano) ???
#define ODOMETER_STR_SIZE 8
    char odometer_str[ODOMETER_STR_SIZE] = {
        ((char)((((int32_t)odometer_km) % 10000) / 1000)) + '0',
        ((char)((((int32_t)odometer_km) % 1000) / 100)) + '0',
        ((char)((((int32_t)odometer_km) % 100) / 10)) + '0',
        ((char)((((int32_t)odometer_km) % 10) / 1)) + '0',
        '.',
        ((char)((int32_t)(odometer_km * 10.f) % 10)) + '0',
        ((char)((int32_t)(odometer_km * 100.f) % 10)) + '0',
        '\0'
    };
    memcpy((void *)(tftDL_RTDData + ESE_ODO_VAL), (void *)odometer_str, ODOMETER_STR_SIZE);

    tftDL_barSetY(&hvSoc_bar, hvSoC);
    tftDL_barSetY(&glvSoc_bar, glvSoC);
    /* Memorator color */
    uint32_t *memorator_color = (void *)(tftDL_RTDData + ESE_MEMO_TEXT_COLOR);
    uint32_t memorator_color_cmd;
    switch (memoratorStatus) {
        case MEMORATOR_CONNECTED_STATE_OK:
            memorator_color_cmd = green;
            break;
        case MEMORATOR_CONNECTED_BAD_STATE:
            memorator_color_cmd = yellow;
            break;
        case MEMORATOR_NOT_CONNECTED:
        default:
            memorator_color_cmd = red;
    }
    *memorator_color = memorator_color_cmd;

    /* Traction Control color */
    uint32_t *tc_color = (void *)(tftDL_RTDData + ESE_TC_COLOR);
    uint32_t tc_color_cmd = tcOn ? green : red;
    *tc_color = tc_color_cmd;

    /* Yaw Rate Control color */
    uint32_t *yrc_color = (void *)(tftDL_RTDData + ESE_YRC_COLOR);
    uint32_t yrc_color_cmd = yrcOn ? green : red;
    *yrc_color = yrc_color_cmd;

    /* Safety Circuit color */
    uint32_t *ss_color = (void *)(tftDL_RTDData + ESE_SAFETY_CIRCUIT_COLOR);
    uint32_t ss_color_cmd = ssOn ? green : red;
    *ss_color = ss_color_cmd;

    /* DRS color */
    uint32_t *drs_color = (void *)(tftDL_RTDData + ESE_DRS_COLOR);
    uint32_t drs_color_cmd = drsOpen ? green : black;
    *drs_color = drs_color_cmd;

    /* Radio color */
    uint32_t *radio_color = (void *)(tftDL_RTDData + ESE_RADIO_COLOR);
    uint32_t radio_color_cmd = getAcknowledgeButton() ? green : black;
    *radio_color = radio_color_cmd;

    /* GPS color */
    uint32_t *gps_color = (void *)(tftDL_RTDData + ESE_GPS_TEXT_COLOR);
    uint32_t gps_color_cmd;
    switch (movellaStatus->gnss_fix) {
        case 1:
            // GPS working and position found
            gps_color_cmd = green;
            break;
        case 0:
            // GPS connected, not working
            gps_color_cmd = red;
            break;
        default:
            // GPS not connected
            gps_color_cmd = red;
    }
    *gps_color = gps_color_cmd;


    cmr_canHVCPackMinMaxCellVolages_t* packVoltagesStruct = getPackVoltages();

    /* Pack Voltages */
    tftDL_RTDwriteInt(tftDL_RTDData, ESE_RAM_MIN_CELL, 5, "%4ld", packVoltagesStruct->minCellVoltage_mV);
    tftDL_RTDwriteInt(tftDL_RTDData, ESE_RAM_MAX_CELL, 5, "%4ld", packVoltagesStruct->maxCellVoltage_mV);

    tftDL_showStates(tftDL_RTDData, ESE_STATE_STR, ESE_VSM_STATE_COLOR, ESE_GEAR_STR, ESE_DRS_MODE_STR, false);
    tftDL_showRAMMsg(tftDL_RTDData, ESE_RAM_MIN_CELL, ESE_RAM_MAX_CELL, ESE_RAM_MSG_STR);


    uint32_t *fl_color = (void *)(tftDL_RTDData + ESE_RTD_FL_COLOR);
    uint32_t *fr_color = (void *)(tftDL_RTDData + ESE_RTD_FR_COLOR);
    uint32_t *rl_color = (void *)(tftDL_RTDData + ESE_RTD_RL_COLOR);
    uint32_t *rr_color = (void *)(tftDL_RTDData + ESE_RTD_RR_COLOR);
    uint32_t fl_color_cmd = black;
    uint32_t fr_color_cmd = black;
    uint32_t rl_color_cmd = black;
    uint32_t rr_color_cmd = black;
    switch (hottest_motor) {
        case FL:
            fl_color_cmd = red;
            break;
        case FR:
            fr_color_cmd = red;
            break;
        case RL:
            rl_color_cmd = red;
            break;
        case RR:
            rr_color_cmd = red;
            break;
    }
    *fl_color = fl_color_cmd;
    *fr_color = fr_color_cmd;
    *rl_color = rl_color_cmd;
    *rr_color = rr_color_cmd;
}

void tftDL_racingScreenUpdate(
    int32_t motorTemp_C,
    int32_t acTemp_C,
    int32_t mcTemp_C,
    uint8_t hvSoC,
    bool drsOpen) {
    tftDL_RTDwriteInt(tftDL_racingData, ESE_RS_MOTOR_TEMP_STR, 4, "%3ld", motorTemp_C);
    tftDL_RTDwriteInt(tftDL_racingData, ESE_RS_MC_TEMP_STR, 4, "%3ld", mcTemp_C);

    tftDL_RTDwriteInt(tftDL_racingData, ESE_RS_AC_TEMP_STR, 4, "%3ld", acTemp_C);
    tftDL_RTDwriteInt(tftDL_racingData, ESE_RS_HV_SOC_VAL, 3, "%2ld", (uint32_t)hvSoC);

    // set HV SoC bar
    tftDL_barSetX(&hvHorizSoc_bar, (uint32_t)hvSoC);

    /* DRS color */
    uint32_t *drs_color = (void *)(tftDL_racingData + ESE_RS_DRS_COLOR);
    uint32_t drs_color_cmd = drsOpen ? green : black;
    *drs_color = drs_color_cmd;

    /* Radio color */
    uint32_t *radio_color = (void *)(tftDL_racingData + ESE_RS_RADIO_COL);
    uint32_t radio_color_cmd = getAcknowledgeButton() ? green : black;
    *radio_color = radio_color_cmd;

    tftDL_showStates(tftDL_racingData, ESE_RS_VSM_STATE_STR, ESE_RS_VSM_STATE_COLOR, ESE_RS_GEAR_STR, ESE_RS_DRS_MODE_STR, true);
    tftDL_showRAMMsg(tftDL_racingData, ESE_RS_RAM_LAST_LAP, ESE_RS_RAM_TARG_LAP, ESE_RS_RAM_MSG_STR);
}

/**
 * @brief Set the text color of an error depending on whether it is active.
 *
 * @param addr location the location of the text color to change.
 * @param val condition boolean value of whether the error is active.
 */
static void tftDL_showErrorState(uint32_t location, bool condition) {
    uint32_t color_err = 0x04ff0a0a;
    uint32_t color_none = 0x04303030;
    uint32_t *color_location = (void *)(tftDL_errorData + location);
    *color_location = condition ? color_err : color_none;
}

static void tftDL_ERRwriteInt(uint32_t location, uint32_t length, char *formatString, uint32_t number) {
    char *print_location = (void *)(tftDL_errorData + location);
    snprintf(print_location, length, formatString, number);
}

static void tftDL_showBMBStatus(volatile cmr_canHVCBMBErrors_t *BMBerr) {
    tftDL_ERRwriteInt(ESE_BMB_0_NUM_STR, 3, "%02X", BMBerr->BMB1_2_Errs);
    tftDL_ERRwriteInt(ESE_BMB_1_NUM_STR, 3, "%02X", BMBerr->BMB3_4_Errs);
    tftDL_ERRwriteInt(ESE_BMB_2_NUM_STR, 3, "%02X", BMBerr->BMB5_6_Errs);
    tftDL_ERRwriteInt(ESE_BMB_3_NUM_STR, 3, "%02X", BMBerr->BMB7_8_Errs);
    tftDL_ERRwriteInt(ESE_BMB_4_NUM_STR, 3, "%02X", BMBerr->BMB9_10_Errs);
    tftDL_ERRwriteInt(ESE_BMB_5_NUM_STR, 3, "%02X", BMBerr->BMB11_12_Errs);
    tftDL_ERRwriteInt(ESE_BMB_6_NUM_STR, 3, "%02X", BMBerr->BMB13_14_Errs);
    tftDL_ERRwriteInt(ESE_BMB_7_NUM_STR, 3, "%02X", BMBerr->BMB15_16_Errs);

    // If any are non-zero, display red
    bool bmbRed = (bool)(BMBerr->BMB1_2_Errs | BMBerr->BMB3_4_Errs | BMBerr->BMB5_6_Errs |
                         BMBerr->BMB7_8_Errs | BMBerr->BMB9_10_Errs | BMBerr->BMB11_12_Errs |
                         BMBerr->BMB13_14_Errs | BMBerr->BMB15_16_Errs);
    tftDL_showErrorState(ESE_HVC_BMB_STATUS_COLOR, bmbRed);
}

static void tftDL_showAMKError(uint32_t strlocation, uint32_t colorLocation, uint16_t errorCode) {
    char *print_location = (void *)(tftDL_errorData + strlocation);
    const size_t print_len = 13;
    // Spaces are to align text, so each string has 12 characters followed by a \0
    switch (errorCode) {
        case 2347:
            snprintf(print_location, print_len, "MOTOR TEMP  ");
            break;
        case 2346:
            snprintf(print_location, print_len, "IGBT TEMP   ");
            break;
        case 2310:
            snprintf(print_location, print_len, "ENCODER PROB");
            break;
        case 3587:
            snprintf(print_location, print_len, "SOFTWARE CAN");
            break;
        case 1049:
            snprintf(print_location, print_len, "HV LOW VOLT ");
            break;
        default:
            // No text, so display error number
            snprintf(print_location, print_len, "%04d        ", errorCode);
            break;
    }
    tftDL_showErrorState(colorLocation, errorCode != 0);
}

/**
 * @brief Updates the error screen.
 *
 * @param err Error statuses to display.
 * @param BMBerr BMB error statuses to display.
 */
void tftDL_errorUpdate(
    tft_errors_t *err,
    volatile cmr_canHVCBMBErrors_t *BMBerr) {
    static struct {
        char buf[4];
    } *const glvVoltage_V_str = (void *)(tftDL_errorData + ESE_ERR_GLV_STR);

    snprintf(
        glvVoltage_V_str->buf, sizeof(glvVoltage_V_str->buf),
        "%2lu", err->glvVoltage_V);

    /* Timeouts */
    tftDL_showErrorState(ESE_PTC_COLOR, err->ptcTimeout);
    tftDL_showErrorState(ESE_HVC_COLOR, err->hvcTimeout);
    tftDL_showErrorState(ESE_VSM_COLOR, err->vsmTimeout);
    tftDL_showErrorState(ESE_CDC_COLOR, err->cdcTimeout);

    /* HVC */
    tftDL_showErrorState(ESE_HVC_OVERVOLT_COLOR, err->overVolt);
    tftDL_showErrorState(ESE_HVC_UNDERVOLT_COLOR, err->underVolt);
    tftDL_showErrorState(ESE_HVC_OVERTEMP_COLOR, err->hvcoverTemp);
    tftDL_showErrorState(ESE_HVC_BMB_TIMEOUT_COLOR, err->hvcBMBTimeout);
    tftDL_showErrorState(ESE_HVC_BMB_FAULT_COLOR, err->hvcBMBFault);

    static struct {
        char buf[5];
    } *const hvc_error_num_str = (void *)(tftDL_errorData + ESE_HVC_ERROR_NUM_STR);
    snprintf(
        hvc_error_num_str->buf, sizeof(hvc_error_num_str->buf),
        "%04x", err->hvcErrorNum);
    tftDL_showErrorState(ESE_HVC_ERROR_COLOR, err->hvcErrorNum != 0);

    // BMB Status
    tftDL_showBMBStatus(BMBerr);

    /* Latching */
    tftDL_showErrorState(ESE_IMD_COLOR, err->imdError);
    tftDL_showErrorState(ESE_AMS_COLOR, err->amsError);
    tftDL_showErrorState(ESE_BSPD_COLOR, err->bspdError);
    tftDL_showErrorState(ESE_GLV_COLOR, err->glvLowVolt);

    /* Display AMK errors */
    tftDL_showAMKError(ESE_AMK_FL_STR, ESE_AMK_FL_COLOR, err->amkFLErrorCode);
    tftDL_showAMKError(ESE_AMK_FR_STR, ESE_AMK_FR_COLOR, err->amkFRErrorCode);
    tftDL_showAMKError(ESE_AMK_BL_STR, ESE_AMK_BL_COLOR, err->amkBLErrorCode);
    tftDL_showAMKError(ESE_AMK_BR_STR, ESE_AMK_BR_COLOR, err->amkBRErrorCode);
}

/**
 * @brief Loads the display list's content into graphics memory.
 *
 * @param tft The display.
 * @param tftDL The display list.
 */
void tftDLContentLoad(tft_t *tft, const tftDL_t *tftDL) {
    for (size_t i = 0; i < tftDL->contentLen; i++) {
        const tftContent_t *tftContent = tftDL->content[i];
        tftContentLoad(tft, tftContent);
    }
}

/**
 * @brief Writes a display list.
 *
 * @param tft The display.
 * @param tftDL The display list.
 */
void tftDLWrite(tft_t *tft, const tftDL_t *tftDL) {
    tftCoCmd(tft, tftDL->len, tftDL->data);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void setConfigContextString(int8_t scroll_index) {
    char *context_string = config_menu_main_array[scroll_index].ESE_context_text_variable;
    uint32_t context_string_address_offset = ESE_CONTEXT_VAL;
    uint32_t *context_string_pointer = (void *)(tftDL_configData + context_string_address_offset);
    sprintf((char *)context_string_pointer, context_string);
}

// implements wraparound. Can't use modulo since it's min is not always 0. Could do (val % max-min) + min but that's less readable
uint8_t configValueIncrementer(uint8_t value, uint8_t value_min, uint8_t value_max, bool up_requested, bool down_requested) {
    uint8_t new_value = value;
    if (up_requested) {
        if (value + 1 > value_max) {
            new_value = value_min;
        } else {
            new_value++;
        }
    }
    if (down_requested) {
        if (value - 1 < value_min) {
            new_value = value_max;
        } else {
            new_value--;
        }
    }
    return new_value;
}

void setConfigIncrementValue(int8_t scroll_index, bool up_requested, bool down_requested) {
    // calculate the varoius addresses to modify
    uint32_t value_address_offset = config_menu_main_array[scroll_index].ESE_value_variable;
    uint32_t *value_address_pointer = (void *)(tftDL_configData + value_address_offset);

    // lut for custom enum
    char **custom_enum_lut = config_menu_main_array[scroll_index].ESE_value_string_lut;

    // type of value being modified
    cmr_config_type_t value_type = config_menu_main_array[scroll_index].value.type;
    // current value of the item
    uint8_t value = config_menu_main_array[scroll_index].value.value;
    uint8_t value_min = config_menu_main_array[scroll_index].min;
    uint8_t value_max = config_menu_main_array[scroll_index].max;

    // unsigned_integer,
    // custom_enum

    char buffer[5];

    switch (value_type) {
        case unsigned_integer:
            // treat it like an integer
        case integer:
            value = configValueIncrementer(value, value_min, value_max, up_requested, down_requested);
            snprintf((char *)value_address_pointer, 4, "%3d", value);
            break;
        case boolean:
            if (up_requested || down_requested) {
                value = !value;
            }
            sprintf((char *)value_address_pointer, config_boolean_string_lut[value]);
            break;
        case float_1_decimal:
            value = configValueIncrementer(value, value_min, value_max, up_requested, down_requested);
            snprintf(buffer, 5, "%4d", value);
            buffer[0] = buffer[1];
            buffer[1] = buffer[2];
            buffer[2] = '.';
            sprintf((char *)value_address_pointer, buffer);
            break;
        case float_2_decimal:
            value = configValueIncrementer(value, value_min, value_max, up_requested, down_requested);
            snprintf(buffer, 5, "%4d", value);
            buffer[0] = buffer[1];
            buffer[1] = '.';
            sprintf((char *)value_address_pointer, buffer);
            break;
        case custom_enum:
            // -1 since index off of 0
            value = configValueIncrementer(value, value_min, value_max - 1, up_requested, down_requested);
            size_t len = config_menu_main_array[scroll_index].ESE_string_len;

            memcpy((void *)value_address_pointer, (void *)custom_enum_lut[value], len);
    };

    // Flush the modified value
    config_menu_main_array[scroll_index].value.value = value;
}

void setConfigSelectionColor(int8_t scroll_index, int8_t restore_index) {
    // calculate the varoius addresses to modify
    uint32_t background_address_offset = config_menu_main_array[scroll_index].ESE_background_color_variable;
    uint32_t *background_item_pointer = (void *)(tftDL_configData + background_address_offset);

    uint32_t restore_background_address_offset = config_menu_main_array[restore_index].ESE_background_color_variable;
    uint32_t *restore_background_item_pointer = (void *)(tftDL_configData + restore_background_address_offset);

    // modify the actual addresses
    *background_item_pointer = (uint32_t)SELECTED_MENU_COLOR;
    *restore_background_item_pointer = (uint32_t)NOT_SELECTED_MENU_COLOR;

    return;
}

void drawLatestConfigValues() {
    // loop through all the elements of the array and appropriately render the variables
    for (int i = 0; i < MAX_MENU_ITEMS; i++) {
        setConfigIncrementValue(i, false, false);
    }
}

// TODO: Document
void tftDL_configUpdate() {
    static int8_t current_scroll_index = DRIVER_PROFILE_INDEX;
    static bool paddle_prev_active = false;
    static uint32_t paddle_time_since_change = 0;

    if (dim_first_time_config_screen) {
        int8_t prev_scroll_index = (current_scroll_index == DRIVER_PROFILE_INDEX) ? DRIVER_PROFILE_INDEX + 1 : current_scroll_index;
        current_scroll_index = DRIVER_PROFILE_INDEX;
        drawLatestConfigValues();
        setConfigSelectionColor(current_scroll_index, prev_scroll_index);
        setConfigContextString(current_scroll_index);
        dim_first_time_config_screen = false;
    }
    if (redraw_new_driver_profiles) {
        drawLatestConfigValues();
        redraw_new_driver_profiles = false;
    }

    // Handles D Pad logic accounting for Driver Profile annoyance
    // If there is a move request or we're loading the page for the first time
    if (config_move_request != 0) {
        // record previous index so we can clear colour
        int8_t prev_scroll_index = current_scroll_index;

        if (current_scroll_index == DRIVER_PROFILE_INDEX) {
            // if we're on the driver profile we either go to first or last grid square
            if (config_move_request > 0) {
                current_scroll_index++;
            } else {
                current_scroll_index = MAX_MENU_ITEMS - 1;
            }
        } else if (config_move_request == CONFIG_SCREEN_NUM_COLS &&
                   current_scroll_index >= MAX_MENU_ITEMS - CONFIG_SCREEN_NUM_COLS) {
            // if we're in the bottom row and go down, we go to driver square
            current_scroll_index = DRIVER_PROFILE_INDEX;
        } else if (config_move_request == -CONFIG_SCREEN_NUM_COLS &&
                   current_scroll_index <= CONFIG_SCREEN_NUM_COLS) {
            // if we're in the top row and go up, we go to the driver square
            current_scroll_index = DRIVER_PROFILE_INDEX;
        } else if (((current_scroll_index - 1) % CONFIG_SCREEN_NUM_COLS) == 0 &&
                   config_move_request == -1) {
            // if we're at the left and go left, we wrap around to the right
            current_scroll_index += CONFIG_SCREEN_NUM_COLS - 1;
        } else if (((current_scroll_index - 1) % CONFIG_SCREEN_NUM_COLS) == CONFIG_SCREEN_NUM_COLS - 1 &&
                   config_move_request == 1) {
            // if we're at the right and go right, we wrap around to the left
            current_scroll_index -= CONFIG_SCREEN_NUM_COLS - 1;
        } else {
            // standard logic accounting for driver profile sqaure
            current_scroll_index += config_move_request;
            current_scroll_index = ((current_scroll_index - 1) % (MAX_MENU_ITEMS - 1)) + 1;
        }

        // clear the selection value just in case no one accidently presses both buttons at the same time
        config_move_request = 0;

        // call the background color updater
        setConfigSelectionColor(current_scroll_index, prev_scroll_index);
        setConfigContextString(current_scroll_index);

        config_increment_up_requested = false;
        config_increment_down_requested = false;
    }

    // if there are no move requests, then check/implement selection values
    else if (config_increment_up_requested || config_increment_down_requested) {
        if (config_increment_down_requested && config_increment_up_requested)
            return;  // both are an error

        if (current_scroll_index == DRIVER_PROFILE_INDEX) {
            flush_config_screen_to_cdc = true;
            waiting_for_cdc_to_confirm_config = true;
            // Wait for CDC to confirm old driver params first
            while (waiting_for_cdc_to_confirm_config) {
            }

            // Change driver
            setConfigIncrementValue(current_scroll_index, config_increment_up_requested, config_increment_down_requested);
            waiting_for_cdc_new_driver_config = true;
            while (waiting_for_cdc_new_driver_config) {
                // wait for the new driver to be selected
            }
        } else {
            setConfigIncrementValue(current_scroll_index, config_increment_up_requested, config_increment_down_requested);
        }

        config_increment_up_requested = false;
        config_increment_down_requested = false;
    } else if (config_paddle_left_request > 0 || config_paddle_right_request > 0) {
        if (config_paddle_left_request > 0 && config_paddle_right_request > 0) {
            // both paddles depressed => do nothing
            paddle_prev_active = false;
            return;
        }
        // don't use paddles to change driver profile
        if (current_scroll_index == DRIVER_PROFILE_INDEX) return;

        if (config_paddle_left_request > 0) {
            // Handle left paddle request
            if (!paddle_prev_active) {
                setConfigIncrementValue(current_scroll_index, false, true);
                paddle_prev_active = true;
                paddle_time_since_change = 0;
            } else {
                paddle_time_since_change += 1;
                float increment_freq = (float)config_paddle_left_request / (float)paddle_time_scale_factor;
                // increment count relative to display update period (ie period of this function)
                float increment_count = (float)TFT_UPDATE_PERIOD_MS / increment_freq;
                if ((float)paddle_time_since_change >= increment_count) {
                    setConfigIncrementValue(current_scroll_index, false, true);
                    paddle_time_since_change = 0;
                }
            }
        } else if (config_paddle_right_request > 0) {
            // Handle right paddle request
            if (!paddle_prev_active) {
                setConfigIncrementValue(current_scroll_index, true, false);
                paddle_prev_active = true;
                paddle_time_since_change = 0;
            } else {
                paddle_time_since_change += 1;
                float increment_freq = (float)config_paddle_right_request / (float)paddle_time_scale_factor;
                // increment count relative to display update period (ie period of this function)
                float increment_count = (float)TFT_UPDATE_PERIOD_MS / increment_freq;
                if ((float)paddle_time_since_change >= increment_count) {
                    setConfigIncrementValue(current_scroll_index, true, false);
                    paddle_time_since_change = 0;
                }
            }
        }

        if (config_paddle_left_request == 0 && config_paddle_right_request == 0) {
            paddle_prev_active = false;
        }
    }

    return;
}