/**
 * @file tftDL.c
 * @brief TFT display list implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdio.h>        // snprintf
#include <string.h>        // memcpy()

#include "tftDL.h"          // Interface to implement
#include "tftContent.h"     // Content interface
#include "adc.h"            // GLV voltage
#include "state.h"          // State interface
#include "can.h"            // Board-specific CAN interface
#include "gpio.h"            // Board-specific CAN interface

// used to calculate increment frequency:
// max paddle val (255) / max increment speed (20Hz) 
static const float paddle_time_scale_factor = 255.0f / 20.0f;

/** @brief Represents a display list. */
struct tftDL {
    size_t len;             /**< @brief Length of the display list, in bytes. */
    const uint32_t *data;   /**< @brief The display list data. */

    size_t contentLen;              /**< @brief Number of content items. */
    const tftContent_t **content;   /**< @brief Associated content. */
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

/** @brief RTD Screen */
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


/** @brief Bitposition of Y-coordinate byte in vertices */
#define TFT_DL_VERTEX_Y_BIT 16

/** @brief How to draw a single bar dynamically. */
typedef struct {
    uint32_t *addr;  /**< @brief Top-left vertex addr */
    uint32_t topY;    /**< @brief Top edge Y coord. */
    uint32_t botY;    /**< @brief Bot edge Y coord. */
    uint32_t maxVal;  /**< @brief Logical value
    * corresponding to bottom edge */
    uint32_t minVal;  /**< @brief Logical value
    * corresponding to top edge */
} tftDL_bar_t;

static const tftDL_bar_t hvSoc_bar = {
    .addr = tftDL_RTDData + ESE_HV_BOX_VAL,
    .topY = 1920,
    .botY = 6120,
    .maxVal = 99,
    .minVal = 0
};

static const tftDL_bar_t glvSoc_bar = {
    .addr = tftDL_RTDData + ESE_GLV_BOX_VAL,
    .topY = 1920,
    .botY = 6120,
    .maxVal = 99,
    .minVal = 0
};

/**
 * @brief Reflect logical value into bar plot for drawing.
 *
 * @param bar The bar to update.
 * @param val The logical value to draw.
 */
static void tftDL_barSetY(const tftDL_bar_t *bar, int32_t val) {
    uint32_t y;
    if (val < bar->minVal) {
        y = bar->botY;
    } else if (val > bar->maxVal) {
        y = bar->topY;
    } else {
        uint32_t len = (
            (val - bar->minVal) * (uint32_t) (bar->botY - bar->topY)
        ) / (bar->maxVal - bar->minVal);
        y = bar->botY - len;
    }

    uint32_t vertex = *bar->addr;
    uint32_t mask = ((~((uint32_t)0)) << TFT_DL_VERTEX_Y_BIT);
    vertex &= mask; //TODO: This is dumb
    vertex |= y;
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
static void tftDL_RTDwriteInt(uint32_t location, uint32_t length,  char* formatString, uint32_t number) {
    char *print_location = (void *) (tftDL_RTDData + location);
    snprintf(print_location, length, formatString, number);
}

/*
 * @brief Writes the current VSM state and gear to the RTD screen.
 */
static void tftDL_showStates() {
    /** @brief Characters for each state. */
    size_t stateCharsLen = 6;
    static const char *stateChars[] = {
        [CMR_CAN_UNKNOWN] =       "????",
        [CMR_CAN_GLV_ON] =        " GLV",
        [CMR_CAN_HV_EN] =         "HVEN",
        [CMR_CAN_RTD] =           " RTD",
        [CMR_CAN_ERROR] =         " ERR",
        [CMR_CAN_CLEAR_ERROR] =   " CLR",
    };

    /** @brief Characters for each gear. */
    static const char* gearChars[CMR_CAN_GEAR_LEN] = {
        [CMR_CAN_GEAR_UNKNOWN] =   "      ??????",
        [CMR_CAN_GEAR_REVERSE] =   "     REVERSE",
        [CMR_CAN_GEAR_SLOW] =      "        SLOW",
        [CMR_CAN_GEAR_FAST] =      "        FAST",
        [CMR_CAN_GEAR_ENDURANCE] = "   ENDURANCE",
        [CMR_CAN_GEAR_AUTOX] =     "   AUTOCROSS",
        [CMR_CAN_GEAR_SKIDPAD] =   "     SKIDPAD",
        [CMR_CAN_GEAR_ACCEL] =     "       ACCEL",
        [CMR_CAN_GEAR_TEST] =      "        TEST"
    };

    size_t drsCharsLen = 6;
    static const char *drsChars[] = {
        [CMR_CAN_DRSM_UNKNOWN] =      "????????????",
        [CMR_CAN_DRSM_CLOSED] =       "       CLOSE",
        [CMR_CAN_DRSM_OPEN] =         "        OPEN",
        [CMR_CAN_DRSM_TOGGLE] =       "      TOGGLE",
        [CMR_CAN_DRSM_HOLD] =         "        HOLD",
        [CMR_CAN_DRSM_AUTO] =         "   AUTOMATIC",
    };

    cmr_canState_t stateVSM = stateGetVSM();
    cmr_canState_t stateVSMReq = stateGetVSMReq();
    cmr_canGear_t gear = stateGetGear();
    cmr_canDrsMode_t drsMode = stateGetDrs();
    uint32_t *state_color = (void *) (tftDL_RTDData + ESE_VSM_STATE_COLOR);

    char stateChar[12];
    if (stateVSM == stateVSMReq) {
        if (stateVSM < stateCharsLen) {
        	strcpy(stateChar, "        ");
        	strcat(stateChar, stateChars[stateVSM]);
        }
        *state_color = white;
    } else {
        if (stateVSM < stateCharsLen && stateVSMReq < stateCharsLen) {
        	strcpy(stateChar, stateChars[stateVSM]);
            strcat(stateChar, " -> ");
            strcat(stateChar, stateChars[stateVSMReq]);
        } else if (stateVSM < stateCharsLen) {
        	strcpy(stateChar, stateChars[stateVSM]);
            strcat(stateChar, " -> ");
            strcat(stateChar, stateChars[CMR_CAN_UNKNOWN]);
        } else {
        	strcpy(stateChar, stateChars[CMR_CAN_UNKNOWN]);
            strcat(stateChar, " -> ");
            strcat(stateChar, stateChars[stateVSMReq]);
        }
        *state_color = grey;
    }

    const char* gearChar = (gear < CMR_CAN_GEAR_LEN)
        ? gearChars[gear]
        : gearChars[CMR_CAN_GEAR_UNKNOWN];

    const char* drsChar = (drsMode < drsCharsLen)
            ? drsChars[drsMode]
            : drsChars[CMR_CAN_DRSM_UNKNOWN];

    static struct {
        char buf[STATEDISPLAYLEN];
    } *const vsmState_str = (void *) (tftDL_RTDData + ESE_VSM_STATE_STR);

    memcpy((void *) vsmState_str->buf, (void *) stateChar, STATEDISPLAYLEN);

    static struct {
        char buf[GEARDISPLAYLEN];
    } *const gearState_str = (void *) (tftDL_RTDData + ESE_GEAR_STR);

    memcpy((void *) gearState_str->buf, (void *) gearChar, GEARDISPLAYLEN);

    static struct {
        char buf[DRSDISPLAYLEN];
    } *const drsState_str = (void *) (tftDL_RTDData + ESE_DRS_MODE_STR);

    memcpy((void *) drsState_str->buf, (void *) drsChar, DRSDISPLAYLEN);
}

/**
 * @brief sets the display message from the RAM
 * Sets at top and 2 notes on right side
 */
void tftDL_showRAMMsg() {
    static struct {
            char buf[RAMDISPLAYLEN];
        } *const ramMsg_str = (void *) (tftDL_RTDData + ESE_RAM_MSG1_STR);
        memcpy((void *) ramMsg_str->buf, (void *) RAMBUF, RAMDISPLAYLEN);
    static struct {
            char buf[NOTEDISPLAYLEN];
        } *const note1_str = (void *) (tftDL_RTDData + ESE_RAM_MSG2_STR);
        memcpy((void *) note1_str->buf, (void *) &(RAMBUF[NOTE1_INDEX]), NOTEDISPLAYLEN);
    static struct {
            char buf[NOTEDISPLAYLEN];
        } *const note2_str = (void *) (tftDL_RTDData + ESE_RAM_MSG3_STR);
        memcpy((void *) note2_str->buf, (void *) &(RAMBUF[NOTE2_INDEX]), NOTEDISPLAYLEN);
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
    uint32_t* background_p = (void *) (tftDL_RTDData + background_index);
    *background_p = temp_red ? dark_red : (temp_yellow ? yellow : black);
    uint32_t* text_p = (void *) (tftDL_RTDData + text_index);
    *text_p = (temp_yellow && !temp_red) ? black : white;
}

/**
 * @brief Updates the ready-to-drive screen.
 *
 * @param memoratorPresent Memorator present (based on heartbeat)
 * @param sbgStatus SBG INS Status
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
    bool memoratorPresent,
    SBG_status_t sbgStatus,
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
    bool drsClosed
) {
    tftDL_RTDwriteInt(ESE_HV_VOLTAGE_VAL, 4, "%3ld", hvVoltage_mV / 1000);
    tftDL_RTDwriteInt(ESE_MOTOR_TEMP_STR, 4, "%3ld", motorTemp_C);
    tftDL_RTDwriteInt(ESE_AC_TEMP_STR, 4, "%3ld", acTemp_C);
    tftDL_RTDwriteInt(ESE_MC_TEMP_STR, 4, "%3ld", mcTemp_C);
    tftDL_RTDwriteInt(ESE_GLV_VOLTAGE_VAL, 3, "%2d", glvVoltage_V);
    tftDL_RTDwriteInt(ESE_POWER_VAL, 3, "%2ld", power_kW);
    tftDL_RTDwriteInt(ESE_SPEED_VAL, 4, "%3ld", (int32_t)speed_kmh);
    tftDL_RTDwriteInt(ESE_HV_SOC_VAL, 3, "%2ld", (int32_t)hvSoC);
    tftDL_RTDwriteInt(ESE_GLV_SOC_VAL, 3, "%2ld", (int32_t)glvSoC);

    // Doing this jank buffer because snprintf doesnt work for floats on embedded
	#define ODOMETER_STR_SIZE 8
    char odometer_str[ODOMETER_STR_SIZE] = {
        ((char) ((((int32_t) odometer_km) % 10000) / 1000)) + '0',
        ((char) ((((int32_t) odometer_km) % 1000) / 100)) + '0',
        ((char) ((((int32_t) odometer_km) % 100) / 10)) + '0',
        ((char) ((((int32_t) odometer_km) % 10) / 1)) + '0',
        '.',
        ((char) ((int32_t)(odometer_km * 10.f) % 10)) + '0',
        ((char) ((int32_t)(odometer_km * 100.f) % 10)) + '0',
        '\0'
    };
    memcpy((void *) (tftDL_RTDData + ESE_ODO_VAL), (void *) odometer_str, ODOMETER_STR_SIZE);

    tftDL_barSetY(&hvSoc_bar, (uint32_t)hvSoC);
    tftDL_barSetY(&glvSoc_bar, (uint32_t)glvSoC);
    /* Memorator color */
    uint32_t *memorator_color = (void *) (tftDL_RTDData + ESE_MEMO_TEXT_COLOR);
    uint32_t memorator_color_cmd = memoratorPresent ? green : red;
    *memorator_color = memorator_color_cmd;

    /* Traction Control color */
    uint32_t *tc_color = (void *) (tftDL_RTDData + ESE_TC_COLOR);
    uint32_t tc_color_cmd = tcOn ? green : red;
    *tc_color = tc_color_cmd;

    /* Yaw Rate Control color */
    uint32_t *yrc_color = (void *) (tftDL_RTDData + ESE_YRC_COLOR);
    uint32_t yrc_color_cmd = yrcOn ? green : red;
    *yrc_color = yrc_color_cmd;

    /* Safety Circuit color */
    uint32_t *ss_color = (void *) (tftDL_RTDData + ESE_SAFETY_CIRCUIT_COLOR);
    uint32_t ss_color_cmd = ssOn ? green : red;
    *ss_color = ss_color_cmd;

    /* DRS color */
    uint32_t *drs_color = (void *) (tftDL_RTDData + ESE_DRS_COLOR);
    uint32_t drs_color_cmd = drsClosed ? white : green;
    *drs_color = drs_color_cmd;

    /* GPS color */
    uint32_t *gps_color = (void *) (tftDL_RTDData + ESE_GPS_TEXT_COLOR);
    uint32_t gps_color_cmd;
    switch (sbgStatus) {
    case SBG_STATUS_WORKING_POS_FOUND:
        // GPS working and position found
        gps_color_cmd = green;
        break;
    case SBG_STATUS_WORKING_NO_POS_FOUND:
        // GPS connected, not working
        gps_color_cmd = yellow;
        break;
    case SBG_STATUS_NOT_CONNECTED:
    default:
        // GPS not connected
        gps_color_cmd = red;
    }
    *gps_color = gps_color_cmd;

    /* Temperature backgrounds */
    setTempColor(ESE_MOTOR_TEMP_BG_COLOR, ESE_MOTOR_TEMP_COLOR, motorTemp_yellow, motorTemp_red);
    setTempColor(ESE_AC_TEMP_BG_COLOR, ESE_AC_TEMP_COLOR, acTemp_yellow, acTemp_red);
    setTempColor(ESE_MC_TEMP_BG_COLOR, ESE_MC_TEMP_COLOR, mcTemp_yellow, mcTemp_red);

    tftDL_showStates();
    tftDL_showRAMMsg();
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
    uint32_t *color_location = (void *) (tftDL_errorData + location);
    *color_location = condition ? color_err : color_none;
}

/**
 * @brief Updates the error screen.
 *
 * @param err Error statuses to display.
 */
void tftDL_errorUpdate(
    tft_errors_t *err
) {

    static struct {
        char buf[4];
    } *const glvVoltage_V_str = (void *) (tftDL_errorData + ESE_ERR_GLV_STR);

    snprintf(
        glvVoltage_V_str->buf, sizeof(glvVoltage_V_str->buf),
        "%2dV", err->glvVoltage_V
    );

    tftDL_showErrorState(ESE_PTCp_COLOR, err->ptcTimeout);
    tftDL_showErrorState(ESE_PTCf_COLOR, err->ptcTimeout);
    tftDL_showErrorState(ESE_APC_COLOR, err->apcTimeout);
    tftDL_showErrorState(ESE_HVC_COLOR, err->hvcTimeout);
    tftDL_showErrorState(ESE_VSM_COLOR, err->vsmTimeout);
    tftDL_showErrorState(ESE_FSM_COLOR, err->fsmTimeout);
    tftDL_showErrorState(ESE_CDC_COLOR, err->cdcTimeout);
    tftDL_showErrorState(ESE_OVERVOLT_COLOR, err->overVolt);
    tftDL_showErrorState(ESE_UNDERVOLT_COLOR, err->underVolt);
    tftDL_showErrorState(ESE_HVC_OVERTEMP_COLOR, err->hvcoverTemp);
    tftDL_showErrorState(ESE_HVC_ERROR_COLOR, err->hvc_Error);
    tftDL_showErrorState(ESE_OVERSPEED_COLOR, err->overSpeed);
    tftDL_showErrorState(ESE_MC_OVERTEMP_COLOR, err->mcoverTemp);
    tftDL_showErrorState(ESE_OVERCURRENT_COLOR, err->overCurrent);
    tftDL_showErrorState(ESE_MC_ERROR_COLOR, err->mcError);
    tftDL_showErrorState(ESE_IMD_COLOR, err->imdError);
    tftDL_showErrorState(ESE_AMS_COLOR, err->amsError);
    tftDL_showErrorState(ESE_BSPD_COLOR, err->bspdError);
    tftDL_showErrorState(ESE_GLV_COLOR, err->glvLowVolt);

    static struct {
        char buf[11];
    } *const hvc_error_num_str = (void *) (tftDL_errorData + ESE_HVC_ERROR_NUM_STR);

    snprintf(
        hvc_error_num_str->buf, sizeof(hvc_error_num_str->buf),
        "%04x", err->hvcErrorNum
    );

    static struct {
        char buf[15];
    } *const mc_error_num_str = (void *) (tftDL_errorData + ESE_MC_ERROR_NUM_STR);

    snprintf(
        mc_error_num_str->buf, sizeof(mc_error_num_str->buf),
        "%08x", err->mcErrorNum
    );

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
    tftCoCmd(tft, tftDL->len, tftDL->data, true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void setConfigContextString(int8_t scroll_index) {
    char* context_string = config_menu_main_array[scroll_index].ESE_context_text_variable;
    uint32_t context_string_address_offset = ESE_CONTEXT_VAL;
    uint32_t *context_string_pointer = (void *) (tftDL_configData + context_string_address_offset);
    sprintf((char *) context_string_pointer, context_string);

}

// implements wraparound. Can't use modulo since it's min is not always 0. Could do (val % max-min) + min but that's less readable
uint8_t configValueIncrementer(uint8_t value, uint8_t value_min, uint8_t value_max, bool up_requested, bool down_requested) {
    uint8_t new_value = value;
    if(up_requested){
        if (value + 1 > value_max)
            new_value = value_min;
        else
            new_value++;
    }
    if(down_requested){
        if (value - 1 < value_min)
            new_value = value_max;
        else
            new_value--;
    }
    return new_value;
}

void setConfigIncrementValue(int8_t scroll_index, bool up_requested, bool down_requested) {
    // calculate the varoius addresses to modify 
    uint32_t value_address_offset = config_menu_main_array[scroll_index].ESE_value_variable;
    uint32_t *value_address_pointer = (void *) (tftDL_configData + value_address_offset);

    // lut for custom enum
    char** custom_enum_lut = config_menu_main_array[scroll_index].ESE_value_string_lut;
    
    // type of value being modified
    cmr_config_t value_type = config_menu_main_array[scroll_index].value.type;
    // current value of the item
    uint8_t value = config_menu_main_array[scroll_index].value.value;
    uint8_t value_min = config_menu_main_array[scroll_index].min;
    uint8_t value_max = config_menu_main_array[scroll_index].max;
    
    // unsigned_integer,
    // custom_enum

    char buffer[5];

    switch(value_type){
        case unsigned_integer: 
            // treat it like an integer
        case integer:
            value = configValueIncrementer(value, value_min, value_max, up_requested, down_requested); 
            snprintf((char *) value_address_pointer, 4, "%3d", value);
            break;
        case boolean:
            if(up_requested || down_requested){
            	value = !value;
            }
            sprintf((char *) value_address_pointer, config_boolean_string_lut[value]);
            break;
        case float_1_decimal:
            value = configValueIncrementer(value, value_min, value_max, up_requested, down_requested); 
            snprintf(buffer, 5, "%4d", value);
            buffer[0] = buffer[1];
            buffer[1] = buffer[2];
            buffer[2] = '.';
            sprintf((char *) value_address_pointer, buffer);
            break;
        case float_2_decimal:
            value = configValueIncrementer(value, value_min, value_max, up_requested, down_requested); 
            snprintf(buffer, 5, "%4d", value);
            buffer[0] = buffer[1];
            buffer[1] = '.';
            sprintf((char *) value_address_pointer, buffer);
            break;
        case custom_enum:
        	// -1 since index off of 0
            value = configValueIncrementer(value, value_min, value_max - 1, up_requested, down_requested);
            sprintf((char *) value_address_pointer, custom_enum_lut[value]);
    };

    //Flush the modified value
    config_menu_main_array[scroll_index].value.value = value;
}

void setConfigSelectionColor(int8_t scroll_index, int8_t restore_index) {
  
    // calculate the varoius addresses to modify 
    uint32_t background_address_offset = config_menu_main_array[scroll_index].ESE_background_color_variable;
    uint32_t *background_item_pointer = (void *) (tftDL_configData + background_address_offset);

    uint32_t restore_background_address_offset = config_menu_main_array[restore_index].ESE_background_color_variable;
    uint32_t *restore_background_item_pointer = (void *) (tftDL_configData + restore_background_address_offset);

    // modify the actual addresses 
    *background_item_pointer = (uint32_t) SELECTED_MENU_COLOR;
    *restore_background_item_pointer = (uint32_t) NOT_SELECTED_MENU_COLOR;

    return;
}

void drawLatestConfigValues(){
    // loop through all the elements of the array and appropriately render the variables
    for(int i = 0; i < MAX_MENU_ITEMS; i++){
        setConfigIncrementValue(i, false, false);
    }
}

// TODO: Document
void tftDL_configUpdate(){
    if (dim_first_time_config_screen){
    	drawLatestConfigValues();
    	dim_first_time_config_screen = false;
    }
    if (redraw_new_driver_profiles){
        drawLatestConfigValues();
        redraw_new_driver_profiles = false;
    }

    static int8_t current_scroll_index = 0; // start with a value of -1 to enter driver config first
    static bool paddle_prev_active = false;
    static uint32_t paddle_time_since_change = 0;
    // Handles D Pad logic accounting for Driver Profile annoyance
    // If there is a move request
    if (config_move_request !=0) {
        // record previous index so we can clear colour
        int8_t prev_scroll_index = current_scroll_index;

        if (current_scroll_index == DRIVER_PROFILE_INDEX) {
            // if we're on the driver profile we either go to first or last grid square
            if (config_move_request > 0) {
                current_scroll_index++;
            } else {
                current_scroll_index = MAX_MENU_ITEMS - 1;
            }
        } else if  (config_move_request == CONFIG_SCREEN_NUM_COLS &&
                    current_scroll_index >= MAX_MENU_ITEMS - CONFIG_SCREEN_NUM_COLS) {
            // if we're in the bottom row and go down, we go to driver square
            current_scroll_index = DRIVER_PROFILE_INDEX;
        } else if  (config_move_request == -CONFIG_SCREEN_NUM_COLS &&
                    current_scroll_index <= CONFIG_SCREEN_NUM_COLS) {
            // if we're in the top row and go up, we go to the driver square
            current_scroll_index = DRIVER_PROFILE_INDEX;
        } else if (((current_scroll_index - 1) % CONFIG_SCREEN_NUM_COLS) == 0 &&
                 config_move_request == -1) {
            // if we're at the left and go left, we wrap around to the right
            current_scroll_index += CONFIG_SCREEN_NUM_COLS - 1;
        } else if (((current_scroll_index - 1) % CONFIG_SCREEN_NUM_COLS) == CONFIG_SCREEN_NUM_COLS -1 &&
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
            return; // both are an error
        
        // TODO: Finish multiple driver integration
        // if (current_scroll_index == DRIVER_PROFILE_INDEX) return;
        // handle new driver requested
        if (current_scroll_index == DRIVER_PROFILE_INDEX) {
            flush_config_screen_to_cdc = true;
            waiting_for_cdc_to_confirm_config = true;
            // Wait for CDC to confirm old driver params first
            while (waiting_for_cdc_to_confirm_config) {    
            }

            // Change driver
            setConfigIncrementValue(current_scroll_index, config_increment_up_requested, config_increment_down_requested);
            waiting_for_cdc_new_driver_config = true;
            while(waiting_for_cdc_new_driver_config){
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

        // left paddle logic
        if (config_paddle_left_request > 0) {
            if (!paddle_prev_active) {
                setConfigIncrementValue(current_scroll_index, false, true);
                paddle_prev_active = true;
                paddle_time_since_change = 0;
            } else {
                paddle_time_since_change += 1;
                float increment_freq = (float)config_paddle_left_request / (float)paddle_time_scale_factor;
                // increment count relative to display update period (ie period of this function)
                float increment_count =  (float)TFT_UPDATE_PERIOD_MS / increment_freq;
                if ((float)paddle_time_since_change >= increment_count) {
                    setConfigIncrementValue(current_scroll_index, false, true);
                    paddle_time_since_change = 0;
                }
            }
        } else if (config_paddle_right_request > 0) {
            if (!paddle_prev_active) {
                setConfigIncrementValue(current_scroll_index, true, false);
                paddle_prev_active = true;
                paddle_time_since_change = 0;
            } else {
                paddle_time_since_change += 1;
                float increment_freq = (float)config_paddle_right_request / (float)paddle_time_scale_factor;
                // increment count relative to display update period (ie period of this function)
                float increment_count =  (float)TFT_UPDATE_PERIOD_MS / increment_freq;
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
