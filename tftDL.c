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

/** @brief Packets to send to the DL on error.
 * See datasheet */
static const tftContent_t *tftDL_errorContent[] = {
    &tftContent_RobotoMono_Bold_72_L4,
    &tftContent_RobotoMono_Bold_40_L4,
};

/** @brief Complete data required to draw the error screen.
 * Exposed to interface consumers. */
const tftDL_t tftDL_error = {
    .len = sizeof(tftDL_errorData),
    .data = tftDL_errorData,

    .contentLen = sizeof(tftDL_errorContent) / sizeof(tftDL_errorContent[0]),
    .content = tftDL_errorContent
};

/** @brief Config Screen */
static uint32_t tftDL_configData[] = {
#include <DIM-ESE/config.rawh>
};

/** @brief Packets to send to the DL on error.
 * See datasheet */
static const tftContent_t *tftDL_configContent[] = {
    &tftContent_RobotoMono_Bold_72_L4,
    &tftContent_RobotoMono_Bold_40_L4,
};

/** @brief Complete data required to draw the error screen.
 * Exposed to interface consumers. */
const tftDL_t tftDL_config = {
    .len = sizeof(tftDL_configData),
    .data = tftDL_configData,

    .contentLen = sizeof(tftDL_configContent) / sizeof(tftDL_configContent[0]),
    .content = tftDL_configContent
};

/** @brief RTD Screen */
static uint32_t tftDL_RTDData[] = {
#include <DIM-ESE/RTD.rawh>
};

/** @brief Complete data required to draw the
 * ready-to-drive screen.
 * Exposed to interface consumers. */
static const tftContent_t *tftDL_RTDContent[] = {
    &tftContent_RobotoMono_Bold_72_L4,
    &tftContent_RobotoMono_Bold_40_L4,
};

/** @brief Complete data required to draw the
 * Ready-to-drive screen.
 * Exposed to interface consumers. */
const tftDL_t tftDL_RTD = {
    .len = sizeof(tftDL_RTDData),
    .data = tftDL_RTDData,

    .contentLen = sizeof(tftDL_RTDContent) / sizeof(tftDL_RTDContent[0]),
    .content = tftDL_RTDContent
};

/** @brief How to draw a single bar dynamically. */
typedef struct {
    uint32_t *addr;  /**< @brief Top-left vertex addr */
    uint8_t topY;    /**< @brief Top edge Y coord. */
    uint8_t botY;    /**< @brief Bot edge Y coord. */
    int32_t maxVal;  /**< @brief Logical value
    * corresponding to bottom edge */
    int32_t minVal;  /**< @brief Logical value
    * corresponding to top edge */
} tftDL_bar_t;

/** @brief Bitposition of Y-coordinate byte in vertices */
#define TFT_DL_VERTEX_Y_BIT 12

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
static void tftDL_showGear() {
    /** @brief Characters for each state. */
    size_t stateCharsLen = 6;
    static const char *stateChars[] = {
        [CMR_CAN_UNKNOWN] =      "?????",
        [CMR_CAN_GLV_ON] =       "  GLV",
        [CMR_CAN_HV_EN] =        " HVEN",
        [CMR_CAN_RTD] =          "  RTD",
        [CMR_CAN_ERROR] =        "ERROR",
        [CMR_CAN_CLEAR_ERROR] =  "CLEAR",
    };

    /** @brief Characters for each gear. */
    static const char* gearChars[CMR_CAN_GEAR_LEN] = {
        [CMR_CAN_GEAR_UNKNOWN] =   "?????",
        [CMR_CAN_GEAR_REVERSE] =   "  Rev",
        [CMR_CAN_GEAR_SLOW] =      " Slow",
        [CMR_CAN_GEAR_FAST] =      " Fast",
        [CMR_CAN_GEAR_ENDURANCE] = "Endur",
        [CMR_CAN_GEAR_AUTOX] =     "AutoX",
        [CMR_CAN_GEAR_SKIDPAD] =   " Skid",
        [CMR_CAN_GEAR_ACCEL] =     "Accel",
        [CMR_CAN_GEAR_TEST] =      " Test"
    };
    cmr_canState_t stateVSM = stateGetVSM();
    cmr_canGear_t gear = stateGetGear();
    cmr_canState_t stateVSMReq = stateGetVSMReq();

    const char* stateChar = (stateVSM < stateCharsLen)
        ? stateChars[stateVSM]
        : stateChars[CMR_CAN_UNKNOWN];

    const char* gearChar = (gear < CMR_CAN_GEAR_LEN)
        ? gearChars[gear]
        : gearChars[CMR_CAN_GEAR_UNKNOWN];

    const char* stateReqChar = (stateVSMReq < stateCharsLen)
            ? stateChars[stateVSMReq]
            : stateChars[CMR_CAN_UNKNOWN];

    static struct {
        char buf[STATEDISPLAYLEN];
    } *const vsmState_str = (void *) (tftDL_RTDData + ESE_VSM_STATE_STR);

    memcpy((void *) vsmState_str->buf, (void *) stateChar, STATEDISPLAYLEN);

    static struct {
        char buf[GEARDISPLAYLEN];
    } *const gearState_str = (void *) (tftDL_RTDData + ESE_GEAR_STR);

    memcpy((void *) gearState_str->buf, (void *) gearChar, GEARDISPLAYLEN);

    static struct {
        char buf[STATEDISPLAYLEN];
    } *const reqState_str = (void *) (tftDL_RTDData + ESE_REQ_STATE_STR);

    memcpy((void *) reqState_str->buf, (void *) stateReqChar, STATEDISPLAYLEN);
}

/**
 * @brief sets the display message from the RAM
 * Sets at top and 3 notes on right side
 */
void tftDL_showRAMMsg() {
    static struct {
            char buf[RAMDISPLAYLEN];
        } *const ramMsg_str = (void *) (tftDL_RTDData + ESE_RAM_MSG_STR);
        memcpy((void *) ramMsg_str->buf, (void *) RAMBUF, RAMDISPLAYLEN);
    static struct {
            char buf[NOTEDISPLAYLEN];
        } *const note1_str = (void *) (tftDL_RTDData + ESE_NOTE_1_STR);
        memcpy((void *) note1_str->buf, (void *) &(RAMBUF[NOTE1_INDEX]), NOTEDISPLAYLEN);
    static struct {
            char buf[NOTEDISPLAYLEN];
        } *const note2_str = (void *) (tftDL_RTDData + ESE_NOTE_2_STR);
        memcpy((void *) note2_str->buf, (void *) &(RAMBUF[NOTE2_INDEX]), NOTEDISPLAYLEN);
    static struct {
            char buf[NOTEDISPLAYLEN];
        } *const note3_str = (void *) (tftDL_RTDData + ESE_NOTE_3_STR);
        memcpy((void *) note3_str->buf, (void *) &(RAMBUF[NOTE3_INDEX]), NOTEDISPLAYLEN);
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
    uint32_t speed_mph,
    int32_t hvVoltage_mV,
    int32_t power_kW,
    bool motorTemp_yellow,
    bool motorTemp_red,
    bool acTemp_yellow,
    bool acTemp_red,
    bool mcTemp_yellow,
    bool mcTemp_red,
    int32_t motorTemp_C,
    int32_t acTemp_C,
    int32_t mcTemp_C,
    int32_t glvVoltage_V
) {
    tftDL_RTDwriteInt(ESE_HV_VOLTAGE_STR, 4, "%3ld", hvVoltage_mV / 1000);
    tftDL_RTDwriteInt(ESE_MOTOR_TEMP_STR, 3, "%2ld", motorTemp_C);
    tftDL_RTDwriteInt(ESE_AC_TEMP_STR, 3, "%2ld", acTemp_C);
    tftDL_RTDwriteInt(ESE_MC_TEMP_STR, 3, "%2ld", mcTemp_C);
    tftDL_RTDwriteInt(ESE_RTD_GLV_STR, 3, "%2d", glvVoltage_V);
    tftDL_RTDwriteInt(ESE_POWER_STR, 3, "%2ld", power_kW);


    /* Memorator color */
    uint32_t *memorator_color = (void *) (tftDL_RTDData + ESE_MEMO_TEXT_COLOR);
    uint32_t memorator_color_cmd = memoratorPresent ? green : red;
    *memorator_color = memorator_color_cmd;

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

    tftDL_showGear();
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



    tftDL_showErrorState(ESE_PTCp_COLOR, err->ptcpTimeout);
    tftDL_showErrorState(ESE_APC_COLOR, err->apcTimeout);
    tftDL_showErrorState(ESE_HVC_COLOR, err->hvcTimeout);
    tftDL_showErrorState(ESE_VSM_COLOR, err->vsmTimeout);
    tftDL_showErrorState(ESE_VSM_COLOR, err->vsmTimeout);
    tftDL_showErrorState(ESE_VSM_COLOR, err->vsmTimeout);
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
    uint32_t context_string_address_offset = ESE_CONTEXT_VALUE;
    uint32_t *context_string_pointer = (void *) (tftDL_configData + context_string_address_offset);
    sprintf(context_string_pointer, context_string);

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
            snprintf(value_address_pointer, 4, "%3d", value);
            break;
        case boolean:
            if(up_requested || down_requested){
            	value = !value;
                sprintf(value_address_pointer, config_boolean_string_lut[value]);
            }
            break;
        case float_1_decimal:
            value = configValueIncrementer(value, value_min, value_max, up_requested, down_requested); 
            snprintf(buffer, 5, "%4d", value);
            buffer[0] = buffer[1];
            buffer[1] = buffer[2];
            buffer[2] = '.';
            sprintf(value_address_pointer, buffer);
            break;
        case float_2_decimal:
            value = configValueIncrementer(value, value_min, value_max, up_requested, down_requested); 
            snprintf(buffer, 5, "%4d", value);
            buffer[0] = buffer[1];
            buffer[1] = '.';
            sprintf(value_address_pointer, buffer);
            break;
        case custom_enum:
        	// -1 since index off of 0
            value = configValueIncrementer(value, value_min, value_max - 1, up_requested, down_requested);
            sprintf(value_address_pointer, custom_enum_lut[value]);
    };

    //Flush the modified value
    config_menu_main_array[scroll_index].value.value = value;
}

void setConfigSelectionColor(int8_t scroll_index) {
    // index who's color to restore
    uint8_t restore_index = scroll_index - 1; // underflow is expected :)
	if (scroll_index == 0){
		restore_index = MAX_MENU_ITEMS;
		restore_index--; // issues with this not working in one line since MAX_MENU_ITEMS is a #define hence 2 lines
	}
    
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

// TODO: Document
void tftDL_configUpdate(){
    static int8_t current_scroll_index = -1; // start with a value of -1 to enter driver config first
    
    // update scroll and clear selection values
    if (config_scroll_requested) {
    	current_scroll_index++;
        current_scroll_index = current_scroll_index % MAX_MENU_ITEMS;


        // clear the selection value just in case no one accidently presses both buttons at the same time
        config_scroll_requested = false;

        // call the background color updater
        setConfigSelectionColor(current_scroll_index);
        setConfigContextString(current_scroll_index);
        
        config_increment_up_requested = false;
        config_increment_down_requested = false;
    }

    // if there are no scroll values, then check/implement selection values
    else if (config_increment_up_requested || config_increment_down_requested) {
        if (config_increment_down_requested && config_increment_up_requested)
            return; // both are an error
        
        setConfigIncrementValue(current_scroll_index, config_increment_up_requested, config_increment_down_requested);
        config_increment_up_requested = false;
        config_increment_down_requested = false;

        // handle new driver requested
        if(current_scroll_index == DRIVER_PROFILE_INDEX){
            waiting_for_cdc_new_driver_config = true;
            flush_config_screen_to_cdc = true;
            while(waiting_for_cdc_new_driver_config){
                // wait for the new driver to be selected
            }
        }
    }

    return;
    
}
