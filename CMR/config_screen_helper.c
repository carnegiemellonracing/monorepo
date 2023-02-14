#include <stdint.h>
#include <stdio.h>        // snprintf
#include <string.h>        // memcpy()
#include <CMR/config_screen_helper.h> // for config_screen_data tx

/*************** Various on screen string luts ***************/
char* config_boolean_string_lut[2] = {"Off", "On"};
char* config_driver_string_lut[9] = {"Default", "Saral", "Pravir", "Gabe", "Test1", "Test2", "Test3", "Test4", "Test5"};
char* config_drs_string_lut[5] = {"Dynmk", "Off", "Slpry", "Ar brk", ""};
char* config_regen_string_lut[5] = {"Off", "Prll", "One P", "Hybr", ""};
/************************************************************/


volatile config_menu_item_t config_menu_main_array[MAX_MENU_ITEMS] = {
    [DRIVER_PROFILE_INDEX] = {
        .name = "Driver Profile",
        .ESE_background_color_variable = ESE_DRIVER_BOX,
        .ESE_value_color_variable = ESE_DRIVER_COLOR, 
        .ESE_value_variable = ESE_DRIVER_VAL,
        .ESE_context_text_variable = "The driver profile",
        .ESE_value_string_lut = config_driver_string_lut,
        .value = {
            .type = custom_enum,
            .value = 0
        },
        .min = 0, // scaled by 10 bc 1 decimal point 'float'
        .max = num_values_driver_enum, // scaled by 10 bc 1 decimal point 'float'
    },
    [ACCEL_TGT_INDEX] = {
        .name = "Accl Tgt",
        .ESE_background_color_variable = ESE_ACCL_TGT_BOX,
        .ESE_value_color_variable = ESE_ACCEL_TGT_COLOR, 
        .ESE_value_variable = ESE_ACCEL_TGT_VAL,
        .ESE_context_text_variable = "FF accel for launch ctl [m/s2]",
        .value = {
            .type = float_1_decimal,
            .value = 13
        },
        .min = 10, // scaled by 10 bc 1 decimal point 'float'
        .max = 200, // scaled by 10 bc 1 decimal point 'float'
    },
    [SLIP_RATIO_ACCEL_INDEX] = {
        .name = "Slip Ratio Accel",
        .ESE_background_color_variable = ESE_SLIP_RATIO_ACCEL_BOX,
        .ESE_value_color_variable = ESE_SLIP_RATIO_ACCEL_COLOR,
        .ESE_value_variable = ESE_SLIP_RATIO_ACCEL_VAL,
        .ESE_context_text_variable = "FB correction for lnc ctl",
        .value = {
            .type = float_1_decimal,
            .value = 14
        },
        .min = 10,
        .max = 20,
    },
    [POWER_LIMIT_INDEX] = {
        .name = "Pwr Lmt",
        .ESE_background_color_variable = ESE_BURNOUT_BOX,
        .ESE_value_color_variable = ESE_BURNOUT_COLOR,
        .ESE_value_variable = ESE_BURNOUT_VAL,
        .ESE_context_text_variable = "Power limit in endurance [kW]",
        .value = {
            .type = integer,
            .value = 5
        },
        .min = 1,
        .max = 70,
    },
    [TORQUE_BIAS_INDEX] = {
        .name = "Torque Bias",
        .ESE_background_color_variable = ESE_TRQ_BIAS_BOX,
        .ESE_value_color_variable = ESE_TRQ_BIAS_COLOR,
        .ESE_value_variable = ESE_TRQ_BIAS_VAL,
        .ESE_context_text_variable = "All mode torque bias. 0 is FWD",
        .value = {
            .type = integer,
            .value = 60
        },
        .min = 0,
        .max = 100,
    },
    [MAX_REGEN_FORCE_INDEX] = {
        .name = "Max Regen Force",
        .ESE_background_color_variable = ESE_MAX_RGN_BOX,
        .ESE_value_color_variable = ESE_MAX_RGN_COLOR,
        .ESE_value_variable = ESE_MAX_RGN_VAL,
        .ESE_context_text_variable = "Max regen force at Max Pssr",
        .value = {
            .type = integer,
            .value = 50
        },
        .min = 0,
        .max = 100,
    },
    [MAX_REGEN_PRESSURE_INDEX] = {
        .name = "Max Regen Pressure",
        .ESE_background_color_variable = ESE_MAX_PSSR_BOX,
        .ESE_value_color_variable = ESE_MAX_PSSR_COLOR, 
        .ESE_value_variable = ESE_MAX_PSSR_VAL,
        .ESE_context_text_variable = "When max regen is applied",
        .value = {
            .type = integer,
            .value = 50
        },
        .min = 0,
        .max = 150,
    },
    [REGEN_INDEX] = {
        .name = "Regen",
        .ESE_background_color_variable = ESE_REGEN_BOX,
        .ESE_value_color_variable = ESE_REGEN_COLOR, 
        .ESE_value_variable = ESE_REGEN_VAL,
        .ESE_context_text_variable = "Enable various regen modes",
        .ESE_value_string_lut = config_regen_string_lut,
        .value = {
            .type = custom_enum,
            .value = 0
        },
        .min = 0,
        .max = num_values_regen_enum,
    },
    [REGEN_BIAS_INDEX] = {
        .name = "Regen Bias",
        .ESE_background_color_variable = ESE_RGN_BIAS_BOX,
        .ESE_value_color_variable = ESE_RGN_BIAS_COLOR,
        .ESE_value_variable = ESE_RGN_BIAS_VAL,
        .ESE_context_text_variable = "RegenBrake bias. 0 is FWBrake",
        .value = {
            .type = integer,
            .value = 50
        },
        .min = 0,
        .max = 100,
    },
    [TRACTION_CONTROL_INDEX] = {
        .name = "Traction control",
        .ESE_background_color_variable = ESE_TRAC_CTL_BOX,
        .ESE_value_color_variable = ESE_TRAC_CTL_COLOR, 
        .ESE_value_variable = ESE_TRAC_CTL_VAL,
        .ESE_context_text_variable = "Enable traction control",
        .value = {
            .type = boolean,
            .value = 0
        },
        .min = 0,
        .max = 1,
    },
    [SLIP_RATIO_TRACTION_INDEX] = {
        .name = "Slip Ratio traction control",
        .ESE_background_color_variable = ESE_SLIP_RATIO_DRV_BOX,
        .ESE_value_color_variable = ESE_SLIP_RATIO_DRV_COLOR, 
        .ESE_value_variable = ESE_SLIP_RATIO_DRV_VAL,
        .ESE_context_text_variable = "Max SR before traction Ctl",
        .value = {
            .type = float_1_decimal,
            .value = 15
        },
        .min = 10,
        .max = 30, //TODO: annotate this
    },
    [TORQUE_VECTORING_INDEX] = {
        .name = "Torque Vectoring",
        .ESE_background_color_variable = ESE_TRQ_VEC_BOX,
        .ESE_value_color_variable = ESE_MAX_TRQ_COLOR,
        .ESE_value_variable = ESE_TRQ_VEC_VAL,
        .ESE_context_text_variable = "Enable torque vectoring",
        .value = {
            .type = boolean,
            .value = 0
        },
        .min = 0,
        .max = 1,
    },
    [TORQUE_VECTORING_GAIN_INDEX] = {
        .name = "Torque Vectoring Gain",
        .ESE_background_color_variable = ESE_TV_GAIN_BOX,
        .ESE_value_color_variable = ESE_TV_GAIN_COLOR,
        .ESE_value_variable = ESE_TV_GAIN_VAL,
        .ESE_context_text_variable = "Torque vectoring gain",
        .value = {
            .type = integer,
            .value = 2
        },
        .min = 0,
        .max = 30,
    },
    [MAX_TORQUE_INDEX] = {
        .name = "Max Torque",
        .ESE_background_color_variable = ESE_MAX_TRQ_BOX,
        .ESE_value_color_variable = ESE_MAX_TRQ_COLOR, 
        .ESE_value_variable = ESE_MAX_TRQ_VAL,
        .ESE_context_text_variable = "Max torque, all modes",
        .value = {
            .type = integer,
            .value = 50
        },
        .min = 0,
        .max = 100,
    },
    [MAX_SPEED_INDEX] = {
        .name = "Max Speed",
        .ESE_background_color_variable = ESE_MAX_SPD_BOX,
        .ESE_value_color_variable = ESE_MAX_SPD_COLOR, 
        .ESE_value_variable = ESE_MAX_SPD_VAL,
        .ESE_context_text_variable = "Max speed, all modes",
        .value = {
            .type = integer,
            .value = 30
        },
        .min = 2,
        .max = 100,
    },
    [WET_INDEX] = {
        .name = "Wet",
        .ESE_background_color_variable = ESE_WET_BOX,
        .ESE_value_color_variable = ESE_WET_COLOR,
        .ESE_value_variable = ESE_WET_VAL,
        .ESE_context_text_variable = "High TC & capped torque",
        .value = {
            .type = boolean,
            .value = 0
        },
        .min = 0,
        .max = 1,
    },
    [DRS_SWANGLE_INDEX] = {
        .name = "DRS Swangle Threshold",
        .ESE_background_color_variable = ESE_DRS_SWANGLE_BOX,
        .ESE_value_color_variable = ESE_DRS_SWANGLE_COLOR,
        .ESE_value_variable = ESE_DRS_SWANGLE_VAL,
        .ESE_context_text_variable = "High TC & capped torque",
        .value = {
            .type = boolean,
            .value = 0
        },
        .min = 0,
        .max = 1,
    },
    [DRS_THROTTLE_INDEX] = {
        .name = "DRS Throttle Threshold",
        .ESE_background_color_variable = ESE_DRS_THROTTLE_BOX,
        .ESE_value_color_variable = ESE_DRS_THROTTLE_COLOR,
        .ESE_value_variable = ESE_DRS_THROTTLE_VAL,
        .ESE_context_text_variable = "High TC & capped torque",
        .value = {
            .type = boolean,
            .value = 0
        },
        .min = 0,
        .max = 1,
    },
    [DRS_BRAKE_INDEX] = {
        .name = "DRS Brake Threshold",
        .ESE_background_color_variable = ESE_DRS_BRAKE_BOX,
        .ESE_value_color_variable = ESE_DRS_BRAKE_COLOR,
        .ESE_value_variable = ESE_DRS_BRAKE_VAL,
        .ESE_context_text_variable = "High TC & capped torque",
        .value = {
            .type = boolean,
            .value = 0
        },
        .min = 0,
        .max = 1,
    },

};

// burnout value breaks everything (potentially s?)
// max pressure also breaks everything (probs psi?)


// ese screen editor
// drs value at value not label


//////// HELPER FUNCTIONS /////////////////
/**
 * @param returnPointer the pointer where you want your return value. Type is based on expected_type
 * @param index use the cmr_config_t enum to index into the appropriate value
 * @param expected_type the cmr_config_t type of pointer passed into returnPointer. Used for error checking
 * 
 * @return bool of whether the value has been fetched correctly
 */
bool getProcessedValue(void* returnPointer, int index, cmr_config_t expected_type){
    if (config_menu_main_array[index].value.type != expected_type){
        return false;
    }
    switch(config_menu_main_array[index].value.type){
        case float_1_decimal: 
            *(float*)returnPointer = ((float) config_menu_main_array[index].value.value) / 10.f;
            return true;
        case float_2_decimal: 
            *(float*)returnPointer = ((float) config_menu_main_array[index].value.value) / 100.f;
            return true;
        case boolean: 
            *(bool*)returnPointer = ((bool) config_menu_main_array[index].value.value);
            return true;
        case integer: 
            *(int8_t*)returnPointer = ((int8_t) config_menu_main_array[index].value.value);
            return true;
        case unsigned_integer: 
            *(uint8_t*)returnPointer = ((uint8_t) config_menu_main_array[index].value.value);
            return true;
        case custom_enum: 
            // note, custom enum error checking doesn't exist. That's ok bc its not safety critical
            *(uint8_t*)returnPointer = ((uint8_t)config_menu_main_array[index].value.value);
            return true;
        default:
            return false;
    }
}
