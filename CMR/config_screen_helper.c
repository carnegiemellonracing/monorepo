#include <stdint.h>
#include <stdio.h>        // snprintf
#include <string.h>        // memcpy()
#include <CMR/config_screen_helper.h> // for config_screen_data tx

/*************** Various on screen string luts ***************/
char* config_boolean_string_lut[2] = {"Off", "On"};
char* config_driver_string_lut[8] = {"Saral", "Pravir", "Gabe", "Test1", "Test2", "Test3", "Test4", "Test5"};
char* config_drs_string_lut[5] = {"Dynmk", "Off", "Slpry", "Ar brk", ""};
char* config_regen_string_lut[5] = {"Off", "Prll", "One P", "Hybr", ""};
/************************************************************/


config_menu_item_t config_menu_main_array[MAX_MENU_ITEMS] = {
    {
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
    {
        .name = "Accl Tgt",
        .ESE_background_color_variable = ESE_ACCL_TGT_BOX,
        .ESE_value_color_variable = ESE_ACCEL_TGT_COLOR, 
        .ESE_value_variable = ESE_ACCEL_TGT_VAL,
        .ESE_context_text_variable = "FF accel for launch ctl [m/s2]",
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 10, // scaled by 10 bc 1 decimal point 'float'
        .max = 200, // scaled by 10 bc 1 decimal point 'float'
    },
    {
        .name = "Slip Ratio Accel",
        .ESE_background_color_variable = ESE_SLIP_RATIO_ACCEL_BOX,
        .ESE_value_color_variable = ESE_SLIP_RATIO_ACCEL_COLOR,
        .ESE_value_variable = ESE_SLIP_RATIO_ACCEL_VAL,
        .ESE_context_text_variable = "FB correction for lnc ctl",
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 10,
        .max = 20,
    },
    {
        .name = "Burnout",
        .ESE_background_color_variable = ESE_BURNOUT_BOX,
        .ESE_value_color_variable = ESE_BURNOUT_COLOR,
        .ESE_value_variable = ESE_BURNOUT_VAL,
        .ESE_context_text_variable = "Initial burnout t in launch ctl",
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 0,
        .max = 30,
    },
    {
        .name = "Torque Bias",
        .ESE_background_color_variable = ESE_TRQ_BIAS_BOX,
        .ESE_value_color_variable = ESE_TRQ_BIAS_COLOR,
        .ESE_value_variable = ESE_TRQ_BIAS_VAL,
        .ESE_context_text_variable = "All mode torque bias. 0 is FWD",
        .value = {
            .type = integer,
            .value = 0
        },
        .min = 0,
        .max = 100,
    },
    {
        .name = "Max Regen Force",
        .ESE_background_color_variable = ESE_MAX_RGN_BOX,
        .ESE_value_color_variable = ESE_MAX_RGN_COLOR,
        .ESE_value_variable = ESE_MAX_RGN_VAL,
        .ESE_context_text_variable = "Max regen force at Max Pssr",
        .value = {
            .type = integer,
            .value = 0
        },
        .min = 0,
        .max = 100,
    },
    {
        .name = "Max Regen Pressure",
        .ESE_background_color_variable = ESE_MAX_PSSR_BOX,
        .ESE_value_color_variable = ESE_MAX_PSSR_COLOR, 
        .ESE_value_variable = ESE_MAX_PSSR_VAL,
        .ESE_context_text_variable = "When max regen is applied",
        .value = {
            .type = integer,
            .value = 0
        },
        .min = 0,
        .max = 150,
    },
    {
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
    {
        .name = "Regen Bias",
        .ESE_background_color_variable = ESE_RGN_BIAS_BOX,
        .ESE_value_color_variable = ESE_RGN_BIAS_COLOR,
        .ESE_value_variable = ESE_RGN_BIAS_VAL,
        .ESE_context_text_variable = "RegenBrake bias. 0 is FWBrake",
        .value = {
            .type = integer,
            .value = 0
        },
        .min = 0,
        .max = 100,
    },
    {
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
    {
        .name = "Slip Ratio traction control",
        .ESE_background_color_variable = ESE_SLIP_RATIO_DRV_BOX,
        .ESE_value_color_variable = ESE_SLIP_RATIO_DRV_COLOR, 
        .ESE_value_variable = ESE_SLIP_RATIO_DRV_VAL,
        .ESE_context_text_variable = "Max SR before traction Ctl",
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 10,
        .max = 30, //TODO: annotate this
    },
    {
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
    {
        .name = "Torque Vectoring Gain",
        .ESE_background_color_variable = ESE_TV_GAIN_BOX,
        .ESE_value_color_variable = ESE_TV_GAIN_COLOR,
        .ESE_value_variable = ESE_TV_GAIN_VAL,
        .ESE_context_text_variable = "Torque vectoring gain",
        .value = {
            .type = integer,
            .value = 0
        },
        .min = 0,
        .max = 100,
    },
    {
        .name = "Max Torque",
        .ESE_background_color_variable = ESE_MAX_TRQ_BOX,
        .ESE_value_color_variable = ESE_MAX_TRQ_COLOR, 
        .ESE_value_variable = ESE_MAX_TRQ_VAL,
        .ESE_context_text_variable = "Max torque, all modes",
        .value = {
            .type = integer,
            .value = 0
        },
        .min = 0,
        .max = 100,
    },
    {
        .name = "Max Speed",
        .ESE_background_color_variable = ESE_MAX_SPD_BOX,
        .ESE_value_color_variable = ESE_MAX_SPD_COLOR, 
        .ESE_value_variable = ESE_MAX_SPD_VAL,
        .ESE_context_text_variable = "Max speed, all modes",
        .value = {
            .type = integer,
            .value = 0
        },
        .min = 2,
        .max = 100,
    },
    {
        .name = "DRS",
        .ESE_background_color_variable = ESE_DRS_BOX,
        .ESE_value_color_variable = ESE_DRS_COLOR,
        .ESE_value_variable = ESE_DRS_VAL,
        .ESE_context_text_variable = "DRS Control policy, all modes", 
        .ESE_value_string_lut = config_drs_string_lut,
        .value = {
            .type = custom_enum,
            .value = 0
        },
        .min = 0,
        .max = num_values_drs_enum,
    },
    {
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
    }
};

// burnout value breaks everything (potentially s?)
// max pressure also breaks everything (probs psi?)


// ese screen editor
// drs value at value not label
