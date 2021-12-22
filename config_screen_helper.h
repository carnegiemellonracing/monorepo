#include <stdint.h>
#include <stdio.h>        // snprintf
#include <string.h>        // memcpy()

static uint32_t this_stops_the_compiler_from_complaining[] = {
#include "ESE/config.rawh"
};

#define NOT_SELECTED_MENU_COLOR 0x04000000
#define SELECTED_MENU_COLOR 0x04AA0000 
#define MAX_MENU_ITEMS 17;

enum config_menu_items{
    ACCL_TGT, 
    SLIP_RATIO_ACCEL,
    BURNOUT,
    TRQ_BIAS,
    MAX_RGN,
    MAX_PSSR,
    ONE_PDL,
    RGN_BIAS,
    TRAC_CTL,
    SLIP_RATIO_DRIVE,
    TRQ_VEC,
    TV_GAIN,
    MAX_TRQ,
    MAX_SPD,
    DRS,
    WET
};

typedef enum{
    float_1_decimal,
    float_2_decimal,
    boolean,
    integer,
    unsigned_integer,
    custom_enum,
} cmr_config_t;

/**
 * @brief the value struct for the config menu
 * 
 * floats with 1 decimal are stored as ints with 10^1 multiplier
 * floats with 2 decimal are stored as ints with 10^2 multiplier
 * booleans are stored as ints with 0 as false, 1 as true
 * integers are stored as uints and simply casted at the time they're needed
 * 
 * restricted to a uint8_t for now since that's what is transmitted over CAN
 */
typedef struct{
    cmr_config_t type;
    uint8_t value; 
}cmr_config_value_t;

typedef struct {
    char *name;
    int32_t ESE_background_color_variable;
    int32_t ESE_value_color_variable;
    int32_t ESE_value_variable;
    char *ESE_context_text_variable;
    cmr_config_value_t value;
    uint8_t min; // these will have to be converted at the time of initing
    uint8_t max; // these will have to be converted at the time of initing;
}config_menu_item_t;


config_menu_item_t test;
config_menu_item_t config_menu_main_array[17] = {
    {
        .name = "Driver Profile",
        .ESE_background_color_variable = ESE_DRIVER_BOX,
        .ESE_value_color_variable = ESE_DRIVER_COLOR, 
        .ESE_value_variable = ESE_DRIVER_VAL,
        .ESE_context_text_variable = "The driver profile",
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 10, // scaled by 10 bc 1 decimal point 'float'
        .max = 200, // scaled by 10 bc 1 decimal point 'float'
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
        .ESE_context_text_variable = "FB correction solely for lnc ctl",
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
        .ESE_value_color_variable = ESE_BURNOUT_COLOR,
        .ESE_value_variable = ESE_BURNOUT_VAL,
        .ESE_context_text_variable = "Max regen force at Max Pressure",
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
        .ESE_context_text_variable = "Brake pssr when max regen is applied",
        .value = {
            .type = float_1_decimal,
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
        .value = {
            .type = custom_enum,
            .value = 0
        },
        .min = 0,
        .max = 3,
    },
    {
        .name = "Regen Bias",
        .ESE_background_color_variable = ESE_RGN_BIAS_BOX,
        .ESE_value_color_variable = ESE_RGN_BIAS_COLOR,
        .ESE_value_variable = ESE_RGN_BIAS_VAL,
        .ESE_context_text_variable = "All mode regen bias. 0 is FWBrake",
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
        .ESE_context_text_variable = "Enable or disable traction control",
        .value = {
            .type = boolean,
            .value = 0
        },
        .min = 0,
        .max = 1,
    },
    {
        .name = "Torque Vectoring",
        .ESE_background_color_variable = ESE_TRQ_VEC_BOX,
        .ESE_value_color_variable = ESE_MAX_TRQ_COLOR,
        .ESE_value_variable = ESE_TRQ_VEC_VAL,
        .ESE_context_text_variable = "Enable or disable torque vectoring",
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
        .ESE_context_text_variable = "Aggressiveness of torque vectoring",
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
        .ESE_context_text_variable = "Max available torque for all modes",
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
        .ESE_context_text_variable = "Max speed for all modes",
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
        .ESE_context_text_variable = "DRS Control policy in all modes", 
        .value = {
            .type = boolean,
            .value = 0
        },
        .min = 0,
        .max = 1,
    },
    {
        .name = "Wet",
        .ESE_background_color_variable = ESE_WET_BOX,
        .ESE_value_color_variable = ESE_WET_COLOR,
        .ESE_value_variable = ESE_WET_VAL,
        .ESE_context_text_variable = "Wet mode, aggressive TC & capped TRQ",
        .value = {
            .type = boolean,
            .value = 0
        },
        .min = 0,
        .max = 1,
    }
};
