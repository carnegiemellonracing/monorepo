#include <stdint.h>
#include <CMR/config_screen_helper.h> // for config_screen_data tx

__attribute__((unused)) static uint32_t rawh_import[] = {
#include <DIM-ESE/config.rawh>
};
/*************** Various on screen string luts ***************/
char* config_boolean_string_lut[2] = {"Off", "On\0"};
// Pad the length to always be 10
// There can be at most 1 null terminator (to center it)
char* config_driver_string_lut[4] = {
    " Default \0",
    "  Trent  \0",
    "  Pravir  ",
    "   Tony   "};
/************************************************************/


volatile config_menu_item_t config_menu_main_array[MAX_MENU_ITEMS] = {
    [DRIVER_PROFILE_INDEX] = {
        .name = "Driver Profile",
        .ESE_background_color_variable = ESE_DRIVER_BOX,
        .ESE_value_color_variable = ESE_DRIVER_COLOR,
        .ESE_value_variable = ESE_DRIVER_VAL,
        .ESE_context_text_variable = "The driver profile",
        .ESE_value_string_lut = config_driver_string_lut,
        .ESE_string_len = 11, // including null terminator
        .value = {
            .type = custom_enum,
            .value = 0
        },
        .min = 0, // scaled by 10 bc 1 decimal point 'float'
        .max = num_values_driver_enum, // scaled by 10 bc 1 decimal point 'float'
    },
    [POWER_LIM_INDEX] = {
        .name = "Power Limit",
        .ESE_background_color_variable = ESE_POWER_LIM_BOX,
        .ESE_value_color_variable = ESE_POWER_LIM_COLOR,
        .ESE_value_variable = ESE_POWER_LIM_VAL,
        .ESE_context_text_variable = "Power Limit for Endurance [kW]",
        .value = {
            .type = unsigned_integer,
            .value = 30
        },
        .min = 0,
        .max = 150,
    },
    [SF_OP_MARGIN_INDEX] = {
        .name = "SF OP Sfty Mgn",
        .ESE_background_color_variable = ESE_SF_OP_MARGIN_BOX,
        .ESE_value_color_variable = ESE_SF_OP_MARGIN_COLOR,
        .ESE_value_variable = ESE_SF_OP_MARGIN_VAL,
        .ESE_context_text_variable = "Safety Filter Over Pwr Margin [kW]",
        .value = {
            .type = float_1_decimal,
            .value = 100
        },
        .min = 0,
        .max = 255,
    },
    [YRC_KP_INDEX] = {
        .name = "YRC Kp",
        .ESE_background_color_variable = ESE_YRC_KP_BOX,
        .ESE_value_color_variable = ESE_YRC_KP_COLOR,
        .ESE_value_variable = ESE_YRC_KP_VAL,
        .ESE_context_text_variable = "Yaw Rate Controller Kp",
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 0,
        .max = 255,
    },
    [TC_LUT_Y_SCALE_INDEX] = {
        .name = "TC LUT Horiz Scl",
        .ESE_background_color_variable = ESE_TC_LUT_Y_SCALE_BOX,
        .ESE_value_color_variable = ESE_TC_LUT_Y_SCALE_COLOR,
        .ESE_value_variable = ESE_TC_LUT_Y_SCALE_VAL,
        .ESE_context_text_variable = "Traction Control LUT Horiz Scale",
        .value = {
            .type = float_2_decimal,
            .value = 100
        },
        .min = 0,
        .max = 255,
    },
    [TC_LUT_X_SCALE_INDEX] = {
        .name = "TC LUT Vert Scl",
        .ESE_background_color_variable = ESE_TC_LUT_X_SCALE_BOX,
        .ESE_value_color_variable = ESE_TC_LUT_X_SCALE_COLOR,
        .ESE_value_variable = ESE_TC_LUT_X_SCALE_VAL,
        .ESE_context_text_variable = "Traction Control LUT Vert Scale",
        .value = {
            .type = float_2_decimal,
            .value = 100
        },
        .min = 0,
        .max = 255,
    },
    [PEDAL_REGEN_STRENGTH_INDEX] = {
        .name = "Pedal Regen Str",
        .ESE_background_color_variable = ESE_PEDAL_REGEN_STRENGTH_BOX,
        .ESE_value_color_variable = ESE_PEDAL_REGEN_STRENGTH_COLOR,
        .ESE_value_variable = ESE_PEDAL_REGEN_STRENGTH_VAL,
        .ESE_context_text_variable = "Pedal Regen Braking Strength",
        .value = {
            .type = unsigned_integer,
            .value = 0
        },
        .min = 0,
        .max = 100,
    },
    [PADDLE_MAX_REGEN_INDEX] = {
        .name = "Paddl Max Regen",
        .ESE_background_color_variable = ESE_PADDLE_MAX_REGEN_BOX,
        .ESE_value_color_variable = ESE_PADDLE_MAX_REGEN_COLOR,
        .ESE_value_variable = ESE_PADDLE_MAX_REGEN_VAL,
        .ESE_context_text_variable = "Paddle Regen Braking Max",
        .value = {
            .type = unsigned_integer,
            .value = 50
        },
        .min = 0,
        .max = 100,
    },
    [FFLAUNCH_FEEDBACK_INDEX] = {
        .name = "FF Launch",
        .ESE_background_color_variable = ESE_FF_LAUNCH_FB_BOX,
        .ESE_value_color_variable = ESE_FF_LAUNCH_FB_COLOR,
        .ESE_value_variable = ESE_FF_LAUNCH_FB_VAL,
        .ESE_context_text_variable = "FFLaunch",
        .ESE_string_len = 4, // including null terminator
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 0,
        .max = 100,
    },
    [DRS_THROTTLE_THRESH_INDEX] = {
        .name = "DRS Throtl Thrsh",
        .ESE_background_color_variable = ESE_DRS_THROTTLE_THRESH_BOX,
        .ESE_value_color_variable = ESE_DRS_THROTTLE_THRESH_COLOR,
        .ESE_value_variable = ESE_DRS_THROTTLE_THRESH_VAL,
        .ESE_context_text_variable = "DRS Throttle Threshold",
        .value = {
            .type = unsigned_integer,
            .value = 20
        },
        .min = 0,
        .max = 255,
    },
    [DRS_SWANGLE_THRESH_INDEX] = {
        .name = "DRS Swngl Thrsh",
        .ESE_background_color_variable = ESE_DRS_SWANGLE_THRESH_BOX,
        .ESE_value_color_variable = ESE_DRS_SWANGLE_THRESH_COLOR,
        .ESE_value_variable = ESE_DRS_SWANGLE_THRESH_VAL,
        .ESE_context_text_variable = "DRS Swangle Threshold",
        .value = {
            .type = unsigned_integer,
            .value = 20
        },
        .min = 0,
        .max = 120,
    },
    [K_LIN_INDEX] = {
        .name = "k_lin",
        .ESE_background_color_variable = ESE_K_LIN_BOX,
        .ESE_value_color_variable = ESE_K_LIN_COLOR,
        .ESE_value_variable = ESE_K_LIN_VAL,
        .ESE_context_text_variable = "k_lin",
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 0,
        .max = 100,
    },
    [K_YAW_INDEX] = {
        .name = "k_yaw",
        .ESE_background_color_variable = ESE_K_YAW_BOX,
        .ESE_value_color_variable = ESE_K_YAW_COLOR,
        .ESE_value_variable = ESE_K_YAW_VAL,
        .ESE_context_text_variable = "k_yaw",
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 0,
        .max = 100,
    },
    [K_TIE_INDEX] = {
        .name = "k_tie",
        .ESE_background_color_variable = ESE_K_TIE_BOX,
        .ESE_value_color_variable = ESE_K_TIE_COLOR,
        .ESE_value_variable = ESE_K_TIE_VAL,
        .ESE_context_text_variable = "k_tie",
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 0,
        .max = 100,
    },
    [K_EFF_INDEX] = {
        .name = "k_eff",
        .ESE_background_color_variable = ESE_K_EFF_BOX,
        .ESE_value_color_variable = ESE_K_EFF_COLOR,
        .ESE_value_variable = ESE_K_EFF_VAL,
        .ESE_context_text_variable = "k_eff",
        .ESE_string_len = 4, // including null terminator
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 0,
        .max = 100,
    },
    [LAUNCH_SLOPE_INDEX] = {
        .name = "Launch Slope (m/s^2)",
        .ESE_background_color_variable = ESE_LAUNCH_SLOPE_BOX,
        .ESE_value_color_variable = ESE_LAUNCH_SLOPE_COLOR,
        .ESE_value_variable = ESE_LAUNCH_SLOPE_VAL,
        .ESE_context_text_variable = "Launch Slope",
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 0,
        .max = 255,
    },
    [SLOW_SPEED_INDEX] = {
        .name = "Slow Speed (m/s)",
        .ESE_background_color_variable = ESE_SLOW_SPEED_BOX,
        .ESE_value_color_variable = ESE_SLOW_SPEED_COLOR,
        .ESE_value_variable = ESE_SLOW_SPEED_VAL,
        .ESE_context_text_variable = "Slow Speed",
        .value = {
            .type = float_1_decimal,
            .value = 0
        },
        .min = 0,
        .max = 100,
    }
};


//////// HELPER FUNCTIONS /////////////////
/**
 * @param returnPointer the pointer where you want your return value. Type is based on expected_type
 * @param index use the cmr_config_type_t enum to index into the appropriate value
 * @param expected_type the cmr_config_type_t type of pointer passed into returnPointer. Used for error checking
 *
 * @return bool of whether the value has been fetched correctly
 */
bool getProcessedValue(void* returnPointer, int index, cmr_config_type_t expected_type){
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
