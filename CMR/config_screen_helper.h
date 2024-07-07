#ifndef CONFIG_SCREEN_HELPER_H
#define CONFIG_SCREEN_HELPER_H
#include <stdint.h>
#include <stdio.h>        // snprintf
#include <string.h>        // memcpy()
#include <stdbool.h>

__attribute__((unused)) static uint32_t this_stops_the_compiler_from_complaining[] = {
#include <config.rawh>
};


#define NOT_SELECTED_MENU_COLOR 0x04000000
#define SELECTED_MENU_COLOR 0x04AA0000

typedef enum{
    float_1_decimal,
    float_2_decimal,
    boolean,
    integer,
    unsigned_integer,
    custom_enum,
} cmr_config_type_t;
typedef enum{
    Default,
    Trent,
    Pravir,
    Tony,
    num_values_driver_enum
} cmr_driver_profile_t;

typedef enum {
    DRIVER_PROFILE_INDEX,
    POWER_LIM_INDEX,
    SF_OP_MARGIN_INDEX,
    YRC_KP_INDEX,
    TC_LUT_Y_SCALE_INDEX,
    TC_LUT_X_SCALE_INDEX,
    PEDAL_REGEN_STRENGTH_INDEX,
    PADDLE_MAX_REGEN_INDEX,
    FFLAUNCH_FEEDBACK_INDEX,
    K_LIN_INDEX,
    K_YAW_INDEX,
    K_TIE_INDEX,
    K_EFF_INDEX,
    DRS_THROTTLE_THRESH_INDEX,
    DRS_SWANGLE_THRESH_INDEX,
    PLACEHOLDER_1_INDEX,
    PLACEHOLDER_2_INDEX,
    MAX_MENU_ITEMS // The elements in the config array
} config_menu_main_array_index_t;


/*************** Various on screen string luts ***************/
extern char* config_boolean_string_lut[2];
extern char* config_driver_string_lut[4];
/************************************************************/


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
    cmr_config_type_t type;
    uint8_t value;
}cmr_config_value_t;

typedef struct {
    char *name;
    int32_t ESE_background_color_variable;
    int32_t ESE_value_color_variable;
    int32_t ESE_value_variable;
    char *ESE_context_text_variable;
    char** ESE_value_string_lut;
    cmr_config_value_t value;
    size_t ESE_string_len;
    uint8_t min; // these will have to be converted at the time of initing
    uint8_t max; // these will have to be converted at the time of initing;
}config_menu_item_t;

extern volatile config_menu_item_t config_menu_main_array[MAX_MENU_ITEMS];

//////// HELPER FUNCTIONS /////////////////
bool getProcessedValue(void* returnPointer, int index, cmr_config_type_t expected_type);

#endif
