#ifndef CONFIG_SCREEN_HELPER_H
#define CONFIG_SCREEN_HELPER_H
#include <stdint.h>
#include <stdio.h>        // snprintf
#include <string.h>        // memcpy()
#include <stdbool.h>

__attribute__((unused)) static uint32_t this_stops_the_compiler_from_complaining[] = {
#include <DIM-ESE/config.rawh>
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
} cmr_config_t;

// TODO: Move this to the stm32 drivers
typedef enum{
    drs_off,
    drs_dynamic,
    drs_slippery,
    drs_airbrake,
    num_values_drs_enum,
} cmr_drs_policy_t;

// TODO: Move this to the stm32 drivers
typedef enum{
    regen_off,
    regen_parallel,
    regen_one_pedal,
    regen_parallel_one_pedal,
    num_values_regen_enum,
} cmr_regen_policy_t;

typedef enum{
    Default,
    Saral,
    Pravir,
    Gabe,
    Test1,
    Test2,
    Test3,
    Test4,
    Test5,
    num_values_driver_enum
} cmr_driver_profile_t;

typedef enum{
    DRIVER_PROFILE_INDEX,
    ACCEL_TGT_INDEX,
    SLIP_RATIO_ACCEL_INDEX,
    POWER_LIMIT_INDEX,
    TORQUE_BIAS_INDEX,
	MAX_REGEN_FORCE_INDEX,
    MAX_REGEN_PRESSURE_INDEX,
    REGEN_INDEX,
    REGEN_BIAS_INDEX,
    TRACTION_CONTROL_INDEX,
    SLIP_RATIO_TRACTION_INDEX,
    TORQUE_VECTORING_INDEX,
    TORQUE_VECTORING_GAIN_INDEX,
    MAX_TORQUE_INDEX,
    MAX_SPEED_INDEX,
    WET_INDEX,
    DRS_SWANGLE_INDEX,
    DRS_THROTTLE_INDEX,
    DRS_BRAKE_INDEX,
    MAX_MENU_ITEMS // The elements in the config array
} config_menu_main_array_index_t;


/*************** Various on screen string luts ***************/
extern char* config_boolean_string_lut[2];
extern char* config_driver_string_lut[9];
extern char* config_drs_string_lut[5];
extern char* config_regen_string_lut[5];
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
    cmr_config_t type;
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
    uint8_t min; // these will have to be converted at the time of initing
    uint8_t max; // these will have to be converted at the time of initing;
}config_menu_item_t;


config_menu_item_t test;
extern volatile config_menu_item_t config_menu_main_array[MAX_MENU_ITEMS];

//////// HELPER FUNCTIONS /////////////////
bool getProcessedValue(void* returnPointer, int index, cmr_config_t expected_type);

#endif
