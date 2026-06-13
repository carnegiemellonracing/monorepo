/**
 * @file as_error.c
 * @brief  Checks for any DV errors and if they exist then trigger the DV Relay
 * also logs the errors onto CAN flags
 *
 * @author Ayush Garg
 */

#include "can.h"        
#include "gpio.h" 
#include "sensors.h"  
#include "state.h"       

/**
 * @brief 
 * @param errors pointer to the current error flags which we insert the flags onto
 * 
 * @sideeffect updates the DV relay and the AS_ERROR GPIO pin
 * @return nothing
 */
void update_dv_errors(cmr_canError_t* errors) {

    cmr_canState_t vsm_state = stateGetVSM();
    cmr_canError_t error_present = CMR_CAN_ERROR_NONE;
    bool error_present_bool = false;
    bool ASMS_on = getASMS();
    bool is_AS_active_state =  ASMS_on && (vsm_state == CMR_CAN_AS_READY || 
                                vsm_state == CMR_CAN_AS_DRIVING);


    if (is_AS_active_state && cmr_sensorListGetError(&sensorList, SENSOR_CH_EBS_CURRENT_1_MA) != CMR_SENSOR_ERR_NONE){
        error_present |= CMR_CAN_ERROR_DIM_SOLENOID_CURRENT_1_OOR;
        error_present_bool = true;
    }

    if (is_AS_active_state && cmr_sensorListGetError(&sensorList, SENSOR_CH_EBS_CURRENT_2_MA) != CMR_SENSOR_ERR_NONE ){
        error_present |= CMR_CAN_ERROR_DIM_SOLENOID_CURRENT_2_OOR;
        error_present_bool = true;
    }

    if (ASMS_on && cmr_sensorListGetError(&sensorList, SENSOR_CH_EBS_PRESSURE_1_DECI_BAR) != CMR_SENSOR_ERR_NONE){
        error_present |= CMR_CAN_ERROR_DIM_TANK_PRESSURE_1_OOR;
        error_present_bool = true;
    }

    if (ASMS_on && cmr_sensorListGetError(&sensorList, SENSOR_CH_EBS_PRESSURE_2_DECI_BAR) != CMR_SENSOR_ERR_NONE){
        error_present |= CMR_CAN_ERROR_DIM_TANK_PRESSURE_2_OOR;
        error_present_bool = true;
    }

    cmr_gpioWrite(GPIO_AS_ERROR, error_present_bool);
    *errors |= error_present;
}
