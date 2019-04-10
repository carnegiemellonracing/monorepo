/**
 * @file gpio.h
 * @brief DIM state request handling logic
 *
 * @author Carnegie Mellon Racing
 */

#include "state.h"  // interface to implement
#include "main.h"   // shared DIM specific stuff
#include "gpio.h"   // for debugging purposes right now

 /**
  * @brief Handler for the state up button presses.
  *
  * @param pressed whether the button is depressed.
  */
 void state_up_button(bool pressed) {
     if (pressed) {
         if (VSM_state == CMR_CAN_ERROR) {
             DIM_requested_state = CMR_CAN_GLV_ON;
             return;
         }
         cmr_canState_t newState = DIM_requested_state + 1;
         if (is_valid_state_request(VSM_state, newState)) {
             DIM_requested_state = newState;
         }
     }
 }

 /**
  * @brief Handler for the state down button presses.
  *
  * @param pressed whether the button is depressed.
  */
 void state_down_button(bool pressed) {
     if (pressed) {
         cmr_canState_t newState = DIM_requested_state - 1;
         if (is_valid_state_request(VSM_state, newState)) {
             DIM_requested_state = newState;
         }
     }
 }

 bool is_valid_state_request(cmr_canState_t curr_state, cmr_canState_t new_state) {
     switch (curr_state) {
         case CMR_CAN_UNKNOWN :
             return (new_state == CMR_CAN_GLV_ON);
             break;
         case CMR_CAN_GLV_ON :
             return (new_state == CMR_CAN_GLV_ON) ||
                    (new_state == CMR_CAN_HV_EN);
             break;
         case CMR_CAN_HV_EN :
             return (new_state == CMR_CAN_GLV_ON) ||
                    (new_state == CMR_CAN_HV_EN) ||
                    (new_state == CMR_CAN_RTD);
             break;
         case CMR_CAN_RTD :
             return (new_state == CMR_CAN_HV_EN) ||
                    (new_state == CMR_CAN_RTD);
             break;
         case CMR_CAN_ERROR :
             return (new_state == CMR_CAN_GLV_ON);
             break;
         case CMR_CAN_CLEAR_ERROR :
             return (new_state == CMR_CAN_GLV_ON);
             break;
         default :
             return 0;
     }
 }
