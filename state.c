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
         cmr_canState_t newState = DIM_requested_state + 1;
         if (is_valid_state_request(newState)) {
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
         if (is_valid_state_request(newState)) {
             DIM_requested_state = newState;
         }
     }
 }

 bool is_valid_state_request(cmr_canState_t state) {
     return (state == CMR_CAN_GLV_ON ||
             state == CMR_CAN_HV_EN ||
             state == CMR_CAN_RTD);
 }
