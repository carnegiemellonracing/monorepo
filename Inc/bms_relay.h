/*
 * bms_relay.h
 *
 *  Created on: Sep 5, 2020
 *      Author: vamsi
 */

#ifndef BMS_RELAY_H_
#define BMS_RELAY_H_

#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"

//CMR BMS States
typedef enum {AIR_POS_RELAY = 0x03,
              AIR_NEG_RELAY = 0x02,
              PRECHARGE_RELAY = 0x01,
              DISCHARGE_RELAY = 0x00} BMS_relay_t;
              
typedef enum {OPEN = 0x01,
              CLOSED = 0x02} BMS_relay_state_t;
              
const static uint8_t RELAY_CONTACTOR_MASK = 0x01;
const static uint8_t RELAY_STATUS_MASK = 0x10;

void relaySetup(void);
              
uint8_t setRelay(BMS_relay_t relay, BMS_relay_state_t state);

uint8_t getRelayStatus(void);//BMS_relay_t relay);
bool checkRelayPowerFault();

#endif /* BMS_RELAY_H_ */
