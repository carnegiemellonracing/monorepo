/*
 * bms_relay.c
 *
 *  Created on: Sep 5, 2020
 *      Author: vamsi
 */



#include "bms_relay.h"

static volatile uint8_t relayUsageVector = 0;

void relaySetup() {
    // Discharge relay is default closed, initialize pin accordingly
    relayUsageVector = RELAY_CONTACTOR_MASK << DISCHARGE_RELAY;
}

uint8_t setRelay(BMS_relay_t relay, BMS_relay_state_t state) {
    
    switch (relay) {
        case AIR_POS_RELAY:
            if (state == OPEN) {
                relayUsageVector &= ~(RELAY_CONTACTOR_MASK << AIR_POS_RELAY);
                cmr_gpioWrite(GPIO_RELAY_PIN_AIR_POS, 0);
            } else {
                // Close relay
                relayUsageVector |= RELAY_CONTACTOR_MASK << AIR_POS_RELAY;
                cmr_gpioWrite(GPIO_RELAY_PIN_AIR_POS, 1);
            }
            break;
        case AIR_NEG_RELAY:
            if (state == OPEN) {
                relayUsageVector &= ~(RELAY_CONTACTOR_MASK << AIR_NEG_RELAY);
                cmr_gpioWrite(GPIO_RELAY_PIN_AIR_NEG, 0);
            } else {
                // Close relay
                relayUsageVector |= RELAY_CONTACTOR_MASK << AIR_NEG_RELAY;
                cmr_gpioWrite(GPIO_RELAY_PIN_AIR_NEG, 1);
            }
            break;
        case PRECHARGE_RELAY:
            if (state == OPEN) {
                relayUsageVector &= ~(RELAY_CONTACTOR_MASK << PRECHARGE_RELAY);
                cmr_gpioWrite(GPIO_RELAY_PIN_PRECHARGE, 0);
            } else {
                // Close relay
                relayUsageVector |= RELAY_CONTACTOR_MASK << PRECHARGE_RELAY;
                cmr_gpioWrite(GPIO_RELAY_PIN_PRECHARGE, 1);
            }
            break;
        case DISCHARGE_RELAY:
            if (state == OPEN) {
                relayUsageVector &= ~(RELAY_CONTACTOR_MASK << DISCHARGE_RELAY);
                // Discharge relay is default close, set high pin to open
                cmr_gpioWrite(GPIO_RELAY_PIN_DISCHARGE, 0);
            } else {
                // Close relay
                relayUsageVector |= RELAY_CONTACTOR_MASK << DISCHARGE_RELAY;
                cmr_gpioWrite(GPIO_RELAY_PIN_DISCHARGE, 1);
            }
            break;
        default:
            // Error, open relays
            cmr_gpioWrite(GPIO_RELAY_PIN_AIR_POS, 0);
            cmr_gpioWrite(GPIO_RELAY_PIN_AIR_NEG, 0);
            cmr_gpioWrite(GPIO_RELAY_PIN_PRECHARGE, 0);
            cmr_gpioWrite(GPIO_RELAY_PIN_DISCHARGE, 0);
            relayUsageVector = 0;//RELAY_CONTACTOR_MASK << DISCHARGE_RELAY;
            break;
    }
    
    return relayUsageVector;
    
}

uint8_t getRelayStatus(){
    // Status pin is high for OK, low for FAULT
    uint8_t relayStatusVector = 0;

	// AIR status feedback functionality removed

    return relayUsageVector | relayStatusVector;
}

bool checkRelayPowerFault() {
    return !cmr_gpioRead(GPIO_RELAY_PIN_POWER_FAULT_L);
}
