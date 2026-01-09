/**
 * @file sensoric.c
 * @brief Functions for parsing sensoric CAN messages
 *
 * @author Carnegie Mellon Racing
 */

#include "movella.h"
#include <CMR/can_types.h> 
#include <math.h>

volatile sensoric_state_t sensoric_state;

void sensoric_parse(uint16_t canID, volatile void *payload) {
    /**
 * @brief Parses sensoric CAN data and stores to sensoric_state.
 *
 * @note returning false indicates no sensoric message to parse.
 */

    volatile void* payload;

    canDaqRX_t sensoric_msg;
    bool msg_found = false;
    for(sensoric_msg = CANRX_DAQ_SENSORIC_VEL_ANG_POI; sensoric_msg <= CANRX_DAQ_SENSORIC_DIST_VEL_SP; sensoric_msg++) {
        if(canID == canDaqRXMeta[sensoric_msg].canID) {
            msg_found = true;
            break;
        }
    }

    if(!msg_found) 
        return;

    switch (sensoric_msg)
    {

    case CANRX_DAQ_SENSORIC_VEL_ANG_POI:
        memcpy(payload, &(sensoric_state.vel_ang_poi), sizeof(sensoric_state.vel_ang_poi));
        break;

    case CANRX_DAQ_SENSORIC_DIST_POI:
        memcpy(payload, &(sensoric_state.dist_poi), sizeof(sensoric_state.dist_poi));
        break;

    case CANRX_DAQ_SENSORIC_PITCH_ROLL:
        memcpy(payload, &(sensoric_state.pitch_roll), sizeof(sensoric_state.pitch_roll));
        break;
    
    case CANRX_DAQ_SENSORIC_ACC_HOR:
        memcpy(payload, &(sensoric_state.acc_hor), sizeof(sensoric_state.acc_hor));
        break;

    case CANRX_DAQ_SENSORIC_RATE_HOR:
        memcpy(payload, &(sensoric_state.rate_hor), sizeof(sensoric_state.rate_hor));
        break;
    
    case CANRX_DAQ_SENSORIC_VEL_ANG:
        memcpy(payload, &(sensoric_state.vel_ang), sizeof(sensoric_state.vel_ang));
        break;
    
    case CANRX_DAQ_SENSORIC_DIST:
        memcpy(payload, &(sensoric_state.dist), sizeof(sensoric_state.dist));
        break;

    case CANRX_DAQ_SENSORIC_ACC:
        memcpy(payload, &(sensoric_state.acc), sizeof(sensoric_state.acc));
        break;

    case CANRX_DAQ_SENSORIC_RATE:
        memcpy(payload, &(sensoric_state.rate), sizeof(sensoric_state.rate));
        break;

    case CANRX_DAQ_SENSORIC_VEL_ANG_SP:
        memcpy(payload, &(sensoric_state.vel_ang_sp), sizeof(sensoric_state.vel_ang_sp));
        break;

    case CANRX_DAQ_SENSORIC_DIST_VEL_SP:
        memcpy(payload, &(sensoric_state.dist_vel_sp), sizeof(sensoric_state.dist_vel_sp));
        break;
    
    default:
        break;
    }
}