/**
 * @file sensoric.c
 * @brief Functions for parsing sensoric CAN messages
 *
 * @author Carnegie Mellon Racing
 */

#include "sensoric.h"
#include <CMR/can_types.h> 
#include <math.h>


#define KMH2MPS(x) ((x) / 3.6f)
#define DEG2RAD(x) ((x) * ((float)M_PI / 180.0f))

#define SCALE_VEL_KMH   0.02f
#define SCALE_ACC       0.02f
#define SCALE_RATE_DPS  0.02f
#define SCALE_ANGS_DEG  0.003f
#define SCALE_DIST_M    0.001f
#define SCALE_RADIUS_M  0.01f
#define SCALE_ACCC      0.02f

volatile sensoric_state_t sensoric_state;

// TODO: FLOAT STRUCT VERSION

void sensoric_parse(uint16_t canID, volatile void *payload) {
    /**
 * @brief Parses sensoric CAN data and stores to sensoric_state.
 *
 * @note returning false indicates no sensoric message to parse.
 */


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

        // swtiched payload 

    case CANRX_DAQ_SENSORIC_VEL_ANG_POI:
        memcpy((void*)&(sensoric_state.vel_ang_poi), payload, sizeof(sensoric_state.vel_ang_poi));
        break;

    case CANRX_DAQ_SENSORIC_DIST_POI:
        memcpy((void*)&(sensoric_state.dist_poi), payload, sizeof(sensoric_state.dist_poi));
        break;

    case CANRX_DAQ_SENSORIC_PITCH_ROLL:
        memcpy((void*)&(sensoric_state.pitch_roll), payload, sizeof(sensoric_state.pitch_roll));
        break;
    
    case CANRX_DAQ_SENSORIC_ACC_HOR:
        memcpy((void*)&(sensoric_state.acc_hor), payload, sizeof(sensoric_state.acc_hor));
        break;

    case CANRX_DAQ_SENSORIC_RATE_HOR:
        memcpy((void*)&(sensoric_state.rate_hor), payload, sizeof(sensoric_state.rate_hor));
        break;
    
    case CANRX_DAQ_SENSORIC_VEL_ANG:
        memcpy((void*)&(sensoric_state.vel_ang), payload, sizeof(sensoric_state.vel_ang));
        break;
    
    case CANRX_DAQ_SENSORIC_DIST:
        memcpy((void*)&(sensoric_state.dist), payload, sizeof(sensoric_state.dist));
        break;

    case CANRX_DAQ_SENSORIC_ACC:
        memcpy((void*)&(sensoric_state.acc), payload, sizeof(sensoric_state.acc));
        break;

    case CANRX_DAQ_SENSORIC_RATE:
        memcpy((void*)&(sensoric_state.rate), payload, sizeof(sensoric_state.rate));
        break;

    case CANRX_DAQ_SENSORIC_VEL_ANG_SP:
        memcpy((void*)&(sensoric_state.vel_ang_sp), payload, sizeof(sensoric_state.vel_ang_sp));
        break;

    case CANRX_DAQ_SENSORIC_DIST_VEL_SP:
        memcpy((void*)&(sensoric_state.dist_vel_sp), payload, sizeof(sensoric_state.dist_vel_sp));
        break;
    
    default:
        break;
    }
}