#include "movella.h"
#include <CMR/can_types.h> 
#include <math.h>

volatile sensoric_state_t sensoric_state;

void sensoric_parse(uint16_t canID, volatile void *payload) {
    
    volatile void* payload;

    canDaqRX_t sensoric_msg;
    bool msg_found = false;
    for(sensoric_msg = CANRX_DAQ_SENSORIC_VEL_ANG_POI; sensoric_msg <= CANRX_DAQ_SENSORIC_INFO; sensoric_msg++) {
        if(canID == canDaqRXMeta[sensoric_msg].canID) {
            msg_found = true;
            break;
        }
    }

    if(!msg_found) 
        return;

    switch (sensoric_msg)
    {
        // TODO: parse int stuff? what does parse_int do..

    case CANRX_DAQ_SENSORIC_VEL_ANG_POI:
        volatile cmr_canSensoricVelAngPoi_t *vel_ang_poi = payload;
        volatile int16_t vel_X_poi = &vel_ang_poi -> vel_X_poi;
        volatile int16_t vel_Y_poi = &vel_ang_poi -> vel_Y_poi;
        volatile int16_t vel_Z_poi = &vel_ang_poi -> vel_Z_poi;
        volatile int16_t ang_S_poi = &vel_ang_poi -> ang_S_poi;
        sensoric_state.vel_ang_poi.x = vel_X_poi;
        sensoric_state.vel_ang_poi.y = vel_Y_poi;
        sensoric_state.vel_ang_poi.z = vel_Z_poi;  
        sensoric_state.vel_ang_poi.ang_s = ang_S_poi;
        break;

    case CANRX_DAQ_SENSORIC_DIST_POI:
        volatile cmr_canSensoricDistPoi_t *dist_poi = payload;
        volatile int32_t dist_A_poi = &dist_poi -> dist_A_poi;
        volatile int16_t radius_poi = &dist_poi -> radius_poi;
        volatile int16_t acc_C_poi = &dist_poi -> acc_C_poi;
        sensoric_state.dist_poi.dist_a = dist_A_poi;
        sensoric_state.dist_poi.radius = radius_poi;
        sensoric_state.dist_poi.acc_c = acc_C_poi;
        break;

    case CANRX_DAQ_SENSORIC_PITCH_ROLL:
        volatile cmr_canSensoricPitchRoll_t *pitch_roll = payload;
        volatile int16_t pitch = &pitch_roll -> pitch;
        volatile int16_t roll = &pitch_roll -> roll;
        sensoric_state.pitch_roll.pitch = pitch;
        sensoric_state.pitch_roll.roll = roll;
        break;
    
    case CANRX_DAQ_SENSORIC_ACC_HOR:
        volatile cmr_canSensoricAccHor_t *acc_hor = payload;
        volatile int16_t acc_X_hor = &acc_hor -> acc_X_hor;
        volatile int16_t acc_Y_hor = &acc_hor -> acc_Y_hor;
        volatile int16_t acc_Z_hor = &acc_hor -> acc_Z_hor;
        sensoric_state.acc_hor.x = acc_X_hor;
        sensoric_state.acc_hor.y = acc_Y_hor;
        sensoric_state.acc_hor.z = acc_Z_hor;
        break;

    case CANRX_DAQ_SENSORIC_RATE_HOR:
        volatile cmr_canSensoricRateHor_t *rate_hor = payload;
        volatile int16_t rate_X_hor = &rate_hor -> rate_X_hor;
        volatile int16_t rate_Y_hor = &rate_hor -> rate_Y_hor;
        volatile int16_t rate_Z_hor = &rate_hor -> rate_Z_hor;
        sensoric_state.rate_hor.x = rate_X_hor;
        sensoric_state.rate_hor.y = rate_Y_hor;
        sensoric_state.rate_hor.z = rate_Z_hor;
        break;
    
    case CANRX_DAQ_SENSORIC_VEL_ANG:
        volatile cmr_canSensoricVelAng_t *vel_ang = payload;
        volatile int16_t vel_X = &vel_ang -> vel_X;
        volatile int16_t vel_Y = &vel_ang -> vel_Y; 
        volatile int16_t vel_A = &vel_ang -> vel_A;
        volatile int16_t ang_S = &vel_ang -> ang_S;
        sensoric_state.vel_ang.x = vel_X;
        sensoric_state.vel_ang.y = vel_Y;
        sensoric_state.vel_ang.z = vel_A;
        sensoric_state.vel_ang.ang_s = ang_S;
        break;
    
    case CANRX_DAQ_SENSORIC_DIST:
        volatile cmr_canSensoricDist_t *dist = payload;
        volatile int32_t dist_A = &dist -> dist_A;
        volatile int16_t radius = &dist -> radius;
        volatile int16_t acc_C = &dist -> acc_C;
        sensoric_state.dist.dist_a = dist_A;
        sensoric_state.dist.radius = radius;
        sensoric_state.dist.acc_c = acc_C;
        break;

    case CANRX_DAQ_SENSORIC_ACC:
        volatile cmr_canSensoricAcc_t *acc = payload;
        volatile int16_t acc_X = &acc -> acc_X;
        volatile int16_t acc_Y = &acc -> acc_Y;
        volatile int16_t acc_Z = &acc -> acc_Z;
        sensoric_state.acc.x = acc_X;
        sensoric_state.acc.y = acc_Y;
        sensoric_state.acc.z = acc_Z;
        break;

    case CANRX_DAQ_SENSORIC_RATE:
        volatile cmr_canSensoricRate_t *rate = payload;
        volatile int16_t rate_X = &rate -> rate_X;
        volatile int16_t rate_Y = &rate -> rate_Y;
        volatile int16_t rate_Z = &rate -> rate_Z;
        sensoric_state.rate.x = rate_X;
        sensoric_state.rate.y = rate_Y;
        sensoric_state.rate.z = rate_Z;
        break;

    case CANRX_DAQ_SENSORIC_VEL_ANG_SP:
        volatile cmr_canSensoricVelAngSp_t *vel_ang_sp = payload;
        volatile int16_t vel_A_sp = &vel_ang_sp -> vel_A_sp;
        volatile int16_t vel_S_sp = &vel_ang_sp -> vel_S_sp;
        volatile uint16_t quality_ch0 = &vel_ang_sp -> quality_ch0;
        volatile uint16_t quality_ch1 = &vel_ang_sp -> quality_ch1;
        sensoric_state.vel_ang_sp.vel_a = vel_A_sp;
        sensoric_state.vel_ang_sp.vel_s = vel_S_sp;
        sensoric_state.vel_ang_sp.quality_ch0 = quality_ch0;
        sensoric_state.vel_ang_sp.quality_ch1 = quality_ch1;
        break;

    case CANRX_DAQ_SENSORIC_DIST_VEL_SP:
        volatile cmr_canSensoricDistVelSp_t *dist_vel_sp = payload;
        volatile int32_t dist_A_sp = &dist_vel_sp -> dist_A_sp;
        volatile int16_t vel_X_sp = &dist_vel_sp -> vel_X_sp;
        volatile int16_t vel_Y_sp = &dist_vel_sp -> vel_Y_sp;
        sensoric_state.dist_vel_sp.dist_a = dist_A_sp;
        sensoric_state.dist_vel_sp.vel_x = vel_X_sp;
        sensoric_state.dist_vel_sp.vel_y = vel_Y_sp;
        break;
    
    default:
        break;
    }
}