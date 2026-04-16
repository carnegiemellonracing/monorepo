/**
 * @file sensoric.h
 * @brief Functions for parsing sensoric CAN messages
 *
 * @author Carnegie Mellon Racing
 */

#ifndef SENSORIC_H
#define SENSORIC_H

#include <math.h>
#include "can.h"

// SCALING FACTORS FROM SENSORICSOLUTIONSOMSRACE.DBC 
// APPLY AS PHYSICAL_VALUE = RAW * SCALE_* 

#define KMH2MPS(x) ((x) / 3.6f) // convert km/h to m/s
#define DEG2RAD(x) ((x) * ((float)M_PI / 180.0f)) // convert degrees to radians

#define SCALE_VEL_KMH   0.02f // for velX, velY, velA, velXSp, velYSp, velASp,           [km/h]
#define SCALE_ACC       0.02f // for accX, accY, accZ, accXHor/YHor, ZHor, AccCPoi, AccC [m/s^2]
#define SCALE_RATE_DPS  0.02f // for rateX, rateY, rateZ, rateXHor/YHor/ZHor,            [deg/s]

#define SCALE_ANGS_DEG  0.003f // for AngS, AngSPoi, AnSSp, Roll, Pitch                  [deg]
#define SCALE_DIST_M    0.001f // for distance DistA, DistAPoi, DistASp                  [m]
#define SCALE_RADIUS_M  0.01f // for radius, Radius, RadiusPoi                           [m]

// RAW TO SI UNIT CONVERSIONS FOR CONVENIENCE
#define SENSORIC_VEL_TO_MPS(raw)        KMH2MPS((raw) * SCALE_VEL_KMH)
#define SENSORIC_RATE_TO_RADPS(raw)     DEG2RAD((raw) * SCALE_RATE_DPS)
#define SENSORIC_ANGS_TO_RAD(raw)       DEG2RAD((raw) * SCALE_ANGS_DEG)
#define SENSORIC_ACC_TO_MS2(raw)        ((raw) * SCALE_ACC)
#define SENSORIC_DIST_TO_M(raw)         ((raw) * SCALE_DIST_M)
#define SENSORIC_RADIUS_TO_M(raw)       ((raw) * SCALE_RADIUS_M)


typedef struct {

    struct {
        int16_t x;
        int16_t y;
        int16_t a;
        int16_t ang_s;
    } vel_ang_poi;

    struct {
        int32_t dist_a;
        int16_t radius;
        int16_t acc_c;
    } dist_poi;
    
    struct {
        int16_t roll;
        int16_t pitch;
    } pitch_roll;
    
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } acc_hor;

    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } rate_hor;
    
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        int16_t ang_s;
    } vel_ang;

    struct {
        int32_t dist_a;
        int16_t radius;
        int16_t acc_c;
    } dist;

    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } acc;

    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } rate;

    struct {
        int16_t vel_a;
        int16_t vel_s;
        uint16_t quality_ch0;
        uint16_t quality_ch1;
    } vel_ang_sp;

    struct {
        int32_t dist_a;
        int16_t vel_x;
        int16_t vel_y;
    } dist_vel_sp;

} sensoric_state_t;

extern volatile sensoric_state_t sensoric_state;

void sensoric_parse(uint16_t canID, volatile void *payload);

#endif // SENSORIC_H