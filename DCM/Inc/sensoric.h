#ifndef SENSORIC_H
#define SENSORIC_H

#include "can.h"

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
        int16_t acc_c
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

#endif