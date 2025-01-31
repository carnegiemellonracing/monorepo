//
// Created by 26958 on 7/11/2024.
//

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#define NUM_VARS 4

typedef enum {
    LOWER = 0,
    UPPER,
    UNCONSTRAINED,
} box_variable_role_e;

typedef struct {
    double lower;
    double upper;
    box_variable_role_e role;
} box_variable_t;

#endif //COMMON_H
