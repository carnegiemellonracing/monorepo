/**
 * @file error.h
 * @brief  Checks for any errors in the DIM module
 *
 * @author Ayush Garg
 */

 #pragma  once

#include <CMR/can_types.h>

void update_errors(cmr_canError_t* errors);