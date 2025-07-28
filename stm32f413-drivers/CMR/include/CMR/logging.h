/**
 * @file logging.h
 * @brief logging interface.
 *
 * @author Carnegie Mellon Racing
 */

#pragma once
#include <stdint.h>     // uint32_t

void cmr_logging_init(cmr_can_t* can_obj);
void cmr_log(cmr_log_errors_t error, int location);


//*************** DO NOT MODIFY BELOW THIS LINE. AUTOMATICALLY GENERATED ********************

static uint32_t hash_key = 1;

typedef enum {
  CMR_LOG_A,
  CMR_LOG_B,
  CMR_LOG_C,
} cmr_log_errors_t;
