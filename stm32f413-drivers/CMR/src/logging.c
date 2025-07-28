/**
 * @file logging.c
 * @brief logging interface
 * 
 * @author Carnegie Mellon Racing
 */

#include "CMR/logging.h"
#include "CMR/can.h"
#include "CMR/can_ids.h"
#include "CMR/can_types.h"


#include <CMR/rcc.h>    // cmr_rccADCClockEnable(), cmr_rccGPIOClockEnable()
#include <CMR/panic.h>  // cmr_panic()
#include <string.h>  // cmr_panic()

static cmr_can_t* can;

/**
 * @brief Initializes the logging module
 * 
 * @param can_obj the can interface we will be sending over
 */
void cmr_logging_init(cmr_can_t* can_obj){
  can = can_obj;
}

/**
 * @brief Hashes a string into a uint8_t
 * 
 * @param str the string we wish to hash
 */
static uint8_t hash_string_u8(const char *str) {
    uint32_t hash = hash_key;

    while (*str) {
        hash = hash * 33 + (uint8_t) (*str); 
        str++;
    }

    return (uint8_t)(hash & 0xFF); 
}

/**
 * @brief Callback for CAN receive FIFO message pending.
 * 
 * @param error the error we are trying to log
 * @param location Where the error is called from. Use __FILE__ macro
 * @param location Where the error is called from. Use __LINE__ macros
 */
void cmr_log(cmr_log_errors_t error, const char* file_name, uint16_t lineNumber){

  cmr_canLogging_t log;
  log.errorCode = (uint8_t) error;
  log.fileHash = hash_string_u8(file_name);;
  log.lineNumber = lineNumber;

  TickType_t logging_can_timeout_ms = 10;
  cmr_canTX(can, CAN_ID_LOGGING, &log, sizeOf(log), logging_can_timeout_ms);
}


