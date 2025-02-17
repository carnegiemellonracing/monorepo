/**
 * @file tftDL.h
 * @brief TFT display list interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef TFTDL_H
#define TFTDL_H

#include <CMR/can_types.h>
#include <stddef.h>  // size_t
#include <stdint.h>  // uint32_t

#include "tft.h"         // tft interface

/** @brief Represents a display list. */
typedef struct tftDL tftDL_t;

/** @brief Exported startup screen definition for interface consumers. */
extern const tftDL_t tftDL_startup;

/** @brief Exported ready-to-drive screen definition
 * for interface consumers. */
extern const tftDL_t tftDL_RTD;

/** @brief Exported ready-to-drive screen definition
 * for interface consumers. */
extern const tftDL_t tftDL_racing_screen;

/** @brief Exported error screen definition for interface consumers. */
extern const tftDL_t tftDL_error;

/** @brief Exported error screen definition for interface consumers. */
extern const tftDL_t tftDL_config;

/** @brief Exported safety screen definition for interface consumers. */
extern const tftDL_t tftDL_safety_screen;

/** @brief Text buffer for messages from RAM */
// extern char RAMBUF[];

/** @brief draw out all the cells if the first time the screen is being displayed */
extern volatile bool dim_first_time_config_screen;
/** @brief Checks to see if the screen needs to be redrawn after getting new driver profiles */
extern volatile bool redraw_new_driver_profiles;

// Sizes for displaying to screen
#define GEARDISPLAYLEN 10
#define STATEDISPLAYLEN 13
#define DRSDISPLAYLEN 7

// Indices for accessing RAM Buffer + their lengths
#define PREV_TIME_INDEX 0
#define TARGET_TIME_INDEX 8
#define TIMEDISPLAYLEN 8

#define MESSAGE_INDEX 16
#define MESSAGEDISPLAYLEN 21

enum AMK_ERRORS{
    AMK_DC_BUS_ERROR = 1049,
    AMK_ENCODER_ERROR = 2310,
    AMK_CONVERTER_TEMP_ERROR = 2346,
    AMK_MOTOR_TEMP_ERROR = 2347,
    AMK_SYS_DIAG_1 = 3584,
    AMK_SYS_DIAG_2 = 3585,
    AMK_SYS_DIAG_3 = 3586,
    AMK_SYS_DIAG_4 = 3587,
    AMK_SYS_DIAG_5 = 3588,
    AMK_SYS_DIAG_6 = 3589,
    AMK_SYS_DIAG_7 = 3590,
    AMK_SYS_DIAG_8 = 3591,
    AMK_SYS_DIAG_9 = 3592,
    AMK_SYS_DIAG_10 = 3593,
    AMK_SYS_DIAG_11 = 3594,
    AMK_SYS_DIAG_12 = 3595,
    AMK_SYS_DIAG_13 = 3596,
    AMK_SYS_DIAG_14 = 3597,
    AMK_SYS_DIAG_15 = 3598,
    AMK_SYS_DIAG_16 = 3599,
    AMK_SYS_DIAG_17 = 3600,
    AMK_SYS_DIAG_18 = 3601,
    AMK_SYS_DIAG_19 = 3602,
};

void tftDL_RTDUpdate(
    memorator_status_t memoratorStatus,
    SBG_status_t sbgStatus,
    int32_t hvVoltage_mV,
    int32_t power_kW,
    uint32_t speed_kph,
    int32_t motorTemp_C,
    int32_t acTemp_C,
    int32_t mcTemp_C,
    int32_t glvVoltage_V,
    uint8_t glvSoC,
    uint8_t hvSoC,
    bool yrcOn,
    bool tcOn,
    bool ssOn,
    float odometer_km,
    bool drsOpen,
    cornerId_t hottest_motor);

void tftDL_racingScreenUpdate(
    int32_t motorTemp_C,
    int32_t acTemp_C,
    int32_t mcTemp_C,
    uint8_t hvSoC,
    bool drsOpen);

void tftDL_errorUpdate(
    tft_errors_t *err,
    volatile cmr_canHVCBMBErrors_t *BMBerr);

void tftDL_configUpdate();

void setTempColor(uint32_t background_index, uint32_t text_index, bool temp_yellow, bool temp_red);

void tftDLContentLoad(tft_t *tft, const tftDL_t *tftDL);
void tftDLWrite(tft_t *tft, const tftDL_t *tftDL);

#endif /* TFTDL_H */
