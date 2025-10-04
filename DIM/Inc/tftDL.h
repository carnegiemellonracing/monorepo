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
//#include "tftPrivate.h"  // Private interface

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

void tftDL_RTDUpdate(
    memorator_status_t memoratorStatus,
    uint8_t movellaStatus,
    int32_t hvVoltage_mV,
    int32_t power_kW,
    uint32_t speed_kph,
    bool motorTemp_yellow,
    bool motorTemp_red,
    bool acTemp_yellow,
    bool acTemp_red,
    bool mcTemp_yellow,
    bool mcTemp_red,
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