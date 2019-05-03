/**
 * @file tftDL.h
 * @brief TFT display list interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef TFTDL_H
#define TFTDL_H

#include <stddef.h>     // size_t
#include <stdint.h>     // uint32_t

#include "tftPrivate.h"     // Private interface
#include "tft.h"            // tft interface

/** @brief Represents a display list. */
typedef struct tftDL tftDL_t;

extern const tftDL_t tftDL_startup;

extern const tftDL_t tftDL_RTD;

extern const tftDL_t tftDL_error;

void tftDL_RTDUpdate(
    uint32_t speed_mph,
    int32_t hvVoltage,
    int32_t power_kW,
    int32_t dcdcTemp,
    int32_t motorTemp,
    int32_t acTemp,
    int32_t mcTemp
);

void tftDL_errorUpdate(
    tft_errors_t *err
);

void tftDLContentLoad(tft_t *tft, const tftDL_t *tftDL);
void tftDLWrite(tft_t *tft, const tftDL_t *tftDL);

#endif /* TFTDL_H */

