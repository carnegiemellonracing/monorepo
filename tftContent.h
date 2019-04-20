/**
 * @file tftContent.h
 * @brief TFT content interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stddef.h>     // size_t
#include <stdint.h>     // uint8_t

#include "tftPrivate.h"     // Private interface

typedef struct tftContent tftContent_t;

extern const tftContent_t tftContent_startup_lut;
extern const tftContent_t tftContent_startup;

extern const tftContent_t tftContent_RobotoMono_Bold_72_L4;
extern const tftContent_t tftContent_RobotoMono_Bold_40_L4;

void tftContentLoad(tft_t *tft, const tftContent_t *tftContent);

