/**
 * @file tftContent.h
 * @brief TFT content interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stddef.h>  // size_t
#include <stdint.h>  // uint8_t

#include "tftPrivate.h"  // Private interface

/** @brief Forward declare exported content type. */
typedef struct tftContent tftContent_t;

extern const tftContent_t tftContent_startup_lut;
extern const tftContent_t tftContent_startup;

void tftContentLoad(tft_t *tft, const tftContent_t *tftContent);
