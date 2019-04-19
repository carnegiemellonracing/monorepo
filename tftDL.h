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

typedef struct {
    size_t len;             /**< @brief Length of the display list, in bytes. */
    const uint32_t *data;   /**< @brief The display list data. */
} tftDL_t;

extern const tftDL_t tftDLStartup;
extern const tftDL_t tftDLRTD;

#endif /* TFTDL_H */

