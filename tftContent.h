/**
 * @file tftContent.h
 * @brief TFT content interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stddef.h>     // size_t
#include <stdint.h>     // uint8_t

typedef struct {
    size_t len;             /**< @brief The content's length, in bytes. */
    size_t addr;            /**< @brief The content's address in `RAM_G`. */
    const uint8_t *data;    /**< @brief The content. */
} tftContent_t;

extern const tftContent_t tftContent_RobotoMono_Bold_72_L4;
extern const tftContent_t tftContent_RobotoMono_Bold_40_L4;

