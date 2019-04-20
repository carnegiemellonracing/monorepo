/**
 * @file tftPrivate.h
 * @brief Private TFT interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef TFT_PRIVATE_H
#define TFT_PRIVATE_H

#include <stdbool.h>

#include <CMR/qspi.h>   // QuadSPI interface

/** @brief Expected chip ID. */
#define TFT_CHIP_ID 0x00011208

/** @brief Display reset time, in milliseconds. */
#define TFT_RESET_MS 50

/** @brief Display initialization time, in milliseconds. */
#define TFT_INIT_MS 400

/** @brief Display startup time, in milliseconds. */
#define TFT_STARTUP_MS 3000

/** @brief Flag for indicating a write to the display. */
#define TFT_WRITE_FLAG (1 << 23)

/** @brief Dummy cycles for reading data from the display. */
#define TFT_READ_DUMMY_CYCLES 8

/** @brief General purpose graphics RAM size, in bytes. */
#define TFT_RAM_G_SIZE (1024 * 1024)

/** @brief Display list RAM size, in bytes. */
#define TFT_RAM_DL_SIZE (8 * 1024)

/** @brief Coprocessor command buffer RAM size, in bytes. */
#define TFT_RAM_CMD_SIZE (4 * 1024)

/** @brief Represents a display command. */
typedef enum {
    TFT_CMD_ACTIVE = 0x00,  /**< @brief Enter "ACTIVE" mode (send twice). */
    TFT_CMD_CLKEXT = 0x44   /**< @brief Use external clock. */
} tftCmd_t;

/** @brief Represents a display address. */
typedef enum {
    // Diagnostics.
    TFT_ADDR_CHIP_ID = 0x0C0000,    /**< @brief Chip identifier. */

    // Video parameters.
    TFT_ADDR_HCYCLE = 0x30202C,     /**< @brief Horizontal cycle time. */
    TFT_ADDR_HOFFSET = 0x302030,    /**< @brief Horizontal offset time. */
    TFT_ADDR_HSYNC0 = 0x302038,     /**< @brief Horizontal sync time 0. */
    TFT_ADDR_HSYNC1 = 0x30203C,     /**< @brief Horizontal sync time 1. */
    TFT_ADDR_VCYCLE = 0x302040,     /**< @brief Vertical cycle time. */
    TFT_ADDR_VOFFSET = 0x302044,    /**< @brief Vertical offset time. */
    TFT_ADDR_VSYNC0 = 0x30204C,     /**< @brief Vertical sync time 0. */
    TFT_ADDR_VSYNC1 = 0x302050,     /**< @brief Vertical sync time 1. */
    TFT_ADDR_SWIZZLE = 0x302064,    /**< @brief RGB signal swizzle. */
    TFT_ADDR_CSPREAD = 0x302068,    /**< @brief Clock spreading enable. */
    TFT_ADDR_HSIZE = 0x302034,      /**< @brief Horizontal pixel count. */
    TFT_ADDR_VSIZE = 0x302048,      /**< @brief Vertical pixel count. */

    // GPIO.
    TFT_ADDR_GPIOX_DIR = 0x302098,  /**< @brief GPIO directions. */
    TFT_ADDR_GPIOX = 0x30209C,      /**< @brief GPIO values. */

    // Clock configuration.
    TFT_ADDR_PCLK_POL = 0x30206C,   /**< @brief PCLK polarity. */
    TFT_ADDR_PCLK = 0x302070,       /**< @brief PCLK frequency divider. */

    // Coprocessor registers.
    TFT_ADDR_CMD_READ = 0x3020F8,   /**< @brief Coprocessor read pointer. */
    TFT_ADDR_CMD_WRITE = 0x3020FC,  /**< @brief Coprocessor write pointer. */
    TFT_ADDR_CMD_DL = 0x302100,     /**< @brief Coprocessor DL RAM offset. */

    // RAM areas.
    TFT_ADDR_RAM_G = 0x000000,      /**< @brief General purpose graphics RAM. */
    TFT_ADDR_RAM_DL = 0x300000,     /**< @brief Display list RAM. */
    TFT_ADDR_RAM_CMD = 0x308000     /**< @brief Coprocessor command buffer. */
} tftAddr_t;

/** @brief Represents a TFT display.  */
typedef struct {
    cmr_qspi_t qspi;        /**< @brief The display's QuadSPI port. */
    uint16_t coCmdRd;       /**< @brief Coprocessor command read address. */
    uint16_t coCmdWr;       /**< @brief Coprocessor command write address. */
} tft_t;

void tftCmd(tft_t *tft, tftCmd_t cmd, uint8_t param);
void tftWrite(tft_t *tft, tftAddr_t addr, size_t len, const void *data);
void tftRead(tft_t *tft, tftAddr_t addr, size_t len, void *data);

void tftCoCmd(tft_t *tft, size_t len, const void *data, bool wait);

#endif /* TFT_PRIVATE_H */

