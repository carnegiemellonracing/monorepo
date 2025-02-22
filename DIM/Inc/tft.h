/**
 * @file tft.h
 * @brief TFT display interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef TFT_H
#define TFT_H

#include <stdbool.h>
#include <stdint.h>
#include <CMR/qspi.h>  // QuadSPI interface
#include <stdbool.h>
/** @brief setting the thresholds for the backgrounds to turn yellow and red*/
#define AC_YELLOW_THRESHOLD 55
#define AC_RED_THRESHOLD 57
#define MOTOR_YELLOW_THRESHOLD 100
#define MOTOR_RED_THRESHOLD 115
#define MC_YELLOW_THRESHOLD 48
#define MC_RED_THRESHOLD 58
/** @brief Display startup time, in milliseconds. */
#define TFT_STARTUP_MS 3000

/** @brief Display reset time, in milliseconds. */
#define TFT_RESET_MS 50


/** @brief Display update period. */
#define TFT_UPDATE_PERIOD_MS 20

void tftUpdate(void *pvParameters);
void tftInitSequence();
void tftInit(void);
void drawRacingScreen(void);
void drawConfigScreen(void);
void drawSafetyScreen(void);
void drawErrorScreen(void);
void drawRTDScreen(void);
/** @brief All of the errors to be drawn on-screen
 * in error state. */
typedef struct {
    bool cdcTimeout;         /**< @brief Has the CDC timed out? */
    bool ptcTimeout;         /**< @brief Has the PTC timed out? */
    bool vsmTimeout;         /**< @brief Has the VSM timed out? */
    bool hvcTimeout;         /**< @brief Has the HVC timed out? */
    bool imdError;           /**< @brief Has the IMD faulted? */
    bool amsError;           /**< @brief Has the AMS faulted? */
    bool bspdError;          /**< @brief Has the BSPD tripped? */
    bool overVolt;           /**< @brief Has the HVC errored w/ OV? */
    bool underVolt;          /**< @brief Has the HVC errored w/ UV? */
    bool glvLowVolt;         /**< @brief Has the GLV errored w/ UV? */
    bool hvcoverTemp;        /**< @brief Has the HVC errored w/ over temp? */
    bool hvcBMBTimeout;      /**< @brief Has a BMB timed out? */
    bool hvcBMBFault;        /**< @brief Has a BMB raised a fault? */
    uint16_t hvcErrorNum;    /**< @brief Give the error number,
                              * So the driver can relay even if the error is not accounted for above. */
    uint16_t amkFLErrorCode; /**< @brief AMK FL Error Code */
    uint16_t amkFRErrorCode; /**< @brief AMK FR Error Code */
    uint16_t amkBLErrorCode; /**< @brief AMK BL Error Code */
    uint16_t amkBRErrorCode; /**< @brief AMK BR Error Code */
    uint32_t glvVoltage_V;   /**< @brief GLV Voltage in Volts */
} tft_errors_t;

typedef enum {
    SBG_STATUS_NOT_CONNECTED = 0,    /** @brief INS not connected/not sending info */
    SBG_STATUS_WORKING_NO_POS_FOUND, /** @brief INS working but doesn't have fix on position */
    SBG_STATUS_WORKING_POS_FOUND     /** @brief INS working and has fix on position */
} SBG_status_t;

typedef enum {
    MEMORATOR_NOT_CONNECTED = 0,   /** @brief Memorator not connected/not sending info */
    MEMORATOR_CONNECTED_BAD_STATE, /** @brief Memorator transmitting, but not sending correctly */
    MEMORATOR_CONNECTED_STATE_OK   /** @brief Memorator transmitting correctly */
} memorator_status_t;

typedef enum {
    FL = 0,
    FR,
    RL,
    RR,
    NUM_CORNERS,
    NONE
} cornerId_t;

/** @brief General purpose graphics RAM size, in bytes. */
#define TFT_RAM_G_SIZE (1024 * 1024)

/** @brief Display list RAM size, in bytes. */
#define TFT_RAM_DL_SIZE (8 * 1024)

/** @brief Coprocessor command buffer RAM size, in bytes. */
#define TFT_RAM_CMD_SIZE (4 * 1024)

/** @brief Represents a display command. */
typedef enum {
    TFT_CMD_ACTIVE = 0x00, /**< @brief Enter "ACTIVE" mode (send twice). */
    TFT_CMD_CLKEXT = 0x44,  /**< @brief Use external clock. */
    TFT_CMD_INFLATE =  0xFFFFFF22
} tftCmd_t;

/** @brief Represents a display address. */
typedef enum {
    // Diagnostics.
    TFT_ADDR_CHIP_ID = 0x0C'0000, /**< @brief Chip identifier. */

    // Video parameters.
    TFT_ADDR_HCYCLE = 0x30202C,  /**< @brief Horizontal cycle time. */
    TFT_ADDR_HOFFSET = 0x302030, /**< @brief Horizontal offset time. */
    TFT_ADDR_HSYNC0 = 0x302038,  /**< @brief Horizontal sync time 0. */
    TFT_ADDR_HSYNC1 = 0x30203C,  /**< @brief Horizontal sync time 1. */
    TFT_ADDR_VCYCLE = 0x302040,  /**< @brief Vertical cycle time. */
    TFT_ADDR_VOFFSET = 0x302044, /**< @brief Vertical offset time. */
    TFT_ADDR_VSYNC0 = 0x30204C,  /**< @brief Vertical sync time 0. */
    TFT_ADDR_VSYNC1 = 0x302050,  /**< @brief Vertical sync time 1. */
    TFT_ADDR_SWIZZLE = 0x302064, /**< @brief RGB signal swizzle. */
    TFT_ADDR_DITHER = 0x302060,  /**< @brief RGB signal dithering. */
    TFT_ADDR_CSPREAD = 0x302068, /**< @brief Clock spreading enable. */
    TFT_ADDR_HSIZE = 0x302034,   /**< @brief Horizontal pixel count. */
    TFT_ADDR_VSIZE = 0x302048,   /**< @brief Vertical pixel count. */

    // GPIO.
    TFT_ADDR_GPIOX_DIR = 0x302098, /**< @brief GPIO directions. */
    TFT_ADDR_GPIOX = 0x30209C,     /**< @brief GPIO values. */

    /**
     * @brief SPI width.
     *
     * `0` = 1-bit; `1` = 2-bit; `2` = 4-bit.
     */
    TFT_ADDR_SPI_WIDTH = 0x302188,

    // Clock configuration.
    TFT_ADDR_PCLK_POL = 0x30'206C, /**< @brief PCLK polarity. */
    TFT_ADDR_PCLK = 0x30'2070,     /**< @brief PCLK frequency divider. */

    // Coprocessor registers.
    TFT_ADDR_CMD_READ = 0x30'20F8,  /**< @brief Coprocessor read pointer. */
    TFT_ADDR_CMD_WRITE = 0x30'20FC, /**< @brief Coprocessor write pointer. */
    TFT_ADDR_CMD_DL = 0x30'2100,    /**< @brief Coprocessor DL RAM offset. */
    TFT_ADDR_CMDB_SPACE = 0x302574,
    TFT_ADDR_CMDB_WRITE = 0x302578,

    // RAM areas.
    TFT_ADDR_RAM_G = 0x00'0000,  /**< @brief General purpose graphics RAM. */
    TFT_ADDR_RAM_DL = 0x30'0000, /**< @brief Display list RAM. */
    TFT_ADDR_RAM_REG = 0x30'2000, /**< @brief Registers */
    TFT_ADDR_RAM_CMD = 0x30'8000, /**< @brief Coprocessor command buffer. */

} tftAddr_t;

/** @brief Represents a TFT display.  */
typedef struct {
    cmr_qspi_t qspi;  /**< @brief The display's QuadSPI port. */
    bool inited;      /**< @brief `true` if the init sequence is done. */
    uint16_t coCmdRd; /**< @brief Coprocessor command read address. */
    uint16_t coCmdWr; /**< @brief Coprocessor command write address. */
} tft_t;

extern tft_t tft;

void tftCmd(tft_t *tft, tftCmd_t cmd, uint8_t param);
void tftWrite(tft_t *tft, tftAddr_t addr, size_t len, const void *data);
void tftRead(tft_t *tft, tftAddr_t addr, size_t len, void *data);

void tftCoCmd(tft_t *tft, size_t len, const void *data, bool wait);

/** @brief Forward declare exported content type. */
typedef struct tftContent tftContent_t;

extern const tftContent_t tftContent_startup_lut;
extern const tftContent_t tftContent_startup;

void tftContentLoad(tft_t *tft, const tftContent_t *tftContent);

#endif /* TFT_H */
