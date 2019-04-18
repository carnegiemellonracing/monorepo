/**
 * @file segments.c
 * @brief Segment display implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdint.h>
#include <FreeRTOS.h>   // configASSERT()

#include <CMR/i2c.h>    // I2C interface.

#include "segments.h"   // Interface to implement.

/** @brief Number of segments in the display. */
#define SEGMENTS_LEN 8

/** @brief Segment display I2C timeout in milliseconds. */
#define SEGMENTS_I2C_TIMEOUT_MS 100

/** @brief Segment display I2C clock speed. */
#define SEGMENTS_I2C_CLOCK 100000

/** @brief Segment display I2C address. */
#define SEGMENTS_I2C_ADDR 0x60

/** @brief Commands for interacting with the segment display. */
typedef enum {
    /** @brief Maximum brightness register. */
    SEGMENTS_CMD_MAX_BRIGHTNESS = 0x02,
    /** @brief Configuration register. */
    SEGMENTS_CMD_CONFIGURATION = 0x04,
    /** @brief Display test register. */
    SEGMENTS_CMD_DISPLAY_TEST = 0x07,
    /** @brief Segment configuration register. */
    SEGMENTS_CMD_SEGMENT_CONFIG = 0x0C,
    /** @brief Segment data base. */
    SEGMENTS_CMD_SEGMENT_DATA_BASE = 0x20
} segments_cmd_t;

/** @brief Represents a segment display. */
typedef struct {
    cmr_i2c_t i2c;  /**< @brief I2C port. */
} segments_t;

static segments_t segments;

/**
 * @brief Sends the segments a command.
 *
 * @param cmd Command.
 * @param value Command value.
 */
static void segmentsCmd(uint8_t cmd, uint8_t value){
    uint8_t data[] = { cmd, value };
    cmr_i2cTX(
        &segments.i2c, SEGMENTS_I2C_ADDR,
        data, sizeof(data),
        SEGMENTS_I2C_TIMEOUT_MS
    );
}

/**
 * @brief Returns the segment's I2C address based on its index.
 *
 * @param segNum The segment's index.
 *
 * @return The segment's I2C address.
 */
static uint8_t segmentsAddrForSeg(size_t segNum) {
    configASSERT(segNum <= SEGMENTS_LEN);
    return segNum + SEGMENTS_CMD_SEGMENT_DATA_BASE;
}

/**
 * @brief Initializes the segment display.
 */
void segmentsInit(void) {
    cmr_i2cInit(
        &segments.i2c, I2C1,
        100000, 0,
        GPIOB, GPIO_PIN_8,
        GPIOB, GPIO_PIN_7
    );

    segmentsCmd(SEGMENTS_CMD_DISPLAY_TEST, 0x0);
    segmentsCmd(SEGMENTS_CMD_CONFIGURATION, 0x01);
    segmentsCmd(SEGMENTS_CMD_SEGMENT_CONFIG, 0xFF);
    segmentsCmd(SEGMENTS_CMD_MAX_BRIGHTNESS, 0xF);
}

/**
 * @brief Write text on the segment display.
 *
 * @param data Array of data to write.
 * @param len Length of data array
 */
void segmentsWrite(char *data, size_t len) {
    if (len > SEGMENTS_LEN) {
        len = SEGMENTS_LEN;     // Data too long; truncate.
    }

    // Right-align text.
    for (size_t i = 0; i < len; i++) {
        uint8_t addr = segmentsAddrForSeg(SEGMENTS_LEN - len + i);
        segmentsCmd(addr, data[i]);
    }
    for (size_t i = 0; i < SEGMENTS_LEN - len; i++) {
        uint8_t addr = segmentsAddrForSeg(i);
        segmentsCmd(addr, ' ');
    }
}

