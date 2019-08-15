/**
 * @file segments.c
 * @brief Segment display implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdint.h>
#include <stdio.h>      // snprintf()
#include <FreeRTOS.h>   // configASSERT()

#include <CMR/i2c.h>    // I2C interface.
#include <CMR/tasks.h>  // Task interface.

#include "segments.h"   // Interface to implement.
#include "state.h"      // State interface
#include "can.h"        // Board-specific CAN interface

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

/** @brief I2C port on which to command segments. */
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
 * @brief Write text on the segment display.
 *
 * @param data Array of data to write.
 * @param len Length of data array
 */
static void segmentsWrite(char *data, size_t len) {
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

/** @brief Segment display update priority. */
static const uint32_t segmentsUpdate_priority = 3;

/** @brief Segment display update period (milliseconds). */
static const TickType_t segmentsUpdate_period_ms = 100;

/** @brief Segment display update task. */
static cmr_task_t segmentsUpdate_task;

/**
 * @brief Task for updating the segment display.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void segmentsUpdate(void *pvParameters) {
    /** @brief Segment text formats for each state. */
    static const char *stateFmt[] = {
        [CMR_CAN_UNKNOWN] = "????????",
        [CMR_CAN_GLV_ON] = "GLV ON  ",
        [CMR_CAN_HV_EN] = "HV %c %.3u",
        [CMR_CAN_RTD] = "RD %c %.3u",
        [CMR_CAN_ERROR] = "ERROR   ",
        [CMR_CAN_CLEAR_ERROR] = "C_ERROR ",
    };

    /** @brief Characters for each gear. */
    static const char gearChars[CMR_CAN_GEAR_LEN] = {
        [CMR_CAN_GEAR_UNKNOWN] = '-',
        [CMR_CAN_GEAR_REVERSE] = 'R',
        [CMR_CAN_GEAR_SLOW] = 'S',
        [CMR_CAN_GEAR_FAST] = 'F',
        [CMR_CAN_GEAR_ENDURANCE] = 'E',
        [CMR_CAN_GEAR_AUTOX] = 'X',
        [CMR_CAN_GEAR_SKIDPAD] = '8',
        [CMR_CAN_GEAR_ACCEL] = 'A',
        [CMR_CAN_GEAR_TEST] = 'T'
    };

    (void) pvParameters;

    cmr_canRXMeta_t *metaHVCPackVoltage = canRXMeta + CANRX_HVC_PACK_VOLTAGE;
    volatile cmr_canHVCPackVoltage_t *canHVCPackVoltage =
        (void *) metaHVCPackVoltage->payload;

    for (
        TickType_t lastWakeTime = xTaskGetTickCount();
        1;
        vTaskDelayUntil(&lastWakeTime, segmentsUpdate_period_ms)
    ) {
        cmr_canState_t stateVSM = stateGetVSM();
        cmr_canState_t stateVSMReq = stateGetVSMReq();
        cmr_canGear_t gear = stateGetGear();

        uint8_t gearChar = (gear < CMR_CAN_GEAR_LEN)
            ? gearChars[gear]
            : gearChars[CMR_CAN_GEAR_UNKNOWN];

        const char *dispFmt = stateFmt[0];
        switch (stateVSM) {
            case CMR_CAN_UNKNOWN:
            case CMR_CAN_GLV_ON:
            case CMR_CAN_HV_EN:
            case CMR_CAN_RTD:
            case CMR_CAN_ERROR:
            case CMR_CAN_CLEAR_ERROR:
                dispFmt = stateFmt[stateVSM];
                break;
            default:
                break;
        }

        // Need space for NUL-terminator from snprintf().
        char disp[SEGMENTS_LEN + 1];
        snprintf(
            disp, sizeof(disp), dispFmt,
            gearChar, canHVCPackVoltage->hvVoltage
        );

        // Indicate state request (using "decimal points"), if any.
        if (stateVSMReq < stateVSM) {
            for (size_t i = 0; i < SEGMENTS_LEN / 2; i++) {
                disp[i] |= '\x80';
            }
        } else if (stateVSMReq > stateVSM) {
            for (size_t i = SEGMENTS_LEN / 2; i < SEGMENTS_LEN; i++) {
                disp[i] |= '\x80';
            }
        }

        segmentsWrite(disp, SEGMENTS_LEN);
    }
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

    cmr_taskInit(
        &segmentsUpdate_task, "segmentsUpdate",
        segmentsUpdate_priority, segmentsUpdate, NULL
    );
}

