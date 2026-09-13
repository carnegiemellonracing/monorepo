/**
 * @file mlx90640.c
 * @brief MLX90640 IR thermal array implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdint.h>     // fixed-width integers

#include <CMR/tasks.h>  // Task interface

#include <MLX90640_API.h>   // Melexis MLX90640 driver library
#include "mlx90640.h"       // Interface to implement

/** @brief 7-bit I2C address of the sensor. */
#define MLX_I2C_ADDR    0x33

/**
 * @brief Refresh rate control field (0b101 = 16 Hz sub-page rate).
 *
 * A full 24x32 frame is two sub-pages, so this yields an 8 Hz full-frame rate,
 * roughly matching the 10 Hz CAN transmit cadence. 16 Hz is comfortably within
 * the 400 kHz I2C budget; higher rates (32/64 Hz) would need ~1 MHz I2C.
 */
#define MLX_REFRESH     0x05

/**
 * @brief ADC resolution control field (0b00 = 16-bit).
 *
 * Values 0..3 select 16/17/18/19-bit ADC resolution respectively. 16-bit
 * keeps the per-pixel conversion time short, which is what allows the 16 Hz
 * sub-page rate (see MLX_REFRESH) to fit within the I2C budget.
 */
#define MLX_RESOLUTION  0x00

/** @brief Assumed scene emissivity. */
#define MLX_EMISSIVITY  0.95f

/**
 * @brief Offset (degrees C) subtracted from ambient to estimate reflected
 *        temperature when no separate reflected-temperature sensor is present.
 */
#define MLX_TA_SHIFT    8.0f

/** @brief Bad-pixel correction mode (1 = chess, matching MLX_REFRESH setup). */
#define MLX_MODE_CHESS  1

/** @brief Row TX task priority. */
static const uint32_t mlxTask_priority = 2;

/**
 * @brief Row TX task loop period (milliseconds).
 *
 * Guarantees the task yields each iteration even if a frame read fails
 * immediately (e.g. I2C error), preventing a busy-loop that would starve
 * lower-priority tasks.
 */
static const TickType_t mlxTask_period_ms = 10;

/** @brief Row TX task. */
static cmr_task_t mlxTask;

/** @brief Restored calibration parameters. */
static paramsMLX90640 params;

/** @brief EEPROM contents. */
static uint16_t eeData[MLX90640_EEPROM_DUMP_NUM];

/** @brief Raw frame data (768 pixels + 64 aux + 2 status words). */
static uint16_t frameData[834];

/** @brief Latest computed frame temperatures (degrees C). */
static float frameTemps[MLX90640_PIXEL_NUM];

/**
 * @brief Reads both sub-pages of a frame into `frameTemps`.
 */
static void mlxGetFrame(void) {
    for (int i = 0; i < 2; i++) {
        if (MLX90640_GetFrameData(MLX_I2C_ADDR, frameData) < 0) {
            return;
        }
        float tr = MLX90640_GetTa(frameData, &params) - MLX_TA_SHIFT;
        MLX90640_CalculateTo(frameData, &params, MLX_EMISSIVITY, tr, frameTemps);
    }

    MLX90640_BadPixelsCorrection(params.brokenPixels, frameTemps, MLX_MODE_CHESS, &params);
    MLX90640_BadPixelsCorrection(params.outlierPixels, frameTemps, MLX_MODE_CHESS, &params);
}

/**
 * @brief Copies image row `row` into `out` as deci-degrees C (MLX_COLS values).
 */
void mlx90640GetRow(uint8_t row, int16_t *out) {
    int base = row * MLX_COLS;
    for (int col = 0; col < MLX_COLS; col++) {
        out[col] = (int16_t)(frameTemps[base + col] * 10.0f);
    }
}

/**
 * @brief Task for updating frame temperatures.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void mlx90640Task(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        mlxGetFrame();

        vTaskDelayUntil(&lastWakeTime, mlxTask_period_ms);
    }
}

/**
 * @brief Initializes the MLX90640 interface.
 */
void mlx90640Init(void) {
    MLX90640_SetRefreshRate(MLX_I2C_ADDR, MLX_REFRESH);
    MLX90640_SetResolution(MLX_I2C_ADDR, MLX_RESOLUTION);
    MLX90640_SetChessMode(MLX_I2C_ADDR);
    MLX90640_DumpEE(MLX_I2C_ADDR, eeData);
    MLX90640_ExtractParameters(eeData, &params);

    cmr_taskInit(
        &mlxTask,
        "MLX90640",
        mlxTask_priority,
        mlx90640Task,
        NULL
    );
}
