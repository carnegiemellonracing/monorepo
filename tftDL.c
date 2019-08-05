/**
 * @file tftDL.c
 * @brief TFT display list implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdio.h>      // snprintf()

#include "tftDL.h"          // Interface to implement
#include "tftContent.h"     // Content interface

/** @brief Represents a display list. */
struct tftDL {
    size_t len;             /**< @brief Length of the display list, in bytes. */
    const uint32_t *data;   /**< @brief The display list data. */

    size_t contentLen;              /**< @brief Number of content items. */
    const tftContent_t **content;   /**< @brief Associated content. */
};

/** @brief Raw startup screen */
static uint32_t tftDL_startupData[] = {
#include "ESE/startup.rawh"
};

/** @brief Packets to send to the DL on startup.
 * See datasheet */
static const tftContent_t *tftDL_startupContent[] = {
    &tftContent_startup_lut,
    &tftContent_startup
};

/** @brief Complete data required to draw the startup screen.
 * Exposed to interface consumers. */
const tftDL_t tftDL_startup = {
    .len = sizeof(tftDL_startupData),
    .data = tftDL_startupData,

    .contentLen = sizeof(tftDL_startupContent) / sizeof(tftDL_startupContent[0]),
    .content = tftDL_startupContent
};

/** @brief GLV Screen */
static uint32_t tftDL_errorData[] = {
#include "ESE/errors.rawh"
};

/** @brief Packets to send to the DL on error.
 * See datasheet */
static const tftContent_t *tftDL_errorContent[] = {
    &tftContent_RobotoMono_Bold_72_L4,
    &tftContent_RobotoMono_Bold_40_L4,
};

/** @brief Complete data required to draw the error screen.
 * Exposed to interface consumers. */
const tftDL_t tftDL_error = {
    .len = sizeof(tftDL_errorData),
    .data = tftDL_errorData,

    .contentLen = sizeof(tftDL_errorContent) / sizeof(tftDL_errorContent[0]),
    .content = tftDL_errorContent
};

/** @brief RTD Screen */
static uint32_t tftDL_RTDData[] = {
#include "ESE/RTD.rawh"
};

/** @brief Complete data required to draw the
 * ready-to-drive screen.
 * Exposed to interface consumers. */
static const tftContent_t *tftDL_RTDContent[] = {
    &tftContent_RobotoMono_Bold_72_L4,
    &tftContent_RobotoMono_Bold_40_L4,
};

/** @brief Complete data required to draw the
 * Ready-to-drive screen.
 * Exposed to interface consumers. */
const tftDL_t tftDL_RTD = {
    .len = sizeof(tftDL_RTDData),
    .data = tftDL_RTDData,

    .contentLen = sizeof(tftDL_RTDContent) / sizeof(tftDL_RTDContent[0]),
    .content = tftDL_RTDContent
};

/** @brief How to draw a single bar dynamically. */
typedef struct {
    uint32_t *addr;  /**< @brief Top-left vertex addr */
    uint8_t topY;    /**< @brief Top edge Y coord. */
    uint8_t botY;    /**< @brief Bot edge Y coord. */
    int32_t maxVal;  /**< @brief Logical value
    * corresponding to bottom edge */
    int32_t minVal;  /**< @brief Logical value
    * corresponding to top edge */
} tftDL_bar_t;

/** @brief Bitposition of Y-coordinate byte in vertices */
#define TFT_DL_VERTEX_Y_BIT 12

/**
 * @brief Reflect logical value into bar plot for drawing.
 *
 * @param bar The bar to update.
 * @param val The logical value to draw.
 */
static void tftDL_barSetY(const tftDL_bar_t *bar, int32_t val) {
    uint8_t y;
    if (val < bar->minVal) {
        y = bar->botY;
    } else if (val > bar->maxVal) {
        y = bar->topY;
    } else {
        int32_t len = (
            (val - bar->minVal) * (int32_t) (bar->botY - bar->topY)
        ) / (bar->maxVal - bar->minVal);
        y = bar->botY - (uint8_t) len;
    }

    uint32_t vertex = *bar->addr;
    vertex &= ~(((1 << 8) - 1) << TFT_DL_VERTEX_Y_BIT); //TODO: This is dumb
    vertex |= y << TFT_DL_VERTEX_Y_BIT;
    *bar->addr = vertex;
}

/**
 * @brief Updates the ready-to-drive screen.
 *
 * @param speed_mph Speed (from CDC)
 * @param hvVoltage_mV Pack Voltage (from HVC)
 * @param power_kW Electrical power dissipation
 * (inferred from CDC)
 * @param dcdcTemp_C DCDC Thermistor temp.
 * Unused.
 * @param motorTemp_C Motor termperature.
 * Referred from RMS via CDC. Deg. C.
 * @param acTemp_C AC temp (from HVC).
 * Currently Max cell temp. Deg. C.
 * @param mcTemp_C MC internal temp.
 * Referred from RMS via CDC. Deg. C.
 */
void tftDL_RTDUpdate(
    uint32_t speed_mph,
    int32_t hvVoltage_mV,
    int32_t power_kW,
    int32_t dcdcTemp_C,
    int32_t motorTemp_C,
    int32_t acTemp_C,
    int32_t mcTemp_C
) {
    static struct {
        char buf[3];
    } *const speed_mph_str = (void *) (tftDL_RTDData + 72);

    static struct {
        char buf[4];
    } *const hvVoltage_mV_str = (void *) (tftDL_RTDData + 91);

    static struct {
        char buf[3];
    } *const dcdcTemp_C_str = (void *) (tftDL_RTDData + 125);

    static struct {
        char buf[3];
    } *const motorTemp_C_str = (void *) (tftDL_RTDData + 130);

    static struct {
        char buf[3];
    } *const acTemp_C_str = (void *) (tftDL_RTDData + 120);

    static struct {
        char buf[3];
    } *const mcTemp_C_str = (void *) (tftDL_RTDData + 135);

    static const tftDL_bar_t hvVoltage_mV_bar = {
        .addr = tftDL_RTDData + 85,
        .topY = 12,
        .botY = 168,
        .maxVal = 400000,
        .minVal = 300000
    };

    static struct {
        char buf[3];
    } *const power_kW_str = (void *) (tftDL_RTDData + 115);

    static const tftDL_bar_t power_kW_bar = {
        .addr = tftDL_RTDData + 109,
        .topY = 12,
        .botY = 168,
        .maxVal = 85,
        .minVal = 0
    };

    /* Voltage Bar */
    snprintf(
            hvVoltage_mV_str->buf, sizeof(hvVoltage_mV_str->buf),
            "%3ld", hvVoltage_mV / 1000
    );
    tftDL_barSetY(&hvVoltage_mV_bar, hvVoltage_mV);

    /* Power Bar */
    snprintf(
            power_kW_str->buf, sizeof(power_kW_str->buf),
            "%2ld", power_kW
    );

    /* Speed */
    snprintf(
            speed_mph_str->buf, sizeof(speed_mph_str->buf),
            "%2lu", speed_mph
    );
    tftDL_barSetY(&power_kW_bar, power_kW);

    /* Temperatures */
    snprintf(
        dcdcTemp_C_str->buf, sizeof(dcdcTemp_C_str->buf),
        "%2ld", dcdcTemp_C
    );

    snprintf(
        motorTemp_C_str->buf, sizeof(motorTemp_C_str->buf),
        "%2ld", motorTemp_C
    );

    snprintf(
        acTemp_C_str->buf, sizeof(acTemp_C_str->buf),
        "%2ld", acTemp_C
    );

    snprintf(
        mcTemp_C_str->buf, sizeof(mcTemp_C_str->buf),
        "%2ld", mcTemp_C
    );
}

/**
 * @brief Set the color pointed to by addr.
 *
 * @param addr A raw pointer to a DL color.
 * @param val A raw value.
 */
static void tftDL_setColor(uint32_t *addr, uint32_t val) {
    *addr = val; // TODO: This is dumb
}

/**
 * @brief Updates the ready-to-drive screen to the error screen.
 *
 * @param err Error statuses to display.
 */
void tftDL_errorUpdate(
    tft_errors_t *err
) {
    uint32_t color_err = 0x04ff0a0a;
    uint32_t color_none = 0x04303030;

    uint32_t *fsm_color = (void *) (tftDL_errorData + 18);
    uint32_t fsm_color_cmd  = (err->fsmTimeout) ? color_err : color_none;

    uint32_t *cdc_color = (void *) (tftDL_errorData + 25);
    uint32_t cdc_color_cmd  = (err->cdcTimeout) ? color_err : color_none;

    uint32_t *ptc_color = (void *) (tftDL_errorData + 32);
    uint32_t ptc_color_cmd  = (err->ptcTimeout) ? color_err : color_none;

    uint32_t *vsm_color = (void *) (tftDL_errorData + 39);
    uint32_t vsm_color_cmd  = (err->vsmTimeout) ? color_err : color_none;

    uint32_t *afc1_color = (void *) (tftDL_errorData + 46);
    uint32_t afc1_color_cmd  = (err->afc1Timeout) ? color_err : color_none;

    uint32_t *afc2_color = (void *) (tftDL_errorData + 54);
    uint32_t afc2_color_cmd  = (err->afc2Timeout) ? color_err : color_none;

    uint32_t *overVolt_color = (void *) (tftDL_errorData + 72);
    uint32_t overVolt_color_cmd  = (err->overVolt) ? color_err : color_none;

    uint32_t *underVolt_color = (void *) (tftDL_errorData + 80);
    uint32_t underVolt_color_cmd  = (err->underVolt) ? color_err : color_none;

    uint32_t *hvcoverTemp_color = (void *) (tftDL_errorData + 88);
    uint32_t hvcoverTemp_color_cmd  = (err->hvcoverTemp) ? color_err : color_none;

    uint32_t *hvcError_color = (void *) (tftDL_errorData + 95);
    uint32_t hvcError_color_cmd  = (err->hvc_Error) ? color_err : color_none;

    uint32_t *overSpeed_color = (void *) (tftDL_errorData + 115);
    uint32_t overSpeed_color_cmd  = (err->overSpeed) ? color_err : color_none;

    uint32_t *mcoverTemp_color = (void *) (tftDL_errorData + 122);
    uint32_t mcoverTemp_color_cmd  = (err->mcoverTemp) ? color_err : color_none;

    uint32_t *overCurrent_color = (void *) (tftDL_errorData + 129);
    uint32_t overCurrent_color_cmd  = (err->overCurrent) ? color_err : color_none;

    uint32_t *mcError_color = (void *) (tftDL_errorData + 136);
    uint32_t mcError_color_cmd  = (err->mcError) ? color_err : color_none;

    uint32_t *imdError_color = (void *) (tftDL_errorData + 159);
    uint32_t imdError_color_cmd  = (err->imdError) ? color_err : color_none;

    uint32_t *amsError_color = (void *) (tftDL_errorData + 154);
    uint32_t amsError_color_cmd  = (err->amsError) ? color_err : color_none;

    uint32_t *bspdError_color = (void *) (tftDL_errorData + 148);
    uint32_t bspdError_color_cmd  = (err->bspdError) ? color_err : color_none;

    static struct {
        char buf[11];
    } *const hvc_error_num_str = (void *) (tftDL_errorData + 104);

    snprintf(
        hvc_error_num_str->buf, sizeof(hvc_error_num_str->buf),
        "%04x", err->hvcErrorNum
    );

    static struct {
        char buf[15];
    } *const mc_error_num_str = (void *) (tftDL_errorData + 145);

    snprintf(
        mc_error_num_str->buf, sizeof(mc_error_num_str->buf),
        "%08x", err->mcErrorNum
    );

    tftDL_setColor(fsm_color, fsm_color_cmd);
    tftDL_setColor(cdc_color, cdc_color_cmd);
    tftDL_setColor(ptc_color, ptc_color_cmd);
    tftDL_setColor(vsm_color, vsm_color_cmd);
    tftDL_setColor(afc1_color, afc1_color_cmd);
    tftDL_setColor(afc2_color, afc2_color_cmd);
    tftDL_setColor(overVolt_color, overVolt_color_cmd);
    tftDL_setColor(underVolt_color, underVolt_color_cmd);
    tftDL_setColor(hvcoverTemp_color, hvcoverTemp_color_cmd);
    tftDL_setColor(hvcError_color, hvcError_color_cmd);
    tftDL_setColor(overSpeed_color, overSpeed_color_cmd);
    tftDL_setColor(mcoverTemp_color, mcoverTemp_color_cmd);
    tftDL_setColor(overCurrent_color, overCurrent_color_cmd);
    tftDL_setColor(mcError_color, mcError_color_cmd);
    tftDL_setColor(imdError_color, imdError_color_cmd);
    tftDL_setColor(amsError_color, amsError_color_cmd);
    tftDL_setColor(bspdError_color, bspdError_color_cmd);
}

/**
 * @brief Loads the display list's content into graphics memory.
 *
 * @param tft The display.
 * @param tftDL The display list.
 */
void tftDLContentLoad(tft_t *tft, const tftDL_t *tftDL) {
    for (size_t i = 0; i < tftDL->contentLen; i++) {
        const tftContent_t *tftContent = tftDL->content[i];
        tftContentLoad(tft, tftContent);
    }
}

/**
 * @brief Writes a display list.
 *
 * @param tft The display.
 * @param tftDL The display list.
 */
void tftDLWrite(tft_t *tft, const tftDL_t *tftDL) {
    tftCoCmd(tft, tftDL->len, tftDL->data, true);
}

