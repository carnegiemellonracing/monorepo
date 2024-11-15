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
#include <tftPrivate.h>
/** @brief setting the thresholds for the backgrounds to turn yellow and red*/
#define AC_YELLOW_THRESHOLD 55
#define AC_RED_THRESHOLD 57
#define MOTOR_YELLOW_THRESHOLD 100
#define MOTOR_RED_THRESHOLD 115
#define MC_YELLOW_THRESHOLD 48
#define MC_RED_THRESHOLD 58

/** @brief Display reset time, in milliseconds. */
#define TFT_RESET_MS 50


/** @brief Display update period. */
#define TFT_UPDATE_PERIOD_MS 20

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

#endif /* TFT_H */
