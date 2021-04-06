/**
 * @file tft.h
 * @brief TFT display interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef TFT_H
#define TFT_H

/** @brief setting the thresholds for the backgrounds to turn yellow and red*/
#define AC_YELLOW_THRESHOLD 55
#define AC_RED_THRESHOLD 60
#define MOTOR_YELLOW_THRESHOLD 100
#define MOTOR_RED_THRESHOLD 110
#define MC_YELLOW_THRESHOLD 55
#define MC_RED_THRESHOLD 60

void tftInit(void);

/** @brief All of the errors to be drawn on-screen
 * in error state. */
typedef struct {
    bool fsmTimeout;    /**< @brief Has the FSM timed out? */
    bool cdcTimeout;    /**< @brief Has the CDC timed out? */
    bool ptcfTimeout;   /**< @brief Has the PTCf timed out? */
    bool ptcpTimeout;   /**< @brief Has the PTCp timed out? */
    bool vsmTimeout;    /**< @brief Has the VSM timed out? */
    bool apcTimeout;    /**< @brief Has the APC timed out? */
    bool hvcTimeout;    /**< @brief Has the HVC timed out? */
    bool imdError;      /**< @brief Has the IMD faulted? */
    bool amsError;      /**< @brief Has the AMS faulted? */
    bool bspdError;     /**< @brief Has the BSPD tripped? */
    bool overVolt;      /**< @brief Has the HVC errored w/ OV? */
    bool underVolt;     /**< @brief Has the HVC errored w/ UV? */
    bool glvLowVolt;    /**< @brief Has the GLV errored w/ UV? */
    bool hvcoverTemp;   /**< @brief Has the HVC errored w/ over temp? */
    bool hvc_Error;     /**< @brief Has a BMB errored (probably timed out)? */
    bool overSpeed;     /**< @brief Has the RMS errored? */
    bool mcoverTemp;    /**< @brief Has the RMS errored? */
    bool overCurrent;   /**< @brief Has the RMS errored? */
    bool mcError;       /**< @brief Has the RMS errored? */
    uint16_t hvcErrorNum;       /**< @brief Give the error number,
    * So the driver can relay even if the error is not accounted for above. */
    unsigned int mcErrorNum;    /**< @brief Give the error number,
    * So the driver can relay even if the error is not accounted for above. */
    unsigned int glvVoltage_V; /**< @brief GLV Voltage in Volts */
} tft_errors_t;

typedef enum {
    SBG_STATUS_NOT_CONNECTED = 0,        /** @brief INS not connected/not sending info */
    SBG_STATUS_WORKING_NO_POS_FOUND,    /** @brief INS working but doesn't have fix on position */
    SBG_STATUS_WORKING_POS_FOUND        /** @brief INS working and has fix on position */
} SBG_status_t;

#endif /* TFT_H */

