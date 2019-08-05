/**
 * @file tft.h
 * @brief TFT display interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef TFT_H
#define TFT_H

void tftInit(void);

/** @brief All of the errors to be drawn on-screen
 * in error state. */
typedef struct {
    bool fsmTimeout;    /**< @brief Has the FSM timed out? */
    bool cdcTimeout;    /**< @brief Has the CDC timed out? */
    bool ptcTimeout;    /**< @brief Has the PTC timed out? */
    bool vsmTimeout;    /**< @brief Has the VSM timed out? */
    bool afc1Timeout;   /**< @brief Has AFC1 timed out? */
    bool afc2Timeout;   /**< @brief Has AFC2 timed out? */
    bool imdError;      /**< @brief Has the IMD faulted? */
    bool amsError;      /**< @brief Has the AMS faulted? */
    bool bspdError;     /**< @brief Has the BSPD tripped? */
    bool overVolt;      /**< @brief Has the HVC errored w/ OV? */
    bool underVolt;     /**< @brief Has the HVC errored w/ UV? */
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
} tft_errors_t;

#endif /* TFT_H */

