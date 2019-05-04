/**
 * @file tft.h
 * @brief TFT display interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef TFT_H
#define TFT_H

void tftInit(void);

typedef struct {
    bool fsmTimeout;
    bool cdcTimeout;
    bool ptcTimeout;
    bool vsmTimeout;
    bool afc1Timeout;
    bool afc2Timeout;
    bool imdError;
    bool amsError;
    bool bspdError;
    bool overVolt;
    bool underVolt;
    bool hvcoverTemp;
    bool hvc_Error;
    bool overSpeed;
    bool mcoverTemp;
    bool overCurrent;
    bool mcError;
    uint16_t hvcErrorNum;
    unsigned int mcErrorNum;
} tft_errors_t;

#endif /* TFT_H */

