/**
 * @file ledStrip.h
 * @brief LED strip interface
 *
 * @author Carnegie Mellon Racing
 */

#ifndef LED_STRIP_H
#define LED_STRIP_H

#define NUM_LEDS 10

void ledStripInit(void);

void setNumLeds(unsigned int numLeds);

#endif /* LED_STRIP_H */

