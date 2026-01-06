/**
 * @file rtc.h
 * @brief Real-time clock interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef RTC_H
#define RTC_H

#include <stdio.h>
#include <string.h>
#include <time.h>

void SystemClock_Config(void);
void MX_RTC_Init(void);
RTC_TimeTypeDef getRTCTime();
RTC_DateTypeDef getRTCDate();

#endif /* RTC_H */