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
#include <stm32f4xx_hal_rtc.h>
#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_pwr_ex.h>
#include <stm32f4xx_hal_flash_ex.h>

void SystemClock_Config(void);
void MX_RTC_Init(void);
RTC_TimeTypeDef getRTCTime();
RTC_DateTypeDef getRTCDate();

#endif /* RTC_H */