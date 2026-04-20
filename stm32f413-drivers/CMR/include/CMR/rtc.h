/**
 * @file rtc.h
 * @brief Real-time clock interface.
 *
 * @author Carnegie Mellon Racing
 */

 #ifdef F413


 #pragma  once

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stm32f4xx_hal_rtc.h>
#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_pwr_ex.h>
#include <stm32f4xx_hal_flash_ex.h>

void cmr_rtc_init(void);
RTC_TimeTypeDef getRTCTime();
RTC_DateTypeDef getRTCDate();

#endif /* F413 */
