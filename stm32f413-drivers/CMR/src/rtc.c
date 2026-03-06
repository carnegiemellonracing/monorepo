/**
 * @file rtc.c
 * @brief Real-time clock implementation.
 *
 * @author Ayush Garg
 */

#ifdef F413
// Standard library
#include <stdio.h>
#include <string.h>
#include <time.h>

// CMR framework
#include <CMR/panic.h>
#include <CMR/platform.h>
#include <CMR/rtc.h>

RTC_HandleTypeDef hrtc;

RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;
time_t timestamp;
struct tm currTime;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_RTC_Init(void);

RTC_TimeTypeDef getRTCTime() {
    HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
    return currentTime;
}

RTC_DateTypeDef getRTCDate() {
    HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
    return currentDate;
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void cmr_rtc_init(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  HAL_PWR_EnableBkUpAccess();

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
      cmr_panic("RTC clock config failed!");
  }

  __HAL_RCC_RTC_ENABLE();

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET);
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    cmr_panic("RTC Init Failed");
  }

}

#endif /* F413 */
