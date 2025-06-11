#include "pwm.h"

cmr_pwm_t pwmWrite1;
cmr_pwm_t pwmWrite2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

// /**
//  * @brief Initialises PWM write/output interface
//  * 
//  */
// void pwmWriteInit() {
//     const cmr_pwmPinConfig_t pwmPinConfig1 = { // GPIO A1
//         .port = GPIOA, 
//         .pin = GPIO_PIN_1,
//         .channel = TIM_CHANNEL_1,
//         .presc = 107,
//         .period_ticks = 1000,
//         .timer = TIM2
//     };
//     const cmr_pwmPinConfig_t pwmPinConfig2 = { // GPIO A2
//         .port = GPIOA,
//         .pin = GPIO_PIN_2,
//         .channel = TIM_CHANNEL_2,
//         .presc = 107,
//         .period_ticks = 1000,
//         .timer = TIM2
//     };

//     cmr_pwmInit(&pwmWrite1, &pwmPinConfig1);
//     cmr_pwmInit(&pwmWrite2, &pwmPinConfig2);
// }

// /**
//  * @brief Writes a PWM signal to the specified
//  * @param pwm_channel with a
//  * @param duty_cycle in percent (0 to 100)
//  */
// void pwmWrite(uint8_t pwm_channel, uint32_t duty_cycle) {
//     if (pwm_channel == 1) cmr_pwmSetDutyCycle(&pwmWrite1, duty_cycle);
//     if (pwm_channel == 2) cmr_pwmSetDutyCycle(&pwmWrite2, duty_cycle);
// }


uint32_t cnt_full2 = 0;
uint32_t cnt_high2 = 0;
float freq2 = 0;
float duty2 = 0;

uint32_t cnt_full3 = 0;
uint32_t cnt_high3 = 0;
float freq3 = 0;
float duty3 = 0;

uint32_t cnt_full4 = 0;
uint32_t cnt_high4 = 0;
float freq4 = 0;
float duty4 = 0;

uint32_t cnt_full5 = 0;
uint32_t cnt_high5 = 0;
float freq5 = 0;
float duty5 = 0;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		cnt_full2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) + 2;
		cnt_high2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) + 2;

		freq2 = (float) TIMER_CLOCK_FREQ / (cnt_full2);
		duty2 = (float) 100 * cnt_high2 / cnt_full2;
	} else if (htim->Instance == TIM3) {
		cnt_full3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) + 2;
		cnt_high3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) + 2;

		freq3 = (float) TIMER_CLOCK_FREQ / (cnt_full3);
		duty3 = (float) 100 * cnt_high3 / cnt_full3;
	} else if (htim->Instance == TIM4) {
		cnt_full3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) + 2;
		cnt_high3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) + 2;

		freq4 = (float) TIMER_CLOCK_FREQ / (cnt_full4);
		duty4 = (float) 100 * cnt_high4 / cnt_full4;
	} else {
		cnt_full3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) + 2;
		cnt_high3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) + 2;

		freq5 = (float) TIMER_CLOCK_FREQ / (cnt_full5);
		duty5 = (float) 100 * cnt_high5 / cnt_full5;
	} 
}


static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    cmr_panic("Unable to initialise TIM2 for HITL PWM reads");
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    cmr_panic("Unable configure clock source for TIM2");
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    cmr_panic("Unable to initialise internatal capture for TIM2");
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    cmr_panic("Unable to synchronise TIM2 Slave");
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    cmr_panic("Unable to configure TIM2 Channel 1");
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    cmr_panic("Unable to configure TIM2 Channel 2");
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    cmr_panic("Unable to synchronise TIM2 Master");
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM3;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    cmr_panic("Unable to initialise TIM3 for HITL PWM reads");
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    cmr_panic("Unable configure clock source for TIM3");
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    cmr_panic("Unable to initialise internatal capture for TIM3");
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    cmr_panic("Unable to synchronise TIM3 Slave");
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    cmr_panic("Unable to configure TIM3 Channel 1");
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    cmr_panic("Unable to configure TIM3 Channel 2");
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    cmr_panic("Unable to synchronise TIM3 Master");
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM4;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    cmr_panic("Unable to initialise TIM4 for HITL PWM reads");
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    cmr_panic("Unable configure clock source for TIM4");
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    cmr_panic("Unable to initialise internatal capture for TIM4");
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    cmr_panic("Unable to synchronise TIM4 Slave");
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    cmr_panic("Unable to configure TIM4 Channel 1");
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    cmr_panic("Unable to configure TIM4 Channel 2");
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    cmr_panic("Unable to synchronise TIM4 Master");
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM5;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    cmr_panic("Unable to initialise TIM5 for HITL PWM reads");
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    cmr_panic("Unable configure clock source for TIM5");
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    cmr_panic("Unable to initialise internatal capture for TIM5");
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    cmr_panic("Unable to synchronise TIM5 Slave");
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    cmr_panic("Unable to configure TIM5 Channel 1");
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    cmr_panic("Unable to configure TIM5 Channel 2");
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    cmr_panic("Unable to synchronise TIM5 Master");
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}



void pwmReadInit() {
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim5, TIM_CHANNEL_2);
}

void pwmRead(uint8_t tim, uint32_t *frequency, uint32_t *duty_cycle) {
    if (tim == 2) {
        *frequency = freq2;
        *duty_cycle = duty2;
    } else if (tim == 3) {
        *frequency = freq3;
        *duty_cycle = duty3;
    } else if (tim == 4) {
        *frequency = freq4;
        *duty_cycle = duty4;
    } else {
        *frequency = freq5;
        *duty_cycle = duty5;
    }
}
