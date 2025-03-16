/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/can.h>    // CAN interface
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface

#include "gpio.h"       // Board-specific GPIO interface
#include "can.h"        // Board-specific CAN interface
#include "adc.h"        // Board-specific ADC interface
#include "sensors.h"    // Board-specific sensors interface
#include "state.h"      // stateInit()

DAC_HandleTypeDef hdac1;


/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void statusLED(void *pvParameters) {
    (void) pvParameters;

    cmr_gpioWrite(GPIO_OUT_LED_FLASH_RED, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {

      if(LEDerror()){
        cmr_gpioToggle(GPIO_OUT_LED_FLASH_RED);
        cmr_gpioWrite(GPIO_OUT_LED_GREEN, 0);

      }else{
        
        cmr_gpioWrite(GPIO_OUT_LED_GREEN, 1);
      }

      
      vTaskDelayUntil(&lastWakeTime, statusLED_period_ms);

    }
}

static bool LEDerror(void *pvParameters){
  TickType_t lastWakeTime = xTaskGetTickCount();
  if ((getBadModuleState(CANRX_HEARTBEAT_HVC, vsmStatus->canVSMStatus.internalState, lastWakeTime) < 0) || (cmr_gpioRead(GPIO_IN_IMD_ERR) == 1)) {
    return true;
    }
  else{
    return false;
  }

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
//    Error_Handler();
	  cmr_panic("init failed");
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
//    Error_Handler();
	  cmr_panic("failed channel config");
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

  /* USER CODE END DAC1_Init 2 */

}

/**
* @brief DAC MSP Initialization
* This function configures the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdac->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspInit 0 */

  /* USER CODE END DAC1_MspInit 0 */
    /* Peripheral clock enable */
	__HAL_RCC_DAC_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC1 GPIO Configuration    
    PA4     ------> DAC1_OUT1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC1_MspInit 1 */

  /* USER CODE END DAC1_MspInit 1 */
  }

}

/**
 * @brief Firmware entry point.
 *
 * Device configuration and task initialization should be performed here.
 *
 * @return Does not return.
 */
int main(void) {
    // System initialization.
    HAL_Init();
    cmr_rccSystemClockEnable();
    
    HAL_DAC_MspInit(&hdac1);
    MX_DAC1_Init();
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1899);

    // Peripheral configuration.
    gpioInit();
    canInit();
    adcInit();
    sensorsInit();
    stateInit();

    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL
    );

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}
