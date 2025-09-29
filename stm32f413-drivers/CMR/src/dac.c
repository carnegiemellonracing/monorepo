/**
 * @file dac.c
 * @brief Implements DAC.h
 *
 * @author Carnegie Mellon Racing
 */


#ifdef HAL_DAC_MODULE_ENABLED

#include "dac.h"

//Forward Declarations
static void MX_DAC1_Init(void);
static void dacPinConfig(cmr_dacPinConfig_t*);

// Globals
DAC_HandleTypeDef hdac1;


dacInit(uint16* pins, size_t dacConfigsLen){
    MX_DAC1_Init();
    for (uint8_t i=0; i<dacConfigsLen; i++){
        dacPinConfig(pins[i])
    }
} 

void dacSetValue(uint16_t pin, uint16_t voltage_mV){
    uint32_t voltage_mV_32Bits = (uint32_t) voltage_mV;
    voltage_12Bits = voltage_mV_32Bits * (4095)/ 3300;

    if (pin == 4){
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, voltage_12Bits);
    }

    if (pin == 5){
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, voltage_12Bits);
    }

}

/**
  * @brief Intializes a DAC pin
  * @param None
  * @retval None
  */
static void dacPinConfig(uint16_t pin){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hdac->Instance==DAC1)
    {
        __HAL_RCC_DAC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    
    if (pin == 4){
        HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    }

    if (pin == 5){
        HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    }

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{
    DAC_ChannelConfTypeDef sConfig = {0};
    hdac1.Instance = DAC1;

    if (HAL_DAC_Init(&hdac1) != HAL_OK)
    {
        cmr_panic("init failed");
    }

    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    {
        cmr_panic("failed channel config");
    }

}

#endif HAL_DAC_MODULE_ENABLED

