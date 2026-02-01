/**
 * @file dac.c
 * @brief Implements DAC.h
 *
 * @author Ayush Garg
 */

#include <CMR/dac.h>   // Interface to implement
#include "CMR/panic.h"    // cmr_panic()


#ifdef HAL_DAC_MODULE_ENABLED
#ifdef F413

#define MAX_NUMBER_OF_DAC_CHANNELS 2

//Forward Declarations
static void MX_DAC1_Init(void);
static void dacPinConfig(cmr_gpioPin_t pin);
static uint32_t pinToDacChannel(cmr_gpioPin_t pin);

// Globals
DAC_HandleTypeDef hdac1;
static uint32_t pinToChannel[MAX_NUMBER_OF_DAC_CHANNELS];
static size_t numberOfPins;

/**
  * @brief Inits the DAC
  * @param pins The array containing the pins
  * @param dacConfigsLen The number of DAC channels to init
  */
void cmr_dacInit(cmr_gpioPin_t* pins, size_t dacConfigsLen){
    if (MAX_NUMBER_OF_DAC_CHANNELS < numberOfPins){
        cmr_panic("More Pins than DAC Channels");
    }
    numberOfPins = dacConfigsLen;

    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK)
    {
        cmr_panic("Init failed");
    }

    for (uint8_t i=0; i<dacConfigsLen; i++){
        pinToChannel[i] = pinToDacChannel(pins[i]);
        dacPinConfig(pins[i]);
    }
} 

/**
  * @brief Sets a DAC pin voltage
  * @param pin the pin number from the init
  * @param voltage_mV the voltage we want to set
  */
void cmr_dacSetValue(size_t pin, uint16_t voltage_mV){
    uint32_t voltage_mV_32Bits = (uint32_t) voltage_mV;
    uint32_t voltage_12Bits = voltage_mV_32Bits * (4095)/ 3300;
    uint32_t dacChannel = pinToChannel[pin];

    HAL_DAC_SetValue(&hdac1, dacChannel, DAC_ALIGN_12B_R, voltage_12Bits);
}

/**
  * @brief Intializes a DAC pin
  * @param None
  * @retval None
  */
static void dacPinConfig(cmr_gpioPin_t pin){
    uint32_t dacChannel = pinToDacChannel(pin);

    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, dacChannel) != HAL_OK)
    {
        cmr_panic("Failed channel config");
    }

    __HAL_RCC_DAC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    cmr_gpioPinConfig_t pinConfig;
    pinConfig.port = pin.port;
    pinConfig.init.Pin = pin.pin;
    pinConfig.init.Mode = GPIO_MODE_ANALOG;
    pinConfig.init.Pull = GPIO_NOPULL;
    cmr_singleGpioPinInit(&pinConfig);

    HAL_DAC_Start(&hdac1, dacChannel);
}

/**
  * @brief Converts the pin to a DAC Channel
  * @param pin The GPIO pin we want to convert
  * @retval The corresponding channel
  */
static uint32_t pinToDacChannel(cmr_gpioPin_t pin){
    if(pin.port == GPIOA && pin.pin == GPIO_PIN_4){
        return DAC_CHANNEL_1;
    }
    else if (pin.port == GPIOA && pin.pin == GPIO_PIN_5){
        return DAC_CHANNEL_2;
    }
    else{
        cmr_panic("DAC Channel Not Found");
    }
    return 0; //placate compiler
}

#endif /* F413 */ 
#endif /* HAL_DAC_MODULE_ENABLED */ 