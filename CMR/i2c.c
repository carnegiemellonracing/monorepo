/**
 * @file i2c.c
 * @brief Driver for I2C.
 *
 * @author Carnegie Mellon Racing
 */
#include "i2c.h"    //interface
#include "panic.h"  //cmr_panic()


int cmr_i2cTX(int commandAddr, uint8_t *data, int dataLength){
	const int dev_addr = 0x60;
	uint8_t new_data[dataLength + 1];
	for (int i = 0; i < dataLength; i++) {
		new_data[i + 1] = data[i];
	}
	new_data[0] = (uint8_t) commandAddr;
	//Shift the address by 1 per HAL library suggestion
	HAL_I2C_Master_Transmit(&hi2c1, dev_addr << 1, new_data, dataLength + 1, 0xFFFF);
	return 0;
}


/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    cmr_panic("I2C Initialization Failed");
  }
}

void cmr_i2cInit(
    cmr_i2c_t *i2c, I2C_Typedef *instance,
    cmr_adcChannel_t *channels, const size_t channelsLen
) {
    if (channelsLen > CMR_ADC_CHANNELS) {
        cmr_panic("Too many channels");
    }

    *adc = (cmr_adc_t) {
        .handle = {
            .Instance = instance,

            // Configure ADC in discontinuous scan mode.
            // This will allow conversion of a series of channels one at a time.
            .Init = {
                .ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4,
                .Resolution = ADC_RESOLUTION_12B,
                .ScanConvMode = ENABLE,
                .ContinuousConvMode = DISABLE,
                .DiscontinuousConvMode = ENABLE,
                .NbrOfDiscConversion = 1,
                .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
                .ExternalTrigConv = ADC_SOFTWARE_START,
                .DataAlign = ADC_DATAALIGN_RIGHT,
                .NbrOfConversion = channelsLen,
                .DMAContinuousRequests = DISABLE,
                .EOCSelection = ADC_EOC_SINGLE_CONV
            }
        },
        .channels = channels,
        .channelsLen = channelsLen
    };

    cmr_rccADCClockEnable(instance);

    if (HAL_ADC_Init(&adc->handle) != HAL_OK) {
        cmr_panic("HAL_ADC_Init() failed!");
    }

    cmr_adcConfigChannels(adc);

    // Task creation.
    xTaskCreate(
        cmr_adcSample_task, "adcSample",
        configMINIMAL_STACK_SIZE, adc, cmr_adcSample_priority, NULL
    );
}


