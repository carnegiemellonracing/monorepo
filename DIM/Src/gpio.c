/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include "gpio.h"  // Interface to implement

#include <stm32f4xx_hal.h>  // HAL interface

#include "state.h"
#include "newState.h"

static const uint32_t gpioReadButtons_priority = 4;

/** @brief Button input task period (milliseconds). */
static const TickType_t gpioReadButtons_period = 10;

/** @brief Button input task task. */
static cmr_task_t gpioReadButtons_task;

/** @brief Array to store target LED states */
// bool ledTargets[EXP_LED_LEN];

/**
 * @brief Board-specific pin configuration.
 *
 * Replace/add more pin configurations here as appropriate. Each enumeration
 * value of `gpio_t` should get a configuration.
 *
 * @see `stm32f4xx_hal_gpio.h` for various initialization values.
 */
static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN] = {
    // Change LRUD
	//Delete LRUD
	[GPIO_BUTTON_A] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_0,
					.Mode = GPIO_MODE_IT_RISING_FALLING, .Pull = GPIO_PULLUP,
					.Speed = GPIO_SPEED_FREQ_LOW } },
	[GPIO_BUTTON_B] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_1,
				.Mode = GPIO_MODE_IT_RISING_FALLING, .Pull = GPIO_PULLUP,
				.Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_SW1] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_2,
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, 
                        .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_SW2] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_3,
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, 
                        .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_PUSH] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_4,
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, 
                        .Speed = GPIO_SPEED_FREQ_LOW } },
	[GPIO_LED_AMS] = { .port = GPIOA, .init = { .Pin = GPIO_PIN_6,
                       .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL,
                       .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_LED_IMD] = { .port = GPIOA, .init = { .Pin = GPIO_PIN_7,
                       .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL,
                       .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_LED_BSPD] = { .port = GPIOA, .init = { .Pin = GPIO_PIN_8,
                        .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL,
                        .Speed = GPIO_SPEED_FREQ_LOW } },
};

void static Arising();
void static Afalling();
void static Brising();
void static Bfalling();

//Define the two variables that tracks rotary input
volatile int RotaryA = 0;
volatile int RotaryB = 0;
/**
* @brief Adds Interrupt and Programs Callback Function
*/
volatile static int rotaryPosition = 0; //This keeps track of rotary position, the important variable, mod 8
volatile static int pastRotaryPosition = 0; //Keeps track of past rotary position, mod 8


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//TODO: CHANGE TO CMR
	if(GPIO_Pin == GPIO_PIN_0){
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET){
			Arising();
			reqGear();
		} else {
			Afalling();
			reqGear();
		}
    }
	if(GPIO_Pin == GPIO_PIN_1){
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET){
			Brising();
			reqGear();
		} else {
			Bfalling();
			reqGear();
		}
	}
}

/**
* @brief Adds the four function that will be called in the Callback Function
*
* Clockwise is negative, counterclockwise is positive
* To the future firmware people, watch this video to understand it
* https://www.youtube.com/watch?v=v4BbSzJ-hz4&t=157s&ab_channel=HowToMechatronics
*
*/
//TODO: Check with actual switch to figure out can reduce to two cases?
void static Arising(){
	int RotaryA = 1;
	if (RotaryB!=RotaryA){
		pastRotaryPosition = rotaryPosition;
		//wrap around logic
		rotaryPosition = ((rotaryPosition-1)%8+8)%8;
	} else {
		pastRotaryPosition = rotaryPosition;
		rotaryPosition = (rotaryPosition+1)%8;
	}
}
void static Afalling(){
	int RotaryA = 0;
	if (RotaryB!=RotaryA){
		pastRotaryPosition = rotaryPosition;
		//wrap around logic
		rotaryPosition = ((rotaryPosition-1)%8+8)%8;
	} else {
		rotaryPosition = (rotaryPosition+1)%8;
		pastRotaryPosition = rotaryPosition;
	}
}
void static Brising(){
	int RotaryB = 1;
	if (RotaryA!=RotaryB){
		pastRotaryPosition = rotaryPosition;
		rotaryPosition = (rotaryPosition+1)%8;
	} else {
		pastRotaryPosition = rotaryPosition;
		//wrap around logic
		rotaryPosition = ((rotaryPosition-1)%8+8)%8;
	}
}
void static Bfalling(){
	int RotaryB = 0;
	if (RotaryA!=RotaryB){
		pastRotaryPosition = rotaryPosition;
		rotaryPosition = (rotaryPosition+1)%8;
	} else {
		pastRotaryPosition = rotaryPosition;
		//wrap around logic
		rotaryPosition = ((rotaryPosition-1)%8+8)%8;
	}
}


/**
 * @brief Checks current LED state and updates if different from `ledTargets`
 *
 * Not wholly generic since it's too complicated,
 * so it uses the fact that all LEDs are on Main Digital 2 Port 1
 *
 */
// static void checkLEDState()
// {
//     uint8_t targetMask = 0;
//     uint8_t targetState = 0;

//     for (size_t l = EXP_LED_1; l < EXP_LED_LEN; l++)
//     {
//         uint8_t pin = leds[l].pin;
//         targetMask |= 1 << pin;
//         targetState |= ledTargets[l] << pin;
//     }

//     // See note above about non-generic code
//     if ((currentState & targetMask) != targetState)
//     {
//         // uint8_t cmd[2] = {
//         //     PCA9554_OUTPUT_PORT,
//         //     targetState
//         // };

//     }
// }

// void expanderSetLED(expanderLED_t led, bool isOn)
// {
//    ledTargets[led] = isOn;
// }
//declaration for use
static void XYActivate(void);

/*
debouncing for button presses for LRUD
 */
# define DEBOUNCE_DELAY 50 //TODO: in milliseconds, to change
static void canLRUDdebounce (cmr_LRUD_index button){
	//set can state to false to switch it off
	canLRUDStates[button] = false;
	//delay by ((debounce delay time) / (time per tick)) ticks
	vTaskDelay(DEBOUNCE_DELAY / portTICK_PERIOD_MS);
	XYActivate();
	if (gpioLRUDStates[button] == true){
		while(gpioLRUDStates[button] == true){
			vTaskDelay(portTICK_PERIOD_MS);
			XYActivate();
		}
		canLRUDStates[button] = true;
	}
}

// master function for detecting and changing can button states, will be in gpio loop
void canLRUDDetect(void){
	XYActivate();
	for(int i=0; i<LRUDLen; i++){
		if(gpioLRUDStates[i] == true){
			canLRUDdebounce(i);
		}
	}
}

/**
 * @brief Converts analog ADC to 0-3.3V, then scales that to 0-5V
 *
 * @param analog ADC value between 0-4096. Helper function for adcToXY
 *
 * @return a float between 0-5V.
 */

static float adcToVoltage(uint32_t analog){

	//error check
	if(analog > 4096){
		return -1;
	}else{
		//Scale analog to voltage between 0-5V
		float finalVolt = (analog/4096) * 5;

		return finalVolt;
	}
}


/* Reads ADC input and switch case based on voltage values and has corresponding states boolean variables
		*Case 1: below 0.5V both
		*Case 2: X between 0.5V and 4.5V and Y below 0.5V
		*Case 3: Y between 0.5V and 4.5V and X below 0.5V
		*Case 4: above 4.5V both
		*/
//add can buttons to this
static void XYActivate(void){
	float sensorX = adcToVoltage(cmr_sensorListGetValue(&sensorList, SENSOR_CH_X));
	float sensorY = adcToVoltage(cmr_sensorListGetValue(&sensorList, SENSOR_CH_Y));
	// Both sensors less than 0.5V
    // LEFT
	if (sensorX <=0.5 && sensorY <=0.5){
		gpioLRUDStates[LEFT] = true;
	}else{
		gpioLRUDStates[LEFT] = false;
	}
	if (sensorX<=4.5 && sensorX >=0.5 && sensorY <=0.5){
		gpioLRUDStates[RIGHT] = true;
	}else{
		gpioLRUDStates[RIGHT] = false;
	}
	if (sensorX <=0.5 && sensorY>=0.5 && sensorY <=4.5){
		gpioLRUDStates[UP] = true;
	}else{
		gpioLRUDStates[UP] = false;
	}
	if (sensorX >=4.5 && sensorY >=4.5){
		gpioLRUDStates[DOWN] = true;
	}else{
		gpioLRUDStates[DOWN] = false;
	}
}


/**
 * @brief reads state of all buttons
 */
static void gpioReadButtons(void *pvParameters) {
    (void)pvParameters;
    bool pressConfirmed = false;
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // Direct assignment for CAN buttons
        for(int i=0; i<NUM_BUTTONS; i++){
			//TODO: two button states
            canButtonStates[i] = (HAL_GPIO_ReadPin(gpioPinConfigs[i].port, gpioPinConfigs[i].init.Pin) == GPIO_PIN_RESET);
        }
		canLRUDDetect();
    }
}

/**
* @brief Gets rotary position
*/
int getRotaryPosition(){
	return rotaryPosition;
}

/**
* @brief: Gets past rotary position
*/
int getPastRotaryPosition(){
	return pastRotaryPosition;
}


/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    cmr_gpioPinInit(
        gpioPinConfigs, sizeof(gpioPinConfigs) / sizeof(gpioPinConfigs[0]));
    cmr_taskInit(
        &gpioReadButtons_task,
        "gpioReadButtons",
        gpioReadButtons_priority,
        gpioReadButtons,
        NULL);
}