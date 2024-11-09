/**
 * @file newState.c
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

#include "gpio.h"       // Board-specific GPIO interface
#include "can.h"        // Board-specific CAN interface
#include "adc.h"        // Board-specific ADC interface
#include "tft.h"        // TFT display interface.
#include "expanders.h"   // LED strip interface.
#include "test.h"
#include "newState.h"
#include <stdlib.h>
#include <stdio.h>

static const uint32_t stateMachine_priority = 4;

/** @brief Button input task period (milliseconds). */
static const TickType_t stateMachine_period = 10;

/** @brief Button input task task. */
static cmr_task_t stateMachine_task;

static void getReqScreen(void) {
    if(stateGetVSM() == CMR_CAN_ERROR) return ERROR;
    /*case if we use safety screen
    if(stateGetVSM() == CMR_CAN_ERROR) {
        if(stateGetVSMReq() == CMR_CAN_HV_EN) return SAFETY;
        return ERROR;
    }
    */
    switch (currState) {
        case INIT:
            /* Ensure that Chip ID is read correctly */
            uint32_t chipID; 
            tftRead(tft, TFT_ADDR_CHIP_ID, sizeof(chipID), &chipID);
            if(chipID == TFT_CHIP_ID){
                /* Initialize Video Registers. */
                for (size_t i = 0; i < sizeof(tftInits) / sizeof(tftInits[0]); i++) {
                    const tftInit_t *init = tftInits + i;
                    tftWrite(tft, init->addr, sizeof(init->val), &init->val);
                }
                /* Enable Faster Clock Rate now that initialization is complete */
                cmr_qspiSetPrescaler(&tft->qspi, TFT_QSPI_PRESCALER);
                nextState = START; 
                //depends if we want a start screen
            }
            break;
        case START:
            if(stateGetVSMReq() == CMR_CAN_GLV_ON) {
                nextState = NORMAL;
            }
            else nextState = START;
            break;
        case NORMAL:
            if(gpioButtonStates[L]) {
                nextState = CONFIG;
                gpioButtonStates[L] = false;
            }
            else if(gpioButtonStates[R]) {
                nextState = RACING;
                gpioButtonStates[R] = false;
            }
            else nextState = NORMAL;
            break;
        case CONFIG:
            //look into how button move on screen on campus
            if(gpioButtonStates[L]) {
                //move left on screen
                gpioButtonStates[L] = 0;
                nextState = CONFIG;
            }
            else if(gpioButtonStates[R]) {
                //move right on screen
                gpioButtonStates[R] = 0;
                nextState = CONFIG;
            }
            else if(gpioButtonStates[U]) {
                //move up on screen
                gpioButtonStates[U] = 0;
                nextState = CONFIG;
            }
            else if(gpioButtonStates[D]) {
                //move down on screen
                gpioButtonStates[D] = 0;
                nextState = CONFIG;
            }
            else if(gpioButtonStates[SW1]) {
                nextState = NORMAL;
                gpioButtonStates[SW1] = 0;
                nextState = CONFIG;
            }
            else if(gpioButtonStates[SW2]) {
                //move right on screen
                nextState = RACING;
                gpioButtonStates[SW2] = 0;
            }
            else nextState = CONFIG;
            break;
        case ERROR:
            nextState = INIT;
            break;
        case RACING:
            //need to incorporate the other sw buttons, accel?
            if(gpioButtonStates[L] && stateGetVSMReq() == CMR_CAN_GLV_ON) {
                nextState = CONFIG;
                gpioButtonStates[U] = false;
            }
            else if(gpioButtonStates[R]) {
                nextState = NORMAL;
                gpioButtonStates[R] = false;
            }
            else nextState = RACING;
            break;
        default:
            nextState = INIT;
    }
}

static void stateOutput() { 
    //output
    switch(currState) {
        case INIT:
            //initialize buttons to 0
            for(int i=0; i<NUM_BUTTONS; i++){
                //is it necessary to initialize the can buttons to 0 if they are just reading pins??
                canButtonStates[i] = 0;
                gpioButtonStates[i] = 0;
            }
             /* Restarting the Display. */
            TickType_t lastWakeTime = xTaskGetTickCount();
            cmr_gpioWrite(GPIO_PD_N, 0);  // TODO figure out pin
            vTaskDelayUntil(&lastWakeTime, TFT_RESET_MS);
            cmr_gpioWrite(GPIO_PD_N, 1);
            vTaskDelayUntil(&lastWakeTime, TFT_RESET_MS);

            /* Initialize the display. */
            tftCmd(tft, TFT_CMD_CLKEXT, 0x00);
            tftCmd(tft, TFT_CMD_ACTIVE, 0x00);
            tftCmd(tft, TFT_CMD_ACTIVE, 0x00);
            break;
        case START:
            /* Display Startup Screen for fixed time */
            tftDLContentLoad(tft, &tftDL_startup);
            tftDLWrite(tft, &tftDL_startup);
            //    vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS);
            break;
        case NORMAL:
            drawRTDScreen(); //from something
            break;
        case CONFIG:
            drawConfigScreen();
            break;
        case ERROR:
            drawErrorScreen();
            break;
        case RACING:
            drawRacingScreen();
            break;
    }
    currState = getReqScreen();
}

static void stateMachine(void *pvParameters){
    (void)pvParameters;
    TickType_t lastWakeTime = xTaskGetTickCount();
    currState = INIT;
    while (1) {
        taskENTER_CRITICAL();
        getReqScreen();
        state_output();
        taskEXIT_CRITICAL();
    }
    vTaskDelayUntil(&lastWakeTime, stateMachine_period);
}
//want to pack into cmr driver 

/**
 * @brief Initializes the state machine interface.
 */
void stateMachineInit(void) {
    cmr_taskInit(
        &stateMachine_task,
        "stateMachine",
        stateMachine_priority,
        stateMachine,
        NULL);
}

