/****************************************************************************************
* Include files
****************************************************************************************/
#include "setup.h"                                /* bootloader generic header          */
#include "gpio.h"
#include "can.h"
#include "clock.h"
#include "rs232.h"



/************************************************************************************//**
** \brief     Initializes the microcontroller.
** \return    none.
**
****************************************************************************************/
void Init(void)
{
  /* HAL library initialization */
  HAL_Init();
  /* configure system clock */
  clockInit();
  /* configure other peripherals */
  gpioInit();
  canInit();
  rs232Init();
} /*** end of Init ***/


void DeInit(void)
{
    canDeinit();
    gpioDeinit();
    rs232Deinit();

    /* HAL library deinitialization */
    HAL_DeInit();
    clockDeinit();
} /*** end of DeInit ***/

