/****************************************************************************************
* Include files
****************************************************************************************/
#include "setup.h"                                /* bootloader generic header          */
#include "gpio.h"
#include "can.h"
#include "clock.h"
#include <cmr/remote_boot.h>
#include <cmr/board_info.h>

void ValidateConfig(void)
{
    cmr_validateBoardInfo();
} /*** end of ValidateConfig ***/

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
  /* initialize remote boot */
  cmr_remoteBootInit();
} /*** end of Init ***/


void DeInit(void)
{
    /* deconfigure other peripherals */
    canDeinit();
    gpioDeinit();

    /* HAL library deinitialization */
    HAL_DeInit();
    /* Clock deinitialization */
    clockDeinit();
} /*** end of DeInit ***/