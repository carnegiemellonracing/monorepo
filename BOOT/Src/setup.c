/****************************************************************************************
* Include files
****************************************************************************************/
#include "setup.h"                                /* bootloader generic header          */
#include "gpio.h"
#include "can.h"
#include "clock.h"



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

/**
 * @brief Deinitializes board peripherals and HAL.
 *
 * This function performs orderly shutdown of peripherals used by the
 * bootloader (CAN, GPIO, etc.), deinitializes the HAL library and returns
 * the system to a safe state. It complements `Init` which performs the
 * corresponding initialization steps.
 */

