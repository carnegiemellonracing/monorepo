/* CMR bootloader using OpenBLT
 *
 * Defines functions that should be executed in userside.
 * @author Gustav Hansen
*/


#include "bootloader.h"

/**
  * @brief  Vector base address configuration. It should no longer be at the start of
  *         flash memory but moved forward because the first part of flash is
  *         reserved for the bootloader. Note that this is already done by the
  *         bootloader before starting this program. Unfortunately, function
  *         SystemInit() overwrites this change again.
  * @return none.
  */
static void _VectorBase_Config(void)
{
  /* The constant array with vectors of the vector table is declared externally in the
   * c-startup code.
   */
  extern const unsigned long g_pfnVectors[];

  /* Remap the vector table to where the vector table is located for this program. */
  SCB->VTOR = (unsigned long)&g_pfnVectors[0];
}

void cmr_bootloaderInit(void)
{
#ifdef CMR_ENABLE_BOOTLOADER
    _VectorBase_Config();
#else
#warning "Compiling without bootloader support"
#endif
}
