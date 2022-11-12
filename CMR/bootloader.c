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

/**
 * @brief Initialize the bootloader, ie set the correct vectorbase after getting
 * control from the bootloader
 * @return none
 */
void cmr_bootloaderInit(void)
{
#ifdef CMR_ENABLE_BOOTLOADER
    _VectorBase_Config();
#else
#warning "Compiling without bootloader support"
#endif
}

/**
 * @brief Callback for every time that a message is received from CAN to
 * check if we should system reset and enter bootloader
 */
void cmr_bootloaderReceiveCallback(CAN_RxHeaderTypeDef *msg, uint8_t *rxData)
{
#ifdef CMR_ENABLE_BOOTLOADER
    if (msg->StdId == CMR_CANID_OPENBLT_XMP_RX) {
        if (rxData[0] == 0xff && rxData[1] == CMR_ENABLE_BOOTLOADER) {
            NVIC_SystemReset();
        }
        return;
    }
#endif
}

void cmr_bootloaderCanFilter(cmr_can_t *can) {
#ifdef CMR_ENABLE_BOOTLOADER
    cmr_canFilter_t bootloaderFilter[] = {
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_OPENBLT_XMP_RX,
                CMR_CANID_OPENBLT_XMP_RX,
                CMR_CANID_OPENBLT_XMP_RX,
                CMR_CANID_OPENBLT_XMP_RX
            }
        }
    };
    cmr_canFilter(can, bootloaderFilter, 1);
#endif
}
