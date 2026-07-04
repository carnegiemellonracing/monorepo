/************************************************************************************//**
* \file         cmd_flash_board.h
* \brief        Subcommand that flashes application firmware onto a board via OpenBLT,
*               using the XCP on CAN transport only.
****************************************************************************************/
#ifndef CMD_FLASH_BOARD_H
#define CMD_FLASH_BOARD_H

#include <stdint.h>
#include "can_settings.h"

/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Settings, as parsed from the command line, for the flash-board subcommand. */
typedef struct t_flash_board_settings
{
  char const * firmwareFile;   /**< Path to the firmware file to flash onto the board. */
  char const * canDevice;      /**< Name of the CAN device/adapter to use.             */
  uint32_t     canChannel;     /**< Zero based CAN channel index.                      */
} tFlashBoardSettings;

/****************************************************************************************
* Global data
****************************************************************************************/
/** \brief Global settings object for the flash-board subcommand. Populated by
 *         CmdFlashBoardParse() and consumed by CmdFlashBoardRun(). Declared at file
 *         scope (not inline inside a function) so it can be read, returned, and
 *         passed around by any caller.
 */
extern tFlashBoardSettings g_flashBoardSettings;

/****************************************************************************************
* Function prototypes
****************************************************************************************/
void CmdFlashBoardPrintUsage(void);
int  CmdFlashBoardParse(int argc, char * const argv[]);
int  CmdFlashBoardRun(tFlashBoardSettings settings);

#endif /* CMD_FLASH_BOARD_H */
/****************************** end of cmd_flash_board.h *********************************/