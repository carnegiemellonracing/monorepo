/************************************************************************************//**
* \file         cmd_flash_bootloader.h
* \brief        Subcommand that configures, builds and flashes the OpenBLT bootloader
*               (the "BOOT" cmake project) for a given board, using STM32_Programmer_CLI.
****************************************************************************************/
#ifndef CMD_FLASH_BOOTLOADER_H
#define CMD_FLASH_BOOTLOADER_H

/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Default path to the STM32_Programmer_CLI executable. */
#define FLASH_BOOTLOADER_DEFAULT_PROGRAMMER_CLI   "STM32_Programmer_CLI.exe"
/** \brief Default path to the BOOT cmake project, relative to the current directory. */
#define FLASH_BOOTLOADER_DEFAULT_PROJECT_PATH     "../Target"
/** \brief Prefix used to turn a board name into a cmake configure/build preset name. */
#define FLASH_BOOTLOADER_PRESET_PREFIX            "release-"

/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Settings, as parsed from the command line, for the flash-bootloader
 *         subcommand.
 */
typedef struct t_flash_bootloader_settings
{
  char const * boardName;          /**< Name of the board, e.g. "nucleo-f429zi".      */
  char const * programmerCliPath;  /**< Path to the STM32_Programmer_CLI executable.  */
  char const * bootProjectPath;    /**< Path to the BOOT cmake project.               */
} tFlashBootloaderSettings;

/****************************************************************************************
* Global data
****************************************************************************************/
/** \brief Global settings object for the flash-bootloader subcommand. Populated by
 *         CmdFlashBootloaderParse() and consumed by CmdFlashBootloaderRun(). Declared
 *         at file scope (not inline inside a function) so it can be read, returned,
 *         and passed around by any caller.
 */
extern tFlashBootloaderSettings g_flashBootloaderSettings;

/****************************************************************************************
* Function prototypes
****************************************************************************************/
void CmdFlashBootloaderPrintUsage(void);
int  CmdFlashBootloaderParse(int argc, char * const argv[]);
int  CmdFlashBootloaderRun(tFlashBootloaderSettings settings);

#endif /* CMD_FLASH_BOOTLOADER_H */
/*************************** end of cmd_flash_bootloader.h ******************************/