/************************************************************************************//**
* \file         cmd_flash_bootloader.c
* \brief        flash-bootloader subcommand source file.
*
* \details      This subcommand builds the bootloader ("BOOT") cmake project for a
*               specific board using a cmake preset named "release-<board name>" and
*               subsequently flashes the resulting firmware image onto the target using
*               ST's STM32_Programmer_CLI tool over SWD.
*
*               This assumes:
*                 - The BOOT cmake project's CMakePresets.json defines a configure and
*                   build preset named "release-<board name>".
*                 - The build preset's binary directory ends up containing a file
*                   named "BOOT.hex" that STM32_Programmer_CLI can write to the target.
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "common.h"
#include "cmd_flash_bootloader.h"


/****************************************************************************************
* Global data
****************************************************************************************/
/** \brief Global settings object for the flash-bootloader subcommand. See the extern
 *         declaration in cmd_flash_bootloader.h for details.
 */
tFlashBootloaderSettings g_flashBootloaderSettings;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static int BuildBootProject(tFlashBootloaderSettings settings,
                             char * presetNameOut, size_t presetNameOutLen);
static int FlashBootProject(tFlashBootloaderSettings settings, char const * presetName);


/************************************************************************************//**
** \brief     Outputs usage information for the flash-bootloader subcommand.
****************************************************************************************/
void CmdFlashBootloaderPrintUsage(void)
{
  printf("Usage:    blt-flash flash-bootloader [options]\n");
  printf("\n");
  printf("Configures, builds and flashes the OpenBLT bootloader (BOOT cmake project)\n");
  printf("for the specified board, using STM32_Programmer_CLI to flash it over SWD.\n");
  printf("\n");
  printf("Options:\n");
  printf("  -b, --board=[name]        Name of the board to build the bootloader for.\n");
  printf("                            Selects the \"%s[name]\" cmake preset.\n",
         FLASH_BOOTLOADER_PRESET_PREFIX);
  printf("                            (Mandatory).\n");
  printf("  -p, --programmer=[path]   Path to the STM32_Programmer_CLI executable.\n");
  printf("                            (Default = %s).\n",
         FLASH_BOOTLOADER_DEFAULT_PROGRAMMER_CLI);
  printf("  -d, --project-dir=[path]  Path to the BOOT cmake project.\n");
  printf("                            (Default = %s).\n",
         FLASH_BOOTLOADER_DEFAULT_PROJECT_PATH);
  printf("  -h, --help                Displays this usage information.\n");
} /*** end of CmdFlashBootloaderPrintUsage ***/


/************************************************************************************//**
** \brief     Parses the command line arguments for the flash-bootloader subcommand.
** \param     argc Number of subcommand arguments (excluding the subcommand itself).
** \param     argv Array with subcommand argument strings (argv[0] is the first real
**            option, not the program name).
** \param     settings Pointer to the settings structure to populate.
** \return    RESULT_OK if all mandatory settings were successfully parsed, error code
**            otherwise.
****************************************************************************************/
int CmdFlashBootloaderParse(int argc, char * const argv[])
{
  int result = RESULT_OK;
  int opt;
  int longIndex = 0;

  static struct option longOptions[] =
  {
    { "board",       required_argument, NULL, 'b' },
    { "programmer",  required_argument, NULL, 'p' },
    { "project-dir", required_argument, NULL, 'd' },
    { "help",        no_argument,       NULL, 'h' },
    { NULL,          0,                 NULL,  0  }
  };

  /* Set defaults directly on the global settings object. */
  g_flashBootloaderSettings.boardName = NULL;
  g_flashBootloaderSettings.programmerCliPath = FLASH_BOOTLOADER_DEFAULT_PROGRAMMER_CLI;
  g_flashBootloaderSettings.bootProjectPath = FLASH_BOOTLOADER_DEFAULT_PROJECT_PATH;

  /* Reset getopt's global state so it can be reused across subcommands. */
  optind = 1;
  while ((opt = getopt_long(argc, argv, "b:p:d:h", longOptions, &longIndex)) != -1)
  {
    switch (opt)
    {
    case 'b':
      g_flashBootloaderSettings.boardName = optarg;
      break;
    case 'p':
      g_flashBootloaderSettings.programmerCliPath = optarg;
      break;
    case 'd':
      g_flashBootloaderSettings.bootProjectPath = optarg;
      break;
    case 'h':
      CmdFlashBootloaderPrintUsage();
      result = RESULT_ERROR_COMMANDLINE;
      break;
    default:
      result = RESULT_ERROR_COMMANDLINE;
      break;
    }
  }

  /* The board name is mandatory. */
  if ((result == RESULT_OK) && (g_flashBootloaderSettings.boardName == NULL))
  {
    printf("Error: a board name must be specified with -b/--board.\n\n");
    CmdFlashBootloaderPrintUsage();
    result = RESULT_ERROR_COMMANDLINE;
  }

  return result;
} /*** end of CmdFlashBootloaderParse ***/


/************************************************************************************//**
** \brief     Runs the flash-bootloader subcommand: configures and builds the BOOT
**            cmake project for the specified board, then flashes it with
**            STM32_Programmer_CLI.
** \param     settings Pointer to the previously parsed settings.
** \return    RESULT_OK on success, error code otherwise.
****************************************************************************************/
int CmdFlashBootloaderRun(tFlashBootloaderSettings settings)
{
  int result;
  char presetName[128];

  CommonDisplayProgramInfo();
  printf("Building bootloader for board: %s\n", settings.boardName);

  result = BuildBootProject(settings, presetName, sizeof(presetName));

  if (result == RESULT_OK)
  {
    result = FlashBootProject(settings, presetName);
  }

  return result;
} /*** end of CmdFlashBootloaderRun ***/


/************************************************************************************//**
** \brief     Configures and builds the BOOT cmake project using the
**            "release-<board name>" preset.
** \param     settings Pointer to the previously parsed settings.
** \param     presetNameOut Buffer that receives the resolved preset name.
** \param     presetNameOutLen Size of the presetNameOut buffer.
** \return    RESULT_OK on success, RESULT_ERROR_BUILD otherwise.
****************************************************************************************/
static int BuildBootProject(tFlashBootloaderSettings settings,
                             char * presetNameOut, size_t presetNameOutLen)
{
  int result = RESULT_OK;
  char cmd[512];

  snprintf(presetNameOut, presetNameOutLen, "%s%s", FLASH_BOOTLOADER_PRESET_PREFIX,
           settings.boardName);

  printf("Configuring cmake preset '%s'...\n", presetNameOut);
  snprintf(cmd, sizeof(cmd), "cmake -S \"%s\" --preset %s", settings.bootProjectPath,
           presetNameOut);
  result = CommonRunCommand(cmd);

  if (result == RESULT_OK)
  {
    printf("Building cmake preset '%s'...\n", presetNameOut);
    snprintf(cmd, sizeof(cmd), "cmake --build --preset %s", presetNameOut);
    result = CommonRunCommand(cmd);
  }

  if (result != RESULT_OK)
  {
    printf("Error: building the bootloader failed.\n");
  }

  return result;
} /*** end of BuildBootProject ***/


/************************************************************************************//**
** \brief     Flashes the just-built bootloader image onto the target using
**            STM32_Programmer_CLI over SWD.
** \param     settings Pointer to the previously parsed settings.
** \param     presetName Name of the cmake preset that was built.
** \return    RESULT_OK on success, RESULT_ERROR_FLASH_BOOTLOADER otherwise.
****************************************************************************************/
static int FlashBootProject(tFlashBootloaderSettings settings, char const * presetName)
{
  int result;
  char cmd[512];

  /* Note: this assumes the build preset's binary directory follows the convention
   * "<project>/build/<preset name>/BOOT.hex". Adjust to match your CMakePresets.json
   * if your binary directory layout differs.
   */
  printf("Flashing bootloader with STM32_Programmer_CLI...\n");
  snprintf(cmd, sizeof(cmd),
           "\"%s\" -c port=SWD -w \"%s/build/%s/BOOT.hex\" -v -rst",
           settings.programmerCliPath, settings.bootProjectPath, presetName);
  result = CommonRunCommand(cmd);

  if (result != RESULT_OK)
  {
    result = RESULT_ERROR_FLASH_BOOTLOADER;
    printf("Error: flashing the bootloader failed.\n");
  }
  else
  {
    printf("Bootloader flashed successfully.\n");
  }

  return result;
} /*** end of FlashBootProject ***/

/*************************** end of cmd_flash_bootloader.c ******************************/