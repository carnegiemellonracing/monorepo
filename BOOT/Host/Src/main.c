/************************************************************************************//**
* \file         main.c
* \brief        blt-flash program entry point. Dispatches to one of two subcommands:
*                 - flash-bootloader : builds and flashes the OpenBLT bootloader itself.
*                 - flash-board      : flashes application firmware via the bootloader.
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "cmd_flash_bootloader.h"
#include "cmd_flash_board.h"


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void PrintTopLevelUsage(void);


/************************************************************************************//**
** \brief     Program entry point.
** \param     argc Number of program arguments.
** \param     argv Array with program arguments. argv[1], if present, is the subcommand
**            name; everything from argv[2] onwards belongs to the subcommand.
** \return    Program return code. 0 for success, error code otherwise.
****************************************************************************************/
int main(int argc, char * argv[])
{
  int result = RESULT_OK;

  if (argc < 2)
  {
    PrintTopLevelUsage();
    return RESULT_ERROR_COMMANDLINE;
  }

  /* The subcommand's own argument vector starts at the subcommand name itself, so that
   * getopt inside each subcommand's parser sees a conventional argv[0].
   */
  int subArgc = argc - 1;
  char * const * subArgv = &argv[1];

  if (strcmp(argv[1], "flash-bootloader") == 0)
  {
    result = CmdFlashBootloaderParse(subArgc, subArgv);
    if (result == RESULT_OK)
    {
      result = CmdFlashBootloaderRun(g_flashBootloaderSettings);
    }
  }
  else if (strcmp(argv[1], "flash-board") == 0)
  {
    result = CmdFlashBoardParse(subArgc, subArgv);
    if (result == RESULT_OK)
    {
      result = CmdFlashBoardRun(g_flashBoardSettings);
    }
  }
  else if ((strcmp(argv[1], "-h") == 0) || (strcmp(argv[1], "--help") == 0))
  {
    PrintTopLevelUsage();
    result = RESULT_ERROR_COMMANDLINE;
  }
  else
  {
    printf("Error: unknown command '%s'.\n\n", argv[1]);
    PrintTopLevelUsage();
    result = RESULT_ERROR_COMMANDLINE;
  }

  return result;
} /*** end of main ***/


/************************************************************************************//**
** \brief     Outputs top level usage information, listing the available subcommands.
****************************************************************************************/
static void PrintTopLevelUsage(void)
{
  printf("Usage:    blt-flash <command> [options]\n");
  printf("\n");
  printf("Commands:\n");
  printf("  flash-bootloader   Build and flash the OpenBLT bootloader itself.\n");
  printf("  flash-board        Flash application firmware via the bootloader (CAN).\n");
  printf("\n");
  printf("Run 'blt-flash <command> --help' for command specific options.\n");
} /*** end of PrintTopLevelUsage ***/

/*********************************** end of main.c *************************************/