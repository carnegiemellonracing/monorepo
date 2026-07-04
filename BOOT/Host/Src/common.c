/************************************************************************************//**
* \file         common.c
* \brief        Shared helper function implementations.
****************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "common.h"


/************************************************************************************//**
** \brief     Outputs the program banner. Shared by both subcommands.
****************************************************************************************/
void CommonDisplayProgramInfo(void)
{
  printf("--------------------------------------------------------------------------\n");
  printf("blt-flash version 1.0.0. Builds and flashes an OpenBLT bootloader and/or\n");
  printf("flashes application firmware onto it via CAN.\n");
  printf("--------------------------------------------------------------------------\n");
} /*** end of CommonDisplayProgramInfo ***/


/************************************************************************************//**
** \brief     Obtains the [OK]/[ERROR]/[ABORT]/[SKIP] trailer string for the given
**            result value.
** \param     trailerResult The type of result controlling what to output in the trailer.
** \return    Pointer to the character array (string) with the trailer.
****************************************************************************************/
char const * CommonGetTrailerByResult(tTrailerResult trailerResult)
{
  char const * result;
  static char const * trailerStrOk    = "[" OUTPUT_GREEN "OK" OUTPUT_RESET "]";
  static char const * trailerStrError = "[" OUTPUT_RED "ERROR" OUTPUT_RESET "]";
  static char const * trailerStrAbort = "[" OUTPUT_RED "ABORT" OUTPUT_RESET "]";
  static char const * trailerStrSkip  = "[" OUTPUT_YELLOW "SKIP" OUTPUT_RESET "]";

  switch (trailerResult)
  {
  case TRAILER_RESULT_OK:
    result = trailerStrOk;
    break;
  case TRAILER_RESULT_ABORT:
    result = trailerStrAbort;
    break;
  case TRAILER_RESULT_SKIP:
    result = trailerStrSkip;
    break;
  case TRAILER_RESULT_ERROR:
  default:
    result = trailerStrError;
    break;
  }
  return result;
} /*** end of CommonGetTrailerByResult ***/


/************************************************************************************//**
** \brief     Obtains a "[xxx%]" trailer string for the given percentage value.
** \param     percentage Percentage value (0..100) to embed in the trailer.
** \return    Pointer to the character array (string) with the trailer.
****************************************************************************************/
char const * CommonGetTrailerByPercentage(uint8_t percentage)
{
  static char trailerStrPct[32] = "";

  sprintf(trailerStrPct, "[" OUTPUT_YELLOW "%3hhu%%" OUTPUT_RESET "]", percentage);
  return &trailerStrPct[0];
} /*** end of CommonGetTrailerByPercentage ***/


/************************************************************************************//**
** \brief     Erases a previously written percentage trailer from stdout by writing
**            backspace characters.
****************************************************************************************/
void CommonErasePercentageTrailer(void)
{
  uint32_t backspaceCnt;
  uint32_t const trailerLen = (uint32_t)strlen("[100%]");

  for (backspaceCnt = 0; backspaceCnt < trailerLen; backspaceCnt++)
  {
    (void)putchar('\b');
    (void)putchar(' ');
    (void)putchar('\b');
  }
} /*** end of CommonErasePercentageTrailer ***/


/************************************************************************************//**
** \brief     Runs an external command through the shell and streams its output.
** \param     cmd The command line to execute.
** \return    RESULT_OK if the command executed and returned exit code 0, error code
**            otherwise.
****************************************************************************************/
int CommonRunCommand(char const * cmd)
{
  int result = RESULT_OK;
  int sysResult;

  printf("  $ %s\n", cmd);
  (void)fflush(stdout);
  sysResult = system(cmd);
  if (sysResult != 0)
  {
    result = RESULT_ERROR_BUILD;
  }
  return result;
} /*** end of CommonRunCommand ***/

/*********************************** end of common.c *************************************/