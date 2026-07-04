/************************************************************************************//**
* \file         common.h
* \brief        Shared types, constants and helper functions used by both the
*               flash-bootloader and flash-board subcommands.
****************************************************************************************/
#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

/****************************************************************************************
* getopt_long compatibility shim
*
* MSVC (and its CRT) does not ship <getopt.h> / getopt_long() -- that's a POSIX/GNU
* extension. On MSVC we provide our own minimal, source-compatible implementation
* (see common.c). On any other compiler (gcc/clang/MinGW) we just use the real thing.
****************************************************************************************/
#ifdef _MSC_VER

#define no_argument                              (0)
#define required_argument                        (1)
#define optional_argument                         (2)

struct option
{
  char const * name;     /**< Long option name, without the leading "--".         */
  int          has_arg;  /**< One of no_argument/required_argument/optional_argument. */
  int        * flag;     /**< Unused by this shim, always pass NULL.              */
  int          val;      /**< Value returned by getopt_long() on a match.         */
};

extern char * optarg;
extern int    optind;
extern int    opterr;
extern int    optopt;

int getopt_long(int argc, char * const argv[], char const * optstring,
                 struct option const * longopts, int * longindex);

#else
#include <getopt.h>
#endif

/****************************************************************************************
* Program return codes
****************************************************************************************/
#define RESULT_OK                                (0)
#define RESULT_ERROR_COMMANDLINE                 (1)
#define RESULT_ERROR_BUILD                       (2)
#define RESULT_ERROR_FLASH_BOOTLOADER             (3)
#define RESULT_ERROR_FIRMWARE_LOAD               (4)
#define RESULT_ERROR_SESSION_START                (5)
#define RESULT_ERROR_INFO_TABLE_CHECK_FAILED     (6)
#define RESULT_ERROR_INFO_TABLE_CHECK_ERROR      (7)
#define RESULT_ERROR_MEMORY_ERASE                (8)
#define RESULT_ERROR_MEMORY_PROGRAM               (9)

/****************************************************************************************
* Terminal color macros (only enabled on Linux, like the original tool)
****************************************************************************************/
#if defined (PLATFORM_LINUX)
#define OUTPUT_RESET                             "\033[0m"
#define OUTPUT_RED                               "\033[31m"
#define OUTPUT_GREEN                              "\033[32m"
#define OUTPUT_YELLOW                             "\033[33m"
#else
#define OUTPUT_RESET                             ""
#define OUTPUT_RED                                ""
#define OUTPUT_GREEN                              ""
#define OUTPUT_YELLOW                             ""
#endif

/** \brief Enumerated type for the trailer results shown at the end of a status line. */
typedef enum
{
  TRAILER_RESULT_OK,        /**< Shows [OK] in the trailer.    */
  TRAILER_RESULT_ERROR,     /**< Shows [ERROR] in the trailer. */
  TRAILER_RESULT_ABORT,     /**< Shows [ABORT] in the trailer. */
  TRAILER_RESULT_SKIP       /**< Shows [SKIP] in the trailer.  */
} tTrailerResult;

/****************************************************************************************
* Function prototypes
****************************************************************************************/
char const * CommonGetTrailerByResult(tTrailerResult trailerResult);
char const * CommonGetTrailerByPercentage(uint8_t percentage);
void         CommonErasePercentageTrailer(void);
int          CommonRunCommand(char const * cmd);
void         CommonDisplayProgramInfo(void);

#endif /* COMMON_H */
/*********************************** end of common.h ************************************/