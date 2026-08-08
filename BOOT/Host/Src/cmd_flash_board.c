/************************************************************************************//**
* \file         cmd_flash_board.c
* \brief        flash-board subcommand source file.
*
* \details      This subcommand flashes a firmware file onto a target running the
*               OpenBLT bootloader, communicating exclusively over CAN using the XCP
*               version 1.0 protocol (LibOpenBLT's BLT_TRANSPORT_XCP_V10_CAN transport).
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "openblt.h"
#include "candriver.h"
#include "common.h"
#include "cmd_flash_board.h"


/****************************************************************************************
* Global data
****************************************************************************************/
/** \brief Global settings object for the flash-board subcommand. See the extern
 *         declaration in cmd_flash_board.h for details.
 */
tFlashBoardSettings g_flashBoardSettings;

/** \brief Global XCP version 1.0 session settings, built once from fixed defaults and
 *         used for the duration of the flash-board run. Declared at file scope (not
 *         inline inside a function) so it can be built, returned, and passed around.
 */
static tBltSessionSettingsXcpV10 g_sessionSettings;

/** \brief Global XCP on CAN transport settings, built once from g_flashBoardSettings
 *         and the fixed CAN_xxx constants. Declared at file scope for the same reason
 *         as g_sessionSettings above.
 */
static tBltTransportSettingsXcpV10Can g_transportSettings;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static int  LoadFirmware(char const * firmwareFile);
static tBltSessionSettingsXcpV10 BuildSessionSettings(uint8_t board_id);
static tBltTransportSettingsXcpV10Can BuildTransportSettings(tFlashBoardSettings settings);
static int  ResetTarget(void);
static int  ConnectToTarget(void);
static int  CheckInfoTable(void);
static int  EraseSegments(void);
static int  ProgramSegments(void);
static int  StopSession(void);


/************************************************************************************//**
** \brief     Outputs usage information for the flash-board subcommand.
****************************************************************************************/
void CmdFlashBoardPrintUsage(void)
{
  printf("Usage:    blt-flash flash-board [options]\n");
  printf("\n");
  printf("Flashes a firmware file onto a target running the OpenBLT bootloader over\n");
  printf("CAN, using the XCP version 1.0 protocol.\n");
  printf("\n");
  printf("Options:\n");
  printf("  -f, --file=[path]      Path to the firmware file to flash (Mandatory).\n");
  printf("  -d, --device=[name]    Name of the CAN device to use, e.g. peak_pcanusb\n");
  printf("                         or can0 (Default = %s).\n", CAN_DEFAULT_DEVICE);
  printf("  -i, --board_id=[value] ID of the board to flash\n");
  printf("  -c, --channel=[value]  Zero based index of the CAN channel to use, for\n");
  printf("                         adapters with multiple channels (Default = %u).\n",
         (unsigned int)CAN_DEFAULT_CHANNEL);
  printf("  -h, --help             Displays this usage information.\n");
} /*** end of CmdFlashBoardPrintUsage ***/


/************************************************************************************//**
** \brief     Parses the command line arguments for the flash-board subcommand.
** \param     argc Number of subcommand arguments (excluding the subcommand itself).
** \param     argv Array with subcommand argument strings (argv[0] is the first real
**            option, not the program name).
** \param     settings Pointer to the settings structure to populate.
** \return    RESULT_OK if all mandatory settings were successfully parsed, error code
**            otherwise.
****************************************************************************************/
int CmdFlashBoardParse(int argc, char * const argv[])
{
  int result = RESULT_OK;
  int opt;
  int longIndex = 0;

  static struct option longOptions[] =
  {
    { "file",    required_argument, NULL, 'f' },
    { "device",  required_argument, NULL, 'd' },
    { "channel", required_argument, NULL, 'c' },
    { "board_id", required_argument, NULL, 'i' },
    { "help",    no_argument,       NULL, 'h' },
    { NULL,      0,                 NULL,  0  }
  };

  /* Set defaults directly on the global settings object. */
  g_flashBoardSettings.firmwareFile = NULL;
  g_flashBoardSettings.canDevice = CAN_DEFAULT_DEVICE;
  g_flashBoardSettings.canChannel = CAN_DEFAULT_CHANNEL;

  /* Reset getopt's global state so it can be reused across subcommands. */
  optind = 1;
  while ((opt = getopt_long(argc, argv, "f:d:c:h:i", longOptions, &longIndex)) != -1)
  {
    switch (opt)
    {
    case 'f':
      g_flashBoardSettings.firmwareFile = optarg;
      break;
    case 'd':
      g_flashBoardSettings.canDevice = optarg;
      break;
    case 'c':
      g_flashBoardSettings.canChannel = (uint32_t)strtoul(optarg, NULL, 10);
      break;   
    case 'i':
      g_flashBoardSettings.board_id = (uint8_t)strtoul(optarg, NULL, 10);
      break;
    case 'h':
      CmdFlashBoardPrintUsage();
      result = RESULT_ERROR_COMMANDLINE;
      break;
    default:
      result = RESULT_ERROR_COMMANDLINE;
      break;
    }
  }

  /* The firmware file is mandatory. */
  if ((result == RESULT_OK) && (g_flashBoardSettings.firmwareFile == NULL))
  {
    printf("Error: a firmware file must be specified with -f/--file.\n\n");
    CmdFlashBoardPrintUsage();
    result = RESULT_ERROR_COMMANDLINE;
  }

  return result;
} /*** end of CmdFlashBoardParse ***/


/************************************************************************************//**
** \brief     Runs the flash-board subcommand: loads the firmware file, connects to the
**            target over CAN, erases and programs the relevant memory ranges, then
**            disconnects.
** \param     settings Pointer to the previously parsed settings.
** \return    RESULT_OK on success, error code otherwise.
****************************************************************************************/
int CmdFlashBoardRun()
{
  int result;

  CommonDisplayProgramInfo();
  printf("Firmware file:  %s\n", g_flashBoardSettings.firmwareFile);
  printf("CAN device:     %s (channel %u)\n", g_flashBoardSettings.canDevice,
         (unsigned int)g_flashBoardSettings.canChannel);
  printf("CAN baudrate:   %u bit/sec\n\n", (unsigned int)CAN_BAUDRATE);

  result = LoadFirmware(g_flashBoardSettings.firmwareFile);

  if (result == RESULT_OK)
  {
    g_sessionSettings = BuildSessionSettings(g_flashBoardSettings.board_id);
    g_transportSettings = BuildTransportSettings(g_flashBoardSettings);

    result = ResetTarget();
    if( result == RESULT_OK)
    {
      BltUtilTimeDelayMs(10);
      result = ConnectToTarget();
    }
  }

  if (result == RESULT_OK)
  {
    result = CheckInfoTable();
  }

  if (result == RESULT_OK)
  {
    result = EraseSegments();
  }

  if (result == RESULT_OK)
  {
    result = ProgramSegments();
  }

  if (result == RESULT_OK)
  {
    result = StopSession();
  }

  /* Always clean up, regardless of where an error might have occurred. */
  BltSessionTerminate();
  BltFirmwareTerminate();

  if (result == RESULT_OK)
  {
    printf("\nFirmware flashed successfully.\n");
  }

  return result;
} /*** end of CmdFlashBoardRun ***/


/************************************************************************************//**
** \brief     Loads the firmware data from the specified file using the S-record
**            parser.
** \param     firmwareFile Path to the firmware file.
** \return    RESULT_OK on success, RESULT_ERROR_FIRMWARE_LOAD otherwise.
****************************************************************************************/
static int LoadFirmware(char const * firmwareFile)
{
  int result = RESULT_OK;

  printf("Loading firmware data from file..."); (void)fflush(stdout);
  BltFirmwareInit(BLT_FIRMWARE_PARSER_SRECORD);
  if (BltFirmwareLoadFromFile(firmwareFile, 0) != BLT_RESULT_OK)
  {
    result = RESULT_ERROR_FIRMWARE_LOAD;
  }
  if ((result == RESULT_OK) && (BltFirmwareGetSegmentCount() == 0))
  {
    result = RESULT_ERROR_FIRMWARE_LOAD;
  }
  printf("%s\n", CommonGetTrailerByResult(
    (result == RESULT_OK) ? TRAILER_RESULT_OK : TRAILER_RESULT_ERROR));

  return result;
} /*** end of LoadFirmware ***/


/************************************************************************************//**
** \brief     Builds the XCP version 1.0 session settings from fixed defaults.
** \param     board_id The ID of the board for which to build the settings.
** \return    The populated session settings structure.
****************************************************************************************/
static tBltSessionSettingsXcpV10 BuildSessionSettings(uint8_t board_id)
{
  tBltSessionSettingsXcpV10 sessionSettings;

  sessionSettings.timeoutT1 = XCP_TIMEOUT_T1_MS;
  sessionSettings.timeoutT3 = XCP_TIMEOUT_T3_MS;
  sessionSettings.timeoutT4 = XCP_TIMEOUT_T4_MS;
  sessionSettings.timeoutT5 = XCP_TIMEOUT_T5_MS;
  sessionSettings.timeoutT6 = XCP_TIMEOUT_T6_MS;
  sessionSettings.timeoutT7 = XCP_TIMEOUT_T7_MS;
  sessionSettings.seedKeyFile = NULL;
  sessionSettings.connectMode = board_id;

  return sessionSettings;
} /*** end of BuildSessionSettings ***/


/************************************************************************************//**
** \brief     Populates the XCP on CAN transport settings, using the CAN device and
**            channel from the command line and the fixed CAN identifier/baudrate
**            constants.
** \param     settings The previously parsed command line settings.
** \return    The populated transport settings structure.
****************************************************************************************/
static tBltTransportSettingsXcpV10Can BuildTransportSettings(tFlashBoardSettings settings)
{
  tBltTransportSettingsXcpV10Can transportSettings;

  transportSettings.deviceName = settings.canDevice;
  transportSettings.deviceChannel = settings.canChannel;
  transportSettings.baudrate = CAN_BAUDRATE;
  transportSettings.transmitId = CAN_XCP_TRANSMIT_ID;
  transportSettings.receiveId = CAN_XCP_RECEIVE_ID;
  transportSettings.useExtended = CAN_USE_EXTENDED_ID;
  transportSettings.brsBaudrate = CAN_FD_DATA_BAUDRATE;

  return transportSettings;
} /*** end of BuildTransportSettings ***/


static int ResetTarget(void)
{
  tCanSettings canSettings;

  canSettings.devicename = g_transportSettings.deviceName;
  canSettings.channel = g_transportSettings.deviceChannel;
  switch (g_transportSettings.baudrate)
  {
    case 1000000:
      canSettings.baudrate = CAN_BR1M;
      break;
    case 800000:
      canSettings.baudrate = CAN_BR800K;
      break;
    case 500000:
      canSettings.baudrate = CAN_BR500K;
      break;
    case 250000:
      canSettings.baudrate = CAN_BR250K;
      break;
    case 125000:
      canSettings.baudrate = CAN_BR125K;
      break;
    case 100000:
      canSettings.baudrate = CAN_BR100K;
      break;
    case 50000:
      canSettings.baudrate = CAN_BR50K;
      break;
    case 20000:
      canSettings.baudrate = CAN_BR20K;
      break;
    case 10000:
      canSettings.baudrate = CAN_BR10K;
      break;
    default:
      /* Default to 500 kbits/sec in case an unsupported baudrate was specified. */
      canSettings.baudrate = CAN_BR500K;
      break;
  }
  /* Configure the reception acceptance filter to receive only one CAN identifier. */
  canSettings.code = g_transportSettings.receiveId;
  if (g_transportSettings.useExtended)
  {
    canSettings.code |= CAN_MSG_EXT_ID_MASK;
  }
  canSettings.mask = 0x9fffffff;
  switch (g_transportSettings.brsBaudrate)
  {
  case 8000000:
    canSettings.brsbaudrate = CANFD_BR8M;
    break;
  case 5000000:
    canSettings.brsbaudrate = CANFD_BR5M;
    break;
  case 4000000:
    canSettings.brsbaudrate = CANFD_BR4M;
    break;
  case 2000000:
    canSettings.brsbaudrate = CANFD_BR2M;
    break;
  case 1000000:
    canSettings.brsbaudrate = CANFD_BR1M;
    break;
  case 800000:
    canSettings.brsbaudrate = CANFD_BR800K;
    break;
  case 500000:
    canSettings.brsbaudrate = CANFD_BR500K;
    break;
  case 250000:
    canSettings.brsbaudrate = CANFD_BR250K;
    break;
  case 125000:
    canSettings.brsbaudrate = CANFD_BR125K;
    break;
  case 100000:
    canSettings.brsbaudrate = CANFD_BR100K;
    break;
  case 50000:
    canSettings.brsbaudrate = CANFD_BR50K;
    break;
  case 20000:
    canSettings.brsbaudrate = CANFD_BR20K;
    break;
  case 10000:
    canSettings.brsbaudrate = CANFD_BR10K;
    break;
  default:
    /* Default to CAN FD unused in case an unsupported baudrate was specified. */
    canSettings.brsbaudrate = CANFD_DISABLED;
    break;
  }

  /* Initialize the CAN driver. */
  CanInit(&canSettings);
  bool result = CanConnect();
  if (!result)
  {
    printf("%s\n", CommonGetTrailerByResult(TRAILER_RESULT_ERROR));
    return RESULT_ERROR_SESSION_START;
  }

  printf("Resetting target..."); (void)fflush(stdout);

  tCanMsg msg = {
    .id = CMR_CANID_BOOTLOADER_FLASH_READY,
    .len = 1,
    .data = {g_flashBoardSettings.board_id},
  };
  result = CanTransmit(&msg);

  printf("%s\n", CommonGetTrailerByResult(
    (result) ? TRAILER_RESULT_OK : TRAILER_RESULT_ERROR));


  CanDisconnect();
  return RESULT_OK;
} /*** end of ResetTarget ***/

/************************************************************************************//**
** \brief     Initializes and starts an XCP on CAN session with the target, falling
**            back to backdoor entry retries if the target does not respond right away.
** \details   Reads from g_sessionSettings and g_transportSettings, which the caller is
**            expected to have populated first (see CmdFlashBoardRun()).
** \return    RESULT_OK on success, RESULT_ERROR_SESSION_START otherwise.
****************************************************************************************/
static int ConnectToTarget(void)
{
  int result = RESULT_OK;

  printf("Connecting to target bootloader..."); (void)fflush(stdout);
  BltSessionInit(BLT_SESSION_XCP_V10, &g_sessionSettings, BLT_TRANSPORT_XCP_V10_CAN,
                 &g_transportSettings);
  if (BltSessionStart() != BLT_RESULT_OK)
  {
    printf("[" OUTPUT_YELLOW "TIMEOUT" OUTPUT_RESET "]\n");
    (void)fflush(stdout);
    while (BltSessionStart() != BLT_RESULT_OK)
    {
      BltUtilTimeDelayMs(20);
    }
  }
  printf("%s\n", CommonGetTrailerByResult(TRAILER_RESULT_OK));

  return result;
} /*** end of ConnectToTarget ***/


/************************************************************************************//**
** \brief     Performs the bootloader's info table check, if supported.
** \return    RESULT_OK if the check passed or is not supported, error code otherwise.
****************************************************************************************/
static int CheckInfoTable(void)
{
  int result = RESULT_OK;
  uint32_t infoTableCheckResult;
  tTrailerResult trailerResult;

  printf("Performing info table check..."); (void)fflush(stdout);
  infoTableCheckResult = BltSessionCheckInfoTable();
  switch (infoTableCheckResult)
  {
  case BLT_RESULT_OK:
    trailerResult = TRAILER_RESULT_OK;
    break;
  case BLT_RESULT_ERROR_SESSION_INFO_TABLE:
    trailerResult = TRAILER_RESULT_ABORT;
    result = RESULT_ERROR_INFO_TABLE_CHECK_FAILED;
    break;
  case BLT_RESULT_ERROR_SESSION_INFO_TABLE_NOT_SUPPORTED:
    trailerResult = TRAILER_RESULT_SKIP;
    break;
  default:
    trailerResult = TRAILER_RESULT_ERROR;
    result = RESULT_ERROR_INFO_TABLE_CHECK_ERROR;
    break;
  }
  printf("%s\n", CommonGetTrailerByResult(trailerResult));

  return result;
} /*** end of CheckInfoTable ***/


/************************************************************************************//**
** \brief     Erases all target memory segments covered by the loaded firmware data.
** \return    RESULT_OK on success, RESULT_ERROR_MEMORY_ERASE otherwise.
****************************************************************************************/
static int EraseSegments(void)
{
  int result = RESULT_OK;
  uint32_t segmentIdx;
  uint32_t segmentLen;
  uint32_t segmentBase;
  uint8_t const * segmentData;
  uint32_t const eraseChunkSize = 32768;

  for (segmentIdx = 0; segmentIdx < BltFirmwareGetSegmentCount(); segmentIdx++)
  {
    uint32_t currentEraseCnt;
    uint32_t currentEraseBase;
    uint32_t stillToEraseCnt;

    segmentData = BltFirmwareGetSegment(segmentIdx, &segmentBase, &segmentLen);
    assert((segmentData != NULL) && (segmentLen > 0));
    if ((segmentData == NULL) || (segmentLen == 0))
    {
      result = RESULT_ERROR_MEMORY_ERASE;
      break;
    }

    printf("Erasing %u bytes starting at 0x%08x...%s", segmentLen, segmentBase,
           CommonGetTrailerByPercentage(0));
    (void)fflush(stdout);

    stillToEraseCnt = segmentLen;
    currentEraseBase = segmentBase;
    while (stillToEraseCnt > 0)
    {
      uint8_t progressPct;

      currentEraseCnt = (stillToEraseCnt >= eraseChunkSize) ? eraseChunkSize :
                                                               stillToEraseCnt;
      if (BltSessionClearMemory(currentEraseBase, currentEraseCnt) != BLT_RESULT_OK)
      {
        result = RESULT_ERROR_MEMORY_ERASE;
        break;
      }
      currentEraseBase += currentEraseCnt;
      stillToEraseCnt -= currentEraseCnt;

      CommonErasePercentageTrailer();
      progressPct = (uint8_t)(((segmentLen - stillToEraseCnt) * 100ul) / segmentLen);
      printf("%s", CommonGetTrailerByPercentage(progressPct)); (void)fflush(stdout);
    }

    CommonErasePercentageTrailer();
    printf("%s\n", CommonGetTrailerByResult(
      (result == RESULT_OK) ? TRAILER_RESULT_OK : TRAILER_RESULT_ERROR));

    if (result != RESULT_OK)
    {
      break;
    }
  }

  return result;
} /*** end of EraseSegments ***/


/************************************************************************************//**
** \brief     Programs all target memory segments with the loaded firmware data.
** \return    RESULT_OK on success, RESULT_ERROR_MEMORY_PROGRAM otherwise.
****************************************************************************************/
static int ProgramSegments(void)
{
  int result = RESULT_OK;
  uint32_t segmentIdx;
  uint32_t segmentLen;
  uint32_t segmentBase;
  uint8_t const * segmentData;
  uint32_t const writeChunkSize = 256;

  for (segmentIdx = 0; segmentIdx < BltFirmwareGetSegmentCount(); segmentIdx++)
  {
    uint32_t currentWriteCnt;
    uint32_t currentWriteBase;
    uint8_t const * currentWriteDataPtr;
    uint32_t stillToWriteCnt;

    segmentData = BltFirmwareGetSegment(segmentIdx, &segmentBase, &segmentLen);
    assert((segmentData != NULL) && (segmentLen > 0));
    if ((segmentData == NULL) || (segmentLen == 0))
    {
      result = RESULT_ERROR_MEMORY_PROGRAM;
      break;
    }

    printf("Programming %u bytes starting at 0x%08x...%s", segmentLen, segmentBase,
           CommonGetTrailerByPercentage(0));
    (void)fflush(stdout);

    stillToWriteCnt = segmentLen;
    currentWriteBase = segmentBase;
    currentWriteDataPtr = segmentData;
    while (stillToWriteCnt > 0)
    {
      uint8_t progressPct;

      currentWriteCnt = (stillToWriteCnt >= writeChunkSize) ? writeChunkSize :
                                                               stillToWriteCnt;
      if (BltSessionWriteData(currentWriteBase, currentWriteCnt, currentWriteDataPtr)
          != BLT_RESULT_OK)
      {
        result = RESULT_ERROR_MEMORY_PROGRAM;
        break;
      }
      currentWriteBase += currentWriteCnt;
      currentWriteDataPtr += currentWriteCnt;
      stillToWriteCnt -= currentWriteCnt;

      CommonErasePercentageTrailer();
      progressPct = (uint8_t)(((segmentLen - stillToWriteCnt) * 100ul) / segmentLen);
      printf("%s", CommonGetTrailerByPercentage(progressPct)); (void)fflush(stdout);
    }

    CommonErasePercentageTrailer();
    printf("%s\n", CommonGetTrailerByResult(
      (result == RESULT_OK) ? TRAILER_RESULT_OK : TRAILER_RESULT_ERROR));

    if (result != RESULT_OK)
    {
      break;
    }
  }

  return result;
} /*** end of ProgramSegments ***/


/************************************************************************************//**
** \brief     Stops the active OpenBLT session, which also finalizes the programming
**            sequence on the target (e.g. triggers a reset into the application).
** \return    RESULT_OK. The stop operation itself has no failure path in LibOpenBLT.
****************************************************************************************/
static int StopSession(void)
{
  printf("Finishing programming session..."); (void)fflush(stdout);
  BltSessionStop();
  printf("%s\n", CommonGetTrailerByResult(TRAILER_RESULT_OK));

  return RESULT_OK;
} /*** end of StopSession ***/

/****************************** end of cmd_flash_board.c *********************************/