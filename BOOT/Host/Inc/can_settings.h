/************************************************************************************//**
* \file         can_settings.h
* \brief        Fixed constants for the XCP-on-CAN session used to flash a board.
****************************************************************************************/
#ifndef CAN_SETTINGS_H
#define CAN_SETTINGS_H

#include <CMR/can_ids.h>

/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Default CAN device name (Peak System PCAN-USB). */
#define CAN_DEFAULT_DEVICE                       "peak_pcanusb"
/** \brief Default zero based CAN channel index. */
#define CAN_DEFAULT_CHANNEL                      (0u)
/** \brief CAN bus baudrate in bits per second. */
#define CAN_BAUDRATE                             (500000u)
/** \brief CAN FD data phase baudrate in bits per second. 0 means CAN FD is disabled. */
#define CAN_FD_DATA_BAUDRATE                     (0u)
/** \brief CAN identifier used for transmitting XCP command messages, host to target. */
#define CAN_XCP_TRANSMIT_ID                      (CMR_CANID_BOOTLOADER_FLASH_TX)
/** \brief CAN identifier used for receiving XCP response messages, target to host. */
#define CAN_XCP_RECEIVE_ID                       (CMR_CANID_BOOTLOADER_FLASH_RX)
/** \brief Whether 29-bit extended CAN identifiers should be used. */
#define CAN_USE_EXTENDED_ID                      (false)

/* XCP protocol timeouts, in milliseconds, matching LibOpenBLT's defaults. */
#define XCP_TIMEOUT_T1_MS                        (1000u)
#define XCP_TIMEOUT_T3_MS                        (2000u)
#define XCP_TIMEOUT_T4_MS                        (10000u)
#define XCP_TIMEOUT_T5_MS                        (1000u)
#define XCP_TIMEOUT_T6_MS                        (50u)
#define XCP_TIMEOUT_T7_MS                        (2000u)

#endif /* CAN_SETTINGS_H */
/******************************** end of can_settings.h **********************************/
