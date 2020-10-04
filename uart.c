/*
 * uart.c
 *
 *  Created on: Jul 26, 2020
 *      Author: vamsi
 */

#include <CMR/tasks.h>      // CMR task interface
#include <CMR/config.h>     // CMR configuration itnerface
#include <CMR/panic.h>      // bad things

#include "uart.h"       // Interface to implement
#include "crc.h"
#include "can.h"        // Can interface

//-----------------------------------------------------------------------------
// GLOBAL VARIABLE DEFINITIONS                                                |
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// STATIC VARIABLE DEFINITIONS                                                |
//-----------------------------------------------------------------------------

typedef struct uart uart_t;

struct uart {
    cmr_uart_t port;    /**< @brief The underlying UART port. */

    cmr_task_t rxTask;  /**< @brief Receive task. */
    cmr_task_t txTask;  /**< @brief Receive task. */
};

static uart_t uart;

//-----------------------------------------------------------------------------
// STATIC HELPER FUNCTION PROTOTYPES                                          |
//-----------------------------------------------------------------------------
static uint16_t uart_packCommand(const uart_command_t *command, Byte message[]);
static uint16_t uart_unpackResponse(uint8_t frameInitByte, uart_response_t *response);
static uart_result_t uart_getChar(volatile uart_t *uart, uint8_t *c);
static uart_result_t uart_sendMessage(volatile uart_t *uart, Byte message[], uint16_t messageLength);

//-----------------------------------------------------------------------------
// GLOBAL INTERFACE FUNCTIONS                                                 |
//-----------------------------------------------------------------------------
/** UART Init
 * This function will initialize the uart.
 */
void uartInit(void) {
    const UART_InitTypeDef uartInit = {
        .BaudRate = 250000,
		    .WordLength = UART_WORDLENGTH_8B,
		    .StopBits = UART_STOPBITS_1,
		    .Parity = UART_PARITY_NONE,
		    .HwFlowCtl = UART_HWCONTROL_NONE,
		    .Mode = UART_MODE_TX_RX,
		    .OverSampling = UART_OVERSAMPLING_16
    };

    cmr_uartInit(
        &uart.port, USART1, &uartInit,
        GPIOA, GPIO_PIN_10,     /* rx */ //needs changes
        GPIOA, GPIO_PIN_9,      /* tx */ //needs changes
        DMA2_Stream2, DMA_CHANNEL_4,    /* See reference manual */  //needs changes
        DMA2_Stream7, DMA_CHANNEL_4  //needs changes
    );

    cmr_taskInit(&uart.txTask, "UART TX", 8, uartTX_Task, NULL);
    cmr_taskInit(&uart.rxTask, "UART RX", 8, uartRX_Task, NULL);

    crcInit();

    return;
}

/** UART Send Command
 * This function will send the command stored in the input location specified
 * and will store the result (if there is one) of this transaction in the
 * input response array.
 * @param command A reference to the desired command
 * @param response The array containing the bytes we want to send
 * @param responseLength The location for where to store the result length
 * @return The status of the UART result (success or failure)
 */
uart_result_t uart_sendCommand(const uart_command_t *command) {
  
  // Open the command to pack the message and gather response information
  Byte message[MAX_COMMAND_LENGTH];
  uint16_t messageLength = uart_packCommand(command, message);
  
  // Send the message
  uart_result_t retv = uart_sendMessage(&uart, message, messageLength);
  
  return retv;
}

/** UART Receive Response
 * This helper function will receive a response and store the result
 * in the input uart_response_t object using the given USART
 * @param usart A pointer to the USART object we are receiving from
 * @param message The array to store the received message
 * @param messageLength The number of bytes to read
 * @return The status of the UART result (success or failure)
 */
uart_result_t uart_receiveResponse(uart_response_t *response) {

  uart_result_t retvTotal = UART_SUCCESS;
  uart_result_t retv = UART_SUCCESS;

  uint8_t receivedBytes[128] = {0};
  retv = uart_getChar(&uart, &receivedBytes[0]);
  if(retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  uint8_t frameInitByte = receivedBytes[0];

  uint16_t responseLength = uart_unpackResponse(frameInitByte, response);

  uint8_t c = 0;
  uint16_t receivedIndex = 0;
  while((receivedIndex < responseLength) && (retvTotal == UART_SUCCESS)) {
    
    retv = uart_getChar(&uart, &c);
    if(retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
    }
    
    receivedBytes[receivedIndex+1] = c;
    response->data[receivedIndex] = c;
    receivedIndex++;
  }

  uint8_t crcActual[2];
  retv = uart_getChar(&uart, &crcActual[0]);
  if(retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  retv = uart_getChar(&uart, &crcActual[1]);
  if(retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  // Check the CRC value
  crc16_t crcExpected = calculateCRC(receivedBytes, responseLength+1);
  if(crcExpected != ((crcActual[1]<<8) | crcActual[0])) {
    retvTotal = UART_FAILURE;
  }
  
  return retvTotal;
}

//-----------------------------------------------------------------------------
// STATIC HELPER FUNCTIONS                                                    |
//-----------------------------------------------------------------------------
/** UART Pack Command
 * This helper function will take an input command and pack it into a Byte
 * array that can be sent over UART. Additionally, it returns via reference
 * whether or not this input command expects a response based on the message.
 * @param command A reference to the desired command
 * @param message A buffer to hold the message generated by the given command
 * @param expectedResponse A reference to a variable stating whether we expect the slave to respond
 * @return The length of the message
 */
static uint16_t uart_packCommand(const uart_command_t *command, Byte message[]) {
  
  uint16_t messageLength = 0;
  
  // Get the frame initialization object
  frame_init_t frameInit = *(command->frameInit);

  // Get the sizes of the address/data to send
  uint8_t addressSize = 0;
  uint8_t registerAddress[MAX_REGISTER_ADDRESS_LENGTH];
  switch(frameInit.addressSize) {
    case REGISTER_ADDRESS_8_BIT:
      addressSize = 1;
      registerAddress[0] = (command->registerAddress) & 0x00FF;
      break;
    case REGISTER_ADDRESS_16_BIT:
      addressSize = 2;
      registerAddress[0] = (command->registerAddress) & 0xFF00;
      registerAddress[1] = (command->registerAddress) & 0x00FF;
      break;
    default:
      // ERROR CASE: We somehow did not get a valid address size
      break;
  }
  uint8_t dataSize = frameInit.dataSize;

  // Put the components of the initialization byte together
  uint8_t frameInitByte;
  switch(frameInit.frameType) {
    case COMMAND:
      frameInitByte = (frameInit.frameType   << 7) |
              (frameInit.requestType << 4) |
              (frameInit.addressSize << 3) |
              (frameInit.dataSize);
      break;
    case RESPONSE:
      // ERROR CASE: We should not be sending a response--only receiving
      break;
    default:
      // ERROR CASE: We somehow did not get a valid frame type
      break;
  }

  // Put the individual bytes into the message
  message[messageLength] = frameInitByte;
  ++messageLength;
  if( (frameInit.requestType == SINGLE_WRITE_WITH_RESPONSE   ) ||
    (frameInit.requestType == SINGLE_WRITE_WITHOUT_RESPONSE)) {
    message[messageLength] = command->deviceAddress;
    ++messageLength;
  }
  for(uint8_t i = 0; i < addressSize; ++i) {
    message[messageLength] = registerAddress[i];
    ++messageLength;
  }
  for(uint8_t i = 0; i < dataSize; ++i) {
    message[messageLength] = command->data[i];
    ++messageLength;
  }

  // Calculate the CRC and put at the end of the message
  crc16_t crc = calculateCRC(message, messageLength);
  message[messageLength] = (crc) & 0x00FF;
  ++messageLength;
  message[messageLength] = (crc >> 8) & 0x00FF;
  ++messageLength;
  
  return messageLength;
}

/** UART Unpack Response
 * This helper function will read the first byte of the response sent by the
 * slave and unpack it to read the number of expected bytes to be received.
 * @param response The Byte array containing the response sent by the slave
 * @return The length of the expected response
 */
static uint16_t uart_unpackResponse(uint8_t frameInitByte, uart_response_t *response) {
  
  frame_type_t frameType = (frameInitByte >> 7) & 0x01;
  if(frameType != RESPONSE) {
    // ERROR CASE: We received a response that did not have the response type
  }
  uint8_t responseBytes = frameInitByte & 0x7F;
  
  response->frameInit->frameType = frameType;
  response->frameInit->responseBytes = responseBytes;
  
  return responseBytes + 1;
}

/** UART Get Char
 * This helper function will attempt to read one byte from the RX line and
 * store the value of the read character c into the passed reference
 * @param uart A pointer to the UART object we are reading from
 * @param c A reference to the location we want to store the read character
 * @return The status of the UART result (success or failure)
 */
static uart_result_t uart_getChar(volatile uart_t *uart, uint8_t *c) {
  
  uint32_t longC;
  cmr_uartMsg_t rx;
  cmr_uartMsgInit(&rx);
  cmr_uartRX(&uart->port, &rx, c, sizeof(c), CMR_UART_RXOPTS_IDLEABORT);
  size_t len = cmr_uartMsgWait(&rx);
  *c = longC & 0x000000FF;
  if (len != 1)
  {
    return UART_FAILURE;
  }
  return UART_SUCCESS;  
}

/** UART Send Message
 * This helper function will send the message stored in the input message array
 * up to the number of bytes specified by the input messageLength using the
 * given USART
 * @param uart A pointer to the USART object we are sending from
 * @param message The array containing the bytes we want to send
 * @param messageLength The number of bytes from message to send
 * @return The status of the UART result (success or failure)
 */
static uart_result_t uart_sendMessage(volatile uart_t *uart, Byte message[], uint16_t messageLength) {
  
  cmr_uartMsg_t tx;
  cmr_uartMsgInit(&tx);
  cmr_uartTX(&uart->port, &tx, message, messageLength);
  size_t len = cmr_uartMsgWait(&tx);
  if (len != 1)
  {
    return UART_FAILURE;
  }
  return UART_SUCCESS;
}
