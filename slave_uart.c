/*!
	slave_uart.c
	Protocol level delivery and receipt of UART messages

	Author: Homer Baker, Carnegie Mellon Racing
*/

#include "slave_uart.h"

//-----------------------------------------------------------------------------
// GLOBAL VARIABLE DEFINITIONS                                                |
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// STATIC VARIABLE DEFINITIONS                                                |
//-----------------------------------------------------------------------------

// FRAME INITIALIZATION CONSTANTS
static const frame_init_t CMD_BCAST_RESP_RADDR16_DATA2 = {
  .frameType = COMMAND,
  .requestType = BROADCAST_WRITE_WITH_RESPONSE,
  .addressSize = REGISTER_ADDRESS_16_BIT,
  .dataSize = DATA_SIZE_2_BYTE,
};
static const frame_init_t CMD_BCAST_NRESP_RADDR8_DATA1 = {
  .frameType = COMMAND,
  .requestType = BROADCAST_WRITE_WITHOUT_RESPONSE,
  .addressSize = REGISTER_ADDRESS_8_BIT,
  .dataSize = DATA_SIZE_1_BYTE,
};
static const frame_init_t CMD_BCAST_NRESP_RADDR8_DATA2 = {
  .frameType = COMMAND,
  .requestType = BROADCAST_WRITE_WITHOUT_RESPONSE,
  .addressSize = REGISTER_ADDRESS_8_BIT,
  .dataSize = DATA_SIZE_2_BYTE,
};
static const frame_init_t CMD_SINGLE_RESP_RADDR8_DATA1 = {
  .frameType = COMMAND,
  .requestType = SINGLE_WRITE_WITH_RESPONSE,
  .addressSize = REGISTER_ADDRESS_8_BIT,
  .dataSize = DATA_SIZE_1_BYTE,
};
static const frame_init_t CMD_SINGLE_NRESP_RADDR8_DATA1 = {
  .frameType = COMMAND,
  .requestType = SINGLE_WRITE_WITHOUT_RESPONSE,
  .addressSize = REGISTER_ADDRESS_8_BIT,
  .dataSize = DATA_SIZE_1_BYTE,
};
static const frame_init_t CMD_SINGLE_NRESP_RADDR8_DATA2 = {
  .frameType = COMMAND,
  .requestType = SINGLE_WRITE_WITHOUT_RESPONSE,
  .addressSize = REGISTER_ADDRESS_8_BIT,
  .dataSize = DATA_SIZE_2_BYTE,
};
static const frame_init_t CMD_BCAST_RESP_RADDR8_DATA1 = {
  .frameType = COMMAND,
  .requestType = BROADCAST_WRITE_WITH_RESPONSE,
  .addressSize = REGISTER_ADDRESS_8_BIT,
  .dataSize = DATA_SIZE_1_BYTE,
};
static const frame_init_t CMD_BCAST_NRESP_RADDR8_DATA4 = {
  .frameType = COMMAND,
  .requestType = BROADCAST_WRITE_WITHOUT_RESPONSE,
  .addressSize = REGISTER_ADDRESS_8_BIT,
  .dataSize = DATA_SIZE_4_BYTE
};
static const frame_init_t CMD_SINGLE_RESP_RADDR8_DATA2 = {
  .frameType = COMMAND,
  .requestType = SINGLE_WRITE_WITH_RESPONSE,
  .addressSize = REGISTER_ADDRESS_8_BIT,
  .dataSize = DATA_SIZE_2_BYTE,
};

//-----------------------------------------------------------------------------
// STATIC HELPER FUNCTION PROTOTYPES                                          |
//-----------------------------------------------------------------------------
static cmr_uart_result_t slave_uart_clearFaultFlags(uint8_t boardNum);

//-----------------------------------------------------------------------------
// GLOBAL INTERFACE FUNCTIONS                                                 |
//-----------------------------------------------------------------------------
/** Slave UART Auto-Address
 * This function sends the series of commands specified by the BQ76PL455A-Q1
 * protocol datasheet for enabling auto-addressing on each of the BMS slave
 * boards. The steps begin on page 2 of the datasheet.
 * @return The status of the UART result (success or failure)
 */
cmr_uart_result_t slave_uart_autoAddress() {
  
// Wake up and configure slave boards
  cmr_uart_result_t retvTotal = UART_SUCCESS;
  cmr_uart_result_t retv = UART_SUCCESS;
  
  /* From page 2 of BQ76PL455A-Q1 protocol datasheet 1.2.1
  * Enable the default communication params:
  * Set the baud rate to 250K
  * Enable the single-ended transmitter
  * Enable the high-side differential transmitter
  * Enable the low-side differential transmitter
  */
  static const uart_command_t enableDefaultCommunicationParams = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA2,
    .registerAddress = SLAVE_REG_COMCONFIG0,
    // Set UART Baudrate to 250kBauds, and configure UART, CommLH, and FaultLH
    // BQ Datasheet p76
    .data = {0x10, 0xF8},
  };
  
  retv = uart_sendCommand(&enableDefaultCommunicationParams);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  // Disable the user checksum for the boards
  static const uart_command_t disableUserChecksum = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA2,
    .registerAddress = SLAVE_REG_MASK_DEV0,
    .data = {0x80, 0x00},
  };
 
  retv = uart_sendCommand(&disableUserChecksum);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  // From page 3 of BQ76PL455A-Q1 protocol datasheet 1.2.2
  // Put the slave devices into Auto-Address Learning Mode
  static const uart_command_t selectAutoAddressMode = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA1,
    .registerAddress = SLAVE_REG_DEVCONFIG,
    // Configure auto-addressing mode and unlatched faults
    // BQ Datasheet p75
    .data = {0x11},
  };
  static const uart_command_t enterAutoAddressMode = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA1,
    .registerAddress = SLAVE_REG_DEV_CTRL,
    // Begin auto-addressing
    // BQ Datasheet p74
    .data = {0x08},
  };
  
  retv = uart_sendCommand(&selectAutoAddressMode);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  retv = uart_sendCommand(&enterAutoAddressMode);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  // From page 3 of BQ76PL455A-Q1 protocol datasheet 1.2.3
  // Set new addresses for daisy-chained devices
  uart_command_t setDeviceAddress = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA1,
    .registerAddress = SLAVE_REG_ADDR,
    .data = {0x00},
  };
  for(uint8_t boardNum = 0; boardNum < NUM_BMBS; ++boardNum) {
    setDeviceAddress.data[0] = boardNum;
    retv = uart_sendCommand(&setDeviceAddress);
    while (retv != UART_SUCCESS) {
      retvTotal = UART_FAILURE;
    }
  }
  
  uart_command_t getBmbAddress = {
    .frameInit = &CMD_SINGLE_RESP_RADDR8_DATA1,
    .deviceAddress = 0,
    .registerAddress = SLAVE_REG_ADDR,
    .data = {0x00},
  };
  // find top of stack, so do not need to hard code
  for(uint8_t boardNum = 0; boardNum < NUM_BMBS; ++boardNum) {
    getBmbAddress.deviceAddress = boardNum;
    // ask bmb for response
    retv = uart_sendCommand(&getBmbAddress);
    while (retv != UART_SUCCESS) {
      retvTotal = UART_FAILURE;
    }
    // wait for response
    uart_response_t deviceAddressResponse;
    retv = uart_receiveResponse(&deviceAddressResponse);
    while (retv != UART_SUCCESS) {
      retvTotal = UART_FAILURE;
    }
  }
  // From page 5 of BQ76PL455A-Q1 protocol datasheet 1.2.5/1.2.6
  // Receiver/Transmitter for Top/Bottom boards
  static uart_command_t disableHighSideReceiverTopBoard = {
    .frameInit = &CMD_SINGLE_NRESP_RADDR8_DATA2,
    .deviceAddress = TOP_SLAVE_BOARD,
    .registerAddress = SLAVE_REG_COMCONFIG0,
    // Enable only CommL and FaultL
    .data = {0x10, 0x28},
  };
  static uart_command_t disableLowSideTransmitterBotBoard = {
    .frameInit = &CMD_SINGLE_NRESP_RADDR8_DATA2,
    .deviceAddress = BOT_SLAVE_BOARD,
    .registerAddress = SLAVE_REG_COMCONFIG0,
    // Enable only UART, CommH, and FaultH
    .data = {0x10, 0xD0},
  };
  
  retv = uart_sendCommand(&disableHighSideReceiverTopBoard);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  retv = uart_sendCommand(&disableLowSideTransmitterBotBoard);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  // From page 6 of BQ76PL455A-Q1 protocol datasheet 1.2.7
  // Clear all existing faults on the boards, starting at top board
  uart_command_t clearDeviceFault = {
    .frameInit = &CMD_SINGLE_NRESP_RADDR8_DATA2,
    .deviceAddress = TOP_SLAVE_BOARD,
    .registerAddress = SLAVE_REG_FAULT_SUM0,
    .data = {0xFF, 0xC0},
  };
  for(int8_t boardNum = TOP_SLAVE_BOARD; boardNum >= BOT_SLAVE_BOARD; --boardNum) {
    clearDeviceFault.deviceAddress = boardNum;
    retv = uart_sendCommand(&clearDeviceFault);
    while (retv != UART_SUCCESS) {
      retvTotal = UART_FAILURE;
    }
  }
  
  return retvTotal;	
}

/** Slave UART Configure Sampling
 * This function sends the series of commands specified by the BQ76PL455A-Q1
 * protocol datasheet for configuring sampling on an individual BMS slave board
 * The steps begin on page 7 of the datasheet.
 * @param boardNum The number of the board in the BMS slave stack [0,N-1]
 * @return The status of the UART result (success or failure)
 */
cmr_uart_result_t slave_uart_configureSampling(uint8_t boardNum) {
  
  cmr_uart_result_t retvTotal = UART_SUCCESS;
  cmr_uart_result_t retv = UART_SUCCESS;
    
  // From page 7 of BQ76PL455A-Q1 protocol datasheet 2.2.1
  // Configure initial sampling delay
  uart_command_t configureInitialSamplingDelay = {
    .frameInit = &CMD_SINGLE_NRESP_RADDR8_DATA1,
    .deviceAddress = boardNum,
    .registerAddress = SLAVE_REG_SMPL_DLY1,
    .data = {0x00},
  };
  
  retv = uart_sendCommand(&configureInitialSamplingDelay);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  // From page 7 of BQ76PL455A-Q1 protocol datasheet 2.2.2
  // Configure voltage and internal sample period
  uart_command_t configureVoltageSamplePeriod = {
    .frameInit = &CMD_SINGLE_NRESP_RADDR8_DATA1,
    .deviceAddress = boardNum,
    .registerAddress = SLAVE_REG_CELL_SPER,
    .data = {0xBC},
  };
    
  retv = uart_sendCommand(&configureVoltageSamplePeriod);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  // From page 8 of BQ76PL455A-Q1 protocol datasheet 2.2.3
  // Configure the oversampling rate
  uart_command_t configureOversamplingRate = {
    .frameInit = &CMD_SINGLE_NRESP_RADDR8_DATA1,
    .deviceAddress = boardNum,
    .registerAddress = SLAVE_REG_OVERSMPL,
    .data = {0x00},
  };
  
  retv = uart_sendCommand(&configureOversamplingRate);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  // From page 8 of BQ76PL455A-Q1 protocol datasheet 2.2.4
  // Clear all fault flags on BMS slave board
  retv = slave_uart_clearFaultFlags(boardNum);
  if (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
    
  uart_command_t checkUnderVoltageFault = {
    .frameInit = &CMD_SINGLE_RESP_RADDR8_DATA1,
    .deviceAddress = 0x00,
    .registerAddress = SLAVE_REG_FAULT_UV0,
    .data = {0x01}
  };
  retv = uart_sendCommand(&checkUnderVoltageFault);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  uart_response_t undervoltageRegisterCheckResponse = {0};
  retv = uart_receiveResponse(&undervoltageRegisterCheckResponse);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  return retvTotal;
}

cmr_uart_result_t slave_uart_configureChannels() {
  
  cmr_uart_result_t retvTotal = UART_SUCCESS;
  cmr_uart_result_t retv = UART_SUCCESS;
  
  static uart_command_t selectNumberOfChannels = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA1,
    .registerAddress = SLAVE_REG_NCHAN,
    .data = {0x09},
  };
  retv = uart_sendCommand(&selectNumberOfChannels);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  static uart_command_t selectChannelsOnModule = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA4,
    .registerAddress = SLAVE_REG_CHANNELS0,
    .data = {0x01, 0xFF, 0xFF, 0x00},
  };
  retv = uart_sendCommand(&selectChannelsOnModule);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  static uart_command_t setOvervoltageThresholds = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA2,
    .registerAddress = SLAVE_REG_CELL_OV0,
    // Cell over-voltage set to 4.2v, max spec'ed for the cell. Cells won't be charged above
    // 4.15v, but balanacing will have them above this briefly, so it should not be an error.
    // BQ Datasheet p93
    .data = {0xD7, 0x08},
  };
  static uart_command_t setUndervoltageThresholds = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA2,
    .registerAddress = SLAVE_REG_CELL_UV0,
    // Cell under-voltage set to 2.5V, which will only be reached during a large droop while
    // drawing current.
    // BQ Datasheet p93
    .data = {0x80, 0x00},
  };
  
  retv = uart_sendCommand(&setOvervoltageThresholds);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  retv = uart_sendCommand(&setUndervoltageThresholds);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }

  //----------------------------------------------------------------------------
  // ENTER DEBUG
  //----------------------------------------------------------------------------

/*
  static const uart_command_t setComparatorUndervoltageThresholds = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA1,
    .registerAddress = SLAVE_REG_COMP_UV,
    .data = {0x90},
  };
  
  retv = uart_sendCommand(&setComparatorUndervoltageThresholds);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
*/

  //----------------------------------------------------------------------------
  // EXIT DEBUG
  //----------------------------------------------------------------------------

  return retvTotal;
}

cmr_uart_result_t slave_uart_sampleAllChannels(uart_response_t response[NUM_BMBS]) {

	cmr_uart_result_t retvTotal = UART_SUCCESS;
	cmr_uart_result_t retv = UART_SUCCESS;

  static uart_command_t sampleAllChannels = {
    .frameInit = &CMD_BCAST_RESP_RADDR8_DATA1,
    .registerAddress = SLAVE_REG_CMD,
    .data = {TOP_SLAVE_BOARD},
  };

	retv = uart_sendCommand(&sampleAllChannels);
	while (retv != UART_SUCCESS) {
		retvTotal = UART_FAILURE;
	}

	for(int8_t i = TOP_SLAVE_BOARD; i >= 0; --i) {
		retv = uart_receiveResponse(&response[i]);
		while (retv != UART_SUCCESS) {
			retvTotal = UART_FAILURE;
		}
	}

	return retvTotal;
}

cmr_uart_result_t slave_uart_broadcast_sampleAndStore() {

  cmr_uart_result_t retv = UART_SUCCESS;

  static uart_command_t sampleAndStore = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA1,
    .registerAddress = SLAVE_REG_CMD,
    .data = {0x00}, //SYNC SAMPLE command: BQ Protocol p12
  };

  retv = uart_sendCommand(&sampleAndStore);

  return retv;
}

cmr_uart_result_t slave_uart_sampleDeviceChannels(uint8_t deviceAddress, uart_response_t *response) {

    cmr_uart_result_t retvTotal = UART_SUCCESS;
    cmr_uart_result_t retv = UART_SUCCESS;

    // Command to sample all channels on a single BMB
    // BQ Protocol p13
    uart_command_t sampleDeviceChannels = {
        .frameInit = &CMD_SINGLE_RESP_RADDR8_DATA1,
        .registerAddress = SLAVE_REG_CMD,
        .deviceAddress = deviceAddress,
        .data = {0x20},
    };

    retv = uart_sendCommand(&sampleDeviceChannels);

    while (retv != UART_SUCCESS) {
        retvTotal = UART_FAILURE;
    }

    retv = uart_receiveResponse(response);
    int retry = 10;
    while (retv != UART_SUCCESS && retry > 0) {
        retvTotal = UART_FAILURE;
        retv = uart_receiveResponse(response);
        if(retry > 0) retvTotal = retv;
        retry--;
    }

    return retvTotal;
}

cmr_uart_result_t slave_uart_configureGPIODirection(uint8_t DDRVector, uint8_t deviceAddress) {

    cmr_uart_result_t retv = UART_SUCCESS;

    // Command to configure GPIO data direction
    // BQ Protocol p21
    uart_command_t configureGPIODirection = {
        .frameInit = &CMD_SINGLE_NRESP_RADDR8_DATA1,
        .registerAddress = SLAVE_REG_GPIO_DIR,
        .deviceAddress = deviceAddress,
        .data = {DDRVector},
    };

    retv = uart_sendCommand(&configureGPIODirection);

    return retv;
}

cmr_uart_result_t slave_uart_setGPIO(uint8_t data, uint8_t deviceAddress) {

    cmr_uart_result_t retv = UART_SUCCESS;

    // Command to configure GPIO data direction
    // BQ Protocol p21
    uart_command_t setGPIO = {
        .frameInit = &CMD_SINGLE_NRESP_RADDR8_DATA1,
        .registerAddress = SLAVE_REG_GPIO_OUT,
        .deviceAddress = deviceAddress,
        .data = {data},
    };

    retv = uart_sendCommand(&setGPIO);

    return retv;
}

cmr_uart_result_t slave_uart_broadcast_setBMBTimeout() {

    cmr_uart_result_t retv = UART_SUCCESS;

    // Command to configure communications timeout
    // BQ Datasheet p80
    uart_command_t setCTO = {
        .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA1,
        .registerAddress = SLAVE_REG_CTO,
        // High nybble is shutdown timeout, low nybble is fault timeout.
        // Fault MUST be set if shutdown is used, and must be less than shutdown.
        // Set shutdown to 5s, fault to 2s
        .data = {0x54},
    };

    retv = uart_sendCommand(&setCTO);

    return retv;
}


// "cells" is a bit vector containing the cells to be balanced.
// For example, cells = 0x0305 will balance cells 9, 8, 2, and 0.
cmr_uart_result_t slave_uart_sendBalanceCmd(uint16_t cells, uint8_t deviceAddress) {
	cmr_uart_result_t retv = UART_SUCCESS;

	// Clear top 4 bits, since our segments are 12 cells each
	cells &= 0x0FFF;

	uart_command_t balanceCmd = {
		.frameInit = &CMD_SINGLE_NRESP_RADDR8_DATA2,
		.registerAddress = SLAVE_REG_CBENBL0,
		.deviceAddress = deviceAddress,
		.data = {0x00, 0x00}
	};

	// AVR32 and BQ are both big-endian
	balanceCmd.data[0] = (cells >> 8) & 0xFF;
	balanceCmd.data[1] = cells & 0xFF;

	retv = uart_sendCommand(&balanceCmd);

	return retv;
}

cmr_uart_result_t slave_uart_sendEnableTempMuxCmd(uint8_t enable) {
  cmr_uart_result_t retv = UART_SUCCESS;

  uart_command_t EnableTempMuxCmd = {
    .frameInit = &CMD_BCAST_NRESP_RADDR8_DATA1,
    .registerAddress = SLAVE_REG_GPIO_OUT,
    .data = {enable}, //SYNC SAMPLE command: BQ Protocol p12
  };

  retv = uart_sendCommand(&EnableTempMuxCmd);
  return retv;
}

//-----------------------------------------------------------------------------
// STATIC HELPER FUNCTIONS                                                    |
//-----------------------------------------------------------------------------

static cmr_uart_result_t slave_uart_clearFaultFlags(uint8_t boardNum) {
 
  cmr_uart_result_t retvTotal = UART_SUCCESS;
  cmr_uart_result_t retv = UART_SUCCESS;
 
  // From page 8 of BQ76PL455A-Q1 protocol datasheet 2.2.4
  // Clear all fault flags on BMS slave board
  uart_command_t clearFaultFlags = {
    .frameInit = &CMD_SINGLE_NRESP_RADDR8_DATA1,
    .deviceAddress = boardNum,
    .registerAddress = SLAVE_REG_STATUS,
    .data = {0x38},
  };
  retv = uart_sendCommand(&clearFaultFlags);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  
  uart_command_t clearFaultSummaryFlags = {
    .frameInit = &CMD_SINGLE_NRESP_RADDR8_DATA2,
    .deviceAddress = boardNum,
    .registerAddress = SLAVE_REG_FAULT_SUM0,
    .data = {0xFF, 0xC0},
  };
  retv = uart_sendCommand(&clearFaultSummaryFlags);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  

//  /////// START //////
//    // try to see the faults on bmb 1
//    uart_command_t getFaults = {
//      .frameInit = &CMD_SINGLE_RESP_RADDR8_DATA1,
//      .deviceAddress = 1,
//      .registerAddress = SLAVE_REG_FAULT_SYS,
//      .data = {0x00},
//    };
//
//    // ask bmb for response
//    retv = uart_sendCommand(&getFaults);
//    while (retv != UART_SUCCESS) {
//      retvTotal = UART_FAILURE;
//    }
//    // wait for response
//    uart_response_t bmb1Fault_response;
//    retv = uart_receiveResponse(&bmb1Fault_response);
//    while (retv != UART_SUCCESS) {
//      retvTotal = UART_FAILURE;
//    }
//    /////// END //////
//
    /////// START //////
//     // try to see the uv on bmb 1
//  if (boardNum == 1){
//     uart_command_t getFaults2 = {
//       .frameInit = &CMD_SINGLE_RESP_RADDR8_DATA2,
//       .deviceAddress = 1,
//       .registerAddress = SLAVE_REG_FAULT_UV0,
//       .data = {0x00, 0x00},
//     };
//
//     // ask bmb for response
//     retv = uart_sendCommand(&getFaults2);
//     while (retv != UART_SUCCESS) {
//       retvTotal = UART_FAILURE;
//     }
//     // wait for response
//     uart_response_t bmb2Fault_response;
//     retv = uart_receiveResponse(&bmb2Fault_response);
//  }

//     while (retv != UART_SUCCESS) {
//       retvTotal = UART_FAILURE;
//     }
     /////// END //////


  /*
  * Check the system status register, and check result is equal to 0 (no faults)
  * Comes from page 8 of BQ76PL455A-Q1 protocol datasheet 2.2.4
  */
  uart_command_t checkSystemStatusRegister = {
    .frameInit = &CMD_SINGLE_RESP_RADDR8_DATA1,
    .deviceAddress = boardNum,
    .registerAddress = SLAVE_REG_STATUS,
    .data = {0x00},
  };
  retv = uart_sendCommand(&checkSystemStatusRegister);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  frame_init_t statusRegisterCheckResponseFrameInit = {0};
  uart_response_t statusRegisterCheckResponse = {0};
  statusRegisterCheckResponse.frameInit = &statusRegisterCheckResponseFrameInit;
  retv = uart_receiveResponse(&statusRegisterCheckResponse);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  uint8_t statusRegisterData = statusRegisterCheckResponse.data[0];
  if (statusRegisterData != 0x00) {
    // ERROR CASE: We still saw faults in the status register after clearing them
    retvTotal = UART_FAILURE;
  }
  
  /*
  * Check the fault summary register, and check result is equal to 0 (no faults)
  * Comes from page 8 of BQ76PL455A-Q1 protocol datasheet 2.2.4
  */
  uart_command_t checkFaultSummaryRegister = {
    .frameInit = &CMD_SINGLE_RESP_RADDR8_DATA1,
    .deviceAddress = boardNum,
    .registerAddress = SLAVE_REG_FAULT_SUM0,
    .data = {0x01},
  };
  retv = uart_sendCommand(&checkFaultSummaryRegister);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  frame_init_t faultSummaryRegisterCheckResponseFrameInit = {0};
  uart_response_t faultSummaryRegisterCheckResponse = {0};
  faultSummaryRegisterCheckResponse.frameInit = &faultSummaryRegisterCheckResponseFrameInit;
  retv = uart_receiveResponse(&faultSummaryRegisterCheckResponse);
  while (retv != UART_SUCCESS) {
    retvTotal = UART_FAILURE;
  }
  uint16_t faultSumaryRegisterData = ((uint16_t)faultSummaryRegisterCheckResponse.data[0] << 8) |
  (uint16_t)faultSummaryRegisterCheckResponse.data[1];
  while (faultSumaryRegisterData != 0x0000) {
    // ERROR CASE: We still saw faults in the fault summary register after clearing them
    retvTotal = UART_FAILURE;
  }

  return retvTotal;
}

