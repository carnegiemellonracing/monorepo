/**
 * @file can.h
 * @brief Board-specific CAN interface.
 * @author Carnegie Mellon Racing
**/


#ifndef CAN_H
#define CAN_H

#include <CMR/can.h>        // CMR CAN interface
#include <CMR/can_types.h>  // CMR CAN types
#include <CMR/can_ids.h>    // CMR CAN IDs

/**
 * @brief CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_LEN`.
 */
typedef enum {
    CANRX_HEARTBEAT_DAQ = 0,  /**< @brief VSM heartbeat. */
    CANRX_LEN_DAQ     /**< @brief Number of periodic CAN messages. */
} canRX_DAQ_t;

typedef enum {
    CANRX_HEARTBEAT_VSM = 0,  /**< @brief VSM heartbeat. */
	CANRX_HEARTBEAT_VSM1,
	CANRX_HEARTBEAT_VSM2,
	CANRX_HEARTBEAT_VSM3,
	CANRX_HEARTBEAT_VSM4,
    CANRX_LEN_VEH     /**< @brief Number of periodic CAN messages. */
} canRX_VEH_t;

typedef enum {
    CANRX_HEARTBEAT_INV = 0,  /**< @brief VSM heartbeat. */
    CANRX_LEN_TRAC    /**< @brief Number of periodic CAN messages. */
} canRX_TRAC_t;

//typedef enum {
////    CANRX_HEARTBEAT_VSM = 0,
//    CANRX_LEN_VSM
//} canRX_VSM_t;

extern cmr_canRXMeta_t canRXMeta_DAQ[];
extern cmr_canRXMeta_t canRXMeta_VEH[];
extern cmr_canRXMeta_t canRXMeta_TRAC[];
extern cmr_canRXMeta_t canRXMeta_VSM[];

void canInit(void);

int canTX_DAQ(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
int canTX_VEH(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
int canTX_TRAC(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
int canTX_VSM(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);

void *getPayloadDAQ(canRX_DAQ_t rxMsg);
void *getPayloadVEH(canRX_VEH_t rxMsg);
void *getPayloadTRAC(canRX_TRAC_t rxMsg);
//void *getPayloadVSM(canRX_VSM_t rxMsg);

void setHVC_heartbeat(uint16_t errorStatus, uint8_t hvcMode, uint8_t hvcState, uint8_t relayStatus, uint8_t uptime_s);
void setHVI_heartbeat(int16_t packCurrent_dA, uint16_t packVoltage_cV, int32_t packPower_W);
void setFSM_heartbeat(uint8_t state);
void setPTC_heartbeat(uint8_t state, uint8_t *error, uint8_t *warning);
void setDIM_heartbeat(uint8_t state, uint8_t *error, uint8_t *warning);
void setCDC_heartbeat(uint8_t state, uint8_t *error, uint8_t *warning);
void setFSM_data(uint8_t torqueRequested, uint8_t throttlePosition, uint16_t brakePressureFront_PSI, uint8_t brakePedalPosition, int32_t steeringWheelAngle_deg_FL, int32_t steeringWheelAngle_deg_FR);
void setDIM_request(uint8_t requestedState, uint8_t requestedGear, uint8_t requestedDrsMode, uint8_t requestedDriver);
void setinverter1(uint16_t status_bv, int16_t velocity_rpm, int16_t torqueCurrent_raw, int16_t magCurrent_raw);
void setinverter2(uint16_t status_bv, int16_t velocity_rpm, int16_t torqueCurrent_raw, int16_t magCurrent_raw);
void setinverter3(uint16_t status_bv, int16_t velocity_rpm, int16_t torqueCurrent_raw, int16_t magCurrent_raw);
void setinverter4(uint16_t status_bv, int16_t velocity_rpm, int16_t torqueCurrent_raw, int16_t magCurrent_raw);



#endif /* CAN_H */