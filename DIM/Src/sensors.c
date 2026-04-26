

/**
 * @file sensors.c
 * @brief Board-specific sensors implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "sensors.h"  // Interface to implement

#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface
#include <CMR/utils.h>  // Task interface
#include <stdint.h>
#include <stdlib.h>     // abs()
#include <stdint.h>

#include "adc.h"   // Board-specific ADC interface
#include "can.h"   // Board-specific CAN interface
#include "gpio.h"  // Board-specific GPIO interface

/** @brief  Experimentally determined left TPOS ADC Minimum*/
#define LEFT_TPOS_MIN_ADC 2700

/** @brief  Experimentally determined left TPOS ADC Maximum*/
#define LEFT_TPOS_MAX_ADC 3280

/** @brief  Experimentally determined right TPOS ADC Minimum*/
#define RIGHT_TPOS_MIN_ADC 480

/** @brief  Experimentally determined right TPOS ADC Maximum*/
#define RIGHT_TPOS_MAX_ADC 1180

#define LEFT_SWANGLE_MIN_ADC 360.0f
#define CENTER_SWANGLE_ADC 1640.0f
#define RIGHT_SWANGLE_MAX_ADC 2850.0f

#define MAX_OUTER_WHEEL_ANGLE_MILLIDEG 25282.0f
#define MAX_INNER_WHEEL_ANGLE_MILLIDEG 30139.0f

/** @brief See FSAE rule T.6.2.3 for definition of throttle implausibility. */
static const TickType_t TPOS_IMPLAUS_THRES_MS = 100;
/** @brief See FSAE rule T.6.2.3 for definition of throttle implausibility. */
static const uint32_t TPOS_IMPLAUS_THRES = 50;

/** @brief Throttle threshold for brake implausibility. See FSAE rule EV.2.4. */
static const uint8_t BPP_TPOS_IMPLAUS_THRES = UINT8_MAX / 4;
/** @brief Throttle threshold for clearing brake implausibility. See FSAE rule EV.2.4. */
static const uint8_t BPP_TPOS_CLEAR_THRES = UINT8_MAX / 20;
/** @brief Threshold where brakes are considered to be actuated. */
static const uint16_t BRAKE_ACTIVE_THRES_PSI = 100;

/** @brief 90 degree sw left lock adc value. */
#define SWANGLE_90DEG_LEFT 1345
/** @brief -90 degree sw RIGHT lock adc value. */
#define SWANGLE_90DEG_RIGHT 3155

/** @brief Min current for EBS solenoid to be actuated */
#define MIN_EBS_SOLENOID_CURRENT_MA 80

/** @brief Max current for EBS solenoid to be actuated */
#define MAX_EBS_SOLENOID_CURRENT_MA 150

/** @brief Min air pressure for EBS to be actuated */
#define MIN_EBS_AIR_PRES_DECI_BAR 40

/** @brief Max air pressure for EBS to be actuated */
#define MAX_EBS_AIR_PRES_DECI_BAR 120

/**
 * @brief Mapping of sensor channels to ADC channels.
 */
const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN] = {
    [SENSOR_CH_TPOS_L_U8]           = ADC_TPOS_L,
    [SENSOR_CH_TPOS_R_U8]           = ADC_TPOS_R,
    [SENSOR_CH_BPOS_U8]             = ADC_BPRES,
    [SENSOR_CH_BPRES_PSI]           = ADC_BPRES,
    [SENSOR_CH_SWANGLE_DEG_FL]      = ADC_SWANGLE,
    [SENSOR_CH_SWANGLE_DEG_FR]      = ADC_SWANGLE,
    [SENSOR_CH_EBS_PRESSURE_1_DECI_BAR]  = ADC_EBS_AIR_PRES_1,
	[SENSOR_CH_EBS_PRESSURE_2_DECI_BAR]  = ADC_EBS_AIR_PRES_2,
    [SENSOR_CH_EBS_CURRENT_1_MA]    = ADC_EBS_CURRENT_1,
	[SENSOR_CH_EBS_CURRENT_2_MA]    = ADC_EBS_CURRENT_2,
};

/** @brief forward declaration */
static cmr_sensor_t sensors[SENSOR_CH_LEN];

/** @brief The list of sensors sampled by the driver. */
cmr_sensorList_t sensorList;

/**
 * @brief Gets a new value from an ADC sensor.
 *
 * This function simply copies data from the ADC - it does not make any requests
 * to ADC hardware.
 *
 * @param sensor The ADC sensor to sample.
 *
 * @return The latest sampled value from the ADC.
 */
static uint32_t sampleADCSensor(const cmr_sensor_t *sensor) {
    sensorChannel_t sensorChannel = sensor - sensors;
    configASSERT(sensorChannel < SENSOR_CH_LEN);
    return adcRead(sensorsADCChannels[sensorChannel]);
}


/**
 * @brief Rescales ADC value from 12 bit to 8 bit.
 *
 * @param sensor The sensor to read.
 * @param reading The ADC value to convert.
 *
 * @return Value of sensor, rescaled to 0-255.
 */
static int32_t adcToUInt8(const cmr_sensor_t *sensor, uint32_t reading) {
    uint32_t clamped_val = CLAMP(sensor->readingMin, reading, sensor->readingMax);
    int32_t ret_val = INVLERP_SCALED(sensor->readingMin, sensor->readingMax, clamped_val, UINT8_MAX);
    return ret_val;
}

/**
 * @brief Converts a raw ADC value to a brake pressure in PSI.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Front brake pressure in PSI.
 */
static int32_t adcToBPres_PSI(const cmr_sensor_t *sensor, uint32_t reading) {
    (void)sensor;  // Placate compiler.

    static const uint32_t offset = 409;     // 0.333 V offset

    (void) sensor;  // Placate compiler.

    if (reading < offset) {
        // Clamp to 0.
        reading = offset;
    }

    uint32_t brakePres_PSI = (reading - offset) * 1450 / 3277;
    return (int32_t) brakePres_PSI;
}

/**
 * @brief Computes the yaw for the FR wheel
 *
 * @param sensor The sensor to read.
 * @param reading The ADC value to convert.
 *
 * @return the angle of the FR wheel in milli degrees
 */
static int32_t adcToYaw_FL(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;

    // millideg
    if(reading >= CENTER_SWANGLE_ADC){
        // left is outer wheel
        float FL_angle_millideg = INVLERP_SCALED(CENTER_SWANGLE_ADC, RIGHT_SWANGLE_MAX_ADC, reading, MAX_OUTER_WHEEL_ANGLE_MILLIDEG);
        int32_t retval = (int32_t)FL_angle_millideg;
        return retval;
    }
    else {
        // left is inner wheel
        float FL_angle_millideg = INVLERP_SCALED(CENTER_SWANGLE_ADC, LEFT_SWANGLE_MIN_ADC, reading, MAX_INNER_WHEEL_ANGLE_MILLIDEG);
        int32_t retval = -(int32_t)FL_angle_millideg;
        return retval;
    }
}

/**
 * @brief Computes the yaw for the FR wheel
 *
 * @param sensor The sensor to read.
 * @param reading The ADC value to convert.
 *
 * @return the angle of the FR wheel in milli degrees
 */
static int32_t adcToYaw_FR(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;

    // millideg
    if(reading >= CENTER_SWANGLE_ADC){
        // right is inner wheel
        float FR_angle_millideg = INVLERP_SCALED(CENTER_SWANGLE_ADC, RIGHT_SWANGLE_MAX_ADC, reading, MAX_INNER_WHEEL_ANGLE_MILLIDEG);
        int32_t retval = (int32_t)FR_angle_millideg;
        return retval;
    }
    else {
        // right is outer wheel
        float FR_angle_millideg = INVLERP_SCALED(CENTER_SWANGLE_ADC, LEFT_SWANGLE_MIN_ADC, reading, MAX_OUTER_WHEEL_ANGLE_MILLIDEG);
        int32_t retval = -(int32_t)FR_angle_millideg;
        return retval;
    }
}

/** 
 * @brief Samples whether the throttle position is implausible.
 *
 * @param sensor The sensor (ignored).
 *
 * @return 1 if the throttle position is implausible, 0 otherwise.
 */
static uint32_t sampleTPOSDiff(const cmr_sensor_t *sensor) {
    (void)sensor;  // Placate compiler.

    /** @brief Last plausible time. */
    static TickType_t lastPlausible = 0;
    TickType_t now = xTaskGetTickCount();

    uint8_t diff;
    uint8_t leftPosition = (uint8_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_TPOS_L_U8);
    uint8_t rightPosition = (uint8_t)  cmr_sensorListGetValue(&sensorList, SENSOR_CH_TPOS_R_U8);
    diff = abs(leftPosition - rightPosition);

    if (diff < TPOS_IMPLAUS_THRES) {
        // Still plausible; move on.
        lastPlausible = now;
        return 0;
    }

    if (now - lastPlausible < TPOS_IMPLAUS_THRES_MS) {
        // Threshold not elapsed; move on.
        return 0;
    }

    return 1;  // Implausible!
}

/**
 * @brief Computes the Tank Pressure from the reading
 *
 * @param sensor The sensor to read.
 * @param reading The ADC value to convert.
 *
 * @return pressure of the tank in deci-bar
 */
int32_t adcToEBSBrakePressure_deci_bar(const cmr_sensor_t *sensor, uint32_t reading) {
    (void)sensor;  // Placate compiler.

    uint32_t reading_mV_3v3 = reading * 3300 / 4096;
    // Values come from a 9.1K and 4.7k voltage divider
    uint32_t reading_mV  = reading_mV_3v3 * 138 / 91;

    const uint32_t lower_bound_mV = 500; // voltage at 0 bar
    const uint32_t upper_bound_mV = 4500; // voltage at 100 bar
    const uint32_t max_pressure_deci_bar = 100; // voltage at 0 bar
    uint32_t clamped_reading_mV = CLAMP (lower_bound_mV, reading_mV, upper_bound_mV);
    uint32_t reading_deci_bar = INVLERP_SCALED(lower_bound_mV,upper_bound_mV, clamped_reading_mV, max_pressure_deci_bar);
    
    return (uint16_t)reading_deci_bar;
}

/**
 * @brief Computes the current sent to the solenoid based on the ADC reading
 *
 * @param sensor The sensor to read.
 * @param reading The ADC value to convert.
 *
 * @return current to the solenoid in mA
 */
int32_t adc_to_solenoid_current_ma(const cmr_sensor_t *sensor, uint32_t reading) {
    (void)sensor;  // Placate compiler.
    uint32_t reading_voltage_mV = (reading * 3300 / 4096);
    // op=amp circuit has gain of 10
    uint32_t voltage_across_resistor_mV = reading_voltage_mV / 10;
    // the resistor is one ohms so the current is equal to the voltage
    return voltage_across_resistor_mV;
}

/**
 * @brief Sample whether the brake pedal position is implausible.
 *
 * @param sensor The sensor (ignored).
 *
 * @return 1 if the brake pedal position is implausible, 0 otherwise.
 */
static uint32_t sampleBrakeImplaus(const cmr_sensor_t *sensor) {
    (void)sensor;  // Placate compiler.

    /** @brief Current implausibility status. */
    static bool implaus = false;

    cmr_canVSMSensors_t *vsmSensors = getPayload(CANRX_VSM_SENSORS);

    uint8_t throttlePos = throttleGetPos();
    uint16_t brakePres_PSI = vsmSensors->brakePressureRear_PSI;

    // Set brake implausibility
    if ((throttlePos > BPP_TPOS_IMPLAUS_THRES) && (brakePres_PSI > BRAKE_ACTIVE_THRES_PSI)) {
        implaus = true;
    }
    // Clear brake implausibility
    else if (throttlePos < BPP_TPOS_CLEAR_THRES) {
        implaus = false;
    }

    return implaus;
}


/**
 * @brief Gets average throttle position.
 *
 * @note If either sensor has an error or the throttle position is implausible,
 * 0 is returned.
 *
 * @return Throttle position (0-255, inclusive).
 */
uint8_t throttleGetPos(void) {
    if (cmr_sensorListGetValue(&sensorList, SENSOR_CH_TPOS_IMPLAUS) != 0) {
        return 0;
    }

    uint8_t tposL = cmr_sensorListGetValue(&sensorList, SENSOR_CH_TPOS_L_U8);
    uint8_t tposR = cmr_sensorListGetValue(&sensorList, SENSOR_CH_TPOS_R_U8);

    return (uint8_t) MIN(tposL, tposR);
}


static cmr_sensor_t sensors[SENSOR_CH_LEN] = {
    [SENSOR_CH_TPOS_L_U8] = {
        .conv            = adcToUInt8,
        .sample          = sampleADCSensor,
        .readingMin      = LEFT_TPOS_MIN_ADC,
        .readingMax      = LEFT_TPOS_MAX_ADC,
        .outOfRange_pcnt = 10,
        .warnFlag        = CMR_CAN_WARN_FSM_TPOS_L,
    },
    [SENSOR_CH_TPOS_R_U8] = { 
        .conv            = adcToUInt8, 
        .sample          = sampleADCSensor, 
        .readingMin      = RIGHT_TPOS_MIN_ADC, 
        .readingMax      = RIGHT_TPOS_MAX_ADC, 
        .outOfRange_pcnt = 10, 
        .warnFlag        = CMR_CAN_WARN_FSM_TPOS_R,
    },
    [SENSOR_CH_BPOS_U8] = { 
        .conv            = adcToUInt8, 
        .sample          = sampleADCSensor, 
        .readingMin      = 365, 
        .readingMax      = 1651, 
        .outOfRange_pcnt = 10, 
        .warnFlag        = CMR_CAN_WARN_FSM_BPOS,
    },
    [SENSOR_CH_BPRES_PSI] = { 
        .conv            = adcToBPres_PSI, 
        .sample          = sampleADCSensor, 
        .readingMin      = 0, 
        .readingMax      = CMR_ADC_MAX, 
        .outOfRange_pcnt = 10, 
        .warnFlag        = CMR_CAN_WARN_FSM_BPRES,
    },
    [SENSOR_CH_SWANGLE_DEG_FL] = { 
        .conv            = adcToYaw_FL, 
        .sample          = sampleADCSensor, 
        .readingMin      = SWANGLE_90DEG_LEFT, 
        .readingMax      = SWANGLE_90DEG_RIGHT, 
        .outOfRange_pcnt = 10, 
        .warnFlag        = CMR_CAN_WARN_FSM_SWANGLE,
    },
    [SENSOR_CH_SWANGLE_DEG_FR] = { 
        .conv            = adcToYaw_FR, 
        .sample          = sampleADCSensor, 
        .readingMin      = SWANGLE_90DEG_LEFT, 
        .readingMax      = SWANGLE_90DEG_RIGHT, 
        .outOfRange_pcnt = 10, 
        .warnFlag        = CMR_CAN_WARN_FSM_SWANGLE,
    },
    [SENSOR_CH_TPOS_IMPLAUS] = { 
        .conv            = NULL, 
        .sample          = sampleTPOSDiff,
        .readingMin      = 0,    // output is typically 2V max
        .readingMax      = 2600,
        .warnFlag        = CMR_CAN_WARN_FSM_TPOS_IMPLAUSIBLE,
    },
    [SENSOR_CH_BPP_IMPLAUS] = { 
        .conv            = NULL, 
        .sample          = sampleBrakeImplaus,
        .readingMin      = 0,    // output is typically 2V max
        .readingMax      = 2600,
        .warnFlag        = CMR_CAN_WARN_FSM_BPP,
    },
    [SENSOR_CH_EBS_PRESSURE_1_DECI_BAR] = {   
        .conv            = adcToEBSBrakePressure_deci_bar, 
        .sample          = sampleADCSensor,
        .readingMin      = MIN_EBS_AIR_PRES_DECI_BAR,
        .readingMax      = MAX_EBS_AIR_PRES_DECI_BAR,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_EBS_PRESSURE_2_DECI_BAR] = {   
        .conv            = adcToEBSBrakePressure_deci_bar, 
        .sample          = sampleADCSensor,
        .readingMin      = MIN_EBS_AIR_PRES_DECI_BAR,
        .readingMax      = MAX_EBS_AIR_PRES_DECI_BAR,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_EBS_CURRENT_1_MA] = {   
        .conv            = adc_to_solenoid_current_ma, 
        .sample          = sampleADCSensor,
        .readingMin      = MIN_EBS_SOLENOID_CURRENT_MA,
        .readingMax      = MAX_EBS_SOLENOID_CURRENT_MA,
        .outOfRange_pcnt = 10, 
    },
    [SENSOR_CH_EBS_CURRENT_2_MA] = {   
        .conv            = adc_to_solenoid_current_ma, 
        .sample          = sampleADCSensor,
        .readingMin      = MIN_EBS_SOLENOID_CURRENT_MA,
        .readingMax      = MAX_EBS_SOLENOID_CURRENT_MA,
        .outOfRange_pcnt = 10, 
    },
};

/** @brief Sensors update priority. */
static const uint32_t sensorsUpdate_priority = 5;

/** @brief Sensors update period (milliseconds). */
static const TickType_t sensorsUpdate_period_ms = 10;

/** @brief Sensors update task. */
static cmr_task_t sensorsUpdate_task;

/**
 * @brief Task for updating sensor values.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void sensorsUpdate(void *pvParameters) {
    (void)pvParameters;  // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_sensorListUpdate(&sensorList);
        vTaskDelayUntil(&lastWakeTime, sensorsUpdate_period_ms);
    }
}

/**
 * @brief Initializes the sensor interface.
 */
void sensorsInit(void) {
    cmr_sensorListInit(
        &sensorList,
        sensors, sizeof(sensors) / sizeof(sensors[0]));

    // Task creation.
    cmr_taskInit(
        &sensorsUpdate_task,
        "sensor update",
        sensorsUpdate_priority,
        sensorsUpdate,
        NULL);
}