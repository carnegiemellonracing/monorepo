//Based on "DSF - Startup Control Check Flowchat"
#include <stdbool.h>

typedef enum {
    IDLE,
    BEGIN,
    SHUTDOWN_CIRCUIT_WAIT,
    TIME_CHECK,
    EBS_PRESSURE_CHECK,
    HYDRAULIC_PRESSURE_CHECK_1,
    TSAB_WAIT,
    MB_VALVES_SWITCH_ON_OFF,
    HYDRAULIC_PRESSURE_CHECK_2,
    MB_VALVES_SWITCH_OFF_ON,
    HYDRAULIC_PRESSURE_CHECK_3,
    READY,
    EBS_ERROR
} brakecheck_state;

brakecheck_state currState = IDLE;
brakecheck_state nextState = IDLE;
TickType_t lastWakeTime;

static bool checkDSMS();

TickType_t getTime();

static bool checkEBSPressure();

static bool checkHydraulicPressure(float BP3_min, float BP3_max, float BP4_min, float BP4_max);

static void toggleMB(bool MB1_state, bool MB2_state);

static bool checkShutdownCircuit();

static bool checkTSActivated();

#define EBS_PRESSURE_BAR_MIN 8.0
#define EBS_PRESSURE_BAR_MAX 10.0