/*

 This is short status line generation

*/
#define STATUS_LINE_SIZE 22
static char statusLine[STATUS_LINE_SIZE+1]; // status line for telemetry, oled display and otherwise; To be periodically updated


typedef struct armingFlagDescription_t {
    const char *longDescription;
    const char *shortDescription; // used for status line
    const uint8_t inverted;
} armingFlagDescription_t;

// armingFlagDescription - must be kept synced with armingFlags in runtime_config.h
/*
OK_TO_ARM       = (1 << 0),
PREVENT_ARMING  = (1 << 1),
ARMED           = (1 << 2),
WAS_EVER_ARMED  = (1 << 3)
*/
static const armingFlagDescription_t armingFlagDescription[] = {
    { "READY TO ARM", "RDY",0 },
    { "PREVENT ARMING", "PARM",1 },
    { "ARMED", "ARM",1}, // set MSB to indicate inverted chars
    { "NEVER ARMED", "NARM",0}
};

typedef struct flightFlagDescription_t {
    const char *longDescription;
    const char *shortDescription; // used for status line
    const uint8_t inverted;
} flightFlagDescription_t;

// flightFlagDescription - must be kept synced with flightFlags  in runtime_config.h
/*
ANGLE_MODE      = (1 << 0),
HORIZON_MODE    = (1 << 1),
MAG_MODE        = (1 << 2),
BARO_MODE       = (1 << 3),
GPS_HOME_MODE   = (1 << 4),
GPS_HOLD_MODE   = (1 << 5),
HEADFREE_MODE   = (1 << 6),
UNUSED_MODE     = (1 << 7), // old autotune
PASSTHRU_MODE   = (1 << 8),
SONAR_MODE      = (1 << 9),
FAILSAFE_MODE   = (1 << 10),
GTUNE_MODE      = (1 << 11),
*/
static const flightFlagDescription_t flightFlagDescription[] = {
    { "ANGLE MODE", "ANG", 0 },
    { "HORIZON MODE", "HOR", 0 },
    { "MAG MODE", "MAG", 0 },
    { "BARO MODE", "BAR", 0 },
    { "GPS HOME MODE", "GHM", 0 },
    { "GPS HOLD MODE", "GHL", 0 },
    { "HEADFREE MODE", "HFR", 0 },
    { "AUTOTUNE MODE", "ATU", 0 },
    { "PASSTHEU MODE", "PAS", 0 },
    { "SONAR MODE", "SON", 0 },
    { "FAILSAFE MODE", "FLS", 0 },
    { "GTUNE MODE", "GTU", 0 }
};

typedef struct stateFlagDescription_t {
    const char *longDescription;
    const char *shortDescription; // used for status line
    const uint8_t inverted;
} stateFlagDescription_t;

// stateFlagDescription - must be kept synced with stateFlags  in runtime_config.h
/*
GPS_FIX_HOME   = (1 << 0),
GPS_FIX        = (1 << 1),
CALIBRATE_MAG  = (1 << 2),
SMALL_ANGLE    = (1 << 3),
FIXED_WING     = (1 << 4),
*/
static const stateFlagDescription_t stateFlagDescription[] = {
    { "GPS FIX HOME", "GFH", 1 },    // inverted to remaind that home is not fixed
    { "GPS FIX", "GPS", 1},        // inverted to remaind that GPS position is not fixed
    { "CALIBRATE MAG", "CMG", 1 }, // inverted as it is an error
    { "SMALL ANGLE", "SAN", 0 },
    { "FIXED WING", "FWG", 0 }
};


void composeStatus(char * buffer, uint8_t size);
