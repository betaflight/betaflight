#include "board.h"
#include "mw.h"

#ifndef sq
#define sq(x) ((x)*(x))
#endif

// GPS timeout for wrong baud rate/disconnection/etc in milliseconds (default 2.5second)
#define GPS_TIMEOUT (2500)
// How many entries in gpsInitData array below
#define GPS_INIT_ENTRIES (GPS_BAUD_MAX + 1)
#define GPS_BAUD_DELAY (100)

typedef struct gpsInitData_t {
    uint8_t index;
    uint32_t baudrate;
    const char *ubx;
    const char *mtk;
} gpsInitData_t;

// NMEA will cycle through these until valid data is received
static const gpsInitData_t gpsInitData[] = {
    { GPS_BAUD_115200, 115200, "$PUBX,41,1,0003,0001,115200,0*1E\r\n", "$PMTK251,115200*1F\r\n" },
    { GPS_BAUD_57600,   57600, "$PUBX,41,1,0003,0001,57600,0*2D\r\n", "$PMTK251,57600*2C\r\n" },
    { GPS_BAUD_38400,   38400, "$PUBX,41,1,0003,0001,38400,0*26\r\n", "$PMTK251,38400*27\r\n" },
    { GPS_BAUD_19200,   19200, "$PUBX,41,1,0003,0001,19200,0*23\r\n", "$PMTK251,19200*22\r\n" },
    // 9600 is not enough for 5Hz updates - leave for compatibility to dumb NMEA that only runs at this speed
    { GPS_BAUD_9600,     9600, "", "" }
};

static const uint8_t ubloxInit[] = {
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,           // VGS: Course over ground and Ground speed
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,           // GSV: GNSS Satellites in View
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,           // GLL: Latitude and longitude, with time of position fix and status
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,           // GGA: Global positioning system fix data
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,           // GSA: GNSS DOP and Active Satellites
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,           // RMC: Recommended Minimum data
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,           // set POSLLH MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,           // set STATUS MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,           // set SOL MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,           // set VELNED MSG rate
    0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xE5, // set SBAS to auto-detect
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A,             // set rate to 5Hz
};

enum {
    GPS_UNKNOWN,
    GPS_INITIALIZING,
    GPS_INITDONE,
    GPS_RECEIVINGDATA,
    GPS_LOSTCOMMS,
};

typedef struct gpsData_t {
    uint8_t state;                  // GPS thread state. Used for detecting cable disconnects and configuring attached devices
    uint8_t baudrateIndex;          // index into auto-detecting or current baudrate
    int errors;                     // gps error counter - crc error/lost of data/sync etc. reset on each reinit.
    uint32_t lastMessage;           // last time valid GPS data was received (millis)
    uint32_t lastLastMessage;       // last-last valid GPS message. Used to calculate delta.

    uint32_t state_position;           // incremental variable for loops
    uint32_t state_ts;                    // timestamp for last state_position increment

} gpsData_t;

gpsData_t gpsData;

static void gpsNewData(uint16_t c);
static bool gpsNewFrameNMEA(char c);
static bool gpsNewFrameUBLOX(uint8_t data);

static void gpsSetState(uint8_t state)
{
    gpsData.state = state;
    gpsData.state_position = 0;
    gpsData.state_ts = millis();
}

void gpsInit(uint8_t baudrateIndex)
{
    portMode_t mode = MODE_RXTX;

    // init gpsData structure. if we're not actually enabled, don't bother doing anything else
    gpsSetState(GPS_UNKNOWN);
    if (!feature(FEATURE_GPS))
        return;

    gpsData.baudrateIndex = baudrateIndex;
    gpsData.lastMessage = millis();
    gpsData.errors = 0;
    // only RX is needed for NMEA-style GPS
    if (mcfg.gps_type == GPS_NMEA)
        mode = MODE_RX;

    gpsSetPIDs();
    // Open GPS UART, no callback - buffer will be read out in gpsThread()
    core.gpsport = uartOpen(USART2, NULL, gpsInitData[baudrateIndex].baudrate, mode);
    // signal GPS "thread" to initialize when it gets to it
    gpsSetState(GPS_INITIALIZING);
}

void gpsInitHardware(void)
{
    switch (mcfg.gps_type) {
        case GPS_NMEA:
            // nothing to do, just set baud rate and try receiving some stuff and see if it parses
            serialSetBaudRate(core.gpsport, gpsInitData[gpsData.baudrateIndex].baudrate);
            gpsSetState(GPS_RECEIVINGDATA);
            break;

        case GPS_UBLOX:
            // UBX will run at mcfg.gps_baudrate, it shouldn't be "autodetected". So here we force it to that rate

            // Wait until GPS transmit buffer is empty
            if (!isSerialTransmitBufferEmpty(core.gpsport))
                break;

            if (gpsData.state == GPS_INITIALIZING) {
                uint32_t m = millis();
                if (m - gpsData.state_ts < GPS_BAUD_DELAY)
                    return;

                if (gpsData.state_position < GPS_INIT_ENTRIES) {
                    // try different speed to INIT
                    serialSetBaudRate(core.gpsport, gpsInitData[gpsData.state_position].baudrate);
                    // but print our FIXED init string for the baudrate we want to be at
                    serialPrint(core.gpsport, gpsInitData[gpsData.baudrateIndex].ubx);

                    gpsData.state_position++;
                    gpsData.state_ts = m;
                } else {
                    // we're now (hopefully) at the correct rate, next state will switch to it
                    gpsSetState(GPS_INITDONE);
                }
            } else {
                // GPS_INITDONE, set our real baud rate and push some ublox config strings
                if (gpsData.state_position == 0)
                    serialSetBaudRate(core.gpsport, gpsInitData[gpsData.baudrateIndex].baudrate);

                if (gpsData.state_position < sizeof(ubloxInit)) {
                    serialWrite(core.gpsport, ubloxInit[gpsData.state_position]); // send ubx init binary

                    gpsData.state_position++;
                } else {
                    // ublox should be init'd, time to try receiving some junk
                    gpsSetState(GPS_RECEIVINGDATA);
                }
            }
            break;
        case GPS_MTK_NMEA:
        case GPS_MTK_BINARY:
            // TODO. need to find my old piece of shit MTK GPS.
            break;
    }

    // clear error counter
    gpsData.errors = 0;
}

void gpsThread(void)
{
    // read out available GPS bytes
    if (core.gpsport) {
        while (serialTotalBytesWaiting(core.gpsport))
            gpsNewData(serialRead(core.gpsport));
    }

    switch (gpsData.state) {
        case GPS_UNKNOWN:
            break;

        case GPS_INITIALIZING:
        case GPS_INITDONE:
            gpsInitHardware();
            break;

        case GPS_LOSTCOMMS:
            gpsData.errors++;
            // try another rate
            gpsData.baudrateIndex++;
            gpsData.baudrateIndex %= GPS_INIT_ENTRIES;
            gpsData.lastMessage = millis();
            // TODO - move some / all of these into gpsData
            GPS_numSat = 0;
            f.GPS_FIX = 0;
            gpsSetState(GPS_INITIALIZING);
            break;

        case GPS_RECEIVINGDATA:
            // check for no data/gps timeout/cable disconnection etc
            if (millis() - gpsData.lastMessage > GPS_TIMEOUT) {
                // remove GPS from capability
                sensorsClear(SENSOR_GPS);
                gpsSetState(GPS_LOSTCOMMS);
            }
            break;
    }
}

static bool gpsNewFrame(uint8_t c)
{
    switch (mcfg.gps_type) {
        case GPS_NMEA:          // NMEA
        case GPS_MTK_NMEA:      // MTK in NMEA mode
            return gpsNewFrameNMEA(c);
        case GPS_UBLOX:         // UBX binary
            return gpsNewFrameUBLOX(c);
        case GPS_MTK_BINARY:    // MTK in BINARY mode (TODO)
            return false;
    }

    return false;
}



/*-----------------------------------------------------------
 *
 * Multiwii GPS code - revision: 1097
 *
 *-----------------------------------------------------------*/
#define POSHOLD_IMAX           20       // degrees
#define POSHOLD_RATE_IMAX      20       // degrees
#define NAV_IMAX               20       // degrees

/* GPS navigation can control the heading */
#define NAV_TAIL_FIRST             0    // true - copter comes in with tail first
#define NAV_SET_TAKEOFF_HEADING    1    // true - when copter arrives to home position it rotates it's head to takeoff direction

#define GPS_FILTERING              1    // add a 5 element moving average filter to GPS coordinates, helps eliminate gps noise but adds latency
#define GPS_LOW_SPEED_D_FILTER     1    // below .5m/s speed ignore D term for POSHOLD_RATE, theoretically this also removed D term induced noise

static bool check_missed_wp(void);
static void GPS_distance_cm_bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing);
//static void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing);
static void GPS_calc_longitude_scaling(int32_t lat);
static void GPS_calc_velocity(void);
static void GPS_calc_location_error(int32_t * target_lat, int32_t * target_lng, int32_t * gps_lat, int32_t * gps_lng);
static void GPS_calc_poshold(void);
static void GPS_calc_nav_rate(int max_speed);
static void GPS_update_crosstrack(void);
static bool UBLOX_parse_gps(void);
static int16_t GPS_calc_desired_speed(int16_t max_speed, bool _slow);
int32_t wrap_18000(int32_t error);
static int32_t wrap_36000(int32_t angle);

typedef struct {
    int16_t last_velocity;
} LeadFilter_PARAM;

void leadFilter_clear(LeadFilter_PARAM *param)
{
    param->last_velocity = 0;
}

int32_t leadFilter_getPosition(LeadFilter_PARAM *param, int32_t pos, int16_t vel, float lag_in_seconds)
{
    int16_t accel_contribution = (vel - param->last_velocity) * lag_in_seconds * lag_in_seconds;
    int16_t vel_contribution = vel * lag_in_seconds;

    // store velocity for next iteration
    param->last_velocity = vel;

    return pos + vel_contribution + accel_contribution;
}

LeadFilter_PARAM xLeadFilter;
LeadFilter_PARAM yLeadFilter;

typedef struct {
    float kP;
    float kI;
    float kD;
    float Imax;
} PID_PARAM;

static PID_PARAM posholdPID_PARAM;
static PID_PARAM poshold_ratePID_PARAM;
static PID_PARAM navPID_PARAM;

typedef struct {
    float integrator;          // integrator value
    int32_t last_input;        // last input for derivative
    float last_derivative;     // last derivative for low-pass filter
    float output;
    float derivative;
} PID;

static PID posholdPID[2];
static PID poshold_ratePID[2];
static PID navPID[2];

static int32_t get_P(int32_t error, PID_PARAM *pid)
{
    return (float)error * pid->kP;
}

static int32_t get_I(int32_t error, float *dt, PID *pid, PID_PARAM *pid_param)
{
    pid->integrator += ((float)error * pid_param->kI) * *dt;
    pid->integrator = constrain(pid->integrator, -pid_param->Imax, pid_param->Imax);
    return pid->integrator;
}

static int32_t get_D(int32_t input, float *dt, PID *pid, PID_PARAM *pid_param)
{
    pid->derivative = (input - pid->last_input) / *dt;

    // Low pass filter cut frequency for derivative calculation
    // Set to  "1 / ( 2 * PI * gps_lpf )"
#define PID_FILTER       (1.0f / (2.0f * M_PI * (float)cfg.gps_lpf))
    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    pid->derivative = pid->last_derivative + (*dt / (PID_FILTER + *dt)) * (pid->derivative - pid->last_derivative);
    // update state
    pid->last_input = input;
    pid->last_derivative = pid->derivative;
    // add in derivative component
    return pid_param->kD * pid->derivative;
}

static void reset_PID(PID *pid)
{
    pid->integrator = 0;
    pid->last_input = 0;
    pid->last_derivative = 0;
}

#define GPS_X 1
#define GPS_Y 0

/****************** PI and PID controllers for GPS ********************///32938 -> 33160

#define RADX100                    0.000174532925f
#define CROSSTRACK_GAIN            1
#define NAV_SLOW_NAV               true
#define NAV_BANK_MAX               3000 // 30deg max banking when navigating (just for security and testing)

static float dTnav;             // Delta Time in milliseconds for navigation computations, updated with every good GPS read
static int16_t actual_speed[2] = { 0, 0 };
static float GPS_scaleLonDown = 1.0f;  // this is used to offset the shrinking longitude as we go towards the poles

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
static int16_t rate_error[2];
static int32_t error[2];

// Currently used WP
static int32_t GPS_WP[2];

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t target_bearing;
////////////////////////////////////////////////////////////////////////////////
// Crosstrack
////////////////////////////////////////////////////////////////////////////////
// deg * 100, The original angle to the next_WP when the next_WP was set
// Also used to check when we pass a WP
static int32_t original_target_bearing;
// The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
static int16_t crosstrack_error;
////////////////////////////////////////////////////////////////////////////////
// The location of the copter in relation to home, updated every GPS read (1deg - 100)
//static int32_t home_to_copter_bearing;
// distance between plane and home in cm
//static int32_t home_distance;
// distance between plane and next_WP in cm
static uint32_t wp_distance;

// used for slow speed wind up when start navigation;
static int16_t waypoint_speed_gov;

////////////////////////////////////////////////////////////////////////////////////
// moving average filter variables
//
#define GPS_FILTER_VECTOR_LENGTH 5

static uint8_t GPS_filter_index = 0;
static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
static int32_t GPS_filter_sum[2];
static int32_t GPS_read[2];
static int32_t GPS_filtered[2];
static int32_t GPS_degree[2];   //the lat lon degree without any decimals (lat/10 000 000)
static uint16_t fraction3[2];

// This is the angle from the copter to the "next_WP" location
// with the addition of Crosstrack error in degrees * 100
static int32_t nav_bearing;
// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int16_t nav_takeoff_bearing;

static void gpsNewData(uint16_t c)
{
    int axis;
    static uint32_t nav_loopTimer;
    uint32_t dist;
    int32_t dir;
    int16_t speed;

    if (gpsNewFrame(c)) {
        // new data received and parsed, we're in business
        gpsData.lastLastMessage = gpsData.lastMessage;
        gpsData.lastMessage = millis();
        sensorsSet(SENSOR_GPS);
        if (GPS_update == 1)
            GPS_update = 0;
        else
            GPS_update = 1;
        if (f.GPS_FIX && GPS_numSat >= 5) {
            if (!f.ARMED)
                f.GPS_FIX_HOME = 0;
            if (!f.GPS_FIX_HOME && f.ARMED)
                GPS_reset_home_position();
            // Apply moving average filter to GPS data
#if defined(GPS_FILTERING)
            GPS_filter_index = (GPS_filter_index + 1) % GPS_FILTER_VECTOR_LENGTH;
            for (axis = 0; axis < 2; axis++) {
                GPS_read[axis] = GPS_coord[axis];               // latest unfiltered data is in GPS_latitude and GPS_longitude
                GPS_degree[axis] = GPS_read[axis] / 10000000;   // get the degree to assure the sum fits to the int32_t

                // How close we are to a degree line ? its the first three digits from the fractions of degree
                // later we use it to Check if we are close to a degree line, if yes, disable averaging,
                fraction3[axis] = (GPS_read[axis] - GPS_degree[axis] * 10000000) / 10000;

                GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
                GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis] * 10000000);
                GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
                GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis] * 10000000);
                if (nav_mode == NAV_MODE_POSHOLD) {             // we use gps averaging only in poshold mode...
                    if (fraction3[axis] > 1 && fraction3[axis] < 999)
                        GPS_coord[axis] = GPS_filtered[axis];
                }
            }
#endif
            // dTnav calculation
            // Time for calculating x,y speed and navigation pids
            dTnav = (float)(millis() - nav_loopTimer) / 1000.0f;
            nav_loopTimer = millis();
            // prevent runup from bad GPS
            dTnav = min(dTnav, 1.0f);

            // calculate distance and bearings for gui and other stuff continously - From home to copter
            GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dist, &dir);
            GPS_distanceToHome = dist / 100;
            GPS_directionToHome = dir / 100;

            if (!f.GPS_FIX_HOME) {      // If we don't have home set, do not display anything
                GPS_distanceToHome = 0;
                GPS_directionToHome = 0;
            }

            // calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
            GPS_calc_velocity();

            if (f.GPS_HOLD_MODE || f.GPS_HOME_MODE) { // ok we are navigating
                // do gps nav calculations here, these are common for nav and poshold
                GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);
                GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);

                switch (nav_mode) {
                case NAV_MODE_POSHOLD:
                    // Desired output is in nav_lat and nav_lon where 1deg inclination is 100
                    GPS_calc_poshold();
                    break;

                case NAV_MODE_WP:
                    speed = GPS_calc_desired_speed(cfg.nav_speed_max, NAV_SLOW_NAV);    // slow navigation
                    // use error as the desired rate towards the target
                    // Desired output is in nav_lat and nav_lon where 1deg inclination is 100
                    GPS_calc_nav_rate(speed);

                    // Tail control
                    if (cfg.nav_controls_heading) {
                        if (NAV_TAIL_FIRST) {
                            magHold = wrap_18000(nav_bearing - 18000) / 100;
                        } else {
                            magHold = nav_bearing / 100;
                        }
                    }
                    // Are we there yet ?(within x meters of the destination)
                    if ((wp_distance <= cfg.gps_wp_radius) || check_missed_wp()) {      // if yes switch to poshold mode
                        nav_mode = NAV_MODE_POSHOLD;
                        if (NAV_SET_TAKEOFF_HEADING) {
                            magHold = nav_takeoff_bearing;
                        }
                    }
                    break;
                }
            }                   //end of gps calcs
        }
    }
}

void GPS_reset_home_position(void)
{
    if (f.GPS_FIX && GPS_numSat >= 5) {
        GPS_home[LAT] = GPS_coord[LAT];
        GPS_home[LON] = GPS_coord[LON];
        GPS_calc_longitude_scaling(GPS_coord[LAT]); // need an initial value for distance and bearing calc
        nav_takeoff_bearing = heading;              // save takeoff heading
        // Set ground altitude
        f.GPS_FIX_HOME = 1;
    }
}

// reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav(void)
{
    int i;

    for (i = 0; i < 2; i++) {
        GPS_angle[i] = 0;
        nav_rated[i] = 0;
        nav[i] = 0;
        reset_PID(&posholdPID[i]);
        reset_PID(&poshold_ratePID[i]);
        reset_PID(&navPID[i]);
    }
}

// Get the relevant P I D values and set the PID controllers
void gpsSetPIDs(void)
{
    posholdPID_PARAM.kP = (float)cfg.P8[PIDPOS] / 100.0f;
    posholdPID_PARAM.kI = (float)cfg.I8[PIDPOS] / 100.0f;
    posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    poshold_ratePID_PARAM.kP = (float)cfg.P8[PIDPOSR] / 10.0f;
    poshold_ratePID_PARAM.kI = (float)cfg.I8[PIDPOSR] / 100.0f;
    poshold_ratePID_PARAM.kD = (float)cfg.D8[PIDPOSR] / 1000.0f;
    poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    navPID_PARAM.kP = (float)cfg.P8[PIDNAVR] / 10.0f;
    navPID_PARAM.kI = (float)cfg.I8[PIDNAVR] / 100.0f;
    navPID_PARAM.kD = (float)cfg.D8[PIDNAVR] / 1000.0f;
    navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
}

int8_t gpsSetPassthrough(void)
{
    if (gpsData.state != GPS_RECEIVINGDATA)
        return -1;

    LED0_OFF;
    LED1_OFF;

    while(1) {
        if (serialTotalBytesWaiting(core.gpsport)) {
            LED0_ON;
            serialWrite(core.mainport, serialRead(core.gpsport));
            LED0_OFF;
        }
        if (serialTotalBytesWaiting(core.mainport)) {
            LED1_ON;
            serialWrite(core.gpsport, serialRead(core.mainport));
            LED1_OFF;
        }
    }
}

// OK here is the onboard GPS code

////////////////////////////////////////////////////////////////////////////////////
// PID based GPS navigation functions
// Author : EOSBandi
// Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
// Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
static void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (abs((float)lat) / 10000000.0f) * 0.0174532925f;
    GPS_scaleLonDown = cosf(rads);
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t *lat, int32_t *lon)
{
    GPS_WP[LAT] = *lat;
    GPS_WP[LON] = *lon;

    GPS_calc_longitude_scaling(*lat);
    GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);

    nav_bearing = target_bearing;
    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);
    original_target_bearing = target_bearing;
    waypoint_speed_gov = cfg.nav_speed_min;
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp(void)
{
    int32_t temp;
    temp = target_bearing - original_target_bearing;
    temp = wrap_18000(temp);
    return (abs(temp) > 10000); // we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
static void GPS_distance_cm_bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing)
{
    float dLat = *lat2 - *lat1; // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown;
    *dist = sqrtf(sq(dLat) + sq(dLon)) * 1.113195f;

    *bearing = 9000.0f + atan2f(-dLat, dLon) * 5729.57795f;      // Convert the output radians to 100xdeg
    if (*bearing < 0)
        *bearing += 36000;
}

////////////////////////////////////////////////////////////////////////////////////
// keep old calculation function for compatibility (could be removed later) distance in meters, bearing in degree
//
//static void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing) {
//  uint32_t d1;
//  int32_t  d2;
//  GPS_distance_cm_bearing(&lat1,&lon1,&lat2,&lon2,&d1,&d2);
//  *dist = d1 / 100;          //convert to meters
//  *bearing = d2 /  100;      //convert to degrees
//}

////////////////////////////////////////////////////////////////////////////////////
// Calculate our current speed vector from gps position data
//
static void GPS_calc_velocity(void)
{
    static int16_t speed_old[2] = { 0, 0 };
    static int32_t last[2] = { 0, 0 };
    static uint8_t init = 0;
    // y_GPS_speed positve = Up
    // x_GPS_speed positve = Right

    if (init) {
        float tmp = 1.0f / dTnav;
        actual_speed[GPS_X] = (float)(GPS_coord[LON] - last[LON]) * GPS_scaleLonDown * tmp;
        actual_speed[GPS_Y] = (float)(GPS_coord[LAT] - last[LAT]) * tmp;

        actual_speed[GPS_X] = (actual_speed[GPS_X] + speed_old[GPS_X]) / 2;
        actual_speed[GPS_Y] = (actual_speed[GPS_Y] + speed_old[GPS_Y]) / 2;

        speed_old[GPS_X] = actual_speed[GPS_X];
        speed_old[GPS_Y] = actual_speed[GPS_Y];
    }
    init = 1;

    last[LON] = GPS_coord[LON];
    last[LAT] = GPS_coord[LAT];
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//      100     = 1m
//      1000    = 11m    = 36 feet
//      1800    = 19.80m = 60 feet
//      3000    = 33m
//      10000   = 111m
//
static void GPS_calc_location_error(int32_t *target_lat, int32_t *target_lng, int32_t *gps_lat, int32_t *gps_lng)
{
    error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;   // X Error
    error[LAT] = *target_lat - *gps_lat;        // Y Error
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
static void GPS_calc_poshold(void)
{
    int32_t d;
    int32_t target_speed;
    int axis;

    for (axis = 0; axis < 2; axis++) {
        target_speed = get_P(error[axis], &posholdPID_PARAM);       // calculate desired speed from lon error
        rate_error[axis] = target_speed - actual_speed[axis];       // calc the speed error

        nav[axis] = get_P(rate_error[axis], &poshold_ratePID_PARAM) +
                    get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
        d = get_D(error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
        d = constrain(d, -2000, 2000);

        // get rid of noise
#if defined(GPS_LOW_SPEED_D_FILTER)
        if (abs(actual_speed[axis]) < 50)
            d = 0;
#endif

        nav[axis] += d;
        nav[axis] = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        navPID[axis].integrator = poshold_ratePID[axis].integrator;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
//
static void GPS_calc_nav_rate(int max_speed)
{
    float trig[2];
    float temp;
    int axis;

    // push us towards the original track
    GPS_update_crosstrack();

    // nav_bearing includes crosstrack
    temp = (9000l - nav_bearing) * RADX100;
    trig[GPS_X] = cosf(temp);
    trig[GPS_Y] = sinf(temp);

    for (axis = 0; axis < 2; axis++) {
        rate_error[axis] = (trig[axis] * max_speed) - actual_speed[axis];
        rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
        // P + I + D
        nav[axis] = get_P(rate_error[axis], &navPID_PARAM) +
                    get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM) +
                    get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);

        nav[axis] = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        poshold_ratePID[axis].integrator = navPID[axis].integrator;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculating cross track error, this tries to keep the copter on a direct line
// when flying to a waypoint.
//
static void GPS_update_crosstrack(void)
{
    if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {     // If we are too far off or too close we don't do track following
        float temp = (target_bearing - original_target_bearing) * RADX100;
        crosstrack_error = sinf(temp) * (wp_distance * CROSSTRACK_GAIN); // Meters we are off track line
        nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
        nav_bearing = wrap_36000(nav_bearing);
    } else {
        nav_bearing = target_bearing;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow
// speed rampup when starting a navigation
//
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
//                 |                                        +|+
//                 |< we should slow to 1.5 m/s as we hit the target
//
static int16_t GPS_calc_desired_speed(int16_t max_speed, bool _slow)
{
    // max_speed is default 400 or 4m/s
    if (_slow) {
        max_speed = min(max_speed, wp_distance / 2);
    } else {
        max_speed = min(max_speed, wp_distance);
        max_speed = max(max_speed, cfg.nav_speed_min);      // go at least 100cm/s
    }

    // limit the ramp up of the speed
    // waypoint_speed_gov is reset to 0 at each new WP command
    if (max_speed > waypoint_speed_gov) {
        waypoint_speed_gov += (int)(100.0f * dTnav);    // increase at .5/ms
        max_speed = waypoint_speed_gov;
    }
    return max_speed;
}

////////////////////////////////////////////////////////////////////////////////////
// Utilities
//
int32_t wrap_18000(int32_t error)
{
    if (error > 18000)
        error -= 36000;
    if (error < -18000)
        error += 36000;
    return error;
}

static int32_t wrap_36000(int32_t angle)
{
    if (angle > 36000)
        angle -= 36000;
    if (angle < 0)
        angle += 36000;
    return angle;
}

// This code is used for parsing NMEA data

/* Alex optimization
  The latitude or longitude is coded this way in NMEA frames
  dm.f   coded as degrees + minutes + minute decimal
  Where:
    - d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
    - m is always 2 char long
    - f can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000

  EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased
  resolution also increased precision of nav calculations
static uint32_t GPS_coord_to_degrees(char *s)
{
    char *p = s, *d = s;
    uint8_t min, deg = 0;
    uint16_t frac = 0, mult = 10000;

    while (*p) {                // parse the string until its end
        if (d != s) {
            frac += (*p - '0') * mult;  // calculate only fractional part on up to 5 digits  (d != s condition is true when the . is located)
            mult /= 10;
        }
        if (*p == '.')
            d = p;              // locate '.' char in the string
        p++;
    }
    if (p == s)
        return 0;
    while (s < d - 2) {
        deg *= 10;              // convert degrees : all chars before minutes ; for the first iteration, deg = 0
        deg += *(s++) - '0';
    }
    min = *(d - 1) - '0' + (*(d - 2) - '0') * 10;       // convert minutes : 2 previous char before '.'
    return deg * 10000000UL + (min * 100000UL + frac) * 10UL / 6;
}
*/

#define DIGIT_TO_VAL(_x)    (_x - '0')
uint32_t GPS_coord_to_degrees(char* s)
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    unsigned int frac_min = 0;
    int i;

    // scan for decimal point or end of field
    for (p = s; isdigit((unsigned char)*p); p++) {
        if (p >= s + 15)
            return 0; // stop potential fail
    }
    q = s;

    // convert degrees
    while ((p - q) > 2) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }
    // convert minutes
    while (p > q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }
    // convert fractional minutes
    // expect up to four digits, result is in
    // ten-thousandths of a minute
    if (*p == '.') {
        q = p + 1;
        for (i = 0; i < 4; i++) {
            frac_min *= 10;
            if (isdigit((unsigned char)*q))
                frac_min += *q++ - '0';
        }
    }
    return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;
}

// helper functions
static uint32_t grab_fields(char *src, uint8_t mult)
{                               // convert string to uint32
    uint32_t i;
    uint32_t tmp = 0;
    for (i = 0; src[i] != 0; i++) {
        if (src[i] == '.') {
            i++;
            if (mult == 0)
                break;
            else
                src[i + mult] = 0;
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9')
            tmp += src[i] - '0';
        if (i >= 15)
            return 0; // out of bounds
    }
    return tmp;
}

static uint8_t hex_c(uint8_t n)
{                               // convert '0'..'9','A'..'F' to 0..15
    n -= '0';
    if (n > 9)
        n -= 7;
    n &= 0x0F;
    return n;
}

/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
     // added by Mis
     - GPS altitude (for OSD displaying)
     - GPS speed (for OSD displaying)
*/
#define FRAME_GGA  1
#define FRAME_RMC  2

static bool gpsNewFrameNMEA(char c)
{
    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, frame = 0;

    if (c == '$') {
        param = 0;
        offset = 0;
        parity = 0;
    } else if (c == ',' || c == '*') {
        string[offset] = 0;
        if (param == 0) {       //frame identification
            frame = 0;
            if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A')
                frame = FRAME_GGA;
            if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C')
                frame = FRAME_RMC;
        } else if (frame == FRAME_GGA) {
            if (param == 2) {
                GPS_coord[LAT] = GPS_coord_to_degrees(string);
            } else if (param == 3 && string[0] == 'S')
                GPS_coord[LAT] = -GPS_coord[LAT];
            else if (param == 4) {
                GPS_coord[LON] = GPS_coord_to_degrees(string);
            } else if (param == 5 && string[0] == 'W')
                GPS_coord[LON] = -GPS_coord[LON];
            else if (param == 6) {
                f.GPS_FIX = string[0] > '0';
            } else if (param == 7) {
                GPS_numSat = grab_fields(string, 0);
            } else if (param == 9) {
                GPS_altitude = grab_fields(string, 0);  // altitude in meters added by Mis
            }
        } else if (frame == FRAME_RMC) {
            if (param == 7) {
                GPS_speed = (grab_fields(string, 1) * 5144L) / 1000L;   // speed in cm/s added by Mis
            } else if (param == 8) {
                GPS_ground_course = grab_fields(string, 1);             // ground course deg * 10
            }
        }
        param++;
        offset = 0;
        if (c == '*')
            checksum_param = 1;
        else
            parity ^= c;
    } else if (c == '\r' || c == '\n') {
        if (checksum_param) {   // parity checksum
            uint8_t checksum = hex_c(string[0]);
            checksum <<= 4;
            checksum += hex_c(string[1]);
            if (checksum == parity)
                frameOK = 1;
        }
        checksum_param = 0;
    } else {
        if (offset < 15)
            string[offset++] = c;
        if (!checksum_param)
            parity ^= c;
    }
    return frameOK && (frame == FRAME_GGA);
}


// UBX support
typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} ubx_header;

typedef struct {
    uint32_t time;              // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
} ubx_nav_posllh;

typedef struct {
    uint32_t time;              // GPS msToW
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t differential_status;
    uint8_t res;
    uint32_t time_to_first_fix;
    uint32_t uptime;            // milliseconds
} ubx_nav_status;

typedef struct {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
} ubx_nav_solution;

typedef struct {
    uint32_t time;              // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
} ubx_nav_velned;

typedef struct {
    uint8_t chn;                // Channel number, 255 for SVx not assigned to channel
    uint8_t svid;               // Satellite ID
    uint8_t flags;              // Bitmask
    uint8_t quality;            // Bitfield
    uint8_t cno;                // Carrier to Noise Ratio (Signal Strength)
    uint8_t elev;               // Elevation in integer degrees
    int16_t azim;               // Azimuth in integer degrees
    int32_t prRes;              // Pseudo range residual in centimetres
} ubx_nav_svinfo_channel;

typedef struct {
    uint32_t time;              // GPS Millisecond time of week
    uint8_t numCh;              // Number of channels
    uint8_t globalFlags;        // Bitmask, Chip hardware generation 0:Antaris, 1:u-blox 5, 2:u-blox 6
    uint16_t reserved2;         // Reserved
    ubx_nav_svinfo_channel channel[16];         // 16 satellites * 12 byte
} ubx_nav_svinfo;

enum {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_SVINFO = 0x30,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
} ubs_protocol_bytes;

enum {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
} ubs_nav_fix_type;

enum {
    NAV_STATUS_FIX_VALID = 1
} ubx_nav_status_bits;

// Packet checksum accumulators
static uint8_t _ck_a;
static uint8_t _ck_b;

// State machine state
static uint8_t _step;
static uint8_t _msg_id;
static uint16_t _payload_length;
static uint16_t _payload_counter;

static bool next_fix;
static uint8_t _class;

// do we have new position information?
static bool _new_position;

// do we have new speed information?
static bool _new_speed;

// Receive buffer
static union {
    ubx_nav_posllh posllh;
    ubx_nav_status status;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    ubx_nav_svinfo svinfo;
    uint8_t bytes[200];
} _buffer;

void _update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    while (len--) {
        *ck_a += *data;
        *ck_b += *ck_a;
        data++;
    }
}

static bool gpsNewFrameUBLOX(uint8_t data)
{
    bool parsed = false;

    switch (_step) {
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
        case 0:
            if (PREAMBLE1 == data)
                _step++;
            break;
        case 2:
            _step++;
            _class = data;
            _ck_b = _ck_a = data;   // reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _payload_length = data; // payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _payload_length += (uint16_t)(data << 8);
            if (_payload_length > 512) {
                _payload_length = 0;
                _step = 0;
            }
            _payload_counter = 0;   // prepare to receive payload
            break;
        case 6:
            _ck_b += (_ck_a += data);       // checksum byte
            if (_payload_counter < sizeof(_buffer)) {
                _buffer.bytes[_payload_counter] = data;
            }
            if (++_payload_counter == _payload_length)
                _step++;
            break;
        case 7:
            _step++;
            if (_ck_a != data)
                _step = 0;          // bad checksum
            break;
        case 8:
            _step = 0;
            if (_ck_b != data)
                break;              // bad checksum
            if (UBLOX_parse_gps())
                parsed = true;
    }                           //end switch
    return parsed;
}

static bool UBLOX_parse_gps(void)
{
    int i;
    switch (_msg_id) {
    case MSG_POSLLH:
        //i2c_dataset.time                = _buffer.posllh.time;
        GPS_coord[LON] = _buffer.posllh.longitude;
        GPS_coord[LAT] = _buffer.posllh.latitude;
        GPS_altitude = _buffer.posllh.altitude_msl / 10 / 100;  //alt in m
        f.GPS_FIX = next_fix;
        _new_position = true;
        break;
    case MSG_STATUS:
        next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
        if (!next_fix)
            f.GPS_FIX = false;
        break;
    case MSG_SOL:
        next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
        if (!next_fix)
            f.GPS_FIX = false;
        GPS_numSat = _buffer.solution.satellites;
        // GPS_hdop                        = _buffer.solution.position_DOP;
        break;
    case MSG_VELNED:
        // speed_3d                        = _buffer.velned.speed_3d;  // cm/s
        GPS_speed = _buffer.velned.speed_2d;    // cm/s
        GPS_ground_course = (uint16_t) (_buffer.velned.heading_2d / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        _new_speed = true;
        break;
    case MSG_SVINFO:
        GPS_numCh = _buffer.svinfo.numCh;
        if (GPS_numCh > 16)
            GPS_numCh = 16;
        for (i = 0; i < GPS_numCh; i++){
            GPS_svinfo_chn[i]= _buffer.svinfo.channel[i].chn;
            GPS_svinfo_svid[i]= _buffer.svinfo.channel[i].svid;
            GPS_svinfo_quality[i]=_buffer.svinfo.channel[i].quality;
            GPS_svinfo_cno[i]= _buffer.svinfo.channel[i].cno;
        }
        break;
    default:
        return false;
    }

    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed) {
        _new_speed = _new_position = false;
        return true;
    }
    return false;
}
