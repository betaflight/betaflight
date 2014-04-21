#pragma once

#define LAT 0
#define LON 1

typedef enum {
    GPS_NMEA = 0,
    GPS_UBLOX,
    GPS_MTK_NMEA,
    GPS_MTK_BINARY,
    GPS_MAG_BINARY,
    GPS_HARDWARE_MAX = GPS_MAG_BINARY,
} GPSHardware;

typedef enum {
    GPS_BAUD_115200 = 0,
    GPS_BAUD_57600,
    GPS_BAUD_38400,
    GPS_BAUD_19200,
    GPS_BAUD_9600,
    GPS_BAUD_MAX = GPS_BAUD_9600
} GPSBaudRates;

// Serial GPS only variables
// navigation mode
typedef enum NavigationMode
{
    NAV_MODE_NONE = 0,
    NAV_MODE_POSHOLD,
    NAV_MODE_WP
} NavigationMode;

typedef struct gpsProfile_s {
    uint16_t gps_wp_radius;                 // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
    uint8_t gps_lpf;                        // Low pass filter cut frequency for derivative calculation (default 20Hz)
    uint8_t nav_slew_rate;                  // Adds a rate control to nav output, will smoothen out nav angle spikes
    uint8_t nav_controls_heading;           // copter faces toward the navigation point, maghold must be enabled for it
    uint16_t nav_speed_min;                 // cm/sec
    uint16_t nav_speed_max;                 // cm/sec
    uint16_t ap_mode;                       // Temporarily Disables GPS_HOLD_MODE to be able to make it possible to adjust the Hold-position when moving the sticks, creating a deadspan for GPS
} gpsProfile_t;

extern int16_t GPS_angle[ANGLE_INDEX_COUNT];                // it's the angles that must be applied for GPS correction
extern int32_t GPS_coord[2];               // LAT/LON
extern int32_t GPS_home[2];
extern int32_t GPS_hold[2];
extern uint8_t GPS_numSat;
extern uint16_t GPS_distanceToHome;        // distance to home point in meters
extern int16_t GPS_directionToHome;        // direction to home or hol point in degrees
extern uint16_t GPS_altitude;              // altitude in 0.1m
extern uint16_t GPS_speed;                 // speed in 0.1m/s
extern uint16_t GPS_ground_course;         // degrees * 10

extern uint8_t GPS_update;                 // it's a binary toogle to distinct a GPS position update

extern uint8_t GPS_numCh;                  // Number of channels
extern uint8_t GPS_svinfo_chn[16];         // Channel number
extern uint8_t GPS_svinfo_svid[16];        // Satellite ID
extern uint8_t GPS_svinfo_quality[16];     // Bitfield Qualtity
extern uint8_t GPS_svinfo_cno[16];         // Carrier to Noise Ratio (Signal Strength)

extern int8_t nav_mode;                    // Navigation mode

void gpsThread(void);
int8_t gpsSetPassthrough(void);
void GPS_reset_home_position(void);
void GPS_reset_nav(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
void gpsUseProfile(gpsProfile_t *gpsProfileToUse);
void gpsUsePIDs(pidProfile_t *pidProfile);
void updateGpsStateForHomeAndHoldMode(void);
void updateGpsWaypointsAndMode(void);
void updateGpsIndicator(uint32_t currentTime);
