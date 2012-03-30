#include "board.h"
#include "mw.h"

#ifndef sq
#define sq(x) ((x)*(x))
#endif

static void GPS_NewData(uint16_t c);
static bool GPS_newFrame(char c);
static void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t * dist, int16_t * bearing);

void gpsInit(uint32_t baudrate)
{
    uart2Init(baudrate, GPS_NewData);
    sensorsSet(SENSOR_GPS);
}

/*-----------------------------------------------------------
 *
 * Multiwii GPS code
 *
 *-----------------------------------------------------------*/

static void GPS_NewData(uint16_t c)
{
    if (GPS_newFrame(c)) {
        if (GPS_update == 1)
            GPS_update = 0;
        else
            GPS_update = 1;
        if (GPS_fix == 1 && GPS_numSat > 3) {
            if (GPS_fix_home == 0) {
                GPS_fix_home = 1;
                GPS_latitude_home = GPS_latitude;
                GPS_longitude_home = GPS_longitude;
            }
            if (GPSModeHold == 1)
                GPS_distance(GPS_latitude_hold, GPS_longitude_hold, GPS_latitude, GPS_longitude, &GPS_distanceToHold, &GPS_directionToHold);
            else
                GPS_distance(GPS_latitude_home, GPS_longitude_home, GPS_latitude, GPS_longitude, &GPS_distanceToHome, &GPS_directionToHome);
        }
    }
}

/* this is an equirectangular approximation to calculate distance and bearing between 2 GPS points (lat/long)
   it's much more faster than an exact calculation
   the error is neglectible for few kilometers assuming a constant R for earth
   input: lat1/long1 <-> lat2/long2      unit: 1/100000 degree
   output: distance in meters, bearing in degrees
*/
static void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t * dist, int16_t * bearing)
{
    float dLat = (lat2 - lat1); // difference of latitude in 1/100000 degrees
    float dLon = (lon2 - lon1) * cosf(lat1 * (M_PI / 180 / 100000.0));     // difference of longitude in 1/100000 degrees
    *dist = 6372795.0 / 100000.0 * M_PI / 180.0 * (sqrtf(sq(dLat) + sq(dLon)));
    *bearing = 180.0 / M_PI * (atan2f(dLon, dLat));
}

/* The latitude or longitude is coded this way in NMEA frames
  dm.m   coded as degrees + minutes + minute decimal
  Where:
    - d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
    - m is always 2 char long
    - m can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 100 000
*/
static uint32_t GPS_coord_to_degrees(char *s)
{
    char *p, *d = s;
    uint32_t sec, m = 1000;
    uint16_t min, dec = 0;

    if (!*s)
        return 0;
    for (p = s; *p != 0; p++) {
        if (d != s) {
            *p -= '0';
            dec += *p * m;
            m /= 10;
        }
        if (*p == '.')
            d = p;
    }
    m = 10000;
    min = *--d - '0';
    min += (*--d - '0') * 10;
    sec = (m * min + dec) / 6;
    while (d != s) {
        m *= 10;
        *--d -= '0';
        sec += *d * m;
    }
    return sec;
}

// helper functions 
static uint16_t grab_fields(char *src, uint8_t mult)
{                               // convert string to uint16
    uint8_t i;
    uint16_t tmp = 0;
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

static bool GPS_newFrame(char c)
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
                GPS_latitude = GPS_coord_to_degrees(string);
            } else if (param == 3 && string[0] == 'S')
                GPS_latitude = -GPS_latitude;
            else if (param == 4) {
                GPS_longitude = GPS_coord_to_degrees(string);
            } else if (param == 5 && string[0] == 'W')
                GPS_longitude = -GPS_longitude;
            else if (param == 6) {
                GPS_fix = string[0] > '0';
            } else if (param == 7) {
                GPS_numSat = grab_fields(string, 0);
            } else if (param == 9) {
                GPS_altitude = grab_fields(string, 0);
            }                   // altitude in meters added by Mis
        } else if (frame == FRAME_RMC) {
            if (param == 7) {
                GPS_speed = ((uint32_t) grab_fields(string, 1) * 514444L) / 100000L;
            }                   // speed in cm/s added by Mis
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
