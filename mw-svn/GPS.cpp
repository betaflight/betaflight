/* this is an equirectangular approximation to calculate distance and bearing between 2 GPS points (lat/long)
   it's much more faster than an exact calculation
   the error is neglectible for few kilometers assuming a constant R for earth
   input: lat1/long1 <-> lat2/long2      unit: 1/100000 degree
   output: distance in meters, bearing in degrees
*/
void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t * dist, int16_t * bearing)
{
    float dLat = ((lat2 - lat1));       // difference of latitude in 1/100000 degrees
    float dLon = ((lon2 - lon1)) * cos(lat1 * (PI / 180 / 100000.0));   // difference of longitude in 1/100000 degrees
    *dist = 6372795 / 100000.0 * PI / 180 * (sqrt(sq(dLat) + sq(dLon)));
    if (lat1 != lat2)
        *bearing = 180 / PI * (atan2(dLon, dLat));
    else
        *bearing = 0;
}

/* The latitude or longitude is coded this way in NMEA frames
  dm.m   coded as degrees + minutes + minute decimal
  Where:
    - d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
    - m is always 2 char long
    - m can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 100 000
*/
uint32_t GPS_coord_to_degrees(char *s)
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
    min = *--d - '0' + (*--d - '0') * 10;
    sec = (m * min + dec) / 6;
    while (d != s) {
        m *= 10;
        *--d -= '0';
        sec += *d * m;
    }
    return sec;
}


/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
*/
bool GPS_newFrame(char c)
{
    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, GPGGA_frame = 0;

    if (c == '$') {
        param = 0;
        offset = 0;
        parity = 0;
    } else if (c == ',' || c == '*') {
        string[offset] = 0;
        if (param == 0) {       //frame identification
            if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A')
                GPGGA_frame = 1;
            else
                GPGGA_frame = 0;
        } else if (GPGGA_frame == 1) {
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
                if (offset > 1)
                    GPS_numSat = (string[0] - '0') * 10 + string[1] - '0';
                else
                    GPS_numSat = string[0] - '0';
            }
        }
        param++;
        offset = 0;
        if (c == '*')
            checksum_param = 1;
        else
            parity ^= c;
    } else if (c == '\r' || c == '\n') {
        if (checksum_param) {   //parity checksum
            uint8_t checksum = 16 * ((string[0] >= 'A') ? string[0] - 'A' + 10 : string[0] - '0') + ((string[1] >= 'A') ? string[1] - 'A' + 10 : string[1] - '0');
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
    return frameOK && GPGGA_frame;
}
