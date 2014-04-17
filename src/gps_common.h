#pragma once

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
