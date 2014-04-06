/*
 * telemetry_hott.c
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra/cGiesen
 */

#include "board.h"
#include "mw.h"

#include "telemetry_hott.h"


const uint8_t kHoTTv4BinaryPacketSize = 45;
const uint8_t kHoTTv4TextPacketSize = 173;

static uint8_t outBuffer[173];
static void hottV4SerialWrite(uint8_t c);
static inline void hottV4EnableReceiverMode(void);
static inline void hottV4EnableTransmitterMode(void);

static void hottV4SendData(uint8_t *data, uint8_t size);
static void hottV4SendGPS(void);
static void hottV4GPSUpdate(void);
static void hottV4SendEAM(void);
static void hottV4EAMUpdateBattery(void);
static void hottV4EAMUpdateTemperatures(void);
bool batteryWarning;

/*
 * Sends HoTTv4 capable GPS telemetry frame.
 */

void hottV4SendGPS(void)
{
    /** Minimum data set for EAM */
    HoTTV4GPSModule.startByte = 0x7C;
    HoTTV4GPSModule.sensorID = HOTTV4_GPS_SENSOR_ID;
    HoTTV4GPSModule.sensorTextID = HOTTV4_GPS_SENSOR_TEXT_ID;
    HoTTV4GPSModule.endByte = 0x7D;
    /** ### */

    /** Reset alarms */
    HoTTV4GPSModule.alarmTone = 0x0;
    HoTTV4GPSModule.alarmInverse1 = 0x0;

    hottV4GPSUpdate();

    // Clear output buffer
    memset(&outBuffer, 0, sizeof(outBuffer));

    // Copy EAM data to output buffer
    memcpy(&outBuffer, &HoTTV4GPSModule, kHoTTv4BinaryPacketSize);

    // Send data from output buffer
    hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
  }

void hottV4GPSUpdate(void)
{
   //number of Satelites
   HoTTV4GPSModule.GPSNumSat=GPS_numSat;
   if (f.GPS_FIX > 0) {
     /** GPS fix */
     HoTTV4GPSModule.GPS_fix = 0x66; // Displays a 'f' for fix
     //latitude
     HoTTV4GPSModule.LatitudeNS=(GPS_coord[LAT]<0);
     uint8_t deg = GPS_coord[LAT] / 100000;
     uint32_t sec = (GPS_coord[LAT] - (deg * 100000)) * 6;
     uint8_t min = sec / 10000;
     sec = sec % 10000;
     uint16_t degMin = (deg * 100) + min;
     HoTTV4GPSModule.LatitudeMinLow = degMin;
     HoTTV4GPSModule.LatitudeMinHigh = degMin >> 8;
     HoTTV4GPSModule.LatitudeSecLow = sec;
     HoTTV4GPSModule.LatitudeSecHigh = sec >> 8;
     //latitude
     HoTTV4GPSModule.longitudeEW=(GPS_coord[LON]<0);
     deg = GPS_coord[LON] / 100000;
     sec = (GPS_coord[LON] - (deg * 100000)) * 6;
     min = sec / 10000;
     sec = sec % 10000;
     degMin = (deg * 100) + min;
     HoTTV4GPSModule.longitudeMinLow = degMin;
     HoTTV4GPSModule.longitudeMinHigh = degMin >> 8;
     HoTTV4GPSModule.longitudeSecLow = sec;
     HoTTV4GPSModule.longitudeSecHigh = sec >> 8;
     /** GPS Speed in km/h */
     uint16_t speed = (GPS_speed / 100) * 36; // 0.1m/s * 0.36 = km/h
     HoTTV4GPSModule.GPSSpeedLow = speed & 0x00FF;
     HoTTV4GPSModule.GPSSpeedHigh = speed >> 8;
     /** Distance to home */
     HoTTV4GPSModule.distanceLow = GPS_distanceToHome & 0x00FF;
     HoTTV4GPSModule.distanceHigh = GPS_distanceToHome >> 8;
     /** Altitude */
     HoTTV4GPSModule.altitudeLow = GPS_altitude & 0x00FF;
     HoTTV4GPSModule.altitudeHigh = GPS_altitude >> 8;
     /** Altitude */
     HoTTV4GPSModule.HomeDirection = GPS_directionToHome;
   }
   else
   {
     HoTTV4GPSModule.GPS_fix = 0x20; // Displays a ' ' to show nothing or clear the old value
   }
 }

   /**
   * Writes cell 1-4 high, low values and if not available
   * calculates vbat.
   */
  static void hottV4EAMUpdateBattery() {
//    HoTTV4ElectricAirModule.cell1L = 0;
//    HoTTV4ElectricAirModule.cell1H = 0;

//    HoTTV4ElectricAirModule.cell2L = 0;
//    HoTTV4ElectricAirModule.cell2H = 0;

//    HoTTV4ElectricAirModule.cell3L = 0;
//    HoTTV4ElectricAirModule.cell3H = 0;

//    HoTTV4ElectricAirModule.cell4L = 0;
//    HoTTV4ElectricAirModule.cell4H = 0;

    HoTTV4ElectricAirModule.driveVoltageLow = vbat & 0xFF;
    HoTTV4ElectricAirModule.driveVoltageHigh = vbat >> 8;
    HoTTV4ElectricAirModule.battery1Low = vbat & 0xFF;
    HoTTV4ElectricAirModule.battery1High = vbat >> 8;

//    HoTTV4ElectricAirModule.battery2Low = 0 & 0xFF;
//    HoTTV4ElectricAirModule.battery2High = 0 >> 8;

    if (batteryWarning) {
      HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationUndervoltage;
      HoTTV4ElectricAirModule.alarmInverse1 |= 0x80; // Invert Voltage display
    }
  }

  static void hottV4EAMUpdateTemperatures() {
    HoTTV4ElectricAirModule.temp1 = 20 + 0;
    HoTTV4ElectricAirModule.temp2 = 20;

    //if (HoTTV4ElectricAirModule.temp1 >= (20 + MultiHoTTModuleSettings.alarmTemp1)) {
    //  HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationMaxTemperature;
    //  HoTTV4ElectricAirModule.alarmInverse |= 0x8; // Invert Temp1 display
    //}
  }


/**
 * Sends HoTTv4 capable EAM telemetry frame.
 */
void hottV4SendEAM(void) {
  /** Minimum data set for EAM */
  HoTTV4ElectricAirModule.startByte = 0x7C;
  HoTTV4ElectricAirModule.sensorID = HOTTV4_ELECTRICAL_AIR_SENSOR_ID;
  HoTTV4ElectricAirModule.sensorTextID = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
  HoTTV4ElectricAirModule.endByte = 0x7D;
  /** ### */

  /** Reset alarms */
  HoTTV4ElectricAirModule.alarmTone = 0x0;
  HoTTV4ElectricAirModule.alarmInverse1 = 0x0;

  hottV4EAMUpdateBattery();
  hottV4EAMUpdateTemperatures();

  HoTTV4ElectricAirModule.current = 0 / 10;
  HoTTV4ElectricAirModule.height = OFFSET_HEIGHT + 0;
  HoTTV4ElectricAirModule.m2s = OFFSET_M2S;
  HoTTV4ElectricAirModule.m3s = OFFSET_M3S;

  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));

  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTV4ElectricAirModule, kHoTTv4BinaryPacketSize);

  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}

/**
 * Expects an array of at least size bytes. All bytes till size will be transmitted
 * to the HoTT capable receiver. Last byte will always be treated as checksum and is
 * calculated on the fly.
 */
static void hottV4SendData(uint8_t *data, uint8_t size) {
  //hottV4Serial.flush();

  // Protocoll specific waiting time
  // to avoid collisions
  delay(5);

  if (serialTotalBytesWaiting(core.telemport) == 0) {
    hottV4EnableTransmitterMode();

    uint16_t crc = 0;
    uint8_t i;

    for (i = 0; i < (size - 1); i++) {
      crc += data[i];
      hottV4SerialWrite(data[i]);

      // Protocoll specific delay between each transmitted byte
      delayMicroseconds(HOTTV4_TX_DELAY);
    }

    // Write package checksum
    hottV4SerialWrite(crc & 0xFF);

    hottV4EnableReceiverMode();
  }
}

/**
 * Writes out given byte to HoTT serial interface.
 * If in debug mode, data is also written to UART serial interface.
 */
static void hottV4SerialWrite(uint8_t c) {
  //hottV4Serial.write(c);
  serialWrite(core.telemport, c);
}

/**
 * Enables RX and disables TX
 */
static inline void hottV4EnableReceiverMode() {
  //DDRD &= ~(1 << HOTTV4_RXTX);
  //PORTD |= (1 << HOTTV4_RXTX);
}

/**
 * Enabels TX and disables RX
 */
static inline void hottV4EnableTransmitterMode() {
  //DDRD |= (1 << HOTTV4_RXTX);
}

void handleHoTTTelemetry(void)
{
    while (serialTotalBytesWaiting(core.telemport)) {
      uint8_t c = serialRead(core.telemport);
      switch (c) {
        case 0x8A:
            if (sensors(SENSOR_GPS)) hottV4SendGPS();
            break;
        case 0x8E:
            hottV4SendEAM();
            break;
        }
    }
}


