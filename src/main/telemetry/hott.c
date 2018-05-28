/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * telemetry_hott.c
 *
 * Authors:
 * Dominic Clifton - Hydra - Software Serial, Electronics, Hardware Integration and debugging, HoTT Code cleanup and fixes, general telemetry improvements.
 * Carsten Giesen - cGiesen - Baseflight port
 * Oliver Bayer - oBayer - MultiWii-HoTT, HoTT reverse engineering
 * Adam Majerczyk - HoTT-for-ardupilot from which some information and ideas are borrowed.
 *
 * https://github.com/obayer/MultiWii-HoTT
 * https://github.com/oBayer/MultiHoTT-Module
 * https://code.google.com/p/hott-for-ardupilot
 *
 * HoTT is implemented in Graupner equipment using a bi-directional protocol over a single wire.
 *
 * Generally the receiver sends a single request byte out using normal uart signals, then waits a short period for a
 * multiple byte response and checksum byte before it sends out the next request byte.
 * Each response byte must be send with a protocol specific delay between them.
 *
 * Serial ports use two wires but HoTT uses a single wire so some electronics are required so that
 * the signals don't get mixed up.  When cleanflight transmits it should not receive it's own transmission.
 *
 * Connect as follows:
 * HoTT TX/RX -> Serial RX (connect directly)
 * Serial TX -> 1N4148 Diode -(|  )-> HoTT TX/RX (connect via diode)
 *
 * The diode should be arranged to allow the data signals to flow the right way
 * -(|  )- == Diode, | indicates cathode marker.
 *
 * As noticed by Skrebber the GR-12 (and probably GR-16/24, too) are based on a PIC 24FJ64GA-002, which has 5V tolerant digital pins.
 *
 * Note: The softserial ports are not listed as 5V tolerant in the STM32F103xx data sheet pinouts and pin description
 * section.  Verify if you require a 5v/3.3v level shifters.  The softserial port should not be inverted.
 *
 * There is a technical discussion (in German) about HoTT here
 * http://www.rc-network.de/forum/showthread.php/281496-Graupner-HoTT-Telemetrie-Sensoren-Eigenbau-DIY-Telemetrie-Protokoll-entschl%C3%BCsselt/page21
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"


#ifdef USE_TELEMETRY

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/printf.h"
#include "common/time.h"

#include "drivers/serial.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "fc/runtime_config.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"

#include "flight/position.h"
#include "flight/pid.h"

#include "interface/settings.h"

#include "io/gps.h"
#include "io/vtx_control.h"
#include "io/vtx.h"
#include "io/vtx_string.h"

#include "sensors/battery.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"

#include "telemetry/hott.h"
#include "telemetry/telemetry.h"

//#define HOTT_DEBUG

#define HOTT_MESSAGE_PREPARATION_FREQUENCY_5_HZ ((1000 * 1000) / 5) // 200ms
#define HOTT_RX_SCHEDULE 5000  										// 5ms
#define HOTT_TX_DELAY_US 1000  										// 1ms
#define MILLISECONDS_IN_A_SECOND 1000

static const char * const TableDtermLowpassType[] = {
    "PT1",
    "BIQUAD",
//#if defined(USE_FIR_FILTER_DENOISE)
    "FIR"
//#endif
};
static uint32_t lastHoTTRequestCheckAt = 0;
static uint32_t lastMessagesPreparedAt = 0;
static uint32_t lastHottAlarmSoundTime = 0;

static bool hottIsSending = false;

static uint8_t *hottMsg = NULL;
static uint8_t hottMsgRemainingBytesToSendCount;
static uint8_t hottMsgCrc;

#define HOTT_CRC_SIZE (sizeof(hottMsgCrc))

#define HOTT_BAUDRATE 19200
#define HOTT_PORT_MODE MODE_RXTX // must be opened in RXTX so that TX and RX pins are allocated.

static serialPort_t *hottPort = NULL;
static serialPortConfig_t *portConfig;

static bool hottTelemetryEnabled =  false;
static portSharing_e hottPortSharing;

static HOTT_GPS_MSG_t hottGPSMessage;
static HOTT_EAM_MSG_t hottEAMMessage;

static HOTT_TEXTMODE_MSG_t hottTEXTMODEMessage;

static uint8_t page_settings = 1; // page number to display settings
static void initialiseEAMMessage(HOTT_EAM_MSG_t *msg, size_t size)
{
    memset(msg, 0, size);
    msg->start_byte = 0x7C;
    msg->eam_sensor_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
    msg->sensor_id = HOTT_TELEMETRY_EAM_SENSOR_TEXT; // 0x8e
    msg->stop_byte = 0x7D;
}

#ifdef USE_GPS
typedef enum {
    GPS_FIX_CHAR_NONE = '-',
    GPS_FIX_CHAR_2D = '2',
    GPS_FIX_CHAR_3D = '3',
    GPS_FIX_CHAR_DGPS = 'D'
} gpsFixChar_e;

static void initialiseGPSMessage(HOTT_GPS_MSG_t *msg, size_t size)
{
    memset(msg, 0, size);
    msg->start_byte = 0x7C;
    msg->gps_sensor_id = HOTT_TELEMETRY_GPS_SENSOR_ID;
    msg->sensor_id = HOTT_TELEMETRY_GPS_SENSOR_TEXT; // 0x8a
    msg->stop_byte = 0x7D;
}
#endif

static void initialiseMessages(void)
{
    initialiseEAMMessage(&hottEAMMessage, sizeof(hottEAMMessage));
#ifdef USE_GPS
    initialiseGPSMessage(&hottGPSMessage, sizeof(hottGPSMessage));
#endif
}

#ifdef USE_GPS
void addGPSCoordinates(HOTT_GPS_MSG_t *hottGPSMessage, int32_t latitude, int32_t longitude)
{
    int16_t deg = latitude / GPS_DEGREES_DIVIDER;
    int32_t sec = (latitude - (deg * GPS_DEGREES_DIVIDER)) * 6;
    int8_t min = sec / 1000000L;
    sec = (sec % 1000000L) / 100L;
    uint16_t degMin = (deg * 100L) + min;

    hottGPSMessage->pos_NS = (latitude < 0);
    hottGPSMessage->pos_NS_dm_L = degMin;
    hottGPSMessage->pos_NS_dm_H = degMin >> 8;
    hottGPSMessage->pos_NS_sec_L = sec;
    hottGPSMessage->pos_NS_sec_H = sec >> 8;

    deg = longitude / GPS_DEGREES_DIVIDER;
    sec = (longitude - (deg * GPS_DEGREES_DIVIDER)) * 6;
    min = sec / 1000000L;
    sec = (sec % 1000000L) / 100L;
    degMin = (deg * 100L) + min;

    hottGPSMessage->pos_EW = (longitude < 0);
    hottGPSMessage->pos_EW_dm_L = degMin;
    hottGPSMessage->pos_EW_dm_H = degMin >> 8;
    hottGPSMessage->pos_EW_sec_L = sec;
    hottGPSMessage->pos_EW_sec_H = sec >> 8;
}

void hottPrepareGPSResponse(HOTT_GPS_MSG_t *hottGPSMessage)
{
    hottGPSMessage->gps_satelites = gpsSol.numSat;

    if (!STATE(GPS_FIX)) {
        hottGPSMessage->gps_fix_char = GPS_FIX_CHAR_NONE;
        return;
    }

    if (gpsSol.numSat >= 5) {
        hottGPSMessage->gps_fix_char = GPS_FIX_CHAR_3D;
    } else {
        hottGPSMessage->gps_fix_char = GPS_FIX_CHAR_2D;
    }

    addGPSCoordinates(hottGPSMessage, gpsSol.llh.lat, gpsSol.llh.lon);

    // GPS Speed is returned in cm/s (from io/gps.c) and must be sent in km/h (Hott requirement)
    const uint16_t speed = (gpsSol.groundSpeed * 36) / 1000;
    hottGPSMessage->gps_speed_L = speed & 0x00FF;
    hottGPSMessage->gps_speed_H = speed >> 8;

    hottGPSMessage->home_distance_L = GPS_distanceToHome & 0x00FF;
    hottGPSMessage->home_distance_H = GPS_distanceToHome >> 8;

    uint16_t altitude = gpsSol.llh.alt;
    if (!STATE(GPS_FIX)) {
        altitude = getEstimatedAltitude() / 100;
    }

    const uint16_t hottGpsAltitude = (altitude) + HOTT_GPS_ALTITUDE_OFFSET; // gpsSol.llh.alt in m ; offset = 500 -> O m

    hottGPSMessage->altitude_L = hottGpsAltitude & 0x00FF;
    hottGPSMessage->altitude_H = hottGpsAltitude >> 8;

    hottGPSMessage->home_direction = GPS_directionToHome;
}
#endif

static bool shouldTriggerBatteryAlarmNow(void)
{
    return ((millis() - lastHottAlarmSoundTime) >= (telemetryConfig()->hottAlarmSoundInterval * MILLISECONDS_IN_A_SECOND));
}

static inline void updateAlarmBatteryStatus(HOTT_EAM_MSG_t *hottEAMMessage)
{
    if (shouldTriggerBatteryAlarmNow()) {
        lastHottAlarmSoundTime = millis();
        const batteryState_e batteryState = getBatteryState();
        if (batteryState == BATTERY_WARNING  || batteryState == BATTERY_CRITICAL) 
			{
            hottEAMMessage->warning_beeps = 0x10;
            hottEAMMessage->alarm_invers1 = HOTT_EAM_ALARM1_FLAG_BATTERY_1;
			}
		else if (getMAhDrawn() > batteryConfig()->batteryCapacity)
			{
            hottEAMMessage->warning_beeps = 0x16;
            hottEAMMessage->alarm_invers1 = HOTT_EAM_ALARM1_FLAG_MAH;
			}		
		else 
			{
            hottEAMMessage->warning_beeps = HOTT_EAM_ALARM1_FLAG_NONE;
            hottEAMMessage->alarm_invers1 = HOTT_EAM_ALARM1_FLAG_NONE;
			}
    }
}

static inline void hottEAMUpdateBattery(HOTT_EAM_MSG_t *hottEAMMessage)
{
    hottEAMMessage->main_voltage_L = getBatteryVoltage() & 0xFF;
    hottEAMMessage->main_voltage_H = getBatteryVoltage() >> 8;
    hottEAMMessage->batt1_voltage_L = getBatteryVoltage() & 0xFF;
    hottEAMMessage->batt1_voltage_H = getBatteryVoltage() >> 8;

    updateAlarmBatteryStatus(hottEAMMessage);
}

static inline void hottEAMUpdateCurrentMeter(HOTT_EAM_MSG_t *hottEAMMessage)
{
    const int32_t amp = getAmperage() / 10;
    hottEAMMessage->current_L = amp & 0xFF;
    hottEAMMessage->current_H = amp >> 8;
}

static inline void hottEAMUpdateBatteryDrawnCapacity(HOTT_EAM_MSG_t *hottEAMMessage)
{
    const int32_t mAh = getMAhDrawn() / 10;
    hottEAMMessage->batt_cap_L = mAh & 0xFF;
    hottEAMMessage->batt_cap_H = mAh >> 8;
}

static inline void hottEAMUpdateAltitude(HOTT_EAM_MSG_t *hottEAMMessage)
{
    const uint16_t hottEamAltitude = (getEstimatedAltitude() / 100) + HOTT_EAM_OFFSET_HEIGHT;

    hottEAMMessage->altitude_L = hottEamAltitude & 0x00FF;
    hottEAMMessage->altitude_H = hottEamAltitude >> 8;
}

static inline void hottEAMUpdateClimbrate(HOTT_EAM_MSG_t *hottEAMMessage)
{
    const int32_t vario = getEstimatedVario();
    hottEAMMessage->climbrate_L = (30000 + vario) & 0x00FF;
    hottEAMMessage->climbrate_H = (30000 + vario) >> 8;
    hottEAMMessage->climbrate3s = 120 + (vario / 100);
}

void hottPrepareEAMResponse(HOTT_EAM_MSG_t *hottEAMMessage)
{
    // Reset alarms
    hottEAMMessage->warning_beeps = 0x0;
    hottEAMMessage->alarm_invers1 = 0x0;

    hottEAMUpdateBattery(hottEAMMessage);
    hottEAMUpdateCurrentMeter(hottEAMMessage);
    hottEAMUpdateBatteryDrawnCapacity(hottEAMMessage);
    hottEAMUpdateAltitude(hottEAMMessage);
    hottEAMUpdateClimbrate(hottEAMMessage);
}

static void HottInvertLigne(HOTT_TEXTMODE_MSG_t *hottTEXTMODEMessage,int ligne) {
  if (ligne>= 0 && ligne<= 7)
    for(int i=0; i< 21; i++) {
        if (hottTEXTMODEMessage->text[ligne][i] == 0)   //reversing the null character (end of string)
            hottTEXTMODEMessage->text[ligne][i] = (0x80 + 0x20);
        else
            hottTEXTMODEMessage->text[ligne][i] = (0x80 + hottTEXTMODEMessage->text[ligne][i]);
    }
}

static void HottWriteLine(HOTT_TEXTMODE_MSG_t *hottTEXTMODEMessage,uint8_t line, const char *text) {
  uint8_t writeText = 1;

  for (uint8_t index = 0; index < 21; index++) {
    if (0x0 != text[index] && writeText) {
       hottTEXTMODEMessage->text[line][index] = text[index];
    } else {
      writeText = 0;
      hottTEXTMODEMessage->text[line][index] = ' ';
    }
  }
}
static void hottSerialWrite(uint8_t c)
{
    static uint8_t serialWrites = 0;
    serialWrites++;
    serialWrite(hottPort, c);
}

void freeHoTTTelemetryPort(void)
{
    closeSerialPort(hottPort);
    hottPort = NULL;
    hottTelemetryEnabled = false;
}

void initHoTTTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_HOTT);
    hottPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_HOTT);

    initialiseMessages();
}

static void flushHottRxBuffer(void)
{
    while (serialRxBytesWaiting(hottPort) > 0) {
        serialRead(hottPort);
    }
}

static void workAroundForHottTelemetryOnUsart(serialPort_t *instance, portMode_e mode)
{
    closeSerialPort(hottPort);

    portOptions_e portOptions = telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED;

    if (telemetryConfig()->halfDuplex) {
        portOptions |= SERIAL_BIDIR;
    }

    hottPort = openSerialPort(instance->identifier, FUNCTION_TELEMETRY_HOTT, NULL, NULL, HOTT_BAUDRATE, mode, portOptions);
}

static bool hottIsUsingHardwareUART(void)
{
    return !(portConfig->identifier == SERIAL_PORT_SOFTSERIAL1 || portConfig->identifier == SERIAL_PORT_SOFTSERIAL2);
}

static void hottConfigurePortForTX(void)
{
    // FIXME temorary workaround for HoTT not working on Hardware serial ports due to hardware/softserial serial port initialisation differences
    if (hottIsUsingHardwareUART()) {
        workAroundForHottTelemetryOnUsart(hottPort, MODE_TX);
    } else {
        serialSetMode(hottPort, MODE_TX);
    }
    hottIsSending = true;
    hottMsgCrc = 0;
}

static void hottConfigurePortForRX(void)
{
    // FIXME temorary workaround for HoTT not working on Hardware serial ports due to hardware/softserial serial port initialisation differences
    if (hottIsUsingHardwareUART()) {
        workAroundForHottTelemetryOnUsart(hottPort, MODE_RX);
    } else {
        serialSetMode(hottPort, MODE_RX);
    }
    hottMsg = NULL;
    hottIsSending = false;
    flushHottRxBuffer();
}

void configureHoTTTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    portOptions_e portOptions = SERIAL_NOT_INVERTED;

    if (telemetryConfig()->halfDuplex) {
        portOptions |= SERIAL_BIDIR;
    }

    hottPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_HOTT, NULL, NULL, HOTT_BAUDRATE, HOTT_PORT_MODE, portOptions);

    if (!hottPort) {
        return;
    }

    hottConfigurePortForRX();

    hottTelemetryEnabled = true;
}

static void hottSendResponse(uint8_t *buffer, int length)
{
    if (hottIsSending) {
        return;
    }

    hottMsg = buffer;
    hottMsgRemainingBytesToSendCount = length + HOTT_CRC_SIZE;
}

static inline void hottSendGPSResponse(void)
{
    hottSendResponse((uint8_t *)&hottGPSMessage, sizeof(hottGPSMessage));
}

static inline void hottSendEAMResponse(void)
{
    hottSendResponse((uint8_t *)&hottEAMMessage, sizeof(hottEAMMessage));
}

static inline void hottSendTEXTMODEResponse(void)
{
    hottSendResponse((uint8_t *)&hottTEXTMODEMessage, sizeof(hottTEXTMODEMessage));
}
static void hottPrepareMessages(void) {
    hottPrepareEAMResponse(&hottEAMMessage);
#ifdef USE_GPS
    hottPrepareGPSResponse(&hottGPSMessage);
#endif
}

static void processBinaryModeRequest(uint8_t address)
{

#ifdef HOTT_DEBUG
    static uint8_t hottBinaryRequests = 0;
    static uint8_t hottGPSRequests = 0;
    static uint8_t hottEAMRequests = 0;
#endif

    switch (address) {
#ifdef USE_GPS
    case 0x8A:
#ifdef HOTT_DEBUG
        hottGPSRequests++;
#endif
        if (sensors(SENSOR_GPS)) {
            hottSendGPSResponse();
        }
        break;
#endif
    case 0x8E:
#ifdef HOTT_DEBUG
        hottEAMRequests++;
#endif
        hottSendEAMResponse();
        break;
    }

#ifdef HOTT_DEBUG
    hottBinaryRequests++;
    debug[0] = hottBinaryRequests;
#ifdef USE_GPS
    debug[1] = hottGPSRequests;
#endif
    debug[2] = hottEAMRequests;
#endif
}

static void processTextModeRequest(HOTT_TEXTMODE_MSG_t *hottTEXTMODEMessage,uint8_t octet3)
{
	uint8_t id_sensor = (octet3 >> 4);
	uint8_t id_key = octet3 & 0x0f;
	static uint8_t ligne_select = 4 ;
	static int8_t ligne_edit = -1 ;
	char lineText[21];

	hottTEXTMODEMessage->start_byte = 0x7b;
	hottTEXTMODEMessage->esc = 0x00;
	hottTEXTMODEMessage->warning_beeps = 0x00;
	memset((char *)&hottTEXTMODEMessage->text, 0x20, HOTT_TEXTMODE_MSG_TEXT_LEN);
	hottTEXTMODEMessage->stop_byte = 0x7D;
	
	if (id_key == HOTTV4_BUTTON_LEFT && page_settings == 1)
        {   
          hottTEXTMODEMessage->esc = 0x01;
        }
        else
        {
          if (id_sensor == (HOTT_TELEMETRY_EAM_SENSOR_ID & 0x0f) && (!ARMING_FLAG(ARMED))) // Only Text modus when "No ARMED"
            {  
            hottTEXTMODEMessage->esc = HOTT_TELEMETRY_EAM_SENSOR_TEXT;
			
			    // Button Abfrage 
                if (ligne_edit == -1) //keine line gewählt
                {            
                    switch (id_key) {
						case HOTTV4_BUTTON_LEFT:
							{
							page_settings-=1;
							if (page_settings <1)    // unter Seite 1 dann Seite 2
							page_settings = 1;
							break;	
							}
						case HOTTV4_BUTTON_RIGHT:
							{	   
							page_settings+=1;
							if (page_settings >8)   // Über Seite 8 dann Seite 1
								page_settings = 1;
							break;	
						}
						case HOTTV4_BUTTON_UP: 
							{   
							ligne_select+=1;
							if (ligne_select>7)
								ligne_select =  1;
							break;
						}
						case HOTTV4_BUTTON_DOWN:
							{   
							ligne_select-=1;
							if (ligne_select<1)
								ligne_select =  1;
							break;
							}
						case HOTTV4_BUTTON_SET:
							{   
							ligne_edit =  ligne_select ;
							break;
							}  
					}			
                }    
                else if (ligne_edit != -1)
                {
					switch (id_key) {
						case HOTTV4_BUTTON_RIGHT:
							{
							break;
							}
						case HOTTV4_BUTTON_LEFT:
							{   
							break;
							}
						case HOTTV4_BUTTON_UP:
							{   
							if (page_settings == PID_Werte_1) {
								switch (ligne_select) {
									case 1:
										changePidProfile(getCurrentPidProfileIndex() + 1);
									break;
									case 2:
										currentPidProfile->pid[0].P +=1;
									break;
									case 3:
										currentPidProfile->pid[0].I +=1;
									break;
									case 4:
										currentPidProfile->pid[0].D +=1;
									break;
									case 5:
										currentPidProfile->pid[1].P +=1;
									break;
									case 6:
										currentPidProfile->pid[1].I +=1;
									break;
									case 7:
										currentPidProfile->pid[1].D +=1;
									break;   
								} 
							}
							else if (page_settings == PID_Werte_2) {
								switch (ligne_select) {
									case 1:
										changePidProfile(getCurrentPidProfileIndex() + 1);
									break;
									case 2:
										currentPidProfile->pid[2].P +=1;
									break;
									case 3:
										currentPidProfile->pid[2].I +=1;
									break; 
								} 
							}        
							else if (page_settings == RC_Rates_1) {
								switch (ligne_select) {
									case 1:
										changeControlRateProfile(getCurrentControlRateProfileIndex() + 1);
									break;
									case 2:
										currentControlRateProfile->rcRates[FD_ROLL] +=1;
									break;
									case 3:
										currentControlRateProfile->rcRates[FD_YAW] +=1;
									break;
									case 4:
										currentControlRateProfile->rates[0] +=1;
									break;
									case 5:
										currentControlRateProfile->rates[1] +=1;
									break;
									case 6:
										currentControlRateProfile->rates[2] +=1;
									break;  
								} 
							}
							else if (page_settings == RC_Rates_2) {
								switch (ligne_select) {
									case 1:
										changeControlRateProfile(getCurrentControlRateProfileIndex() + 1);
									break;
									case 2:
										currentControlRateProfile->rcExpo[FD_ROLL] +=1;
									break;
									case 3:
										currentControlRateProfile->rcExpo[FD_YAW] +=1;
									break;
								} 
							}
							else if (page_settings == VTX_Config) {
								switch (ligne_select) {
									case 1:
										vtxSettingsConfigMutable()->band = vtxSettingsConfig()->band + 1;
									break;
									case 2:
										vtxSettingsConfigMutable()->channel = vtxSettingsConfig()->channel + 1;
									break;
									case 3:
										vtxSettingsConfigMutable()->power = vtxSettingsConfig()->power + 1;
									break;
								} 
							}
                            else if (page_settings == FILTER_1) {
								switch (ligne_select) {
									case 1:
										currentPidProfile->dterm_filter_type +=1;
									break;
									case 2:
										gyroConfigMutable()->gyro_lowpass_hz = gyroConfig()->gyro_lowpass_hz + 1;
									break;
									case 3:
										gyroConfigMutable()->gyro_soft_notch_hz_1 = gyroConfig()->gyro_soft_notch_hz_1 + 1;
									break;
									case 4:
										gyroConfigMutable()->gyro_soft_notch_cutoff_1 = gyroConfig()->gyro_soft_notch_cutoff_1 + 1;
									break;
									case 5:
										gyroConfigMutable()->gyro_soft_notch_hz_2 = gyroConfig()->gyro_soft_notch_hz_2 + 1;
									break;
									case 6:
										gyroConfigMutable()->gyro_soft_notch_cutoff_2 = gyroConfig()->gyro_soft_notch_cutoff_2 + 1;
									break;
								} 
							}		
							else if (page_settings == FILTER_2) {
								switch (ligne_select) {
									case 1:
										currentPidProfile->dterm_filter_type +=1;
									break;
									case 2:
										currentPidProfile->dterm_lowpass_hz +=1;
									break;
									case 3:
										currentPidProfile->dterm_notch_hz +=1;
									break;
									case 4:											
										currentPidProfile->dterm_notch_cutoff +=1;
									break;
									case 5:								
										currentPidProfile->yaw_lowpass_hz +=1;
									break;
									case 6:
										currentControlRateProfile->tpa_breakpoint +=1;
									break;
								} 
							}	
							else if (page_settings == Battery) {
								switch (ligne_select) {
									case 1:
										batteryConfigMutable()->vbatwarningcellvoltage = batteryConfig()->vbatwarningcellvoltage + 1;
									break;
									case 2:
										batteryConfigMutable()->batteryCapacity = batteryConfig()->batteryCapacity + 10;
									break;
									case 3:
										voltageSensorADCConfigMutable(0)->vbatscale = voltageSensorADCConfig(0)->vbatscale + 1;
									break;
									case 4:											
										currentSensorADCConfigMutable()->scale = currentSensorADCConfig()->scale + 1;
									break;
								} 
							}	
							break;	
							}
						case HOTTV4_BUTTON_DOWN:
							{   
							if (page_settings == PID_Werte_1) {
								switch (ligne_select) {
									case 1:
										changePidProfile(getCurrentPidProfileIndex() - 1);
									break;
									case 2:
										currentPidProfile->pid[0].P -=1;
									break;
									case 3:
										currentPidProfile->pid[0].I -=1;
									break;
									case 4:
										currentPidProfile->pid[0].D -=1;
									break;
									case 5:
										currentPidProfile->pid[1].P -=1;
									break;
									case 6:
										currentPidProfile->pid[1].I -=1;
									break;
									case 7:
										currentPidProfile->pid[1].D -=1;
									break;   
								}  
							}
							else if (page_settings == PID_Werte_2) {
								switch (ligne_select) {
									case 1:
										changePidProfile(getCurrentPidProfileIndex() - 1);
									break;
									case 2:
										currentPidProfile->pid[2].P -=1;
									break;
									case 3:
										currentPidProfile->pid[2].I -=1;
									break; 
								} 
							}        
							else if (page_settings == RC_Rates_1) {
								switch (ligne_select) {
									case 1:
										changeControlRateProfile(getCurrentControlRateProfileIndex() - 1);
									break;
									case 2:
										currentControlRateProfile->rcRates[FD_ROLL] -=1;
									break;
									case 3:
										currentControlRateProfile->rcRates[FD_YAW] -=1;
									break;
									case 4:
										currentControlRateProfile->rates[0] -=1;
									break;
									case 5:
										currentControlRateProfile->rates[1] -=1;
									break;
									case 6:
										currentControlRateProfile->rates[2] -=1;
									break;  
								} 
							}
							else if (page_settings == RC_Rates_2) {
								switch (ligne_select) {
									case 1:
										changeControlRateProfile(getCurrentControlRateProfileIndex() - 1);
									break;
									case 2:
										currentControlRateProfile->rcExpo[FD_ROLL] -=1;
									break;
									case 3:
										currentControlRateProfile->rcExpo[FD_YAW] -=1;
									break;
								} 
							}  
							else if (page_settings == VTX_Config) {
								switch (ligne_select) {
									case 1:
										vtxSettingsConfigMutable()->band = vtxSettingsConfig()->band - 1;
									break;
									case 2:
										vtxSettingsConfigMutable()->channel = vtxSettingsConfig()->channel - 1;
									break;
									case 3:
										vtxSettingsConfigMutable()->power = vtxSettingsConfig()->power - 1;
									break;
								} 
							}
                            else if (page_settings == FILTER_1) {
								switch (ligne_select) {
									case 1:
										currentPidProfile->dterm_filter_type -=1;
									break;
									case 2:
										gyroConfigMutable()->gyro_lowpass_hz = gyroConfig()->gyro_lowpass_hz - 1;
									break;
									case 3:
										gyroConfigMutable()->gyro_soft_notch_hz_1 = gyroConfig()->gyro_soft_notch_hz_1 - 1;
									break;
									case 4:
										gyroConfigMutable()->gyro_soft_notch_cutoff_1 = gyroConfig()->gyro_soft_notch_cutoff_1 - 1;
									break;
									case 5:
										gyroConfigMutable()->gyro_soft_notch_hz_2 = gyroConfig()->gyro_soft_notch_hz_2 - 1;
									break;
									case 6:
										gyroConfigMutable()->gyro_soft_notch_cutoff_2 = gyroConfig()->gyro_soft_notch_cutoff_2 - 1;
									break;
								} 
							}		
							else if (page_settings == FILTER_2) {
								switch (ligne_select) {
									case 1:
										currentPidProfile->dterm_filter_type -=1;
									break;
									case 2:
										currentPidProfile->dterm_lowpass_hz -=1;
									break;
									case 3:
										currentPidProfile->dterm_notch_hz -=1;
									break;
									case 4:											
										currentPidProfile->dterm_notch_cutoff -=1;
									break;
									case 5:								
										currentPidProfile->yaw_lowpass_hz -=1;
									break;
									case 6:
										currentControlRateProfile->tpa_breakpoint -=1;
									break;	
								} 
							}		
							else if (page_settings == Battery) {
								switch (ligne_select) {
									case 1:
										batteryConfigMutable()->vbatwarningcellvoltage = batteryConfig()->vbatwarningcellvoltage - 1;
									break;
									case 2:
										batteryConfigMutable()->batteryCapacity = batteryConfig()->batteryCapacity - 10;
									break;
									case 3:
										voltageSensorADCConfigMutable(0)->vbatscale = voltageSensorADCConfig(0)->vbatscale - 1;
									break;
									case 4:											
										currentSensorADCConfigMutable()->scale = currentSensorADCConfig()->scale - 1;
									break;
								} 
							}	
							break;			
							}
						case HOTTV4_BUTTON_SET:
							{   
								ligne_edit = -1 ;
								writeEEPROM();
								readEEPROM();
							}	  
					}   
				}
			switch (page_settings) { //SETTINGS
				
				case PID_Werte_1: //PAGE PID-Werte 1
					{                   //123456789|123456789|1            
                    tfp_sprintf(lineText,"PID-Werte 1 v%5s <>",FC_VERSION_STRING);
                    HottWriteLine(hottTEXTMODEMessage,0,lineText);
                    tfp_sprintf(lineText," Profile     :%3d    ",getCurrentPidProfileIndex() + 1);
                    HottWriteLine(hottTEXTMODEMessage,1,lineText);
                    tfp_sprintf(lineText," ROLL      P :%3d    ",currentPidProfile->pid[0].P);
                    HottWriteLine(hottTEXTMODEMessage,2,lineText);
                    tfp_sprintf(lineText,"           I :%3d    ",currentPidProfile->pid[0].I);
                    HottWriteLine(hottTEXTMODEMessage,3,lineText);
                    tfp_sprintf(lineText,"           D :%3d    ",currentPidProfile->pid[0].D);
                    HottWriteLine(hottTEXTMODEMessage,4,lineText);
                    tfp_sprintf(lineText," PITCH     P :%3d    ",currentPidProfile->pid[1].P);
                    HottWriteLine(hottTEXTMODEMessage,5,lineText);
                    tfp_sprintf(lineText,"           I :%3d    ",currentPidProfile->pid[1].I);
                    HottWriteLine(hottTEXTMODEMessage,6,lineText);
                    tfp_sprintf(lineText,"           D :%3d    ",currentPidProfile->pid[1].D);
                    HottWriteLine(hottTEXTMODEMessage,7,lineText);
					break;
					}//END PAGE
				case PID_Werte_2: //PAGE PID-Werte 2
					{                   //123456789|123456789|1
                    tfp_sprintf(lineText,"PID-Werte 2 v%5s <>",FC_VERSION_STRING);
                    HottWriteLine(hottTEXTMODEMessage,0,lineText);
                    tfp_sprintf(lineText," Profile     :%3d    ",getCurrentPidProfileIndex() + 1);
                    HottWriteLine(hottTEXTMODEMessage,1,lineText);
                    tfp_sprintf(lineText," YAW       P :%3d    ",currentPidProfile->pid[2].P);
                    HottWriteLine(hottTEXTMODEMessage,2,lineText);
                    tfp_sprintf(lineText,"           I :%3d    ",currentPidProfile->pid[2].I);
                    HottWriteLine(hottTEXTMODEMessage,3,lineText); 
					break;
					}//END PAGE
				case RC_Rates_1: //PAGE RC-Rates 1 
					{                   //123456789|123456789|1
                    tfp_sprintf(lineText,"RC-Rates 1  v%5s <>",FC_VERSION_STRING);
                    HottWriteLine(hottTEXTMODEMessage,0,lineText);
                    tfp_sprintf(lineText," Profile Rates:%3d   ",getCurrentControlRateProfileIndex() + 1);
                    HottWriteLine(hottTEXTMODEMessage,1,lineText);
                    tfp_sprintf(lineText," RC Rate      :%3d   ",currentControlRateProfile->rcRates[FD_ROLL]);
                    HottWriteLine(hottTEXTMODEMessage,2,lineText);
                    tfp_sprintf(lineText," RC Rate Yaw  :%3d   ",currentControlRateProfile->rcRates[FD_YAW]);
                    HottWriteLine(hottTEXTMODEMessage,3,lineText);
                    tfp_sprintf(lineText," SuperRate Rol:%3d   ",currentControlRateProfile->rates[0]);
                    HottWriteLine(hottTEXTMODEMessage,4,lineText);
                    tfp_sprintf(lineText," SuperRate Pit:%3d   ",currentControlRateProfile->rates[1]);
                    HottWriteLine(hottTEXTMODEMessage,5,lineText);
                    tfp_sprintf(lineText," SuperRate Yaw:%3d   ",currentControlRateProfile->rates[2]);
                    HottWriteLine(hottTEXTMODEMessage,6,lineText);    
					break;
					}//END PAGE
				case RC_Rates_2: //PAGE RC-Rates 2
					{                   //123456789|123456789|1
                    tfp_sprintf(lineText,"RC-Rates 2  v%5s <>",FC_VERSION_STRING);
                    HottWriteLine(hottTEXTMODEMessage,0,lineText);
                    tfp_sprintf(lineText," Profile Rates: %3d  ",getCurrentControlRateProfileIndex() + 1);
                    HottWriteLine(hottTEXTMODEMessage,1,lineText);
                    tfp_sprintf(lineText," RC Expo      : %3d  ",currentControlRateProfile->rcExpo[FD_ROLL]);
                    HottWriteLine(hottTEXTMODEMessage,2,lineText);
                    tfp_sprintf(lineText," RC Expo Yaw  : %3d  ",currentControlRateProfile->rcExpo[FD_YAW]);
                    HottWriteLine(hottTEXTMODEMessage,3,lineText);
					break;
					}//END PAGE
                case VTX_Config: //PAGE VTX Config
				    {
                    tfp_sprintf(lineText,"VTX Config  v%5s <>",FC_VERSION_STRING);
                    HottWriteLine(hottTEXTMODEMessage,0,lineText);
                    tfp_sprintf(lineText," Band    : %s        ",(vtx58BandNames[vtxSettingsConfig()->band]));
                    HottWriteLine(hottTEXTMODEMessage,1,lineText);
                    tfp_sprintf(lineText," Channel : %d        ",vtxSettingsConfig()->channel);
                    HottWriteLine(hottTEXTMODEMessage,2,lineText);
					tfp_sprintf(lineText," Frequenz: %4d       ",(vtx58frequencyTable[vtxSettingsConfig()->band-1][vtxSettingsConfig()->channel-1]));
                    HottWriteLine(hottTEXTMODEMessage,3,lineText);
					tfp_sprintf(lineText," Power   : %d        ",vtxSettingsConfig()->power);
                    HottWriteLine(hottTEXTMODEMessage,4,lineText);
					break;
					}//END PAGE  
				case FILTER_1: //PAGE FILTER 1
					{                   //123456789|123456789|1            
                    tfp_sprintf(lineText,"FILTER 1    v%5s <>",FC_VERSION_STRING);
                    HottWriteLine(hottTEXTMODEMessage,0,lineText);
                    tfp_sprintf(lineText," Filter      :%s    ",TableDtermLowpassType[currentPidProfile->dterm_filter_type]);
                    HottWriteLine(hottTEXTMODEMessage,1,lineText);
                    tfp_sprintf(lineText," Gyro_low Hz :%3d    ",gyroConfig()->gyro_lowpass_hz);
                    HottWriteLine(hottTEXTMODEMessage,2,lineText);
                    tfp_sprintf(lineText," Gyro_not1Hz :%3d    ",gyroConfig()->gyro_soft_notch_hz_1);
                    HottWriteLine(hottTEXTMODEMessage,3,lineText);
                    tfp_sprintf(lineText," Gyro_cut1Hz :%3d    ",gyroConfig()->gyro_soft_notch_cutoff_1);
                    HottWriteLine(hottTEXTMODEMessage,4,lineText);
                    tfp_sprintf(lineText," Gyro_not2Hz :%3d    ",gyroConfig()->gyro_soft_notch_hz_2);
                    HottWriteLine(hottTEXTMODEMessage,5,lineText);
                    tfp_sprintf(lineText," Gyro_cut2Hz :%3d    ",gyroConfig()->gyro_soft_notch_cutoff_2);
                    HottWriteLine(hottTEXTMODEMessage,6,lineText);
					break;
					}//END PAGE
				case FILTER_2: //PAGE FILTER 2
					{                   //123456789|123456789|1            
                    tfp_sprintf(lineText,"FILTER 2    v%5s <>",FC_VERSION_STRING);
                    HottWriteLine(hottTEXTMODEMessage,0,lineText);
                    tfp_sprintf(lineText," Filter      :%s    ",TableDtermLowpassType[currentPidProfile->dterm_filter_type]);
                    HottWriteLine(hottTEXTMODEMessage,1,lineText);
                    tfp_sprintf(lineText," D-T_low  Hz :%3d    ",currentPidProfile->dterm_lowpass_hz);
                    HottWriteLine(hottTEXTMODEMessage,2,lineText);
                    tfp_sprintf(lineText," D-T_not  Hz :%3d    ",currentPidProfile->dterm_notch_hz);
                    HottWriteLine(hottTEXTMODEMessage,3,lineText);
                    tfp_sprintf(lineText," D-T_cut  Hz :%3d    ",currentPidProfile->dterm_notch_cutoff);
                    HottWriteLine(hottTEXTMODEMessage,4,lineText);
                    tfp_sprintf(lineText," YAW_low  Hz :%3d    ",currentPidProfile->yaw_lowpass_hz);
                    HottWriteLine(hottTEXTMODEMessage,5,lineText);
					tfp_sprintf(lineText," TPA         :%4d    ",currentControlRateProfile->tpa_breakpoint);
					HottWriteLine(hottTEXTMODEMessage,6,lineText);
					break;
					}//END PAGE
				case Battery: //PAGE Battery
					{                   //123456789|123456789|1            
                    tfp_sprintf(lineText,"Battery     v%5s <>",FC_VERSION_STRING);
                    HottWriteLine(hottTEXTMODEMessage,0,lineText);
                    tfp_sprintf(lineText," cell warn  V:%d.%1d ",batteryConfig()->vbatwarningcellvoltage / 10, batteryConfig()->vbatwarningcellvoltage % 10);
                    HottWriteLine(hottTEXTMODEMessage,1,lineText);
                    tfp_sprintf(lineText," Capacity mAh:%4d    ",batteryConfig()->batteryCapacity);
                    HottWriteLine(hottTEXTMODEMessage,2,lineText);
                    tfp_sprintf(lineText," Vbat scale  :%3d    ",voltageSensorADCConfig(0)->vbatscale);
                    HottWriteLine(hottTEXTMODEMessage,3,lineText);
                    tfp_sprintf(lineText," Curr scale  :%3d    ",currentSensorADCConfig()->scale);
                    HottWriteLine(hottTEXTMODEMessage,4,lineText);
                    tfp_sprintf(lineText," Vbat       V:%d.%1d ",getBatteryVoltage() / 10, getBatteryVoltage() % 10);
                    HottWriteLine(hottTEXTMODEMessage,5,lineText);
					tfp_sprintf(lineText," Curr       A:%d.%1d ",getAmperage() / 100, getAmperage() % 100);
                    HottWriteLine(hottTEXTMODEMessage,6,lineText);
					break;
					}//END PAGE
			    }
                hottTEXTMODEMessage->text[ligne_select][0] = '>';
                HottInvertLigne(hottTEXTMODEMessage,ligne_edit);  
		    }
			else if (id_sensor == (HOTT_TELEMETRY_EAM_SENSOR_ID & 0x0f) && (ARMING_FLAG(ARMED))) {
				hottTEXTMODEMessage->esc = HOTT_TELEMETRY_EAM_SENSOR_TEXT;
				
				tfp_sprintf(lineText,"ARMED no edit        <");
				HottWriteLine(hottTEXTMODEMessage,0,lineText);
			}	
			else {
				tfp_sprintf(lineText,"Unknow sensor module <");
				HottWriteLine(hottTEXTMODEMessage,0,lineText);
				tfp_sprintf(lineText,"Nothing here");
				HottWriteLine(hottTEXTMODEMessage,1,lineText);
			}
		}
	hottSendTEXTMODEResponse();
}
static void hottCheckSerialData(uint32_t currentMicros)
{
    static bool lookingForRequest = true;

    const uint8_t bytesWaiting = serialRxBytesWaiting(hottPort);

    if (bytesWaiting <= 1) {
        return;
    }

    if (bytesWaiting != 2) {
        flushHottRxBuffer();
        lookingForRequest = true;
        return;
    }

    if (lookingForRequest) {
        lastHoTTRequestCheckAt = currentMicros;
        lookingForRequest = false;
        return;
    } else {
        bool enoughTimePassed = currentMicros - lastHoTTRequestCheckAt >= HOTT_RX_SCHEDULE;

        if (!enoughTimePassed) {
            return;
        }
        lookingForRequest = true;
    }

    const uint8_t requestId = serialRead(hottPort);
    const uint8_t address = serialRead(hottPort);

	if (requestId == HOTT_TEXT_MODE_REQUEST_ID) {
		processTextModeRequest(&hottTEXTMODEMessage,address);
	}
    else if ((requestId == 0) || (requestId == HOTT_BINARY_MODE_REQUEST_ID) || (address == HOTT_TELEMETRY_NO_SENSOR_ID)) {
    /*
     * FIXME the first byte of the HoTT request frame is ONLY either 0x80 (binary mode) or 0x7F (text mode).
     * The binary mode is read as 0x00 (error reading the upper bit) while the text mode is correctly decoded.
     * The (requestId == 0) test is a workaround for detecting the binary mode with no ambiguity as there is only
     * one other valid value (0x7F) for text mode.
     * The error reading for the upper bit should nevertheless be fixed
     */
        processBinaryModeRequest(address);
    }
}

static void hottSendTelemetryData(void) {

    if (!hottIsSending) {
        hottConfigurePortForTX();
        return;
    }

    if (hottMsgRemainingBytesToSendCount == 0) {
        hottConfigurePortForRX();
        return;
    }

    --hottMsgRemainingBytesToSendCount;
    if (hottMsgRemainingBytesToSendCount == 0) {
        hottSerialWrite(hottMsgCrc++);
        return;
    }

    hottMsgCrc += *hottMsg;
    hottSerialWrite(*hottMsg++);
}

static inline bool shouldPrepareHoTTMessages(uint32_t currentMicros)
{
    return currentMicros - lastMessagesPreparedAt >= HOTT_MESSAGE_PREPARATION_FREQUENCY_5_HZ;
}

static inline bool shouldCheckForHoTTRequest(void)
{
    if (hottIsSending) {
        return false;
    }
    return true;
}

void checkHoTTTelemetryState(void)
{
    const bool newTelemetryEnabledValue = telemetryDetermineEnabledState(hottPortSharing);

    if (newTelemetryEnabledValue == hottTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue) {
        configureHoTTTelemetryPort();
    } else {
        freeHoTTTelemetryPort();
    }
}

void handleHoTTTelemetry(timeUs_t currentTimeUs)
{
    static timeUs_t serialTimer;

    if (!hottTelemetryEnabled) {
        return;
    }

    if (shouldPrepareHoTTMessages(currentTimeUs)) {
        hottPrepareMessages();
        lastMessagesPreparedAt = currentTimeUs;
    }

    if (shouldCheckForHoTTRequest()) {    // Not hottIsSending
        hottCheckSerialData(currentTimeUs);
    }

    if (!hottMsg)
        return;

    if (hottIsSending) {
        if (currentTimeUs - serialTimer < HOTT_TX_DELAY_US) {
            return;
        }
    }
    hottSendTelemetryData();
    serialTimer = currentTimeUs;
}

#endif
