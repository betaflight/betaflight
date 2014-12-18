/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "platform.h"
#include "version.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"

#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"

#include "common/printf.h"

#include "flight/flight.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "rx/rx.h"
#include "io/rc_controls.h"
#include "flight/mixer.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/autotune.h"
#include "flight/navigation.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "rx/msp.h"
#include "telemetry/telemetry.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "blackbox_fielddefs.h"
#include "blackbox.h"

#define BLACKBOX_BAUDRATE 115200
#define BLACKBOX_INITIAL_PORT_MODE MODE_TX
#define BLACKBOX_I_INTERVAL 32

// Some macros to make writing FLIGHT_LOG_FIELD_PREDICTOR_* constants shorter:
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define CONCAT_HELPER(x,y) x ## y
#define CONCAT(x,y) CONCAT_HELPER(x, y)

#define PREDICT(x) STR(CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x))
#define ENCODING(x) STR(CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x))

static const char blackboxHeader[] =
	"H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
	"H Blackbox version:1\n"
	"H Data version:1\n"
	"H I interval:" STR(BLACKBOX_I_INTERVAL) "\n";

/* These headers have info for all 8 motors on them, we'll trim the final fields off to match the number of motors in the mixer: */
static const char * const blackboxHeaderFields[] = {
	"H Field I name:"
		"loopIteration,time,"
		"axisP[0],axisP[1],axisP[2],"
		"axisI[0],axisI[1],axisI[2],"
		"axisD[0],axisD[1],axisD[2],"
		"rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],"
		"gyroData[0],gyroData[1],gyroData[2],"
		"accSmooth[0],accSmooth[1],accSmooth[2],"
		"motor[0],motor[1],motor[2],motor[3],"
		"motor[4],motor[5],motor[6],motor[7]",

	"H Field I signed:"
		/* loopIteration, time: */
		"0,0,"
		/* PIDs: */
		"1,1,1,1,1,1,1,1,1,"
		/* rcCommand[0..2] */
		"1,1,1,"
		/* rcCommand[3] (Throttle): */
		"0,"
		/* gyroData[0..2]: */
		"1,1,1,"
		/* accSmooth[0..2]: */
		"1,1,1,"
		/* Motor[0..7]: */
		"0,0,0,0,0,0,0,0",

	"H Field I predictor:"
		/* loopIteration, time: */
		PREDICT(0) "," PREDICT(0) ","
		/* PIDs: */
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		/* rcCommand[0..2] */
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		/* rcCommand[3] (Throttle): */
		PREDICT(MINTHROTTLE) ","
		/* gyroData[0..2]: */
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		/* accSmooth[0..2]: */
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		/* Motor[0]: */
		PREDICT(MINTHROTTLE) ","
		/* Motor[1..7]: */
		PREDICT(MOTOR_0) "," PREDICT(MOTOR_0) "," PREDICT(MOTOR_0) ","
		PREDICT(MOTOR_0) "," PREDICT(MOTOR_0) "," PREDICT(MOTOR_0) ","
		PREDICT(MOTOR_0),

     "H Field I encoding:"
		/* loopIteration, time: */
		ENCODING(UNSIGNED_VB) "," ENCODING(UNSIGNED_VB) ","
		/* PIDs: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* rcCommand[0..2] */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* rcCommand[3] (Throttle): */
		ENCODING(UNSIGNED_VB) ","
		/* gyroData[0..2]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* accSmooth[0..2]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* Motor[0]: */
		ENCODING(UNSIGNED_VB) ","
		/* Motor[1..7]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB),

	//Motors and gyros predict an average of the last two measurements (to reduce the impact of noise):
	"H Field P predictor:"
		/* loopIteration, time: */
		PREDICT(INC) "," PREDICT(STRAIGHT_LINE) ","
		/* PIDs: */
		PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) ","
		PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) ","
		PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) ","
		/* rcCommand[0..2] */
		PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) ","
		/* rcCommand[3] (Throttle): */
		PREDICT(PREVIOUS) ","
		/* gyroData[0..2]: */
		PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) ","
		/* accSmooth[0..2]: */
		PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) ","
		/* Motor[0]: */
		PREDICT(AVERAGE_2) ","
		/* Motor[1..7]: */
		PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) ","
		PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) ","
		PREDICT(AVERAGE_2),

	/* RC fields are encoded together as a group, everything else is signed since they're diffs: */
    "H Field P encoding:"
		/* loopIteration, time: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","
		/* PIDs: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(TAG2_3S32) "," ENCODING(TAG2_3S32) ","  ENCODING(TAG2_3S32) ","
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* rcCommand[0..3] */
		ENCODING(TAG8_4S16) "," ENCODING(TAG8_4S16) ","  ENCODING(TAG8_4S16) ","
		ENCODING(TAG8_4S16) ","
		/* gyroData[0..2]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* accSmooth[0..2]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* Motor[0]: */
		ENCODING(SIGNED_VB) ","
		/* Motor[1..7]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB)
};

/**
 * Additional fields to tack on to those above for tricopters (to record tail servo position)
 */
static const char * const blackboxAdditionalFieldsTricopter[] = {
	//Field I name
		"servo[5]",

	//Field I signed
		"0",

	//Field I predictor
		PREDICT(1500),

    //Field I encoding:
		ENCODING(SIGNED_VB),

	//Field P predictor:
		PREDICT(PREVIOUS),

    //Field P encoding:
		ENCODING(SIGNED_VB)
};

static const char blackboxGpsHeader[] =
	"H Field G name:"
		"GPS_numSat,GPS_coord[0],GPS_coord[1],GPS_altitude,GPS_speed\n"
	"H Field G signed:"
		"0,1,1,0,0\n"
	"H Field G predictor:"
		PREDICT(0) "," PREDICT(HOME_COORD) "," PREDICT(HOME_COORD) "," PREDICT(0) "," PREDICT(0) "\n"
	"H Field G encoding:"
		ENCODING(UNSIGNED_VB) "," ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","
		ENCODING(UNSIGNED_VB) "," ENCODING(UNSIGNED_VB) "\n"

	"H Field H name:"
		"GPS_home[0],GPS_home[1]\n"
	"H Field H signed:"
		"1,1\n"
	"H Field H predictor:"
		PREDICT(0) "," PREDICT(0) "\n"
	"H Field H encoding:"
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) "\n";

typedef enum BlackboxState {
	BLACKBOX_STATE_DISABLED = 0,
	BLACKBOX_STATE_STOPPED,
	BLACKBOX_STATE_SEND_HEADER,
	BLACKBOX_STATE_SEND_FIELDINFO,
	BLACKBOX_STATE_SEND_GPS_HEADERS,
	BLACKBOX_STATE_SEND_SYSINFO,
	BLACKBOX_STATE_RUNNING
} BlackboxState;

typedef struct gpsState_t {
	int32_t GPS_home[2], GPS_coord[2];
	uint8_t GPS_numSat;
} gpsState_t;

//From mixer.c:
extern uint8_t motorCount;

//From mw.c:
extern uint32_t currentTime;

static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;
static uint32_t startTime;
static unsigned int headerXmitIndex;
static uint32_t blackboxIteration;

static serialPort_t *blackboxPort;
static portMode_t previousPortMode;
static uint32_t previousBaudRate;

static gpsState_t gpsHistory;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxValues_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackboxValues_t* blackboxHistory[3];

static int isTricopter()
{
	return masterConfig.mixerConfiguration == MULTITYPE_TRI;
}

static void blackboxWrite(uint8_t value)
{
	serialWrite(blackboxPort, value);
}

static void _putc(void *p, char c)
{
    (void)p;
    serialWrite(blackboxPort, c);
}

//printf() to the blackbox serial port with no blocking shenanigans (so it's caller's responsibility to not write too fast!)
static void blackboxPrintf(char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    tfp_format(NULL, _putc, fmt, va);
    va_end(va);
}

/**
 * Write an unsigned integer to the blackbox serial port using variable byte encoding.
 */
static void writeUnsignedVB(uint32_t value)
{
	//While this isn't the final byte (we can only write 7 bits at a time)
	while (value > 127) {
		blackboxWrite((uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
		value >>= 7;
	}
	blackboxWrite(value);
}

/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
static void writeSignedVB(int32_t value)
{
	//ZigZag encode to make the value always positive
	writeUnsignedVB((uint32_t)((value << 1) ^ (value >> 31)));
}

/**
 * Write a 2 bit tag followed by 3 signed fields of 2, 4, 6 or 32 bits
 */
static void writeTag2_3S32(int32_t *values) {
	static const int NUM_FIELDS = 3;

	//Need to be enums rather than const ints if we want to switch on them (due to being C)
	enum { BITS_2  = 0};
	enum { BITS_4  = 1};
	enum { BITS_6  = 2};
	enum { BITS_32 = 3};

	enum { BYTES_1  = 0};
	enum { BYTES_2  = 1};
	enum { BYTES_3  = 2};
	enum { BYTES_4  = 3};

	int x;
	int selector = BITS_2, selector2;

	/*
	 * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
	 * below:
	 *
	 * Selector possibilities
	 *
	 * 2 bits per field  ss11 2233,
	 * 4 bits per field  ss00 1111 2222 3333
	 * 6 bits per field  ss11 1111 0022 2222 0033 3333
	 * 32 bits per field sstt tttt followed by fields of various byte counts
	 */
	for (x = 0; x < NUM_FIELDS; x++) {
		//Require more than 6 bits?
		if (values[x] >= 32 || values[x] < -32) {
			selector = BITS_32;
			break;
		}

		//Require more than 4 bits?
		if (values[x] >= 8 || values[x] < -8) {
			 if (selector < BITS_6)
				 selector = BITS_6;
		} else if (values[x] >= 2 || values[x] < -2) { //Require more than 2 bits?
			if (selector < BITS_4)
				selector = BITS_4;
		}
	}

	switch (selector) {
		case BITS_2:
			blackboxWrite((selector << 6) | ((values[0] & 0x03) << 4) | ((values[1] & 0x03) << 2) | (values[2] & 0x03));
		break;
		case BITS_4:
			blackboxWrite((selector << 6) | (values[0] & 0x0F));
			blackboxWrite((values[1] << 4) | (values[2] & 0x0F));
		break;
		case BITS_6:
			blackboxWrite((selector << 6) | (values[0] & 0x3F));
			blackboxWrite((uint8_t)values[1]);
			blackboxWrite((uint8_t)values[2]);
		break;
		case BITS_32:
			/*
			 * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
			 *
			 * Selector2 field possibilities
			 * 0 - 8 bits
			 * 1 - 16 bits
			 * 2 - 24 bits
			 * 3 - 32 bits
			 */
			selector2 = 0;

			//Encode in reverse order so the first field is in the low bits:
			for (x = NUM_FIELDS - 1; x >= 0; x--) {
				selector2 <<= 2;

				if (values[x] < 128 && values[x] >= -128)
					selector2 |= BYTES_1;
				else if (values[x] < 32768 && values[x] >= -32768)
					selector2 |= BYTES_2;
				else if (values[x] < 8388608 && values[x] >= -8388608)
					selector2 |= BYTES_3;
				else
					selector2 |= BYTES_4;
			}

			//Write the selectors
			blackboxWrite((selector << 6) | selector2);

			//And now the values according to the selectors we picked for them
			for (x = 0; x < NUM_FIELDS; x++, selector2 >>= 2) {
				switch (selector2 & 0x03) {
					case BYTES_1:
						blackboxWrite(values[x]);
					break;
					case BYTES_2:
						blackboxWrite(values[x]);
						blackboxWrite(values[x] >> 8);
					break;
					case BYTES_3:
						blackboxWrite(values[x]);
						blackboxWrite(values[x] >> 8);
						blackboxWrite(values[x] >> 16);
					break;
					case BYTES_4:
						blackboxWrite(values[x]);
						blackboxWrite(values[x] >> 8);
						blackboxWrite(values[x] >> 16);
						blackboxWrite(values[x] >> 24);
					break;
				}
			}
		break;
	}
}

/**
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
static void writeTag8_4S16(int32_t *values) {

	//Need to be enums rather than const ints if we want to switch on them (due to being C)
	enum { FIELD_ZERO  = 0};
	enum { FIELD_4BIT  = 1};
	enum { FIELD_8BIT  = 2};
	enum { FIELD_16BIT = 3};

	uint8_t selector;
	int x;

	/*
	 * 4-bit fields can only be combined with their paired neighbor (there are two pairs), so choose a
	 * larger encoding if that's not possible.
	 */
	const uint8_t rcSelectorCleanup[16] = {
	//          Output selectors     <- Input selectors
		FIELD_ZERO << 2 | FIELD_ZERO,   // zero, zero
		FIELD_ZERO << 2 | FIELD_8BIT,   // zero, 4-bit
		FIELD_ZERO << 2 | FIELD_8BIT,   // zero, 8-bit
		FIELD_ZERO << 2 | FIELD_16BIT,  // zero, 16-bit
		FIELD_8BIT << 2 | FIELD_ZERO,   // 4-bit, zero
		FIELD_4BIT << 2 | FIELD_4BIT,   // 4-bit, 4-bit
		FIELD_8BIT << 2 | FIELD_8BIT,   // 4-bit, 8-bit
		FIELD_8BIT << 2 | FIELD_16BIT,  // 4-bit, 16-bit
		FIELD_8BIT << 2 | FIELD_ZERO,   // 8-bit, zero
		FIELD_8BIT << 2 | FIELD_8BIT,   // 8-bit, 4-bit
		FIELD_8BIT << 2 | FIELD_8BIT,   // 8-bit, 8-bit
		FIELD_8BIT << 2 | FIELD_16BIT,  // 8-bit, 16-bit
		FIELD_16BIT << 2 | FIELD_ZERO,  // 16-bit, zero
		FIELD_16BIT << 2 | FIELD_8BIT,  // 16-bit, 4-bit
		FIELD_16BIT << 2 | FIELD_8BIT,  // 16-bit, 8-bit
		FIELD_16BIT << 2 | FIELD_16BIT, // 16-bit, 16-bit
	};

	selector = 0;
	//Encode in reverse order so the first field is in the low bits:
	for (x = 3; x >= 0; x--) {
		selector <<= 2;

		if (values[x] == 0)
			selector |= FIELD_ZERO;
		else if (values[x] <= 7 && values[x] >= -8)
			selector |= FIELD_4BIT;
		else if (values[x] <= 127 && values[x] >= -128)
			selector |= FIELD_8BIT;
		else
			selector |= FIELD_16BIT;
	}

	selector = rcSelectorCleanup[selector & 0x0F] | (rcSelectorCleanup[selector >> 4] << 4);

	blackboxWrite(selector);

	for (x = 0; x < 4; x++, selector >>= 2) {
		switch (selector & 0x03) {
			case FIELD_4BIT:
				blackboxWrite((values[x] & 0x0F) | (values[x + 1] << 4));

				//We write two selector fields:
				x++;
				selector >>= 2;
			break;
			case FIELD_8BIT:
				blackboxWrite(values[x]);
			break;
			case FIELD_16BIT:
				blackboxWrite(values[x]);
				blackboxWrite(values[x] >> 8);
			break;
		}
	}
}

static void writeIntraframe(void)
{
	blackboxValues_t *blackboxCurrent = blackboxHistory[0];
	int x;

	blackboxWrite('I');

	writeUnsignedVB(blackboxIteration);
	writeUnsignedVB(blackboxCurrent->time);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->axisP[x]);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->axisI[x]);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->axisD[x]);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->rcCommand[x]);

	writeUnsignedVB(blackboxCurrent->rcCommand[3] - masterConfig.escAndServoConfig.minthrottle); //Throttle lies in range [minthrottle..maxthrottle]

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->gyroData[x]);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->accSmooth[x]);

	//Motors can be below minthrottle when disarmed, but that doesn't happen much
	writeUnsignedVB(blackboxCurrent->motor[0] - masterConfig.escAndServoConfig.minthrottle);

	//Motors tend to be similar to each other
	for (x = 1; x < motorCount; x++)
		writeSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);

	if (isTricopter())
		writeSignedVB(blackboxHistory[0]->servo[5] - 1500);

	//Rotate our history buffers:

	//The current state becomes the new "before" state
	blackboxHistory[1] = blackboxHistory[0];
	//And since we have no other history, we also use it for the "before, before" state
	blackboxHistory[2] = blackboxHistory[0];
	//And advance the current state over to a blank space ready to be filled
	blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

static void writeInterframe(void)
{
	int x;
	int32_t deltas[4];

	blackboxValues_t *blackboxCurrent = blackboxHistory[0];
	blackboxValues_t *blackboxLast = blackboxHistory[1];

	blackboxWrite('P');

	//No need to store iteration count since its delta is always 1

	/*
	 * Since the difference between the difference between successive times will be nearly zero, use
	 * second-order differences.
	 */
	writeSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->axisP[x] - blackboxLast->axisP[x]);

	for (x = 0; x < 3; x++)
		deltas[x] = blackboxCurrent->axisI[x] - blackboxLast->axisI[x];

	/* 
	 * The PID I field changes very slowly, most of the time +-2, so use an encoding
	 * that can pack all three fields into one byte in that situation.
	 */
	writeTag2_3S32(deltas);
	
	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->axisD[x] - blackboxLast->axisD[x]);

	for (x = 0; x < 4; x++)
		deltas[x] = blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x];

	/*
	 * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
	 * can pack multiple values per byte:
	 */
	writeTag8_4S16(deltas);

	//Since gyros, accs and motors are noisy, base the prediction on the average of the history:
	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxHistory[0]->gyroData[x] - (blackboxHistory[1]->gyroData[x] + blackboxHistory[2]->gyroData[x]) / 2);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxHistory[0]->accSmooth[x] - (blackboxHistory[1]->accSmooth[x] + blackboxHistory[2]->accSmooth[x]) / 2);

	for (x = 0; x < motorCount; x++)
		writeSignedVB(blackboxHistory[0]->motor[x] - (blackboxHistory[1]->motor[x] + blackboxHistory[2]->motor[x]) / 2);

	if (isTricopter())
		writeSignedVB(blackboxCurrent->servo[5] - blackboxLast->servo[5]);

	//Rotate our history buffers
	blackboxHistory[2] = blackboxHistory[1];
	blackboxHistory[1] = blackboxHistory[0];
	blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

static int gcd(int num, int denom)
{
	if (denom == 0)
		return num;
	return gcd(denom, num % denom);
}

static void validateBlackboxConfig()
{
	int div;

	if (masterConfig.blackbox_rate_num == 0 || masterConfig.blackbox_rate_denom == 0
			|| masterConfig.blackbox_rate_num >= masterConfig.blackbox_rate_denom) {
		masterConfig.blackbox_rate_num = 1;
		masterConfig.blackbox_rate_denom = 1;
	} else {
		div = gcd(masterConfig.blackbox_rate_num, masterConfig.blackbox_rate_denom);

		masterConfig.blackbox_rate_num /= div;
		masterConfig.blackbox_rate_denom /= div;
	}
}

static void configureBlackboxPort(void)
{
	blackboxPort = findOpenSerialPort(FUNCTION_BLACKBOX);
	if (blackboxPort) {
		previousPortMode = blackboxPort->mode;
		previousBaudRate = blackboxPort->baudRate;

		serialSetBaudRate(blackboxPort, BLACKBOX_BAUDRATE);
		serialSetMode(blackboxPort, BLACKBOX_INITIAL_PORT_MODE);
		beginSerialPortFunction(blackboxPort, FUNCTION_BLACKBOX);
	} else {
		blackboxPort = openSerialPort(FUNCTION_BLACKBOX, NULL, BLACKBOX_BAUDRATE, BLACKBOX_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);

		if (blackboxPort) {
			previousPortMode = blackboxPort->mode;
			previousBaudRate = blackboxPort->baudRate;
		}
	}
}

static void releaseBlackboxPort(void)
{
    serialSetMode(blackboxPort, previousPortMode);
    serialSetBaudRate(blackboxPort, previousBaudRate);

    endSerialPortFunction(blackboxPort, FUNCTION_BLACKBOX);
}

void startBlackbox(void)
{
	if (blackboxState == BLACKBOX_STATE_STOPPED) {
		validateBlackboxConfig();

		configureBlackboxPort();

		if (!blackboxPort) {
			blackboxState = BLACKBOX_STATE_DISABLED;
			return;
		}

		startTime = millis();
		headerXmitIndex = 0;
		blackboxIteration = 0;
		blackboxState = BLACKBOX_STATE_SEND_HEADER;

		memset(&gpsHistory, 0, sizeof(gpsHistory));

		//No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it

		blackboxHistory[0] = &blackboxHistoryRing[0];
		blackboxHistory[1] = &blackboxHistoryRing[1];
		blackboxHistory[2] = &blackboxHistoryRing[2];
	}
}

void finishBlackbox(void)
{
	if (blackboxState != BLACKBOX_STATE_DISABLED && blackboxState != BLACKBOX_STATE_STOPPED) {
		blackboxState = BLACKBOX_STATE_STOPPED;
		
		releaseBlackboxPort();
	}
}

static void writeGPSHomeFrame()
{
	blackboxWrite('H');

	writeSignedVB(GPS_home[0]);
	writeSignedVB(GPS_home[1]);
	//TODO it'd be great if we could grab the GPS current time and write that too

	gpsHistory.GPS_home[0] = GPS_home[0];
	gpsHistory.GPS_home[1] = GPS_home[1];
}

static void writeGPSFrame()
{
	blackboxWrite('G');

	writeUnsignedVB(GPS_numSat);
	writeSignedVB(GPS_coord[0] - gpsHistory.GPS_home[0]);
	writeSignedVB(GPS_coord[1] - gpsHistory.GPS_home[1]);
	writeUnsignedVB(GPS_altitude);
	writeUnsignedVB(GPS_speed);

	gpsHistory.GPS_numSat = GPS_numSat;
	gpsHistory.GPS_coord[0] = GPS_coord[0];
	gpsHistory.GPS_coord[1] = GPS_coord[1];
}

/**
 * Fill the current state of the blackbox using values read from the flight controller
 */
static void loadBlackboxState(void)
{
	blackboxValues_t *blackboxCurrent = blackboxHistory[0];
	int i;

	blackboxCurrent->time = currentTime;

	for (i = 0; i < 3; i++)
		blackboxCurrent->axisP[i] = axisP[i];
	for (i = 0; i < 3; i++)
		blackboxCurrent->axisI[i] = axisI[i];
	for (i = 0; i < 3; i++)
		blackboxCurrent->axisD[i] = axisD[i];

	for (i = 0; i < 4; i++)
		blackboxCurrent->rcCommand[i] = rcCommand[i];

	for (i = 0; i < 3; i++)
		blackboxCurrent->gyroData[i] = gyroData[i];

	for (i = 0; i < 3; i++)
		blackboxCurrent->accSmooth[i] = accSmooth[i];

	for (i = 0; i < motorCount; i++)
		blackboxCurrent->motor[i] = motor[i];

	//Tail servo for tricopters
	blackboxCurrent->servo[5] = servo[5];
}

void handleBlackbox(void)
{
	int i;
	const char *additionalHeader;
	const int SERIAL_CHUNK_SIZE = 16;
	static int charXmitIndex = 0;
	int motorsToRemove, endIndex;
	int blackboxIntercycleIndex, blackboxIntracycleIndex;

	union floatConvert_t {
		float f;
		uint32_t u;
	} floatConvert;

	switch (blackboxState) {
		case BLACKBOX_STATE_SEND_HEADER:
			/*
			 * Once the UART has had time to init, transmit the header in chunks so we don't overflow our transmit
			 * buffer.
			 */
			if (millis() > startTime + 100) {
				for (i = 0; i < SERIAL_CHUNK_SIZE && blackboxHeader[headerXmitIndex] != '\0'; i++, headerXmitIndex++)
					blackboxWrite(blackboxHeader[headerXmitIndex]);

				if (blackboxHeader[headerXmitIndex] == '\0') {
					blackboxState = BLACKBOX_STATE_SEND_FIELDINFO;
					headerXmitIndex = 0;
					charXmitIndex = 0;
				}
			}
		break;
		case BLACKBOX_STATE_SEND_FIELDINFO:
			/*
			 * Send information about the fields we're recording to the log.
			 *
			 * Once again, we're chunking up the data so we don't exceed our datarate. This time we're trimming the
			 * excess field defs off the end of the header for motors we don't have.
			 */
			motorsToRemove = 8 - motorCount;

			if (headerXmitIndex < sizeof(blackboxHeaderFields) / sizeof(blackboxHeaderFields[0])){
				endIndex = strlen(blackboxHeaderFields[headerXmitIndex]) - (headerXmitIndex == 0 ? strlen(",motor[x]") : strlen(",x")) * motorsToRemove;

				for (i = charXmitIndex; i < charXmitIndex + SERIAL_CHUNK_SIZE && i < endIndex; i++)
					blackboxWrite(blackboxHeaderFields[headerXmitIndex][i]);

				// Did we complete this line?
				if (i == endIndex) {
					if (isTricopter()) {
						//Add fields to the end for the tail servo
						blackboxWrite(',');

						for (additionalHeader = blackboxAdditionalFieldsTricopter[headerXmitIndex]; *additionalHeader; additionalHeader++)
							blackboxWrite(*additionalHeader);
					}

					blackboxWrite('\n');
					headerXmitIndex++;
					charXmitIndex = 0;
				} else {
					charXmitIndex = i;
				}
			} else {
				//Completed sending field information

				if (feature(FEATURE_GPS)) {
					blackboxState = BLACKBOX_STATE_SEND_GPS_HEADERS;
				} else {
					blackboxState = BLACKBOX_STATE_SEND_SYSINFO;
				}
				headerXmitIndex = 0;
			}
		break;
		case BLACKBOX_STATE_SEND_GPS_HEADERS:
			for (i = 0; i < SERIAL_CHUNK_SIZE && blackboxGpsHeader[headerXmitIndex] != '\0'; i++, headerXmitIndex++)
				blackboxWrite(blackboxGpsHeader[headerXmitIndex]);

			if (blackboxGpsHeader[headerXmitIndex] == '\0') {
				blackboxState = BLACKBOX_STATE_SEND_SYSINFO;
				headerXmitIndex = 0;
			}
		break;
		case BLACKBOX_STATE_SEND_SYSINFO:

			switch (headerXmitIndex) {
				case 0:
					blackboxPrintf("H Firmware type:Cleanflight\n");
				break;
				case 1:
					// Pause to allow more time for previous to transmit (it exceeds our chunk size)
				break;
				case 2:
					blackboxPrintf("H Firmware revision:%s\n", shortGitRevision);
				break;
				case 3:
					// Pause to allow more time for previous to transmit
				break;
				case 4:
					blackboxPrintf("H Firmware date:%s %s\n", buildDate, buildTime);
				break;
				case 5:
					// Pause to allow more time for previous to transmit
				break;
				case 6:
					blackboxPrintf("H rcRate:%d\n", masterConfig.controlRateProfiles[masterConfig.current_profile_index].rcRate8);
				break;
				case 7:
					blackboxPrintf("H minthrottle:%d\n", masterConfig.escAndServoConfig.minthrottle);
				break;
				case 8:
					blackboxPrintf("H maxthrottle:%d\n", masterConfig.escAndServoConfig.maxthrottle);
				break;
				case 9:
					floatConvert.f = gyro.scale;
					blackboxPrintf("H gyro.scale:0x%x\n", floatConvert.u);
				break;
				case 10:
					blackboxPrintf("H acc_1G:%u\n", acc_1G);
				break;
				case 11:
					// One more pause for good luck
				break;
				default:
					blackboxState = BLACKBOX_STATE_RUNNING;
			}

			headerXmitIndex++;
		break;
		case BLACKBOX_STATE_RUNNING:
			// Write a keyframe every 32 frames so we can resynchronise upon missing frames
			blackboxIntercycleIndex = blackboxIteration % BLACKBOX_I_INTERVAL;
			blackboxIntracycleIndex = blackboxIteration / BLACKBOX_I_INTERVAL;

			if (blackboxIntercycleIndex == 0) {
	        	// Copy current system values into the blackbox
	        	loadBlackboxState();
				writeIntraframe();
			} else {
				if ((blackboxIntercycleIndex + masterConfig.blackbox_rate_num - 1) % masterConfig.blackbox_rate_denom < masterConfig.blackbox_rate_num) {
		        	loadBlackboxState();
		        	writeInterframe();
		        }

				if (feature(FEATURE_GPS)) {
					/*
					 * If the GPS home point has been updated, or every 128 intraframes (~10 seconds), write the
					 * GPS home position.
					 *
					 * We write it periodically so that if one Home Frame goes missing, the GPS coordinates can
					 * still be interpreted correctly.
					 */
					if (GPS_home[0] != gpsHistory.GPS_home[0] || GPS_home[1] != gpsHistory.GPS_home[1]
						|| (blackboxIntercycleIndex == BLACKBOX_I_INTERVAL / 2 && blackboxIntracycleIndex % 128 == 0)) {

						writeGPSHomeFrame();
						writeGPSFrame();
					} else if (GPS_numSat != gpsHistory.GPS_numSat || GPS_coord[0] != gpsHistory.GPS_coord[0]
							|| GPS_coord[1] != gpsHistory.GPS_coord[1]) {
						//We could check for velocity changes as well but I doubt it changes independent of position
						writeGPSFrame();
					}
				}
			}

			blackboxIteration++;
		break;
		default:
		break;
	}
}

static bool canUseBlackboxWithCurrentConfiguration(void)
{
    if (!feature(FEATURE_BLACKBOX))
        return false;

    return true;
}

void initBlackbox(void)
{
	if (canUseBlackboxWithCurrentConfiguration())
		blackboxState = BLACKBOX_STATE_STOPPED;
	else
		blackboxState = BLACKBOX_STATE_DISABLED;
}
