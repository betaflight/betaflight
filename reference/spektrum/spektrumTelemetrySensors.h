//////////////////////////////////////////////////////////////////
//
//	Copyright 2013 by Horizon Hobby, Inc.  
//	Released to Public Domain
//
//	This header file may be incorporated into non-Horizon
//	products.
//
//////////////////////////////////////////////////////////////////

#ifndef TELEMETRY_H
#define	TELEMETRY_H

//////////////////////////////////////////////////////////////////
//
//	Assigned I2C Addresses and Device Types
//
//////////////////////////////////////////////////////////////////

#define	TELE_DEVICE_NODATA	(0x00)	// No data in packet
#define	TELE_DEVICE_VOLTAGE	(0x01)	// High-Voltage sensor (INTERNAL)
#define	TELE_DEVICE_TEMPERATURE	(0x02)	// Temperature Sensor (INTERNAL)
#define	TELE_DEVICE_RSV_03	(0x03)	// Reserved
#define	TELE_DEVICE_RSV_04	(0x04)	// Reserved
#define	TELE_DEVICE_RSV_05	(0x05)	// Reserved
#define	TELE_DEVICE_RSV_06	(0x06)	// Reserved
#define	TELE_DEVICE_RSV_07	(0x07)	// Reserved
#define	TELE_DEVICE_RSV_08	(0x08)	// Reserved
#define	TELE_DEVICE_RSV_09	(0x09)	// Reserved
#define	TELE_DEVICE_PBOX	(0x0A)	// PowerBox
#define	TELE_DEVICE_LAPTIMER	(0x0B)	// Lap Timer
#define	TELE_DEVICE_TEXTGEN	(0x0C)	// Text Generator
#define	TELE_DEVICE_AIRSPEED	(0x11)	// Air Speed
#define	TELE_DEVICE_ALTITUDE	(0x12)	// Altitude
#define	TELE_DEVICE_GMETER	(0x14)	// GForce
#define	TELE_DEVICE_JETCAT	(0x15)	// JetCat interface
#define	TELE_DEVICE_GPS_LOC	(0x16)	// GPS Location Data
#define	TELE_DEVICE_GPS_STATS	(0x17)	// GPS Status
#define	TELE_DEVICE_RX_MAH	(0x18)	// Receiver Pack Capacity (Dual)
#define	TELE_DEVICE_JETCAT_2	(0x19)	// JetCat interface, msg 2
#define	TELE_DEVICE_GYRO	(0x1A)	// 3-axis gyro
#define	TELE_DEVICE_ATTMAG	(0x1B)	// Attitude and Magnetic Compass
#define	TELE_DEVICE_AS3X_LEGACYGAIN	(0x1F)	// Active AS3X Gains for legacy mode
#define	TELE_DEVICE_ESC	(0x20)	// ESC
#define	TELE_DEVICE_FUEL	(0x22)	// Fuel Flow Meter
#define	TELE_DEVICE_ALPHA6	(0x24)	// Alpha6 Stabilizer
//	DO NOT USE	(0x30)	// Reserved for internal use
//	DO NOT USE	(0x32)	// Reserved for internal use
#define	TELE_DEVICE_MAH	(0x34)	// Battery Gauge (mAh) (Dual)
#define	TELE_DEVICE_DIGITAL_AIR	(0x36)	// Digital Inputs & Tank Pressure
#define	TELE_DEVICE_STRAIN	(0x38)	// Thrust/Strain Gauge
#define	TELE_DEVICE_LIPOMON	(0x3A)	// 6S Cell Monitor (LiPo taps)
#define	TELE_DEVICE_LIPOMON_14	(0x3F)	// 14S Cell Monitor (LiPo taps)
#define	TELE_DEVICE_VARIO_S	(0x40)	// Vario
#define	TELE_DEVICE_RSV_43	(0x43)	// Reserved
#define	TELE_DEVICE_USER_16SU	(0x50)	// User-Def, STRU_TELE_USER_16SU
#define	TELE_DEVICE_USER_16SU32U	(0x52)	// User-Def, STRU_TELE_USER_16SU32U
#define	TELE_DEVICE_USER_16SU32S	(0x54)	// User-Def, STRU_TELE_USER_16SU32S
#define	TELE_DEVICE_USER_16U32SU	(0x56)	// User-Def, STRU_TELE_USER_16U32SU
#define	TELE_DEVICE_RSV_60	(0x60)	// Reserved
#define	TELE_DEVICE_RSV_68	(0x68)	// Reserved
#define	TELE_DEVICE_RSV_69	(0x69)	// Reserved
#define	TELE_DEVICE_RSV_6A	(0x6A)	// Reserved
#define	TELE_DEVICE_RSV_6B	(0x6B)	// Reserved
#define	TELE_DEVICE_RSV_6C	(0x6C)	// Reserved
#define	TELE_DEVICE_RSV_6D	(0x6D)	// Reserved
#define	TELE_DEVICE_RSV_6E	(0x6E)	// Reserved
#define	TELE_DEVICE_RSV_6F	(0x6F)	// Reserved
#define	TELE_DEVICE_RSV_70	(0x70)	// Reserved
#define	TELE_DEVICE_RTC	(0x7C)	// Pseudo-device giving timestamp
#define	TELE_DEVICE_FRAMEDATA	(0x7D)	// Transmitter frame data
#define	TELE_DEVICE_RPM	(0x7E)	// RPM sensor
#define	TELE_DEVICE_QOS	(0x7F)	// RxV + flight log data
#define	TELE_DEVICE_MAX	(0x7F)	// Last address available

#define	TELE_DEVICE_SHORTRANGE	(0x80)	// Data is from a TM1100


//////////////////////////////////////////////////////////////////
//
//	Message Data Structures
//
//////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
//
//		Electronic Speed Control
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;	// Source device = 0x20
	UINT8	sID;	// Secondary ID
	UINT16	RPM;	// RPM, 10RPM (0-655340 RPM).     0xFFFF --> "No data"
	UINT16	voltsInput;	// Volts, 0.01v (0-655.34V).      0xFFFF --> "No data"
	UINT16	tempFET;	// Temperature, 0.1C (0-999.8C)   0xFFFF --> "No data"
	UINT16	currentMotor;	// Current, 10mA (0-655.34A).     0xFFFF --> "No data"
	UINT16	tempBEC;	// Temperature, 0.1C (0-999.8C)   0x7FFF --> "No data"
	UINT8	currentBEC;	// BEC Current, 100mA (0-25.4A).  0xFF ----> "No data"
	UINT8	voltsBEC;	// BEC Volts, 0.05V (0-12.70V).   0xFF ----> "No data"
	UINT8	throttle;	// 0.5% (0-127%).                 0xFF ----> "No data"
	UINT8	powerOut;	// Power Output, 0.5% (0-127%).   0xFF ----> "No data"} STRU_TELE_ESC;

//////////////////////////////////////////////////////////////////
//
//		LAP TIMER
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;	// Source device = 0x0B
	UINT8	sID;	// Secondary ID
	UINT8	lapNumber;	// Lap last finished
	UINT8	gateNumber;	// Last gate passed
	UINT32	lastLapTime;	// Time of lap in 1ms increments (NOT duration)
	UINT32	gateTime;	// Duration between last 2 gates
	UINT8	unused[4];
} STRU_TELE_LAPTIMER;

//////////////////////////////////////////////////////////////////
//
//		TEXT GENERATOR
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;	// Source device = 0x0C
	UINT8	sID;	// Secondary ID
	UINT8	lineNumber;	// Line number to display (0 = title, 1-8 for general, 
				//	255 = Erase all text on screen)
	char	text[13];	// 0-terminated text
} STRU_TELE_TEXTGEN;

//////////////////////////////////////////////////////////////////
//
//		(Liquid) Fuel Flow/Capacity (Two Tanks/Engines)
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	id;			// Source device = 0x22
	UINT8	sID;			// Secondary ID
	UINT16	fuelConsumed_A;			// Integrated fuel consumption, 0.1mL
	UINT16	flowRate_A;			// Instantaneous consumption, 0.01mL/min
	UINT16	temp_A;			// Temperature, 0.1C (0-655.34C)
	UINT16	fuelConsumed_B;			// Integrated fuel consumption, 0.1mL
	UINT16	flowRate_B;			// Instantaneous consumption, 0.01mL/min
	UINT16	temp_B;			// Temperature, 0.1C (0-655.34C)
	UINT16	spare;			// Not used
} STRU_TELE_FUEL;

//////////////////////////////////////////////////////////////////
//
//		Battery Current/Capacity (Dual Batteries)
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	id;			// Source device = 0x34
	UINT8	sID;			// Secondary ID
	INT16	current_A;			// Instantaneous current, 0.1A (0-3276.8A)
	INT16	chargeUsed_A;			// Integrated mAh used, 1mAh (0-32.766Ah)
	UINT16	temp_A;			// Temperature, 0.1C (0-150.0C, 
						// 0x7FFF indicates not populated)
	INT16	current_B;			// Instantaneous current, 0.1A (0-6553.4A)
	INT16	chargeUsed_B;			// Integrated mAh used, 1mAh (0-65.534Ah)
	UINT16	temp_B;			// Temperature, 0.1C (0-150.0C, 
						// 0x7FFF indicates not populated)
	UINT16	spare;			// Not used
} STRU_TELE_MAH;

//////////////////////////////////////////////////////////////////
//
//		Digital Input Status (Retract Status) and Tank Pressure
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	id;			// Source device = 0x36
	UINT8	sID;			// Secondary ID
	UINT16	digital;			// Digital inputs (bit per input)
	UINT16	pressure;			// Tank pressure, 0.1PSI (0-6553.4PSI)
} STRU_TELE_DIGITAL_AIR;

//////////////////////////////////////////////////////////////////
//
//		Thrust/Strain Gauge
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	id;			// Source device = 0x38
	UINT8	sID;			// Secondary ID
	UINT16	strain_A,			// Strain sensor A
			strain_B,			// Strain sensor B
			strain_C,			// Strain sensor D
			strain_D;			// Strain sensor C
} STRU_TELE_STRAIN;

//////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT DATA SIGNED/UNSIGNED
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	id;			// Source device = 0x50
	UINT8	sID;			// Secondary ID
	INT16	sField1,			// Signed 16-bit data fields
			sField2,
			sField3;
	UINT16	uField1, 			// Unsigned 16-bit data fields
			uField2,
			uField3,
			uField4;
} STRU_TELE_USER_16SU;

//////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT SIGNED/UNSIGNED AND 32-BIT UNSIGNED
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	id;			// Source device = 0x52
	UINT8	sID;			// Secondary ID
	INT16	sField1,			// Signed 16-bit data fields
			sField2;
	UINT16	uField1, 			// Unsigned 16-bit data fields
			uField2,
			uField3;
	UINT32	u32Field; 			// Unsigned 32-bit data field
} STRU_TELE_USER_16SU32U;

//////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT SIGNED/UNSIGNED AND 32-BIT SIGNED
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	id;			// Source device = 0x54
	UINT8	sID;			// Secondary ID
	INT16	sField1,			// Signed 16-bit data fields
			sField2;
	UINT16	uField1, 			// Unsigned 16-bit data fields
			uField2,
			uField3;
	INT32	u32Field; 			// Signed 32-bit data field
} STRU_TELE_USER_16U32SU;

//////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT UNSIGNED AND 32-BIT SIGNED/UNSIGNED
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	id;			// Source device = 0x56
	UINT8	sID;			// Secondary ID
	UINT16	uField1; 			// Unsigned 16-bit data field
	INT32	u32Field; 			// Signed 32-bit data field
	INT32	u32Field1, 			// Signed 32-bit data fields
			u32Field2;
 } STRU_TELE_USER_16U32SU;

//////////////////////////////////////////////////////////////////
//
//		POWERBOX
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;			// Source device = 0x0A
	UINT8	sID;			// Secondary ID
	UINT16	volt1;			// Volts, 0v01v
	UINT16	volt2;			// Volts, 0.01v
	UINT16	capacity1;			// mAh, 1mAh
	UINT16	capacity2;			// mAh, 1mAh
	UINT16	spare16_1;
	UINT16	spare16_2;
	UINT8	spare;
	UINT8	alarms;			// Alarm bitmask (see below)
} STRU_TELE_POWERBOX;

#define	TELE_PBOX_ALARM_VOLTAGE_1		(0x01)
#define	TELE_PBOX_ALARM_VOLTAGE_2		(0x02)
#define	TELE_PBOX_ALARM_CAPACITY_1	(0x04)
#define	TELE_PBOX_ALARM_CAPACITY_2	(0x08)
//#define	TELE_PBOX_ALARM_RPM		(0x10)
//#define	TELE_PBOX_ALARM_TEMPERATURE	(0x20)
#define	TELE_PBOX_ALARM_RESERVED_1	(0x40)
#define	TELE_PBOX_ALARM_RESERVED_2	(0x80)

//////////////////////////////////////////////////////////////////
//
//		DUAL ENERGY
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	id;			// Source device = 0x18
	UINT8	sID;			// Secondary ID
	INT16	current_A;			// Instantaneous current, 0.01A (0-328.7A)
	INT16	chargeUsed_A;			// Integrated mAh used, 0.1mAh (0-3276.6mAh)
	UINT16	volts_A;			// Voltage, 0.01VC (0-16.00V)
	INT16	current_B;			// Instantaneous current, 0.1A (0-3276.8A)
	INT16	chargeUsed_B;			// Integrated mAh used, 1mAh (0-32.766Ah)
	UINT16	volts_B;			// Voltage, 0.01VC (0-16.00V)
	UINT16	spare;			// Not used
} STRU_TELE_ENERGY_DUAL;

//////////////////////////////////////////////////////////////////
//
//		HIGH-CURRENT
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;			// Source device = 0x03
	UINT8	sID;			// Secondary ID
	INT16	current,			// Range: +/- 150A     
						// Resolution: 300A/2048 = 0.196791 A/tick
			dummy;			// TBD
} STRU_TELE_IHIGH;

#define	IHIGH_RESOLUTION_FACTOR		((FP32)(0.196791))

//////////////////////////////////////////////////////////////////
//
//		VARIO-S
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;			// Source device = 0x40
	UINT8	sID;			// Secondary ID
	INT16	altitude;			// .1m increments
	INT16	delta_0250ms,			// delta last 250ms, 0.1m/s increments
			delta_0500ms,			// delta last 500ms, 0.1m/s increments
			delta_1000ms,			// delta last 1.0 seconds
			delta_1500ms,			// delta last 1.5 seconds
			delta_2000ms,			// delta last 2.0 seconds
			delta_3000ms;			// delta last 3.0 seconds
} STRU_TELE_VARIO_S;

//////////////////////////////////////////////////////////////////
//
//		ALTIMETER
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;
	UINT8	sID;			// Secondary ID
	INT16	altitude;			// .1m increments
	INT16	maxAltitude;			// .1m increments
} STRU_TELE_ALT;

//////////////////////////////////////////////////////////////////
//
//		AIRSPEED
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;
	UINT8	sID;			// Secondary ID
	UINT16	airspeed;			// 1 km/h increments
	UINT16	maxAirspeed;			// 1 km/h increments
} STRU_TELE_SPEED;

//////////////////////////////////////////////////////////////////
//
//		GFORCE
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;			// Source device = 0x14
	UINT8	sID;			// Secondary ID
	INT16	GForceX;			// force is reported as .01G increments
	INT16	GForceY;			// Range = +/-4000 (+/- 40G) in Pro model
	INT16	GForceZ;			// Range = +/-800 (+/- 8G) in Standard model
	INT16	maxGForceX;			// abs(max G X-axis)   FORE/AFT
	INT16	maxGForceY;			// abs (max G Y-axis)  LEFT/RIGHT
	INT16	maxGForceZ;			// max G Z-axis        WING SPAR LOAD
	INT16	minGForceZ;			// min G Z-axis        WING SPAR LOAD
} STRU_TELE_G_METER;

//////////////////////////////////////////////////////////////////
//
//		JETCAT/TURBINE
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;			// Source device = 0x15
	UINT8	sID;			// Secondary ID
	UINT8	status;			// See table below
	UINT8	throttle;			// (BCD) xx Percent
	UINT16	packVoltage;			// (BCD) xx.yy
	UINT16	pumpVoltage;			// (BCD) xx.yy
	UINT32	RPM;			// (BCD) 
	UINT16	EGT;			// (BCD) Temperature, Celsius
	UINT8	offCondition;			// (BCD) See table below
	UINT8	spare;
} STRU_TELE_JETCAT;	

enum JETCAT_ECU_TURBINE_STATE {		// ECU Status definitions
		JETCAT_ECU_STATE_OFF = 0x00,
		JETCAT_ECU_STATE_WAIT_for_RPM = 0x01, // (Stby/Start)
		JETCAT_ECU_STATE_Ignite = 0x02,
		JETCAT_ECU_STATE_Accelerate = 0x03,
		JETCAT_ECU_STATE_Stabilise = 0x04,
		JETCAT_ECU_STATE_Learn_HI = 0x05,
		JETCAT_ECU_STATE_Learn_LO = 0x06,
		JETCAT_ECU_STATE_UNDEFINED = 0x07,
		JETCAT_ECU_STATE_Slow_Down = 0x08,
		JETCAT_ECU_STATE_Manual = 0x09,
		JETCAT_ECU_STATE_AutoOff = 0x10,
		JETCAT_ECU_STATE_Run = 0x11, // (reg.)
		JETCAT_ECU_STATE_Accleleration_delay = 0x12,
		JETCAT_ECU_STATE_SpeedReg = 0x13, // (Speed Ctrl)
		JETCAT_ECU_STATE_Two_Shaft_Regulate = 0x14, // (only for secondary shaft)
		JETCAT_ECU_STATE_PreHeat1 = 0x15,
		JETCAT_ECU_STATE_PreHeat2 = 0x16,
		JETCAT_ECU_STATE_MainFStart = 0x17,
		JETCAT_ECU_STATE_NotUsed = 0x18,
		JETCAT_ECU_STATE_KeroFullOn = 0x19,
		// undefined states 0x1A-0x1F
		EVOJET_ECU_STATE_off = 0x20,
		EVOJET_ECU_STATE_ignt = 0x21,
		EVOJET_ECU_STATE_acce = 0x22,
		EVOJET_ECU_STATE_run = 0x23,
		EVOJET_ECU_STATE_cal = 0x24,
		EVOJET_ECU_STATE_cool = 0x25,
		EVOJET_ECU_STATE_fire = 0x26,
		EVOJET_ECU_STATE_glow = 0x27,
		EVOJET_ECU_STATE_heat = 0x28,
		EVOJET_ECU_STATE_idle = 0x29,
		EVOJET_ECU_STATE_lock = 0x2A,
		EVOJET_ECU_STATE_rel = 0x2B,
		EVOJET_ECU_STATE_spin = 0x2C,
		EVOJET_ECU_STATE_stop = 0x2D,
		// undefined states 0x2E-0x2F
		HORNET_ECU_STATE_OFF = 0x30,
		HORNET_ECU_STATE_SLOWDOWN = 0x31,
		HORNET_ECU_STATE_COOL_DOWN = 0x32,
		HORNET_ECU_STATE_AUTO = 0x33,
		HORNET_ECU_STATE_AUTO_HC = 0x34,
		HORNET_ECU_STATE_BURNER_ON = 0x35,
		HORNET_ECU_STATE_CAL_IDLE = 0x36,
		HORNET_ECU_STATE_CALIBRATE = 0x37,
		HORNET_ECU_STATE_DEV_DELAY = 0x38,
		HORNET_ECU_STATE_EMERGENCY = 0x39,
		HORNET_ECU_STATE_FUEL_HEAT = 0x3A,
		HORNET_ECU_STATE_FUEL_IGNITE = 0x3B,
		HORNET_ECU_STATE_GO_IDLE = 0x3C,
		HORNET_ECU_STATE_PROP_IGNITE = 0x3D,
		HORNET_ECU_STATE_RAMP_DELAY = 0x3E,
		HORNET_ECU_STATE_RAMP_UP = 0x3F,
		HORNET_ECU_STATE_STANDBY = 0x40,
		HORNET_ECU_STATE_STEADY = 0x41,
		HORNET_ECU_STATE_WAIT_ACC = 0x42,
		HORNET_ECU_STATE_ERROR = 0x43,
		// undefined states 0x44-0x4F
		XICOY_ECU_STATE_Temp_High = 0x50,
		XICOY_ECU_STATE_Trim_Low = 0x51,
		XICOY_ECU_STATE_Set_Idle = 0x52,
		XICOY_ECU_STATE_Ready = 0x53,
		XICOY_ECU_STATE_Ignition = 0x54,
		XICOY_ECU_STATE_Fuel_Ramp = 0x55,
		XICOY_ECU_STATE_Glow_Test = 0x56,
		XICOY_ECU_STATE_Running = 0x57,
		XICOY_ECU_STATE_Stop = 0x58,
		XICOY_ECU_STATE_Flameout = 0x59,
		XICOY_ECU_STATE_Speed_Low = 0x5A,
		XICOY_ECU_STATE_Cooling = 0x5B,
		XICOY_ECU_STATE_Igniter_Bad = 0x5C,
		XICOY_ECU_STATE_Starter_F = 0x5D,
		XICOY_ECU_STATE_Weak_Fuel = 0x5E,
		XICOY_ECU_STATE_Start_On = 0x5F,
		XICOY_ECU_STATE_Pre_Heat = 0x60,
		XICOY_ECU_STATE_Battery = 0x61,
		XICOY_ECU_STATE_Time_Out = 0x62,
		XICOY_ECU_STATE_Overload = 0x63,
		XICOY_ECU_STATE_Igniter_Fail = 0x64,
		XICOY_ECU_STATE_Burner_On = 0x65,
		XICOY_ECU_STATE_Starting = 0x66,
		XICOY_ECU_STATE_SwitchOver = 0x67,
		XICOY_ECU_STATE_Cal_Pump = 0x68,
		XICOY_ECU_STATE_Pump_Limit = 0x69,
		XICOY_ECU_STATE_No_Engine = 0x6A,
		XICOY_ECU_STATE_Pwr_Boost = 0x6B,
		XICOY_ECU_STATE_Run_Idle = 0x6C,
		XICOY_ECU_STATE_Run_Max = 0x6D,

		TURBINE_ECU_MAX_STATE = 0x74
};

enum JETCAT_ECU_OFF_CONDITIONS {		// ECU off conditions. Valid only when the ECUStatus = JETCAT_ECU_STATE_OFF
		JETCAT_ECU_OFF_No_Off_Condition_defined = 0,
		JETCAT_ECU_OFF_Shut_down_via_RC,
		JETCAT_ECU_OFF_Overtemperature,
		JETCAT_ECU_OFF_Ignition_timeout,
		JETCAT_ECU_OFF_Acceleration_time_out,
		JETCAT_ECU_OFF_Acceleration_too_slow,
		JETCAT_ECU_OFF_Over_RPM,
		JETCAT_ECU_OFF_Low_Rpm_Off,
		JETCAT_ECU_OFF_Low_Battery,
		JETCAT_ECU_OFF_Auto_Off,
		JETCAT_ECU_OFF_Low_temperature_Off,
		JETCAT_ECU_OFF_Hi_Temp_Off,
		JETCAT_ECU_OFF_Glow_Plug_defective,
		JETCAT_ECU_OFF_Watch_Dog_Timer,
		JETCAT_ECU_OFF_Fail_Safe_Off,
		JETCAT_ECU_OFF_Manual_Off, // (via GSU)
		JETCAT_ECU_OFF_Power_fail, // (Battery fail)
		JETCAT_ECU_OFF_Temp_Sensor_fail, // (only during startup)
		JETCAT_ECU_OFF_Fuel_fail,
		JETCAT_ECU_OFF_Prop_fail,
		JETCAT_ECU_OFF_2nd_Engine_fail,
		JETCAT_ECU_OFF_2nd_Engine_Diff_Too_High,
		JETCAT_ECU_OFF_2nd_Engine_No_Comm,
		JETCAT_ECU_MAX_OFF_COND
};

typedef struct
{
	UINT8	identifier;			// Source device = 0x19
	UINT8	sID;			// Secondary ID
	UINT16	FuelFlowRateMLMin;			// (BCD) mL per Minute
	UINT32	RestFuelVolumeInTankML;		// (BCD) mL remaining in tank
	// 8 bytes left
} STRU_TELE_JETCAT2;

//////////////////////////////////////////////////////////////////
//
//		GPS
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;			// Source device = 0x16
	UINT8	sID;			// Secondary ID
	UINT16	altitudeLow;			// BCD, meters, format 3.1 (Low bits of alt)
	UINT32	latitude;			// BCD, format 4.4, 
						// Degrees * 100 + minutes, < 100 degrees
	UINT32	longitude;			// BCD, format 4.4, 
						// Degrees * 100 + minutes, flag --> > 99deg
	UINT16	course;			// BCD, 3.1
	UINT8	HDOP;			// BCD, format 1.1
	UINT8	GPSflags;			// see definitions below
} STRU_TELE_GPS_LOC;

typedef struct
{
	UINT8	identifier;			// Source device = 0x17
	UINT8	sID;			// Secondary ID
	UINT16	speed;			// BCD, knots, format 3.1
	UINT32	UTC;			// BCD, format HH:MM:SS.S, format 6.1
	UINT8	numSats;			// BCD, 0-99
	UINT8	altitudeHigh;			// BCD, meters, format 2.0 (High bits alt)
} STRU_TELE_GPS_STAT;

// GPS flags definitions:
#define	GPS_INFO_FLAGS_IS_NORTH_BIT	(0)
#define	GPS_INFO_FLAGS_IS_NORTH		(1 << GPS_INFO_FLAGS_IS_NORTH_BIT)
#define	GPS_INFO_FLAGS_IS_EAST_BIT	(1)
#define	GPS_INFO_FLAGS_IS_EAST		(1 << GPS_INFO_FLAGS_IS_EAST_BIT)
#define	GPS_INFO_FLAGS_LONG_GREATER_99_BIT	(2)
#define	GPS_INFO_FLAGS_LONG_GREATER_99 	(1 << GPS_INFO_FLAGS_LONG_GREATER_99_BIT)
#define	GPS_INFO_FLAGS_GPS_FIX_VALID_BIT	(3)
#define	GPS_INFO_FLAGS_GPS_FIX_VALID	(1 << GPS_INFO_FLAGS_GPS_FIX_VALID_BIT)
#define	GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT	(4)
#define	GPS_INFO_FLAGS_GPS_DATA_RECEIVED	(1 << GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT)
#define	GPS_INFO_FLAGS_3D_FIX_BIT		(5)
#define	GPS_INFO_FLAGS_3D_FIX		(1 << GPS_INFO_FLAGS_3D_FIX_BIT)
#define	GPS_INFO_FLAGS_NEGATIVE_ALT_BIT	(7)
#define	GPS_INFO_FLAGS_NEGATIVE_ALT	(1 << GPS_INFO_FLAGS_NEGATIVE_ALT_BIT)

//////////////////////////////////////////////////////////////////
//
//		GYRO
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;		// Source device = 0x1A
	UINT8		sID;			// Secondary ID
	INT16		gyroX;			// Rotation rates of the body - Rate 
							// is about the X Axis which is
							// defined out the nose of the
							// vehicle.
	INT16		gyroY;			// Units are 0.1 deg/sec  - Rate is
							// about the Y Axis which is defined
							// out the right wing of the vehicle.
	INT16		gyroZ;			// Rate is about the Z axis which is
							// defined down from the vehicle.
	INT16		maxGyroX;		// Max rates (absolute value)
	INT16		maxGyroY;
	INT16		maxGyroZ;
} STRU_TELE_GYRO;

//////////////////////////////////////////////////////////////////
//
//		Alpha6 Stabilizer
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;		// Source device = 0x24
	UINT8		sID;			// Secondary ID
	UINT16		volts;			// 0.01V increments
	UINT8		state_FM;		// Flight Mode and System State 
							// (see below)
	UINT8		gainRoll,		// Roll Gain,  high bit --> 
							//    Heading Hold
				gainPitch,		// Pitch Gain
				gainYaw;		// Yaw Gain
	INT16		attRoll,		// Roll Attitude, 0.1degree, RHR
				attPitch,		// Pitch Attitude
				attYaw;		// Yaw Attitude
	UINT16		spare;
} STRU_TELE_ALPHA6;

#define	GBOX_STATE_BOOT		(0x00)		// Alpha6 State - Boot
#define	GBOX_STATE_INIT		(0x01)		// Init
#define	GBOX_STATE_READY		(0x02)		// Ready
#define	GBOX_STATE_SENSORFAULT	(0x03)		// Sensor Fault
#define	GBOX_STATE_POWERFAULT	(0x04)		// Power Fault
#define	GBOX_STATE_MASK		(0x0F)

#define	GBOX_FMODE_FM0		(0x00)		// FM0 through FM4
#define	GBOX_FMODE_FM1		(0x10)
#define	GBOX_FMODE_FM2		(0x20)
#define	GBOX_FMODE_FM3		(0x30)
#define	GBOX_FMODE_FM4		(0x40)
#define	GBOX_FMODE_PANIC		(0x50)
#define	GBOX_FMODE_MASK		(0xF0)

//////////////////////////////////////////////////////////////////
//
//		6S LiPo Cell Monitor
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;		// Source device = 0x3A
	UINT8		sID;			// Secondary ID
	UINT16		cell[6];		// Voltage across cell 1, .01V steps
							// 0x7FFF --> cell not present
	UINT16		temp;			// Temperature, 0.1C (0-655.34C)
} STRU_TELE_LIPOMON;

//////////////////////////////////////////////////////////////////
//
//		14S LiPo Cell Monitor
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;		// Source device = 0x3F
	UINT8		sID;			// Secondary ID
	UINT8		cell[14];		// Voltage across cell 1, .01V steps,
							// excess of 2.56V (ie, 3.00V would
							// report 300-256 = 44)
							// 0xFF --> cell not present
} STRU_TELE_LIPOMON_14;

//////////////////////////////////////////////////////////////////
//
//		ATTITUDE & MAG COMPASS
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;		// Source device = 0x1B
	UINT8		sID;			// Secondary ID
	INT16		attRoll;		// Attitude, 3 axes.  Roll is a
							// rotation about the X Axis of 
							// the vehicle using the RHR.
	INT16		attPitch;		// Units are 0.1 deg - Pitch is a
							// rotation about the Y Axis of the
							// vehicle using the RHR.
	INT16		attYaw;		// Yaw is a rotation about the Z 
							// Axis of the vehicle using the RHR.
	INT16		magX;			// Magnetic Compass, 3 axes
	INT16		magY;			// Units are TBD
	INT16		magZ;			// 
} STRU_TELE_ATTMAG;

//////////////////////////////////////////////////////////////////
//
//				Real-Time Clock
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;		// Source device = 0x7C
	UINT8		sID;			// Secondary ID
	UINT8		spare[6];
	UINT64		UTC64;			// Linux 64-bit time_t for 
							// post-2038 date compatibility
} STRU_TELE_RTC;

//////////////////////////////////////////////////////////////////
//
//		RPM/Volts/Temperature
//
//////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;			// Source device = 0x7E
	UINT8	sID;			// Secondary ID
	UINT16	microseconds;			// microseconds between pulse leading edges
	UINT16	volts;			// 0.01V increments
	INT16	temperature;			// degrees F
	INT8	dBm_A,			// Average signal for A antenna in dBm
			dBm_B;			// Average signal for B antenna in dBm.  
						// If only 1 antenna, set B = A
} STRU_TELE_RPM;

//////////////////////////////////////////////////////////////////
//
//		QoS DATA
//
//////////////////////////////////////////////////////////////////
//
//	NOTE:  AR6410-series send:
//			id = 7F
//			sID = 0
//			A = 0
//			B = 0
//			L = 0
//			R = 0
//			F = fades
//			H = holds
//			rxV = 0xFFFF
//
typedef struct
{
	UINT8	identifier;			// Source device = 0x7F
	UINT8	sID;			// Secondary ID
	UINT16	A;
	UINT16	B;
	UINT16	L;
	UINT16	R;
	UINT16	F;
	UINT16	H;
	UINT16	rxVoltage;			// Volts, 0.01V increments
} STRU_TELE_QOS;

//////////////////////////////////////////////////////////////////
//
//		UNION OF ALL DEVICE MESSAGES
//
//////////////////////////////////////////////////////////////////
//
typedef union
{
	UINT16		raw[8];
	STRU_TELE_RTC	rtc;
	STRU_TELE_QOS	qos;
	STRU_TELE_RPM	rpm;
	STRU_TELE_FRAMEDATA	frame;
	STRU_TELE_ALT	alt;
	STRU_TELE_SPEED	speed;
	STRU_TELE_ENERGY_DUAL	eDual;
	STRU_TELE_VARIO_S	varioS;
	STRU_TELE_G_METER	accel;
	STRU_TELE_JETCAT	jetcat;
	STRU_TELE_JETCAT2	jetcat2;
	STRU_TELE_GPS_LOC	gpsloc;
	STRU_TELE_GPS_STAT	gpsstat;
	STRU_TELE_GYRO	gyro;
	STRU_TELE_ATTMAG	attMag;
	STRU_TELE_POWERBOX	powerBox;
	STRU_TELE_ESC	escGeneric;
	STRU_TELE_LAPTIMER	lapTimer;
	STRU_TELE_TEXTGEN	textgen;
	STRU_TELE_FUEL	fuel;
	STRU_TELE_MAH	mAh;
	STRU_TELE_DIGITAL_AIR	digAir;
	STRU_TELE_STRAIN	strain;
	STRU_TELE_LIPOMON	lipomon;
	STRU_TELE_LIPOMON_14	lipomon14;
	STRU_TELE_USER_16SU	user_16SU;
	STRU_TELE_USER_16SU32U	user_16SU32U;
	STRU_TELE_USER_16SU32S	user_16SU32S;
	STRU_TELE_USER_16U32SU	user_16U32SU;
} UN_TELEMETRY;			// All telemetry messages

