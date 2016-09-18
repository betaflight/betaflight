/*
 Created by Marcin Baliniak
 some functions based on MinimOSD
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include <string.h>
#include "platform.h"

#include "debug.h"
#include "version.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/printf.h"
#include "common/typeconversion.h"

#include "drivers/gpio.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/compass.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "drivers/light_ws2811strip.h"
#include "drivers/sound_beeper.h"
#include "drivers/max7456.h"
#include "drivers/max7456_symbols.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/flashfs.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/beeper.h"
#include "io/osd.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#define IS_HI(X) (rcData[X] > 1750)
#define IS_LO(X) (rcData[X] < 1250)
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)

//key definiotion because API provide menu navigation over MSP/GUI app - not used NOW
#define KEY_ENTER	0
#define KEY_UP		1
#define KEY_DOWN	2
#define KEY_LEFT	3
#define KEY_RIGHT	4
#define KEY_ESC		5

//osd current screen - to reduce long lines ;-)
#define OSD_cfg masterConfig.osdProfile
#define curr_profile masterConfig.profile[masterConfig.current_profile_index]

uint16_t RefreshTimeout = 0;

#define VISIBLE_FLAG	0x0800
#define BLINK_FLAG	0x0400
uint8_t BLINK_STATE = 1;

#define OSD_POS(x,y)	(x | (y << 5))
#define OSD_X(x)	(x & 0x001F)
#define OSD_Y(x)	((x >> 5) & 0x001F)
#define VISIBLE(x)	(x & VISIBLE_FLAG)
#define BLINK(x)	((x & BLINK_FLAG) && BLINK_STATE)
#define BLINK_OFF(x) (x & ~BLINK_FLAG)
extern uint8_t RSSI;
static uint16_t fly_time = 0;

tStatistic stats;

#define BUTTON_TIME		2
#define BUTTON_PAUSE	5
#define REFRESH_1S	12

#define LEFT_MENU_COLUMN	1
#define RIGHT_MENU_COLUMN	23
#define MAX_MENU_ITEMS	(max7456_get_rows_count()-2)

uint8_t OSD_ROWS;

//type of elements
typedef enum
{
    OME_Label,
    OME_Back,
    OME_OSD_Exit,
    OME_Submenu,
    OME_Bool,
    OME_INT8,
    OME_UINT8,
    OME_UINT16,
    OME_INT16,
    OME_FLOAT, //only up to 255 value and cant be 2.55 or 25.5, just for PID's
    //wlasciwosci elementow
    OME_VISIBLE,
    OME_POS,
    OME_TAB,
    OME_END,
} OSD_MenuElement;

//local variable to detect arm/disarm and show statistic etc
uint8_t arm_state;
uint8_t	blackbox_feature = 0;
uint8_t	ledstrip_feature = 0;

#if defined(VTX) || defined(USE_RTC6705)
uint8_t vtx_feature = 0, vtx_band, vtx_channel;
#endif

//we are in menu flag
uint8_t OSD_MENU = 0;

typedef void (* OSDMenuFuncPtr)(void *data);

void OSD_Exit(void * ptr);
void OSD_MenuBack(void);
void OSD_EditElement(void *ptr);
void OSD_EraseFlash(void *ptr);
void OSD_UpdateMaxRows(void);
void OSD_ChangeScreen(void * ptr);

typedef struct
{
	char *text;
	OSD_MenuElement type;
	OSDMenuFuncPtr func;
	void *data;
} OSD_Entry;

typedef struct
{
	uint8_t *val;
	uint8_t min;
	uint8_t max;
	uint8_t step;
} OSD_UINT8_Struct;

typedef struct
{
	int8_t *val;
	int8_t min;
	int8_t max;
	int8_t step;
} OSD_INT8_Struct;

typedef struct
{
	int16_t *val;
	int16_t min;
	int16_t max;
	int16_t step;
} OSD_INT16_Struct;

typedef struct
{
	uint16_t *val;
	uint16_t min;
	uint16_t max;
	uint16_t step;
} OSD_UINT16_Struct;

typedef struct
{
	uint8_t *val;
	uint8_t min;
	uint8_t max;
	uint8_t step;
	uint16_t multipler;
} OSD_FLOAT_Struct;

typedef struct
{
	uint8_t *val;
	uint8_t max;
	const char * const *names;
} OSD_TAB_Struct;

OSD_Entry *MenuStack[10]; //tab to save menu stack
uint8_t MenuStackPos[10]; //current position in menu stack
uint8_t MenuStackIdx = 0;

OSD_Entry *CurrentMenu;
OSD_Entry *NextPage = NULL;

int8_t CurrentMenuPos = 0;
uint8_t CurrentMaxIdx = 0;
uint16_t *CurrentElement = NULL;

OSD_Entry MenuAlarms[];
OSD_Entry MenuLayout[];
OSD_Entry MenuLayoutActiv[];
OSD_Entry MenuLayoutPos[];
OSD_Entry MenuFeatures[];
OSD_Entry MenuBlackbox[];
#ifdef LED_STRIP
OSD_Entry MenuLedstrip[];
#endif
#if defined(VTX) || defined(USE_RTC6705)
OSD_Entry MenuVtx[];
#endif
OSD_Entry MenuIMU[];
OSD_Entry MenuPID[];
OSD_Entry MenuRc[];
OSD_Entry MenuRateExpo[];

uint8_t stat_RSSI;

OSD_Entry MainMenu[]=
{
	{"----MAIN MENU----", OME_Label, NULL, NULL},
	{"SCREEN LAYOUT", OME_Submenu, OSD_ChangeScreen, &MenuLayout[0]},
	{"ALARMS", OME_Submenu, OSD_ChangeScreen, &MenuAlarms[0]},
	{"CFG. IMU", OME_Submenu, OSD_ChangeScreen, &MenuIMU[0]},
	{"FEATURES", OME_Submenu, OSD_ChangeScreen, &MenuFeatures[0]},
	{"SAVE & EXIT", OME_OSD_Exit, OSD_Exit, (void*)1},
	{"EXIT", OME_OSD_Exit, OSD_Exit, (void*)0},
	{NULL,OME_END, NULL, NULL}
};

OSD_Entry MenuFeatures[]=
{
	{"----- FEATURES -----", OME_Label, NULL, NULL},
	{"BLACKBOX", OME_Submenu, OSD_ChangeScreen, &MenuBlackbox[0]},
	#ifdef LED_STRIP
	{"LED STRIP", OME_Submenu, OSD_ChangeScreen, &MenuLedstrip[0]},
	#endif
	#if defined(VTX) || defined(USE_RTC6705)
	{"VTX", OME_Submenu, OSD_ChangeScreen, &MenuVtx[0]},
	#endif
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};


OSD_UINT8_Struct blackbox_rate_denom_entry = {&masterConfig.blackbox_rate_denom,1,32,1};

OSD_Entry MenuBlackbox[]=
{
	{"--- BLACKBOX ---", OME_Label, NULL, NULL},
	{"ENABLED", OME_Bool, NULL, &blackbox_feature},
	{"RATE DENOM", OME_UINT8, NULL, &blackbox_rate_denom_entry},
#ifdef USE_FLASHFS
	{"ERASE FLASH", OME_Submenu, OSD_EraseFlash, NULL},
#endif
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};

#ifdef LED_STRIP

//local variable to keep color value
uint8_t ledColor;

static const char * const ledColorNames[] = {
    "      BLACK",
    "      WHITE",
    "        RED",
    "     ORANGE",
    "     YELLOW",
    " LIME GREEN",
    "      GREEN",
    " MINT GREEN",
    "       CYAN",
    " LIGHT BLUE",
    "       BLUE",
    "DARK VIOLET",
    "    MAGNETA",
    "  DEEP PINK",
};

//find first led with color flag and restore color index
//after saving all leds with flags color will have color set in OSD
void getLedColor(void)
{
    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        const ledConfig_t *ledConfig = &masterConfig.ledConfigs[ledIndex];

        int fn = ledGetFunction(ledConfig);

        if (fn == LED_FUNCTION_COLOR)
		{
			ledColor = ledGetColor(ledConfig);
			break;
		}
    }
}

//udate all leds with flag color
void applyLedColor(void * ptr)
{
	UNUSED(ptr);
    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        ledConfig_t *ledConfig = &masterConfig.ledConfigs[ledIndex];
        if (ledGetFunction(ledConfig) == LED_FUNCTION_COLOR)
            *ledConfig = DEFINE_LED(ledGetX(ledConfig), ledGetY(ledConfig), ledColor, ledGetDirection(ledConfig), ledGetFunction(ledConfig), ledGetOverlay(ledConfig), 0);
    }
    updateLedStrip();
}

OSD_TAB_Struct led_entry = {&ledColor,13,&ledColorNames[0]};

OSD_Entry MenuLedstrip[]=
{
	{"--- LED TRIP ---", OME_Label, NULL, NULL},
	{"ENABLED", OME_Bool, NULL, &ledstrip_feature},
	{"LED COLOR", OME_TAB, applyLedColor, &led_entry},
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};
#endif


#if defined(VTX) || defined(USE_RTC6705)
static const char * const vtxBandNames[] = {
	"BOACAM A",
	"BOSCAM B",
	"BOSCAM E",
	"FATSHARK",
	"RACEBAND",
};

OSD_TAB_Struct vtx_band_entry = {&vtx_band,4,&vtxBandNames[0]};
OSD_UINT8_Struct vtx_channel_entry =  {&vtx_channel, 1, 8, 1};

#ifdef VTX
OSD_UINT8_Struct vtx_mode_entry =  {&masterConfig.vtx_mode, 0, 2, 1};
OSD_UINT16_Struct vtx_mhz_entry =  {&masterConfig.vtx_mhz, 5600, 5950, 1};
#endif

OSD_Entry MenuVtx[]=
{
	{"--- VTX ---", OME_Label, NULL, NULL},
	{"ENABLED", OME_Bool, NULL, &vtx_feature},
#ifdef VTX
	{"VTX MODE", OME_UINT8, NULL, &vtx_mode_entry},
	{"VTX MHZ", OME_UINT16, NULL, &vtx_mhz_entry},
#endif
	{"BAND", OME_TAB, NULL, &vtx_band_entry},
	{"CHANNEL", OME_UINT8, NULL, &vtx_channel_entry},
#ifdef USE_RTC6705
	{"LOW POWER", OME_Bool, NULL, &masterConfig.vtx_power},
#endif
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};
#endif

OSD_UINT16_Struct minthrottle_entry = {&masterConfig.escAndServoConfig.minthrottle, 1020, 1300, 10};
OSD_UINT8_Struct gyro_soft_lpf_hz_entry = {&masterConfig.gyro_soft_lpf_hz, 0, 255, 1};
OSD_UINT16_Struct dterm_lpf_entry = {&masterConfig.profile[0].pidProfile.dterm_lpf_hz, 0, 500, 1};


OSD_Entry MenuMisc[]=
{
	{"----- MISC -----", OME_Label, NULL, NULL},
	{"GYRO LOWPASS", OME_UINT8, NULL, &gyro_soft_lpf_hz_entry},
	{"DTERM LPF", OME_UINT16, NULL, &dterm_lpf_entry},
	{"MINTHROTTLE", OME_UINT16, NULL, &minthrottle_entry},
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};


OSD_UINT8_Struct pid_profile_entry = {&masterConfig.current_profile_index, 0, MAX_PROFILE_COUNT, 1};

OSD_Entry MenuIMU[]=
{
	{"-----CFG. IMU-----", OME_Label, NULL, NULL},
	{"PID", OME_Submenu, OSD_ChangeScreen, &MenuPID[0]},
	{"PID PROFILE", OME_UINT8, NULL, &pid_profile_entry},
	{"RATE & RXPO", OME_Submenu, OSD_ChangeScreen, &MenuRateExpo[0]},
	{"RC PREVIEW", OME_Submenu, OSD_ChangeScreen, &MenuRc[0]},
	{"MISC", OME_Submenu, OSD_ChangeScreen, &MenuMisc[0]},
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};


uint8_t temp_pids[4][3];

static OSD_UINT8_Struct RollP = {&temp_pids[PIDROLL][0],10,150,1};
static OSD_UINT8_Struct RollI = {&temp_pids[PIDROLL][1],1,150,1};
static OSD_UINT8_Struct RollD = {&temp_pids[PIDROLL][2],0,150,1};

static OSD_UINT8_Struct PitchP = {&temp_pids[PIDPITCH][0],10,150,1};
static OSD_UINT8_Struct PitchI = {&temp_pids[PIDPITCH][1],1,150,1};
static OSD_UINT8_Struct PitchD = {&temp_pids[PIDPITCH][2],0,150,1};

static OSD_UINT8_Struct YawP = {&temp_pids[PIDYAW][0],10,150,1};
static OSD_UINT8_Struct YawI = {&temp_pids[PIDYAW][1],1,150,1};
static OSD_UINT8_Struct YawD = {&temp_pids[PIDYAW][2],0,150,1};

//static OSD_UINT8_Struct LevelP = {&temp_pids[3][0],10,150,1};
//static OSD_FLOAT_Struct LevelI = {&temp_pids[3][1],1,150,1,1};
//static OSD_FLOAT_Struct LevelD = {&temp_pids[3][2],10,150,1,1000};

OSD_Entry MenuPID[]=
{
	{"------- PID -------", OME_Label, NULL, NULL},
	{"ROLL P", OME_UINT8, NULL, &RollP},
	{"ROLL I", OME_UINT8, NULL, &RollI},
	{"ROLL D", OME_UINT8, NULL, &RollD},

	{"PITCH P", OME_UINT8, NULL, &PitchP},
	{"PITCH I", OME_UINT8, NULL, &PitchI},
	{"PITCH D", OME_UINT8, NULL, &PitchD},

	{"YAW P", OME_UINT8, NULL, &YawP},
	{"YAW I", OME_UINT8, NULL, &YawI},
	{"YAW D", OME_UINT8, NULL, &YawD},

//	{"LEVEL A", OME_FLOAT, NULL, &LevelP},
	//{"LEVEL H", OME_FLOAT, NULL, &LevelI},
	//{"LEVEL H SENS.", OME_FLOAT, NULL, &LevelD},

	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};

controlRateConfig_t rateProfile;

static OSD_FLOAT_Struct RollRate = {&rateProfile.rates[0],0,250,1,10};
static OSD_FLOAT_Struct PitchRate = {&rateProfile.rates[1],0,250,1,10};
static OSD_FLOAT_Struct YawRate = {&rateProfile.rates[2],1,250,1,10};
static OSD_FLOAT_Struct RCRate = {&rateProfile.rcRate8,30,200,1,10};
static OSD_FLOAT_Struct RCExpo = {&rateProfile.rcExpo8,1,100,1,10};
static OSD_FLOAT_Struct RCYawExpo = {&rateProfile.rcYawExpo8,1,100,1,10};
static OSD_FLOAT_Struct TPAentry = {&rateProfile.dynThrPID,0,70,1,10};
static OSD_UINT16_Struct TPAbreak = {&rateProfile.tpa_breakpoint,1100,1800,10};

static OSD_FLOAT_Struct P_Setpoint_entry = {&masterConfig.profile[0].pidProfile.ptermSRateWeight,0,100,1,100};
static OSD_FLOAT_Struct D_Setpoint_entry = {&masterConfig.profile[0].pidProfile.dtermSetpointWeight,0,255,1,100};

OSD_Entry MenuRateExpo[]=
{
	{"----RATE & EXPO----", OME_Label, NULL, NULL},
	{"ROLL RATE", OME_FLOAT, NULL, &RollRate},
	{"PITCH RATE", OME_FLOAT, NULL, &PitchRate},
	{"YAW RATE", OME_FLOAT, NULL, &YawRate},
	{"RC RATE", OME_FLOAT, NULL, &RCRate},
	{"RC EXPO", OME_FLOAT, NULL, &RCExpo},
	{"RC YAW EXPO", OME_FLOAT, NULL, &RCYawExpo},
	{"THR. PID ATT.", OME_FLOAT, NULL, &TPAentry},
	{"TPA BREAKPOINT", OME_UINT16, NULL, &TPAbreak},
	{"PTERM SRATE RATIO", OME_FLOAT, NULL, &P_Setpoint_entry},
	{"D SETPOINT", OME_FLOAT, NULL, &D_Setpoint_entry},
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};

static OSD_INT16_Struct rollEntry = {&rcData[ROLL],1,2500,0};
static OSD_INT16_Struct pitchEntry = {&rcData[PITCH],1,2500,0};
static OSD_INT16_Struct throttleEntry = {&rcData[THROTTLE],1,2500,0};
static OSD_INT16_Struct yawEntry = {&rcData[YAW],1,2500,0};
static OSD_INT16_Struct aux1Entry = {&rcData[AUX1],1,2500,0};
static OSD_INT16_Struct aux2Entry = {&rcData[AUX2],1,2500,0};
static OSD_INT16_Struct aux3Entry = {&rcData[AUX3],1,2500,0};
static OSD_INT16_Struct aux4Entry = {&rcData[AUX4],1,2500,0};

OSD_Entry MenuRc[]=
{
	{"---- RC PREVIEW ----", OME_Label, NULL, NULL},
	{"ROLL", OME_INT16, NULL, &rollEntry},
	{"PITCH", OME_INT16, NULL, &pitchEntry},
	{"THROTTLE", OME_INT16, NULL, &throttleEntry},
	{"YAW", OME_INT16, NULL, &yawEntry},
	{"AUX1", OME_INT16, NULL, &aux1Entry},
	{"AUX2", OME_INT16, NULL, &aux2Entry},
	{"AUX3", OME_INT16, NULL, &aux3Entry},
	{"AUX4", OME_INT16, NULL, &aux4Entry},
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};


OSD_Entry MenuLayout[]=
{
	{"---SCREEN LAYOUT---", OME_Label, NULL, NULL},
	{"ACTIVE ELEM.", OME_Submenu, OSD_ChangeScreen, &MenuLayoutActiv[0]},
	{"POSITION", OME_Submenu, OSD_ChangeScreen, &MenuLayoutPos[0]},
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};


OSD_UINT8_Struct RSSI_entry = {&OSD_cfg.rssi_alarm,5,90,5};
OSD_UINT16_Struct CAP_entry = {&OSD_cfg.cap_alarm, 50, 30000, 50};
OSD_UINT16_Struct FLY_TIME_entry = {&OSD_cfg.time_alarm, 1, 200, 1};

OSD_Entry MenuAlarms[]=
{
	{"------ ALARMS ------", OME_Label, NULL, NULL},
	{"RSSI", OME_UINT8, NULL, &RSSI_entry},
	{"MAIN BATT.", OME_UINT16, NULL, &CAP_entry},
	{"FLY TIME", OME_UINT16, NULL, &FLY_TIME_entry},
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};

OSD_Entry MenuLayoutPos[]=
{
	{"---POSITION---", OME_Label, NULL, NULL},
	{"RSSI", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_RSSI_VALUE]},
	{"MAIN BATTERY", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_MAIN_BATT_VOLTAGE]},
	{"UPTIME", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_ONTIME]},
	{"FLY TIME", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_FLYTIME]},
	{"FLY MODE", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_FLYMODE]},
	{"NAME", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_CRAFT_NAME]},
	{"THROTTLE POS", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_THROTTLE_POS]},
#ifdef VTX
	{"VTX CHAN", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_VTX_CHANNEL]},
#endif
	{"CURRENT (A)", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_CURRENT_DRAW]},
	{"USED MAH", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_MAH_DRAWN]},
#ifdef GPS
	{"GPS SPEED", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_GPS_SPEED]},
	{"GPS SATS.", OME_POS, OSD_EditElement, &masterConfig.osdProfile.item_pos[OSD_GPS_SATS]},
#endif
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};

OSD_Entry MenuLayoutActiv[]=
{
	{" --ACTIV ELEM.-- ", OME_Label, NULL, NULL},
	{"RSSI", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_RSSI_VALUE]},
	{"MAIN BATTERY", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_MAIN_BATT_VOLTAGE]},
	{"HORIZON", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_ARTIFICIAL_HORIZON]},
	{"HORIZON SIDEBARS", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_HORIZON_SIDEBARS]},
	{"UPTIME", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_ONTIME]},
	{"FLY TIME", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_FLYTIME]},
	{"FLY MODE", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_FLYMODE]},
	{"NAME", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_CRAFT_NAME]},
	{"THROTTLE POS", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_THROTTLE_POS]},
#ifdef VTX
	{"VTX CHAN", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_VTX_CHANNEL]},
#endif
	{"CURRENT (A)", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_CURRENT_DRAW]},
	{"USED MAH", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_MAH_DRAWN]},
#ifdef GPS
	{"GPS SPEED", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_GPS_SPEED]},
	{"GPS SATS.", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_GPS_SATS]},
#endif
	{"BACK", OME_Back, NULL, NULL},
	{NULL, OME_END, NULL, NULL}
};

void resetOsdConfig(osd_profile *osdProfile)
{

	osdProfile->item_pos[OSD_RSSI_VALUE] = OSD_POS(22,0) | VISIBLE_FLAG;
	osdProfile->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(12,0) | VISIBLE_FLAG;
	osdProfile->item_pos[OSD_ARTIFICIAL_HORIZON] = OSD_POS(8,6) | VISIBLE_FLAG;
	osdProfile->item_pos[OSD_HORIZON_SIDEBARS] = OSD_POS(8,6) | VISIBLE_FLAG;
	osdProfile->item_pos[OSD_ONTIME] = OSD_POS(22,11) | VISIBLE_FLAG;
	osdProfile->item_pos[OSD_FLYTIME] = OSD_POS(22,12) | VISIBLE_FLAG;
	osdProfile->item_pos[OSD_FLYMODE] = OSD_POS(12,11) | VISIBLE_FLAG;
	osdProfile->item_pos[OSD_CRAFT_NAME] = OSD_POS(12,12);
	osdProfile->item_pos[OSD_THROTTLE_POS] = OSD_POS(1,4);
	osdProfile->item_pos[OSD_VTX_CHANNEL] = OSD_POS(8,6);
	osdProfile->item_pos[OSD_CURRENT_DRAW] = OSD_POS(1,3);
	osdProfile->item_pos[OSD_MAH_DRAWN] = OSD_POS(15,3);
	osdProfile->item_pos[OSD_GPS_SPEED] = OSD_POS(2,2);
	osdProfile->item_pos[OSD_GPS_SATS] = OSD_POS(2,12);

	osdProfile->rssi_alarm = 20;
	osdProfile->cap_alarm = 2200;
	osdProfile->time_alarm = 10; //in minutes

	osdProfile->video_system = 0;
}



void osdInit(void)
{
	char x, string_buffer[30];

    arm_state = ARMING_FLAG(ARMED);

	max7456_init(masterConfig.osdProfile.video_system);

	max7456_clear_screen();

    // display logo and help
    x =  160;
    for (int i = 1; i < 5; i++) {
        for (int j = 3; j < 27; j++)
            if (x != 255)
				max7456_write_char(j,i,x++);
    }
    sprintf(string_buffer, "BF VERSION: %s", FC_VERSION_STRING);
    max7456_write(5, 6, string_buffer);
    max7456_write(7, 7, "MENU: THRT MID");
    max7456_write(13, 8, "YAW RIGHT");
    max7456_write(13, 9, "PITCH UP");
    max7456_refresh_all();

    RefreshTimeout = 4*REFRESH_1S;
}

extern uint16_t rssi; // FIXME dependency on mw.c
void OSD_RSSI(void)
{
	uint16_t val;
	char buff[5];

	if (!VISIBLE(OSD_cfg.item_pos[OSD_RSSI_VALUE]) || BLINK(OSD_cfg.item_pos[OSD_RSSI_VALUE]))
		return;

	val = rssi * 100 / 1024; //change range

	if (val >= 100)
		val = 99;

	buff[0] = SYM_RSSI;
	sprintf(buff+1, "%d", val);

	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_RSSI_VALUE]), OSD_Y(OSD_cfg.item_pos[OSD_RSSI_VALUE]), buff);
}

void OSD_VBAT(void)
{
	char buff[8];

	if (!VISIBLE(OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE]) || BLINK(OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE]))
		return;

	buff[0] = SYM_BATT_5;
	sprintf(buff+1, "%d.%1dV", vbat / 10, vbat % 10);

	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE]), OSD_Y(OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE]), buff);
}


void OSD_CURRENT(void)
{
	char buff[10];

	if (!VISIBLE(OSD_cfg.item_pos[OSD_CURRENT_DRAW]) || BLINK(OSD_cfg.item_pos[OSD_CURRENT_DRAW]))
		return;

	buff[0] = SYM_AMP;
    sprintf(buff+1, "%d.%02d", abs(amperage) / 100, abs(amperage) % 100);

	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_CURRENT_DRAW]), OSD_Y(OSD_cfg.item_pos[OSD_CURRENT_DRAW]), buff);
}



void OSD_CAPACITY(void)
{
	char buff[10];

	if (!VISIBLE(OSD_cfg.item_pos[OSD_MAH_DRAWN]) || BLINK(OSD_cfg.item_pos[OSD_MAH_DRAWN]))
		return;

    buff[0] = SYM_MAH;
	sprintf(buff+1, "%d", mAhDrawn);

	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_MAH_DRAWN]), OSD_Y(OSD_cfg.item_pos[OSD_MAH_DRAWN]), buff);
}

#define AHIPITCHMAX 200             // Specify maximum AHI pitch value displayed. Default 200 = 20.0 degrees
#define AHIROLLMAX  400             // Specify maximum AHI roll value displayed. Default 400 = 40.0 degrees
#define AHISIDEBARWIDTHPOSITION 7
#define AHISIDEBARHEIGHTPOSITION 3

// Write the artifical horizon to the screen buffer
void OSD_HORIZON() {
    uint16_t position = 194;
    int rollAngle = attitude.values.roll;
    int pitchAngle = attitude.values.pitch;

	if (max_screen_size == VIDEO_BUFFER_CHARS_PAL)
		position += 30;

    uint8_t *screenBuffer = max7456_get_screen_buffer();

    if (pitchAngle > AHIPITCHMAX)
        pitchAngle = AHIPITCHMAX;
    if (pitchAngle < -AHIPITCHMAX)
        pitchAngle = -AHIPITCHMAX;
    if (rollAngle > AHIROLLMAX)
        rollAngle = AHIROLLMAX;
    if (rollAngle < -AHIROLLMAX)
        rollAngle = -AHIROLLMAX;

    for (uint8_t X = 0; X <= 8; X++) {
        if (X == 4)
            X = 5;
        int Y = (rollAngle * (4 - X)) / 64;
        Y -= pitchAngle / 8;
        Y += 41;
        if (Y >= 0 && Y <= 81) {
            uint16_t pos = position - 7 + LINE * (Y / 9) + 3 - 4 * LINE + X;
            screenBuffer[pos] = (SYM_AH_BAR9_0 + (Y % 9));
        }
    }
    screenBuffer[position - 1] = (SYM_AH_CENTER_LINE);
    screenBuffer[position + 1] = (SYM_AH_CENTER_LINE_RIGHT);
    screenBuffer[position]     = (SYM_AH_CENTER);

    if (VISIBLE(OSD_cfg.item_pos[OSD_HORIZON_SIDEBARS])) {
        // Draw AH sides
        int8_t hudwidth  = AHISIDEBARWIDTHPOSITION;
        int8_t hudheight = AHISIDEBARHEIGHTPOSITION;
        for (int8_t X = -hudheight; X <= hudheight; X++) {
            screenBuffer[position - hudwidth + (X * LINE)] = (SYM_AH_DECORATION);
            screenBuffer[position + hudwidth + (X * LINE)] = (SYM_AH_DECORATION);
        }
        // AH level indicators
        screenBuffer[position-hudwidth+1] =  (SYM_AH_LEFT);
        screenBuffer[position+hudwidth-1] =  (SYM_AH_RIGHT);
    }
}

#ifdef GPS
void OSD_SATS(void)
{
	char buff[8];

	if (!VISIBLE(OSD_cfg.item_pos[OSD_GPS_SATS]) || BLINK(OSD_cfg.item_pos[OSD_GPS_SATS]))
		return;

	buff[0] = 0x1f;
	sprintf(buff+1, "%d", GPS_numSat);

	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_GPS_SATS]), OSD_Y(OSD_cfg.item_pos[OSD_GPS_SATS]), buff);
}

void OSD_SPEED(void)
{
	char buff[8];

	if (!VISIBLE(OSD_cfg.item_pos[OSD_GPS_SPEED]) || BLINK(OSD_cfg.item_pos[OSD_GPS_SPEED]))
		return;

	sprintf(buff, "%d", GPS_speed * 36 / 1000);

	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_GPS_SPEED]), OSD_Y(OSD_cfg.item_pos[OSD_GPS_SPEED]), buff);
}

#endif

void OSD_ON_TIME(void)
{
	char buff[8];
	uint32_t seconds;

	if (!VISIBLE(OSD_cfg.item_pos[OSD_ONTIME]) || BLINK(OSD_cfg.item_pos[OSD_ONTIME]))
		return;

	buff[0] = SYM_ON_M;
    seconds = micros() / 1000000;
	sprintf(buff+1, "%02d:%02d", seconds / 60, seconds % 60);

	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_ONTIME]), OSD_Y(OSD_cfg.item_pos[OSD_ONTIME]), buff);
}

void OSD_FLY_TIME(void)
{
	char buff[8];

	if (!VISIBLE(OSD_cfg.item_pos[OSD_FLYTIME]) || BLINK(OSD_cfg.item_pos[OSD_FLYTIME]))
		return;

	buff[0] = SYM_FLY_M;
	sprintf(buff+1, "%02d:%02d", fly_time / 60, fly_time % 60);

	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_FLYTIME]), OSD_Y(OSD_cfg.item_pos[OSD_FLYTIME]), buff);
}

void OSD_MODE(void)
{
	char *p = "ACRO";

	if (!VISIBLE(OSD_cfg.item_pos[OSD_FLYMODE]) || BLINK(OSD_cfg.item_pos[OSD_FLYMODE]))
		return;

	if (isAirmodeActive())
		p = "AIR";

	if (FLIGHT_MODE(FAILSAFE_MODE))
		p = "!FS";
	else if (FLIGHT_MODE(ANGLE_MODE))
		p = "STAB";
	else if (FLIGHT_MODE(HORIZON_MODE))
		p = "HOR";

	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_FLYMODE]), OSD_Y(OSD_cfg.item_pos[OSD_FLYMODE]), p);
}

void OSD_NAME(void)
{
	char line[32];
	if (!VISIBLE(OSD_cfg.item_pos[OSD_CRAFT_NAME]) || BLINK(OSD_cfg.item_pos[OSD_CRAFT_NAME]))
		return;

	if (strlen(masterConfig.name) == 0)
		strcpy(line,"CRAFT_NAME");
	else
	for (uint8_t i = 0; i < MAX_NAME_LENGTH; i++) {
		line[i] = toupper((unsigned char)masterConfig.name[i]);
		if (masterConfig.name[i] == 0)
			break;
    }

	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_CRAFT_NAME]), OSD_Y(OSD_cfg.item_pos[OSD_CRAFT_NAME]), line);
}

void OSD_THROTTLE(void)
{
	char line[32];
	if (!VISIBLE(OSD_cfg.item_pos[OSD_THROTTLE_POS]) || BLINK(OSD_cfg.item_pos[OSD_THROTTLE_POS]))
		return;

	line[0] = SYM_THR;
	line[1] = SYM_THR1;
	sprintf(line+2, "%d", (constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN));

	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_THROTTLE_POS]), OSD_Y(OSD_cfg.item_pos[OSD_THROTTLE_POS]), line);
}

#ifdef VTX

void OSD_VTX_CHAN(void)
{
	char line[32];

	if (!VISIBLE(OSD_cfg.item_pos[OSD_VTX_CHANNEL]) || BLINK(OSD_cfg.item_pos[OSD_VTX_CHANNEL]))
		return;

    sprintf(line,  "CH:%d", current_vtx_channel % CHANNELS_PER_BAND + 1);
    max7456_write(OSD_X(OSD_cfg.item_pos[OSD_VTX_CHANNEL]), OSD_Y(OSD_cfg.item_pos[OSD_VTX_CHANNEL]), line);
}
#endif

void OSD_UpdateAlarms(void)
{
	uint16_t rval = rssi * 100 / 1024; //zmiana zakresu

	if (rval < OSD_cfg.rssi_alarm)
		OSD_cfg.item_pos[OSD_RSSI_VALUE] |= BLINK_FLAG;
	else
		OSD_cfg.item_pos[OSD_RSSI_VALUE] &= ~BLINK_FLAG;

	if (vbat <= (batteryWarningVoltage - 1))
		OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE] |= BLINK_FLAG;
	else
		OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE] &= ~BLINK_FLAG;

	if (STATE(GPS_FIX) == 0)
		OSD_cfg.item_pos[OSD_GPS_SATS] |= BLINK_FLAG;
	else
		OSD_cfg.item_pos[OSD_GPS_SATS] &= ~BLINK_FLAG;

	if (fly_time / 60 >= OSD_cfg.time_alarm && ARMING_FLAG(ARMED))
		OSD_cfg.item_pos[OSD_FLYTIME] |= BLINK_FLAG;
	else
		OSD_cfg.item_pos[OSD_FLYTIME] &= ~BLINK_FLAG;

	if (mAhDrawn >= OSD_cfg.cap_alarm)
		OSD_cfg.item_pos[OSD_MAH_DRAWN] |= BLINK_FLAG;
	else
		OSD_cfg.item_pos[OSD_MAH_DRAWN] &= ~BLINK_FLAG;
}

void OSD_TurnOffAlarms(void)
{
	OSD_cfg.item_pos[OSD_RSSI_VALUE] &= ~BLINK_FLAG;
	OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE] &= ~BLINK_FLAG;
	OSD_cfg.item_pos[OSD_GPS_SATS] &= ~BLINK_FLAG;
	OSD_cfg.item_pos[OSD_FLYTIME] &= ~BLINK_FLAG;
	OSD_cfg.item_pos[OSD_MAH_DRAWN] &= ~BLINK_FLAG;
}

void OSD_HELP(void)
{
	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]), OSD_Y(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]),   "---  HELP --- ");
	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]), OSD_Y(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON])+1, "USE ROLL/PITCH");
	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]), OSD_Y(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON])+2, "TO MOVE ELEM. ");
	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]), OSD_Y(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON])+3, "              ");
	max7456_write(OSD_X(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]), OSD_Y(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON])+4, "YAW - EXIT    ");
}

void OSD_DrawElements(void)
{
	max7456_clear_screen();

	if (CurrentElement)
		OSD_HELP();
	else if (sensors(SENSOR_ACC) || OSD_MENU)
		OSD_HORIZON();

	OSD_VBAT();
	OSD_RSSI();
	OSD_FLY_TIME();
	OSD_ON_TIME();
	OSD_MODE();
	OSD_THROTTLE();
#ifdef VTX
    OSD_VTX_CHAN();
#endif
	OSD_CURRENT();
	OSD_CAPACITY();
	OSD_NAME();

#ifdef GPS
	if (sensors(SENSOR_GPS) || OSD_MENU)
	{
		OSD_SATS();
		OSD_SPEED();
	}
#endif
}

uint8_t OSDHandleKey(uint8_t key)
{
	uint8_t res = BUTTON_TIME;
	OSD_Entry *p;

	if (!CurrentMenu)
		return res;

	if (key == KEY_ESC)
	{
		OSD_MenuBack();
		return BUTTON_PAUSE;
	}

	if (key == KEY_DOWN)
	{
		if (CurrentMenuPos < CurrentMaxIdx)
			CurrentMenuPos++;
		else
		{
			if (NextPage) //we have more pages
			{
				max7456_clear_screen();
				p = NextPage;
				NextPage = CurrentMenu;
				CurrentMenu = (OSD_Entry*)p;
				CurrentMenuPos = 0;
				OSD_UpdateMaxRows();
			}
			CurrentMenuPos = 0;
		}
	}

	if (key == KEY_UP)
	{
		CurrentMenuPos--;

		if ((CurrentMenu+CurrentMenuPos)->type == OME_Label && CurrentMenuPos > 0)
			CurrentMenuPos--;

		if (CurrentMenuPos == -1 || (CurrentMenu+CurrentMenuPos)->type == OME_Label)
		{
			if (NextPage)
			{
				max7456_clear_screen();
				p = NextPage;
				NextPage = CurrentMenu;
				CurrentMenu = (OSD_Entry*)p;
				CurrentMenuPos = 0;
				OSD_UpdateMaxRows();
			}
			CurrentMenuPos = CurrentMaxIdx;
		}
	}

	if (key == KEY_DOWN || key == KEY_UP)
		return res;

	p = CurrentMenu+CurrentMenuPos;

	switch (p->type)
	{
	case OME_POS:
		if (key == KEY_RIGHT)
		{
			uint32_t address = (uint32_t)p->data;
			uint16_t *val;

			val = (uint16_t*)address;
			if ( !(*val & VISIBLE_FLAG) ) //no submenu for hidden elements
				break;
		}
	case OME_Submenu:
	case OME_OSD_Exit:
		if (p->func && key == KEY_RIGHT)
		{
			p->func(p->data);
			res = BUTTON_PAUSE;
		}
		break;
	case OME_Back:
		OSD_MenuBack();
		res = BUTTON_PAUSE;
		break;
	case OME_Bool:
		if (p->data)
		{
			uint8_t *val = p->data;
			if (key == KEY_RIGHT)
				*val = 1;
			else
				*val = 0;
		}
		break;
	case OME_VISIBLE:
		if (p->data)
		{
			uint32_t address = (uint32_t)p->data;
			uint16_t *val;

			val = (uint16_t*)address;

			if (key == KEY_RIGHT)
				*val |= VISIBLE_FLAG;
			else
				*val %= ~VISIBLE_FLAG;
		}
		break;
	case OME_UINT8:
	case OME_FLOAT:
		if (p->data)
		{
			OSD_UINT8_Struct *ptr = p->data;
			if (key == KEY_RIGHT)
			{
				if (*ptr->val < ptr->max)
					*ptr->val+=ptr->step;
			}
			else
			{
				if (*ptr->val > ptr->min)
					*ptr->val-=ptr->step;
			}
		}
		break;
	case OME_TAB:
		if (p->type == OME_TAB)
		{
			OSD_TAB_Struct *ptr = p->data;

			if (key == KEY_RIGHT)
			{
				if (*ptr->val < ptr->max)
					*ptr->val+=1;
			}
			else
			{
				if (*ptr->val > 0)
					*ptr->val-=1;
			}
			if (p->func)
				p->func(p->data);
		}
		break;
	case OME_INT8:
		if (p->data)
		{
			OSD_INT8_Struct *ptr = p->data;
			if (key == KEY_RIGHT)
			{
				if (*ptr->val < ptr->max)
					*ptr->val+=ptr->step;
			}
			else
			{
				if (*ptr->val > ptr->min)
					*ptr->val-=ptr->step;
			}
		}
		break;
	case OME_UINT16:
		if (p->data)
		{
			OSD_UINT16_Struct *ptr = p->data;
			if (key == KEY_RIGHT)
			{
				if (*ptr->val < ptr->max)
					*ptr->val+=ptr->step;
			}
			else
			{
				if (*ptr->val > ptr->min)
					*ptr->val-=ptr->step;
			}
		}
		break;
	case OME_INT16:
		if (p->data)
		{
			OSD_INT16_Struct *ptr = p->data;
			if (key == KEY_RIGHT)
			{
				if (*ptr->val < ptr->max)
					*ptr->val+=ptr->step;
			}
			else
			{
				if (*ptr->val > ptr->min)
					*ptr->val-=ptr->step;
			}
		}
		break;
	case OME_Label:
	case OME_END:
		break;
	}
	return res;
}

void OSD_UpdateMaxRows(void)
{
	OSD_Entry *ptr;

	CurrentMaxIdx = 0;
	for (ptr=CurrentMenu; ptr->type != OME_END; ptr++)
		CurrentMaxIdx++;

	if (CurrentMaxIdx > MAX_MENU_ITEMS)
		CurrentMaxIdx = MAX_MENU_ITEMS;

	CurrentMaxIdx--;
}

void OSD_MenuBack(void)
{
	uint8_t i;

	//becasue pids and rates meybe stored in profiles we need some thicks to manipulate it
	//hack to save pid profile
	if (CurrentMenu == &MenuPID[0])
	{
		for (i = 0; i < 3; i++) {
			curr_profile.pidProfile.P8[i] = temp_pids[i][0];
			curr_profile.pidProfile.I8[i] = temp_pids[i][1];
			curr_profile.pidProfile.D8[i] = temp_pids[i][2];
		}

		curr_profile.pidProfile.P8[PIDLEVEL] = temp_pids[3][0];
		curr_profile.pidProfile.I8[PIDLEVEL] = temp_pids[3][1];
		curr_profile.pidProfile.D8[PIDLEVEL] = temp_pids[3][2];
	}

	//hack - save rate config for current profile
	if (CurrentMenu == &MenuRateExpo[0])
		memcpy(&masterConfig.profile[masterConfig.current_profile_index].controlRateProfile[masterConfig.profile[masterConfig.current_profile_index].activeRateProfile], &rateProfile, sizeof(controlRateConfig_t));

	if (MenuStackIdx)
	{
		max7456_clear_screen();
		MenuStackIdx--;
		NextPage = NULL;
		CurrentMenu = MenuStack[MenuStackIdx];
		CurrentMenuPos = MenuStackPos[MenuStackIdx];

		OSD_UpdateMaxRows();
	}
	else
		OSD_OpenMenu();
}

void simple_ftoa(int32_t value, char *floatString)
{
		uint8_t k;
		//np. 3450

    itoa(100000+value, floatString, 10);   // Create string from abs of integer value

		// 103450

		floatString[0] = floatString[1];
		floatString[1] = floatString[2];
		floatString[2] = '.';

		// 03.450
		//usuwam koncowe zera i kropke
		for (k = 5;k > 1; k--)
			if (floatString[k] == '0' || floatString[k] == '.')
				floatString[k]=0;
			else
				break;

			//oraz zero wiodonce
		if (floatString[0] == '0')
			floatString[0] = ' ';
}

void OSD_DrawMenu(void)
{
	uint8_t i = 0;
	OSD_Entry *p;
	char buff[10];
	uint8_t top = (OSD_ROWS - CurrentMaxIdx)/2 - 1;
	if (!CurrentMenu)
		return;

	if ((CurrentMenu+CurrentMenuPos)->type == OME_Label) //skip label
		CurrentMenuPos++;

	for (p=CurrentMenu; p->type != OME_END; p++)
	{
		if (CurrentMenuPos == i)
			max7456_write(LEFT_MENU_COLUMN, i+top, " >");
		else
			max7456_write(LEFT_MENU_COLUMN, i+top, "  ");
		max7456_write(LEFT_MENU_COLUMN+2, i+top, p->text);

		switch (p->type)
		{
		case OME_POS:
		{
			uint32_t address = (uint32_t)p->data;
			uint16_t *val;

			val = (uint16_t*)address;
			if ( !(*val & VISIBLE_FLAG) )
				break;
		}
		case OME_Submenu:
			max7456_write(RIGHT_MENU_COLUMN, i+top, ">");
			break;
		case OME_Bool:
			if (p->data)
			{
				if (*((uint8_t*)(p->data)))
					max7456_write(RIGHT_MENU_COLUMN, i+top, "YES");
				else
					max7456_write(RIGHT_MENU_COLUMN, i+top, "NO ");
			}
			break;
		case OME_TAB:
			{
				OSD_TAB_Struct *ptr = p->data;
				max7456_write(RIGHT_MENU_COLUMN-5, i+top, (char*)ptr->names[*ptr->val]);
			}
			break;
		case OME_VISIBLE:
			if (p->data)
			{
				uint32_t address = (uint32_t)p->data;
				uint16_t *val;

				val = (uint16_t*)address;

				if (VISIBLE(*val))
					max7456_write(RIGHT_MENU_COLUMN, i+top, "YES");
				else
					max7456_write(RIGHT_MENU_COLUMN, i+top, "NO ");
			}
			break;
		case OME_UINT8:
			if (p->data)
			{
				OSD_UINT8_Struct *ptr = p->data;
				itoa( *ptr->val, buff, 10);
				max7456_write(RIGHT_MENU_COLUMN, i+top, "     ");
				max7456_write(RIGHT_MENU_COLUMN, i+top, buff);
			}
			break;
		case OME_INT8:
			if (p->data)
			{
				OSD_INT8_Struct *ptr = p->data;
				itoa( *ptr->val, buff, 10);
				max7456_write(RIGHT_MENU_COLUMN, i+top, "     ");
				max7456_write(RIGHT_MENU_COLUMN, i+top, buff);
			}
			break;
		case OME_UINT16:
			if (p->data)
			{
				OSD_UINT16_Struct *ptr = p->data;
				itoa( *ptr->val, buff, 10);
				max7456_write(RIGHT_MENU_COLUMN, i+top, "     ");
				max7456_write(RIGHT_MENU_COLUMN, i+top, buff);
			}
			break;
		case OME_INT16:
			if (p->data)
			{
				OSD_UINT16_Struct *ptr = p->data;
				itoa( *ptr->val, buff, 10);
				max7456_write(RIGHT_MENU_COLUMN, i+top, "     ");
				max7456_write(RIGHT_MENU_COLUMN, i+top, buff);
			}
			break;
		case OME_FLOAT:
			if (p->data)
			{
				OSD_FLOAT_Struct *ptr = p->data;
				simple_ftoa(*ptr->val * ptr->multipler, buff);
				max7456_write(RIGHT_MENU_COLUMN-1, i+top, "      ");
				max7456_write(RIGHT_MENU_COLUMN-1, i+top, buff);
			}
			break;
		case OME_OSD_Exit:
		case OME_Label:
		case OME_END:
		case OME_Back:
			break;
		}
		i++;

		if (i == MAX_MENU_ITEMS) //max per page
		{
			NextPage=CurrentMenu+i;
			if (NextPage->type == OME_END)
				NextPage = NULL;
			break;
		}
	}
}

void OSD_ResetStats(void)
{
	stats.max_current = 0;
	stats.max_speed = 0;
	stats.min_voltage = 500;
	stats.max_current = 0;
	stats.min_rssi = 99;
}

void OSD_UpdateStats(void)
{
	int16_t value;

	value = GPS_speed * 36 / 1000;
	if (stats.max_speed < value)
		stats.max_speed = value;

	if (stats.min_voltage > vbat)
		stats.min_voltage = vbat;

	value = amperage / 100;
	if (stats.max_current < value)
		stats.max_current = value;

	if (stats.min_rssi > stat_RSSI)
		stats.min_rssi = stat_RSSI;
}

void OSD_Stats(void)
{
	uint8_t top = 2;
	char buff[10];

	max7456_clear_screen();
	max7456_write(2, top++, "  --- STATS ---");

	if (STATE(GPS_FIX))
	{
		max7456_write(2, top, "MAX SPEED        :");
		itoa(stats.max_speed, buff, 10);
		max7456_write(22, top++, buff);
	}

	max7456_write(2, top, "MIN BATTERY      :");
	sprintf(buff, "%d.%1dV", stats.min_voltage / 10, stats.min_voltage % 10);
	max7456_write(22, top++, buff);

	max7456_write(2, top, "MIN RSSI         :");
	itoa(stats.min_rssi, buff, 10);
	strcat(buff,"%");
	max7456_write(22, top++, buff);

	if (feature(FEATURE_CURRENT_METER))
	{
		max7456_write(2, top, "MAX CURRENT     :");
		itoa(stats.max_current/10, buff, 10);
		strcat(buff, "A");
		max7456_write(22, top++, buff);

		max7456_write(2, top, "USED MAH          :");
		itoa(mAhDrawn, buff, 10);
		strcat(buff,"\x07");
		max7456_write(22, top++, buff);
	}
	RefreshTimeout = 60*REFRESH_1S;
}

//called when motors armed
void OSD_ArmMotors(void)
{
	max7456_clear_screen();
	max7456_write(7, 5, " ARMED ");
	RefreshTimeout = REFRESH_1S/2;
	OSD_ResetStats();
}

//simple function used to display message with defined timeout 10 = 1sec
void OSD_Message(char *line1, char *line2, uint8_t timeout)
{
	max7456_clear_screen();
	if (line1)
		max7456_write(1, 6, line1);
	if (line2)
		max7456_write(1, 7, line2);
	RefreshTimeout = timeout * REFRESH_1S / 10;
}

void updateOsd(void)
{
	static uint32_t counter;
#ifdef MAX7456_DMA_CHANNEL_TX
	//don't touch buffers if DMA transaction is in progress
	if (max7456_dma_in_progres())
		return;
#endif
    //redraw values in buffer
    if (counter++ % 5 == 0)
        OSD_Update(0);
    else //rest of time redraw screen 10 chars per idle to don't lock the main idle
		max7456_draw_screen();
	//do not allow ARM if we are in menu
	if (OSD_MENU)
		DISABLE_ARMING_FLAG(OK_TO_ARM);
}

void OSD_Update(uint8_t guiKey)
{
	static uint8_t rcDelay = BUTTON_TIME;
	static uint8_t last_sec = 0;
	uint8_t	key=0, sec;

	//detect enter to menu
	if (IS_MID(THROTTLE) && IS_HI(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED))
		OSD_OpenMenu();

    //detect arm/disarm
    if (arm_state != ARMING_FLAG(ARMED))
    {
        if (ARMING_FLAG(ARMED))
            OSD_ArmMotors(); //reset statistic etc
        else
            OSD_Stats(); //schow statistic

        arm_state = ARMING_FLAG(ARMED);
    }

	OSD_UpdateStats();

	sec = millis()/1000;

	if (ARMING_FLAG(ARMED) && sec != last_sec)
	{
		fly_time++;
		last_sec = sec;
	}

	if (RefreshTimeout)
	{
		if (IS_HI(THROTTLE) || IS_HI(PITCH)) //hide statistics
			RefreshTimeout = 1;
		RefreshTimeout--;
		if (!RefreshTimeout)
			max7456_clear_screen();
		return;
	}

	BLINK_STATE	= (millis()/200) % 2;

	if (OSD_MENU)
	{
		if (rcDelay)
		{
			rcDelay--;
		}
		else if (IS_HI(PITCH))
		{
			key = KEY_UP;
			rcDelay = BUTTON_TIME;
		}
		else if (IS_LO(PITCH))
		{
			key = KEY_DOWN;
			rcDelay = BUTTON_TIME;
		}
		else if (IS_LO(ROLL))
		{
			key = KEY_LEFT;
			rcDelay = BUTTON_TIME;
		}
		else if (IS_HI(ROLL))
		{
			key = KEY_RIGHT;
			rcDelay = BUTTON_TIME;
		}
		else if ((IS_HI(YAW) || IS_LO(YAW)) && CurrentMenu != MenuRc) //this menu is used to check transmitter signals so can exit using YAW
		{
			key = KEY_ESC;
			rcDelay = BUTTON_TIME;
		}

		if (guiKey)
			key = guiKey;

		if (key && !CurrentElement)
		{
			rcDelay = OSDHandleKey(key);
			return;
		}
		if (CurrentElement) //edit position of element
		{
			if (key)
			{
				if (key == KEY_ESC)
				{
					//exit
					OSD_MenuBack();
					rcDelay = BUTTON_PAUSE;
					*CurrentElement &= ~BLINK_FLAG;
					CurrentElement = NULL;
					return;
				}
				else
				{
					uint8_t x,y;
					x = OSD_X(*CurrentElement);
					y = OSD_Y(*CurrentElement);
					switch (key)
					{
					case KEY_UP:
						y--;
						break;
					case KEY_DOWN:
						y++;
						break;
					case KEY_RIGHT:
						x++;
						break;
					case KEY_LEFT:
						x--;
						break;
					}

					*CurrentElement &= 0xFC00;
					*CurrentElement |=OSD_POS(x,y);
					max7456_clear_screen();
				}
			}
			OSD_DrawElements();
		}
		else
			OSD_DrawMenu();
	}
	else
	{
		OSD_UpdateAlarms();
		OSD_DrawElements();
	}
}

void OSD_ChangeScreen(void * ptr)
{
	uint8_t i;
	if (ptr)
	{
		max7456_clear_screen();
		//hack - save profile to temp
		if (ptr == &MenuPID[0])
		{
			for (i = 0; i < 3; i++) {
				temp_pids[i][0] = curr_profile.pidProfile.P8[i];
				temp_pids[i][1] = curr_profile.pidProfile.I8[i];
				temp_pids[i][2] = curr_profile.pidProfile.D8[i];
			}
			temp_pids[3][0] = curr_profile.pidProfile.P8[PIDLEVEL];
			temp_pids[3][1] = curr_profile.pidProfile.I8[PIDLEVEL];
			temp_pids[3][2] = curr_profile.pidProfile.D8[PIDLEVEL];
		}

		if (ptr == &MenuRateExpo[0])
			memcpy(&rateProfile, &masterConfig.profile[masterConfig.current_profile_index].controlRateProfile[masterConfig.profile[masterConfig.current_profile_index].activeRateProfile], sizeof(controlRateConfig_t));

		MenuStack[MenuStackIdx] = CurrentMenu;
		MenuStackPos[MenuStackIdx] = CurrentMenuPos;
		MenuStackIdx++;

		CurrentMenu = (OSD_Entry*)ptr;
		CurrentMenuPos = 0;
		OSD_UpdateMaxRows();
	}
}

#ifdef USE_FLASHFS

void OSD_EraseFlash(void *ptr)
{
    UNUSED(ptr);

	max7456_clear_screen();
	max7456_write(5, 3, "ERASING FLASH...");
	max7456_refresh_all();

	flashfsEraseCompletely();
	while (!flashfsIsReady()) {
        delay(100);
    }
	max7456_clear_screen();
	max7456_refresh_all();
}

#endif // USE_FLASHFS

void OSD_EditElement(void *ptr)
{
	uint32_t address = (uint32_t)ptr;
	//zsave position on menu stack
	MenuStack[MenuStackIdx] = CurrentMenu;
	MenuStackPos[MenuStackIdx] = CurrentMenuPos;
	MenuStackIdx++;

	CurrentElement = (uint16_t*)address;

	*CurrentElement |= BLINK_FLAG;
	max7456_clear_screen();
}

void OSD_Exit(void * ptr)
{
	max7456_clear_screen();
	max7456_write(5, 3, "RESTARTING IMU...");
	max7456_refresh_all();
	stopPwmAllMotors();
	if (ptr)
	{
		//save local variables to configuration
		if (blackbox_feature)
			featureSet(FEATURE_BLACKBOX);

		if (ledstrip_feature)
			featureSet(FEATURE_LED_STRIP);
#if defined(VTX) || defined(USE_RTC6705)
		if (vtx_feature)
			featureSet(FEATURE_VTX);
#endif
#ifdef VTX
        masterConfig.vtx_band = vtx_band;
        masterConfig.vtx_channel = vtx_channel-1;
#endif

#ifdef USE_RTC6705
        masterConfig.vtx_channel = vtx_band*8 + vtx_channel-1;
#endif // USE_RTC6705

		saveConfigAndNotify();
	}
	systemReset();
}

void OSD_HandleGui(uint8_t cmd)
{
	switch (cmd)
	{
		case KEY_ENTER:
			OSD_OpenMenu();
		break;
		default:
			OSD_Update(cmd);
			break;
	}
}

void OSD_OpenMenu(void)
{
	if (OSD_MENU)
		return;

	if (feature(FEATURE_LED_STRIP))
		ledstrip_feature = 1;

	if (feature(FEATURE_BLACKBOX))
		blackbox_feature = 1;
#if defined(VTX) || defined(USE_RTC6705)
	if (feature(FEATURE_VTX))
		vtx_feature = 1;
#endif

#ifdef VTX
        vtx_band = masterConfig.vtx_band;
        vtx_channel = masterConfig.vtx_channel +1;
#endif

#ifdef USE_RTC6705
		vtx_band = masterConfig.vtx_channel / 8;
		vtx_channel = masterConfig.vtx_channel%8 + 1;
#endif

	OSD_ROWS = max7456_get_rows_count();
	OSD_MENU = 1;
	RefreshTimeout = 0;
	max7456_clear_screen();
	CurrentMenu = &MainMenu[0];
	OSD_TurnOffAlarms();
	OSD_ChangeScreen(CurrentMenu);
#ifdef LED_STRIP
	getLedColor();
#endif // LED_STRIP
}


