#pragma once

#include <stdint.h>

typedef enum {
    OSD_RSSI_VALUE,
    OSD_MAIN_BATT_VOLTAGE,
    OSD_ARTIFICIAL_HORIZON,
    OSD_HORIZON_SIDEBARS,
    OSD_ONTIME,
    OSD_FLYTIME,
    OSD_FLYMODE,
    OSD_CRAFT_NAME,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
    OSD_GPS_SPEED,
    OSD_GPS_SATS,
    OSD_MAX_ITEMS, // MUST BE LAST
} osd_items_t;

typedef struct {
	uint16_t item_pos[OSD_MAX_ITEMS];
	//alarms
	uint8_t rssi_alarm;
	uint16_t cap_alarm;
	uint16_t time_alarm;
	uint8_t video_system;
} osd_profile;

typedef struct {
	int16_t max_height;
	int16_t max_speed;
	int16_t min_voltage;	// /10
	int16_t max_current;  // /10
	int16_t min_rssi;
} tStatistic;

void osdInit(void);
void OSD_Update(uint8_t guiKey);
void OSD_OpenMenu(void);
void OSD_HandleGui(uint8_t cmd);
void OSD_ResetSettings(void);
void OSD_Message(char *line1, char *line2, uint8_t timeout);
void resetOsdConfig(osd_profile *osdProfile);
void updateOsd(void);
