#pragma once

typedef struct batteryConfig_s {
    uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
    uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery out alarms, in 0.1V units, default is 33 (3.3V)
} batteryConfig_t;

extern uint16_t batteryWarningVoltage;

uint16_t batteryAdcToVoltage(uint16_t src);
bool shouldSoundBatteryAlarm(void);
void updateBatteryVoltage(void);
void batteryInit(batteryConfig_t *initialBatteryConfig);

