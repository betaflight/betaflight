#pragma once

typedef struct batteryConfig_s {
    uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
    uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery out alarms, in 0.1V units, default is 33 (3.3V)

    uint16_t currentMeterScale;             // scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
    uint16_t currentMeterOffset;            // offset of the current sensor in millivolt steps

    // FIXME this doesn't belong in here since it's a concern of MSP, not of the battery code.
    uint8_t multiwiiCurrentMeterOutput;     // if set to 1 output the amperage in milliamp steps instead of 0.01A steps via msp
} batteryConfig_t;

extern uint8_t vbat;
extern uint8_t batteryCellCount;
extern uint16_t batteryWarningVoltage;
extern int32_t amperage;
extern uint32_t mAhdrawn;

uint16_t batteryAdcToVoltage(uint16_t src);
bool shouldSoundBatteryAlarm(void);
void updateBatteryVoltage(void);
void batteryInit(batteryConfig_t *initialBatteryConfig);

void updateCurrentMeter(uint32_t lastUpdateAt);
int32_t currentMeterToCentiamps(uint16_t src);
