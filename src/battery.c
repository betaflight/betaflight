#include "stdbool.h"
#include "stdint.h"

#include "drivers/adc_common.h"
#include "drivers/system_common.h"

#include "battery.h"

// Battery monitoring stuff
uint8_t batteryCellCount = 3;       // cell count
uint16_t batteryWarningVoltage;     // annoying buzzer after this one, battery ready to be dead

uint8_t vbat = 0;                   // battery voltage in 0.1V steps

int32_t amperage = 0;               // amperage read by current sensor in centiampere (1/100th A)
uint32_t mAhdrawn = 0;              // milliampere hours drawn from the battery since start

static batteryConfig_t *batteryConfig;

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return (((src) * 3.3f) / 0xFFF) * batteryConfig->vbatscale;
}


void updateBatteryVoltage(void)
{
    static uint16_t vbatSamples[8];
    static uint8_t currentSampleIndex = 0;
    uint8_t index;
    uint16_t vbatSampleTotal = 0;

    // store the battery voltage with some other recent battery voltage readings
    vbatSamples[(currentSampleIndex++) % 8] = adcGetChannel(ADC_BATTERY);

    // calculate vbat based on the average of recent readings
    for (index = 0; index < 8; index++) {
        vbatSampleTotal += vbatSamples[index];
    }
    vbat = batteryAdcToVoltage(vbatSampleTotal / 8);
}

bool shouldSoundBatteryAlarm(void)
{
    return !((vbat > batteryWarningVoltage) || (vbat < batteryConfig->vbatmincellvoltage));
}

void batteryInit(batteryConfig_t *initialBatteryConfig)
{
    batteryConfig = initialBatteryConfig;

    uint32_t i;
    uint32_t voltage = 0;

    // average up some voltage readings
    for (i = 0; i < 32; i++) {
        voltage += adcGetChannel(ADC_BATTERY);
        delay(10);
    }

    voltage = batteryAdcToVoltage((uint16_t)(voltage / 32));

    // autodetect cell count, going from 2S..8S
    for (i = 1; i < 8; i++) {
        if (voltage < i * batteryConfig->vbatmaxcellvoltage)
            break;
    }
    batteryCellCount = i;
    batteryWarningVoltage = i * batteryConfig->vbatmincellvoltage; // 3.3V per cell minimum, configurable in CLI
}

#define ADCVREF 33L
int32_t currentSensorToCentiamps(uint16_t src)
{
    int32_t millivolts;

    millivolts = ((uint32_t)src * ADCVREF * 100) / 4095;
    millivolts -= batteryConfig->currentMeterOffset;

    return (millivolts * 1000) / (int32_t)batteryConfig->currentMeterScale; // current in 0.01A steps
}

void updateCurrentMeter(uint32_t lastUpdateAt)
{
    static int32_t amperageRaw = 0;
    static uint32_t mAhdrawnRaw = 0;

	amperageRaw -= amperageRaw / 8;
	amperageRaw += adcGetChannel(ADC_CURRENT);
	amperage = currentSensorToCentiamps(amperageRaw / 8);

	mAhdrawnRaw += (amperage * lastUpdateAt) / 1000; // will overflow at ~11000mAh
	mAhdrawn = mAhdrawnRaw / (3600 * 100);
}
