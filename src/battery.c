#include "stdbool.h"
#include "stdint.h"

#include "drivers/adc_common.h"
#include "drivers/system_common.h"

#include "battery.h"

// Battery monitoring stuff
uint8_t batteryCellCount = 3;       // cell count
uint16_t batteryWarningVoltage;     // annoying buzzer after this one, battery ready to be dead

uint8_t vbat;                   // battery voltage in 0.1V steps

static batteryConfig_t *batteryConfig;

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 4095 = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return (((src) * 3.3f) / 4095) * batteryConfig->vbatscale;
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

    // autodetect cell count, going from 2S..6S
    for (i = 1; i < 6; i++) {
        if (voltage < i * batteryConfig->vbatmaxcellvoltage)
            break;
    }
    batteryCellCount = i;
    batteryWarningVoltage = i * batteryConfig->vbatmincellvoltage; // 3.3V per cell minimum, configurable in CLI
}

