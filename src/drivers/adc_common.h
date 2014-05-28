#pragma once

typedef enum {
    ADC_BATTERY = 0,
    ADC_RSSI = 1,
    ADC_EXTERNAL1 = 2,
    ADC_CHANNEL_MAX = ADC_EXTERNAL1
} AdcChannel;

#define ADC_CHANNEL_COUNT (ADC_CHANNEL_MAX + 1)

typedef struct adc_config_t {
    uint8_t adcChannel;         // ADC1_INxx channel number
    uint8_t dmaIndex;           // index into DMA buffer in case of sparse channels
    bool enabled;
    uint8_t sampleTime;
} adc_config_t;

typedef struct drv_adc_config_t {
    bool enableRSSI;
} drv_adc_config_t;

void adcInit(drv_adc_config_t *init);
uint16_t adcGetChannel(uint8_t channel);
