#ifdef FY90Q
#include "board.h"

#define ADC_CHANNELS 9

volatile uint16_t adcData[ADC_CHANNELS] = { 0, };
extern uint16_t acc_1G;

static void adcAccRead(int16_t *accelData);
static void adcAccAlign(int16_t *accelData);
static void adcGyroRead(int16_t *gyroData);
static void adcGyroAlign(int16_t *gyroData);
static void adcDummyInit(void);

void adcSensorInit(sensor_t *acc, sensor_t *gyro)
{
    acc->init = adcDummyInit;
    acc->read = adcAccRead;
    acc->align = adcAccAlign;

    gyro->init = adcDummyInit;
    gyro->read = adcGyroRead;
    gyro->align = adcGyroAlign;
    gyro->scale = 1.0f;

    acc_1G = 376;
}

void adcCalibrateADC(ADC_TypeDef *ADCx, int n)
{
    while (n > 0) {
        delay(5);
        // Enable ADC reset calibration register
        ADC_ResetCalibration(ADCx);
        // Check the end of ADC reset calibration register
        while(ADC_GetResetCalibrationStatus(ADCx));
        // Start ADC calibration
        ADC_StartCalibration(ADCx);
        // Check the end of ADC calibration
        while(ADC_GetCalibrationStatus(ADCx));
        n--;
    }
}

void adcInit(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    // ADC assumes all the GPIO was already placed in 'AIN' mode
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adcData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = ADC_CHANNELS;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = ADC_CHANNELS;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_28Cycles5); // GY_X
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_28Cycles5); // GY_Y
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_28Cycles5); // GY_Z

    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_28Cycles5); // ACC_X
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 5, ADC_SampleTime_28Cycles5); // ACC_Y
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 6, ADC_SampleTime_28Cycles5); // ACC_Z

    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 7, ADC_SampleTime_28Cycles5); // POT_ELE
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 8, ADC_SampleTime_28Cycles5); // POT_AIL
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 9, ADC_SampleTime_28Cycles5); // POT_RUD

    ADC_DMACmd(ADC1, ENABLE);

    ADC_Cmd(ADC1, ENABLE);

    // Calibrate ADC
    adcCalibrateADC(ADC1, 2);

    // Fire off ADC
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

static void adcAccRead(int16_t *accelData)
{
    // ADXL335
    // 300mV/g
    // Vcc 3.0V
    accelData[0] = adcData[3];
    accelData[1] = adcData[4];
    accelData[2] = adcData[5];
}

static void adcAccAlign(int16_t *accelData)
{
    // align  OK
}

static void adcGyroRead(int16_t *gyroData)
{
    // Vcc: 3.0V
    // Pitch/Roll: LPR550AL, 2000dps mode.
    // 0.5mV/dps
    // Zero-rate: 1.23V
    // Yaw: LPY550AL, 2000dps mode.
    // 0.5mV/dps
    // Zero-rate: 1.23V

    // Need to match with: 14.375lsb per dps
    // 12-bit ADC

    gyroData[0] = adcData[0] * 2;
    gyroData[1] = adcData[1] * 2;
    gyroData[2] = adcData[2] * 2;
}

static void adcGyroAlign(int16_t *gyroData)
{
    // align OK
}

static void adcDummyInit(void)
{
    // nothing to init here
}

uint16_t adcGetBattery(void)
{
    return 0;
}
#endif
