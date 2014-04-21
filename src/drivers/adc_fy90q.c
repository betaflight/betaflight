#ifdef FY90Q

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "sensors_common.h" // FIXME dependency into the main code
#include "accgyro_common.h"

#include "system_common.h"

#include "adc_fy90q.h"

//#include "boardalignment.h"

volatile uint16_t adcData[ADC_CHANNELS] = { 0, };

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

uint16_t adcGetChannel(uint8_t channel)
{
    return 0; // Not Supported
}

#endif
