/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define UM324xF
#define USE_ADC
#define USE_ADC_INTERNAL
#define USE_DMA
#define USE_DMA_SPEC
#define USE_OPA
#define PLATFORM_TRAIT_ADC_DEVICE 1
#define PLATFORM_TRAIT_RCC 1
#include "../platform.h"

// Mock the vendor ADC and DMA handles used by the UM32 driver.
typedef struct {
    DMA_TypeDef *Instance;
    uint32_t DmaChannelSel;
    struct {
        uint32_t Direction, DstMSize, SrcMSize, SrcInc, DstInc;
        uint32_t SrcDataAlignment, DstDataAlignment, SrcHsSel, DstHsSel;
        uint32_t SrcPer, DstPer, SrcReload, DstReload, FIFOMode, FCMode, Priority;
    } Init;
} DMA_HandleTypeDef;

typedef struct {
    ADC_TypeDef *Instance;
    DMA_HandleTypeDef *DMA_Handle;
    struct {
        uint32_t Opamp_en, ContinuousConvMode, DiscontinuousConvMode;
        uint32_t NbrOfDiscConversion, ExternalTrigConv, ExternalTrigConvEdge;
        uint32_t ClockPrescaler, NbrOfConversion;
    } Init;
} ADC_HandleTypeDef;

typedef struct {
    uint32_t Channel, Rank, AverageTimes;
} ADC_ChannelConfTypeDef;

typedef struct {
    uint32_t Opax, Opa_Mode;
    struct {
        uint32_t Capen, Fbresen, GainSel, Otpen, Seln, Selp;
    } Init;
} OPA_HandleTypeDef;

typedef struct {
    uint32_t Instance;
    struct {
        uint32_t Mode, WorkMode, ChopEn, ChopDiv, irq_en;
    } Init;
} TS_HandleTypeDef;

typedef struct {
    uint32_t CFGR0;
} RCM_TypeDef;

enum { HAL_OK, HAL_ERROR };
enum { OPA0, OPA1 };
enum {
    HAL_OPA_MODE_UNITBUFF = 1, OPA_CAPEN_OPEN, OPA_FBRESEN_OPEN, OPA_GAINSEL_1X,
    OPA_OTPEN_CLOSE, OPA_SELN_RESERVE, OPA_SELP_ADCMUX,
};
enum {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_6 = 6,
    ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11,
    ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15,
};
enum { ADC_SOFTWARE_START, ADC_EXTERNALTRIGCONVEDGE_NONE, ADC_CHANNEL_AVERAGE_TIMES_1 };
enum { TS = 1, TS_NORMAL_MODE = 0, TS_HIGH_SPEED_MODE = 1, TS_CHOP_CLK_EN = 1 };
enum {
    DMA_PERIPH_TO_MEMORY, DMA_BURST_DST_NUM_1, DMA_BURST_SRC_NUM_1,
    DMA_SRCINC_NOC, DMA_DSTINC_INC, DMA_SRCDATAALIGN_HALFWORD, DMA_DSTDATAALIGN_HALFWORD,
    DMA_SRC_HS_HW, DMA_DST_HS_HW, DMA_SRC_RELOAD_ENABLE, DMA_DST_RELOAD_ENABLE,
    DMA_FIFOMODE_DISABLE, DMA_FCMODE_DISABLE, LL_DMA_PRIORITY_1,
};

extern ADC_TypeDef mockAdc[2];
extern DMA_TypeDef mockDma[2];
extern RCM_TypeDef mockRcm;
extern uint16_t mockTemperatureCalibration[2];
#define ADC1 (&mockAdc[0])
#define ADC2 (&mockAdc[1])
#define DMA1 (&mockDma[0])
#define DMA2 (&mockDma[1])
#define RCM (&mockRcm)
#define RCH_VALUE 48000000U
#define RCM_CFGR0_RCH_DIV 3U
#define RCM_CFGR0_RCH_DIV_Pos 0U
#define RCM_APB2CKENR_ADC1EN (1U << 0)
#define RCM_APB2CKENR_ADC2EN (1U << 1)
#define DMA_RAM_R
#define __HAL_LINKDMA(handle, field, dma) ((handle)->field = &(dma))
#define IO_CONFIG(mode, speed, pull) ((mode) | (speed) | (pull))
#define GPIO_MODE_ANALOG 1
#define GPIO_SPEED_FREQ_HIGH 2
#define GPIO_NOPULL 0
#define READ_BIT(reg, mask) ((reg) & (mask))

int HAL_OPA_Init(OPA_HandleTypeDef *handle);
int HAL_ADC_Init(ADC_HandleTypeDef *handle);
int HAL_ADC_DeInit(ADC_HandleTypeDef *handle);
int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *handle, ADC_ChannelConfTypeDef *config);
int HAL_ADC_Start_DMA(ADC_HandleTypeDef *handle, uint32_t *buffer, uint32_t count);
int HAL_DMA_Init(DMA_HandleTypeDef *handle);
int HAL_DMA_DeInit(DMA_HandleTypeDef *handle);
int HAL_TS_Init(TS_HandleTypeDef *handle);
uint16_t HAL_TS_get_data(TS_HandleTypeDef *handle);
