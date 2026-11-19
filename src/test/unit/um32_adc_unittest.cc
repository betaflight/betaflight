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

#include <array>
#include <cstdlib>

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"
#include "drivers/adc.h"
#include "drivers/dma_impl.h"
#include "drivers/io.h"
#include "platform/adc_impl.h"
#include "platform/rcc.h"
#include "pg/adc.h"

ADC_TypeDef mockAdc[ADCDEV_COUNT];
DMA_TypeDef mockDma[2];
RCM_TypeDef mockRcm;
uint16_t mockTemperatureCalibration[2] = {1785, 3618};
int32_t adcVREFINTCAL, adcTSCAL1, adcTSCAL2, adcTSSlopeK;
struct dmaResource_s { uint8_t stream; };
dmaResource_t mockDmaStreams[2];
dmaChannelDescriptor_t dmaDescriptors[2];
adcOperatingConfig_t adcOperatingConfig[ADC_SOURCE_COUNT];
volatile uint16_t adcValues[ADC_SOURCE_COUNT];
extern volatile uint16_t adcConversionBuffer[ADC_SOURCE_COUNT];
}

namespace {
struct AdcState {
    unsigned opaInitCalls;
    unsigned adcInitCalls;
    unsigned configuredCount;
    unsigned rankCount;
    unsigned startCount;
    std::ptrdiff_t offset;
};

std::array<AdcState, ADCDEV_COUNT> adcState;
std::array<dmaChannelSpec_t, ADCDEV_COUNT> dmaSpecs;
rccPeriphTag_t currentClock;
bool started;

class Um32AdcTest : public ::testing::Test {
protected:
    adcConfig_t config{};

    void SetUp() override
    {
        adcState = {};
        currentClock = 0;
        started = false;
        for (unsigned stream = 0; stream < 2; stream++) {
            mockDmaStreams[stream].stream = stream;
            dmaDescriptors[stream] = {};
            dmaDescriptors[stream].ref = &mockDmaStreams[stream];
        }

        // Give the mock ADCs distinct DMA streams unless a test exercises a collision.
        dmaSpecs[ADCDEV_1] = {DMA_CODE(2, 4, 0), &mockDmaStreams[0], 4};
        dmaSpecs[ADCDEV_2] = {DMA_CODE(2, 2, 1), &mockDmaStreams[1], 2};
        config.device = ADC_DEV_TO_CFG(ADCDEV_1);
        config.current = {true, IO_TAG(PC1), ADC_DEV_TO_CFG(ADCDEV_1)};
        config.vbat = {true, IO_TAG(PC0), ADC_DEV_TO_CFG(ADCDEV_2)};
        config.rssi = {true, IO_TAG(PC2), ADC_DEV_TO_CFG(ADCDEV_2)};
    }

    void expectAdc(adcDevice_e dev, unsigned count, std::ptrdiff_t offset)
    {
        const AdcState &state = adcState[dev];
        EXPECT_EQ(count != 0, state.opaInitCalls);
        EXPECT_EQ(count != 0, state.adcInitCalls);
        EXPECT_EQ(count, state.configuredCount);
        EXPECT_EQ(count, state.rankCount);
        EXPECT_EQ(count, state.startCount);
        if (count != 0) {
            EXPECT_EQ(offset, state.offset);
        }
    }

    void expectSample(adcSource_e source, unsigned index, uint16_t sample)
    {
        ASSERT_TRUE(adcOperatingConfig[source].enabled);
        EXPECT_EQ(index, adcOperatingConfig[source].dmaIndex);
        EXPECT_EQ(sample, adcValues[index]);
    }
};
}

extern "C" {

void Error_Handler(void)
{
    std::abort();
}

IO_t IOGetByTag(ioTag_t)
{
    return IO_NONE;
}

void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIO(IO_t, ioConfig_t) {}

void RCC_ClockCmd(rccPeriphTag_t clock, FunctionalState state)
{
    EXPECT_EQ(ENABLE, state);
    currentClock = clock;
}

int dmaGetHandlerCount(void)
{
    return 2;
}

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t option)
{
    EXPECT_EQ(DMA_PERIPH_ADC, device);
    return index < ADCDEV_COUNT && option == 0 ? &dmaSpecs[index] : nullptr;
}

void dmaEnable(dmaIdentifier_e id)
{
    EXPECT_EQ(OWNER_ADC, dmaGetOwner(id)->owner);
}

int HAL_OPA_Init(OPA_HandleTypeDef *handle)
{
    if (handle->Opax > OPA1) {
        return HAL_ERROR;
    }
    const unsigned dev = handle->Opax;
    EXPECT_EQ(dev == ADCDEV_1 ? RCC_APB2(ADC1) : RCC_APB2(ADC2), currentClock);
    EXPECT_EQ(HAL_OPA_MODE_UNITBUFF, handle->Opa_Mode);
    EXPECT_EQ(OPA_SELP_ADCMUX, handle->Init.Selp);
    EXPECT_EQ(0U, adcState[dev].adcInitCalls);
    adcState[dev].opaInitCalls++;
    return HAL_OK;
}

int HAL_ADC_Init(ADC_HandleTypeDef *handle)
{
    AdcState &state = adcState[handle->Instance - mockAdc];
    EXPECT_EQ(1U, state.opaInitCalls);
    EXPECT_EQ(ENABLE, handle->Init.Opamp_en);
    state.adcInitCalls++;
    state.configuredCount = handle->Init.NbrOfConversion;
    return HAL_OK;
}

int HAL_ADC_DeInit(ADC_HandleTypeDef *)
{
    EXPECT_EQ(0U, adcState[ADCDEV_1].adcInitCalls);
    EXPECT_EQ(0U, adcState[ADCDEV_2].adcInitCalls);
    return HAL_OK;
}

int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *handle, ADC_ChannelConfTypeDef *config)
{
    EXPECT_FALSE(started);
    AdcState &state = adcState[handle->Instance - mockAdc];
    EXPECT_EQ(++state.rankCount, config->Rank);
    return HAL_OK;
}

int HAL_DMA_Init(DMA_HandleTypeDef *)
{
    EXPECT_FALSE(started);
    return HAL_OK;
}

int HAL_DMA_DeInit(DMA_HandleTypeDef *)
{
    EXPECT_FALSE(started);
    return HAL_OK;
}

int HAL_TS_Init(TS_HandleTypeDef *handle)
{
    EXPECT_EQ(TS, handle->Instance);
    return HAL_OK;
}

uint16_t HAL_TS_get_data(TS_HandleTypeDef *)
{
    return 0;
}

int HAL_ADC_Start_DMA(ADC_HandleTypeDef *handle, uint32_t *buffer, uint32_t count)
{
    const auto dev = handle->Instance - mockAdc;
    AdcState &state = adcState[dev];
    EXPECT_NE(nullptr, handle->DMA_Handle);
    EXPECT_EQ(state.configuredCount, count);
    EXPECT_EQ(state.rankCount, count);
    const auto values = reinterpret_cast<uint16_t *>(buffer);
    state.offset = values - adcConversionBuffer;
    if (state.offset < 0 || state.offset + count > ADC_EXTERNAL_COUNT) {
        return HAL_ERROR;
    }
    started = true;
    state.startCount = count;
    for (uint32_t i = 0; i < count; i++) {
        values[i] = 0xF000 | ((dev + 1) * 100 + i);
    }
    return HAL_OK;
}

}

TEST_F(Um32AdcTest, InitializesEachSelectedOpaBeforeItsAdc)
{
    adcInit(&config);
    adcGetChannelValues();

    expectAdc(ADCDEV_1, 1, 0);
    expectAdc(ADCDEV_2, 2, 1);
    expectSample(ADC_CURRENT, 0, 100);
    expectSample(ADC_BATTERY, 1, 200);
    expectSample(ADC_RSSI, 2, 201);
}

TEST_F(Um32AdcTest, StartsAdc2AtFirstBufferSlotWhenAdc1HasNoDmaOption)
{
    config.dmaopt[ADCDEV_1] = DMA_OPT_UNUSED;
    adcInit(&config);
    adcGetChannelValues();

    expectAdc(ADCDEV_1, 0, 0);
    expectAdc(ADCDEV_2, 2, 0);
    EXPECT_FALSE(adcOperatingConfig[ADC_CURRENT].enabled);
    EXPECT_TRUE(adcOperatingConfig[ADC_VREFINT].enabled);
    EXPECT_TRUE(adcOperatingConfig[ADC_TEMPSENSOR].enabled);
    expectSample(ADC_BATTERY, 0, 200);
    expectSample(ADC_RSSI, 1, 201);
}

TEST_F(Um32AdcTest, StartsAdc1WhenAdc2HasNoDmaOption)
{
    config.dmaopt[ADCDEV_2] = DMA_OPT_UNUSED;
    adcInit(&config);
    adcGetChannelValues();

    expectAdc(ADCDEV_1, 1, 0);
    expectAdc(ADCDEV_2, 0, 0);
    expectSample(ADC_CURRENT, 0, 100);
    EXPECT_FALSE(adcOperatingConfig[ADC_BATTERY].enabled);
    EXPECT_FALSE(adcOperatingConfig[ADC_RSSI].enabled);
}

TEST_F(Um32AdcTest, StartsAdc2AtFirstBufferSlotWhenAdc1DmaIsOccupied)
{
    ASSERT_TRUE(dmaAllocate(DMA_FIRST_HANDLER, OWNER_MOTOR, 1));
    adcInit(&config);
    adcGetChannelValues();

    expectAdc(ADCDEV_1, 0, 0);
    expectAdc(ADCDEV_2, 2, 0);
    EXPECT_FALSE(adcOperatingConfig[ADC_CURRENT].enabled);
    expectSample(ADC_BATTERY, 0, 200);
    expectSample(ADC_RSSI, 1, 201);
}

TEST_F(Um32AdcTest, PreservesAdc1WhenAdc2RequestsTheSameStream)
{
    dmaSpecs[ADCDEV_2] = {DMA_CODE(2, 2, 0), &mockDmaStreams[0], 2};
    adcInit(&config);
    adcGetChannelValues();

    expectAdc(ADCDEV_1, 1, 0);
    expectAdc(ADCDEV_2, 0, 0);
    EXPECT_EQ(RESOURCE_INDEX(ADCDEV_1), dmaGetOwner(DMA_FIRST_HANDLER)->index);
    expectSample(ADC_CURRENT, 0, 100);
    EXPECT_FALSE(adcOperatingConfig[ADC_BATTERY].enabled);
    EXPECT_FALSE(adcOperatingConfig[ADC_RSSI].enabled);
}

TEST_F(Um32AdcTest, SharesOneRankAndDmaSlotForDuplicateInputs)
{
    config.external1 = config.current;
    adcInit(&config);
    adcGetChannelValues();

    expectAdc(ADCDEV_1, 1, 0);
    expectAdc(ADCDEV_2, 2, 1);
    expectSample(ADC_CURRENT, 0, 100);
    expectSample(ADC_EXTERNAL1, 0, 100);
    expectSample(ADC_BATTERY, 1, 200);
}
