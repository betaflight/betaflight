/*
 * Minimal compile-time stubs for the early X32M76x bring-up target.
 * These keep the link closed until the real X32 peripheral backends land.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/dshot.h"
#include "drivers/resource.h"
#include "drivers/servo_impl.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"

void _init(void)
{
}

#ifndef USE_TIMER

void timerInitTarget(void)
{
}

void timerInit(void)
{
}

void timerConfigure(const timerHardware_t *timHw, uint16_t period, uint32_t hz)
{
    UNUSED(timHw);
    UNUSED(period);
    UNUSED(hz);
}

void timerStart(const timerHardware_t *timHw)
{
    UNUSED(timHw);
}

void timerChannelConfigInput(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterSamples)
{
    UNUSED(timHw);
    UNUSED(polarityRising);
    UNUSED(inputFilterSamples);
}

void timerChannelConfigInputDual(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterSamples)
{
    UNUSED(timHw);
    UNUSED(polarityRising);
    UNUSED(inputFilterSamples);
}

void timerChannelInputPolarity(const timerHardware_t *timHw, bool polarityRising)
{
    UNUSED(timHw);
    UNUSED(polarityRising);
}

void timerChannelConfigOutput(const timerHardware_t *timHw, bool outEnable, bool stateHigh)
{
    UNUSED(timHw);
    UNUSED(outEnable);
    UNUSED(stateHigh);
}

void timerChannelConfigGPIO(const timerHardware_t *timHw, ioConfig_t mode)
{
    UNUSED(timHw);
    UNUSED(mode);
}

void timerChannelEnable(const timerHardware_t *timHw)
{
    UNUSED(timHw);
}

void timerChannelDisable(const timerHardware_t *timHw)
{
    UNUSED(timHw);
}

void timerChannelEdgeHandlerInit(timerEdgeHandlerRec_t *self, timerCCHandlerCallback *fn)
{
    self->fn = fn;
}

void timerChannelOverflowHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
    self->fn = fn;
    self->next = NULL;
}

void timerChannelConfigCallbacks(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    UNUSED(timHw);
    UNUSED(edgeCallback);
    UNUSED(overflowCallback);
}

void timerChannelConfigInterrupt(const timerHardware_t *timHw, FunctionalState newState)
{
    UNUSED(timHw);
    UNUSED(newState);
}

void timerChannelClearFlag(const timerHardware_t *timHw)
{
    UNUSED(timHw);
}

void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq)
{
    UNUSED(timHw);
    UNUSED(type);
    UNUSED(irqPriority);
    UNUSED(irq);
}

void timerConfigUpdateCallback(const timerHardware_t *timHw, timerOvrHandlerRec_t *updateCallback)
{
    UNUSED(timHw);
    UNUSED(updateCallback);
}

uint32_t timerClock(const timerHardware_t *timHw)
{
    UNUSED(timHw);
    return SystemCoreClock;
}

void timerReconfigureTimeBase(const timerHardware_t *timHw, uint16_t period, uint32_t hz)
{
    UNUSED(timHw);
    UNUSED(period);
    UNUSED(hz);
}

uint8_t timerInputInterrupt(const timerHardware_t *timHw)
{
    UNUSED(timHw);
    return 0;
}

const timerHardware_t *timerGetConfiguredByTag(ioTag_t ioTag)
{
    UNUSED(ioTag);
    return NULL;
}

const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex)
{
    UNUSED(ioTag);
    UNUSED(owner);
    UNUSED(resourceIndex);
    return NULL;
}

const timerHardware_t *timerGetByTagAndIndex(ioTag_t ioTag, unsigned timerIndex)
{
    UNUSED(ioTag);
    UNUSED(timerIndex);
    return NULL;
}

const timerHardware_t *timerGetAllocatedByNumberAndChannel(int8_t timerNumber, uint16_t timerChannel)
{
    UNUSED(timerNumber);
    UNUSED(timerChannel);
    return NULL;
}

const resourceOwner_t *timerGetOwner(const timerHardware_t *timer)
{
    UNUSED(timer);
    return &resourceOwnerFree;
}

int8_t timerGetNumberByIndex(uint8_t index)
{
    UNUSED(index);
    return -1;
}

int8_t timerGetIndexByNumber(uint8_t number)
{
    UNUSED(number);
    return -1;
}

int8_t timerGetTIMNumber(const timerHardware_t *timHw)
{
    UNUSED(timHw);
    return -1;
}

void timerReset(const timerHardware_t *timHw)
{
    UNUSED(timHw);
}

void timerSetPeriod(const timerHardware_t *timHw, uint32_t period)
{
    UNUSED(timHw);
    UNUSED(period);
}

uint32_t timerGetPeriod(const timerHardware_t *timHw)
{
    UNUSED(timHw);
    return 0;
}

void timerSetCounter(const timerHardware_t *timHw, uint32_t counter)
{
    UNUSED(timHw);
    UNUSED(counter);
}

void timerDisable(const timerHardware_t *timHw)
{
    UNUSED(timHw);
}

void timerEnable(const timerHardware_t *timHw)
{
    UNUSED(timHw);
}

void timerEnableInterrupt(const timerHardware_t *timHw)
{
    UNUSED(timHw);
}

uint32_t timerGetPrescaler(const timerHardware_t *timHw)
{
    UNUSED(timHw);
    return 0;
}

void pwmICConfig(void *tim, uint8_t channel, uint16_t polarity, uint8_t filter)
{
    UNUSED(tim);
    UNUSED(channel);
    UNUSED(polarity);
    UNUSED(filter);
}

#endif

#ifndef USE_SERVOS
void servoDevInit(const servoDevConfig_t *servoDevConfig)
{
    UNUSED(servoDevConfig);
}

void servoWrite(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
}
#endif

#ifndef USE_DSHOT
bool useDshotTelemetry = false;
uint8_t dshotMotorCount = 0;

void initDshotTelemetry(const timeUs_t looptimeUs)
{
    UNUSED(looptimeUs);
}

void updateDshotTelemetry(void)
{
}

uint16_t getDshotErpm(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

float getDshotRpm(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0.0f;
}

float getDshotRpmAverage(void)
{
    return 0.0f;
}

float getMotorFrequencyHz(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0.0f;
}

float getMinMotorFrequencyHz(void)
{
    return 0.0f;
}

bool isDshotMotorTelemetryActive(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return false;
}

bool isDshotTelemetryActive(void)
{
    return false;
}

void dshotCleanTelemetryData(void)
{
}

float erpmToRpm(uint32_t erpm)
{
    UNUSED(erpm);
    return 0.0f;
}

int16_t getDshotTelemetryMotorInvalidPercent(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

bool getDshotSensorData(escSensorData_t *dest, int motorIndex)
{
    UNUSED(dest);
    UNUSED(motorIndex);
    return false;
}
#endif

#if defined(USE_DSHOT) && !defined(USE_DSHOT_TELEMETRY)
bool useDshotTelemetry = false;

void initDshotTelemetry(const timeUs_t looptimeUs)
{
    UNUSED(looptimeUs);
}

void updateDshotTelemetry(void)
{
}

uint16_t getDshotErpm(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

float getDshotRpm(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0.0f;
}

float getDshotRpmAverage(void)
{
    return 0.0f;
}

float getMotorFrequencyHz(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0.0f;
}

float getMinMotorFrequencyHz(void)
{
    return 0.0f;
}

bool isDshotMotorTelemetryActive(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return false;
}

bool isDshotTelemetryActive(void)
{
    return false;
}

void dshotCleanTelemetryData(void)
{
}

float erpmToRpm(uint32_t erpm)
{
    UNUSED(erpm);
    return 0.0f;
}

int16_t getDshotTelemetryMotorInvalidPercent(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

bool getDshotSensorData(escSensorData_t *dest, int motorIndex)
{
    UNUSED(dest);
    UNUSED(motorIndex);
    return false;
}
#endif

#ifndef USE_ESC_SENSOR
static escSensorData_t X32StubEscSensorData;

bool escSensorInit(void)
{
    return false;
}

void escSensorProcess(timeUs_t currentTime)
{
    UNUSED(currentTime);
}

escSensorData_t *getEscSensorData(uint8_t motorNumber)
{
    UNUSED(motorNumber);
    memset(&X32StubEscSensorData, 0, sizeof(X32StubEscSensorData));
    X32StubEscSensorData.dataAge = ESC_DATA_INVALID;
    return &X32StubEscSensorData;
}

void startEscDataRead(uint8_t *frameBuffer, uint8_t frameLength)
{
    UNUSED(frameBuffer);
    UNUSED(frameLength);
}

uint8_t getNumberEscBytesRead(void)
{
    return 0;
}

uint8_t calculateCrc8(const uint8_t *buf, const uint8_t bufLen)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < bufLen; i++) {
        crc ^= buf[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)(0x07U ^ (uint8_t)(crc << 1)) : (uint8_t)(crc << 1);
        }
    }

    return crc;
}
#endif

