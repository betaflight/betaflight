/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"
#include "drivers/time.h"

#include "drivers/io.h"
#include "pwm_output.h"
#include "timer.h"
#include "drivers/pwm_output.h"

static pwmWriteFn *pwmWrite;
static pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmCompleteWriteFn *pwmCompleteWrite = NULL;

#ifdef USE_DSHOT
loadDmaBufferFn *loadDmaBuffer;
#endif

#ifdef USE_SERVOS
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];
#endif

#ifdef BEEPER
static pwmOutputPort_t beeperPwm;
static uint16_t freqBeep = 0;
#endif

static bool pwmMotorsEnabled = false;
static bool isDshot = false;
#ifdef USE_DSHOT_DMAR
#ifdef ENABLE_DSHOT_DMAR
bool useBurstDshot = true;
#else
bool useBurstDshot = false;
#endif
#endif

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
#if defined(USE_HAL_DRIVER)
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) return;

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
    TIM_OCInitStructure.OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_SET;
    TIM_OCInitStructure.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.Pulse = value;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(Handle, &TIM_OCInitStructure, channel);
#else
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    TIM_OCInitStructure.TIM_Pulse = value;

    timerOCInit(tim, channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(tim, channel, TIM_OCPreload_Enable);
#endif
}

void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion)
{
#if defined(USE_HAL_DRIVER)
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(timerHardware->tim);
    if (Handle == NULL) return;
#endif

    configTimeBase(timerHardware->tim, period, hz);
    pwmOCConfig(timerHardware->tim,
        timerHardware->channel,
        value,
        inversion ? timerHardware->output ^ TIMER_OUTPUT_INVERTED : timerHardware->output
        );

#if defined(USE_HAL_DRIVER)
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL)
        HAL_TIMEx_PWMN_Start(Handle, timerHardware->channel);
    else
        HAL_TIM_PWM_Start(Handle, timerHardware->channel);
    HAL_TIM_Base_Start(Handle);
#else
    TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
    TIM_Cmd(timerHardware->tim, ENABLE);
#endif

    channel->ccr = timerChCCR(timerHardware);

    channel->tim = timerHardware->tim;

    *channel->ccr = 0;
}

static void pwmWriteUnused(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
}

static void pwmWriteStandard(uint8_t index, float value)
{
    /* TODO: move value to be a number between 0-1 (i.e. percent throttle from mixer) */
    *motors[index].channel.ccr = lrintf((value * motors[index].pulseScale) + motors[index].pulseOffset);
}

#ifdef USE_DSHOT
static void pwmWriteDshot(uint8_t index, float value)
{
    pwmWriteDshotInt(index, lrintf(value));
}

static uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet)
{
    for (int i = 0; i < 16; i++) {
        dmaBuffer[i * stride] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  // MSB first
        packet <<= 1;
    }

    return DSHOT_DMA_BUFFER_SIZE;
}

static uint8_t loadDmaBufferProshot(uint32_t *dmaBuffer, int stride, uint16_t packet)
{
    for (int i = 0; i < 4; i++) {
        dmaBuffer[i * stride] = PROSHOT_BASE_SYMBOL + ((packet & 0xF000) >> 12) * PROSHOT_BIT_WIDTH;  // Most significant nibble first
        packet <<= 4;   // Shift 4 bits
    }

    return PROSHOT_DMA_BUFFER_SIZE;
}
#endif

void pwmWriteMotor(uint8_t index, float value)
{
    pwmWrite(index, value);
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    for (int index = 0; index < motorCount; index++) {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        if (motors[index].channel.ccr) {
            *motors[index].channel.ccr = 0;
        }
    }
}

void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors(MAX_SUPPORTED_MOTORS);
    pwmMotorsEnabled = false;
}

void pwmEnableMotors(void)
{
    /* check motors can be enabled */
    pwmMotorsEnabled = (pwmWrite != &pwmWriteUnused);
}

bool pwmAreMotorsEnabled(void)
{
    return pwmMotorsEnabled;
}

static void pwmCompleteWriteUnused(uint8_t motorCount)
{
    UNUSED(motorCount);
}

static void pwmCompleteOneshotMotorUpdate(uint8_t motorCount)
{
    for (int index = 0; index < motorCount; index++) {
        if (motors[index].forceOverflow) {
            timerForceOverflow(motors[index].channel.tim);
        }
        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        *motors[index].channel.ccr = 0;
    }
}

void pwmCompleteMotorUpdate(uint8_t motorCount)
{
    pwmCompleteWrite(motorCount);
}

void motorDevInit(const motorDevConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount)
{
    memset(motors, 0, sizeof(motors));

    bool useUnsyncedPwm = motorConfig->useUnsyncedPwm;

    float sMin = 0;
    float sLen = 0;
    switch (motorConfig->motorPwmProtocol) {
    default:
    case PWM_TYPE_ONESHOT125:
        sMin = 125e-6f;
        sLen = 125e-6f;
        break;
    case PWM_TYPE_ONESHOT42:
        sMin = 42e-6f;
        sLen = 42e-6f;
        break;
    case PWM_TYPE_MULTISHOT:
        sMin = 5e-6f;
        sLen = 20e-6f;
        break;
    case PWM_TYPE_BRUSHED:
        sMin = 0;
        useUnsyncedPwm = true;
        idlePulse = 0;
        break;
    case PWM_TYPE_STANDARD:
        sMin = 1e-3f;
        sLen = 1e-3f;
        useUnsyncedPwm = true;
        idlePulse = 0;
        break;
#ifdef USE_DSHOT
    case PWM_TYPE_PROSHOT1000:
        pwmWrite = &pwmWriteDshot;
        loadDmaBuffer = &loadDmaBufferProshot;
        pwmCompleteWrite = &pwmCompleteDshotMotorUpdate;
        isDshot = true;
        break;
    case PWM_TYPE_DSHOT1200:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT150:
        pwmWrite = &pwmWriteDshot;
        loadDmaBuffer = &loadDmaBufferDshot;
        pwmCompleteWrite = &pwmCompleteDshotMotorUpdate;
        isDshot = true;
#ifdef USE_DSHOT_DMAR
        if (motorConfig->useBurstDshot) {
            useBurstDshot = true;
        }
#endif
        break;
#endif
    }

    if (!isDshot) {
        pwmWrite = &pwmWriteStandard;
        pwmCompleteWrite = useUnsyncedPwm ? &pwmCompleteWriteUnused : &pwmCompleteOneshotMotorUpdate;
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const ioTag_t tag = motorConfig->ioTags[motorIndex];
        const timerHardware_t *timerHardware = timerGetByTag(tag, TIM_USE_ANY);

        if (timerHardware == NULL) {
            /* not enough motors initialised for the mixer or a break in the motors */
            pwmWrite = &pwmWriteUnused;
            pwmCompleteWrite = &pwmCompleteWriteUnused;
            /* TODO: block arming and add reason system cannot arm */
            return;
        }

        motors[motorIndex].io = IOGetByTag(tag);
        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

#ifdef USE_DSHOT
        if (isDshot) {
            pwmDshotMotorHardwareConfig(timerHardware,
                motorIndex,
                motorConfig->motorPwmProtocol,
                motorConfig->motorPwmInversion ? timerHardware->output ^ TIMER_OUTPUT_INVERTED : timerHardware->output);
            motors[motorIndex].enabled = true;
            continue;
        }
#endif

#if defined(STM32F1)
        IOConfigGPIO(motors[motorIndex].io, IOCFG_AF_PP);
#else
        IOConfigGPIOAF(motors[motorIndex].io, IOCFG_AF_PP, timerHardware->alternateFunction);
#endif

        /* standard PWM outputs */
        // margin of safety is 4 periods when unsynced
        const unsigned pwmRateHz = useUnsyncedPwm ? motorConfig->motorPwmRate : ceilf(1 / ((sMin + sLen) * 4));

        const uint32_t clock = timerClock(timerHardware->tim);
        /* used to find the desired timer frequency for max resolution */
        const unsigned prescaler = ((clock / pwmRateHz) + 0xffff) / 0x10000; /* rounding up */
        const uint32_t hz = clock / prescaler;
        const unsigned period = useUnsyncedPwm ? hz / pwmRateHz : 0xffff;

        /*
            if brushed then it is the entire length of the period.
            TODO: this can be moved back to periodMin and periodLen
            once mixer outputs a 0..1 float value.
        */
        motors[motorIndex].pulseScale = ((motorConfig->motorPwmProtocol == PWM_TYPE_BRUSHED) ? period : (sLen * hz)) / 1000.0f;
        motors[motorIndex].pulseOffset = (sMin * hz) - (motors[motorIndex].pulseScale * 1000);

        pwmOutConfig(&motors[motorIndex].channel, timerHardware, hz, period, idlePulse, motorConfig->motorPwmInversion);

        bool timerAlreadyUsed = false;
        for (int i = 0; i < motorIndex; i++) {
            if (motors[i].channel.tim == motors[motorIndex].channel.tim) {
                timerAlreadyUsed = true;
                break;
            }
        }
        motors[motorIndex].forceOverflow = !timerAlreadyUsed;
        motors[motorIndex].enabled = true;
    }

    pwmMotorsEnabled = true;
}

pwmOutputPort_t *pwmGetMotors(void)
{
    return motors;
}

bool isMotorProtocolDshot(void)
{
    return isDshot;
}

#ifdef USE_DSHOT
uint32_t getDshotHz(motorPwmProtocolTypes_e pwmProtocolType)
{
    switch (pwmProtocolType) {
    case(PWM_TYPE_PROSHOT1000):
        return MOTOR_PROSHOT1000_HZ;
    case(PWM_TYPE_DSHOT1200):
        return MOTOR_DSHOT1200_HZ;
    case(PWM_TYPE_DSHOT600):
        return MOTOR_DSHOT600_HZ;
    case(PWM_TYPE_DSHOT300):
        return MOTOR_DSHOT300_HZ;
    default:
    case(PWM_TYPE_DSHOT150):
        return MOTOR_DSHOT150_HZ;
    }
}

void pwmWriteDshotCommand(uint8_t index, uint8_t motorCount, uint8_t command)
{
    if (isDshot && (command <= DSHOT_MAX_COMMAND)) {
        unsigned repeats;
        switch (command) {
        case DSHOT_CMD_SPIN_DIRECTION_1:
        case DSHOT_CMD_SPIN_DIRECTION_2:
        case DSHOT_CMD_3D_MODE_OFF:
        case DSHOT_CMD_3D_MODE_ON:
        case DSHOT_CMD_SAVE_SETTINGS:
        case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
        case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
            repeats = 10;
            break;
        default:
            repeats = 1;
            break;
        }

        for (; repeats; repeats--) {
            for (uint8_t i = 0; i < motorCount; i++) {
                if ((i == index) || (index == ALL_MOTORS)) {
                    motorDmaOutput_t *const motor = getMotorDmaOutput(i);
                    motor->requestTelemetry = true;
                    pwmWriteDshotInt(i, command);
                }
            }

            pwmCompleteDshotMotorUpdate(0);
            delay(1);
        }
    }
}

uint16_t prepareDshotPacket(motorDmaOutput_t *const motor, const uint16_t value)
{
    uint16_t packet = (value << 1) | (motor->requestTelemetry ? 1 : 0);
    motor->requestTelemetry = false;    // reset telemetry request to make sure it's triggered only once in a row

    // compute checksum
    int csum = 0;
    int csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    csum &= 0xf;
    // append checksum
    packet = (packet << 4) | csum;

    return packet;
}
#endif

#ifdef USE_SERVOS
void pwmWriteServo(uint8_t index, float value)
{
    if (index < MAX_SUPPORTED_SERVOS && servos[index].channel.ccr) {
        *servos[index].channel.ccr = lrintf(value);
    }
}

void servoDevInit(const servoDevConfig_t *servoConfig)
{
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        const ioTag_t tag = servoConfig->ioTags[servoIndex];

        if (!tag) {
            break;
        }

        servos[servoIndex].io = IOGetByTag(tag);

        IOInit(servos[servoIndex].io, OWNER_SERVO, RESOURCE_INDEX(servoIndex));

        const timerHardware_t *timer = timerGetByTag(tag, TIM_USE_ANY);
#if defined(USE_HAL_DRIVER)
        IOConfigGPIOAF(servos[servoIndex].io, IOCFG_AF_PP, timer->alternateFunction);
#else
        IOConfigGPIO(servos[servoIndex].io, IOCFG_AF_PP);
#endif

        if (timer == NULL) {
            /* flag failure and disable ability to arm */
            break;
        }
        pwmOutConfig(&servos[servoIndex].channel, timer, PWM_TIMER_1MHZ, PWM_TIMER_1MHZ / servoConfig->servoPwmRate, servoConfig->servoCenterPulse, 0);
        servos[servoIndex].enabled = true;
    }
}

#endif

#ifdef BEEPER
void pwmWriteBeeper(bool onoffBeep)
{
        if (!beeperPwm.io)
            return;
        if (onoffBeep == true) {
            *beeperPwm.channel.ccr = (PWM_TIMER_1MHZ / freqBeep) / 2;
            beeperPwm.enabled = true;
        } else {
            *beeperPwm.channel.ccr = 0;
            beeperPwm.enabled = false;
        }
}

void pwmToggleBeeper(void)
{
        pwmWriteBeeper(!beeperPwm.enabled);
}

void beeperPwmInit(const ioTag_t tag, uint16_t frequency)
{
        beeperPwm.io = IOGetByTag(tag);
        const timerHardware_t *timer = timerGetByTag(tag, TIM_USE_BEEPER);
        if (beeperPwm.io && timer) {
            IOInit(beeperPwm.io, OWNER_BEEPER, RESOURCE_INDEX(0));
#if defined(USE_HAL_DRIVER)
            IOConfigGPIOAF(beeperPwm.io, IOCFG_AF_PP, timer->alternateFunction);
#else
            IOConfigGPIO(beeperPwm.io, IOCFG_AF_PP);
#endif
            freqBeep = frequency;
            pwmOutConfig(&beeperPwm.channel, timer, PWM_TIMER_1MHZ, PWM_TIMER_1MHZ / freqBeep, (PWM_TIMER_1MHZ / freqBeep) / 2, 0);
        }
        *beeperPwm.channel.ccr = 0;
        beeperPwm.enabled = false;
}
#endif
