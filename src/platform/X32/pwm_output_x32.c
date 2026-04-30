#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_PWM_OUTPUT

#include "drivers/io.h"
#include "drivers/motor_impl.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output_impl.h"
#include "drivers/servo_impl.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "platform/timer.h"

#include "pg/motor.h"

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
    TIM_OCInitTypeDef ocInit;
    TIM_OCStructInit(&ocInit);
    ocInit.OCMode = TIM_OCMODE_PWM1;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        ocInit.OutputNState = TIM_OUTPUT_NSTATE_ENABLE;
        ocInit.OCNIdleState = TIM_OCN_IDLE_STATE_RESET;
        ocInit.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCN_POLARITY_LOW : TIM_OCN_POLARITY_HIGH;
    } else {
        ocInit.OutputState = TIM_OUTPUT_STATE_ENABLE;
        ocInit.OCIdleState = TIM_OC_IDLE_STATE_SET;
        ocInit.OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OC_POLARITY_LOW : TIM_OC_POLARITY_HIGH;
    }

    ocInit.Pulse = value;
    timerOCInit(tim, channel, &ocInit);
    timerOCPreloadConfig(tim, channel, TIM_OC_PRE_LOAD_ENABLE);
}

void pwmOutputConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion)
{
    timerReconfigureTimeBase(timerHardware, period, hz);
    pwmOCConfig((TIM_TypeDef *)timerHardware->tim,
        timerHardware->channel,
        value,
        inversion ? (timerHardware->output ^ TIMER_OUTPUT_INVERTED) : timerHardware->output);
    timerStart(timerHardware);

    channel->ccr = timerChCCR(timerHardware);
    channel->tim = timerHardware->tim;

    *channel->ccr = 0;
}

void pwmWriteChannel(timerChannel_t *channel, uint32_t value)
{
    *channel->ccr = value;
}

static FAST_DATA_ZERO_INIT motorDevice_t *pwmMotorDevice;
static bool useContinuousUpdate = true;

static void pwmWriteStandard(uint8_t index, float value)
{
    *pwmMotors[index].channel.ccr = lrintf((value * pwmMotors[index].pulseScale) + pwmMotors[index].pulseOffset);
}

static void pwmShutdownPulsesForAllMotors(void)
{
    for (int index = 0; index < pwmMotorCount; index++) {
        if (pwmMotors[index].channel.ccr) {
            *pwmMotors[index].channel.ccr = 0;
        }
    }
}

static void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors();
}

static void pwmCompleteMotorUpdate(void)
{
    if (useContinuousUpdate) {
        return;
    }

    for (int index = 0; index < pwmMotorCount; index++) {
        if (pwmMotors[index].forceOverflow) {
            timerForceOverflow(pwmMotors[index].channel.tim);
        }
        *pwmMotors[index].channel.ccr = 0;
    }
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static const motorVTable_t motorPwmVTable = {
    .postInit = motorPostInitNull,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .isMotorEnabled = pwmIsMotorEnabled,
    .shutdown = pwmShutdownPulsesForAllMotors,
    .convertExternalToMotor = pwmConvertFromExternal,
    .convertMotorToExternal = pwmConvertToExternal,
    .write = pwmWriteStandard,
    .decodeTelemetry = motorDecodeTelemetryNull,
    .updateComplete = pwmCompleteMotorUpdate,
    .requestTelemetry = NULL,
    .isMotorIdle = NULL,
    .getMotorIO = pwmGetMotorIO,
};

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig, uint16_t idlePulse)
{
    memset(pwmMotors, 0, sizeof(pwmMotors));

    if (!device) {
        return false;
    }

    device->vTable = &motorPwmVTable;
    pwmMotorDevice = device;
    pwmMotorCount = device->count;
    useContinuousUpdate = motorConfig->useContinuousUpdate;

    float sMin = 0.0f;
    float sLen = 0.0f;
    switch (motorConfig->motorProtocol) {
    default:
    case MOTOR_PROTOCOL_ONESHOT125:
        sMin = 125e-6f;
        sLen = 125e-6f;
        break;
    case MOTOR_PROTOCOL_ONESHOT42:
        sMin = 42e-6f;
        sLen = 42e-6f;
        break;
    case MOTOR_PROTOCOL_MULTISHOT:
        sMin = 5e-6f;
        sLen = 20e-6f;
        break;
    case MOTOR_PROTOCOL_BRUSHED:
        sMin = 0.0f;
        useContinuousUpdate = true;
        idlePulse = 0;
        break;
    case MOTOR_PROTOCOL_PWM:
        sMin = 1e-3f;
        sLen = 1e-3f;
        useContinuousUpdate = true;
        idlePulse = 0;
        break;
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < pwmMotorCount; motorIndex++) {
        const unsigned reorderedMotorIndex = motorConfig->motorOutputReordering[motorIndex];
        const ioTag_t tag = motorConfig->ioTags[reorderedMotorIndex];
        const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));

        if (timerHardware == NULL) {
            device->vTable = NULL;
            pwmMotorCount = 0;
            return false;
        }

        pwmMotors[motorIndex].io = IOGetByTag(tag);
        IOInit(pwmMotors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));
        IOConfigGPIOAF(pwmMotors[motorIndex].io, IOCFG_AF_PP, timerHardware->alternateFunction);

        const unsigned pwmRateHz = useContinuousUpdate ? motorConfig->motorPwmRate : ceilf(1.0f / ((sMin + sLen) * 4.0f));
        const uint32_t clock = timerClock(timerHardware);
        const unsigned prescaler = ((clock / pwmRateHz) + 0xffffU) / 0x10000U;
        const uint32_t hz = clock / prescaler;
        const unsigned period = useContinuousUpdate ? (hz / pwmRateHz) : 0xffffU;

        pwmMotors[motorIndex].pulseScale = ((motorConfig->motorProtocol == MOTOR_PROTOCOL_BRUSHED) ? period : (sLen * hz)) / 1000.0f;
        pwmMotors[motorIndex].pulseOffset = (sMin * hz) - (pwmMotors[motorIndex].pulseScale * 1000.0f);

        pwmOutputConfig(&pwmMotors[motorIndex].channel, timerHardware, hz, period, idlePulse, motorConfig->motorInversion);

        bool timerAlreadyUsed = false;
        for (int i = 0; i < motorIndex; i++) {
            if (pwmMotors[i].channel.tim == pwmMotors[motorIndex].channel.tim) {
                timerAlreadyUsed = true;
                break;
            }
        }
        pwmMotors[motorIndex].forceOverflow = !timerAlreadyUsed;
        pwmMotors[motorIndex].enabled = true;
    }

    return true;
}

pwmOutputPort_t *pwmGetMotors(void)
{
    return pwmMotors;
}

#ifdef USE_SERVOS
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

void servoWrite(uint8_t index, float value)
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

        const timerHardware_t *timer = timerAllocate(tag, OWNER_SERVO, RESOURCE_INDEX(servoIndex));
        if (timer == NULL) {
            break;
        }

        IOConfigGPIOAF(servos[servoIndex].io, IOCFG_AF_PP, timer->alternateFunction);
        pwmOutputConfig(&servos[servoIndex].channel, timer, PWM_TIMER_1MHZ, PWM_TIMER_1MHZ / servoConfig->servoPwmRate, servoConfig->servoCenterPulse, 0);
        servos[servoIndex].enabled = true;
    }
}
#endif

#endif
