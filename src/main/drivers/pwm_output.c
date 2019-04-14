/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
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

static FAST_RAM_ZERO_INIT pwmWriteFn *pwmWrite;
static FAST_RAM_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static FAST_RAM_ZERO_INIT pwmCompleteWriteFn *pwmCompleteWrite = NULL;
#ifdef USE_DSHOT_TELEMETRY
static FAST_RAM_ZERO_INIT pwmStartWriteFn *pwmStartWrite = NULL;
#endif

#ifdef USE_DSHOT
FAST_RAM_ZERO_INIT loadDmaBufferFn *loadDmaBuffer;
#define DSHOT_INITIAL_DELAY_US 10000
#define DSHOT_COMMAND_DELAY_US 1000
#define DSHOT_ESCINFO_DELAY_US 12000
#define DSHOT_BEEP_DELAY_US 100000
#define DSHOT_MAX_COMMANDS 3

typedef enum {
    DSHOT_COMMAND_STATE_IDLEWAIT,   // waiting for motors to go idle
    DSHOT_COMMAND_STATE_STARTDELAY, // initial delay period before a sequence of commands
    DSHOT_COMMAND_STATE_ACTIVE,     // actively sending the command (with optional repeated output)
    DSHOT_COMMAND_STATE_POSTDELAY   // delay period after the command has been sent
} dshotCommandState_e;

typedef struct dshotCommandControl_s {
    dshotCommandState_e state;
    uint32_t nextCommandCycleDelay;
    timeUs_t delayAfterCommandUs;
    uint8_t repeats;
    uint8_t command[MAX_SUPPORTED_MOTORS];
} dshotCommandControl_t;

static timeUs_t dshotCommandPidLoopTimeUs = 125; // default to 8KHz (125us) loop to prevent possible div/0
                                                 // gets set to the actual value when the PID loop is initialized

static dshotCommandControl_t commandQueue[DSHOT_MAX_COMMANDS + 1];
static uint8_t commandQueueHead;
static uint8_t commandQueueTail;
#endif

#ifdef USE_SERVOS
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];
#endif

#ifdef USE_BEEPER
static pwmOutputPort_t beeperPwm;
static uint16_t freqBeep = 0;
#endif

static bool pwmMotorsEnabled = false;
static bool isDshot = false;
#ifdef USE_DSHOT_DMAR
FAST_RAM_ZERO_INIT bool useBurstDshot = false;
#endif
#ifdef USE_DSHOT_TELEMETRY
FAST_RAM_ZERO_INIT bool useDshotTelemetry = false;
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
static FAST_CODE void pwmWriteDshot(uint8_t index, float value)
{
    pwmWriteDshotInt(index, lrintf(value));
}

static FAST_CODE uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet)
{
    for (int i = 0; i < 16; i++) {
        dmaBuffer[i * stride] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  // MSB first
        packet <<= 1;
    }
    dmaBuffer[16 * stride] = 0;
    dmaBuffer[17 * stride] = 0;

    return DSHOT_DMA_BUFFER_SIZE;
}

static uint8_t loadDmaBufferProshot(uint32_t *dmaBuffer, int stride, uint16_t packet)
{
    for (int i = 0; i < 4; i++) {
        dmaBuffer[i * stride] = PROSHOT_BASE_SYMBOL + ((packet & 0xF000) >> 12) * PROSHOT_BIT_WIDTH;  // Most significant nibble first
        packet <<= 4;   // Shift 4 bits
    }
    dmaBuffer[4 * stride] = 0;
    dmaBuffer[5 * stride] = 0;

    return PROSHOT_DMA_BUFFER_SIZE;
}

void setDshotPidLoopTime(uint32_t pidLoopTime)
{
    dshotCommandPidLoopTimeUs = pidLoopTime;
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

#ifdef USE_DSHOT_TELEMETRY
static bool pwmStartWriteUnused(uint8_t motorCount)
{
    UNUSED(motorCount);
    return true;
}
#endif

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

#ifdef USE_DSHOT_TELEMETRY
bool pwmStartMotorUpdate(uint8_t motorCount)
{
    return pwmStartWrite(motorCount);
}
#endif

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
#ifdef USE_DSHOT_TELEMETRY
        pwmStartWrite = &pwmStartDshotMotorUpdate;
        useDshotTelemetry = motorConfig->useDshotTelemetry;
#endif
        isDshot = true;
        break;
    case PWM_TYPE_DSHOT1200:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT150:
        pwmWrite = &pwmWriteDshot;
        loadDmaBuffer = &loadDmaBufferDshot;
        pwmCompleteWrite = &pwmCompleteDshotMotorUpdate;
#ifdef USE_DSHOT_TELEMETRY
        pwmStartWrite = &pwmStartDshotMotorUpdate;
        useDshotTelemetry = motorConfig->useDshotTelemetry;
#endif
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
#ifdef USE_DSHOT_TELEMETRY
        pwmStartWrite = pwmStartWriteUnused;
#endif
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const ioTag_t tag = motorConfig->ioTags[motorIndex];
        const timerHardware_t *timerHardware = timerGetByTag(tag);

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

bool allMotorsAreIdle(uint8_t motorCount)
{
    bool allMotorsIdle = true;
    for (unsigned i = 0; i < motorCount; i++) {
        const motorDmaOutput_t *motor = getMotorDmaOutput(i);
        if (motor->value) {
            allMotorsIdle = false;
        }
    }

    return allMotorsIdle;
}

FAST_CODE bool pwmDshotCommandQueueFull()
{
    return (commandQueueHead + 1) % (DSHOT_MAX_COMMANDS + 1) == commandQueueTail;
}

FAST_CODE bool pwmDshotCommandIsQueued(void)
{
    return commandQueueHead != commandQueueTail;
}

static FAST_CODE bool isLastDshotCommand(void)
{
    return ((commandQueueTail + 1) % (DSHOT_MAX_COMMANDS + 1) == commandQueueHead);
}

FAST_CODE bool pwmDshotCommandIsProcessing(void)
{
    if (!pwmDshotCommandIsQueued()) {
        return false;
    }
    dshotCommandControl_t* command = &commandQueue[commandQueueTail];
    const bool commandIsProcessing = command->state == DSHOT_COMMAND_STATE_STARTDELAY
                                     || command->state == DSHOT_COMMAND_STATE_ACTIVE
                                     || (command->state == DSHOT_COMMAND_STATE_POSTDELAY && !isLastDshotCommand());
    return commandIsProcessing;
}

static FAST_CODE bool pwmDshotCommandQueueUpdate(void)
{
    if (pwmDshotCommandIsQueued()) {
        commandQueueTail = (commandQueueTail + 1) % (DSHOT_MAX_COMMANDS + 1);
        if (pwmDshotCommandIsQueued()) {
            // There is another command in the queue so update it so it's ready to output in
            // sequence. It can go directly to the DSHOT_COMMAND_STATE_ACTIVE state and bypass
            // the DSHOT_COMMAND_STATE_IDLEWAIT and DSHOT_COMMAND_STATE_STARTDELAY states.
            dshotCommandControl_t* nextCommand = &commandQueue[commandQueueTail];
            nextCommand->state = DSHOT_COMMAND_STATE_ACTIVE;
            nextCommand->nextCommandCycleDelay = 0;
            return true;
        }
    }
    return false;
}

static FAST_CODE uint32_t dshotCommandCyclesFromTime(timeUs_t delayUs)
{
    // Find the minimum number of motor output cycles needed to
    // provide at least delayUs time delay
    uint32_t ret = delayUs / dshotCommandPidLoopTimeUs;
    if (delayUs % dshotCommandPidLoopTimeUs) {
        ret++;
    }
    return ret;
}

static dshotCommandControl_t* addCommand()
{
    int newHead = (commandQueueHead + 1) % (DSHOT_MAX_COMMANDS + 1);
    if (newHead == commandQueueTail) {
        return NULL;
    }
    dshotCommandControl_t* control = &commandQueue[commandQueueHead];
    commandQueueHead = newHead;
    return control;
}


void pwmWriteDshotCommand(uint8_t index, uint8_t motorCount, uint8_t command, bool blocking)
{
    if (!isMotorProtocolDshot() || (command > DSHOT_MAX_COMMAND) || pwmDshotCommandQueueFull()) {
        return;
    }

    uint8_t repeats = 1;
    timeUs_t delayAfterCommandUs = DSHOT_COMMAND_DELAY_US;

    switch (command) {
    case DSHOT_CMD_SPIN_DIRECTION_1:
    case DSHOT_CMD_SPIN_DIRECTION_2:
    case DSHOT_CMD_3D_MODE_OFF:
    case DSHOT_CMD_3D_MODE_ON:
    case DSHOT_CMD_SAVE_SETTINGS:
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
    case DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE:
    case DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY:
        repeats = 10;
        break;
    case DSHOT_CMD_BEACON1:
    case DSHOT_CMD_BEACON2:
    case DSHOT_CMD_BEACON3:
    case DSHOT_CMD_BEACON4:
    case DSHOT_CMD_BEACON5:
        delayAfterCommandUs = DSHOT_BEEP_DELAY_US;
        break;
    default:
        break;
    }

    if (blocking) {
        delayMicroseconds(DSHOT_INITIAL_DELAY_US - DSHOT_COMMAND_DELAY_US);
        for (; repeats; repeats--) {
            delayMicroseconds(DSHOT_COMMAND_DELAY_US);

#ifdef USE_DSHOT_TELEMETRY
            timeUs_t currentTimeUs = micros();
            while (!pwmStartDshotMotorUpdate(motorCount) &&
                   cmpTimeUs(micros(), currentTimeUs) < 1000);
#endif
            for (uint8_t i = 0; i < motorCount; i++) {
                if ((i == index) || (index == ALL_MOTORS)) {
                    motorDmaOutput_t *const motor = getMotorDmaOutput(i);
                    motor->requestTelemetry = true;
                    pwmWriteDshotInt(i, command);
                }
            }

            pwmCompleteDshotMotorUpdate(0);
        }
        delayMicroseconds(delayAfterCommandUs);
    } else {
        dshotCommandControl_t *commandControl = addCommand();
        if (commandControl) {
            commandControl->repeats = repeats;
            commandControl->delayAfterCommandUs = delayAfterCommandUs;
            for (unsigned i = 0; i < motorCount; i++) {
                if (index == i || index == ALL_MOTORS) {
                    commandControl->command[i] = command;
                } else {
                    commandControl->command[i] = DSHOT_CMD_MOTOR_STOP;
                }
            }
            if (allMotorsAreIdle(motorCount)) {
                // we can skip the motors idle wait state
                commandControl->state = DSHOT_COMMAND_STATE_STARTDELAY;
                commandControl->nextCommandCycleDelay = dshotCommandCyclesFromTime(DSHOT_INITIAL_DELAY_US);
            } else {
                commandControl->state = DSHOT_COMMAND_STATE_STARTDELAY;
                commandControl->nextCommandCycleDelay = 0;  // will be set after idle wait completes
            }
        }
    }
}

uint8_t pwmGetDshotCommand(uint8_t index)
{
    return commandQueue[commandQueueTail].command[index];
}

// This function is used to synchronize the dshot command output timing with
// the normal motor output timing tied to the PID loop frequency. A "true" result
// allows the motor output to be sent, "false" means delay until next loop. So take
// the example of a dshot command that needs to repeat 10 times at 1ms intervals.
// If we have a 8KHz PID loop we'll end up sending the dshot command every 8th motor output.
FAST_CODE_NOINLINE bool pwmDshotCommandOutputIsEnabled(uint8_t motorCount)
{
    dshotCommandControl_t* command = &commandQueue[commandQueueTail];
    switch (command->state) {
    case DSHOT_COMMAND_STATE_IDLEWAIT:
        if (allMotorsAreIdle(motorCount)) {
            command->state = DSHOT_COMMAND_STATE_STARTDELAY;
            command->nextCommandCycleDelay = dshotCommandCyclesFromTime(DSHOT_INITIAL_DELAY_US);
        }
        break;

    case DSHOT_COMMAND_STATE_STARTDELAY:
        if (command->nextCommandCycleDelay-- > 1) {
            return false;  // Delay motor output until the start of the command seequence
        }
        command->state = DSHOT_COMMAND_STATE_ACTIVE;
        command->nextCommandCycleDelay = 0;  // first iteration of the repeat happens now
        FALLTHROUGH;

    case DSHOT_COMMAND_STATE_ACTIVE:
        if (command->nextCommandCycleDelay-- > 1) {
            return false;  // Delay motor output until the next command repeat
        }

        command->repeats--;
        if (command->repeats) {
            command->nextCommandCycleDelay = dshotCommandCyclesFromTime(DSHOT_COMMAND_DELAY_US);
        } else {
            command->state = DSHOT_COMMAND_STATE_POSTDELAY;
            command->nextCommandCycleDelay = dshotCommandCyclesFromTime(command->delayAfterCommandUs);
            if (!isLastDshotCommand() && command->nextCommandCycleDelay > 0) {
                // Account for the 1 extra motor output loop between commands.
                // Otherwise the inter-command delay will be DSHOT_COMMAND_DELAY_US + 1 loop.
                command->nextCommandCycleDelay--;
            }
        }
        break;

    case DSHOT_COMMAND_STATE_POSTDELAY:
        if (command->nextCommandCycleDelay-- > 1) {
            return false;  // Delay motor output until the end of the post-command delay
        }
        if (pwmDshotCommandQueueUpdate()) {
            // Will be true if the command queue is not empty and we
            // want to wait for the next command to start in sequence.
            return false;
        }
    }

    return true;
}

FAST_CODE uint16_t prepareDshotPacket(motorDmaOutput_t *const motor)
{
    uint16_t packet = (motor->value << 1) | (motor->requestTelemetry ? 1 : 0);
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

        const timerHardware_t *timer = timerGetByTag(tag);

        if (timer == NULL) {
            /* flag failure and disable ability to arm */
            break;
        }

#if defined(STM32F1)
        IOConfigGPIO(servos[servoIndex].io, IOCFG_AF_PP);
#else
        IOConfigGPIOAF(servos[servoIndex].io, IOCFG_AF_PP, timer->alternateFunction);
#endif

        pwmOutConfig(&servos[servoIndex].channel, timer, PWM_TIMER_1MHZ, PWM_TIMER_1MHZ / servoConfig->servoPwmRate, servoConfig->servoCenterPulse, 0);
        servos[servoIndex].enabled = true;
    }
}

#endif

#ifdef USE_BEEPER
void pwmWriteBeeper(bool onoffBeep)
{
    if (!beeperPwm.io) {
        return;
    }

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
    const timerHardware_t *timer = timerGetByTag(tag);
    IO_t beeperIO = IOGetByTag(tag);

    if (beeperIO && timer) {
        beeperPwm.io = beeperIO;
        IOInit(beeperPwm.io, OWNER_BEEPER, RESOURCE_INDEX(0));
#if defined(STM32F1)
        IOConfigGPIO(beeperPwm.io, IOCFG_AF_PP);
#else
        IOConfigGPIOAF(beeperPwm.io, IOCFG_AF_PP, timer->alternateFunction);
#endif
        freqBeep = frequency;
        pwmOutConfig(&beeperPwm.channel, timer, PWM_TIMER_1MHZ, PWM_TIMER_1MHZ / freqBeep, (PWM_TIMER_1MHZ / freqBeep) / 2, 0);

        *beeperPwm.channel.ccr = 0;
        beeperPwm.enabled = false;
    }
}
#endif
