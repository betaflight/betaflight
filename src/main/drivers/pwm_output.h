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

#pragma once

#include "platform.h"

#include "common/time.h"

#include "drivers/io_types.h"
#include "drivers/pwm_output_counts.h"
#include "drivers/timer.h"


#define ALL_MOTORS 255
#define DSHOT_MAX_COMMAND 47
#define DSHOT_TELEMETRY_INPUT_LEN 32
#define PROSHOT_TELEMETRY_INPUT_LEN 8

#define MOTOR_OUTPUT_LIMIT_PERCENT_MIN 1
#define MOTOR_OUTPUT_LIMIT_PERCENT_MAX 100

/*
  DshotSettingRequest (KISS24). Spin direction, 3d and save Settings reqire 10 requests.. and the TLM Byte must always be high if 1-47 are used to send settings

  3D Mode:
  0 = stop
  48   (low) - 1047 (high) -> negative direction
  1048 (low) - 2047 (high) -> positive direction
 */

typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47
} dshotCommands_e;

#define DSHOT_MIN_THROTTLE       48
#define DSHOT_MAX_THROTTLE     2047

typedef enum {
    PWM_TYPE_STANDARD = 0,
    PWM_TYPE_ONESHOT125,
    PWM_TYPE_ONESHOT42,
    PWM_TYPE_MULTISHOT,
    PWM_TYPE_BRUSHED,
#ifdef USE_DSHOT
    PWM_TYPE_DSHOT150,
    PWM_TYPE_DSHOT300,
    PWM_TYPE_DSHOT600,
    PWM_TYPE_DSHOT1200,
    PWM_TYPE_PROSHOT1000,
#endif
    PWM_TYPE_MAX
} motorPwmProtocolTypes_e;

#define PWM_TIMER_1MHZ        MHZ_TO_HZ(1)

#ifdef USE_DSHOT
#define MAX_DMA_TIMERS        8

#define MOTOR_DSHOT1200_HZ    MHZ_TO_HZ(24)
#define MOTOR_DSHOT600_HZ     MHZ_TO_HZ(12)
#define MOTOR_DSHOT300_HZ     MHZ_TO_HZ(6)
#define MOTOR_DSHOT150_HZ     MHZ_TO_HZ(3)

#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       20

#define MOTOR_PROSHOT1000_HZ         MHZ_TO_HZ(24)
#define PROSHOT_BASE_SYMBOL          24 // 1uS
#define PROSHOT_BIT_WIDTH            3
#define MOTOR_NIBBLE_LENGTH_PROSHOT  96 // 4uS

#define DSHOT_TELEMETRY_DEADTIME_US   (2 * 30 + 10) // 2 * 30uS to switch lines plus 10us grace period
#endif


#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */
#define PROSHOT_DMA_BUFFER_SIZE 6  /* resolution + frame reset (2us) */

#ifdef USE_DSHOT_TELEMETRY
#ifdef USE_DSHOT_TELEMETRY_STATS
#define DSHOT_TELEMETRY_QUALITY_WINDOW 1       // capture a rolling 1 second of packet stats
#define DSHOT_TELEMETRY_QUALITY_BUCKET_MS 100  // determines the granularity of the stats and the overall number of rolling buckets
#define DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT (DSHOT_TELEMETRY_QUALITY_WINDOW * 1000 / DSHOT_TELEMETRY_QUALITY_BUCKET_MS)

typedef struct dshotTelemetryQuality_s {
    uint32_t packetCountSum;
    uint32_t invalidCountSum;
    uint32_t packetCountArray[DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT];
    uint32_t invalidCountArray[DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT];
    uint8_t lastBucketIndex;
}  dshotTelemetryQuality_t;
#endif // USE_DSHOT_TELEMETRY_STATS
#endif // USE_DSHOT_TELEMETRY

typedef struct {
    TIM_TypeDef *timer;
#if defined(USE_DSHOT) && defined(USE_DSHOT_DMAR)
#ifdef STM32F3
    DMA_Channel_TypeDef *dmaBurstRef;
#else
    DMA_Stream_TypeDef *dmaBurstRef;
#endif
    uint16_t dmaBurstLength;
    uint32_t dmaBurstBuffer[DSHOT_DMA_BUFFER_SIZE * 4];
    timeUs_t inputDirectionStampUs;
#endif
    uint16_t timerDmaSources;
} motorDmaTimer_t;

typedef struct {
    ioTag_t ioTag;
    const timerHardware_t *timerHardware;
    uint16_t value;
#ifdef USE_DSHOT
    uint16_t timerDmaSource;
    bool configured;
    uint8_t output;
    uint8_t index;
#ifdef USE_DSHOT_TELEMETRY
    bool useProshot;
    volatile bool isInput;
    volatile bool hasTelemetry;
    uint16_t dshotTelemetryValue;
    timeDelta_t dshotTelemetryDeadtimeUs;
    bool dshotTelemetryActive;
#ifdef USE_DSHOT_TELEMETRY_STATS
    dshotTelemetryQuality_t dshotTelemetryQuality;
#endif
#ifdef USE_HAL_DRIVER
    LL_TIM_OC_InitTypeDef ocInitStruct;
    LL_TIM_IC_InitTypeDef icInitStruct;
    LL_DMA_InitTypeDef    dmaInitStruct;
    uint32_t llChannel;
#else
    TIM_OCInitTypeDef ocInitStruct;
    TIM_ICInitTypeDef icInitStruct;
    DMA_InitTypeDef   dmaInitStruct;
#endif
    uint8_t dmaInputLen;
#endif
#ifdef STM32F3
    DMA_Channel_TypeDef *dmaRef;
#else
    DMA_Stream_TypeDef *dmaRef;
#endif
#endif
    motorDmaTimer_t *timer;
    volatile bool requestTelemetry;
#ifdef USE_DSHOT_TELEMETRY
    uint32_t dmaBuffer[DSHOT_TELEMETRY_INPUT_LEN];
#else
#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7)
    uint32_t dmaBuffer[DSHOT_DMA_BUFFER_SIZE];
#else
    uint8_t dmaBuffer[DSHOT_DMA_BUFFER_SIZE];
#endif
#endif
} motorDmaOutput_t;

motorDmaOutput_t *getMotorDmaOutput(uint8_t index);

struct timerHardware_s;
typedef void pwmWriteFn(uint8_t index, float value);  // function pointer used to write motors
typedef void pwmCompleteWriteFn(uint8_t motorCount);   // function pointer used after motors are written
typedef bool pwmStartWriteFn(uint8_t motorCount);   // function pointer used before motors are written

typedef struct {
    volatile timCCR_t *ccr;
    TIM_TypeDef       *tim;
} timerChannel_t;

typedef struct {
    timerChannel_t channel;
    float pulseScale;
    float pulseOffset;
    bool forceOverflow;
    bool enabled;
    IO_t io;
} pwmOutputPort_t;

//CAVEAT: This is used in the `motorConfig_t` parameter group, so the parameter group constraints apply
typedef struct motorDevConfig_s {
    uint16_t motorPwmRate;                  // The update rate of motor outputs (50-498Hz)
    uint8_t  motorPwmProtocol;              // Pwm Protocol
    uint8_t  motorPwmInversion;             // Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
    uint8_t  useUnsyncedPwm;
    uint8_t  useBurstDshot;
    uint8_t  useDshotTelemetry;
    ioTag_t  ioTags[MAX_SUPPORTED_MOTORS];
} motorDevConfig_t;

extern bool useBurstDshot;
#ifdef USE_DSHOT_TELEMETRY
extern bool useDshotTelemetry;
extern uint32_t dshotInvalidPacketCount;
#endif

void motorDevInit(const motorDevConfig_t *motorDevConfig, uint16_t idlePulse, uint8_t motorCount);

typedef struct servoDevConfig_s {
    // PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
    uint16_t servoCenterPulse;              // This is the value for servos when they should be in the middle. e.g. 1500.
    uint16_t servoPwmRate;                  // The update rate of servo outputs (50-498Hz)
    ioTag_t  ioTags[MAX_SUPPORTED_SERVOS];
} servoDevConfig_t;

void servoDevInit(const servoDevConfig_t *servoDevConfig);

void pwmServoConfig(const struct timerHardware_s *timerHardware, uint8_t servoIndex, uint16_t servoPwmRate, uint16_t servoCenterPulse);

bool isMotorProtocolDshot(void);

#ifdef USE_DSHOT
typedef uint8_t loadDmaBufferFn(uint32_t *dmaBuffer, int stride, uint16_t packet);  // function pointer used to encode a digital motor value into the DMA buffer representation

uint16_t prepareDshotPacket(motorDmaOutput_t *const motor);

extern loadDmaBufferFn *loadDmaBuffer;

uint32_t getDshotHz(motorPwmProtocolTypes_e pwmProtocolType);
void pwmWriteDshotCommandControl(uint8_t index);
void pwmWriteDshotCommand(uint8_t index, uint8_t motorCount, uint8_t command, bool blocking);
void pwmWriteDshotInt(uint8_t index, uint16_t value);
void pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output);
#ifdef USE_DSHOT_TELEMETRY
bool pwmStartDshotMotorUpdate(uint8_t motorCount);
#endif
void pwmCompleteDshotMotorUpdate(uint8_t motorCount);

bool pwmDshotCommandIsQueued(void);
bool pwmDshotCommandIsProcessing(void);
uint8_t pwmGetDshotCommand(uint8_t index);
bool pwmDshotCommandOutputIsEnabled(uint8_t motorCount);
uint16_t getDshotTelemetry(uint8_t index);
bool isDshotMotorTelemetryActive(uint8_t motorIndex);
void setDshotPidLoopTime(uint32_t pidLoopTime);
#ifdef USE_DSHOT_TELEMETRY_STATS
int16_t getDshotTelemetryMotorInvalidPercent(uint8_t motorIndex);
#endif

#endif

#ifdef USE_BEEPER
void pwmWriteBeeper(bool onoffBeep);
void pwmToggleBeeper(void);
void beeperPwmInit(const ioTag_t tag, uint16_t frequency);
#endif
void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion);

void pwmWriteMotor(uint8_t index, float value);
void pwmShutdownPulsesForAllMotors(uint8_t motorCount);
void pwmCompleteMotorUpdate(uint8_t motorCount);
bool pwmStartMotorUpdate(uint8_t motorCount);

void pwmWriteServo(uint8_t index, float value);

pwmOutputPort_t *pwmGetMotors(void);
bool pwmIsSynced(void);
void pwmDisableMotors(void);
void pwmEnableMotors(void);
bool pwmAreMotorsEnabled(void);
