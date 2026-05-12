#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"

#include "pg/timerup.h"

#include "platform/dma.h"
#include "platform/rcc.h"
#include "platform/timer.h"

#include "dshot_dpwm.h"
#include "pwm_output_dshot_shared.h"

static const dmaChannelSpec_t *X32DshotGetTimerUpdateDmaSpec(const timerHardware_t *timerHardware)
{
#if defined(USE_TIMER_UP_CONFIG)
    const int8_t timerNumber = timerGetTIMNumber(timerHardware);
    const int8_t timerIndex = timerGetIndexByNumber(timerNumber);

    if (timerNumber < 0 || timerIndex < 0) {
        return NULL;
    }

    return dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, (uint8_t)timerIndex, timerUpConfig(timerIndex)->dmaopt);
#else
    UNUSED(timerHardware);
    return NULL;
#endif
}

static void pwmDshotSetDirectionOutput(motorDmaOutput_t * const motor, TIM_OCInitTypeDef *pOcInit, DMA_InitTypeDef *pDmaInit)
{
    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = (TIM_TypeDef *)timerHardware->tim;

    xDMA_DeInit(motor->dmaRef);

    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OC_PRE_LOAD_DISABLE);
    timerOCInit(timer, timerHardware->channel, pOcInit);
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OC_PRE_LOAD_ENABLE);

    xDMA_Init(motor->dmaRef, pDmaInit);
    xDMA_ITConfig(motor->dmaRef, DMA_IT_TCIF | DMA_IT_TEIF, ENABLE);
}

void pwmCompleteDshotMotorUpdate(void)
{
    if (!dshotCommandQueueEmpty() && !dshotCommandOutputIsEnabled(dshotMotorCount)) {
        return;
    }

    for (unsigned i = 0; i < dmaMotorTimerCount; i++) {
        TIM_TypeDef *timer = (TIM_TypeDef *)dmaMotorTimers[i].timer;

#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_SetCurrDataCounter(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            xDMA_Cmd(dmaMotorTimers[i].dmaBurstRef, ENABLE);
            TIM_ConfigDma(timer, TIM_DMABASE_CAPCMPDAT1, TIM_DMABURST_LENGTH_4TRANSFERS);
            TIM_EnableDma(timer, TIM_DMA_UPDATE, ENABLE);
            continue;
        }
#endif

        TIM_ConfigArPreload(timer, DISABLE);
        TIM_SetAutoReload(timer, dmaMotorTimers[i].outputPeriod);
        TIM_ConfigArPreload(timer, ENABLE);
        TIM_SetCnt(timer, 0);
        TIM_EnableDma(timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
        dmaMotorTimers[i].timerDmaSources = 0;
    }
}

static FAST_CODE void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (!DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        return;
    }

    motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
    TIM_TypeDef *timer = (TIM_TypeDef *)motor->timerHardware->tim;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        xDMA_Cmd(motor->timer->dmaBurstRef, DISABLE);
        TIM_EnableDma(timer, TIM_DMA_UPDATE, DISABLE);
    } else
#endif
    {
        xDMA_Cmd(motor->dmaRef, DISABLE);
        TIM_EnableDma(timer, motor->timerDmaSource, DISABLE);
    }

    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF | DMA_IT_TEIF);
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorProtocolTypes_e pwmProtocolType, uint8_t output)
{
    TIM_OCInitTypeDef ocInitStruct;
    DMA_InitTypeDef dmaInitStruct;

    const dmaChannelSpec_t *dmaSpec = NULL;
    dmaResource_t *dmaRef = NULL;
    uint32_t dmaMuxId = 0;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaSpec = X32DshotGetTimerUpdateDmaSpec(timerHardware);
    } else
#endif
    {
        dmaSpec = dmaGetChannelSpecByTimer(timerHardware);
    }

    if (dmaSpec != NULL) {
        dmaRef = dmaSpec->ref;
        dmaMuxId = dmaSpec->dmaMuxId;
    }

    if (dmaRef == NULL) {
        return false;
    }

    const dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaRef);
    bool dmaIsConfigured = false;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        const resourceOwner_t *owner = dmaGetOwner(dmaIdentifier);
        const int8_t timerNumber = timerGetTIMNumber(timerHardware);

        if (owner->owner == OWNER_TIMUP && owner->index == timerNumber) {
            dmaIsConfigured = true;
        } else if (!dmaAllocate(dmaIdentifier, OWNER_TIMUP, timerNumber - 1)) {
            return false;
        }
    } else
#endif
    {
        if (!dmaAllocate(dmaIdentifier, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
            return false;
        }
    }

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    TIM_TypeDef *timer = (TIM_TypeDef *)timerHardware->tim;
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == (dmaMotorTimerCount - 1U));
    const IO_t motorIO = IOGetByTag(timerHardware->tag);

    motor->timer = &dmaMotorTimers[timerIndex];
    motor->index = motorIndex;
    motor->timerHardware = timerHardware;
    motor->output = output;

    motor->iocfg = IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SLEW_RATE_FAST,
        (output & TIMER_OUTPUT_INVERTED) ? GPIO_PULL_DOWN : GPIO_PULL_UP, 0x00);
    IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);

    const uint16_t outputPeriod = (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000 ? MOTOR_NIBBLE_LENGTH_PROSHOT : MOTOR_BITLENGTH) - 1U;
    motor->timer->outputPeriod = outputPeriod;

    if (configureTimer) {
        RCC_ClockCmd(timerRCC(timerHardware->tim), ENABLE);
        TIM_Enable(timer, DISABLE);
        timerReconfigureTimeBase(timerHardware, outputPeriod + 1U, getDshotHz(pwmProtocolType));
    }

    TIM_OCStructInit(&ocInitStruct);
    ocInitStruct.OCMode = TIM_OCMODE_PWM1;
    ocInitStruct.Pulse = 0;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        ocInitStruct.OutputNState = TIM_OUTPUT_NSTATE_ENABLE;
        ocInitStruct.OCNIdleState = TIM_OCN_IDLE_STATE_RESET;
        ocInitStruct.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCN_POLARITY_LOW : TIM_OCN_POLARITY_HIGH;
    } else {
        ocInitStruct.OutputState = TIM_OUTPUT_STATE_ENABLE;
        ocInitStruct.OCIdleState = TIM_OC_IDLE_STATE_SET;
        ocInitStruct.OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OC_POLARITY_LOW : TIM_OC_POLARITY_HIGH;
    }

    DMA_ChannelStructInit(&dmaInitStruct);
    dmaInitStruct.TfrType = DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK;
    dmaInitStruct.ChannelPriority = DMA_CH_PRIORITY_2;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;
        motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];

        dmaInitStruct.TfrTypeFlowCtrl = DMA_CH_TRANSFER_FLOW_M2P_DMA;
        dmaInitStruct.SrcAddr = (uint32_t)motor->timer->dmaBurstBuffer;
        dmaInitStruct.DstAddr = (uint32_t)&timer->DADDR;
        dmaInitStruct.SrcAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
        dmaInitStruct.DstAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
        dmaInitStruct.SrcTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
        dmaInitStruct.DstTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
        dmaInitStruct.BlkTfrSize = DSHOT_DMA_BUFFER_SIZE * 4U;
        dmaInitStruct.SrcHandshaking = DMA_CH_SRC_HANDSHAKING_SOFTWARE;
        dmaInitStruct.DstHandshaking = DMA_CH_DST_HANDSHAKING_HARDWARE;
        dmaInitStruct.DstHsInterface = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)dmaRef);
        dmaInitStruct.DstHsInterfacePol = DMA_CH_HANDSHAKING_IF_POL_H;
    } else
#endif
    {
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        motor->timer->timerDmaSources &= ~motor->timerDmaSource;
        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];

        dmaInitStruct.TfrTypeFlowCtrl = DMA_CH_TRANSFER_FLOW_M2P_DMA;
        dmaInitStruct.SrcAddr = (uint32_t)motor->dmaBuffer;
        dmaInitStruct.DstAddr = (uint32_t)timerChCCR(timerHardware);
        dmaInitStruct.SrcAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
        dmaInitStruct.DstAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
        dmaInitStruct.SrcTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
        dmaInitStruct.DstTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
        dmaInitStruct.BlkTfrSize = DSHOT_DMA_BUFFER_SIZE;
        dmaInitStruct.SrcHandshaking = DMA_CH_SRC_HANDSHAKING_SOFTWARE;
        dmaInitStruct.DstHandshaking = DMA_CH_DST_HANDSHAKING_HARDWARE;
        dmaInitStruct.DstHsInterface = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)dmaRef);
        dmaInitStruct.DstHsInterfacePol = DMA_CH_HANDSHAKING_IF_POL_H;
    }

    motor->dmaRef = dmaRef;

    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);

    if (!dmaIsConfigured) {
        dmaEnable(dmaIdentifier);
        dmaMuxEnable(dmaIdentifier, dmaMuxId);
    }

    pwmDshotSetDirectionOutput(motor, &ocInitStruct, &dmaInitStruct);

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        if (!dmaIsConfigured) {
            dmaSetHandler(dmaIdentifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
        }
    } else
#endif
    {
        dmaSetHandler(dmaIdentifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
    }

    timerChannelEnable(timerHardware);

    if (configureTimer) {
        TIM_ConfigArPreload(timer, ENABLE);
        timerStart(timerHardware);
    }

    motor->configured = true;
    return true;
}

#endif
