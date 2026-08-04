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

#ifdef USE_DSHOT_TELEMETRY

static void x32TimerCaptureBothEdges(TIM_TypeDef *timer, uint16_t channel)
{
    uint32_t polarityMask;

    switch (channel) {
    case TIM_CH_1:
        polarityMask = TIM_CCEN_CC1P | TIM_CCEN_CC1NP;
        break;
    case TIM_CH_2:
        polarityMask = TIM_CCEN_CC2P | TIM_CCEN_CC2NP;
        break;
    case TIM_CH_3:
        polarityMask = TIM_CCEN_CC3P | TIM_CCEN_CC3NP;
        break;
    case TIM_CH_4:
        polarityMask = TIM_CCEN_CC4P | TIM_CCEN_CC4NP;
        break;
    default:
        return;
    }

    timer->CCEN |= polarityMask;
}

void dshotEnableChannels(unsigned motorCount)
{
    for (unsigned i = 0; i < motorCount; i++) {
        timerChannelEnable(dmaMotors[i].timerHardware);
    }
}

#endif

static const dmaChannelSpec_t *x32DshotGetTimerUpdateDmaSpec(const timerHardware_t *timerHardware)
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

void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor
#ifndef USE_DSHOT_TELEMETRY
    , TIM_OCInitTypeDef *pOcInit, DMA_InitTypeDef *pDmaInit
#endif
)
{
#ifdef USE_DSHOT_TELEMETRY
    TIM_OCInitTypeDef *pOcInit = &motor->ocInitStruct;
    DMA_InitTypeDef *pDmaInit = &motor->dmaInitStruct;
#endif

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = (TIM_TypeDef *)timerHardware->tim;

    xDMA_DeInit(motor->dmaRef);

#ifdef USE_DSHOT_TELEMETRY
    motor->isInput = false;
#endif

    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OC_PRE_LOAD_DISABLE);
    timerOCInit(timer, timerHardware->channel, pOcInit);
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OC_PRE_LOAD_ENABLE);

#ifdef USE_DSHOT_TELEMETRY
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        pDmaInit->TfrTypeFlowCtrl = DMA_CH_TRANSFER_FLOW_M2P_DMA;
        pDmaInit->SrcAddr = (uint32_t)motor->timer->dmaBurstBuffer;
        pDmaInit->DstAddr = (uint32_t)&timer->DADDR;
        pDmaInit->BlkTfrSize = DSHOT_DMA_BUFFER_SIZE * 4U;
        pDmaInit->SrcAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
        pDmaInit->DstAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
        pDmaInit->SrcTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
        pDmaInit->DstTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
        pDmaInit->SrcHandshaking = DMA_CH_SRC_HANDSHAKING_SOFTWARE;
        pDmaInit->DstHandshaking = DMA_CH_DST_HANDSHAKING_HARDWARE;
        pDmaInit->DstHsInterface = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)motor->dmaRef);
        pDmaInit->DstHsInterfacePol = DMA_CH_HANDSHAKING_IF_POL_H;
    } else
#endif
    {
        pDmaInit->TfrTypeFlowCtrl = DMA_CH_TRANSFER_FLOW_M2P_DMA;
        pDmaInit->SrcAddr = (uint32_t)motor->dmaBuffer;
        pDmaInit->DstAddr = (uint32_t)timerChCCR(timerHardware);
        pDmaInit->BlkTfrSize = DSHOT_DMA_BUFFER_SIZE;
        pDmaInit->SrcAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
        pDmaInit->DstAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
        pDmaInit->SrcTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
        pDmaInit->DstTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
        pDmaInit->SrcHandshaking = DMA_CH_SRC_HANDSHAKING_SOFTWARE;
        pDmaInit->DstHandshaking = DMA_CH_DST_HANDSHAKING_HARDWARE;
        pDmaInit->DstHsInterface = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)motor->dmaRef);
        pDmaInit->DstHsInterfacePol = DMA_CH_HANDSHAKING_IF_POL_H;
    }
#endif

    xDMA_Init(motor->dmaRef, pDmaInit);
    xDMA_ITConfig(motor->dmaRef, DMA_IT_TCIF | DMA_IT_TEIF, ENABLE);
}

#ifdef USE_DSHOT_TELEMETRY
FAST_CODE static void pwmDshotSetDirectionInput(motorDmaOutput_t * const motor)
{
    DMA_InitTypeDef *pDmaInit = &motor->dmaInitStruct;

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = (TIM_TypeDef *)timerHardware->tim;

    xDMA_DeInit(motor->dmaRef);

    motor->isInput = true;
    if (!inputStampUs) {
        inputStampUs = micros();
    }

    TIM_ConfigArPreload(timer, ENABLE);
    TIM_SetAutoReload(timer, 0xffffffff);

    TIM_ICInit(timer, &motor->icInitStruct);
    x32TimerCaptureBothEdges(timer, timerHardware->channel);

    pDmaInit->TfrTypeFlowCtrl = DMA_CH_TRANSFER_FLOW_P2M_DMA;
    pDmaInit->SrcAddr = (uint32_t)timerChCCR(timerHardware);
    pDmaInit->DstAddr = (uint32_t)motor->dmaBuffer;
    pDmaInit->SrcAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
    pDmaInit->DstAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
    pDmaInit->SrcTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
    pDmaInit->DstTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
    pDmaInit->SrcHandshaking = DMA_CH_SRC_HANDSHAKING_HARDWARE;
    pDmaInit->DstHandshaking = DMA_CH_DST_HANDSHAKING_SOFTWARE;
    pDmaInit->SrcHsInterface = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)motor->dmaRef);
    pDmaInit->SrcHsInterfacePol = DMA_CH_HANDSHAKING_IF_POL_H;
    pDmaInit->BlkTfrSize = GCR_TELEMETRY_INPUT_LEN;

    xDMA_Init(motor->dmaRef, pDmaInit);
}
#endif

void pwmCompleteDshotMotorUpdate(void)
{
    if (!dshotCommandQueueEmpty() && !dshotCommandOutputIsEnabled(dshotMotorCount)) {
        return;
    }

    for (unsigned i = 0; i < dmaMotorTimerCount; i++) {
        TIM_TypeDef *timer = (TIM_TypeDef *)dmaMotorTimers[i].timer;

#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_Cmd(dmaMotorTimers[i].dmaBurstRef, DISABLE);
            xDMA_MemoryTargetConfig(dmaMotorTimers[i].dmaBurstRef, (uint32_t)dmaMotorTimers[i].dmaBurstBuffer, 0);
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
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF)) {
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
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF | DMA_IT_TCIF);
        return;
    }

    if (!DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        return;
    }

    motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
    TIM_TypeDef *timer = (TIM_TypeDef *)motor->timerHardware->tim;

#ifdef USE_DSHOT_TELEMETRY
    dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
#endif

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

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        pwmDshotSetDirectionInput(motor);
        xDMA_SetCurrDataCounter(motor->dmaRef, GCR_TELEMETRY_INPUT_LEN);
        xDMA_Cmd(motor->dmaRef, ENABLE);
        TIM_EnableDma(timer, motor->timerDmaSource, ENABLE);
        dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();
    }
#endif

    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF | DMA_IT_TEIF);
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorProtocolTypes_e pwmProtocolType, uint8_t output)
{
#ifdef USE_DSHOT_TELEMETRY
#define OCINIT motor->ocInitStruct
#define DMAINIT motor->dmaInitStruct
#else
    TIM_OCInitTypeDef ocInitStruct;
    DMA_InitTypeDef dmaInitStruct;
#define OCINIT ocInitStruct
#define DMAINIT dmaInitStruct
#endif

    const dmaChannelSpec_t *dmaSpec = NULL;
    dmaResource_t *dmaRef = NULL;
    uint32_t dmaMuxId = 0;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaSpec = x32DshotGetTimerUpdateDmaSpec(timerHardware);
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
        } else if (!dmaAllocate(dmaIdentifier, OWNER_TIMUP, timerNumber)) {
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

    uint8_t pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PULLDOWN : GPIO_PULLUP;
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        output ^= TIMER_OUTPUT_INVERTED;
    }
#endif

    motor->iocfg = IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH,
        pupMode);
    IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);

    const uint16_t outputPeriod = (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000 ? MOTOR_NIBBLE_LENGTH_PROSHOT : MOTOR_BITLENGTH) - 1U;
    motor->timer->outputPeriod = outputPeriod;

    if (configureTimer) {
        RCC_ClockCmd(timerRCC(timerHardware->tim), ENABLE);
        TIM_Enable(timer, DISABLE);
        timerReconfigureTimeBase(timerHardware, outputPeriod + 1U, getDshotHz(pwmProtocolType));
    }

    TIM_OCStructInit(&OCINIT);
    OCINIT.OCMode = TIM_OCMODE_PWM1;
    OCINIT.Pulse = 0;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        OCINIT.OutputNState = TIM_OUTPUT_NSTATE_ENABLE;
        OCINIT.OCNIdleState = TIM_OCN_IDLE_STATE_RESET;
        OCINIT.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCN_POLARITY_LOW : TIM_OCN_POLARITY_HIGH;
    } else {
        OCINIT.OutputState = TIM_OUTPUT_STATE_ENABLE;
        OCINIT.OCIdleState = TIM_OC_IDLE_STATE_SET;
        OCINIT.OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OC_POLARITY_LOW : TIM_OC_POLARITY_HIGH;
    }

#ifdef USE_DSHOT_TELEMETRY
    TIM_InitIcStruct(&motor->icInitStruct);
    motor->icInitStruct.Channel = timerHardware->channel;
    motor->icInitStruct.ICPolarity = TIM_IC_POLARITY_RISING;
    motor->icInitStruct.ICSelection = TIM_IC_SELECTION_DIRECTTI;
    motor->icInitStruct.ICPrescaler = TIM_IC_PSC_DIV1;
    motor->icInitStruct.ICFilter = 2;
#endif

    DMA_ChannelStructInit(&DMAINIT);
    DMAINIT.TfrType = DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK;
    DMAINIT.ChannelPriority = DMA_CH_PRIORITY_2;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;
        motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];


        DMAINIT.ChCtrl             = 0x0ULL;
        DMAINIT.SrcGatherCtrl      = 0x0U;
        DMAINIT.DstScatterCtrl     = 0x0U;
        DMAINIT.IntEn              = 0x1U;
        DMAINIT.SrcTfrWidth        = DMA_CH_TRANSFER_WIDTH_32;
        DMAINIT.DstTfrWidth        = DMA_CH_TRANSFER_WIDTH_32;
        DMAINIT.DstAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
        DMAINIT.SrcAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
        DMAINIT.DstBurstLen        = DMA_CH_BURST_LENGTH_1;
        DMAINIT.SrcBurstLen        = DMA_CH_BURST_LENGTH_1;
        DMAINIT.SrcGatherEn        = 0x0U;
        DMAINIT.DstScatterEn       = 0x0U;
        DMAINIT.TfrTypeFlowCtrl    = DMA_CH_TRANSFER_FLOW_M2P_DMA;
        DMAINIT.DstMasterSelect    = (uint64_t)DMA_CH_AHB_MASTER_1;
        DMAINIT.SrcMasterSelect    = (uint64_t)DMA_CH_AHB_MASTER_1;
        DMAINIT.BlkTfrSize         = DSHOT_DMA_BUFFER_SIZE * 4U;
        DMAINIT.SrcAddr            = (uint32_t)motor->timer->dmaBurstBuffer;
        DMAINIT.DstAddr            = (uint32_t)&timer->DADDR;
        DMAINIT.pLinkListItem      = NULL;
        DMAINIT.SrcGatherInterval  = 0x0U;
        DMAINIT.SrcGatherCount     = 0x0U;
        DMAINIT.DstScatterInterval = 0x0U;
        DMAINIT.DstScatterCount    = 0x0U;
        DMAINIT.TfrType            = DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK;
        DMAINIT.ChannelPriority    = DMA_CH_PRIORITY_2;
        DMAINIT.SrcHandshaking     = DMA_CH_SRC_HANDSHAKING_SOFTWARE;
        DMAINIT.SrcHsInterface     = DMA_CH_HARDWARE_HANDSHAKING_IF_0;
        DMAINIT.SrcHsInterfacePol  = DMA_CH_HANDSHAKING_IF_POL_H;
        DMAINIT.DstHandshaking     = DMA_CH_DST_HANDSHAKING_HARDWARE;
        DMAINIT.DstHsInterface     = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)dmaRef);
        DMAINIT.DstHsInterfacePol  = DMA_CH_HANDSHAKING_IF_POL_H;
    } else
#endif
    {
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        motor->timer->timerDmaSources &= ~motor->timerDmaSource;
        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];

        DMAINIT.ChCtrl             = 0x0ULL;
        DMAINIT.SrcGatherCtrl      = 0x0U;
        DMAINIT.DstScatterCtrl     = 0x0U;
        DMAINIT.IntEn              = 0x1U;
        DMAINIT.SrcTfrWidth        = DMA_CH_TRANSFER_WIDTH_32;
        DMAINIT.DstTfrWidth        = DMA_CH_TRANSFER_WIDTH_32;
        DMAINIT.DstAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
        DMAINIT.SrcAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
        DMAINIT.DstBurstLen        = DMA_CH_BURST_LENGTH_1;
        DMAINIT.SrcBurstLen        = DMA_CH_BURST_LENGTH_1;
        DMAINIT.SrcGatherEn        = 0x0U;
        DMAINIT.DstScatterEn       = 0x0U;
        DMAINIT.TfrTypeFlowCtrl    = DMA_CH_TRANSFER_FLOW_M2P_DMA;
        DMAINIT.DstMasterSelect    = (uint64_t)DMA_CH_AHB_MASTER_1;
        DMAINIT.SrcMasterSelect    = (uint64_t)DMA_CH_AHB_MASTER_1;
        DMAINIT.BlkTfrSize         = DSHOT_DMA_BUFFER_SIZE;
        DMAINIT.SrcAddr = (uint32_t)motor->dmaBuffer;
        DMAINIT.DstAddr = (uint32_t)timerChCCR(timerHardware);
        DMAINIT.pLinkListItem      = NULL;
        DMAINIT.SrcGatherInterval  = 0x0U;
        DMAINIT.SrcGatherCount     = 0x0U;
        DMAINIT.DstScatterInterval = 0x0U;
        DMAINIT.DstScatterCount    = 0x0U;
        DMAINIT.TfrType            = DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK;
        DMAINIT.ChannelPriority    = DMA_CH_PRIORITY_2;
        DMAINIT.SrcHandshaking     = DMA_CH_SRC_HANDSHAKING_SOFTWARE;
        DMAINIT.SrcHsInterface     = DMA_CH_HARDWARE_HANDSHAKING_IF_0;
        DMAINIT.SrcHsInterfacePol  = DMA_CH_HANDSHAKING_IF_POL_H;
        DMAINIT.DstHandshaking     = DMA_CH_DST_HANDSHAKING_HARDWARE;
        DMAINIT.DstHsInterface     = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)dmaRef);
        DMAINIT.DstHsInterfacePol  = DMA_CH_HANDSHAKING_IF_POL_H;
    }

    motor->dmaRef = dmaRef;

    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);

    if (!dmaIsConfigured) {
        dmaEnable(dmaIdentifier);
        dmaMuxEnable(dmaIdentifier, dmaMuxId);
    }

#ifdef USE_DSHOT_TELEMETRY
    motor->dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 *
        (16 * MOTOR_BITLENGTH) / getDshotHz(pwmProtocolType);
    motor->timer->outputPeriod = outputPeriod;
    motor->dmaInputLen = GCR_TELEMETRY_INPUT_LEN;
    pwmDshotSetDirectionOutput(motor);
#else
    pwmDshotSetDirectionOutput(motor, &OCINIT, &DMAINIT);
#endif

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

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        *timerChCCR(timerHardware) = 0xffff;
    }
#endif

    motor->configured = true;
    return true;
}

#endif
