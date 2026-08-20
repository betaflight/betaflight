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

// DShot motor output and telemetry for HPMicro.
// Motor values are fed to PWM compare registers by DMA; telemetry is captured
// via dual DMA channels (pos/neg edge) on GPTMR capture channels.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "drivers/dma.h"
#include "dma_hpmicro.h"
#include "drivers/nvic.h"
#include "nvic_hpmicro.h"
#include "drivers/io.h"
#include "io_hpmicro.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/system.h"
#include "timer_hpmicro.h"
#include "hpm_soc.h"
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_dmav2_drv.h"
#else
#include "hpm_dma_drv.h"
#endif
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_pwmv2_drv.h"
#else
#include "hpm_pwm_drv.h"
#endif
#include "hpm_clock_drv.h"
#include "drivers/dshot.h"
#include "dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "pwm_output_dshot_shared.h"
#include "hpm_trgm_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_gptmr_drv.h"

#include "timer_hw_ext.h"
#include "trgm_dshot_resource.h"
#include "hpm_dma_reqmap.h"

#define GPTMR_CAPTURE_IN_CH_POS     2
#define GPTMR_CAPTURE_IN_CH_NEG     3
#define DSHOT_SHADOW_CMP_INDEX      23

FAST_DATA_ZERO_INIT uint32_t dshot_duty_count;
static FAST_DATA_ZERO_INIT dma_channel_config_t dshotDmaConfig[MAX_SUPPORTED_MOTORS];
#ifdef USE_DSHOT_TELEMETRY
static FAST_DATA_ZERO_INIT dma_channel_config_t dshotCapPosEdgeConfig[MAX_SUPPORTED_MOTORS];
static FAST_DATA_ZERO_INIT dma_channel_config_t dshotCapNegEdgeConfig[MAX_SUPPORTED_MOTORS];
#endif

/* TRGM resources allocated per motor. */
static trgmDshotResource_t trgmDshotRes[MAX_SUPPORTED_MOTORS];

static FAST_CODE int gptmrIndexFromHwExt(const hpmicroTimerHwExt_t *hwExt)
{
    return trgmDshotGptmrIndexByGptmr((hwExt) ? hwExt->gptmr : NULL);
}

#if defined(HPMSOC_HAS_HPMSDK_PWM) && defined(HPMSOC_HAS_HPMSDK_DMA)
FAST_CODE void pwmDshotSetDirectionOutput(motorDmaOutput_t *const motor)
{
#ifdef USE_DSHOT_TELEMETRY
    motor->isInput = false;
#endif

    const timerHardware_t *timerHw = motor->timerHardware;
    const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(timerHw);
    trgmDshotResource_t *res = &trgmDshotRes[motor->index];

#ifdef HPM_USE_PWM_OUTPUT_DSHOT
    const IO_t motorIO = IOGetByTag(timerHw->tag);
#endif

    if (hwExt == NULL) {
        return;
    }

    int gptmrIndex = gptmrIndexFromHwExt(hwExt);

    if (gptmrIndex < 0) {
        return;
    }
    const dmaChannelSpec_t *capNegSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, false);
    const dmaChannelSpec_t *capPosSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, true);
    const dmaChannelSpec_t *outSpec = hpmDmaGetPwmOutChannelSpec(motor->index);

    if (capNegSpec == NULL || capPosSpec == NULL || outSpec == NULL) {
        return;
    }
    // Disable DSHOT input dma channels
    dma_disable_channel(capNegSpec->ref->base, capNegSpec->ref->channel);
    dma_disable_channel(capPosSpec->ref->base, capPosSpec->ref->channel);

    // Set IO to output and enable PWM channel
#ifdef HPM_USE_PWM_OUTPUT_DSHOT
    IOConfigGPIOAF(motorIO, IOCFG_AF_PP, timerHw->alternateFunction);
#else
    trgm_enable_io_output(trgmDshotResourceTrgm(res), 1 << (res->port_index));
#endif
    pwm_enable_output((PWM_Type *) timerHw->tim, timerHw->channel);

    // Map dma channel to output dma request
    dmamux_config(HPM_DMAMUX,
                  DMA_SOC_CHN_TO_DMAMUX_CHN(outSpec->ref->base, outSpec->ref->channel), hwExt->pwm_dmamux_src, true);

    // Clear input data.  Bound the clear by the actual row allocation: without
    // USE_DSHOT_TELEMETRY the telemetry buffers shrink to DSHOT_DMA_BUFFER_SIZE.
    memset(motor->dmaBuffer_neg_edge, 0, DSHOT_DMA_BUFFER_ALLOC_SIZE * sizeof(uint32_t));
    memset(motor->dmaBuffer_pos_edge, 0, DSHOT_DMA_BUFFER_ALLOC_SIZE * sizeof(uint32_t));
}


#ifdef USE_DSHOT_TELEMETRY
FAST_CODE static bool pwmDshotSetDirectionInput(motorDmaOutput_t *const motor)
{
    const timerHardware_t *timerHw = motor->timerHardware;
    const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(timerHw);
    trgmDshotResource_t *res = &trgmDshotRes[motor->index];

#ifdef HPM_USE_PWM_OUTPUT_DSHOT
    const IO_t motorIO = IOGetByTag(timerHw->tag);
#endif
    GPTMR_Type *gptmrBase = (hwExt) ? hwExt->gptmr : NULL;

    if (hwExt == NULL || gptmrBase == NULL) {
        return false;
    }

    int gptmrIndex = gptmrIndexFromHwExt(hwExt);

    if (gptmrIndex < 0) {
        return false;
    }
    const dmaChannelSpec_t *capNegSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, false);
    const dmaChannelSpec_t *capPosSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, true);

    if (capNegSpec == NULL || capPosSpec == NULL) {
        return false;
    }
    DMA_Type *dmaNeg = capNegSpec->ref->base;
    DMA_Type *dmaPos = capPosSpec->ref->base;

    dmamux_config(HPM_DMAMUX, DMA_SOC_CHN_TO_DMAMUX_CHN(dmaNeg, capNegSpec->ref->channel), capNegSpec->dmaMuxId, true);

    if (status_success != dma_setup_channel(dmaNeg,
                                            capNegSpec->ref->channel,
                                            &dshotCapNegEdgeConfig[motor->index], false)) {
        return false;
    }
    if (status_success != dma_setup_channel(dmaPos,
                                            capPosSpec->ref->channel,
                                            &dshotCapPosEdgeConfig[motor->index], false)) {
        dma_disable_channel(dmaNeg, capNegSpec->ref->channel);
        return false;
    }

    /* Switch the pin only after both capture channels are ready.  If setup
     * fails, leave the motor in output mode and skip telemetry for this frame. */
#ifdef HPM_USE_PWM_OUTPUT_DSHOT
    IOConfigGPIOAF(motorIO, IOCFG_IN_FLOATING, 0);
#else
    trgm_disable_io_output(trgmDshotResourceTrgm(res), 1 << (res->port_index));
#endif
    motor->isInput = true;

    if (!inputStampUs) {
        inputStampUs = micros();
    }

    gptmr_channel_reset_count(gptmrBase, GPTMR_CAPTURE_IN_CH_NEG);
    gptmr_channel_reset_count(gptmrBase, GPTMR_CAPTURE_IN_CH_POS);
    gptmr_start_counter(gptmrBase, GPTMR_CAPTURE_IN_CH_NEG);
    gptmr_start_counter(gptmrBase, GPTMR_CAPTURE_IN_CH_POS);


    gptmr_trigger_channel_software_sync(gptmrBase, 0xF);
    dma_enable_channel(dmaPos, capPosSpec->ref->channel);
    dma_enable_channel(dmaNeg, capNegSpec->ref->channel);

    return true;
}
#endif


FAST_CODE void pwmCompleteDshotMotorUpdate(void)
{
    // HPM starts each motor DMA from pwmWriteDshotInt().  Command timing is
    // therefore gated there, before the first motor transfer is started.
}

FAST_CODE static void motorDshotTransferDoneHandler(dmaChannelDescriptor_t *descriptor)
{
    motorDmaOutput_t *const motor = &dmaMotors[descriptor->userParam];

    if (hpmDmaGetChannelStatus(descriptor) & (DMA_CHANNEL_STATUS_ERROR | DMA_CHANNEL_STATUS_ABORT)) {
        const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(motor->timerHardware);
        if (hwExt) {
            pwm_disable_dma_request((PWM_Type *) motor->timerHardware->tim,
                                    PWM_DMAEN_CMPENX_SET(1 << hwExt->dma_req_cmp_index));
        }
        return;
    }
#ifdef USE_DSHOT_TELEMETRY
    if (!motor->isInput) {
        dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
        if (useDshotTelemetry && pwmDshotSetDirectionInput(motor)) {
            dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();
        }
    }
#endif
}

#ifdef USE_DSHOT_TELEMETRY

// Setup the DSHOT telemetry capture DMA channels
FAST_CODE static void setupDshotTelemetryDma(motorDmaOutput_t *const motor, motorProtocolTypes_e pwmProtocolType)
{
    const timerHardware_t *timerHardware = motor->timerHardware;
    const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(timerHardware);
    trgmDshotResource_t *res = &trgmDshotRes[motor->index];

    if (hwExt == NULL) {
        return;
    }
    // Source Info From fullTimerHardware in timer_hpm6750.c
    int gptmrIndex = gptmrIndexFromHwExt(hwExt);

    if (gptmrIndex < 0) {
        return;
    }
    const dmaChannelSpec_t *capNegSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, false);
    const dmaChannelSpec_t *capPosSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, true);

    if (capNegSpec == NULL || capPosSpec == NULL || res->gptmr == NULL) {
        return;
    }

    DMA_Type *dmaNeg = capNegSpec->ref->base;
    DMA_Type *dmaPos = capPosSpec->ref->base;
    GPTMR_Type *gptmr = res->gptmr;
    uint8_t motorIdx = motor->index;
    dma_channel_config_t *dmaConfig;

    gptmr_channel_config_t config;
    uint32_t gptmrFreq = clock_get_frequency(res->gptmr_clock);

    gptmr_channel_get_default_config(gptmr, &config);

    // dshot_reload_counter is used to decode dshot telemetry message.
    // Multiply before dividing with a 64-bit intermediate: dividing first
    // truncates (200MHz/12MHz=16 -> 256 instead of 266, a systematic ~4%
    // error that shifts the decoder's rounding boundary and rejects valid
    // packets), and gptmrFreq * MOTOR_BITLENGTH * 4 overflows uint32_t.
    dshotTelemetryBitWidth[motorIdx] = (uint32_t) (((uint64_t) gptmrFreq * MOTOR_BITLENGTH * 4) /
                                                    ((uint64_t) getDshotHz(pwmProtocolType) * 5));

    config.reload = gptmrFreq / 10 - 1;
    config.enable_software_sync = true;
    config.dma_request_event = gptmr_dma_request_on_input_signal_toggle;

    config.mode = gptmr_work_mode_capture_at_falling_edge;
    gptmr_channel_config(gptmr, GPTMR_CAPTURE_IN_CH_NEG, &config, false);
    config.mode = gptmr_work_mode_capture_at_rising_edge;
    gptmr_channel_config(gptmr, GPTMR_CAPTURE_IN_CH_POS, &config, false);

    // Setup dshotCapNegEdgeConfig and dshotCapPosEdgeConfig to decrease change dshot direction time cost
    dmaConfig = &dshotCapNegEdgeConfig[motorIdx];
    dma_default_channel_config(dmaNeg, dmaConfig);
    dmaConfig->src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    dmaConfig->src_width = DMA_TRANSFER_WIDTH_WORD;
    dmaConfig->src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    dmaConfig->src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    dmaConfig->dst_width = DMA_TRANSFER_WIDTH_WORD;
    dmaConfig->dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    dmaConfig->dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    dmaConfig->size_in_byte = MAX_GCR_EDGES * sizeof(uint32_t);
    dmaConfig->linked_ptr = 0;
    // Capture completion is polled by the decoder; only surface failures.
    dmaConfig->interrupt_mask = DMA_INTERRUPT_MASK_TERMINAL_COUNT;
    dmaConfig->dst_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t) motor->dmaBuffer_neg_edge);
    dmaConfig->src_addr = (uint32_t) &gptmr->CHANNEL[GPTMR_CAPTURE_IN_CH_NEG].CAPNEG;

    dmaConfig = &dshotCapPosEdgeConfig[motorIdx];
    dma_default_channel_config(dmaPos, dmaConfig);
    dmaConfig->src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    dmaConfig->src_width = DMA_TRANSFER_WIDTH_WORD;
    dmaConfig->src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    dmaConfig->src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    dmaConfig->dst_width = DMA_TRANSFER_WIDTH_WORD;
    dmaConfig->dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    dmaConfig->dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    dmaConfig->size_in_byte = MAX_GCR_EDGES * sizeof(uint32_t);
    dmaConfig->linked_ptr = 0;
    // Capture completion is polled by the decoder; only surface failures.
    dmaConfig->interrupt_mask = DMA_INTERRUPT_MASK_TERMINAL_COUNT;
    dmaConfig->dst_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t) motor->dmaBuffer_pos_edge);
    dmaConfig->src_addr = (uint32_t) &gptmr->CHANNEL[GPTMR_CAPTURE_IN_CH_POS].CAPPOS;

}
#endif

FAST_CODE void pwmDshotStartTransfer(motorDmaOutput_t *motor, uint32_t size)
{
    const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(motor->timerHardware);

    if (hwExt == NULL) {
        return;
    }

    const dmaChannelSpec_t *outSpec = hpmDmaGetPwmOutChannelSpec(motor->index);

    if (outSpec == NULL) {
        return;
    }

    DMA_Type *dmaBase = outSpec->ref->base;
    dma_channel_config_t *chConfig = &dshotDmaConfig[motor->index];

    chConfig->size_in_byte = size;

    // Disable the PWM DMA request before reconfiguring the channel so that
    // any pending request from the free-running timer does not trigger a
    // spurious transfer mid-setup.  The request is re-enabled after the
    // channel has been fully configured.
    pwm_disable_dma_request((PWM_Type *) motor->timerHardware->tim,
                            PWM_DMAEN_CMPENX_SET(1 << (hwExt->dma_req_cmp_index)));

    // RISC-V weak memory ordering: ensure CPU writes to the DMA buffer are
    // visible to the DMA engine before the channel is started.  "fence w, w"
    // only orders writes against writes; the channel start below is a device
    // (MMIO) store, so the barrier must also order against I/O.
    __asm__ volatile ("fence iorw, iorw":::"memory");

    dma_disable_channel(dmaBase, outSpec->ref->channel);
    if (status_success != dma_setup_channel(dmaBase, outSpec->ref->channel, chConfig, true)) {
        printf(" dma setup channel failed\n");
        failureMode(FAILURE_DEVELOPER);
    }
    // Re-enable the PWM DMA request.  The next free-running timer compare
    // match starts this motor's transfer.
    pwm_enable_dma_request((PWM_Type *) motor->timerHardware->tim,
                           PWM_DMAEN_CMPENX_SET(1 << hwExt->dma_req_cmp_index));
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex,
                                 motorProtocolTypes_e pwmProtocolType, uint8_t output)
{
    // Stagger reference compare events by motor while keeping the value
    // deterministic across motor-device reinitialization.
    const uint8_t cmpOffset = 3U * (motorIndex + 1U);

    const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(timerHardware);
    if (hwExt == NULL) {
        return false;
    }

    trgmDshotResource_t *res = &trgmDshotRes[motorIndex];

    int gptmrIndex = gptmrIndexFromHwExt(hwExt);

    if (gptmrIndex < 0) {
        return false;
    }
    const dmaChannelSpec_t *outSpec = hpmDmaGetPwmOutChannelSpec(motorIndex);
    const dmaChannelSpec_t *capNegSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, false);
    const dmaChannelSpec_t *capPosSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, true);

    if (outSpec == NULL || capNegSpec == NULL || capPosSpec == NULL) {
        return false;
    }

    dmaResource_t *dmaRef = outSpec->ref;
    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaRef);

    if (!dmaAllocate(dmaIdentifier, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
        return false;
    }

    const dmaIdentifier_e dmaIdentifierCapNeg = dmaGetIdentifier(capNegSpec->ref);

    if (!dmaAllocate(dmaIdentifierCapNeg, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
        hpmDmaRelease(dmaIdentifier);
        return false;
    }
    const dmaIdentifier_e dmaIdentifierCapPos = dmaGetIdentifier(capPosSpec->ref);

    if (!dmaAllocate(dmaIdentifierCapPos, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
        hpmDmaRelease(dmaIdentifierCapNeg);
        hpmDmaRelease(dmaIdentifier);
        return false;
    }
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        output ^= TIMER_OUTPUT_INVERTED;
    }
#endif
    motorDmaOutput_t *const motor = &dmaMotors[motorIndex];
    TIM_TypeDef *timer = (PWM_Type *) timerHardware->tim;

    /* Allocate TRGM pin and PWM reference source for this motor. */
    if (!trgmDshotResourceAlloc(timerHardware->tag, res)) {
        hpmDmaRelease(dmaIdentifierCapPos);
        hpmDmaRelease(dmaIdentifierCapNeg);
        hpmDmaRelease(dmaIdentifier);
        return false;
    }
    if (hwExt->pwm_ref_src != HPM_PWM_REF_SRC_NONE) {
        if (!trgmDshotResourceAllocPwmRef(res, hwExt->pwm_ref_src)) {
            trgmDshotResourceFree(res);
            hpmDmaRelease(dmaIdentifierCapPos);
            hpmDmaRelease(dmaIdentifierCapNeg);
            hpmDmaRelease(dmaIdentifier);
            return false;
        }
    }
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry && !trgmDshotResourceAllocInput(res, hwExt->gptmr)) {
        trgmDshotResourceFree(res);
        hpmDmaRelease(dmaIdentifierCapPos);
        hpmDmaRelease(dmaIdentifierCapNeg);
        hpmDmaRelease(dmaIdentifier);
        return false;
    }
#endif

    uint8_t timerIndex = getTimerIndex(timer);

    if (timerIndex >= MAX_DMA_TIMERS) {
        trgmDshotResourceFree(res);
        hpmDmaRelease(dmaIdentifierCapPos);
        hpmDmaRelease(dmaIdentifierCapNeg);
        hpmDmaRelease(dmaIdentifier);
        return false;
    }
    // Capture TC is masked in its channel config, but error/abort events still
    // need the shared controller IRQ enabled and dispatched.  Install handlers
    // only after every fallible resource allocation has succeeded.
    dmaSetHandler(dmaIdentifierCapNeg, NULL, NVIC_PRIO_DSHOT_DMA, motorIndex);
    dmaSetHandler(dmaIdentifierCapPos, NULL, NVIC_PRIO_DSHOT_DMA, motorIndex);

    dma_channel_config_t *chConfig = &dshotDmaConfig[motorIndex];
    const uint32_t pwmFreq = getPWMFre(timer);
    uint32_t reload = 0;

    reload = (float) pwmFreq / getDshotHz(pwmProtocolType) * MOTOR_BITLENGTH - 1;

    // dshot_duty_count is a single global shared by all motors via the
    // MOTOR_BIT_0 / MOTOR_BIT_1 macros.  The macros compute correct bit
    // timing only when every PWM timer is clocked at the same frequency.
    // Verify this invariant once at init time.
    {
        static uint32_t firstPwmFreq;
        if (firstPwmFreq == 0) {
            firstPwmFreq = pwmFreq;
        } else if (pwmFreq != firstPwmFreq) {
            failureMode(FAILURE_DEVELOPER);
        }
    }
    dshot_duty_count = reload;

    pwm_cmp_config_t cmpConfig[2] = { 0 };
    pwm_config_t pwmConfig = { 0 };

    // Boolean configureTimer is always true when different channels of the same timer are processed in sequence,
    // causing the timer and the associated DMA initialized more than once.
    // To fix this, getTimerIndex must be expanded to return if a new timer has been requested.
    // However, since the initialization is idempotent, it is left as is in a favor of flash space (for now).
    motor->timer = &dmaMotorTimers[timerIndex];
    motor->timer->trgmIndex = hwExt->pwm_trgm_index;
    motor->index = motorIndex;
    motor->timerHardware = timerHardware;
    DMA_Type *base = outSpec->ref->base;

    pwm_pair_config_t cmpPairConfig = { 0 };

    if (dmaMotorTimers[timerIndex].inited == false) {
        clock_add_to_group(timerRCC(timer), 0);
        pwm_stop_counter(timer);
        reload = (float) pwmFreq / getDshotHz(pwmProtocolType) * MOTOR_BITLENGTH - 1;
        /*
         * reload and start counter
         */
        pwm_set_reload(timer, 0, reload);
        pwm_set_start_count(timer, 0, 0);
    }
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        pwm_get_default_pwm_pair_config(timer, &cmpPairConfig);
        cmpPairConfig.pwm[0].invert_output = (output & TIMER_OUTPUT_INVERTED) ? false : true;
        cmpPairConfig.pwm[0].force_cmd_shadow_update_trigger = pwm_shadow_register_update_on_modify;
        cmpPairConfig.pwm[0].enable_output = true;
    } else {
        pwm_get_default_pwm_config(timer, &pwmConfig);
        pwmConfig.force_cmd_shadow_update_trigger = pwm_shadow_register_update_on_hw_event;
        pwmConfig.enable_output = true;
        pwmConfig.dead_zone_in_half_cycle = 0;
        pwmConfig.invert_output = (output & TIMER_OUTPUT_INVERTED) ? false : true;
    }
    cmpConfig[0].mode = pwm_cmp_mode_output_compare;
    cmpConfig[0].cmp = reload;
    cmpConfig[0].update_trigger = pwm_shadow_register_update_on_hw_event;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        if (status_success != pwm_setup_waveform_in_pair(timer,
                                                         timerHardware->channel,
                                                         &cmpPairConfig, hwExt->cmp_index, &cmpConfig[0], 1)) {
            printf("failed to setup waveform\n");
            failureMode(FAILURE_DEVELOPER);
        }
    } else {
        if (status_success != pwm_setup_waveform(timer,
                                                 timerHardware->channel,
                                                 &pwmConfig, hwExt->cmp_index, &cmpConfig[0], 1)) {
            printf("failed to setup waveform\n");
            failureMode(FAILURE_DEVELOPER);
        }
        cmpConfig[0].mode = pwm_cmp_mode_output_compare;
        cmpConfig[0].cmp = 1U + cmpOffset;
        cmpConfig[0].update_trigger = pwm_shadow_register_update_on_modify;
        pwmConfig.enable_output = false;
        /*
         * config pwm as output driven by cmp
         */
        if (status_success != pwm_setup_waveform(timer,
                                                 hwExt->channel_ref,
                                                 &pwmConfig, hwExt->dma_req_cmp_index, &cmpConfig[0], 1)) {
            printf("failed to setup waveform\n");
            failureMode(FAILURE_DEVELOPER);
        }
    }
    if (dmaMotorTimers[timerIndex].inited == false) {
        cmpConfig[0].mode = pwm_cmp_mode_output_compare;
        cmpConfig[0].cmp = reload - 2;
        cmpConfig[0].update_trigger = pwm_shadow_register_update_on_modify;
        pwm_load_cmp_shadow_on_match(timer, DSHOT_SHADOW_CMP_INDEX, &cmpConfig[0]);
        pwm_start_counter(timer);
        dmaMotorTimers[timerIndex].inited = true;
    }

    pwm_issue_shadow_register_lock_event(timer);

    /* PWM half reload generate dma request */
    trgm_dma_request_config(trgmDshotTrgmByIndex(hwExt->pwm_trgm_index),
                            hwExt->trgm_dma_group, hwExt->pwm_trgm_dma_src);
    /* dma request trigger dma channel x to work */
    pwm_enable_dma_request(timer, PWM_DMAEN_CMPENX_SET(1 << hwExt->dma_req_cmp_index));

    if (hwExt->pwm_ref_src != HPM_PWM_REF_SRC_NONE) {
        trgm_output_t trgmIoConfig = { 0 };

        trgmIoConfig.invert = 0;
        trgmIoConfig.type = trgm_output_same_as_input;
        trgmIoConfig.input = res->pwm_ref_src;
        trgm_output_config(trgmDshotResourceTrgm(res), res->trgm_p_dst, &trgmIoConfig);
    }

    motor->timerDmaSource = PWM_DMAEN_CMPENX_SET(1 << hwExt->dma_req_cmp_index);
    motor->timer->timerDmaSources &= ~motor->timerDmaSource;

    motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];
    motor->dmaBuffer_pos_edge = &dshot_telemetry_pos_buf[motorIndex][0];
    motor->dmaBuffer_neg_edge = &dshot_telemetry_neg_buf[motorIndex][0];

    motor->dmaRef = dmaRef;

#ifdef USE_DSHOT_TELEMETRY
    motor->dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 *
                                      (16 * MOTOR_BITLENGTH) / getDshotHz(pwmProtocolType);
    motor->timer->outputPeriod =
        ((pwmProtocolType == PWM_TYPE_PROSHOT1000) ? MOTOR_NIBBLE_LENGTH_PROSHOT : MOTOR_BITLENGTH) - 1;
#endif
    pwmDshotSetDirectionOutput(motor);

    // dmaSetHandler() also enables the channel TC interrupt and the IRQ at this priority
    dmaSetHandler(dmaIdentifier, motorDshotTransferDoneHandler, NVIC_PRIO_DSHOT_DMA, motor->index);

    const IO_t capPin = IOGetByTag(res->pin);

    IOConfigGPIOAF(capPin, IOCFG_OUT_PP_UP, res->ioc_function);
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        // avoid high line during startup to prevent bootloader activation

        clock_add_to_group(res->gptmr_clock, 0);
        setupDshotTelemetryDma(motor, pwmProtocolType);

        /* Route TRGMx_Py to GPTMRx_IN2/3 for telemetry input capture. */
        trgm_output_t trgmIoConfig = { 0 };

        trgmIoConfig.invert = 0;
        trgmIoConfig.type = trgm_output_same_as_input;
        trgmIoConfig.input = res->trgm_p_src;
        trgm_output_config(trgmDshotResourceTrgm(res), res->gptmr_in2_dst, &trgmIoConfig);
        memset(&trgmIoConfig, 0, sizeof(trgmIoConfig));
        trgmIoConfig.invert = 0;
        trgmIoConfig.type = trgm_output_same_as_input;
        trgmIoConfig.input = res->trgm_p_src;
        trgm_output_config(trgmDshotResourceTrgm(res), res->gptmr_in3_dst, &trgmIoConfig);

        dmamux_config(HPM_DMAMUX,
                      DMA_SOC_CHN_TO_DMAMUX_CHN(capPosSpec->ref->base, capPosSpec->ref->channel),
                      capPosSpec->dmaMuxId, true);
    }
#endif

    dma_default_channel_config(base, chConfig);
    chConfig->src_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t) motor->dmaBuffer);
    chConfig->dst_addr = (uint32_t) timerChCCR(motor->timerHardware);
    chConfig->src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    chConfig->src_width = DMA_TRANSFER_WIDTH_WORD;
    chConfig->src_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    chConfig->src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    chConfig->dst_width = DMA_TRANSFER_WIDTH_WORD;
    chConfig->dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    chConfig->dst_mode = DMA_HANDSHAKE_MODE_NORMAL;

    // Pad drive config re-applied by dshotPwmEnableMotors() on arming
    motor->iocfg = IOCFG_OUT_PP_UP;
    motor->configured = true;
    return true;
}

/*
 * Release the resources a successfully configured motor acquired in
 * pwmDshotMotorHardwareConfig(): the three DMA channels plus the TRGM pin and
 * its PWM-reference / GPTMR-input bindings.  A motor whose hardware config
 * failed part-way rolls its own partial allocations back internally, so
 * motor->configured marks a motor that fully owns resources.
 */
static void pwmDshotMotorHardwareRelease(motorDmaOutput_t *motor)
{
    if (!motor->configured) {
        return;
    }

    pwm_disable_dma_request((PWM_Type *) motor->timerHardware->tim, motor->timerDmaSource);
    pwm_disable_output((PWM_Type *) motor->timerHardware->tim, motor->timerHardware->channel);

    const dmaChannelSpec_t *outSpec = hpmDmaGetPwmOutChannelSpec(motor->index);
    if (outSpec != NULL) {
        dma_disable_channel(outSpec->ref->base, outSpec->ref->channel);
        hpmDmaRelease(dmaGetIdentifier(outSpec->ref));
    }

    const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(motor->timerHardware);
    const int gptmrIndex = gptmrIndexFromHwExt(hwExt);
    if (gptmrIndex >= 0) {
        const dmaChannelSpec_t *capNegSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, false);
        const dmaChannelSpec_t *capPosSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, true);
        if (capNegSpec != NULL) {
            dma_disable_channel(capNegSpec->ref->base, capNegSpec->ref->channel);
            hpmDmaRelease(dmaGetIdentifier(capNegSpec->ref));
        }
        if (capPosSpec != NULL) {
            dma_disable_channel(capPosSpec->ref->base, capPosSpec->ref->channel);
            hpmDmaRelease(dmaGetIdentifier(capPosSpec->ref));
        }
    }

    trgmDshotResourceFree(&trgmDshotRes[motor->index]);

    motor->configured = false;
}

/*
 * Roll back every motor initialized by pwmDshotMotorHardwareConfig() and reset
 * the shared per-timer bookkeeping.  Called when motor-device init fails
 * part-way (so earlier motors do not leak their statically tracked DMA
 * channels and TRGM pins until reboot) and at the start of a re-init attempt
 * (so the same resources can be acquired again without a reboot).
 *
 * Note: timer and IO pin ownership taken by timerAllocate()/IOInit() have no
 * release API, so those stay owned; only the DMA and TRGM pools are restored.
 */
void pwmDshotReleaseAllMotors(void)
{
    for (unsigned i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        pwmDshotMotorHardwareRelease(&dmaMotors[i]);
    }
    dmaMotorTimerCount = 0;
    memset(dmaMotorTimers, 0, sizeof(dmaMotorTimers));
}
#endif
#endif
