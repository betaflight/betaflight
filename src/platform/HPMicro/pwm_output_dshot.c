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

#ifdef USE_DSHOT_TELEMETRY
static FAST_CODE int gptmrIndexFromHwExt(const hpmicroTimerHwExt_t *hwExt)
{
    return trgmDshotGptmrIndexByGptmr((hwExt) ? hwExt->gptmr : NULL);
}
#endif

#if defined(HPMSOC_HAS_HPMSDK_PWM) && defined(HPMSOC_HAS_HPMSDK_DMA)
FAST_CODE void pwmDshotSetDirectionOutput(motorDmaOutput_t *const motor)
{
    const timerHardware_t *timerHw = motor->timerHardware;
    const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(timerHw);
    trgmDshotResource_t *res = &motor->trgmRes;

    if (hwExt == NULL) {
        return;
    }

    const dmaChannelSpec_t *outSpec = hpmDmaGetPwmOutChannelSpec(motor->index);
    if (outSpec == NULL) {
        return;
    }

#ifdef USE_DSHOT_TELEMETRY
    if (res->gptmr != NULL) {
        const int gptmrIndex = gptmrIndexFromHwExt(hwExt);
        if (gptmrIndex >= 0) {
            const dmaChannelSpec_t *capNegSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, false);
            const dmaChannelSpec_t *capPosSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, true);

            if (capNegSpec != NULL) {
                dma_disable_channel(capNegSpec->ref->base, capNegSpec->ref->channel);
            }
            if (capPosSpec != NULL) {
                dma_disable_channel(capPosSpec->ref->base, capPosSpec->ref->channel);
            }
        }
    }
#endif

    // Drive the shared motor pin from its PWM reference through TRGM.
    trgm_enable_io_output(trgmDshotResourceTrgm(res), 1 << (res->port_index));
    pwm_enable_output((PWM_Type *) timerHw->tim, timerHw->channel);

    // Map dma channel to output dma request
    dmamux_config(HPM_DMAMUX,
                  DMA_SOC_CHN_TO_DMAMUX_CHN(outSpec->ref->base, outSpec->ref->channel), hwExt->pwm_dmamux_src, true);

#ifdef USE_DSHOT_TELEMETRY
    motor->isInput = false;
    if (useDshotTelemetry) {
        memset(motor->dmaBuffer_neg_edge, 0, sizeof(dshot_telemetry_neg_buf[motor->index]));
        memset(motor->dmaBuffer_pos_edge, 0, sizeof(dshot_telemetry_pos_buf[motor->index]));
    }
#endif
}


#ifdef USE_DSHOT_TELEMETRY
FAST_CODE static bool pwmDshotSetDirectionInput(motorDmaOutput_t *const motor)
{
    const timerHardware_t *timerHw = motor->timerHardware;
    const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(timerHw);
    trgmDshotResource_t *res = &motor->trgmRes;

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
                                            &motor->capNegEdgeDmaConfig, false)) {
        return false;
    }
    if (status_success != dma_setup_channel(dmaPos,
                                            capPosSpec->ref->channel,
                                            &motor->capPosEdgeDmaConfig, false)) {
        dma_disable_channel(dmaNeg, capNegSpec->ref->channel);
        return false;
    }

    /* Switch the shared pin only after both capture channels are ready.  If
     * setup fails, leave it in output mode and skip telemetry for this frame. */
    trgm_disable_io_output(trgmDshotResourceTrgm(res), 1 << (res->port_index));
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
    trgmDshotResource_t *res = &motor->trgmRes;

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

    // Prepare the capture DMA configs to reduce direction-switch latency.
    dmaConfig = &motor->capNegEdgeDmaConfig;
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

    dmaConfig = &motor->capPosEdgeDmaConfig;
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
    dma_channel_config_t *chConfig = &motor->outputDmaConfig;

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

typedef struct pwmDshotHardwareConfig_s {
    const timerHardware_t *timerHardware;
    const hpmicroTimerHwExt_t *hwExt;
    const dmaChannelSpec_t *outSpec;
#ifdef USE_DSHOT_TELEMETRY
    const dmaChannelSpec_t *capNegSpec;
    const dmaChannelSpec_t *capPosSpec;
#endif
    dmaIdentifier_e outDmaIdentifier;
#ifdef USE_DSHOT_TELEMETRY
    dmaIdentifier_e capNegDmaIdentifier;
    dmaIdentifier_e capPosDmaIdentifier;
#endif
    motorDmaOutput_t *motor;
    PWM_Type *timer;
    uint8_t motorIndex;
    uint8_t timerIndex;
#ifdef USE_DSHOT_TELEMETRY
    bool telemetryEnabled;
#endif
} pwmDshotHardwareConfig_t;

static bool pwmDshotPrepareHardwareConfig(pwmDshotHardwareConfig_t *config,
                                          const timerHardware_t *timerHardware,
                                          uint8_t motorIndex)
{
    config->timerHardware = timerHardware;
    config->hwExt = hpmicroTimerHwExtByTimer(timerHardware);
    if (config->hwExt == NULL) {
        return false;
    }

    config->outSpec = hpmDmaGetPwmOutChannelSpec(motorIndex);
    if (config->outSpec == NULL) {
        return false;
    }

    config->motor = &dmaMotors[motorIndex];
    config->outDmaIdentifier = dmaGetIdentifier(config->outSpec->ref);
    config->timer = (PWM_Type *) timerHardware->tim;
    config->motorIndex = motorIndex;

#ifdef USE_DSHOT_TELEMETRY
    config->telemetryEnabled = useDshotTelemetry;
    if (config->telemetryEnabled) {
        const int gptmrIndex = gptmrIndexFromHwExt(config->hwExt);
        if (gptmrIndex < 0) {
            return false;
        }

        config->capNegSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, false);
        config->capPosSpec = hpmDmaGetGptmrCapChannelSpec((uint8_t) gptmrIndex, true);
        if (config->capNegSpec == NULL || config->capPosSpec == NULL) {
            return false;
        }

        config->capNegDmaIdentifier = dmaGetIdentifier(config->capNegSpec->ref);
        config->capPosDmaIdentifier = dmaGetIdentifier(config->capPosSpec->ref);
    }
#endif

    config->timerIndex = getTimerIndex(config->timer);
    if (config->timerIndex >= MAX_DMA_TIMERS) {
        // getTimerIndex() does not change timer bookkeeping on overflow.
        return false;
    }

    return true;
}

static bool pwmDshotAllocateDmaResources(const pwmDshotHardwareConfig_t *config, uint8_t reorderedMotorIndex)
{
    const uint8_t resourceIndex = RESOURCE_INDEX(reorderedMotorIndex);

    if (!dmaAllocate(config->outDmaIdentifier, OWNER_MOTOR, resourceIndex)) {
        return false;
    }

#ifdef USE_DSHOT_TELEMETRY
    if (!config->telemetryEnabled) {
        return true;
    }
    if (!dmaAllocate(config->capNegDmaIdentifier, OWNER_MOTOR, resourceIndex)) {
        hpmDmaRelease(config->outDmaIdentifier);
        return false;
    }
    if (!dmaAllocate(config->capPosDmaIdentifier, OWNER_MOTOR, resourceIndex)) {
        hpmDmaRelease(config->capNegDmaIdentifier);
        hpmDmaRelease(config->outDmaIdentifier);
        return false;
    }
#endif

    return true;
}

static void pwmDshotReleaseDmaResources(const pwmDshotHardwareConfig_t *config)
{
#ifdef USE_DSHOT_TELEMETRY
    if (config->telemetryEnabled) {
        hpmDmaRelease(config->capPosDmaIdentifier);
        hpmDmaRelease(config->capNegDmaIdentifier);
    }
#endif
    hpmDmaRelease(config->outDmaIdentifier);
}

static bool pwmDshotAllocateTrgmResources(const pwmDshotHardwareConfig_t *config)
{
    const hpmicroTimerHwExt_t *hwExt = config->hwExt;
    trgmDshotResource_t *res = &config->motor->trgmRes;

    if (!trgmDshotResourceAlloc(config->timerHardware->tag, res)) {
        return false;
    }
    if (!trgmDshotResourceAllocPwmRef(res, hwExt->pwm_ref_src)) {
        trgmDshotResourceFree(res);
        return false;
    }
#ifdef USE_DSHOT_TELEMETRY
    if (config->telemetryEnabled && !trgmDshotResourceAllocInput(res, hwExt->gptmr)) {
        trgmDshotResourceFree(res);
        return false;
    }
#endif

    return true;
}

static void pwmDshotConfigurePwm(pwmDshotHardwareConfig_t *config,
                                 motorProtocolTypes_e pwmProtocolType,
                                 uint8_t output)
{
    motorDmaOutput_t *motor = config->motor;
    PWM_Type *timer = config->timer;
    const hpmicroTimerHwExt_t *hwExt = config->hwExt;
    const uint32_t pwmFreq = getPWMFre(timer);
    const uint32_t reload = (float) pwmFreq / getDshotHz(pwmProtocolType) * MOTOR_BITLENGTH - 1;
    const bool initializeTimer = !dmaMotorTimers[config->timerIndex].inited;

    motor->timer = &dmaMotorTimers[config->timerIndex];
    motor->timer->trgmIndex = hwExt->pwm_trgm_index;
    motor->index = config->motorIndex;
    motor->timerHardware = config->timerHardware;
    motor->dshotDutyCount = reload;

    if (initializeTimer) {
        clock_add_to_group(timerRCC(timer), 0);
        pwm_stop_counter(timer);
        pwm_set_reload(timer, 0, reload);
        pwm_set_start_count(timer, 0, 0);
    }

    pwm_cmp_config_t cmpConfig = {
        .mode = pwm_cmp_mode_output_compare,
        .cmp = reload,
        .update_trigger = pwm_shadow_register_update_on_hw_event,
    };

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        pwm_pair_config_t pairConfig = { 0 };
        pwm_get_default_pwm_pair_config(timer, &pairConfig);
        pairConfig.pwm[0].invert_output = (output & TIMER_OUTPUT_INVERTED) ? false : true;
        pairConfig.pwm[0].force_cmd_shadow_update_trigger = pwm_shadow_register_update_on_modify;
        pairConfig.pwm[0].enable_output = true;

        if (status_success != pwm_setup_waveform_in_pair(timer, config->timerHardware->channel,
                                                         &pairConfig, hwExt->cmp_index, &cmpConfig, 1)) {
            printf("failed to setup waveform\n");
            failureMode(FAILURE_DEVELOPER);
        }
    } else {
        pwm_config_t pwmConfig = { 0 };
        pwm_get_default_pwm_config(timer, &pwmConfig);
        pwmConfig.force_cmd_shadow_update_trigger = pwm_shadow_register_update_on_hw_event;
        pwmConfig.enable_output = true;
        pwmConfig.dead_zone_in_half_cycle = 0;
        pwmConfig.invert_output = (output & TIMER_OUTPUT_INVERTED) ? false : true;

        if (status_success != pwm_setup_waveform(timer, config->timerHardware->channel,
                                                 &pwmConfig, hwExt->cmp_index, &cmpConfig, 1)) {
            printf("failed to setup waveform\n");
            failureMode(FAILURE_DEVELOPER);
        }

        // Stagger reference compare events by motor while keeping them stable
        // across motor-device reinitialization.
        cmpConfig.cmp = 1U + 3U * (config->motorIndex + 1U);
        cmpConfig.update_trigger = pwm_shadow_register_update_on_modify;
        pwmConfig.enable_output = false;
        if (status_success != pwm_setup_waveform(timer, hwExt->channel_ref,
                                                 &pwmConfig, hwExt->dma_req_cmp_index, &cmpConfig, 1)) {
            printf("failed to setup waveform\n");
            failureMode(FAILURE_DEVELOPER);
        }
    }

    if (initializeTimer) {
        cmpConfig.cmp = reload - 2;
        cmpConfig.update_trigger = pwm_shadow_register_update_on_modify;
        pwm_load_cmp_shadow_on_match(timer, DSHOT_SHADOW_CMP_INDEX, &cmpConfig);
        pwm_start_counter(timer);
        dmaMotorTimers[config->timerIndex].inited = true;
    }

    pwm_issue_shadow_register_lock_event(timer);
}

static void pwmDshotConfigureDmaRequest(const pwmDshotHardwareConfig_t *config)
{
    const hpmicroTimerHwExt_t *hwExt = config->hwExt;
    trgmDshotResource_t *res = &config->motor->trgmRes;

    trgm_dma_request_config(trgmDshotTrgmByIndex(hwExt->pwm_trgm_index),
                            hwExt->trgm_dma_group, hwExt->pwm_trgm_dma_src);
    pwm_enable_dma_request(config->timer, PWM_DMAEN_CMPENX_SET(1 << hwExt->dma_req_cmp_index));

    trgm_output_t trgmIoConfig = {
        .invert = 0,
        .type = trgm_output_same_as_input,
        .input = res->pwm_ref_src,
    };
    trgm_output_config(trgmDshotResourceTrgm(res), res->trgm_p_dst, &trgmIoConfig);
}

static void pwmDshotInitializeMotorState(const pwmDshotHardwareConfig_t *config,
                                         motorProtocolTypes_e pwmProtocolType)
{
    motorDmaOutput_t *motor = config->motor;

    motor->timerDmaSource = PWM_DMAEN_CMPENX_SET(1 << config->hwExt->dma_req_cmp_index);
    motor->timer->timerDmaSources &= ~motor->timerDmaSource;
    motor->dmaBuffer = &dshotDmaBuffer[config->motorIndex][0];
    motor->dmaRef = config->outSpec->ref;

#ifdef USE_DSHOT_TELEMETRY
    if (config->telemetryEnabled) {
        motor->dmaBuffer_pos_edge = &dshot_telemetry_pos_buf[config->motorIndex][0];
        motor->dmaBuffer_neg_edge = &dshot_telemetry_neg_buf[config->motorIndex][0];
        motor->dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 *
                                          (16 * MOTOR_BITLENGTH) / getDshotHz(pwmProtocolType);
        motor->timer->outputPeriod =
            ((pwmProtocolType == PWM_TYPE_PROSHOT1000) ? MOTOR_NIBBLE_LENGTH_PROSHOT : MOTOR_BITLENGTH) - 1;
    }
#else
    UNUSED(pwmProtocolType);
#endif

    pwmDshotSetDirectionOutput(motor);
    // dmaSetHandler() also enables the channel TC interrupt and the IRQ at this priority.
    dmaSetHandler(config->outDmaIdentifier, motorDshotTransferDoneHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
}

#ifdef USE_DSHOT_TELEMETRY
static void pwmDshotConfigureTelemetry(const pwmDshotHardwareConfig_t *config, motorProtocolTypes_e pwmProtocolType)
{
    if (!config->telemetryEnabled) {
        return;
    }

    trgmDshotResource_t *res = &config->motor->trgmRes;

    clock_add_to_group(res->gptmr_clock, 0);
    setupDshotTelemetryDma(config->motor, pwmProtocolType);

    /* Route TRGMx_Py to GPTMRx_IN2/3 for telemetry input capture. */
    trgm_output_t trgmIoConfig = {
        .invert = 0,
        .type = trgm_output_same_as_input,
        .input = res->trgm_p_src,
    };
    trgm_output_config(trgmDshotResourceTrgm(res), res->gptmr_in2_dst, &trgmIoConfig);
    trgm_output_config(trgmDshotResourceTrgm(res), res->gptmr_in3_dst, &trgmIoConfig);

    dmamux_config(HPM_DMAMUX,
                  DMA_SOC_CHN_TO_DMAMUX_CHN(config->capPosSpec->ref->base,
                                            config->capPosSpec->ref->channel),
                  config->capPosSpec->dmaMuxId, true);
}
#endif

static void pwmDshotPrepareOutputDmaConfig(const pwmDshotHardwareConfig_t *config)
{
    dma_channel_config_t *chConfig = &config->motor->outputDmaConfig;

    dma_default_channel_config(config->outSpec->ref->base, chConfig);
    chConfig->src_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t) config->motor->dmaBuffer);
    chConfig->dst_addr = (uint32_t) timerChCCR(config->timerHardware);
    chConfig->src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    chConfig->src_width = DMA_TRANSFER_WIDTH_WORD;
    chConfig->src_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    chConfig->src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    chConfig->dst_width = DMA_TRANSFER_WIDTH_WORD;
    chConfig->dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    chConfig->dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
}

// Allocate resources and configure the hardware for one DShot motor.
bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex,
                                 motorProtocolTypes_e pwmProtocolType, uint8_t output)
{
    pwmDshotHardwareConfig_t config = { 0 };

    // Resolve the timer, DMA mappings, and motor state used below.
    if (!pwmDshotPrepareHardwareConfig(&config, timerHardware, motorIndex)) {
        return false;
    }

    // Reserve DMA and TRGM resources before programming the hardware.
    if (!pwmDshotAllocateDmaResources(&config, reorderedMotorIndex)) {
        return false;
    }
    if (!pwmDshotAllocateTrgmResources(&config)) {
        pwmDshotReleaseDmaResources(&config);
        return false;
    }

#ifdef USE_DSHOT_TELEMETRY
    // Bidirectional DShot uses inverted output and capture DMA IRQ dispatch.
    if (config.telemetryEnabled) {
        output ^= TIMER_OUTPUT_INVERTED;

        // Capture TC is masked, but error/abort events still need controller IRQ dispatch.
        dmaSetHandler(config.capNegDmaIdentifier, NULL, NVIC_PRIO_DSHOT_DMA, motorIndex);
        dmaSetHandler(config.capPosDmaIdentifier, NULL, NVIC_PRIO_DSHOT_DMA, motorIndex);
    }
#endif

    // Program the signal path, initialize motor state, and prepare output DMA.
    pwmDshotConfigurePwm(&config, pwmProtocolType, output);
    pwmDshotConfigureDmaRequest(&config);
    pwmDshotInitializeMotorState(&config, pwmProtocolType);
#ifdef USE_DSHOT_TELEMETRY
    pwmDshotConfigureTelemetry(&config, pwmProtocolType);
#endif
    pwmDshotPrepareOutputDmaConfig(&config);

    // Mark resource ownership only after every configuration step succeeds.
    config.motor->configured = true;
    return true;
}

/*
 * Release the resources a successfully configured motor acquired in
 * pwmDshotMotorHardwareConfig(): the output DMA, optional telemetry capture
 * DMA channels, plus the TRGM pin and its PWM-reference / GPTMR-input bindings.
 * A motor whose hardware config failed part-way rolls its partial allocations
 * back internally, so motor->configured marks full resource ownership.
 */
static void pwmDshotMotorHardwareRelease(motorDmaOutput_t *motor)
{
    if (!motor->configured) {
        return;
    }

    trgmDshotResource_t *res = &motor->trgmRes;

    pwm_disable_dma_request((PWM_Type *) motor->timerHardware->tim, motor->timerDmaSource);
    pwm_disable_output((PWM_Type *) motor->timerHardware->tim, motor->timerHardware->channel);

    const dmaChannelSpec_t *outSpec = hpmDmaGetPwmOutChannelSpec(motor->index);
    if (outSpec != NULL) {
        dma_disable_channel(outSpec->ref->base, outSpec->ref->channel);
        hpmDmaRelease(dmaGetIdentifier(outSpec->ref));
    }

#ifdef USE_DSHOT_TELEMETRY
    const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(motor->timerHardware);
    if (res->gptmr != NULL) {
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
    }
#endif

    trgmDshotResourceFree(res);

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
