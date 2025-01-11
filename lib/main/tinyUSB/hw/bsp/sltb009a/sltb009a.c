/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Rafael Silva (@perigoso)
 * Copyright (c) 2021 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "../board_api.h"

#include "em_device.h"

/*--------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM                                        */
/*--------------------------------------------------------------------*/

#define LED_PORT              0     // A
#define LED_PIN_R             12    // 12
#define LED_PIN_B             13    // 13
#define LED_PIN_G             14    // 14
#define LED_STATE_ON          0     // active-low

#define BUTTON_PORT           3     // D
#define BUTTON_PIN            5     // 5
#define BUTTON_STATE_ACTIVE   0     // active-low

/*--------------------------------------------------------------------*/
/* Forward USB interrupt events to TinyUSB IRQ Handler                */
/*--------------------------------------------------------------------*/

void USB_IRQHandler(void)
{
  tud_int_handler(0);
}

/*--------------------------------------------------------------------*/
/* Fault Handlers                                                     */
/*--------------------------------------------------------------------*/

void HardFault_Handler(void)
{
  asm("bkpt");
}

void MemManage_Handler(void)
{
  asm("bkpt");
}

void BusFault_Handler(void)
{
  asm("bkpt");
}

void UsageFault_Handler(void)
{
  asm("bkpt");
}

/*--------------------------------------------------------------------*/
/* Startup                                                            */
/*--------------------------------------------------------------------*/

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}

/*--------------------------------------------------------------------*/
/* Initing Funcs                                                      */
/*--------------------------------------------------------------------*/

void emu_init(uint8_t immediate_switch)
{
  EMU->PWRCTRL = (immediate_switch ? EMU_PWRCTRL_IMMEDIATEPWRSWITCH : 0) | EMU_PWRCTRL_REGPWRSEL_DVDD | EMU_PWRCTRL_ANASW_AVDD;
}

void emu_reg_init(float target_voltage)
{
    if(target_voltage < 2300.f || target_voltage >= 3800.f)
        return;

    uint8_t level = ((target_voltage - 2300.f) / 100.f);

    EMU->R5VCTRL = EMU_R5VCTRL_INPUTMODE_AUTO;

    EMU->R5VOUTLEVEL = level; /* Reg output to 3.3V*/
}

void emu_dcdc_init(float target_voltage, float max_ln_current, float max_lp_current, float max_reverse_current)
{
    if(target_voltage < 1800.f || target_voltage >= 3000.f)
        return;

    if(max_ln_current <= 0.f || max_ln_current > 200.f)
        return;

    if(max_lp_current <= 0.f || max_lp_current > 10000.f)
        return;

    if(max_reverse_current < 0.f || max_reverse_current > 160.f)
        return;

    // Low Power & Low Noise current limit
    uint8_t lp_bias = 0;

    if(max_lp_current < 75.f)
        lp_bias = 0;
    else if(max_lp_current < 500.f)
        lp_bias = 1;
    else if(max_lp_current < 2500.f)
        lp_bias = 2;
    else
        lp_bias = 3;

    EMU->DCDCMISCCTRL = (EMU->DCDCMISCCTRL & ~_EMU_DCDCMISCCTRL_LPCMPBIASEM234H_MASK) | ((uint32_t)lp_bias << _EMU_DCDCMISCCTRL_LPCMPBIASEM234H_SHIFT);
    EMU->DCDCMISCCTRL |= EMU_DCDCMISCCTRL_LNFORCECCM; // Force CCM to prevent reverse current
    EMU->DCDCLPCTRL |= EMU_DCDCLPCTRL_LPVREFDUTYEN; // Enable duty cycling of the bias for LP mode
    EMU->DCDCLNFREQCTRL = (EMU->DCDCLNFREQCTRL & ~_EMU_DCDCLNFREQCTRL_RCOBAND_MASK) | 4; // Set RCO Band to 7MHz

    uint8_t fet_count = 0;

    if(max_ln_current < 20.f)
        fet_count = 4;
    else if(max_ln_current >= 20.f && max_ln_current < 40.f)
        fet_count = 8;
    else
        fet_count = 16;

    EMU->DCDCMISCCTRL = (EMU->DCDCMISCCTRL & ~_EMU_DCDCMISCCTRL_NFETCNT_MASK) | ((uint32_t)(fet_count - 1) << _EMU_DCDCMISCCTRL_NFETCNT_SHIFT);
    EMU->DCDCMISCCTRL = (EMU->DCDCMISCCTRL & ~_EMU_DCDCMISCCTRL_PFETCNT_MASK) | ((uint32_t)(fet_count - 1) << _EMU_DCDCMISCCTRL_PFETCNT_SHIFT);

    uint8_t ln_current_limit = (((max_ln_current + 40.f) * 1.5f) / (5.f * fet_count)) - 1;
    uint8_t lp_current_limit = 1; // Recommended value

    EMU->DCDCMISCCTRL = (EMU->DCDCMISCCTRL & ~(_EMU_DCDCMISCCTRL_LNCLIMILIMSEL_MASK | _EMU_DCDCMISCCTRL_LPCLIMILIMSEL_MASK)) | ((uint32_t)ln_current_limit << _EMU_DCDCMISCCTRL_LNCLIMILIMSEL_SHIFT) | ((uint32_t)lp_current_limit << _EMU_DCDCMISCCTRL_LPCLIMILIMSEL_SHIFT);

    uint8_t z_det_limit = ((max_reverse_current + 40.f) * 1.5f) / (2.5f * fet_count);

    EMU->DCDCZDETCTRL = (EMU->DCDCZDETCTRL & ~_EMU_DCDCZDETCTRL_ZDETILIMSEL_MASK) | ((uint32_t)z_det_limit << _EMU_DCDCZDETCTRL_ZDETILIMSEL_SHIFT);

    EMU->DCDCCLIMCTRL |= EMU_DCDCCLIMCTRL_BYPLIMEN; // Enable bypass current limiter to prevent overcurrent when switching modes

    // Output Voltage
    if(target_voltage > 1800.f)
    {
        float max_vout = 3000.f;
        float min_vout = 1800.f;
        float diff_vout = max_vout - min_vout;

        uint8_t ln_vref_high = (DEVINFO->DCDCLNVCTRL0 & _DEVINFO_DCDCLNVCTRL0_3V0LNATT1_MASK) >> _DEVINFO_DCDCLNVCTRL0_3V0LNATT1_SHIFT;
        uint8_t ln_vref_low = (DEVINFO->DCDCLNVCTRL0 & _DEVINFO_DCDCLNVCTRL0_1V8LNATT1_MASK) >> _DEVINFO_DCDCLNVCTRL0_1V8LNATT1_SHIFT;

        uint8_t ln_vref = ((target_voltage - min_vout) * (float)(ln_vref_high - ln_vref_low)) / diff_vout;
        ln_vref += ln_vref_low;

        EMU->DCDCLNVCTRL = (ln_vref << _EMU_DCDCLNVCTRL_LNVREF_SHIFT) | EMU_DCDCLNVCTRL_LNATT;

        uint8_t lp_vref_low = 0;
        uint8_t lp_vref_high = 0;

        switch(lp_bias)
        {
            case 0:
            {
                lp_vref_high = (DEVINFO->DCDCLPVCTRL2 & _DEVINFO_DCDCLPVCTRL2_3V0LPATT1LPCMPBIAS0_MASK) >> _DEVINFO_DCDCLPVCTRL2_3V0LPATT1LPCMPBIAS0_SHIFT;
                lp_vref_low = (DEVINFO->DCDCLPVCTRL2 & _DEVINFO_DCDCLPVCTRL2_1V8LPATT1LPCMPBIAS0_MASK) >> _DEVINFO_DCDCLPVCTRL2_1V8LPATT1LPCMPBIAS0_SHIFT;
            }
            break;
            case 1:
            {
                lp_vref_high = (DEVINFO->DCDCLPVCTRL2 & _DEVINFO_DCDCLPVCTRL2_3V0LPATT1LPCMPBIAS1_MASK) >> _DEVINFO_DCDCLPVCTRL2_3V0LPATT1LPCMPBIAS1_SHIFT;
                lp_vref_low = (DEVINFO->DCDCLPVCTRL2 & _DEVINFO_DCDCLPVCTRL2_1V8LPATT1LPCMPBIAS1_MASK) >> _DEVINFO_DCDCLPVCTRL2_1V8LPATT1LPCMPBIAS1_SHIFT;
            }
            break;
            case 2:
            {
                lp_vref_high = (DEVINFO->DCDCLPVCTRL3 & _DEVINFO_DCDCLPVCTRL3_3V0LPATT1LPCMPBIAS2_MASK) >> _DEVINFO_DCDCLPVCTRL3_3V0LPATT1LPCMPBIAS2_SHIFT;
                lp_vref_low = (DEVINFO->DCDCLPVCTRL3 & _DEVINFO_DCDCLPVCTRL3_1V8LPATT1LPCMPBIAS2_MASK) >> _DEVINFO_DCDCLPVCTRL3_1V8LPATT1LPCMPBIAS2_SHIFT;
            }
            break;
            case 3:
            {
                lp_vref_high = (DEVINFO->DCDCLPVCTRL3 & _DEVINFO_DCDCLPVCTRL3_3V0LPATT1LPCMPBIAS3_MASK) >> _DEVINFO_DCDCLPVCTRL3_3V0LPATT1LPCMPBIAS3_SHIFT;
                lp_vref_low = (DEVINFO->DCDCLPVCTRL3 & _DEVINFO_DCDCLPVCTRL3_1V8LPATT1LPCMPBIAS3_MASK) >> _DEVINFO_DCDCLPVCTRL3_1V8LPATT1LPCMPBIAS3_SHIFT;
            }
            break;
        }

        uint8_t lp_vref = ((target_voltage - min_vout) * (float)(lp_vref_high - lp_vref_low)) / diff_vout;
        lp_vref += lp_vref_low;

        EMU->DCDCLPVCTRL = (lp_vref << _EMU_DCDCLPVCTRL_LPVREF_SHIFT) | EMU_DCDCLPVCTRL_LPATT;
    }
    else
    {
        float max_vout = 1800.f;
        float min_vout = 1200.f;
        float diff_vout = max_vout - min_vout;

        uint8_t ln_vref_high = (DEVINFO->DCDCLNVCTRL0 & _DEVINFO_DCDCLNVCTRL0_1V8LNATT0_MASK) >> _DEVINFO_DCDCLNVCTRL0_1V8LNATT0_SHIFT;
        uint8_t ln_vref_low = (DEVINFO->DCDCLNVCTRL0 & _DEVINFO_DCDCLNVCTRL0_1V2LNATT0_MASK) >> _DEVINFO_DCDCLNVCTRL0_1V2LNATT0_SHIFT;

        uint8_t ln_vref = ((target_voltage - min_vout) * (float)(ln_vref_high - ln_vref_low)) / diff_vout;
        ln_vref += ln_vref_low;

        EMU->DCDCLNVCTRL = ln_vref << _EMU_DCDCLNVCTRL_LNVREF_SHIFT;

        uint8_t lp_vref_low = 0;
        uint8_t lp_vref_high = 0;

        switch(lp_bias)
        {
            case 0:
            {
                lp_vref_high = (DEVINFO->DCDCLPVCTRL0 & _DEVINFO_DCDCLPVCTRL2_3V0LPATT1LPCMPBIAS0_MASK) >> _DEVINFO_DCDCLPVCTRL2_3V0LPATT1LPCMPBIAS0_SHIFT;
                lp_vref_low = (DEVINFO->DCDCLPVCTRL0 & _DEVINFO_DCDCLPVCTRL2_1V8LPATT1LPCMPBIAS0_MASK) >> _DEVINFO_DCDCLPVCTRL2_1V8LPATT1LPCMPBIAS0_SHIFT;
            }
            break;
            case 1:
            {
                lp_vref_high = (DEVINFO->DCDCLPVCTRL0 & _DEVINFO_DCDCLPVCTRL2_3V0LPATT1LPCMPBIAS1_MASK) >> _DEVINFO_DCDCLPVCTRL2_3V0LPATT1LPCMPBIAS1_SHIFT;
                lp_vref_low = (DEVINFO->DCDCLPVCTRL0 & _DEVINFO_DCDCLPVCTRL2_1V8LPATT1LPCMPBIAS1_MASK) >> _DEVINFO_DCDCLPVCTRL2_1V8LPATT1LPCMPBIAS1_SHIFT;
            }
            break;
            case 2:
            {
                lp_vref_high = (DEVINFO->DCDCLPVCTRL1 & _DEVINFO_DCDCLPVCTRL3_3V0LPATT1LPCMPBIAS2_MASK) >> _DEVINFO_DCDCLPVCTRL3_3V0LPATT1LPCMPBIAS2_SHIFT;
                lp_vref_low = (DEVINFO->DCDCLPVCTRL1 & _DEVINFO_DCDCLPVCTRL3_1V8LPATT1LPCMPBIAS2_MASK) >> _DEVINFO_DCDCLPVCTRL3_1V8LPATT1LPCMPBIAS2_SHIFT;
            }
            break;
            case 3:
            {
                lp_vref_high = (DEVINFO->DCDCLPVCTRL1 & _DEVINFO_DCDCLPVCTRL3_3V0LPATT1LPCMPBIAS3_MASK) >> _DEVINFO_DCDCLPVCTRL3_3V0LPATT1LPCMPBIAS3_SHIFT;
                lp_vref_low = (DEVINFO->DCDCLPVCTRL1 & _DEVINFO_DCDCLPVCTRL3_1V8LPATT1LPCMPBIAS3_MASK) >> _DEVINFO_DCDCLPVCTRL3_1V8LPATT1LPCMPBIAS3_SHIFT;
            }
            break;
        }

        uint8_t lp_vref = ((target_voltage - min_vout) * (float)(lp_vref_high - lp_vref_low)) / diff_vout;
        lp_vref += lp_vref_low;

        EMU->DCDCLPVCTRL = lp_vref << _EMU_DCDCLPVCTRL_LPVREF_SHIFT;
    }

    EMU->DCDCLPCTRL = (EMU->DCDCLPCTRL & ~_EMU_DCDCLPCTRL_LPCMPHYSSELEM234H_MASK) | (((DEVINFO->DCDCLPCMPHYSSEL1 & (((uint32_t)0xFF) << (lp_bias * 8))) >> (lp_bias * 8)) << _EMU_DCDCLPCTRL_LPCMPHYSSELEM234H_SHIFT);

    while(EMU->DCDCSYNC & EMU_DCDCSYNC_DCDCCTRLBUSY); // Wait for configuration to write

    // Calibration
    //EMU->DCDCLNCOMPCTRL = 0x57204077; // Compensation for 1uF DCDC capacitor
    EMU->DCDCLNCOMPCTRL = 0xB7102137; // Compensation for 4.7uF DCDC capacitor

    // Enable DCDC converter
    EMU->DCDCCTRL = EMU_DCDCCTRL_DCDCMODEEM4_EM4LOWPOWER | EMU_DCDCCTRL_DCDCMODEEM23_EM23LOWPOWER | EMU_DCDCCTRL_DCDCMODE_LOWNOISE;

    // Switch digital domain to DVDD
    EMU->PWRCTRL = EMU_PWRCTRL_REGPWRSEL_DVDD | EMU_PWRCTRL_ANASW_AVDD;
}

void cmu_hfxo_startup_calib(uint16_t ib_trim, uint16_t c_tune)
{
  if(CMU->STATUS & CMU_STATUS_HFXOENS)
      return;

  CMU->HFXOSTARTUPCTRL = (CMU->HFXOSTARTUPCTRL & ~(_CMU_HFXOSTARTUPCTRL_CTUNE_MASK | _CMU_HFXOSTARTUPCTRL_IBTRIMXOCORE_MASK)) | (((uint32_t)c_tune << _CMU_HFXOSTARTUPCTRL_CTUNE_SHIFT) & _CMU_HFXOSTARTUPCTRL_CTUNE_MASK) | (((uint32_t)ib_trim << _CMU_HFXOSTARTUPCTRL_IBTRIMXOCORE_SHIFT) & _CMU_HFXOSTARTUPCTRL_IBTRIMXOCORE_MASK);
}

void cmu_hfxo_steady_calib(uint16_t ib_trim, uint16_t c_tune)
{
  if(CMU->STATUS & CMU_STATUS_HFXOENS)
      return;

  CMU->HFXOSTEADYSTATECTRL = (CMU->HFXOSTEADYSTATECTRL & ~(_CMU_HFXOSTEADYSTATECTRL_CTUNE_MASK | _CMU_HFXOSTEADYSTATECTRL_IBTRIMXOCORE_MASK)) | (((uint32_t)c_tune << _CMU_HFXOSTEADYSTATECTRL_CTUNE_SHIFT) & _CMU_HFXOSTEADYSTATECTRL_CTUNE_MASK) | (((uint32_t)ib_trim << _CMU_HFXOSTEADYSTATECTRL_IBTRIMXOCORE_SHIFT) & _CMU_HFXOSTEADYSTATECTRL_IBTRIMXOCORE_MASK);
}

void cmu_hfrco_calib(uint32_t calibration)
{
    if(CMU->STATUS & CMU_STATUS_DPLLENS)
        return;

    while(CMU->SYNCBUSY & CMU_SYNCBUSY_HFRCOBSY);

    CMU->HFRCOCTRL = calibration;

    while(CMU->SYNCBUSY & CMU_SYNCBUSY_HFRCOBSY);
}

void cmu_ushfrco_calib(uint8_t enable, uint32_t calibration)
{
    if(CMU->USBCRCTRL & CMU_USBCRCTRL_USBCREN)
        return;

    if(!enable)
    {
        CMU->OSCENCMD = CMU_OSCENCMD_USHFRCODIS;
        while(CMU->STATUS & CMU_STATUS_USHFRCOENS);

        return;
    }

    while(CMU->SYNCBUSY & CMU_SYNCBUSY_USHFRCOBSY);

    CMU->USHFRCOCTRL = calibration | CMU_USHFRCOCTRL_FINETUNINGEN;

    while(CMU->SYNCBUSY & CMU_SYNCBUSY_USHFRCOBSY);

    if(enable && !(CMU->STATUS & CMU_STATUS_USHFRCOENS))
    {
        CMU->OSCENCMD = CMU_OSCENCMD_USHFRCOEN;

        while(!(CMU->STATUS & CMU_STATUS_USHFRCORDY));
    }
}

void cmu_auxhfrco_calib(uint8_t enable, uint32_t calibration)
{
    if(!enable)
    {
        CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCODIS;
        while(CMU->STATUS & CMU_STATUS_AUXHFRCOENS);

        return;
    }

    while(CMU->SYNCBUSY & CMU_SYNCBUSY_AUXHFRCOBSY);

    CMU->AUXHFRCOCTRL = calibration;

    while(CMU->SYNCBUSY & CMU_SYNCBUSY_AUXHFRCOBSY);

    if(enable && !(CMU->STATUS & CMU_STATUS_AUXHFRCOENS))
    {
        CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

        while(!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));
    }
}


void cmu_init(void)
{
    // Change SDIO clock to HFXO if HFRCO selected and disable it
    CMU->SDIOCTRL = CMU_SDIOCTRL_SDIOCLKDIS | CMU_SDIOCTRL_SDIOCLKSEL_HFXO;
    while(CMU->STATUS & CMU_STATUS_SDIOCLKENS);

    // Change QSPI clock to HFXO if HFRCO selected and disable it
    CMU->QSPICTRL = CMU_QSPICTRL_QSPI0CLKDIS | CMU_QSPICTRL_QSPI0CLKSEL_HFXO;
    while(CMU->STATUS & CMU_STATUS_QSPI0CLKENS);

    // Disable DPLL if enabled
    if(CMU->STATUS & CMU_STATUS_DPLLENS)
    {
        CMU->OSCENCMD = CMU_OSCENCMD_DPLLDIS;
        while(CMU->STATUS & CMU_STATUS_DPLLENS);
    }

    // Disable HFXO if enabled
    if(CMU->STATUS & CMU_STATUS_HFXOENS)
    {
        CMU->OSCENCMD = CMU_OSCENCMD_HFXODIS;
        while(CMU->STATUS & CMU_STATUS_HFXOENS);
    }

    // Setup HFXO
    CMU->HFXOCTRL = CMU_HFXOCTRL_PEAKDETMODE_AUTOCMD | CMU_HFXOCTRL_MODE_XTAL;
    CMU->HFXOCTRL1 = CMU_HFXOCTRL1_PEAKDETTHR_DEFAULT;
    CMU->HFXOSTEADYSTATECTRL |= CMU_HFXOSTEADYSTATECTRL_PEAKMONEN;
    CMU->HFXOTIMEOUTCTRL = (7 << _CMU_HFXOTIMEOUTCTRL_PEAKDETTIMEOUT_SHIFT) | (8 << _CMU_HFXOTIMEOUTCTRL_STEADYTIMEOUT_SHIFT) | (12 << _CMU_HFXOTIMEOUTCTRL_STARTUPTIMEOUT_SHIFT);

    // Enable HFXO and wait for it to be ready
    CMU->OSCENCMD = CMU_OSCENCMD_HFXOEN;
    while(!(CMU->STATUS & CMU_STATUS_HFXORDY));

    // Switch main clock to HFXO and wait for it to be selected
    CMU->HFCLKSEL = CMU_HFCLKSEL_HF_HFXO;
    while((CMU->HFCLKSTATUS & _CMU_HFCLKSTATUS_SELECTED_MASK) != CMU_HFCLKSTATUS_SELECTED_HFXO);

    // Calibrate HFRCO for 72MHz and enable tuning by PLL
    cmu_hfrco_calib((DEVINFO->HFRCOCAL16) | CMU_HFRCOCTRL_FINETUNINGEN);

    // Setup the PLL
    CMU->DPLLCTRL = CMU_DPLLCTRL_REFSEL_HFXO | CMU_DPLLCTRL_AUTORECOVER | CMU_DPLLCTRL_EDGESEL_RISE | CMU_DPLLCTRL_MODE_FREQLL;
    // 72MHz = 50MHz (HFXO) * 1.44 (144/100)
    CMU->DPLLCTRL1 = (143 << _CMU_DPLLCTRL1_N_SHIFT) | (99 << _CMU_DPLLCTRL1_M_SHIFT); // fHFRCO = fHFXO * (N + 1) / (M + 1)

    // Enable the DPLL and wait for it to be ready
    CMU->OSCENCMD = CMU_OSCENCMD_DPLLEN;
    while(!(CMU->STATUS & CMU_STATUS_DPLLRDY));

    // Config peripherals for the new frequency (freq > 32MHz)
    CMU->CTRL |= CMU_CTRL_WSHFLE;

    // Set prescalers
    CMU->HFPRESC = CMU_HFPRESC_HFCLKLEPRESC_DIV2 | CMU_HFPRESC_PRESC_NODIVISION;
    CMU->HFBUSPRESC = 1 << _CMU_HFBUSPRESC_PRESC_SHIFT;
    CMU->HFCOREPRESC = 0 << _CMU_HFCOREPRESC_PRESC_SHIFT;
    CMU->HFPERPRESC = 1 << _CMU_HFPERPRESC_PRESC_SHIFT;
    CMU->HFEXPPRESC = 0 << _CMU_HFEXPPRESC_PRESC_SHIFT;
    CMU->HFPERPRESCB = 0 << _CMU_HFPERPRESCB_PRESC_SHIFT;
    CMU->HFPERPRESCC = 1 << _CMU_HFPERPRESCC_PRESC_SHIFT;

    // Enable clock to peripherals
    CMU->CTRL |= CMU_CTRL_HFPERCLKEN;

    // Switch main clock to HFRCO and wait for it to be selected
    CMU->HFCLKSEL = CMU_HFCLKSEL_HF_HFRCO;
    while((CMU->HFCLKSTATUS & _CMU_HFCLKSTATUS_SELECTED_MASK) != CMU_HFCLKSTATUS_SELECTED_HFRCO);

    // LFA Clock
    CMU->LFACLKSEL = CMU_LFACLKSEL_LFA_LFRCO;

    // LFB Clock
    CMU->LFBCLKSEL = CMU_LFBCLKSEL_LFB_LFRCO;

    // LFC Clock
    CMU->LFCCLKSEL = CMU_LFCCLKSEL_LFC_LFRCO;

    // LFE Clock
    CMU->LFECLKSEL = CMU_LFECLKSEL_LFE_ULFRCO;
}

void systick_init(void)
{
    SysTick->LOAD = (72000000 / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;

    SCB->SHP[11] = 7 << (8 - __NVIC_PRIO_BITS); // Set priority 3,1 (min)
}

void gpio_init(void)
{
    CMU->HFBUSCLKEN0 |= CMU_HFBUSCLKEN0_GPIO;

    // NC - Not Connected (not available in mcu package)
    // NR - Not routed (no routing to pin on pcb, floating)
    // NU - Not used (not currently in use)

    // Port A
    GPIO->P[0].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (6 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[0].MODEL  = GPIO_P_MODEL_MODE0_DISABLED          // NU
                      | GPIO_P_MODEL_MODE1_DISABLED          // NU
                      | GPIO_P_MODEL_MODE2_DISABLED          // NU
                      | GPIO_P_MODEL_MODE3_DISABLED          // NU
                      | GPIO_P_MODEL_MODE4_DISABLED          // NU
                      | GPIO_P_MODEL_MODE5_DISABLED          // NU
                      | GPIO_P_MODEL_MODE6_DISABLED          // NU
                      | GPIO_P_MODEL_MODE7_DISABLED;         // NC
    GPIO->P[0].MODEH  = GPIO_P_MODEH_MODE8_DISABLED          // GPIO - MIC_ENABLE
                      | GPIO_P_MODEH_MODE9_DISABLED          // NC
                      | GPIO_P_MODEH_MODE10_DISABLED         // NC
                      | GPIO_P_MODEH_MODE11_DISABLED         // NC
                      | GPIO_P_MODEH_MODE12_WIREDAND         // LED0R
                      | GPIO_P_MODEH_MODE13_WIREDAND         // LED0B
                      | GPIO_P_MODEH_MODE14_WIREDAND         // LED0G
                      | GPIO_P_MODEH_MODE15_DISABLED;        // NU
    GPIO->P[0].DOUT   = 0x7000; // Leds off By default
    GPIO->P[0].OVTDIS = 0;

    // Port B
    GPIO->P[1].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (6 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[1].MODEL  = GPIO_P_MODEL_MODE0_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE1_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE2_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE3_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE4_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE5_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE6_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE7_DISABLED;                // MAIN_LFXTAL_P
    GPIO->P[1].MODEH  = GPIO_P_MODEH_MODE8_DISABLED                 // MAIN_LFXTAL_N
                      | GPIO_P_MODEH_MODE9_DISABLED                 // NC
                      | GPIO_P_MODEH_MODE10_DISABLED                // NC
                      | GPIO_P_MODEH_MODE11_DISABLED                // PDM_DAT0 - MIC_DATA
                      | GPIO_P_MODEH_MODE12_DISABLED                // PDM_CLK - MIC_CLOCK
                      | GPIO_P_MODEH_MODE13_DISABLED                // MAIN_HFXTAL_P
                      | GPIO_P_MODEH_MODE14_DISABLED                // MAIN_HFXTAL_N
                      | GPIO_P_MODEH_MODE15_DISABLED;               // NC
    GPIO->P[1].DOUT   = 0;
    GPIO->P[1].OVTDIS = 0;

    // Port C
    GPIO->P[2].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (7 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[2].MODEL  = GPIO_P_MODEL_MODE0_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE1_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE2_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE3_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE4_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE5_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE6_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE7_DISABLED;                // NC
    GPIO->P[2].MODEH  = GPIO_P_MODEH_MODE8_DISABLED                 // NC
                      | GPIO_P_MODEH_MODE9_DISABLED                 // NC
                      | GPIO_P_MODEH_MODE10_DISABLED                // NC
                      | GPIO_P_MODEH_MODE11_DISABLED                // NC
                      | GPIO_P_MODEH_MODE12_DISABLED                // NC
                      | GPIO_P_MODEH_MODE13_DISABLED                // NC
                      | GPIO_P_MODEH_MODE14_DISABLED                // NC
                      | GPIO_P_MODEH_MODE15_DISABLED;               // NC
    GPIO->P[2].DOUT   = 0;
    GPIO->P[2].OVTDIS = 0;

    // Port D
    GPIO->P[3].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (6 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[3].MODEL  = GPIO_P_MODEL_MODE0_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE1_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE2_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE3_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE4_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE5_INPUT                    // GPIO - BTN0
                      | GPIO_P_MODEL_MODE6_WIREDAND                 // LED1R
                      | GPIO_P_MODEL_MODE7_DISABLED;                // NU
    GPIO->P[3].MODEH  = GPIO_P_MODEH_MODE8_INPUT                    // GPIO - BTN1
                      | GPIO_P_MODEH_MODE9_DISABLED                 // NC
                      | GPIO_P_MODEH_MODE10_DISABLED                // NC
                      | GPIO_P_MODEH_MODE11_DISABLED                // NC
                      | GPIO_P_MODEH_MODE12_DISABLED                // NC
                      | GPIO_P_MODEH_MODE13_DISABLED                // NC
                      | GPIO_P_MODEH_MODE14_DISABLED                // NC
                      | GPIO_P_MODEH_MODE15_DISABLED;               // NC
    GPIO->P[3].DOUT   = 0;
    GPIO->P[3].OVTDIS = 0;

    // Port E
    GPIO->P[4].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (6 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[4].MODEL  = GPIO_P_MODEL_MODE0_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE1_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE2_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE3_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE4_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE5_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE6_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE7_DISABLED;                // NU
    GPIO->P[4].MODEH  = GPIO_P_MODEH_MODE8_DISABLED                 // NU
                      | GPIO_P_MODEH_MODE9_DISABLED                 // NU
                      | GPIO_P_MODEH_MODE10_DISABLED                // NU
                      | GPIO_P_MODEH_MODE11_DISABLED                // NU
                      | GPIO_P_MODEH_MODE12_WIREDAND                // LED1B
                      | GPIO_P_MODEH_MODE13_DISABLED                // NU
                      | GPIO_P_MODEH_MODE14_DISABLED                // NU
                      | GPIO_P_MODEH_MODE15_DISABLED;               // NU
    GPIO->P[4].DOUT   = 0;
    GPIO->P[4].OVTDIS = 0;

    // Port F
    GPIO->P[5].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (6 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[5].MODEL  = GPIO_P_MODEL_MODE0_PUSHPULL                 // SWCLK
                      | GPIO_P_MODEL_MODE1_PUSHPULL                 // SWDIO
                      | GPIO_P_MODEL_MODE2_PUSHPULL                 // SWO
                      | GPIO_P_MODEL_MODE3_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE4_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE5_DISABLED                 // NU
                      | GPIO_P_MODEL_MODE6_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE7_DISABLED;                // NC
    GPIO->P[5].MODEH  = GPIO_P_MODEH_MODE8_DISABLED                 // NC
                      | GPIO_P_MODEH_MODE9_DISABLED                 // NC
                      | GPIO_P_MODEH_MODE10_DISABLED                // USB N
                      | GPIO_P_MODEH_MODE11_DISABLED                // USB P
                      | GPIO_P_MODEH_MODE12_WIREDAND                // LED1G
                      | GPIO_P_MODEH_MODE13_DISABLED                // NC
                      | GPIO_P_MODEH_MODE14_DISABLED                // NC
                      | GPIO_P_MODEH_MODE15_DISABLED;               // NC
    GPIO->P[5].DOUT   = 0;

    GPIO->P[5].OVTDIS = 0;

    // Debugger Route
    GPIO->ROUTEPEN &= ~(GPIO_ROUTEPEN_TDIPEN | GPIO_ROUTEPEN_TDOPEN);   // Disable JTAG
    GPIO->ROUTEPEN |= GPIO_ROUTEPEN_SWVPEN;                             // Enable SWO
    GPIO->ROUTELOC0 = GPIO_ROUTELOC0_SWVLOC_LOC0;                       // SWO on PF2

    // External interrupts
    GPIO->EXTIPSELL = GPIO_EXTIPSELL_EXTIPSEL0_PORTE            // NU
                    | GPIO_EXTIPSELL_EXTIPSEL1_PORTB            // NU
                    | GPIO_EXTIPSELL_EXTIPSEL2_PORTB            // NU
                    | GPIO_EXTIPSELL_EXTIPSEL3_PORTB            // NU
                    | GPIO_EXTIPSELL_EXTIPSEL4_PORTA            // NU
                    | GPIO_EXTIPSELL_EXTIPSEL5_PORTA            // NU
                    | GPIO_EXTIPSELL_EXTIPSEL6_PORTC            // NU
                    | GPIO_EXTIPSELL_EXTIPSEL7_PORTC;           // NU
    GPIO->EXTIPSELH = GPIO_EXTIPSELH_EXTIPSEL8_PORTA            // NU
                    | GPIO_EXTIPSELH_EXTIPSEL9_PORTE            // NU
                    | GPIO_EXTIPSELH_EXTIPSEL10_PORTF           // NU
                    | GPIO_EXTIPSELH_EXTIPSEL11_PORTA           // NU
                    | GPIO_EXTIPSELH_EXTIPSEL12_PORTA           // NU
                    | GPIO_EXTIPSELH_EXTIPSEL13_PORTE           // NU
                    | GPIO_EXTIPSELH_EXTIPSEL14_PORTF           // NU
                    | GPIO_EXTIPSELH_EXTIPSEL15_PORTA;          // NU

    GPIO->EXTIPINSELL = GPIO_EXTIPINSELL_EXTIPINSEL0_PIN3       // NU
                      | GPIO_EXTIPINSELL_EXTIPINSEL1_PIN1       // NU
                      | GPIO_EXTIPINSELL_EXTIPINSEL2_PIN2       // NU
                      | GPIO_EXTIPINSELL_EXTIPINSEL3_PIN3       // NU
                      | GPIO_EXTIPINSELL_EXTIPINSEL4_PIN6       // NU
                      | GPIO_EXTIPINSELL_EXTIPINSEL5_PIN7       // NU
                      | GPIO_EXTIPINSELL_EXTIPINSEL6_PIN4       // NU
                      | GPIO_EXTIPINSELL_EXTIPINSEL7_PIN7;      // NU
    GPIO->EXTIPINSELH = GPIO_EXTIPINSELH_EXTIPINSEL8_PIN8       // NU
                      | GPIO_EXTIPINSELH_EXTIPINSEL9_PIN9       // NU
                      | GPIO_EXTIPINSELH_EXTIPINSEL10_PIN11     // NU
                      | GPIO_EXTIPINSELH_EXTIPINSEL11_PIN8      // NU
                      | GPIO_EXTIPINSELH_EXTIPINSEL12_PIN13     // NU
                      | GPIO_EXTIPINSELH_EXTIPINSEL13_PIN15     // NU
                      | GPIO_EXTIPINSELH_EXTIPINSEL14_PIN12     // NU
                      | GPIO_EXTIPINSELH_EXTIPINSEL15_PIN12;    // NU

}

/*--------------------------------------------------------------------*/
/* Board Init                                                         */
/*--------------------------------------------------------------------*/

void board_init(void)
{

  emu_dcdc_init(1800.f, 50.f, 100.f, 0.f); // Init DC-DC converter (1.8 V, 50 mA active, 100 uA sleep, 0 mA reverse limit)
  emu_init(0);
  emu_reg_init(3300.f); // set output regulator to 3.3V

  cmu_hfxo_startup_calib(0x200, 0x145); // Config HFXO Startup for 1280 uA, 36 pF (18 pF + 2 pF CLOAD)
  cmu_hfxo_steady_calib(0x009, 0x145); // Config HFXO Steady for 12 uA, 36 pF (18 pF + 2 pF CLOAD)

  cmu_init(); // Init Clock Management Unit

  cmu_ushfrco_calib(1, DEVINFO->USHFRCOCAL13); // Enable and calibrate USHFRCO for 48 MHz
  cmu_auxhfrco_calib(1, DEVINFO->AUXHFRCOCAL11); // Enable and calibrate AUXHFRCO for 32 MHz

  CMU->USBCRCTRL = CMU_USBCRCTRL_USBCREN; // enable USB clock recovery
  CMU->USBCTRL = CMU_USBCTRL_USBCLKSEL_USHFRCO | CMU_USBCTRL_USBCLKEN;  // select USHFRCO as USB Phy clock source and enable it

  CMU->HFBUSCLKEN0 |= CMU_HFBUSCLKEN0_USB;  // enable USB peripheral clock

  systick_init(); // Init system tick

  gpio_init(); // Init IOs

}

/*--------------------------------------------------------------------*/
/* Board porting API                                                  */
/*--------------------------------------------------------------------*/

void board_led_write(bool state)
{
  // Combine red and blue for pink Because it looks good :)
  GPIO->P[LED_PORT].DOUT = (GPIO->P[LED_PORT].DOUT & ~((1 << LED_PIN_R) | (1 << LED_PIN_B))) | (state << LED_PIN_R) | (state << LED_PIN_B);
}

uint32_t board_button_read(void)
{
  return !!(GPIO->P[BUTTON_PORT].DIN & (1 << BUTTON_PIN));
}

int board_uart_read(uint8_t* buf, int len)
{
  (void) buf; (void) len;
  return 0;
}

int board_uart_write(void const * buf, int len)
{
  (void) buf; (void) len;
  return 0;
}

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void SysTick_Handler(void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
