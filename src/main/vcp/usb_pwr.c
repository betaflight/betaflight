/**
 ******************************************************************************
 * @file    usb_pwr.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Connection/disconnection & power management
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_pwr.h"
#include "hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t bDeviceState = UNCONNECTED; /* USB device status */
__IO boolean fSuspendEnabled = TRUE; /* true when suspend is possible */  // HJI
__IO uint32_t EP[8];

struct {
    __IO RESUME_STATE eState;
    __IO uint8_t bESOFcnt;
} ResumeS;

__IO uint32_t remotewakeupon = 0;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : PowerOn
 * Description    :
 * Input          : None.
 * Output         : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
RESULT PowerOn(void)
{
    uint16_t wRegVal;

    /*** cable plugged-in ? ***/
    USB_Cable_Config(ENABLE);

    /*** CNTR_PWDN = 0 ***/
    wRegVal = CNTR_FRES;
    _SetCNTR(wRegVal);

    /*** CNTR_FRES = 0 ***/
    wInterrupt_Mask = 0;
    _SetCNTR(wInterrupt_Mask);
    /*** Clear pending interrupts ***/
    _SetISTR(0);
    /*** Set interrupt mask ***/
    wInterrupt_Mask = CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;
    _SetCNTR(wInterrupt_Mask);

    return USB_SUCCESS;
}

/*******************************************************************************
 * Function Name  : PowerOff
 * Description    : handles switch-off conditions
 * Input          : None.
 * Output         : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
RESULT PowerOff()
{
    /* disable all interrupts and force USB reset */
    _SetCNTR(CNTR_FRES);
    /* clear interrupt status register */
    _SetISTR(0);
    /* Disable the Pull-Up*/
    USB_Cable_Config(DISABLE);
    /* switch-off device */
    _SetCNTR(CNTR_FRES + CNTR_PDWN);
    /* sw variables reset */
    /* ... */

    return USB_SUCCESS;
}

/*******************************************************************************
 * Function Name  : Suspend
 * Description    : sets suspend mode operating conditions
 * Input          : None.
 * Output         : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
void Suspend(void)
{
    uint32_t i = 0;
    uint16_t wCNTR;
    uint32_t tmpreg = 0;
    __IO uint32_t savePWR_CR = 0;
    /* suspend preparation */
    /* ... */

    /*Store CNTR value */
    wCNTR = _GetCNTR();

    /* This a sequence to apply a force RESET to handle a robustness case */

    /*Store endpoints registers status */
    for (i = 0; i < 8; i++)
        EP[i] = _GetENDPOINT(i);

    /* unmask RESET flag */
    wCNTR |= CNTR_RESETM;
    _SetCNTR(wCNTR);

    /*apply FRES */
    wCNTR |= CNTR_FRES;
    _SetCNTR(wCNTR);

    /*clear FRES*/
    wCNTR &= ~CNTR_FRES;
    _SetCNTR(wCNTR);

    /*poll for RESET flag in ISTR*/
    while ((_GetISTR() & ISTR_RESET) == 0)
        ;

    /* clear RESET flag in ISTR */
    _SetISTR((uint16_t)CLR_RESET);

    /*restore Enpoints*/
    for (i = 0; i < 8; i++)
        _SetENDPOINT(i, EP[i]);

    /* Now it is safe to enter macrocell in suspend mode */
    wCNTR |= CNTR_FSUSP;
    _SetCNTR(wCNTR);

    /* force low-power mode in the macrocell */
    wCNTR = _GetCNTR();
    wCNTR |= CNTR_LPMODE;
    _SetCNTR(wCNTR);

    /*prepare entry in low power mode (STOP mode)*/
    /* Select the regulator state in STOP mode*/
    savePWR_CR = PWR->CR;
    tmpreg = PWR->CR;
    /* Clear PDDS and LPDS bits */
    tmpreg &= ((uint32_t)0xFFFFFFFC);
    /* Set LPDS bit according to PWR_Regulator value */
    tmpreg |= PWR_Regulator_LowPower;
    /* Store the new value */
    PWR->CR = tmpreg;
    /* Set SLEEPDEEP bit of Cortex System Control Register */
#if defined (STM32F303xC) || defined (STM32F37X)
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
#else
    SCB->SCR |= SCB_SCR_SLEEPDEEP;
#endif
    /* enter system in STOP mode, only when wakeup flag in not set */
    if ((_GetISTR() & ISTR_WKUP) == 0) {
        __WFI();
        /* Reset SLEEPDEEP bit of Cortex System Control Register */
#if defined (STM32F303xC) || defined (STM32F37X)
        SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
#else
        SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP);
#endif
    } else {
        /* Clear Wakeup flag */
        _SetISTR(CLR_WKUP);
        /* clear FSUSP to abort entry in suspend mode  */
        wCNTR = _GetCNTR();
        wCNTR &= ~CNTR_FSUSP;
        _SetCNTR(wCNTR);

        /*restore sleep mode configuration */
        /* restore Power regulator config in sleep mode*/
        PWR->CR = savePWR_CR;

        /* Reset SLEEPDEEP bit of Cortex System Control Register */
#if defined (STM32F303xC) || defined (STM32F37X)
        SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
#else
        SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP);
#endif

    }
}

/*******************************************************************************
 * Function Name  : Resume_Init
 * Description    : Handles wake-up restoring normal operations
 * Input          : None.
 * Output         : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
void Resume_Init(void)
{
    uint16_t wCNTR;

    /* ------------------ ONLY WITH BUS-POWERED DEVICES ---------------------- */
    /* restart the clocks */
    /* ...  */

    /* CNTR_LPMODE = 0 */
    wCNTR = _GetCNTR();
    wCNTR &= (~CNTR_LPMODE);
    _SetCNTR(wCNTR);

    /* restore full power */
    /* ... on connected devices */
    Leave_LowPowerMode();

    /* reset FSUSP bit */
    _SetCNTR(IMR_MSK);

    /* reverse suspend preparation */
    /* ... */

}

/*******************************************************************************
 * Function Name  : Resume
 * Description    : This is the state machine handling resume operations and
 *                 timing sequence. The control is based on the Resume structure
 *                 variables and on the ESOF interrupt calling this subroutine
 *                 without changing machine state.
 * Input          : a state machine value (RESUME_STATE)
 *                  RESUME_ESOF doesn't change ResumeS.eState allowing
 *                  decrementing of the ESOF counter in different states.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Resume(RESUME_STATE eResumeSetVal)
{
    uint16_t wCNTR;

    if (eResumeSetVal != RESUME_ESOF)
        ResumeS.eState = eResumeSetVal;
    switch (ResumeS.eState) {
        case RESUME_EXTERNAL:
            if (remotewakeupon == 0) {
                Resume_Init();
                ResumeS.eState = RESUME_OFF;
            } else /* RESUME detected during the RemoteWAkeup signalling => keep RemoteWakeup handling*/
            {
                ResumeS.eState = RESUME_ON;
            }
            break;
        case RESUME_INTERNAL:
            Resume_Init();
            ResumeS.eState = RESUME_START;
            remotewakeupon = 1;
            break;
        case RESUME_LATER:
            ResumeS.bESOFcnt = 2;
            ResumeS.eState = RESUME_WAIT;
            break;
        case RESUME_WAIT:
            ResumeS.bESOFcnt--;
            if (ResumeS.bESOFcnt == 0)
                ResumeS.eState = RESUME_START;
            break;
        case RESUME_START:
            wCNTR = _GetCNTR();
            wCNTR |= CNTR_RESUME;
            _SetCNTR(wCNTR);
            ResumeS.eState = RESUME_ON;
            ResumeS.bESOFcnt = 10;
            break;
        case RESUME_ON:
            ResumeS.bESOFcnt--;
            if (ResumeS.bESOFcnt == 0) {
                wCNTR = _GetCNTR();
                wCNTR &= (~CNTR_RESUME);
                _SetCNTR(wCNTR);
                ResumeS.eState = RESUME_OFF;
                remotewakeupon = 0;
            }
            break;
        case RESUME_OFF:
        case RESUME_ESOF:
        default:
            ResumeS.eState = RESUME_OFF;
            break;
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
