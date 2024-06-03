/**
  ******************************************************************************
  * @file    usbpd_hw_if_it.c
  * @author  MCD Application Team
  * @brief   This file contains HW interface interrupt routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbpd_devices_conf.h"
#include "usbpd_core.h"
#include "usbpd_hw_if.h"
#include "usbpd_trace.h"
#if defined(_LOW_POWER)
#include "usbpd_lowpower.h"
#endif /* _LOW_POWER */
#if defined(_FRS)
#include "usbpd_timersserver.h"
#endif /* _FRS */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void PORTx_IRQHandler(uint8_t PortNum);

void USBPD_PORT0_IRQHandler(void)
{
  PORTx_IRQHandler(USBPD_PORT_0);
}

void PORTx_IRQHandler(uint8_t PortNum)
{
  UCPD_TypeDef *hucpd = Ports[PortNum].husbpd;
  uint32_t _interrupt = LL_UCPD_ReadReg(hucpd, SR);
  static uint8_t ovrflag = 0;

  if ((hucpd->IMR & _interrupt) != 0u)
  {
    /* TXIS no need to enable it all the transfer are done by DMA */
    if (UCPD_SR_TXMSGDISC == (_interrupt & UCPD_SR_TXMSGDISC))
    {
      /* Message has been discarded */
      LL_UCPD_ClearFlag_TxMSGDISC(hucpd);
      SET_BIT(Ports[PortNum].hdmatx->CCR, DMA_CCR_SUSP | DMA_CCR_RESET);
      while ((Ports[PortNum].hdmatx->CCR & DMA_CCR_EN) == DMA_CCR_EN);
      Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted(PortNum, 1);
      return;
    }

    if (UCPD_SR_TXMSGSENT == (_interrupt & UCPD_SR_TXMSGSENT))
    {
      /* Message has been fully transferred */
      LL_UCPD_ClearFlag_TxMSGSENT(hucpd);
      SET_BIT(Ports[PortNum].hdmatx->CCR, DMA_CCR_SUSP | DMA_CCR_RESET);
      while ((Ports[PortNum].hdmatx->CCR & DMA_CCR_EN) == DMA_CCR_EN);
      Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted(PortNum, 0);

#if defined(_LOW_POWER)
      UTIL_LPM_SetStopMode(0 == PortNum ? LPM_PE_0 : LPM_PE_1, UTIL_LPM_ENABLE);
#endif /* _LOW_POWER */
      return;
    }

    if (UCPD_SR_TXMSGABT == (_interrupt & UCPD_SR_TXMSGABT))
    {
      LL_UCPD_ClearFlag_TxMSGABT(hucpd);
      SET_BIT(Ports[PortNum].hdmatx->CCR, DMA_CCR_SUSP | DMA_CCR_RESET);
      while ((Ports[PortNum].hdmatx->CCR &  DMA_CCR_EN) == DMA_CCR_EN);
      Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted(PortNum, 2);
      return;
    }

    /* HRSTDISC : hard reset sending has been discarded */
    if (UCPD_SR_HRSTDISC == (_interrupt & UCPD_SR_HRSTDISC))
    {
      LL_UCPD_ClearFlag_TxHRSTDISC(hucpd);
      return;
    }

    /* TXUND : tx underrun detected */
    if (UCPD_SR_HRSTSENT == (_interrupt & UCPD_SR_HRSTSENT))
    {
      /* Answer not expected by the stack */
      LL_UCPD_ClearFlag_TxHRSTSENT(hucpd);
      Ports[PortNum].cbs.USBPD_HW_IF_TX_HardResetCompleted(PortNum, USBPD_SOPTYPE_HARD_RESET);
      return;
    }

    /* TXUND : tx underrun detected */
    if (UCPD_SR_TXUND == (_interrupt & UCPD_SR_TXUND))
    {
      /* Nothing to do.
         The port partner checks the message integrity with CRC, so PRL will repeat the sending.
         Can be used for debugging purpose */
      LL_UCPD_ClearFlag_TxUND(hucpd);
      return;
    }

    /* RXNE : not needed the stack only perform transfer by DMA */
    /* RXORDDET: not needed so stack will not enabled this interrupt */
    if (UCPD_SR_RXORDDET == (_interrupt & UCPD_SR_RXORDDET))
    {
      if (LL_UCPD_RXORDSET_CABLE_RESET == hucpd->RX_ORDSET)
      {
        /* Cable reset detected */
        Ports[PortNum].cbs.USBPD_HW_IF_RX_ResetIndication(PortNum, USBPD_SOPTYPE_CABLE_RESET);
      }
      LL_UCPD_ClearFlag_RxOrderSet(hucpd);
#if defined(_LOW_POWER)
      UTIL_LPM_SetStopMode(0 == PortNum ? LPM_PE_0 : LPM_PE_1, UTIL_LPM_DISABLE);
#endif /* _LOW_POWER */

      /* Forbid message sending */
      Ports[PortNum].RXStatus = USBPD_TRUE;
      return;
    }

    /* Check RXHRSTDET */
    if (UCPD_SR_RXHRSTDET == (_interrupt & UCPD_SR_RXHRSTDET))
    {
      Ports[PortNum].cbs.USBPD_HW_IF_RX_ResetIndication(PortNum, USBPD_SOPTYPE_HARD_RESET);
      LL_UCPD_ClearFlag_RxHRST(hucpd);
      return;
    }

    /* Check RXOVR */
    if (UCPD_SR_RXOVR == (_interrupt & UCPD_SR_RXOVR))
    {
      /* Nothing to do, the message will be discarded and port Partner will try sending again. */
      ovrflag = 1;
      LL_UCPD_ClearFlag_RxOvr(hucpd);
      return;
    }

    /* Check RXMSGEND an Rx message has been received */
    if (UCPD_SR_RXMSGEND == (_interrupt & UCPD_SR_RXMSGEND))
    {
      Ports[PortNum].RXStatus = USBPD_FALSE;

      /* For DMA mode, add a check to ensure the number of data received matches
         the number of data received by UCPD */
      LL_UCPD_ClearFlag_RxMsgEnd(hucpd);

      /* Disable DMA */
      SET_BIT(Ports[PortNum].hdmarx->CCR, DMA_CCR_SUSP | DMA_CCR_RESET);
      while ((Ports[PortNum].hdmarx->CCR & DMA_CCR_EN) == DMA_CCR_EN);

      /* Ready for next transaction */
      WRITE_REG(Ports[PortNum].hdmarx->CDAR, (uint32_t)Ports[PortNum].ptr_RxBuff);
      MODIFY_REG(Ports[PortNum].hdmarx->CBR1, DMA_CBR1_BNDT, (SIZE_MAX_PD_TRANSACTION_UNCHUNK & DMA_CBR1_BNDT));

      /* Enable the DMA */
      SET_BIT(Ports[PortNum].hdmarx->CCR, DMA_CCR_EN);
#if defined(_LOW_POWER)
      UTIL_LPM_SetStopMode(0 == PortNum ? LPM_PE_0 : LPM_PE_1, UTIL_LPM_ENABLE);
#endif /* _LOW_POWER */

      if (((_interrupt & UCPD_SR_RXERR) == 0u) && (ovrflag == 0u))
      {
        /* Rx message has been received without error */
        Ports[PortNum].cbs.USBPD_HW_IF_RX_Completed(PortNum, hucpd->RX_ORDSET & UCPD_RX_ORDSET_RXORDSET);
      }
      ovrflag = 0;
      return;
    }

    /* Check TYPECEVT1IE/TYPECEVT1IE || check TYPECEVT2IE/TYPECEVT2IE */
    if ((UCPD_SR_TYPECEVT1 == (_interrupt & UCPD_SR_TYPECEVT1))
        || (UCPD_SR_TYPECEVT2 == (_interrupt & UCPD_SR_TYPECEVT2)))
    {
      /* Clear both interrupt */
      LL_UCPD_ClearFlag_TypeCEventCC1(hucpd);
      LL_UCPD_ClearFlag_TypeCEventCC2(hucpd);
      Ports[PortNum].USBPD_CAD_WakeUp();
      /* Wakeup CAD to check the detection event */
      return;
    }

#if defined(_FRS)
    /* Check FRSEVTIE */
    if (UCPD_SR_FRSEVT == (_interrupt & UCPD_SR_FRSEVT))
    {
      LL_UCPD_ClearFlag_FRS(hucpd);
      if ((USBPD_PORTPOWERROLE_SNK == Ports[PortNum].params->PE_PowerRole)
          && (Ports[PortNum].params->PE_SwapOngoing == USBPD_FALSE))
      {
        /* Confirm the FRS by checking if an RP is always present on the current CC line.
           We should wait for maximum FRS timing */
        USBPD_TIM_Start((TIM_identifier)(2 * PortNum), 150);
        while ((USBPD_TIM_IsExpired((TIM_identifier)(2u * PortNum)) == 0u));

        if ((0 != (hucpd->SR & (UCPD_SR_TYPEC_VSTATE_CC1 | UCPD_SR_TYPEC_VSTATE_CC2))) &&
            (USBPD_POWER_EXPLICITCONTRACT == Ports[PortNum].params->PE_Power))
        {
          /* Switch the power to take the control of VBUS.
             When VBUS go under VSAFE5V the sink shall switch ON VBUS in timing < tSrcFRSwap */
          BSP_USBPD_PWR_FRSVBUSEnable(PortNum);
          USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, "FRS received", 12u);
          Ports[PortNum].cbs.USBPD_HW_IF_TX_FRSReception(PortNum);
        }
      }
    }
#endif /* _FRS */
  }
}

