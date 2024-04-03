/**
  ******************************************************************************
  * @file    usbpd_phy_hw_if.c
  * @author  MCD Application Team
  * @brief   This file contains phy interface control functions.
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
#define USBPD_PHY_HW_IF_C

#include "usbpd_devices_conf.h"
#include "usbpd_hw.h"
#include "usbpd_core.h"
#include "usbpd_hw_if.h"
#if !defined(USBPDCORE_LIB_NO_PD)
#include "usbpd_timersserver.h"
#endif /* !USBPDCORE_LIB_NO_PD */
#if defined(_LOW_POWER)
#include "usbpd_lowpower.h"
#include "usbpd_cad_hw_if.h"
#endif /* _LOW_POWER */

/* Private typedef -----------------------------------------------------------*/
#define PHY_ENTER_CRITICAL_SECTION()  uint32_t primask = __get_PRIMASK(); \
  __disable_irq();

#define PHY_LEAVE_CRITICAL_SECTION()  __set_PRIMASK(primask);

/* Private define ------------------------------------------------------------*/

/* If CFG3 register exists */
#if defined(UCPD_CFG3_TRIM_CC1_RP)
/****************  Bits definition for UCPD Trimming register  ****************/
#define UCPD_VALUE_TRIM_CC1_RP_3A0_POS  (4U)
#define UCPD_VALUE_TRIM_CC1_RP_3A0_MSK  (0xFUL << UCPD_VALUE_TRIM_CC1_RP_3A0_POS) /*!< 0x000000F0 */
#define UCPD_VALUE_TRIM_CC1_RP_3A0      UCPD_VALUE_TRIM_CC1_RP_3A0_MSK            /*!< SW trim value for RP 3A0 (CC1) */
#define UCPD_VALUE_TRIM_CC1_RP_1A5_POS  (0U)
#define UCPD_VALUE_TRIM_CC1_RP_1A5_MSK  (0xFUL << UCPD_VALUE_TRIM_CC1_RP_1A5_POS) /*!< 0x0000000F */
#define UCPD_VALUE_TRIM_CC1_RP_1A5      UCPD_VALUE_TRIM_CC1_RP_1A5_MSK            /*!< SW trim value for RP 1A5 (CC1) */
#define UCPD_VALUE_TRIM_CC1_RD_POS      (0U)
#define UCPD_VALUE_TRIM_CC1_RD_MSK      (0xFUL << UCPD_VALUE_TRIM_CC1_RD_POS)     /*!< 0x0000000F */
#define UCPD_VALUE_TRIM_CC1_RD          UCPD_VALUE_TRIM_CC1_RD_MSK                /*!< SW trim value for RD (CC1)     */
#define UCPD_VALUE_TRIM_CC2_RP_3A0_POS  (12U)
#define UCPD_VALUE_TRIM_CC2_RP_3A0_MSK  (0xFUL << UCPD_VALUE_TRIM_CC2_RP_3A0_POS) /*!< 0x0000F000 */
#define UCPD_VALUE_TRIM_CC2_RP_3A0      UCPD_VALUE_TRIM_CC2_RP_3A0_MSK            /*!< SW trim value for RP 3A0 (CC2) */
#define UCPD_VALUE_TRIM_CC2_RP_1A5_POS  (16U)
#define UCPD_VALUE_TRIM_CC2_RP_1A5_MSK  (0xFUL << UCPD_VALUE_TRIM_CC2_RP_1A5_POS) /*!< 0x000F0000 */
#define UCPD_VALUE_TRIM_CC2_RP_1A5      UCPD_VALUE_TRIM_CC2_RP_1A5_MSK            /*!< SW trim value for RP 1A5 (CC2) */
#define UCPD_VALUE_TRIM_CC2_RD_POS      (8U)
#define UCPD_VALUE_TRIM_CC2_RD_MSK      (0xFUL << UCPD_VALUE_TRIM_CC2_RD_POS)     /*!< 0x00000F00 */
#define UCPD_VALUE_TRIM_CC2_RD          UCPD_VALUE_TRIM_CC2_RD_MSK                /*!< SW trim value for RD (CC2)     */
#endif /* UCPD_CFG3_TRIM_CC1_RP */

/* Private variables -----------------------------------------------------------*/

/* If CFG3 register exists */
#if defined(UCPD_CFG3_TRIM_CC1_RP)
/* Variables used for SW trimming procedure */
uint32_t dev_id;
uint32_t rev_id;

/* UCPD software trim data - Non-volatile memory location */
const __IO uint32_t *pUCPD_TRIM_3A0_CC1 = (uint32_t *)(0x4002242CUL);
const __IO uint32_t *pUCPD_TRIM_3A0_CC2 = (uint32_t *)(0x4002242CUL);
const __IO uint32_t *pUCPD_TRIM_1A5_CC1 = (uint32_t *)(0x08FFF844UL);
const __IO uint32_t *pUCPD_TRIM_1A5_CC2 = (uint32_t *)(0x08FFF844UL);
const __IO uint32_t *pUCPD_TRIM_Rd_CC1  = (uint32_t *)(0x4002242CUL);
const __IO uint32_t *pUCPD_TRIM_Rd_CC2  = (uint32_t *)(0x4002242CUL);
#endif /* UCPD_CFG3_TRIM_CC1_RP */

/* Private function prototypes -----------------------------------------------*/
USBPD_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];


/* Private functions ---------------------------------------------------------*/


void USBPD_HW_IF_GlobalHwInit(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

  /* If CFG3 register exists */
#if defined(UCPD_CFG3_TRIM_CC1_RP)
  /* The CC pull-up (Rp) and pull-down (Rd) must be trimmed to meet the required accuracy.
     The trimming values are saved in the non-volatile memory. */

  /* Retrieve device and revision ID */
  dev_id = HAL_GetDEVID();
  rev_id = HAL_GetREVID();

  /* This is only needed for some devices/rev. */
  if (((dev_id == 0x484UL) && (rev_id == 0x1001UL)) ||
      ((dev_id == 0x478UL) && (rev_id == 0x1000UL)))
  {
    CLEAR_BIT(UCPD1->CFG3, (UCPD_CFG3_TRIM_CC1_RP | UCPD_CFG3_TRIM_CC2_RP |
                            UCPD_CFG3_TRIM_CC1_RD | UCPD_CFG3_TRIM_CC2_RD)); /* Clear bits to change */

    uint32_t temp;
    temp = ((((*pUCPD_TRIM_3A0_CC1) & UCPD_VALUE_TRIM_CC1_RP_3A0_MSK) >>
             UCPD_VALUE_TRIM_CC1_RP_3A0_POS) << UCPD_CFG3_TRIM_CC1_RP_Pos);
    SET_BIT(UCPD1->CFG3, temp); /* Apply Rp trimming */
    temp = ((((*pUCPD_TRIM_3A0_CC2) & UCPD_VALUE_TRIM_CC2_RP_3A0_MSK) >>
             UCPD_VALUE_TRIM_CC2_RP_3A0_POS) << UCPD_CFG3_TRIM_CC2_RP_Pos);
    SET_BIT(UCPD1->CFG3, temp); /* Apply Rp trimming */
    temp = ((((*pUCPD_TRIM_Rd_CC1) & UCPD_VALUE_TRIM_CC1_RD_MSK) >> \
             UCPD_VALUE_TRIM_CC1_RD_POS) << UCPD_CFG3_TRIM_CC1_RD_Pos);
    SET_BIT(UCPD1->CFG3, temp); /* Apply Rd trimming */
    temp = ((((*pUCPD_TRIM_Rd_CC2) & UCPD_VALUE_TRIM_CC2_RD_MSK) >> \
             UCPD_VALUE_TRIM_CC2_RD_POS) << UCPD_CFG3_TRIM_CC2_RD_Pos);
    SET_BIT(UCPD1->CFG3, temp); /* Apply Rd trimming */
  }
#endif /* UCPD_CFG3_TRIM_CC1_RP */
}

#if !defined(USBPDCORE_LIB_NO_PD)
void USBPD_HW_IF_StopBISTMode2(uint8_t PortNum)
{
  uint32_t  _cr = READ_REG(Ports[PortNum].husbpd->CR) & ~(UCPD_CR_TXMODE | UCPD_CR_TXSEND);

  LL_UCPD_Disable(Ports[PortNum].husbpd);
  LL_UCPD_Enable(Ports[PortNum].husbpd);

  Ports[PortNum].husbpd->CR = _cr;
}

USBPD_StatusTypeDef USBPD_HW_IF_SendBuffer(uint8_t PortNum, USBPD_SOPType_TypeDef Type, uint8_t *pBuffer, uint32_t Size)
{
  USBPD_StatusTypeDef _status = USBPD_OK;

  if (USBPD_SOPTYPE_HARD_RESET == Type)
  {
    LL_UCPD_SendHardReset(Ports[PortNum].husbpd);
  }
  else
  {
    PHY_ENTER_CRITICAL_SECTION()

    /* If RX is ongoing or if a DMA transfer is active then discard the buffer sending */
    if ((Ports[PortNum].RXStatus == USBPD_TRUE) || ((Ports[PortNum].hdmatx->CCR &  DMA_CCR_EN) == DMA_CCR_EN))
    {
      PHY_LEAVE_CRITICAL_SECTION()
      _status = USBPD_ERROR;
    }
    else
    {
      PHY_LEAVE_CRITICAL_SECTION()

      switch (Type)
      {
        case USBPD_SOPTYPE_SOP :
        {
          LL_UCPD_WriteTxOrderSet(Ports[PortNum].husbpd, LL_UCPD_ORDERED_SET_SOP);
          LL_UCPD_SetTxMode(Ports[PortNum].husbpd, LL_UCPD_TXMODE_NORMAL);
          break;
        }
        case USBPD_SOPTYPE_SOP1 :
        {
          LL_UCPD_WriteTxOrderSet(Ports[PortNum].husbpd, LL_UCPD_ORDERED_SET_SOP1);
          LL_UCPD_SetTxMode(Ports[PortNum].husbpd, LL_UCPD_TXMODE_NORMAL);
          break;
        }
        case USBPD_SOPTYPE_SOP2 :
        {
          LL_UCPD_WriteTxOrderSet(Ports[PortNum].husbpd, LL_UCPD_ORDERED_SET_SOP2);
          LL_UCPD_SetTxMode(Ports[PortNum].husbpd, LL_UCPD_TXMODE_NORMAL);
          break;
        }
        case USBPD_SOPTYPE_CABLE_RESET :
        {
          LL_UCPD_SetTxMode(Ports[PortNum].husbpd, LL_UCPD_TXMODE_CABLE_RESET);
          break;
        }
        case USBPD_SOPTYPE_BIST_MODE_2 :
        {
          LL_UCPD_SetTxMode(Ports[PortNum].husbpd, LL_UCPD_TXMODE_BIST_CARRIER2);
          break;
        }
        default :
          _status = USBPD_ERROR;
          break;
      }

      if (USBPD_OK == _status)
      {
#if defined(_LOW_POWER)
        UTIL_LPM_SetStopMode(0 == PortNum ? LPM_PE_0 : LPM_PE_1, UTIL_LPM_DISABLE);
#endif /* _LOW_POWER */
        SET_BIT(Ports[PortNum].hdmatx->CCR, DMA_CCR_SUSP | DMA_CCR_RESET);
        while ((Ports[PortNum].hdmatx->CCR &  DMA_CCR_EN) == DMA_CCR_EN);

        WRITE_REG(Ports[PortNum].hdmatx->CSAR, (uint32_t)pBuffer);
        MODIFY_REG(Ports[PortNum].hdmatx->CBR1, DMA_CBR1_BNDT, (Size & DMA_CBR1_BNDT));
        SET_BIT(Ports[PortNum].hdmatx->CCR, DMA_CCR_EN);

        LL_UCPD_WriteTxPaySize(Ports[PortNum].husbpd, Size);
        LL_UCPD_SendMessage(Ports[PortNum].husbpd);
      }
    }
  }
  return _status;
}

void USBPD_HW_IF_Send_BIST_Pattern(uint8_t PortNum)
{
  LL_UCPD_SetTxMode(Ports[PortNum].husbpd, LL_UCPD_TXMODE_BIST_CARRIER2);
  LL_UCPD_SendMessage(Ports[PortNum].husbpd);
}
#endif /* !USBPDCORE_LIB_NO_PD */

void USBPDM1_AssertRp(uint8_t PortNum)
{
  switch (Ports[PortNum].params->RpResistor)
  {
    case vRp_Default :
      LL_UCPD_SetRpResistor(Ports[PortNum].husbpd, LL_UCPD_RESISTOR_DEFAULT);
      break;
    case vRp_1_5A:
      LL_UCPD_SetRpResistor(Ports[PortNum].husbpd, LL_UCPD_RESISTOR_1_5A);

      /* If CFG3 register exists */
#if defined(UCPD_CFG3_TRIM_CC1_RP)
      /* The CC pull-up (Rp) must be trimmed to meet the required accuracy.
         The trimming values are saved in the non-volatile memory. */

      /* This is only needed for some devices/rev */
      if (((dev_id == 0x484UL) && (rev_id == 0x1001UL)) ||
          ((dev_id == 0x478UL) && (rev_id == 0x1000UL)))
      {
        CLEAR_BIT(UCPD1->CFG3, (UCPD_CFG3_TRIM_CC1_RP | UCPD_CFG3_TRIM_CC2_RP)); /* Clear bits to change */

        uint32_t temp;
        temp = ((((*pUCPD_TRIM_1A5_CC1) & UCPD_VALUE_TRIM_CC1_RP_1A5_MSK) >>
                 UCPD_VALUE_TRIM_CC1_RP_1A5_POS) << UCPD_CFG3_TRIM_CC1_RP_Pos);
        SET_BIT(UCPD1->CFG3, temp); /* Apply Rp trimming */
        temp = ((((*pUCPD_TRIM_1A5_CC2) & UCPD_VALUE_TRIM_CC2_RP_1A5_MSK) >>
                 UCPD_VALUE_TRIM_CC2_RP_1A5_POS) << UCPD_CFG3_TRIM_CC2_RP_Pos);
        SET_BIT(UCPD1->CFG3, temp); /* Apply Rp trimming */
      }
#endif /* UCPD_CFG3_TRIM_CC1_RP */
      break;
    case vRp_3_0A:
      LL_UCPD_SetRpResistor(Ports[PortNum].husbpd, LL_UCPD_RESISTOR_3_0A);

      /* If CFG3 register exists */
#if defined(UCPD_CFG3_TRIM_CC1_RP)
      /* The CC pull-up (Rp) must be trimmed to meet the required accuracy.
         The trimming values are saved in the non-volatile memory. */

      /* This is only needed for some devices/rev */
      if (((dev_id == 0x484UL) && (rev_id == 0x1001UL)) ||
          ((dev_id == 0x478UL) && (rev_id == 0x1000UL)))
      {
        CLEAR_BIT(UCPD1->CFG3, (UCPD_CFG3_TRIM_CC1_RP | UCPD_CFG3_TRIM_CC2_RP)); /* Clear bits to change */

        uint32_t temp;
        temp = ((((*pUCPD_TRIM_3A0_CC1) & UCPD_VALUE_TRIM_CC1_RP_3A0_MSK) >>
                 UCPD_VALUE_TRIM_CC1_RP_3A0_POS) << UCPD_CFG3_TRIM_CC1_RP_Pos);
        SET_BIT(UCPD1->CFG3, temp); /* Apply Rp trimming */
        temp = ((((*pUCPD_TRIM_3A0_CC2) & UCPD_VALUE_TRIM_CC2_RP_3A0_MSK) >>
                 UCPD_VALUE_TRIM_CC2_RP_3A0_POS) << UCPD_CFG3_TRIM_CC2_RP_Pos);
        SET_BIT(UCPD1->CFG3, temp); /* Apply Rp trimming */
      }
#endif /* UCPD_CFG3_TRIM_CC1_RP */
      break;
    default:
      break;
  }
  LL_UCPD_SetccEnable(Ports[PortNum].husbpd, LL_UCPD_CCENABLE_NONE);
  LL_UCPD_SetSRCRole(Ports[PortNum].husbpd);
  if (CCNONE == Ports[PortNum].CCx)
  {
    LL_UCPD_SetccEnable(Ports[PortNum].husbpd, LL_UCPD_CCENABLE_CC1CC2);
  }
  else
  {
    LL_UCPD_SetccEnable(Ports[PortNum].husbpd,
                        (Ports[PortNum].CCx == CC1) ? LL_UCPD_CCENABLE_CC1 : LL_UCPD_CCENABLE_CC2);
  }

#if defined(TCPP0203_SUPPORT)
  BSP_USBPD_PWR_SetRole(PortNum, POWER_ROLE_SOURCE);
#endif /* TCPP0203_SUPPORT */
}

void USBPDM1_DeAssertRp(uint8_t PortNum)
{
  /* not needed on STM32H5xx, so nothing to do, keep only for compatibility */
  UNUSED(PortNum);
}

void USBPDM1_AssertRd(uint8_t PortNum)
{
  LL_UCPD_TypeCDetectionCC2Disable(Ports[PortNum].husbpd);
  LL_UCPD_TypeCDetectionCC1Disable(Ports[PortNum].husbpd);

  LL_UCPD_SetccEnable(Ports[PortNum].husbpd, LL_UCPD_CCENABLE_NONE);
  LL_UCPD_SetSNKRole(Ports[PortNum].husbpd);
  if (CCNONE == Ports[PortNum].CCx)
  {
    LL_UCPD_SetccEnable(Ports[PortNum].husbpd, LL_UCPD_CCENABLE_CC1CC2);
  }
  else
  {
    LL_UCPD_SetccEnable(Ports[PortNum].husbpd,
                        (Ports[PortNum].CCx == CC1) ? LL_UCPD_CCENABLE_CC1 : LL_UCPD_CCENABLE_CC2);
  }

  HAL_Delay(1);

#ifndef _LOW_POWER
  LL_UCPD_TypeCDetectionCC2Enable(Ports[PortNum].husbpd);
  LL_UCPD_TypeCDetectionCC1Enable(Ports[PortNum].husbpd);
#endif /* _LOW_POWER */

#if defined(TCPP0203_SUPPORT)
  BSP_USBPD_PWR_SetRole(PortNum, POWER_ROLE_SINK);
#endif /* TCPP0203_SUPPORT */
}

void USBPDM1_DeAssertRd(uint8_t PortNum)
{
  /* not needed on STM32H5xx, so nothing to do, keep only for compatibility */
  UNUSED(PortNum);
}

void USBPDM1_EnterErrorRecovery(uint8_t PortNum)
{
  LL_UCPD_SetSRCRole(Ports[PortNum].husbpd);
  LL_UCPD_SetRpResistor(Ports[PortNum].husbpd, LL_UCPD_RESISTOR_NONE);
  LL_UCPD_RxDisable(Ports[PortNum].husbpd);

#if defined(USBPD_REV30_SUPPORT)
  if (Ports[PortNum].settings->PE_PD3_Support.d.PE_FastRoleSwapSupport == USBPD_TRUE)
  {
    /* Set GPIO to disallow the FRSTX handling */
    LL_UCPD_FRSDetectionDisable(Ports[PortNum].husbpd);
  }
#endif /* USBPD_REV30_SUPPORT */
}

void USBPDM1_Set_CC(uint8_t PortNum, CCxPin_TypeDef cc)
{
  /* Set the correct pin on the comparator*/
  Ports[PortNum].CCx = cc;
  LL_UCPD_SetCCPin(Ports[PortNum].husbpd, (cc == CC1) ? LL_UCPD_CCPIN_CC1 : LL_UCPD_CCPIN_CC2);
}

void USBPDM1_RX_EnableInterrupt(uint8_t PortNum)
{
  /* Enable the RX interrupt process */
  MODIFY_REG(Ports[PortNum].husbpd->IMR,
             UCPD_IMR_RXORDDETIE | UCPD_IMR_RXHRSTDETIE | UCPD_IMR_RXOVRIE | UCPD_IMR_RXMSGENDIE,
             UCPD_IMR_RXORDDETIE | UCPD_IMR_RXHRSTDETIE | UCPD_IMR_RXOVRIE | UCPD_IMR_RXMSGENDIE);
  LL_UCPD_RxDMAEnable(Ports[PortNum].husbpd);
}

void USBPD_HW_IF_EnableRX(uint8_t PortNum)
{
  LL_UCPD_RxEnable(Ports[PortNum].husbpd);
}

void USBPD_HW_IF_DisableRX(uint8_t PortNum)
{
  LL_UCPD_RxDisable(Ports[PortNum].husbpd);
}

void HW_SignalAttachement(uint8_t PortNum, CCxPin_TypeDef cc)
{
#if !defined(USBPDCORE_LIB_NO_PD)
  uint32_t _temp;

  /* Init timer to detect the reception of goodCRC */
  USBPD_TIM_Init();

  /* Prepare ucpd to handle PD message
            RX message start listen
            TX prepare the DMA to be transfer ready
            Detection listen only the line corresponding CC=Rd for SRC/SNK */
  Ports[PortNum].hdmatx = USBPD_HW_Init_DMATxInstance(PortNum);
  Ports[PortNum].hdmarx = USBPD_HW_Init_DMARxInstance(PortNum);

  /* Set the RX dma to allow reception */
  _temp = (uint32_t)&Ports[PortNum].husbpd->RXDR;
  WRITE_REG(Ports[PortNum].hdmarx->CSAR, _temp);
  WRITE_REG(Ports[PortNum].hdmarx->CDAR, (uint32_t)Ports[PortNum].ptr_RxBuff);
  MODIFY_REG(Ports[PortNum].hdmarx->CBR1, DMA_CBR1_BNDT, (SIZE_MAX_PD_TRANSACTION_UNCHUNK & DMA_CBR1_BNDT));
  SET_BIT(Ports[PortNum].hdmarx->CCR, DMA_CCR_EN);

  /* Set the TX dma only UCPD address */
  _temp = (uint32_t)&Ports[PortNum].husbpd->TXDR;
  Ports[PortNum].hdmatx->CDAR = _temp;

  /* disabled non Rd line set CC line enable */
#define INTERRUPT_MASK  UCPD_IMR_TXMSGDISCIE | UCPD_IMR_TXMSGSENTIE | UCPD_IMR_HRSTDISCIE  | UCPD_IMR_HRSTSENTIE |  \
  UCPD_IMR_TXMSGABTIE  | UCPD_IMR_TXUNDIE     | UCPD_IMR_RXORDDETIE  | UCPD_IMR_RXHRSTDETIE | \
  UCPD_IMR_RXOVRIE     | UCPD_IMR_RXMSGENDIE

  MODIFY_REG(Ports[PortNum].husbpd->IMR, INTERRUPT_MASK, INTERRUPT_MASK);
#endif /* !USBPDCORE_LIB_NO_PD */

  /* Handle CC enable */
  Ports[PortNum].CCx = cc;

#if !defined(USBPDCORE_LIB_NO_PD)
  /* Set CC pin for PD message */
  LL_UCPD_SetCCPin(Ports[PortNum].husbpd, (Ports[PortNum].CCx == CC1) ? LL_UCPD_CCPIN_CC1 : LL_UCPD_CCPIN_CC2);


#if defined(_VCONN_SUPPORT)
  /* Initialize Vconn management */
  (void)BSP_USBPD_PWR_VCONNInit(PortNum, (Ports[PortNum].CCx == CC1) ? 1u : 2u);
#endif /* _VCONN_SUPPORT */

#if defined(USBPD_REV30_SUPPORT)
  if (Ports[PortNum].settings->PE_PD3_Support.d.PE_FastRoleSwapSupport == USBPD_TRUE)
  {
    /* Set GPIO to allow the FRSTX handling */
    USBPD_HW_SetFRSSignalling(PortNum, (Ports[PortNum].CCx == CC1) ? 1u : 2u);
    LL_UCPD_FRSDetectionEnable(Ports[PortNum].husbpd);
    Ports[PortNum].husbpd->IMR |= UCPD_IMR_FRSEVTIE;
  }
#endif /* USBPD_REV30_SUPPORT */

  /* Disable the Resistor on Vconn PIN */
  if (Ports[PortNum].CCx == CC1)
  {
    LL_UCPD_SetccEnable(Ports[PortNum].husbpd, LL_UCPD_CCENABLE_CC1);
  }
  else
  {
    LL_UCPD_SetccEnable(Ports[PortNum].husbpd, LL_UCPD_CCENABLE_CC2);
  }

  /* Prepare the rx processing */
  LL_UCPD_SetRxMode(Ports[PortNum].husbpd, LL_UCPD_RXMODE_NORMAL);
  LL_UCPD_RxDMAEnable(Ports[PortNum].husbpd);
  LL_UCPD_TxDMAEnable(Ports[PortNum].husbpd);
#endif /* !USBPDCORE_LIB_NO_PD */
}


void HW_SignalDetachment(uint8_t PortNum)
{
#if !defined(USBPDCORE_LIB_NO_PD)
  /* stop DMA RX/TX */
  LL_UCPD_RxDMADisable(Ports[PortNum].husbpd);
  LL_UCPD_TxDMADisable(Ports[PortNum].husbpd);
  LL_UCPD_RxDisable(Ports[PortNum].husbpd);

#if !defined(_LOW_POWER)
  /* Enable only detection interrupt */
  WRITE_REG(Ports[PortNum].husbpd->IMR, UCPD_IMR_TYPECEVT1IE | UCPD_IMR_TYPECEVT2IE);
#else
  if (USBPD_PORTPOWERROLE_SRC == Ports[PortNum].params->PE_PowerRole)
  {
    /* Enable detection interrupt */
    WRITE_REG(Ports[PortNum].husbpd->IMR, UCPD_IMR_TYPECEVT1IE | UCPD_IMR_TYPECEVT2IE);
  }
#endif /* !_LOW_POWER */

  USBPD_HW_DeInit_DMATxInstance(PortNum);
  USBPD_HW_DeInit_DMARxInstance(PortNum);

  LL_UCPD_SetccEnable(Ports[PortNum].husbpd, LL_UCPD_CCENABLE_CC1CC2);

  if (USBPD_PORTPOWERROLE_SNK == Ports[PortNum].params->PE_PowerRole)
  {
#if defined(_VCONN_SUPPORT)
    /* DeInitialize Vconn management */
    (void)BSP_USBPD_PWR_VCONNDeInit(PortNum, (Ports[PortNum].CCx == CC1) ? 1u : 2u);
#endif /* _VCONN_SUPPORT */
    /* DeInitialise VBUS power */
    (void)BSP_USBPD_PWR_VBUSDeInit(PortNum);
  }

#if defined(USBPD_REV30_SUPPORT)
  if (Ports[PortNum].settings->PE_PD3_Support.d.PE_FastRoleSwapSupport == USBPD_TRUE)
  {
    /* Set GPIO to disallow the FRSTX handling */
    LL_UCPD_FRSDetectionDisable(Ports[PortNum].husbpd);
  }
#endif /* USBPD_REV30_SUPPORT */

#endif /* !USBPDCORE_LIB_NO_PD */
  Ports[PortNum].CCx = CCNONE;
#if !defined(USBPDCORE_LIB_NO_PD)
  /* DeInit timer to detect the reception of goodCRC */
  USBPD_TIM_DeInit();
#endif /* !USBPDCORE_LIB_NO_PD */
}

void USBPD_HW_IF_SetResistor_SinkTxNG(uint8_t PortNum)
{
  /* set the resistor SinkTxNG 1.5A5V */
  LL_UCPD_SetRpResistor(Ports[PortNum].husbpd, LL_UCPD_RESISTOR_1_5A);

  /* If CFG3 register exists */
#if defined(UCPD_CFG3_TRIM_CC1_RP)
  /* The CC pull-up (Rp) must be trimmed to meet the required accuracy.
     The trimming values are saved in the non-volatile memory. */

  /* This is only needed for some devices/rev */
  if (((dev_id == 0x484UL) && (rev_id == 0x1001UL)) ||
      ((dev_id == 0x478UL) && (rev_id == 0x1000UL)))
  {
    CLEAR_BIT(UCPD1->CFG3, (UCPD_CFG3_TRIM_CC1_RP | UCPD_CFG3_TRIM_CC2_RP)); /* Clear bits to change */

    uint32_t temp;
    temp = ((((*pUCPD_TRIM_1A5_CC1) & UCPD_VALUE_TRIM_CC1_RP_1A5_MSK) >>
             UCPD_VALUE_TRIM_CC1_RP_1A5_POS) << UCPD_CFG3_TRIM_CC1_RP_Pos);
    SET_BIT(UCPD1->CFG3, temp); /* Apply Rp trimming */
    temp = ((((*pUCPD_TRIM_1A5_CC2) & UCPD_VALUE_TRIM_CC2_RP_1A5_MSK) >>
             UCPD_VALUE_TRIM_CC2_RP_1A5_POS) << UCPD_CFG3_TRIM_CC2_RP_Pos);
    SET_BIT(UCPD1->CFG3, temp); /* Apply Rp trimming */
  }
#endif /* UCPD_CFG3_TRIM_CC1_RP */
}

void USBPD_HW_IF_SetResistor_SinkTxOK(uint8_t PortNum)
{
  /* set the resistor SinkTxNG 3.0A5V */
  LL_UCPD_SetRpResistor(Ports[PortNum].husbpd, LL_UCPD_RESISTOR_3_0A);

  /* If CFG3 register exists */
#if defined(UCPD_CFG3_TRIM_CC1_RP)
  /* The CC pull-up (Rp) must be trimmed to meet the required accuracy.
     The trimming values are saved in the non-volatile memory. */

  /* This is only needed for some devices/rev */
  if (((dev_id == 0x484UL) && (rev_id == 0x1001UL)) ||
      ((dev_id == 0x478UL) && (rev_id == 0x1000UL)))
  {
    CLEAR_BIT(UCPD1->CFG3, (UCPD_CFG3_TRIM_CC1_RP | UCPD_CFG3_TRIM_CC2_RP)); /* Clear bits to change */

    uint32_t temp;
    temp = ((((*pUCPD_TRIM_3A0_CC1) & UCPD_VALUE_TRIM_CC1_RP_3A0_MSK) >>
             UCPD_VALUE_TRIM_CC1_RP_3A0_POS) << UCPD_CFG3_TRIM_CC1_RP_Pos);
    SET_BIT(UCPD1->CFG3, temp); /* Apply Rp trimming */
    temp = ((((*pUCPD_TRIM_3A0_CC2) & UCPD_VALUE_TRIM_CC2_RP_3A0_MSK) >>
             UCPD_VALUE_TRIM_CC2_RP_3A0_POS) << UCPD_CFG3_TRIM_CC2_RP_Pos);
    SET_BIT(UCPD1->CFG3, temp); /* Apply Rp trimming */
  }
#endif /* UCPD_CFG3_TRIM_CC1_RP */
}

uint8_t USBPD_HW_IF_IsResistor_SinkTxOk(uint8_t PortNum)
{
#if defined(_LOW_POWER)
  /* When in low power mode, the type C state machine is turned off.
     To retrieve any potential updates of the SR register, the state machine needs to be re-enabled briefly. */

  /* Enable type C state machine */
  CLEAR_BIT(Ports[PortNum].husbpd->CR, (UCPD_CR_CC1TCDIS | UCPD_CR_CC2TCDIS));

  /* Let time for internal state machine to refresh his state */
  for (uint32_t index = 0; index < CAD_DELAY_READ_CC_STATUS; index++)
  {
    __DSB();
  }

  /* Disable type C state machine */
  SET_BIT(Ports[PortNum].husbpd->CR, (UCPD_CR_CC1TCDIS | UCPD_CR_CC2TCDIS));
#endif /* _LOW_POWER */

  switch (Ports[PortNum].CCx)
  {
    case CC1 :
      if ((Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC1) == LL_UCPD_SNK_CC1_VRP30A)
      {
        return USBPD_TRUE;
      }
      break;
    case CC2 :
      if ((Ports[PortNum].husbpd->SR & UCPD_SR_TYPEC_VSTATE_CC2) == LL_UCPD_SNK_CC2_VRP30A)
      {
        return USBPD_TRUE;
      }
      break;
    default:
      break;
  }

  return USBPD_FALSE;
}

void USBPD_HW_IF_FastRoleSwapSignalling(uint8_t PortNum)
{
  LL_UCPD_SignalFRSTX(Ports[PortNum].husbpd);
}
