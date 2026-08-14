
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DC_UM324xF_H
#define __USB_DC_UM324xF_H

/* Includes ------------------------------------------------------------------*/
#include "um324xF.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#if !defined(UNUSED)
#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
#endif /* UNUSED */

/* Exported macro ------------------------------------------------------------*/

/* endpoints enumeration */
#define ENDP0       ((uint8_t)0)
#define ENDP1       ((uint8_t)1)
#define ENDP2       ((uint8_t)2)
#define ENDP3       ((uint8_t)3)
#define ENDP4       ((uint8_t)4)

/******************************************************************************/
/*             WORKINGMODE register bits definitions                          */
/******************************************************************************/
#define USB_SPEED_MODE                (0x00000001)
#define USB_USB_SUSPEND               (0x00000002)
#define USB_FORCE_RST                 (0x00000004)
#define USB_BUS_AUTO_RST_EN           (0x00000008)
#define USB_DPPU                      (0x00000010)
#define USB_DPPU_LO                   (0x00000040)
#define USB_LINE_STATE_DM             (0x00000100)
#define USB_LINE_STATE_DP             (0x00000200)
#define USB_CRCERR_NAK_EN             (0x00000400)
#define USB_IN_TIMEOUT_NAK_EN         (0x00000800)
#define USB_MORETHAN64_NAK_EN         (0x00001000)
#define USB_REMOTE_WAKEUP             (0x00100000)
#define USB_TOKEN_NOEOP_NAK_EN        (0x00200000)
#define USB_DATA_NOEOP_NAK_EN         (0x00400000)
#define USB_TOKEN_NOEOP_EN            (0x00800000)
#define USB_DATA_NOEOP_EN             (0x01000000)
#define USB_EP0_ZOD_INTR_EN           (0x02000000)

/******************************************************************************/
/*             INTSTATRAW/INTCLR/INTCLR register bits definitions             */
/******************************************************************************/
#define INT_BUS_RESET               (0x00000001)
#define INT_SUSPEND                 (0x00000002)
#define INT_RESUME                  (0x00000004)
#define INT_SOF                     (0x00000008)
#define INT_SETUPTOK                (0x00000010)
#define INT_SUDAV                   (0x00000020)
#define INT_EP0_IN                  (0x00000040)
#define INT_EP0_OUT                 (0x00000080)
#define INT_EP0_ACK                 (0x00000100)
#define INT_EP1_IN                  (0x00000200)
#define INT_EP1_OUT                 (0x00000400)
#define INT_EP1_ACK                 (0x00000800)
#define INT_EP2_IN                  (0x00001000)
#define INT_EP2_OUT                 (0x00002000)
#define INT_EP2_ACK                 (0x00004000)
#define INT_EP3_IN                  (0x00008000)
#define INT_EP3_OUT                 (0x00010000)
#define INT_EP3_ACK                 (0x00020000)
#define INT_EP4_IN                  (0x00040000)
#define INT_EP4_OUT                 (0x00080000)
#define INT_EP4_ACK                 (0x00100000)
#define INT_TURNAROUND_ERROR        (0x00200000)
#define INT_SETADDR                 (0x00400000)
#define INT_CRC_ERR                 (0x00800000)
#define INT_DATA_BYTE_MORETHAN_64   (0x01000000)
#define INT_EP0_IN_HANDSHAKE_ERR    (0x02000000)
#define INT_EP1_IN_HANDSHAKE_ERR    (0x04000000)
#define INT_EP2_IN_HANDSHAKE_ERR    (0x08000000)
#define INT_EP3_IN_HANDSHAKE_ERR    (0x10000000)
#define INT_EP4_IN_HANDSHAKE_ERR    (0x20000000)
#define INT_NOEOP_ERR               (0x40000000)
#define INT_TOGGLE_STATE_ERR        (0x80000000)

#define INT_ALL                     (0xFFFFFFFF)
#define INT_ACK                     (INT_EP0_ACK | INT_EP1_ACK | INT_EP2_ACK | INT_EP3_ACK | INT_EP4_ACK)

#define EP_RXTX_STALL  (0x00001000) /* EndPoint TX STALLed */
#define EP_TX_VALID    (0x00000400) /* EndPoint TX VALID */
#define EP_RX_VALID    (0x00000800) /* EndPoint TX VALID */

#define IMR_MSK (INT_ACK | INT_RESUME | INT_SUSPEND | INT_SOF | INT_BUS_RESET)

/* Exported macro ------------------------------------------------------------*/

//#define USBD USB

/* ClrFlag */
#define _ClrFlag(wRegValue)  (USBD->INTCLR = wRegValue)

/* GetFlag */
#define _GetFlag()   (USBD->INTSTATRAW)

/* SetWORKINGMODE */
#define _SetWORKINGMODE(wRegValue)  (USBD->WORKINGMODE = wRegValue)

/* GetWORKINGMODE */
#define _GetWORKINGMODE()  (USBD->WORKINGMODE)

/* SetINTEN */
#define _SetINTEN(wRegValue)  (USBD->INTEN = wRegValue)

/* GetSETUP03DATA */
#define _GetSETUP03DATA()   (USBD->SETUP03DATA)

/* GetSETUP47DATA */
#define _GetSETUP47DATA()   (USBD->SETUP47DATA)

/*******************************************************************************
* Macro Name     : _GetEPIndex.
* Description    : gets EP index.
* Input          : bEpNum: endpoint number.
* Output         : None.
* Return         : Counter value.
*******************************************************************************/
#define _GetEPIndex(wRegValue)		    ((wRegValue & INT_EP0_ACK) ? ENDP0 :  \
                                        \
                                        ((wRegValue & INT_EP1_ACK) ? ENDP1 :  \
                                        \
                                        ((wRegValue & INT_EP2_ACK) ? ENDP2 :  \
                                        \
                                        ((wRegValue & INT_EP3_ACK) ? ENDP3 : ENDP4))))
																				
/*******************************************************************************
* Macro Name     : SetEPTxCount.
* Description    : sets counter for the tx buffer.
* Input          : bEpNum: endpoint number.
*                  wCount: Counter value.
* Output         : None.
* Return         : None.
*******************************************************************************/
#define _SetEPTxCount(bEpNum, wCount)  (USBD->EPSENDBN[bEpNum].EPSENDBN_bit.SEND_BYTE = wCount)

/*******************************************************************************
* Macro Name     : GetEPRxCount.
* Description    : gets counter of the rx buffer.
* Input          : bEpNum: endpoint number.
* Output         : None.
* Return         : Counter value.
*******************************************************************************/
#define _GetEPRxCount(bEpNum)   ((uint8_t)(USBD->EPCSR[bEpNum].EPCSR_bit.RECEIVED_BYTE))

/*******************************************************************************
* Macro Name     : GetEPTxCount.
* Description    : gets counter of the Tx buffer.
* Input          : bEpNum: endpoint number.
* Output         : None.
* Return         : Counter value.
*******************************************************************************/
#define _GetEPTxCount(bEpNum)   ((uint8_t)(USBD->EPSENDBN[bEpNum].EPSENDBN_bit.SEND_BYTE))

/*******************************************************************************
* Macro Name     : GetEPTxAddr / GetEPRxAddr.
* Description    : Gets address of the tx/rx buffer.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : address of the buffer.
*******************************************************************************/
#define _GetEPTxAddr(bEpNum) ((uint32_t)&USBD->EPMEM[bEpNum])
#define _GetEPRxAddr(bEpNum) ((uint32_t)&USBD->EPMEM[bEpNum])

/*******************************************************************************
* Macro Name     : IsEPOut.
* Description    : is EP out.
* Input          : bEpNum: endpoint number.
* Output         : None.
* Return         : Counter value.
*******************************************************************************/
#define _IsEPOut(wRegValue, bEpIndex)	  ((bEpIndex == ENDP0) ? (wRegValue & INT_EP0_OUT) :  \
                                          \
                                          ((bEpIndex == ENDP1) ? (wRegValue & INT_EP1_OUT) :  \
                                          \
                                          ((bEpIndex == ENDP2) ? (wRegValue & INT_EP2_OUT) :  \
                                          \
                                          ((bEpIndex == ENDP3) ? (wRegValue & INT_EP3_OUT) :  (wRegValue & INT_EP4_OUT)))))


/*******************************************************************************
* Macro Name     : SetEPOpen/SetEPClose.
* Description    : sets ep open/close.
* Input          : bEpNum: endpoint number.
* Output         : None.
* Return         : None.
*******************************************************************************/
#define _SetEPOpen(bEpNum)   (USBD->EPCSR[bEpNum].EPCSR_bit.EN = 1)
#define _SetEPClose(bEpNum)  (USBD->EPCSR[bEpNum].EPCSR_bit.EN = 0)

/*******************************************************************************
* Macro Name     : SetEPStall/ClrEPStall.
* Description    : set/clr ep stall.
* Input          : bEpNum: endpoint number.
* Output         : None.
* Return         : None.
*******************************************************************************/
#define _SetEPStall(bEpNum)   (USBD->EPCSR[bEpNum].EPCSR_bit.SEND_STALL = 1)
#define _ClrEPStall(bEpNum)   (USBD->EPCSR[bEpNum].EPCSR_bit.SEND_STALL = 0)

/*******************************************************************************
* Macro Name     : _IsEPStall.
* Description    : is ep stall.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : stall.
*******************************************************************************/
#define _IsEPStall(bEpNum)	((uint8_t)USBD->EPCSR[bEpNum].EPCSR_bit.SEND_STALL)

/*******************************************************************************
* Macro Name     : SetTxValid/SetRxValid.
* Description    : set Tx/Rx valid.
* Input          : bEpNum: endpoint number.
* Output         : None.
* Return         : None.
*******************************************************************************/
#define _SetTxValid(bEpNum)   (USBD->EPCSR[bEpNum].EPCSR_bit.DATA_START = 1)
#define _SetRxValid(bEpNum)   (USBD->EPCSR[bEpNum].EPCSR_bit.RECEIVED_DONE = 1)


/* Exported functions ------------------------------------------------------- */
void um324xf_write_pma(uint8_t *pbUsrBuf, uint32_t wPMABufAddr, uint8_t wNBytes);
void um324xf_read_pma(uint8_t *pbUsrBuf, uint32_t wPMABufAddr, uint8_t wNBytes);

/* Private variables --------------------------------------------------------- */

#endif /* __USB_DC_UM324xF_H */
