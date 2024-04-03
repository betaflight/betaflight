/**
  ******************************************************************************
  * @file    usbpd_def.h
  * @author  MCD Application Team
  * @brief   Global defines for USB-PD library
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef USBPD_DEF_H_
#define USBPD_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cmsis_compiler.h"

#include <stdint.h>
#include <stddef.h>

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_DEF
  * @{
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup USBPD_CORE_DEF_Exported_Macros USBPD CORE DEF Exported Macros
  * @{
  */
/**
  * @brief  Compare two variables and return the smallest
  * @param  __VAR1__ First variable to be compared
  * @param  __VAR2__ Second variable to be compared
  * @retval Returns the smallest variable
  */
#define USBPD_MIN(__VAR1__, __VAR2__) (((__VAR1__) > (__VAR2__))?(__VAR2__):(__VAR1__))

/**
  * @brief  Compare two variables and return the biggest
  * @param  __VAR1__ First variable to be compared
  * @param  __VAR2__ Second variable to be compared
  * @retval Returns the biggest variable
  */
#define USBPD_MAX(__VAR1__, __VAR2__) (((__VAR1__) < (__VAR2__))?(__VAR2__):(__VAR1__))

/**
  * @brief  Check if the requested voltage is valid
  * @param  __MV__    Requested voltage in mV units
  * @param  __MAXMV__ Max Requested voltage in mV units
  * @param  __MINMV__ Min Requested voltage in mV units
  * @retval 1 if valid voltage else 0
  */
#define USBPD_IS_VALID_VOLTAGE(__MV__, __MAXMV__, __MINMV__) ((((__MV__) <= (__MAXMV__))\
                                                               && ((__MV__) >= (__MINMV__)))? 1U: 0U)

#define DIV_ROUND_UP(x, y) (((x) + ((y) - 1u)) / (y))
#define MV2ADC(__X__)           ( (__X__*4095) / 3300 )
#define ADC2MV(__X__)           ( (__X__*3300) / 4095 )

/* Macros for integer division with various rounding variants default integer
   division rounds down. */
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

#define USBPD_WRITE32(addr,data)                                                   \
  do {                                                                             \
    uint8_t bindex;                                                                \
    for(bindex = 0u; bindex < 4u; bindex++)                                        \
    {                                                                              \
      ((uint8_t *)addr)[bindex] = ((uint8_t)(data >> (8U * bindex)) & 0x000000FFU);\
    }                                                                              \
  } while(0u);

#define USPBPD_WRITE32 USBPD_WRITE32 /* For legacy purpose */

#define USBPD_LE16(addr) (((uint16_t)(*((uint8_t *)(addr))))\
                          + (((uint16_t)(*(((uint8_t *)(addr)) + 1U))) << 8U))

#define USBPD_LE32(addr) ((((uint32_t)(*(((uint8_t *)(addr)) + 0U))) +         \
                           (((uint32_t)(*(((uint8_t *)(addr)) + 1U))) << 8U) + \
                           (((uint32_t)(*(((uint8_t *)(addr)) + 2U))) << 16U)+ \
                           (((uint32_t)(*(((uint8_t *)(addr)) + 3U))) << 24U)))

/**
  * @}
  */

/* Exported defines   --------------------------------------------------------*/
/** @defgroup USBPD_CORE_DEF_Exported_Defines USBPD CORE DEF Exported Defines
  * @{
  */
#define USBPD_TRUE  1U
#define USBPD_FALSE 0U

/** @defgroup USBPD_CORE_DEF_Exported_Defines_Swiches USBPD Compilations switches
  * @brief List of compilation switches which can be used to reduce size of the CORE library
  * @{
  */
#if defined(USBPDCORE_LIB_PD3_CONFIG_3)
#define USBPDCORE_BIST
#define USBPDCORE_GIVESNKCAP
#define USBPDCORE_GETSRCCAP
#define USBPDCORE_ERROR_RECOVERY
#define USBPDCORE_SNK
#define USBPDCORE_SNK_CAPA_EXT
#define USBPD_REV30_SUPPORT
#define USBPDCORE_PECABLE
#define USBPDCORE_PPS
#define USBPDCORE_SVDM
#define USBPDCORE_UVDM
#elif defined(USBPDCORE_LIB_NO_PD)
#define USBPDCORE_NOPD
#else
/* Default Switch */
#define USBPDCORE_GOTOMIN
#define USBPDCORE_BIST
#define USBPDCORE_GETSNKCAP
#define USBPDCORE_GETSRCCAP
#define USBPDCORE_GIVESNKCAP
#define USBPDCORE_ERROR_RECOVERY

#if defined(USBPDCORE_LIB_PD3_FULL) || defined(USBPDCORE_LIB_PD3_CONFIG_1) || defined(USBPD_TCPM_LIB_PD3_FULL) ||      \
    defined(USBPD_TCPM_LIB_PD3_CONFIG_1) || defined(USBPDCORE_LIB_PD3_CONFIG_MINSRC) ||                                \
    defined(USBPDCORE_LIB_PD3_CONFIG_MINSNK) || defined(USBPDCORE_LIB_PD3_CONFIG_MINDRP)

/*
   USBPDCORE_LIB_PD3_FULL
   USBPDCORE_LIB_PD3_CONFIG_1  : NO VDM
   USBPDCORE_LIB_PD3_CONFIG_MINSRC : ONLY SRC & VCONN and  NO option
   USBPDCORE_LIB_PD3_CONFIG_MINSNK : ONLY SNK, and NO option
   USBPDCORE_LIB_PD3_CONFIG_MINDRP : SRC + SNK + VCONN, and NO option
   USBPD_TCPM_LIB_PD3_FULL     : PD3.0 TCPM FULL
   USBPD_TCPM_LIB_PD3_CONFIG_1 : PD3.0 TCPM FULL without VDM
*/
#define USBPD_REV30_SUPPORT

#if !defined(USBPDCORE_LIB_PD3_CONFIG_MINSNK)
#define USBPDCORE_SRC
#define USBPDCORE_VCONN_SUPPORT
#endif /* USBPDCORE_LIB_PD3_CONFIG_MINSNK */

#if !defined(USBPDCORE_LIB_PD3_CONFIG_MINSRC)
#define USBPDCORE_SNK
#define USBPDCORE_SNK_CAPA_EXT
#endif /* USBPDCORE_LIB_PD3_CONFIG_MINSRC */

#if defined(USBPDCORE_LIB_PD3_FULL) || defined(USBPD_TCPM_LIB_PD3_FULL)
#define USBPDCORE_SVDM
#endif /* USBPDCORE_LIB_PD3_FULL || USBPD_TCPM_LIB_PD3_FULL */

#if defined(USBPDCORE_LIB_PD3_CONFIG_MINSNK)||defined(USBPDCORE_LIB_PD3_CONFIG_MINSRC)
#else
#define USBPDCORE_DRP
#define USBPDCORE_DATA_SWAP
#if !defined(USBPDCORE_LIB_PD3_CONFIG_MINDRP)
#define USBPDCORE_UVDM
#define USBPDCORE_FASTROLESWAP
#define USBPDCORE_PPS
#define USBPDCORE_ALERT
#define USBPDCORE_SRC_CAPA_EXT
#define USBPDCORE_STATUS
#define USBPDCORE_BATTERY
#define USBPDCORE_MANU_INFO
#define USBPDCORE_SECURITY_MSG
#define USBPDCORE_FWUPD
#define USBPDCORE_COUNTRY_MSG
#define USBPDCORE_PING_SUPPORT
#endif /* !USBPDCORE_LIB_PD3_CONFIG_MINDRP */
#endif /* USBPDCORE_LIB_PD3_CONFIG_MINSNK || USBPDCORE_LIB_PD3_CONFIG_MINSRC */

#if defined(USBPD_TCPM_LIB_PD3_FULL) || defined(USBPD_TCPM_LIB_PD3_CONFIG_1)
#define USBPDCORE_TCPM_SUPPORT
#endif /* TCPM */

#endif /* PD3.0 */

/* List of compilation switches which can be used to reduce size of the CORE library */
#if defined(USBPDCORE_LIB_PD2_FULL) || defined(USBPDCORE_LIB_PD2_CONFIG_1) ||                                          \
    defined(USBPDCORE_LIB_PD2_CONFIG_MINSRC) || defined(USBPDCORE_LIB_PD2_CONFIG_MINSNK) ||                            \
    defined(USBPD_TCPM_LIB_PD2_FULL) || defined(USBPD_TCPM_LIB_PD2_CONFIG_1) || defined(USBPD_TCPM_LIB_PD2_MINSRC) ||  \
    defined(USBPD_TCPM_LIB_PD2_MINSNK)
/*
   USBPDCORE_LIB_PD2_FULL
   USBPDCORE_LIB_PD2_CONFIG_1 : NO VDM
   USBPDCORE_LIB_PD2_CONFIG_MINSRC : ONLY SRC & VCONN and  NO option
   USBPDCORE_LIB_PD2_CONFIG_MINSNK : ONLY SNK, and NO option
   USBPD_TCPM_LIB_PD2_FULL     : PD2.0 TCPM FULL
   USBPD_TCPM_LIB_PD2_CONFIG_1 : PD2.0 TCPM FULL without VDM
   USBPD_TCPM_LIB_PD2_MINSRC : PD2.0 TCPM Only SRC
   USBPD_TCPM_LIB_PD2_MINSNK : PD2.0 TCPM Only SNK
*/
#define USBPD_REV20_SUPPORT

#if !defined(USBPDCORE_LIB_PD2_CONFIG_MINSNK) && !defined(USBPD_TCPM_LIB_PD2_MINSNK)
#define USBPDCORE_SRC
#define USBPDCORE_VCONN_SUPPORT
#endif /* !defined(USBPDCORE_LIB_PD2_CONFIG_MINSNK) && !defined(USBPD_TCPM_LIB_PD2_MINSNK) */

#if !defined(USBPDCORE_LIB_PD2_CONFIG_MINSRC) && !defined(USBPD_TCPM_LIB_PD2_MINSRC)
#define USBPDCORE_SNK
#endif /* !defined(USBPDCORE_LIB_PD2_CONFIG_MINSRC) && !defined(USBPD_TCPM_LIB_PD2_MINSRC)*/

#if defined(USBPDCORE_LIB_PD2_CONFIG_MINSRC) || defined(USBPDCORE_LIB_PD2_CONFIG_MINSNK) ||                            \
    defined(USBPD_TCPM_LIB_PD2_MINSRC) || defined(USBPD_TCPM_LIB_PD2_MINSNK)
#else
#define USBPDCORE_DRP
#define USBPDCORE_DATA_SWAP
#define USBPDCORE_UVDM
#endif /* USBPDCORE_LIB_PD2_CONFIG_MINSRC || USBPDCORE_LIB_PD2_CONFIG_MINSNK ||
          USBPD_TCPM_LIB_PD2_MINSRC || USBPD_TCPM_LIB_PD2_MINSNK   */

#if defined(USBPDCORE_LIB_PD2_FULL) || defined(USBPD_TCPM_LIB_PD2_FULL)
#define USBPDCORE_SVDM
#endif /* USBPDCORE_LIB_PD3_FULL || USBPD_TCPM_LIB_PD2_FULL */

#if defined(USBPD_TCPM_LIB_PD2_FULL) || defined(USBPD_TCPM_LIB_PD2_CONFIG_1) || defined(USBPD_TCPM_LIB_PD2_MINSRC) ||  \
    defined(USBPD_TCPM_LIB_PD2_MINSNK)
#define USBPDCORE_TCPM_SUPPORT
#endif /* TCPM */

#endif /* PD2.0 */

#if defined(USBPDCORE_LIB_PD3_CONFIG_2)
#undef USBPDCORE_GOTOMIN
#undef USBPDCORE_BIST
#undef USBPDCORE_GETSNKCAP
#undef USBPDCORE_GETSRCCAP
#undef USBPDCORE_GIVESNKCAP
#undef USBPDCORE_SNK_CAPA_EXT

#define USBPDCORE_SNK
#define USBPD_REV30_SUPPORT
#define USBPDCORE_FWUPD
#define USBPDCORE_UVDM
#endif /* USBPDCORE_LIB_PD3_CONFIG_2 */

/* No need to enable USBPDCORE_UNCHUNCKED_MODE
  if FW Update et Security messages are not supported by the configuration */
#if defined(USBPDCORE_SECURITY_MSG) || defined(USBPDCORE_FWUPD)
#define USBPDCORE_UNCHUNCKED_MODE
#endif /* USBPDCORE_SECURITY_MSG || USBPDCORE_FWUPD */

#endif /* USBPDCORE_LIB_NO_PD */



/* _LIB_ID definition */
/*
  _LIB_ID constructs like this: 0xXYVVVWWW
  * X: 3 (PD3.0) or 2 (PD2.0)
  * Y: 0 (CORE) or 1 (TCPM)
  * VVV: Stack version (ex 200 for Stack 2.0.0)
  * WWW: 0 (FULL VERSION) or config_x
*/
/* Defines for PD revision */
#define LIB_PD_VERSION_POS  28U
#define LIB_PD_VERSION_MSK  (0xFU << LIB_PD_VERSION_POS)
#define LIB_PD2             (2U   << LIB_PD_VERSION_POS)
#define LIB_PD3             (3U   << LIB_PD_VERSION_POS)
/* Defines for CORE or TCPM */
#define LIB_CORE_TCPM_POS   24U
#define LIB_CORE_TCPM_MSK   (0xFU << LIB_CORE_TCPM_POS)
#define LIB_CORE            (0U   << LIB_CORE_TCPM_POS)
#define LIB_TCPM            (1U   << LIB_CORE_TCPM_POS)
/* Defines for STACK version */
#define LIB_STACK_VER_POS   12U
#define LIB_STACK_VER_MSK   (0xFFFU << LIB_STACK_VER_POS)
#define LIB_STACK_VER       (0x410U  << LIB_STACK_VER_POS)
/* Defines for configuration */
#define LIB_CONFIG_MSK      0xFFFU
#define LIB_FULL            0x000U
#define LIB_CONFIG_1        0x001U
#define LIB_CONFIG_MINSRC   0x002U
#define LIB_CONFIG_MINSNK   0x004U
#define LIB_CONFIG_2        0x010U
#define LIB_CONFIG_NOPD     0x100U
#define LIB_CONFIG_MINDRP   0x200U
#define LIB_CONFIG_3        0x400U

#define _LIB_ID LIB_ID /* done for CubeMX compatibility purpose */

#ifdef USBPDCORE_LIB_PD3_CONFIG_3
#define LIB_ID   (LIB_PD3 | LIB_CORE | LIB_STACK_VER | LIB_CONFIG_3)
#endif /* USBPDCORE_LIB_PD3_CONFIG_3*/
#ifdef USBPDCORE_LIB_PD3_FULL
#define LIB_ID   (LIB_PD3 | LIB_CORE | LIB_STACK_VER | LIB_FULL)
#endif /* USBPDCORE_LIB_PD3_FULL */
#ifdef USBPDCORE_LIB_PD3_CONFIG_1
#define LIB_ID   (LIB_PD3 | LIB_CORE | LIB_STACK_VER | LIB_CONFIG_1)
#endif /* USBPDCORE_LIB_PD3_CONFIG_1 */
#ifdef USBPDCORE_LIB_PD2_FULL
#define LIB_ID   (LIB_PD2 | LIB_CORE | LIB_STACK_VER | LIB_FULL)
#endif /* USBPDCORE_LIB_PD2_FULL */
#ifdef USBPDCORE_LIB_PD2_CONFIG_1
#define LIB_ID   (LIB_PD2 | LIB_CORE | LIB_STACK_VER | LIB_CONFIG_1)
#endif /* USBPDCORE_LIB_PD2_CONFIG_1 */
#ifdef USBPDCORE_LIB_PD2_CONFIG_MINSRC
#define LIB_ID   (LIB_PD2 | LIB_CORE | LIB_STACK_VER | LIB_CONFIG_MINSRC)
#endif /* USBPDCORE_LIB_PD2_CONFIG_MINSRC */
#ifdef USBPDCORE_LIB_PD3_CONFIG_MINSRC
#define LIB_ID   (LIB_PD3 | LIB_CORE | LIB_STACK_VER | LIB_CONFIG_MINSRC)
#endif /* USBPDCORE_LIB_PD3_CONFIG_MINSRC */
#ifdef USBPDCORE_LIB_PD3_CONFIG_MINDRP
#define LIB_ID   (LIB_PD3 | LIB_CORE | LIB_STACK_VER | LIB_CONFIG_MINDRP)
#endif /* USBPDCORE_LIB_PD3_CONFIG_MINSRC */
#ifdef USBPDCORE_LIB_PD2_CONFIG_MINSNK
#define LIB_ID   (LIB_PD2 | LIB_CORE | LIB_STACK_VER | LIB_CONFIG_MINSNK)
#endif /* USBPDCORE_LIB_PD2_CONFIG_MINSNK */
#ifdef USBPDCORE_LIB_PD3_CONFIG_MINSNK
#define LIB_ID   (LIB_PD3 | LIB_CORE | LIB_STACK_VER | LIB_CONFIG_MINSNK)
#endif /* USBPDCORE_LIB_PD3_CONFIG_MINSNK */
#ifdef USBPD_TCPM_LIB_PD2_FULL
#define LIB_ID   (LIB_PD2 | LIB_TCPM | LIB_STACK_VER | LIB_FULL)
#endif /* USBPD_TCPM_LIB_PD2_FULL */
#ifdef USBPD_TCPM_LIB_PD2_MINSRC
#define LIB_ID   (LIB_PD2 | LIB_TCPM | LIB_STACK_VER | LIB_CONFIG_MINSRC)
#endif /* USBPD_TCPM_LIB_PD2_MINSRC */
#ifdef USBPD_TCPM_LIB_PD2_MINSNK
#define LIB_ID   (LIB_PD2 | LIB_TCPM | LIB_STACK_VER | LIB_CONFIG_MINSNK)
#endif /* USBPD_TCPM_LIB_PD2_MINSNK */
#ifdef USBPD_TCPM_LIB_PD2_CONFIG_1
#define LIB_ID   (LIB_PD2 | LIB_TCPM | LIB_STACK_VER | LIB_CONFIG_1)
#endif /* USBPD_TCPM_LIB_PD2_CONFIG_1 */
#ifdef USBPD_TCPM_LIB_PD3_CONFIG_1
#define LIB_ID   (LIB_PD3 | LIB_TCPM | LIB_STACK_VER | LIB_CONFIG_1)
#endif /* USBPD_TCPM_LIB_PD3_CONFIG_1 */
#ifdef USBPD_TCPM_LIB_PD3_FULL
#define LIB_ID   (LIB_PD3 | LIB_TCPM | LIB_STACK_VER | LIB_FULL)
#endif /* USBPD_TCPM_LIB_PD3_FULL */
#if defined(USBPDCORE_LIB_PD3_CONFIG_2)
#define LIB_ID   (LIB_PD3 | LIB_TCPM | LIB_STACK_VER | LIB_CONFIG_2)
#endif /* USBPDCORE_LIB_PD3_CONFIG_2 */
#if defined(USBPDCORE_LIB_NO_PD)
#define LIB_ID   (LIB_STACK_VER | LIB_CONFIG_NOPD)
#endif /* USBPDCORE_LIB_NO_PD */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
#define USBPD_PORT_0           (0U)    /*!< Port 0 identifier */
#define USBPD_PORT_1           (1U)    /*!< Port 1 identifier */
#define USBPD_PORT_2           (2U)    /*!< Port 2 identifier */

#define USBPD_MAX_NB_PDO       (7U)    /*!< Maximum number of supported Power Data Objects: fix by the Specification */
#define BIST_CARRIER_MODE_MS   (50U)   /*!< Time in ms of the BIST signal*/

/*
  @brief Maximum size of the RX buffer allocated in the stack to receive a PD frame
  @note TX buffer size is used internally in the stack (size if available in @ref USBPD_PHY_SendMessage)
   */
#if defined(USBPDCORE_UNCHUNCKED_MODE)
#define USBPD_MAX_RX_BUFFER_SIZE (264U) /*!< Maximum size of Rx buffer used when unchuncked is supported by the stack */
#else
#define USBPD_MAX_RX_BUFFER_SIZE (30U) /*!< Maximum size of Rx buffer used when unchuncked
                                            is NOT supported by the stack */
#endif /* USBPDCORE_UNCHUNCKED_MODE */

/*
 * Maximum size of a Power Delivery packet (in bits on the wire) :
 *    16-bit header + 0..7 32-bit data objects  (+ 4b5b encoding)
 *    64-bit preamble + SOP (4x 5b) + header (16-bit) + message in 4b5b + 32-bit CRC  + EOP (1x 5b)
 * =  64bit           + 4*5bit      + 16bit * 5/4 + 7 * 32bit * 5/4 + 32bit * 5/4 + 5
 */
#define PHY_BIT_LEN             ((uint16_t)429U)
#define PHY_MAX_RAW_SIZE        ((uint16_t)((PHY_BIT_LEN*2u) + 3U))
#define PHY_MAX_RAW_BYTE_SIZE   ((uint8_t)60U) /*!<PHY_BIT_LEN / 8 + SAFE Bytes */

/** @defgroup USBPD_PDO_Index_And_Mask_Constants Index and Mask constants used in PDO bits handling
  * @{
  */
#define USBPD_PDO_TYPE_Pos       (30U)                                                         /*!< PDO Type bits position                          */
#define USBPD_PDO_TYPE_Msk       (0x3U << USBPD_PDO_TYPE_Pos)                                  /*!< PDO Type bits mask : 0xC0000000                 */
#define USBPD_PDO_TYPE_FIXED     (uint32_t)(USBPD_CORE_PDO_TYPE_FIXED << USBPD_PDO_TYPE_Pos)   /*!< PDO Type = FIXED                                */
#define USBPD_PDO_TYPE_BATTERY   (uint32_t)(USBPD_CORE_PDO_TYPE_BATTERY << USBPD_PDO_TYPE_Pos) /*!< PDO Type = BATTERY                              */
#define USBPD_PDO_TYPE_VARIABLE  (uint32_t)(USBPD_CORE_PDO_TYPE_VARIABLE << USBPD_PDO_TYPE_Pos)/*!< PDO Type = VARIABLE                             */
#if defined(USBPD_REV30_SUPPORT)
#define USBPD_PDO_TYPE_APDO      (uint32_t)(USBPD_CORE_PDO_TYPE_APDO<< USBPD_PDO_TYPE_Pos)     /*!< PDO Type = APDO                                 */
#endif /* USBPD_REV30_SUPPORT */

/* Source Fixed type PDO elements */
#define USBPD_PDO_SRC_FIXED_DRP_SUPPORT_Pos          (29U)                                         /*!< DRP Support bit position                        */
#define USBPD_PDO_SRC_FIXED_DRP_SUPPORT_Msk          (0x1U << USBPD_PDO_SRC_FIXED_DRP_SUPPORT_Pos) /*!< DRP Support bit mask : 0x20000000               */
#define USBPD_PDO_SRC_FIXED_DRP_NOT_SUPPORTED        (0U)                                          /*!< DRP not supported                               */
#define USBPD_PDO_SRC_FIXED_DRP_SUPPORTED            USBPD_PDO_SRC_FIXED_DRP_SUPPORT_Msk           /*!< DRP supported                                   */

#define USBPD_PDO_SRC_FIXED_USBSUSPEND_Pos           (28U)                                         /*!< USB Suspend Support bit position                */
#define USBPD_PDO_SRC_FIXED_USBSUSPEND_Msk           (0x1U << USBPD_PDO_SRC_FIXED_USBSUSPEND_Pos)  /*!< USB Suspend Support bit mask : 0x10000000       */
#define USBPD_PDO_SRC_FIXED_USBSUSPEND_NOT_SUPPORTED (0U)                                          /*!< USB Suspend not supported                       */
#define USBPD_PDO_SRC_FIXED_USBSUSPEND_SUPPORTED     USBPD_PDO_SRC_FIXED_USBSUSPEND_Msk            /*!< USB Suspend supported                           */

#define USBPD_PDO_SRC_FIXED_EXT_POWER_Pos            (27U)                                         /*!< External Power available bit position           */
#define USBPD_PDO_SRC_FIXED_EXT_POWER_Msk            (0x1U << USBPD_PDO_SRC_FIXED_EXT_POWER_Pos)   /*!< External Power available bit mask : 0x08000000  */
#define USBPD_PDO_SRC_FIXED_EXT_POWER_NOT_AVAILABLE  (0U)                                          /*!< External Power not available                    */
#define USBPD_PDO_SRC_FIXED_EXT_POWER_AVAILABLE      USBPD_PDO_SRC_FIXED_EXT_POWER_Msk             /*!< External Power available                        */

#define USBPD_PDO_SRC_FIXED_USBCOMM_Pos              (26U)                                         /*!< USB Communication Support bit position          */
#define USBPD_PDO_SRC_FIXED_USBCOMM_Msk              (0x1U << USBPD_PDO_SRC_FIXED_USBCOMM_Pos)     /*!< USB Communication Support bit mask : 0x04000000 */
#define USBPD_PDO_SRC_FIXED_USBCOMM_NOT_SUPPORTED    (0U)                                          /*!< USB Communication not supported                 */
#define USBPD_PDO_SRC_FIXED_USBCOMM_SUPPORTED        USBPD_PDO_SRC_FIXED_USBCOMM_Msk               /*!< USB Communication supported                     */

#define USBPD_PDO_SRC_FIXED_DRD_SUPPORT_Pos          (25U)                                         /*!< Dual Role Data Support bit position             */
#define USBPD_PDO_SRC_FIXED_DRD_SUPPORT_Msk          (0x1U << USBPD_PDO_SRC_FIXED_DRD_SUPPORT_Pos) /*!< Dual Role Data Support bit mask : 0x02000000    */
#define USBPD_PDO_SRC_FIXED_DRD_NOT_SUPPORTED        (0U)                                          /*!< Dual Role Data not supported                    */
#define USBPD_PDO_SRC_FIXED_DRD_SUPPORTED            USBPD_PDO_SRC_FIXED_DRD_SUPPORT_Msk           /*!< Dual Role Data supported                        */

#if defined(USBPD_REV30_SUPPORT)
#define USBPD_PDO_SRC_FIXED_UNCHUNK_SUPPORT_Pos      (24U)                                            /*!< Unchunked Extended Messages Support bit position             */
#define USBPD_PDO_SRC_FIXED_UNCHUNK_SUPPORT_Msk      (0x1U << USBPD_PDO_SRC_FIXED_UNCHUNK_SUPPORT_Pos)/*!< Unchunked Extended Messages Support bit mask : 0x01000000    */
#define USBPD_PDO_SRC_FIXED_UNCHUNK_NOT_SUPPORTED    (0U)                                             /*!< Unchunked Extended Messages not supported                    */
#define USBPD_PDO_SRC_FIXED_UNCHUNK_SUPPORTED        USBPD_PDO_SRC_FIXED_UNCHUNK_SUPPORT_Msk          /*!< Unchunked Extended Messages supported                        */
#endif /* USBPD_REV30_SUPPORT */

#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos          (20U)                                                             /*!< Peak Current info bits position            */
#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_Msk          (0x3U << USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos)                     /*!< Peak Current info bits mask : 0x00300000   */
#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_EQUAL        (USBPD_CORE_PDO_PEAKEQUAL << USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos) /*!< Peak Current info : Equal to Ioc           */
#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_OVER1        (USBPD_CORE_PDO_PEAKOVER1 << USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos) /*!< Peak Current info : Overload Cap 1         */
#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_OVER2        (USBPD_CORE_PDO_PEAKOVER2 << USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos) /*!< Peak Current info : Overload Cap 2         */
#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_OVER3        (USBPD_CORE_PDO_PEAKOVER3 << USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos) /*!< Peak Current info : Overload Cap 3         */

#define USBPD_PDO_SRC_FIXED_VOLTAGE_Pos              (10U)                                            /*!< Voltage in 50 mV units bits position               */
#define USBPD_PDO_SRC_FIXED_VOLTAGE_Msk              (0x3FFU << USBPD_PDO_SRC_FIXED_VOLTAGE_Pos)      /*!< Voltage in 50 mV units bits mask : 0x000FFC00      */

#define USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos          (0U)                                             /*!< Max current in 10 mA units bits position           */
#define USBPD_PDO_SRC_FIXED_MAX_CURRENT_Msk          (0x3FFU << USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos)  /*!< Max current in 10 mA units bits mask : 0x000003FF  */

/* Source Variable type PDO elements */
#define USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Pos       (20U)                                              /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Msk       (0x3FFU << USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Pos) /*!< Max Voltage in 50 mV units bits mask : 0x3FF00000  */

#define USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Pos       (10U)                                              /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Msk       (0x3FFU << USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Pos) /*!< Max Voltage in 50 mV units bits mask : 0x000FFC00  */

#define USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Pos       (0U)                                               /*!< Max current in 10 mA units bits position           */
#define USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Msk       (0x3FFU << USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Pos) /*!< Max current in 10 mA units bits mask : 0x000003FF  */

/* Source Battery type PDO elements */
#define USBPD_PDO_SRC_BATTERY_MAX_VOLTAGE_Pos        (20U)                                              /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SRC_BATTERY_MAX_VOLTAGE_Msk        (0x3FFU << USBPD_PDO_SRC_BATTERY_MAX_VOLTAGE_Pos)  /*!< Max Voltage in 50 mV units bits mask : 0x3FF00000  */

#define USBPD_PDO_SRC_BATTERY_MIN_VOLTAGE_Pos        (10U)                                              /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SRC_BATTERY_MIN_VOLTAGE_Msk        (0x3FFU << USBPD_PDO_SRC_BATTERY_MIN_VOLTAGE_Pos)  /*!< Max Voltage in 50 mV units bits mask : 0x000FFC00  */

#define USBPD_PDO_SRC_BATTERY_MAX_POWER_Pos          (0U)                                               /*!< Max allowable power in 250mW units bits position          */
#define USBPD_PDO_SRC_BATTERY_MAX_POWER_Msk          (0x3FFU << USBPD_PDO_SRC_BATTERY_MAX_POWER_Pos)    /*!< Max allowable power in 250mW units bits mask : 0x000003FF */

/* Sink Fixed type PDO elements */
#define USBPD_PDO_SNK_FIXED_DRP_SUPPORT_Pos          (29U)                                              /*!< DRP Support bit position                        */
#define USBPD_PDO_SNK_FIXED_DRP_SUPPORT_Msk          (0x1U << USBPD_PDO_SNK_FIXED_DRP_SUPPORT_Pos)      /*!< DRP Support bit mask : 0x20000000               */
#define USBPD_PDO_SNK_FIXED_DRP_NOT_SUPPORTED        (0U)                                               /*!< DRP not supported                               */
#define USBPD_PDO_SNK_FIXED_DRP_SUPPORTED            USBPD_PDO_SNK_FIXED_DRP_SUPPORT_Msk                /*!< DRP supported                                   */

#define USBPD_PDO_SNK_FIXED_HIGHERCAPAB_Pos           (28U)                                             /*!< Higher capability support bit position          */
#define USBPD_PDO_SNK_FIXED_HIGHERCAPAB_Msk           (0x1U << USBPD_PDO_SNK_FIXED_HIGHERCAPAB_Pos)     /*!< Higher capability support bit mask : 0x10000000 */
#define USBPD_PDO_SNK_FIXED_HIGHERCAPAB_NOT_SUPPORTED (0U)                                              /*!< Higher capability not supported                 */
#define USBPD_PDO_SNK_FIXED_HIGHERCAPAB_SUPPORTED     USBPD_PDO_SNK_FIXED_HIGHERCAPAB_Msk               /*!< Higher capability supported                     */

#define USBPD_PDO_SNK_FIXED_EXT_POWER_Pos            (27U)                                                     /*!< External Power available bit position           */
#define USBPD_PDO_SNK_FIXED_EXT_POWER_Msk            (0x1U << USBPD_PDO_SNK_FIXED_EXT_POWER_Pos)               /*!< External Power available bit mask : 0x08000000  */
#define USBPD_PDO_SNK_FIXED_EXT_POWER_NOT_AVAILABLE  (0U)                                                      /*!< External Power not available                    */
#define USBPD_PDO_SNK_FIXED_EXT_POWER_AVAILABLE      USBPD_PDO_SNK_FIXED_EXT_POWER_Msk                         /*!< External Power available                        */

#define USBPD_PDO_SNK_FIXED_USBCOMM_Pos              (26U)                                                     /*!< USB Communication Support bit position          */
#define USBPD_PDO_SNK_FIXED_USBCOMM_Msk              (0x1U << USBPD_PDO_SNK_FIXED_USBCOMM_Pos)                 /*!< USB Communication Support bit mask : 0x04000000 */
#define USBPD_PDO_SNK_FIXED_USBCOMM_NOT_SUPPORTED    (0U)                                                      /*!< USB Communication not supported                 */
#define USBPD_PDO_SNK_FIXED_USBCOMM_SUPPORTED        USBPD_PDO_SNK_FIXED_USBCOMM_Msk                           /*!< USB Communication supported                     */

#define USBPD_PDO_SNK_FIXED_DRD_SUPPORT_Pos          (25U)                                                     /*!< Dual Role Data Support bit position             */
#define USBPD_PDO_SNK_FIXED_DRD_SUPPORT_Msk          (0x1U << USBPD_PDO_SNK_FIXED_DRD_SUPPORT_Pos)             /*!< Dual Role Data Support bit mask : 0x02000000    */
#define USBPD_PDO_SNK_FIXED_DRD_NOT_SUPPORTED        (0U)                                                      /*!< Dual Role Data not supported                    */
#define USBPD_PDO_SNK_FIXED_DRD_SUPPORTED            USBPD_PDO_SNK_FIXED_DRD_SUPPORT_Msk                       /*!< Dual Role Data supported                        */

#if defined(USBPD_REV30_SUPPORT)
#define USBPD_PDO_SNK_FIXED_FRS_SUPPORT_Pos          (23U)                                                     /*!< Fast Role Swap required Current bit position             */
#define USBPD_PDO_SNK_FIXED_FRS_SUPPORT_Msk          (0x3U << USBPD_PDO_SNK_FIXED_FRS_SUPPORT_Pos)             /*!< Fast Role Swap required Current bit mask : 0x01800000    */
#define USBPD_PDO_SNK_FIXED_FRS_NOT_SUPPORTED        (0U)                                                      /*!< Fast Role Swap not supported                             */
#define USBPD_PDO_SNK_FIXED_FRS_DEFAULT              (USBPD_CORE_PDO_FRS_DEFAULT_USB_POWER << USBPD_PDO_SNK_FIXED_FRS_SUPPORT_Pos)  /*!< Fast Role Swap required default USB power  */
#define USBPD_PDO_SNK_FIXED_FRS_1_5A                 (USBPD_CORE_PDO_FRS_1_5A_5V << USBPD_PDO_SNK_FIXED_FRS_SUPPORT_Pos)            /*!< Fast Role Swap 1.5A at 5V                  */
#define USBPD_PDO_SNK_FIXED_FRS_3A                   (USBPD_CORE_PDO_FRS_3A_5V << USBPD_PDO_SNK_FIXED_FRS_SUPPORT_Pos)              /*!< Fast Role Swap 3A at 5V                    */
#endif /* USBPD_REV30_SUPPORT */

#define USBPD_PDO_SNK_FIXED_VOLTAGE_Pos              (10U)                                                     /*!< Voltage in 50 mV units bits position               */
#define USBPD_PDO_SNK_FIXED_VOLTAGE_Msk              (0x3FFU << USBPD_PDO_SNK_FIXED_VOLTAGE_Pos)               /*!< Voltage in 50 mV units bits mask : 0x000FFC00      */

#define USBPD_PDO_SNK_FIXED_OP_CURRENT_Pos           (0U)                                                      /*!< Operational current in 10 mA units bits position           */
#define USBPD_PDO_SNK_FIXED_OP_CURRENT_Msk           (0x3FFU << USBPD_PDO_SNK_FIXED_OP_CURRENT_Pos)            /*!< Operational current in 10 mA units bits mask : 0x000003FF  */

/* Sink Variable type PDO elements */
#define USBPD_PDO_SNK_VARIABLE_MAX_VOLTAGE_Pos       (20U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SNK_VARIABLE_MAX_VOLTAGE_Msk       (0x3FFU << USBPD_PDO_SNK_VARIABLE_MAX_VOLTAGE_Pos)        /*!< Max Voltage in 50 mV units bits mask : 0x3FF00000  */

#define USBPD_PDO_SNK_VARIABLE_MIN_VOLTAGE_Pos       (10U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SNK_VARIABLE_MIN_VOLTAGE_Msk       (0x3FFU << USBPD_PDO_SNK_VARIABLE_MIN_VOLTAGE_Pos)        /*!< Max Voltage in 50 mV units bits mask : 0x000FFC00  */

#define USBPD_PDO_SNK_VARIABLE_OP_CURRENT_Pos        (0U)                                                      /*!< Operational current in 10 mA units bits position           */
#define USBPD_PDO_SNK_VARIABLE_OP_CURRENT_Msk        (0x3FFU << USBPD_PDO_SNK_VARIABLE_OP_CURRENT_Pos)         /*!< Operational current in 10 mA units bits mask : 0x000003FF  */

/* Sink Battery type PDO elements */
#define USBPD_PDO_SNK_BATTERY_MAX_VOLTAGE_Pos        (20U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SNK_BATTERY_MAX_VOLTAGE_Msk        (0x3FFU << USBPD_PDO_SNK_BATTERY_MAX_VOLTAGE_Pos)         /*!< Max Voltage in 50 mV units bits mask : 0x3FF00000  */

#define USBPD_PDO_SNK_BATTERY_MIN_VOLTAGE_Pos        (10U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SNK_BATTERY_MIN_VOLTAGE_Msk        (0x3FFU << USBPD_PDO_SNK_BATTERY_MIN_VOLTAGE_Pos)         /*!< Max Voltage in 50 mV units bits mask : 0x000FFC00  */

#define USBPD_PDO_SNK_BATTERY_OP_POWER_Pos           (0U)                                                      /*!< Operational power in 250mW units bits position          */
#define USBPD_PDO_SNK_BATTERY_OP_POWER_Msk           (0x3FFU << USBPD_PDO_SNK_BATTERY_OP_POWER_Pos)            /*!< Operational power in 250mW units bits mask : 0x000003FF */

#if defined(USBPD_REV30_SUPPORT)
#ifdef USBPDCORE_PPS
/* Source APDO type PDO elements */
#define USBPD_PDO_SRC_APDO_PPS_Pos                   (28U)                                                     /*!< Programmable Power Supply bit position           */
#define USBPD_PDO_SRC_APDO_PPS_Msk                   (0x3U << USBPD_PDO_SRC_APDO_PPS_Pos)                      /*!< Programmable Power Supply bit mask : 0x300000000 */
#define USBPD_PDO_SRC_APDO_PPS                       (0U)                                                      /*!< Programmable Power Supply field value 00         */

#define USBPD_PDO_SRC_APDO_PPS_PWR_LIMITED_Pos       (27U)                                                     /*!< PPS Power Limited bit position                   */
#define USBPD_PDO_SRC_APDO_PPS_PWR_LIMITED_SET       (0x1U << USBPD_PDO_SRC_APDO_PPS_PWR_LIMITED_Pos)          /*!< PPS Power Limited bit is set                     */
#define USBPD_PDO_SRC_APDO_PPS_PWR_LIMITED_CLEAR     (0x0U << USBPD_PDO_SRC_APDO_PPS_PWR_LIMITED_Pos)          /*!< PPS Power Limited bit is cleared                 */

#define USBPD_PDO_SRC_APDO_MAX_VOLTAGE_Pos           (17U)                                                     /*!< APDO Max Voltage in 100 mV increments bits position  */
#define USBPD_PDO_SRC_APDO_MAX_VOLTAGE_Msk           (0xFFU << USBPD_PDO_SRC_APDO_MAX_VOLTAGE_Pos)             /*!< APDO Max Voltage in 100 mV increments bits mask : 0x01FE0000 */

#define USBPD_PDO_SRC_APDO_MIN_VOLTAGE_Pos           (8U)                                                      /*!< APDO Min Voltage in 100 mV increments bits position  */
#define USBPD_PDO_SRC_APDO_MIN_VOLTAGE_Msk           (0xFFU << USBPD_PDO_SRC_APDO_MIN_VOLTAGE_Pos)             /*!< APDO Min Voltage in 100 mV increments bits mask : 0x0000FF00 */

#define USBPD_PDO_SRC_APDO_MAX_CURRENT_Pos           (0U)                                                      /*!< APDO Max Current in 50 mA increments bits position  */
#define USBPD_PDO_SRC_APDO_MAX_CURRENT_Msk           (0x7FU << USBPD_PDO_SRC_APDO_MAX_CURRENT_Pos)             /*!< APDO Max Current in 50 mA increments bits mask : 0x0000007F */

/* Sink APDO type PDO elements */
#define USBPD_PDO_SNK_APDO_PPS_Pos                   (28U)                                                     /*!< Programmable Power Supply bit position           */
#define USBPD_PDO_SNK_APDO_PPS_Msk                   (0x3U << USBPD_PDO_SNK_APDO_PPS_Pos)                      /*!< Programmable Power Supply bit mask : 0x300000000 */
#define USBPD_PDO_SNK_APDO_PPS                       (0U)                                                      /*!< Programmable Power Supply field value 00         */

#define USBPD_PDO_SNK_APDO_MAX_VOLTAGE_Pos           (17U)                                                     /*!< APDO Max Voltage in 100 mV increments bits position  */
#define USBPD_PDO_SNK_APDO_MAX_VOLTAGE_Msk           (0xFFU << USBPD_PDO_SNK_APDO_MAX_VOLTAGE_Pos)             /*!< APDO Max Voltage in 100 mV increments bits mask : 0x01FE0000 */

#define USBPD_PDO_SNK_APDO_MIN_VOLTAGE_Pos           (8U)                                                      /*!< APDO Min Voltage in 100 mV increments bits position  */
#define USBPD_PDO_SNK_APDO_MIN_VOLTAGE_Msk           (0xFFU << USBPD_PDO_SNK_APDO_MIN_VOLTAGE_Pos)             /*!< APDO Min Voltage in 100 mV increments bits mask : 0x0000FF00 */

#define USBPD_PDO_SNK_APDO_MAX_CURRENT_Pos           (0U)                                                      /*!< APDO Max Current in 50 mA increments bits position  */
#define USBPD_PDO_SNK_APDO_MAX_CURRENT_Msk           (0x7FU << USBPD_PDO_SNK_APDO_MAX_CURRENT_Pos)             /*!< APDO Max Current in 50 mA increments bits mask : 0x0000007F */
#endif /* USBPDCORE_PPS */
#endif /* USBPD_REV30_SUPPORT */

#define USBPD_EXTENDED_MESSAGE                       (0x80U)                                                   /*!< Flag to indicate that it is a extended message     */

/**
  * @}
  */

#if defined(USBPD_REV30_SUPPORT)
/** @defgroup USBPD_ADO_TYPE_ALERT USB-PD Type alert definition used for Alert Data Object
  * @{
  */
#define USBPD_ADO_TYPE_ALERT_BATTERY_STATUS (1U << 1U) /*!< Battery Status Change Event(Attach/Detach/charging/discharging/idle) */
#define USBPD_ADO_TYPE_ALERT_OCP            (1U << 2U) /*!< Over-Current Protection event when set (Source only, for Sink Reserved and Shall be set to zero) */
#define USBPD_ADO_TYPE_ALERT_OTP            (1U << 3U) /*!< Over-Temperature Protection event when set  */
#define USBPD_ADO_TYPE_ALERT_OPERATING_COND (1U << 4U) /*!< Operating Condition Change when set */
#define USBPD_ADO_TYPE_ALERT_SRC_INPUT      (1U << 5U) /*!< Source Input Change Event when set */
#define USBPD_ADO_TYPE_ALERT_OVP            (1U << 6U) /*!< Over-Voltage Protection event when set (Sink only, for Source Reserved and Shall be set to zero) */
#define USBPD_ADO_TYPE_ALERT_EXT            (1U << 7U) /*!< Extended Alert Event */
/**
  * @}
  */

/** @defgroup USBPD_ADO_FIXED_BATT USB-PD Fixed Batteries definition used for Alert Data Object
  * @{
  */
#define USBPD_ADO_FIXED_BATT_BATTERY_0 (1U << 0U) /*!< Fixed Batterie 0 had a status change */
#define USBPD_ADO_FIXED_BATT_BATTERY_1 (1U << 1U) /*!< Fixed Batterie 1 had a status change */
#define USBPD_ADO_FIXED_BATT_BATTERY_2 (1U << 2U) /*!< Fixed Batterie 2 had a status change */
#define USBPD_ADO_FIXED_BATT_BATTERY_3 (1U << 3U) /*!< Fixed Batterie 3 had a status change */
/**
  * @}
  */

/** @defgroup USBPD_ADO_HOT_SWAP_BATT USB-PD Hot Swappable Batteries definition used for Alert Data Object
  * @{
  */
#define USBPD_ADO_HOT_SWAP_BATT_BATTERY_4 (1U << 0U) /*!< Hot Swappable Batterie 4 had a status change */
#define USBPD_ADO_HOT_SWAP_BATT_BATTERY_5 (1U << 1U) /*!< Hot Swappable Batterie 5 had a status change */
#define USBPD_ADO_HOT_SWAP_BATT_BATTERY_6 (1U << 2U) /*!< Hot Swappable Batterie 6 had a status change */
#define USBPD_ADO_HOT_SWAP_BATT_BATTERY_7 (1U << 3U) /*!< Hot Swappable Batterie 7 had a status change */
/**
  * @}
  */

/** @defgroup USBPD_ADO_EXT_TYPE_ALERT USB-PD Extended Alert Event Type definition used for Alert Data Object
  * @{
  */
#define USBPD_ADO_EXT_TYPE_ALERT_PWR_STATE_CHANGE   1U /*!< Power state change (DFP only) */
#define USBPD_ADO_EXT_TYPE_ALERT_PWR_BUTTON_PRESS   2U /*!< Power button press (UFP only) */
#define USBPD_ADO_EXT_TYPE_ALERT_PWR_BUTTON_RELEASE 3U /*!< Power button release (UFP only) */
#define USBPD_ADO_EXT_TYPE_ALERT_CTRL_INITIATED     4U /*!< Controller initiated wake e.g. Wake on Lan (UFP only) */
/**
  * @}
  */

/** @defgroup USBPD_SDB_PRESENT_INPUT USB-PD Status Data Block - Present Input
  * @{
  */
#define USBPD_SDB_PRESENT_INPUT_EXT_PWR             (1U << 1U) /*!< External Power when set */
#define USBPD_SDB_PRESENT_INPUT_EXT_PWR_ACDC        (1U << 2U) /*!< External Power AC/DC (Valid when Bit 1 set)
                                                                      0: DC
                                                                      1: AC
                                                                    Reserved when Bit 1 is zero*/
#define USBPD_SDB_PRESENT_INPUT_INT_PWR_FROM_BAT    (1U << 3U) /*!< Internal Power from Battery when set */
#define USBPD_SDB_PRESENT_INPUT_INT_PWR_FROM_N0_BAT (1U << 4U) /*!< Internal Power from non-Battery power source when set */
#define USBPD_SDB_PRESENT_INPUT_INT_MASK            (0x0FU << 1U) /*!< Present Input mask*/
/**
  * @}
  */

/** @defgroup USBPD_SDB_EVENT_FLAGS USB-PD Status Data Block - EventFlags
  * @{
  */
#define USBPD_SDB_EVENT_FLAGS_OCP                   (1U << 1U) /*!< OCP event when set                      */
#define USBPD_SDB_EVENT_FLAGS_OTP                   (1U << 2U) /*!< OTP event when set                      */
#define USBPD_SDB_EVENT_FLAGS_OVP                   (1U << 3U) /*!< OVP event when set                      */
#define USBPD_SDB_EVENT_FLAGS_CF_CV                 (1U << 4U) /*!< CF mode when set, CV mode when cleared  */
#define USBPD_SDB_EVENT_FLAGS_MASK                  (0x0FU << 1U) /*!< EventFlags mask                      */
/**
  * @}
  */

/** @defgroup USBPD_SDB_TEMP_STATUS USB-PD Status Data Block - Temperature Status
  * @{
  */
#define USBPD_SDB_EVENT_TEMP_STATUS_NOT_SUPP        (0U << 1U) /*!< 00 - Not Supported.                     */
#define USBPD_SDB_EVENT_TEMP_STATUS_NORMAL          (1U << 1U) /*!< 01 - Normal                             */
#define USBPD_SDB_EVENT_TEMP_STATUS_WARNING         (2U << 1U) /*!< 10 - Warning                            */
#define USBPD_SDB_EVENT_TEMP_STATUS_OVER_TEMP       (3U << 1U) /*!< 11 - Over temperature                   */
#define USBPD_SDB_EVENT_TEMP_STATUS_MASK            (3U << 1U) /*!< Temp status mask                        */
/**
  * @}
  */

/** @defgroup USBPD_SDB_POWER_STATUS USB-PD Status Data Block - Power Status
  * @{
  */
#define USBPD_SDB_POWER_STATUS_CABLE                  (1U << 1U) /*!< Source power limited due to cable supported current */
#define USBPD_SDB_POWER_STATUS_INSUFFICIENT_POWER     (1U << 2U) /*!< Source power limited due to insufficient power
                                                                      available while sourcing other ports */
#define USBPD_SDB_POWER_STATUS_INSUFFICIENT_EXT_POWER (1U << 3U) /*!< Source power limited due to insufficient external power */
#define USBPD_SDB_POWER_STATUS_EVENT_FLAGS            (1U << 4U) /*!< Source power limited due to Event Flags in place
                                                                      (Event Flags must also be set) */
#define USBPD_SDB_POWER_STATUS_TEMPERATURE            (1U << 5U) /*!< Source power limited due to temperature */
#define USBPD_SDB_POWER_STATUS_MASK                   (0x1FU << 1U) /*!< Power status mask */
/**
  * @}
  */

/** @defgroup USBPD_SDB_PWR_STATE USB-PD Status Data Block - Power State Change
  * @{
  */
#define USBPD_SDB_PWR_STATE_NEW_NOT_SUPP            (0U << 0U) /*!< New Power State: Status not supported   */
#define USBPD_SDB_PWR_STATE_NEW_S0                  (1U << 0U) /*!< New Power State: S0                     */
#define USBPD_SDB_PWR_STATE_NEW_MODERN_STDBY        (2U << 0U) /*!< New Power State: Modern Standby         */
#define USBPD_SDB_PWR_STATE_NEW_S3                  (3U << 0U) /*!< New Power State: S3                     */
#define USBPD_SDB_PWR_STATE_NEW_S4                  (4U << 0U) /*!< New Power State: S4                     */
#define USBPD_SDB_PWR_STATE_NEW_S5                  (5U << 0U) /*!< New Power State: S5 (Off with battery,
                                                                    wake events supported)                  */
#define USBPD_SDB_PWR_STATE_NEW_G3                  (6U << 0U) /*!< New Power State: G3 (Off with no battery,
                                                                    wake events not supported)              */
#define USBPD_SDB_PWR_STATE_NEW_INDIC_OFF_LED       (0U << 3U) /*!< New power state indicator: Off LED      */
#define USBPD_SDB_PWR_STATE_NEW_INDIC_ON_LED        (1U << 3U) /*!< New power state indicator: Off LED      */
#define USBPD_SDB_PWR_STATE_NEW_INDIC_BLINK_LED     (2U << 3U) /*!< New power state indicator: Blinking LED */
#define USBPD_SDB_PWR_STATE_NEW_INDIC_BREATH_LED    (3U << 3U) /*!< New power state indicator: Breathing LED*/
#define USBPD_SDB_PWR_STATE_MASK                    (0x3FU)    /*!< Power State Change mask                 */
/**
  * @}
  */

/** @defgroup USBPD_MANUFINFO_TARGET USB-PD Manufacturer Info Target
  * @{
  */
#define USBPD_MANUFINFO_TARGET_PORT_CABLE_PLUG 0U /*!< Manufacturer Info Target Port/Cable Plug */
#define USBPD_MANUFINFO_TARGET_BATTERY         1U /*!< Manufacturer Info Target  Battery        */
/**
  * @}
  */

/** @defgroup USBPD_MANUFINFO_REF USB-PD Manufacturer Info Ref
  * @{
  */
#define USBPD_MANUFINFO_REF_MAX_VALUES 7U /*!< Manufacturer Info Ref 0..3:Fixed Batteries and 4..7: Hot Swappable Batteries*/
/**
  * @}
  */

/** @defgroup USBPD_BSDO_BATT_INFO USB-PD Battery Status - Info
  * @{
  */
#define USBPD_BSDO_BATT_INFO_INVALID_REF              (1U << 0U) /*!<  Battery Status - Info: Invalid Battery reference */
#define USBPD_BSDO_BATT_INFO_BATT_PRESENT             (1U << 1U) /*!<  Battery Status - Info: Battery is present when set*/
#define USBPD_BSDO_BATT_INFO_BATT_ISCHARGING          (0U << 2U) /*!<  Battery Status - Info: Battery is Charging*/
#define USBPD_BSDO_BATT_INFO_BATT_ISDISCHARGING       (1U << 2U) /*!<  Battery Status - Info: Battery is Discharging*/
#define USBPD_BSDO_BATT_INFO_BATT_ISIDLE              (2U << 2U) /*!<  Battery Status - Info: Battery is Idle*/
/**
  * @}
  */

#ifdef USBPDCORE_PPS
/** @defgroup USBPD_CORE_DEF_REAL_TIME_FLAGS USBPD CORE DEF Real Time Flags
  * @{
  */
#define USBPD_PPS_REALTIMEFLAGS_PTF_NOT_SUPPORTED     (00U << 1U) /*!< PTF: 00 - Not Supported                              */
#define USBPD_PPS_REALTIMEFLAGS_PTF_NORMAL            (01U << 1U) /*!< PTF: 01 - Normal                                     */
#define USBPD_PPS_REALTIMEFLAGS_PTF_WARNING           (10U << 1U) /*!< PTF: 10 - Warning                                    */
#define USBPD_PPS_REALTIMEFLAGS_PTF_OVER_TEMPERATURE  (11U << 1U) /*!< PTF: 11 - Over temperature                           */
#define USBPD_PPS_REALTIMEFLAGS_OMF_ENABLED           (1U  << 3U) /*!< OMF set when operating in Current Foldback mode      */
#define USBPD_PPS_REALTIMEFLAGS_OMF_DISABLED          (0U  << 3U) /*!< OMF set when operating in Current Foldback mode      */
/**
  * @}
  */
#endif /* USBPDCORE_PPS */

#if defined(USBPDCORE_SNK_CAPA_EXT)
/** @defgroup USBPD_SKEDB_VERSION USB-PD Sink Capabilities Extended - SKEDB version
  * @{
  */
#define USBPD_SKEDB_VERSION_1P0                  1U /*!< Version 1.0 */
/**
  * @}
  */

/** @defgroup USBPD_SKEDB_LOADSTEP USB-PD Sink Capabilities Extended - Load Step field
  * @{
  */
#define USBPD_SKEDB_LOADSTEP_150MA               (0U << 0U) /*!< 150mA/1s Load Step (default) */
#define USBPD_SKEDB_LOADSTEP_500MA               (1U << 0U) /*!< 500mA/1s Load Step */
/**
  * @}
  */

/** @defgroup USBPD_SKEDB_COMPLIANCE USB-PD Sink Capabilities Extended - Compliance field
  * @{
  */
#define USBPD_SKEDB_COMPLIANCE_LPS               (1U << 0U) /*!< Requires LPS Source when set */
#define USBPD_SKEDB_COMPLIANCE_PS1               (1U << 1U) /*!< Requires PS1 Source when set */
#define USBPD_SKEDB_COMPLIANCE_PS2               (1U << 2U) /*!< Requires PS2 Source when set */
/**
  * @}
  */

/** @defgroup USBPD_SKEDB_TOUCHTEMP USB-PD Sink Capabilities Extended - Touch Temperature
  * @{
  */
#define USBPD_SKEDB_TOUCHTEMP_NA                  (0U) /*!< Temperature conforms to Not applicable          */
#define USBPD_SKEDB_TOUCHTEMP_DEFAULT             (1U) /*!< Temperature conforms to [IEC 60950-1] (default) */
#define USBPD_SKEDB_TOUCHTEMP_TS1                 (2U) /*!< Temperature conforms to [IEC 62368-1] TS1       */
#define USBPD_SKEDB_TOUCHTEMP_TS2                 (2U) /*!< Temperature conforms to [IEC 62368-1] TS2       */
/**
  * @}
  */

/** @defgroup USBPD_SKEDB_SINKMODES USB-PD Sink Capabilities Extended - Sink Modes
  * @{
  */
#define USBPD_SKEDB_SINKMODES_PPS                 (1U << 0U) /*!< 1: PPS charging supported         */
#define USBPD_SKEDB_SINKMODES_VBUS                (1U << 1U) /*!< 1: VBUS powered                   */
#define USBPD_SKEDB_SINKMODES_MAINS               (1U << 2U) /*!< 1: Mains powered                  */
#define USBPD_SKEDB_SINKMODES_BATPOW              (1U << 3U) /*!< 1: Battery powered                */
#define USBPD_SKEDB_SINKMODES_BATUNL              (1U << 4U) /*!< 1: Battery essentially unlimited  */
/**
  * @}
  */
#endif /* USBPDCORE_SNK_CAPA_EXT */

#if defined(USBPDCORE_VPD)
/** @defgroup USBPD_FWUPD_MSGTYPE USB-PD Firmware Update Message Request and Responses Defines
  * @{
  */
typedef enum
{
  VPD_NONE                   = 0U,  /*!< status none, no VPD detection ongoing                      */
  VPD_UNKNOWN                = 1U,  /*!< status unknown                                             */
  VPD_NOPD                   = 2U,  /*!< status NOPD support                                        */
  VPD_FAILED_ENTER_ALTERNATE = 3U,  /*!< status Failed to enter alternate mode                      */
  VPD_DETECTED               = 4U   /*!< status VPD detected                                        */
} USBPD_VPD_Status;
/**
  *@}
  */
#endif /* USBPDCORE_VPD */

#if defined(USBPDCORE_FWUPD)

/** @defgroup USBPD_FWUPD_MSGTYPE USB-PD Firmware Update Message Request and Responses Defines
  * @{
  */

typedef enum
{
  USBPD_FWUPD_MSGTYPE_NONE                = 0x00U, /*!< Reserved value                                                */
  USBPD_FWUPD_MSGTYPE_RSP_GET_FW_ID       = 0x01U, /*!< Response is used to respond to a GET_FW_ID Request            */
  USBPD_FWUPD_MSGTYPE_RSP_PDFU_INITIATE   = 0x02U, /*!< Response is used to respond to a PDFU_INITIATE Request        */
  USBPD_FWUPD_MSGTYPE_RSP_PDFU_DATA       = 0x03U, /*!< Response is used to respond to a PDFU_DATA Request            */
  USBPD_FWUPD_MSGTYPE_RSP_PDFU_VALIDATE   = 0x05U, /*!< Response is used to respond to a PDFU_VALIDATE Request        */
  USBPD_FWUPD_MSGTYPE_RSP_PDFU_DATA_PAUSE = 0x07U, /*!< Response is used to respond to a PDFU_DATA_PAUSE Request      */
  USBPD_FWUPD_MSGTYPE_RSP_VENDOR_SPECIFIC = 0x7FU, /*!< Response is used to respond to a VENDOR_SPECIFIC Request      */
  USBPD_FWUPD_MSGTYPE_REQ_GET_FW_ID       = 0x81U, /*!< Request is used to retrieve information about a PDFU Responder
                                                        and determine if a firmware update is necessary               */
  USBPD_FWUPD_MSGTYPE_REQ_PDFU_INITIATE   = 0x82U, /*!< Request is used to initiate firmware update                   */
  USBPD_FWUPD_MSGTYPE_REQ_PDFU_DATA       = 0x83U, /*!< Request is used to transfer a data block from a firmware image
                                                        (response required)                                           */
  USBPD_FWUPD_MSGTYPE_REQ_PDFU_DATA_NR    = 0x84U, /*!< Request is used to transfer a data block from a firmware image
                                                        (response not required)                                       */
  USBPD_FWUPD_MSGTYPE_REQ_PDFU_VALIDATE   = 0x85U, /*!< Request is used to request validation of a firmware image     */
  USBPD_FWUPD_MSGTYPE_REQ_PDFU_ABORT      = 0x86U, /*!< Request is used to end firmware image update prematurely      */
  USBPD_FWUPD_MSGTYPE_REQ_PDFU_DATA_PAUSE = 0x87U, /*!< Request is used to pause a firmware image update              */
  USBPD_FWUPD_MSGTYPE_REQ_VENDOR_SPECIFIC = 0xFFU, /*!< Request is for vendor-specific use                            */
}
USBPD_FWUPD_MsgType_TypeDef;

/**
  * @}
  */

/** @defgroup USBPD_FWUPD_PROT_VER USB-PD Firmware Update Message Protocol version Defines
  * @{
  */

#define  USBPD_FWUPD_PROT_VER_V1P0                0x01u /*!< USB PD Firmware Update Protocol Version 1.0 */

/**
  * @}
  */

/** @defgroup USBPD_FWUPD_SIZE_PAYLOAD USB-PD Firmware Update Size Payload Defines
  * @{
  */

#define  USBPD_FWUPD_SIZE_PAYLOAD_RSP_GET_FW_ID        (sizeof(USBPD_FWUPD_GetFwIDRspPayload_TypeDef)) /*!< Payload size of Response is used to respond to a GET_FW_ID Request             */
#define  USBPD_FWUPD_SIZE_PAYLOAD_RSP_PDFU_INITIATE    (sizeof(USBPD_FWUPD_PdfuInitRspPayload_TypeDef)) /*!< Payload size of Response is used to respond to a PDFU_INITIATE Request         */
#define  USBPD_FWUPD_SIZE_PAYLOAD_RSP_PDFU_DATA        (sizeof(USBPD_FWUPD_PdfuDataRspPayload_TypeDef)) /*!< Payload size of Response is used to respond to a PDFU_DATA Request             */
#define  USBPD_FWUPD_SIZE_PAYLOAD_RSP_PDFU_VALIDATE    (sizeof(USBPD_FWUPD_PdfuValidateRspPayload_TypeDef)) /*!< Payload size of Response is used to respond to a PDFU_VALIDATE Request         */
#define  USBPD_FWUPD_SIZE_PAYLOAD_RSP_PDFU_DATA_PAUSE  (sizeof(USBPD_FWUPD_PdfuDataPauseRspPayload_TypeDef)) /*!< Payload size of Response is used to respond to a PDFU_DATA_PAUSE Request       */
#define  USBPD_FWUPD_SIZE_PAYLOAD_RSP_VENDOR_SPECIFIC  (sizeof(USBPD_FWUPD_VendorSpecificRspPayload_TypeDef)) /*!< Payload size of Response is used to respond to a VENDOR_SPECIFIC Request       */
#define  USBPD_FWUPD_SIZE_PAYLOAD_REQ_GET_FW_ID        0u /*!< Payload size of Request is used to retrieve information */
#define  USBPD_FWUPD_SIZE_PAYLOAD_REQ_PDFU_INITIATE    8u /*!< Payload size of Request is used to initiate firmware update                    */
#define  USBPD_FWUPD_SIZE_PAYLOAD_REQ_PDFU_DATA_MAX    (2u + 256u) /*!< Payload size of Request is used to transfer a data block from a firmware image (response required) */
#define  USBPD_FWUPD_SIZE_PAYLOAD_REQ_PDFU_DATA_NR_MAX (2u + 256u) /*!< Payload size of Request is used to transfer a data block from a firmware image (response not required)                                        */
#define  USBPD_FWUPD_SIZE_PAYLOAD_REQ_PDFU_VALIDATE    0u /*!< Payload size of Request is used to request validation of a firmware image      */
#define  USBPD_FWUPD_SIZE_PAYLOAD_REQ_PDFU_ABORT       0u /*!< Payload size of Request is used to end firmware image update prematurely       */
#define  USBPD_FWUPD_SIZE_PAYLOAD_REQ_PDFU_DATA_PAUSE  0u /*!< Payload size of Request is used to pause a firmware image update               */
#define  USBPD_FWUPD_SIZE_PAYLOAD_REQ_VENDOR_SPECIFIC_MAX  (2u + 256u) /*!< Payload size of Request is for vendor-specific use                */

/**
  * @}
  */
#endif /* USBPDCORE_FWUPD */
#endif /* USBPD_REV30_SUPPORT */

/** @defgroup USBPD_SupportedSOP_TypeDef USB PD Supported SOP Types structure definition
  * @{
  */
#define USBPD_SUPPORTED_SOP_NONE       (     0u) /*<! Not supported */
#define USBPD_SUPPORTED_SOP_SOP        (1u << 0u) /*<! SOP           */
#define USBPD_SUPPORTED_SOP_SOP1       (1u << 1u) /*<! SOP '         */
#define USBPD_SUPPORTED_SOP_SOP2       (1u << 2u) /*<! SOP ''        */
#define USBPD_SUPPORTED_SOP_SOP1_DEBUG (1u << 3u) /*<! SOP' Debug    */
#define USBPD_SUPPORTED_SOP_SOP2_DEBUG (1u << 4u) /*<! SOP '' Debug  */

typedef uint32_t USBPD_SupportedSOP_TypeDef;

/**
  * @}
  */

/** @defgroup USBPD_CORE_PE_ET_HR_STATUS USBPD CORE PE Hard Reset Status
  * @{
  */
typedef enum
{
  USBPD_HR_STATUS_START_ACK,
  USBPD_HR_STATUS_START_REQ,
  USBPD_HR_STATUS_MSG_SENT,
  USBPD_HR_STATUS_WAIT_VBUS_VSAFE0V,
  USBPD_HR_STATUS_WAIT_VBUS_VSAFE5V,
  USBPD_HR_STATUS_COMPLETED,
  USBPD_HR_STATUS_FAILED,
}
USBPD_HR_Status_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_PE_ET_PRS_STATUS USBPD CORE PE Power Role Swap status
  * @{
  */
typedef enum
{
  USBPD_PRS_STATUS_NA,
  USBPD_PRS_STATUS_START_ACK,
  USBPD_PRS_STATUS_START_REQ,
  USBPD_PRS_STATUS_ACCEPTED,
  USBPD_PRS_STATUS_REJECTED,
  USBPD_PRS_STATUS_WAIT,
  USBPD_PRS_STATUS_VBUS_OFF,
  USBPD_PRS_STATUS_SRC_RP2RD,
  USBPD_PRS_STATUS_SRC_PS_READY_SENT,
  USBPD_PRS_STATUS_SNK_PS_READY_RECEIVED,
  USBPD_PRS_STATUS_SNK_RD2RP,
  USBPD_PRS_STATUS_VBUS_ON,
  USBPD_PRS_STATUS_SNK_PS_READY_SENT,
  USBPD_PRS_STATUS_SRC_PS_READY_RECEIVED,
  USBPD_PRS_STATUS_COMPLETED,
  USBPD_PRS_STATUS_FAILED,
  USBPD_PRS_STATUS_ABORTED,
} USBPD_PRS_Status_TypeDef;
/**
  * @}
  */

/**
  * @brief Status of VSafe
  * @{
  */
typedef enum
{
  USBPD_VSAFE_0V,           /*!< USBPD VSAFE0V   */
  USBPD_VSAFE_5V,           /*!< USBPD VSAFE5V   */
}
USBPD_VSAFE_StatusTypeDef;
/**
  * @}
  */

/**
  * @brief USB Power Delivery Status structures definition
  */
typedef enum
{
  USBPD_OK,
  USBPD_NOTSUPPORTED,
  USBPD_ERROR,
  USBPD_BUSY,
  USBPD_TIMEOUT,

  /* PRL status */
  USBPD_PRL_GOODCRC,
  USBPD_PRL_DISCARDED,
  USBPD_PRL_SOFTRESET,
  USBPD_PRL_CABLERESET,
#if defined(USBPD_REV30_SUPPORT)
  USBPD_PRL_SNKTX,
#endif /* USBPD_REV30_SUPPORT */

  /* Message reply */
  USBPD_ACCEPT,
  USBPD_GOTOMIN,
  USBPD_REJECT,
  USBPD_WAIT,
  USBPD_NAK,
  USBPD_ACK,

  USBPD_FAIL,
  USBPD_RXEVENT_SOP,
  USBPD_RXEVENT_SOP1,
  USBPD_RXEVENT_SOP2,
  USBPD_NOEVENT,
  USBPD_DISCARDRX,

  /* Stack initialization errors  */
  USBPD_MALLOCERROR,         /*<! Malloc error during CORE handles creation                                      */
  USBPD_INVALID_PORT_NUMBER, /*<! Exceed the maximum of supported ports by the stack (@ref USBPD_MAXPORT_COUNT)  */

  /* PDFU status  */
#if defined(USBPDCORE_FWUPD)
  USBPD_PDFU_NODATA,
  USBPD_PDFU_PAUSE,
  USBPD_PDFU_RESUME,
#endif /* USBPDCORE_FWUPD */

  USPD_ERROR_CALLBACKMISSING,
}
USBPD_StatusTypeDef;

/**
  * @brief USB PD CC lines structures definition
  */
#define CCNONE                          0x00u
#define CC1                             0x01u
#define CC2                             0x02u

typedef uint32_t CCxPin_TypeDef;


/** @defgroup USBPD_SpecRev_TypeDef USB PD Specification Revision structure definition
  * @brief  USB PD Specification Revision structure definition
  * @{
  */
#define USBPD_SPECIFICATION_REV1        0x00u  /*!< Revision 1.0      */
#define USBPD_SPECIFICATION_REV2        0x01u  /*!< Revision 2.0      */
#define USBPD_SPECIFICATION_REV3        0x02u  /*!< Revision 3.0      */

typedef uint32_t USBPD_SpecRev_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_VDM_SpecRev_TypeDef USB PD VDM Specification Revision structure definition
  * @brief  USB PD VDM Specification Revision structure definition
  * @{
  */
#define USBPD_VDMVERSION_REV1           0x00u  /*!< Revision 1.0      */
#define USBPD_VDMVERSION_REV2           0x01u  /*!< Revision 2.0 only used if USBPD_SPECIFICATION_REV3 */

typedef uint32_t USBPD_VDMVersion_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_PASSIVE_CABLE_SpecRev_TypeDef USB PD Passive Cable VDOs version structure definition
  * @brief  USB PD Passive Cable VDOs version structure definition
  * @{
  */
#define USBPD_VDM_VDO_PASSIVE_CABLE_VERSION_REV1P0 0u /*!< Version Number of the Passive calbe VDO Revision 1.0      */

typedef uint32_t USBPD_VDM_VDO_PassiveCable_Version_TypeDef;
/**
  * @}
  */

#if defined(USBPD_REV30_SUPPORT)
/** @defgroup USBPD_ACTIVE_CABLE_SpecRev_TypeDef USB PD Active Cable VDOs version structure definition
  * @brief  USB PD Active Cable VDOs version structure definition
  * @{
  */
#define USBPD_VDM_VDO_ACTIVE_CABLE_VERSION_REV1P3  3u  /*!< Version Number of the Active Passive VDO Revision 1.3    */

typedef uint32_t USBPD_VDM_VDO_ActiveCable_Version_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_VDO_UFP_SpecRev_TypeDef USB PD UFP VDO version structure definition
  * @brief  USB PD UFP VDO version structure definition
  * @{
  */
#define USBPD_VDM_VDO_UFP_VERSION_REV1P1           1u  /*!< Version Number of the UFP VDO Revision 1.1      */
#define USBPD_VDM_VDO_UFP_VERSION_REV1P2           2u  /*!< Version Number of the UFP VDO Revision 1.2      */

typedef uint32_t USBPD_VDM_VDO_UFP_Version_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_VDO_DFP_SpecRev_TypeDef USB PD DFP VDO version structure definition
  * @brief  USB PD DFP VDO version structure definition
  * @{
  */
#define USBPD_VDM_VDO_DFP_VERSION_REV1P1           1u  /*!< Version Number of the DFP VDO Revision 1.1      */

typedef uint32_t USBPD_VDM_VDO_DFP_Version_TypeDef;
/**
  * @}
  */

#if defined(USBPDCORE_VPD)
/** @defgroup USBPD_VDO_VPD_SpecRev_TypeDef USB PD Vconn Powered USB Device VDO version structure definition
  * @brief  USB PD Vconn Powered USB Device VDO version structure definition
  * @{
  */
#define USBPD_VDM_VDO_VPD_VERSION_REV1P0           0u  /*!< Version Number of the VPD VDO Revision 1.0      */

typedef uint32_t USBPD_VDM_VDO_VPD_Version_TypeDef;
/**
  * @}
  */
#endif /* USBPDCORE_VPD */
#endif /* USBPD_REV30_SUPPORT */

/**
  * @brief CAD event value
  * @{
  */
typedef enum
{
  USBPD_CAD_EVENT_NONE      = 0U,           /*!< USBPD CAD event None                                */
  USBPD_CAD_EVENT_DETACHED  = 1U,           /*!< USBPD CAD event No cable detected                   */
  USBPD_CAD_EVENT_ATTACHED  = 2U,           /*!< USBPD CAD event Port partner attached               */
  USBPD_CAD_EVENT_EMC       = 3U,           /*!< USBPD CAD event Electronically Marked Cable detected*/
  USBPD_CAD_EVENT_ATTEMC    = 4U,           /*!< USBPD CAD event Port Partner detected through EMC   */
  USBPD_CAD_EVENT_ACCESSORY = 5U,           /*!< USBPD CAD event Accessory detected                  */
  USBPD_CAD_EVENT_DEBUG     = 6U,           /*!< USBPD CAD event Debug detected                      */
  /*  USBPD_CAD_EVENT_LEGACY    = 7u  ,*/   /*!< USBPD CAD event legacy cables detected              */
  USPPD_CAD_EVENT_VPD       = 8U,           /*!< USBPD CAD event VPD                                 */
  USPPD_CAD_EVENT_UNKNOW    = 9U,           /*!< USBPD CAD event unknown                             */
  USBPD_CAD_EVENT_CABLE_ATTACHED = 10U,     /*!< USBPD CAD event cable attached                      */
  USBPD_CAD_EVENT_CABLE_DETACHED = 11u      /*!< USBPD CAD event cable detached                      */
} USBPD_CAD_EVENT;
/**
  * @}
  */

/** @defgroup USBPD_PortDataRole_TypeDef USB PD Port Data Role Types structure definition
  * @brief  USB PD Port Data Role Types structure definition
  * @{
  */
#define USBPD_PORTDATAROLE_UFP        0x00u  /*!< UFP        */
#define USBPD_PORTDATAROLE_SOP1_SOP2  USBPD_PORTDATAROLE_UFP  /*!<  For all other SOP* Packets the Port Data Role
                                                                    field is Reserved and shall be set to zero.  */
#define USBPD_PORTDATAROLE_DFP        0x01u   /*!< DFP        */

typedef uint32_t USBPD_PortDataRole_TypeDef;
/**
  * @}
  */

/**
  * @brief  USB PD Control Message Types structure definition
  *
  */
typedef enum
{
  USBPD_CONTROLMSG_GOODCRC               = 0x01U,  /*!< GoodCRC Control Message         */
  USBPD_CONTROLMSG_GOTOMIN               = 0x02U,  /*!< GotoMin Control Message         */
  USBPD_CONTROLMSG_ACCEPT                = 0x03U,  /*!< Accept Control Message          */
  USBPD_CONTROLMSG_REJECT                = 0x04U,  /*!< Reject Control Message          */
  USBPD_CONTROLMSG_PING                  = 0x05U,  /*!< Ping Control Message            */
  USBPD_CONTROLMSG_PS_RDY                = 0x06U,  /*!< PS_RDY Control Message          */
  USBPD_CONTROLMSG_GET_SRC_CAP           = 0x07U,  /*!< Get_Source_Cap Control Message  */
  USBPD_CONTROLMSG_GET_SNK_CAP           = 0x08U,  /*!< Get_Sink_Cap Control Message    */
  USBPD_CONTROLMSG_DR_SWAP               = 0x09U,  /*!< DR_Swap Control Message         */
  USBPD_CONTROLMSG_PR_SWAP               = 0x0AU,  /*!< PR_Swap Control Message         */
  USBPD_CONTROLMSG_VCONN_SWAP            = 0x0BU,  /*!< VCONN_Swap Control Message      */
  USBPD_CONTROLMSG_WAIT                  = 0x0CU,  /*!< Wait Control Message            */
  USBPD_CONTROLMSG_SOFT_RESET            = 0x0DU,  /*!< Soft_Reset Control Message      */
#if defined(USBPD_REV30_SUPPORT)
#if defined(USBPDCORE_USBDATA)
  USBPD_CONTROLMSG_DATA_RESET            = 0x0EU,  /*!< data_Reset Control Message      */
  USBPD_CONTROLMSG_DATA_RESET_COMPLETE   = 0x0FU,  /*!< data_Reset_complete Control Message*/
#endif /* USBPDCORE_USBDATA */
  USBPD_CONTROLMSG_NOT_SUPPORTED         = 0x10U,  /*!< Not supported                   */
  USBPD_CONTROLMSG_GET_SRC_CAPEXT        = 0x11U,  /*!< Get source capability extended  */
  USBPD_CONTROLMSG_GET_STATUS            = 0x12U,  /*!< Get status                      */
  USBPD_CONTROLMSG_FR_SWAP               = 0x13U,  /*!< Fast role swap                  */
  USBPD_CONTROLMSG_GET_PPS_STATUS        = 0x14U,  /*!< Get PPS Status                  */
  USBPD_CONTROLMSG_GET_COUNTRY_CODES     = 0x15U,  /*!< Get Country codes               */
#if defined(USBPDCORE_SNK_CAPA_EXT)
  USBPD_CONTROLMSG_GET_SNK_CAPEXT        = 0x16U,  /*!< Get Sink Capability extended    */
#endif /* USBPDCORE_SNK_CAPA_EXT */

  USBPD_CONTROLMSG_GET_REVISION          = 0x18U,  /*!< Get revision                     */
#endif /* USBPD_REV30_SUPPORT */
} USBPD_ControlMsg_TypeDef;

/**
  * @brief  USB PD Data Message Types structure definition
  *
  */
typedef enum
{
  USBPD_DATAMSG_SRC_CAPABILITIES         = 0x01U,  /*!< Source Capabilities Data Message  */
  USBPD_DATAMSG_REQUEST                  = 0x02U,  /*!< Request Data Message              */
  USBPD_DATAMSG_BIST                     = 0x03U,  /*!< BIST Data Message                 */
  USBPD_DATAMSG_SNK_CAPABILITIES         = 0x04U,  /*!< Sink_Capabilities Data Message    */
#if defined(USBPD_REV30_SUPPORT)
  USBPD_DATAMSG_BATTERY_STATUS           = 0x05U,  /*!< Battery status                    */
  USBPD_DATAMSG_ALERT                    = 0x06U,  /*!< Alert                             */
  USBPD_DATAMSG_GET_COUNTRY_INFO         = 0x07U,  /*!< Get country info                  */
  USBPD_DATAMSG_REVISION                 = 0x0Cu,  /*!< Revision                          */
#if defined(USBPDCORE_USBDATA)
  USBPD_DATAMSG_ENTER_USB                = 0x08U,  /*!< Enter usb                         */
#endif /* USBPDCORE_USBDATA */
#endif /* USBPD_REV30_SUPPORT */
  USBPD_DATAMSG_VENDOR_DEFINED           = 0x0Fu   /*!< Vendor_Defined Data Message       */
} USBPD_DataMsg_TypeDef;

/**
  * @brief Sink CC pins Multiple Source Current Advertisements
  */
#define vRd_Undefined     0x00u    /*!< Port Power Role Source   */
#define vRd_USB           0x01u    /*!< Default USB Power   */
#define vRd_1_5A          0x02u    /*!< USB Type-C Current @ 1.5 A   */
#define vRd_3_0A          0x03u    /*!< USB Type-C Current @ 3 A   */

typedef uint32_t CAD_SNK_Source_Current_Adv_Typedef;


/**
  * @brief Sink CC pins Multiple Source Current Advertisements
  */
#define vRp_Default             0x00u    /*!< Default USB Power   */
#define vRp_1_5A                0x01u    /*!< USB Type-C Current @ 1.5 A   */
#define vRp_3_0A                0x02u    /*!< USB Type-C Current @ 3 A   */

typedef uint32_t CAD_RP_Source_Current_Adv_Typedef;

/**
  * @brief USB PD SOP Message Types Structure definition
  */
#define USBPD_SOPTYPE_SOP            0u     /*!< SOP*  MESSAGES               */
#define USBPD_SOPTYPE_SOP1           1u     /*!< SOP'  MESSAGES               */
#define USBPD_SOPTYPE_SOP2           2u     /*!< SOP'' MESSAGES               */
#define USBPD_SOPTYPE_SOP1_DEBUG     3u     /*!< SOP'  DEBUG_MESSAGES         */
#define USBPD_SOPTYPE_SOP2_DEBUG     4u     /*!< SOP'' DEBUG_MESSAGES         */
#define USBPD_SOPTYPE_HARD_RESET     5u     /*!< HARD RESET MESSAGE           */
#define USBPD_SOPTYPE_CABLE_RESET    6u     /*!< CABLE RESET MESSAGE          */
#define USBPD_SOPTYPE_BIST_MODE_2    7u     /*!< BIST_MODE2 MESSAGE           */
#define USBPD_SOPTYPE_INVALID        0xFFu  /*!< Invalid type                 */
#define USBPD_SOPType_TypeDef uint8_t

/**
  * @brief USB funtionnal state Types enum definition
  *
  */
typedef enum
{
  USBPD_DISABLE = 0U,
  USBPD_ENABLE = !USBPD_DISABLE
} USBPD_FunctionalState;


/**
  * @brief USB PD Port Power Role Types structure definition
  *
  */
#define USBPD_CABLEPLUG_FROMDFPUFP      0x00u                           /*!< Message originated from a DFP or UFP    */
#define USBPD_PORTPOWERROLE_SNK         USBPD_CABLEPLUG_FROMDFPUFP      /*!< Sink                                    */
#define USBPD_CABLEPLUG_FROMCABLEPLUG   0x01u                           /*!< Message originated from a Cable Plug    */
#define USBPD_PORTPOWERROLE_SRC         USBPD_CABLEPLUG_FROMCABLEPLUG   /*!< Source                                  */

typedef uint32_t USBPD_PortPowerRole_TypeDef;

/**
  * @brief  USB PD Extended Message Types structure definition
  *
  */
#define USBPD_EXT_NONE                  0x00u
#define USBPD_EXT_SOURCE_CAPABILITIES   0x01u  /*!< sent by Source or Dual-Role Power    - SOP only  */
#define USBPD_EXT_STATUS                0x02u  /*!< sent by Source                       - SOP only  */
#define USBPD_EXT_GET_BATTERY_CAP       0x03u  /*!< sent by Source or Sink               - SOP only  */
#define USBPD_EXT_GET_BATTERY_STATUS    0x04u  /*!< sent by Source or Sink               - SOP only  */
#define USBPD_EXT_BATTERY_CAPABILITIES  0x05u  /*!< sent by Source or Sink               - SOP only  */
#define USBPD_EXT_GET_MANUFACTURER_INFO 0x06u  /*!< sent by Source or Sink or Cable Plug - SOP*      */
#define USBPD_EXT_MANUFACTURER_INFO     0x07u  /*!< sent by Source or Sink or Cable Plug - SOP*      */
#define USBPD_EXT_SECURITY_REQUEST      0x08u  /*!< sent by Source or Sink               - SOP*      */
#define USBPD_EXT_SECURITY_RESPONSE     0x09u  /*!< sent by Source or Sink or Cable Plug - SOP*      */
#define USBPD_EXT_FIRM_UPDATE_REQUEST   0x0Au  /*!< sent by Source or Sink               - SOP*      */
#define USBPD_EXT_FIRM_UPDATE_RESPONSE  0x0Bu  /*!< sent by Source or Sink or Cable Plug - SOP*      */
#define USBPD_EXT_PPS_STATUS            0x0Cu  /*!< sent by Source                       - SOP only  */
#define USBPD_EXT_COUNTRY_INFO          0x0Du  /*!< sent by Source or Sink               - SOP only  */
#define USBPD_EXT_COUNTRY_CODES         0x0Eu  /*!< sent by Source or Sink               - SOP only  */
#if defined(USBPDCORE_SNK_CAPA_EXT)
#define USBPD_EXT_SINK_CAPABILITIES     0x0Fu  /*!< sent by Sink or Dual-Role Power      - SOP only  */
#endif /* USBPDCORE_SNK_CAPA_EXT */

typedef uint8_t USBPD_ExtendedMsg_TypeDef;

/**
  * @brief  USB PD BIST Mode Types structure definition
  *
  */
typedef enum
{
  USBPD_BIST_CARRIER_MODE2              = 0x05U,  /*!< Request Transmitter to enter BIST Carrier Mode   */
  USBPD_BIST_TEST_DATA                  = 0x08U,  /*!< Sends a Test Data Frame.                         */
#if defined(USBPDCORE_DRP) || defined(USBPDCORE_SRC)
  USBPD_BIST_SHARED_TEST_MODE_ENTRY     = 0x09U,  /*!< Requests UUT to enter Shared Capacity Test Mode  */
  USBPD_BIST_SHARED_TEST_MODE_EXIT      = 0x0AU,  /*!< Requests UUT to exit Shared Capacity Test Mode   */
#endif /* USBPDCORE_DRP || USBPDCORE_SRC */
} USBPD_BISTMsg_TypeDef;

/** @defgroup USBPD_CORE_PDO_Type_TypeDef PDO type definition
  * @brief  PDO type values in PDO definition
  * @{
  */
#define USBPD_CORE_PDO_TYPE_FIXED       0x00u            /*!< Fixed Supply PDO                             */
#define USBPD_CORE_PDO_TYPE_BATTERY     0x01u            /*!< Battery Supply PDO                           */
#define USBPD_CORE_PDO_TYPE_VARIABLE    0x02u            /*!< Variable Supply (non-battery) PDO            */
#if defined(USBPD_REV30_SUPPORT) && defined(USBPDCORE_PPS)
#define USBPD_CORE_PDO_TYPE_APDO        0x03u            /*!< Augmented Power Data Object (APDO)           */
#endif /*_USBPD_REV30_SUPPORT && PPS*/

typedef uint32_t USBPD_CORE_PDO_Type_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_POWER_Type_TypeDef USB-PD power state
  * @brief  USB Power state
  * @{
  */
#define USBPD_POWER_NO                  0x0u /*!< No power contract                      */
#define USBPD_POWER_DEFAULT5V           0x1u /*!< Default 5V                             */
#define USBPD_POWER_IMPLICITCONTRACT    0x2u /*!< Implicit contract                      */
#define USBPD_POWER_EXPLICITCONTRACT    0x3u /*!< Explicit contract                      */
#define USBPD_POWER_TRANSITION          0x4u /*!< Power transition                       */

typedef uint32_t USBPD_POWER_StateTypedef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_DRPowerSupport_TypeDef DRP Support type
  * @brief  DRP support values in PDO definition (Source or Sink)
  * @{
  */
#define USBPD_CORE_PDO_DRP_NOT_SUPPORTED        0x00u          /*!< Dual Role Power not supported                */
#define USBPD_CORE_PDO_DRP_SUPPORTED            0x01u          /*!< Dual Role Power supported                    */

typedef uint32_t USBPD_CORE_PDO_DRPowerSupport_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_USBSuspendSupport_TypeDef USB Suspend type
  * @brief  USB Suspend support values in PDO definition (Source)
  * @{
  */
#define USBPD_CORE_PDO_USBSUSP_NOT_SUPPORTED    0x00u      /*!< USB Suspend not supported                    */
#define USBPD_CORE_PDO_USBSUSP_SUPPORTED        0x01u      /*!< USB Suspend supported                        */

typedef uint32_t USBPD_CORE_PDO_USBSuspendSupport_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_ExtPowered_TypeDef Externally Powered type
  * @brief  Fixed Power Source Externally Powered indication values in PDO definition (Source or Sink)
  * @{
  */
#define USBPD_CORE_PDO_NOT_EXT_POWERED  0x00u            /*!< No external power source is available        */
#define USBPD_CORE_PDO_EXT_POWERED      0x01u            /*!< External power source is available           */

typedef uint32_t USBPD_CORE_PDO_ExtPowered_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_USBCommCapable_TypeDef USB Communication capability type
  * @brief  USB Communication capability over USB Data lines indication values in PDO definition (Source or Sink)
  * @{
  */
#define USBPD_CORE_PDO_USBCOMM_NOT_CAPABLE      0x00u  /*!< Device not capable of communication over USB Data lines */
#define USBPD_CORE_PDO_USBCOMM_CAPABLE          0x01u  /*!< Device capable of communication over USB Data lines     */

typedef uint32_t USBPD_CORE_PDO_USBCommCapable_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_DRDataSupport_TypeDef Dual Role Data Support type
  * @brief  Dual Role Data support values in PDO definition (Source or Sink)
  * @{
  */
#define USBPD_CORE_PDO_DRD_NOT_SUPPORTED        0x00u  /*!< Dual Role Data not supported                 */
#define USBPD_CORE_PDO_DRD_SUPPORTED            0x01u  /*!< Dual Role Data supported                     */

typedef uint32_t USBPD_CORE_PDO_DRDataSupport_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_PeakCurr_TypeDef Peak Current Capability type
  * @brief  Fixed Power Source Peak Current Capability type structure definition (Source)
  * @{
  */
#define   USBPD_CORE_PDO_PEAKEQUAL 0x00u                  /*!< Peak current equals                          */
#define   USBPD_CORE_PDO_PEAKOVER1 0x01u                  /*!< Overload Capabilities:
  1. Peak current equals 150% IOC for 1ms @ 5% duty cycle (low current equals 97% IOC for 19ms)
  2. Peak current equals 125% IOC for 2ms @ 10% duty cycle (low current equals 97% IOC for 18ms)
  3. Peak current equals 110% IOC for 10ms @ 50% duty cycle (low current equals 90% IOC for 10ms */
#define   USBPD_CORE_PDO_PEAKOVER2 0x02U                  /*!< Overload Capabilities:
  1. Peak current equals 200% IOC for 1ms @ 5% duty cycle (low current equals 95% IOC for 19ms)
  2. Peak current equals 150% IOC for 2ms @ 10% duty cycle (low current equals 94% IOC for 18ms)
  3. Peak current equals 125% IOC for 10ms @ 50% duty cycle (low current equals 75% IOC for 10ms)*/
#define   USBPD_CORE_PDO_PEAKOVER3 0x03u                  /*!< Overload Capabilities:
  1. Peak current equals 200% IOC for 1ms @ 5% duty cycle (low current equals 95% IOC for 19ms)
  2. Peak current equals 175% IOC for 2ms @ 10% duty cycle (low current equals 92% IOC for 18ms)
  3. Peak current equals 150% IOC for 10ms @ 50% duty cycle (low current equals 50% IOC for 10ms)*/

typedef uint32_t USBPD_CORE_PDO_PeakCurr_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_HigherCapability_TypeDef USB Higher Capability type
  * @brief  Values in PDO definition (Sink) indicating if Sink needs more than vSafe5V to provide full functionality
  * @{
  */
typedef enum
{
  USBPD_CORE_PDO_NO_HIGHER_CAPABILITY  = 0x00U,      /*!< No need for more than vSafe5V to provide full functionality */
  USBPD_CORE_PDO_HIGHER_CAPABILITY     = 0x01U,      /*!< Sink needs more than vSafe5V to provide full functionality  */
} USBPD_CORE_PDO_HigherCapability_TypeDef;
/**
  * @}
  */

#if defined(USBPD_REV30_SUPPORT)
/** @defgroup USBPD_CORE_PDO_UnchunkSupport_TypeDef Unchunked Extended Messages Support type
  * @brief  Unchunked Extended Messages support values in PDO definition (Source)
  * @{
  */
typedef enum
{
  USBPD_CORE_PDO_UNCHUNK_NOT_SUPPORTED = 0x00U,      /*!< Unchunked Extended Messages not supported    */
  USBPD_CORE_PDO_UNCHUNK_SUPPORTED     = 0x01U,      /*!< Unchunked Extended Messages supported        */
} USBPD_CORE_PDO_UnchunkSupport_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_FastRoleSwapRequiredCurrent_TypeDef Fast Role Swap Required current type
  * @brief  Fast Role Swap Required Current values in PDO definition (Sink)
  * @{
  */
typedef enum
{
  USBPD_CORE_PDO_FRS_NOT_SUPPORTED      = 0x00U,     /*!< Fast Role Swap not supported    */
  USBPD_CORE_PDO_FRS_DEFAULT_USB_POWER  = 0x01U,     /*!< Default USB Power               */
  USBPD_CORE_PDO_FRS_1_5A_5V            = 0x02U,     /*!< 1_5A at 5V supported            */
  USBPD_CORE_PDO_FRS_3A_5V              = 0x03U,     /*!< 3A at 5V supported              */
} USBPD_CORE_PDO_FastRoleSwapRequiredCurrent_TypeDef;
/**
  * @}
  */
#endif /*_USBPD_REV30_SUPPORT*/

/** @defgroup USBPD_NotifyEventValue_TypeDef USBPD notification event type value
  * @brief notification envent used inside PE callbacks (USBPD_PE_NotifyDPM) to inform DPM
  * @{
  */
typedef enum
{
  USBPD_NOTIFY_REQUEST_ACCEPTED        = 1U,
  USBPD_NOTIFY_REQUEST_REJECTED        = 2U,
  USBPD_NOTIFY_REQUEST_WAIT            = 3U,
  USBPD_NOTIFY_REQUEST_GOTOMIN         = 4U,
  USBPD_NOTIFY_GETSNKCAP_SENT          = 5U,
  USBPD_NOTIFY_GETSNKCAP_RECEIVED      = 6U,
  USBPD_NOTIFY_GETSNKCAP_ACCEPTED      = 7U,
  USBPD_NOTIFY_GETSNKCAP_REJECTED      = 8U,
  USBPD_NOTIFY_GETSNKCAP_TIMEOUT       = 9U,
  USBPD_NOTIFY_SNKCAP_SENT             = 10U,
  USBPD_NOTIFY_GETSRCCAP_SENT          = 11U,
  USBPD_NOTIFY_GETSRCCAP_RECEIVED      = 12U,
  USBPD_NOTIFY_GETSRCCAP_ACCEPTED      = 13U,
  USBPD_NOTIFY_GETSRCCAP_REJECTED      = 14U,
  USBPD_NOTIFY_SRCCAP_SENT             = 15U,
  USBPD_NOTIFY_POWER_EXPLICIT_CONTRACT = 16U,
  USBPD_NOTIFY_POWER_SRC_READY         = 17U,
  USBPD_NOTIFY_POWER_SNK_READY         = 18U,
  USBPD_NOTIFY_POWER_SNK_STOP          = 19U,
  USBPD_NOTIFY_POWER_SWAP_TO_SNK_DONE  = 20U,
  USBPD_NOTIFY_POWER_SWAP_TO_SRC_DONE  = 21U,
  USBPD_NOTIFY_POWER_SWAP_REJ          = 22U,
  USBPD_NOTIFY_POWER_SWAP_NOT_SUPPORTED = 23U,
  USBPD_NOTIFY_RESISTOR_ASSERT_RP      = 24U,
  USBPD_NOTIFY_RESISTOR_ASSERT_RD      = 25U,
  USBPD_NOTIFY_CABLERESET_REQUESTED    = 26U,
  USBPD_NOTIFY_PROTOCOL_ERROR          = 27U,
  USBPD_NOTIFY_VCONN_SWAP_NOT_COMPLETED = 28U,
  /*USBPD_NOTIFY_SVDM_TIMEOUT            =29U,*/
  USBPD_NOTIFY_HARDRESET_RX            = 30U,
  USBPD_NOTIFY_HARDRESET_TX            = 31U,
  USBPD_NOTIFY_STATE_SNK_READY         = 32U,
  USBPD_NOTIFY_STATE_SRC_DISABLED      = 33U,
  USBPD_NOTIFY_DATAROLESWAP_SENT       = 34U,
  USBPD_NOTIFY_DATAROLESWAP_RECEIVED   = 35U,
  USBPD_NOTIFY_DATAROLESWAP_UFP        = 36U,
  USBPD_NOTIFY_DATAROLESWAP_DFP        = 37U,
  USBPD_NOTIFY_DATAROLESWAP_WAIT       = 38U,
  USBPD_NOTIFY_DATAROLESWAP_REJECTED   = 39U,
  USBPD_NOTIFY_DATAROLESWAP_NOT_SUPPORTED = 40U,
  USBPD_NOTIFY_GOTOMIN_SENT            = 41U,
  USBPD_NOTIFY_GOTOMIN_POWERREADY      = 42U,
  USBPD_NOTIFY_SNK_GOTOMIN             = 43U,
  USBPD_NOTIFY_SNK_GOTOMIN_READY       = 44U,
  USBPD_NOTIFY_REQUEST_ERROR           = 45U,
  USBPD_NOTIFY_REQUEST_COMPLETE        = 46U,
  USBPD_NOTIFY_REQUEST_CANCELED        = 47U,
  USBPD_NOTIFY_SOFTRESET_SENT          = 48U,
  USBPD_NOTIFY_SOFTRESET_ACCEPTED      = 49U,
  USBPD_NOTIFY_SOFTRESET_RECEIVED      = 50U,
  USBPD_NOTIFY_PING_RECEIVED           = 51U,
  USBPD_NOTIFY_REQUEST_ENTER_MODE      = 52U,
  USBPD_NOTIFY_REQUEST_ENTER_MODE_ACK  = 53U,
  USBPD_NOTIFY_REQUEST_ENTER_MODE_NAK  = 54U,
  USBPD_NOTIFY_REQUEST_ENTER_MODE_BUSY = 55U,
  USBPD_NOTIFY_PD_SPECIFICATION_CHANGE = 56U,
  USBPD_NOTIFY_POWER_SWAP_SENT         = 57U,
  USBPD_NOTIFY_POWER_SWAP_ACCEPTED     = 58U,
  USBPD_NOTIFY_POWER_SWAP_WAIT         = 59U,
  USBPD_NOTIFY_POWER_SWAP_RECEIVED     = 60U,
  USBPD_NOTIFY_VCONN_SWAP_RECEIVED     = 61U,
  USBPD_NOTIFY_VCONN_SWAP_SENT         = 62U,
  USBPD_NOTIFY_VCONN_SWAP_ACCEPTED     = 63U,
  USBPD_NOTIFY_VCONN_SWAP_WAIT         = 64U,
  USBPD_NOTIFY_VCONN_SWAP_REJECTED     = 65U,
  USBPD_NOTIFY_VCONN_SWAP_COMPLETE     = 66U,
  USBPD_NOTIFY_VCONN_SWAP_NOT_SUPPORTED = 67U,
  USBPD_NOTIFY_CTRL_MSG_SENT           = 68U,
  USBPD_NOTIFY_DATA_MSG_SENT           = 69U,
  USBPD_NOTIFY_GET_SRC_CAP_EXT_RECEIVED = 70U,
  USBPD_NOTIFY_SRC_CAP_EXT_RECEIVED    = 71U,
  USBPD_NOTIFY_SRC_CAP_EXT_SENT        = 72U,
  USBPD_NOTIFY_GET_PPS_STATUS_RECEIVED = 73U,
  USBPD_NOTIFY_GET_PPS_STATUS_SENT     = 74U,
  USBPD_NOTIFY_PPS_STATUS_RECEIVED     = 75U,
  USBPD_NOTIFY_PPS_STATUS_SENT         = 76U,
  USBPD_NOTIFY_GET_STATUS_RECEIVED     = 77U,
  USBPD_NOTIFY_STATUS_RECEIVED         = 78U,
  USBPD_NOTIFY_STATUS_SENT             = 79U,
  USBPD_NOTIFY_ALERT_RECEIVED          = 80U,
  USBPD_NOTIFY_VDM_IDENTIFY_RECEIVED   = 81U,
  USBPD_NOTIFY_VDM_CABLE_IDENT_RECEIVED = 82U,
  USBPD_NOTIFY_VDM_SVID_RECEIVED       = 83U,
  USBPD_NOTIFY_VDM_MODE_RECEIVED       = 84U,
  USBPD_NOTIFY_REQUEST_EXIT_MODE       = 85U,
  USBPD_NOTIFY_REQUEST_EXIT_MODE_ACK   = 86U,
  USBPD_NOTIFY_REQUEST_EXIT_MODE_NAK   = 87U,
  USBPD_NOTIFY_REQUEST_EXIT_MODE_BUSY  = 88U,
  USBPD_NOTIFY_MSG_NOT_SUPPORTED       = 89U,
  USBPD_NOTIFY_POWER_STATE_CHANGE      = 90U,
  USBPD_NOTIFY_REQUEST_DISCARDED       = 91U,
  USBPD_NOTIFY_AMS_INTERRUPTED         = 92U,
  USBPD_NOTIFY_ALERT_SENT              = 93U,
  USBPD_NOTIFY_CABLERESET_TX           = 94U,
  USBPD_NOTIFY_PE_DISABLED             = 95U,
  USBPD_NOTIFY_GET_SNK_CAP_EXT_RECEIVED = 96U,
  USBPD_NOTIFY_SNK_CAP_EXT_SENT        = 97U,
  USBPD_NOTIFY_SNK_CAP_EXT_RECEIVED    = 98U,
  USBPD_NOTIFY_DETACH                  = 99U,
  USBPD_NOTIFY_CABLERESET_RX           = 100U,
  USBPD_NOTIFY_BIST_SHARED_TEST_MODE_ENTRY = 101U,
  USBPD_NOTIFY_BIST_SHARED_TEST_MODE_EXIT  = 102U,
  USBPD_NOTIFY_STATE_SRC_READY             = 103U,
  USBPD_NOTIFY_USBSTACK_START              = 104U,
  USBPD_NOTIFY_USBSTACK_STOP               = 105U,
  USBPD_NOTIFY_ENTERUSB_INVALID            = 106U,
  USBPD_NOTIFY_ENTERUSB_SENT               = 107U,
  USBPD_NOTIFY_ENTERUSB_ACCEPTED           = 108U,
  USBPD_NOTIFY_ENTERUSB_REJECTED           = 109U,
  USBPD_NOTIFY_DATARESET_EXECUTE           = 110U,
  USBPD_NOTIFY_DATARESET_RESTORE           = 111U,
  USBPD_NOTIFY_ALL                         = USBPD_NOTIFY_DATARESET_RESTORE + 1U,
} USBPD_NotifyEventValue_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_VDM_Exported_Defines USBPD CORE VDM Exported Defines
  * @{
  */
#define MAX_MODES_PER_SVID  6u

/**
  * @brief Product Type field in ID Header
  * @{
  */

/* ##### SOP #####*/
/* Product Type (UFP/DFP): */
#define PRODUCT_TYPE_HUB                1u /*!< PDUSB Hub (UFP or DFP)                        */

/* Product Type (UFP): */
#define PRODUCT_TYPE_NOT_UFP            0u /*!< Not a UFP                              */
#define PRODUCT_TYPE_PERIPHERAL         2u /*!< PDUSB Peripheral (UFP)                       */
#if defined(USBPD_REV30_SUPPORT)
#define PRODUCT_TYPE_PSD                3u /*!< PSD, e.g. power bank (UFP)             */
#endif /* USBPD_REV30_SUPPORT */

/* Product Type (DFP): */
#define PRODUCT_TYPE_NOT_DFP            0u /*!< Not a DFP                              */
#if defined(USBPD_REV30_SUPPORT)
#define PRODUCT_TYPE_HOST               2u /*!< PDUSB Host  (DFP)                      */
#define PRODUCT_TYPE_POWER_BRICK        3u /*!< Power Brick (DFP)                      */
#endif /* USBPD_REV30_SUPPORT */

/* ##### SOP1 (Cable Plug/VPD) #####*/
/* Product Type (Cable Plug): */
#define PRODUCT_TYPE_PASSIVE_CABLE      3u /*!< Passive Cable (Cable Plug)             */
#define PRODUCT_TYPE_ACTIVE_CABLE       4u /*!< Active Cable (Cable Plug)              */
#if defined(USBPD_REV30_SUPPORT) && defined(USBPDCORE_VPD)
#define PRODUCT_TYPE_VPD                6u /*!< VCONN-Powered USB Device (VPD)   */
#endif /* USBPD_REV30_SUPPORT && USBPDCORE_VPD */

typedef uint32_t USBPD_ProductType_TypeDef;

/* Keep for PD2.0 legacy reasons - should NOT be more used in PD3.0 */
#define PRODUCT_TYPE_UNDEFINED          PRODUCT_TYPE_NOT_UFP /* or PRODUCT_TYPE_NOT_DFP if DFP Product type */
/* Product Type (UFP): */
#define PRODUCT_TYPE_AMA                5u /*!< NOT be more used - Alternate Mode Adapter (AMA) (UFP)     */
#if defined(USBPD_REV30_SUPPORT)
#define PRODUCT_TYPE_AMC                4u /*!< NOT be more used - Alternate Mode Controller (AMC) (DFP) */
#endif /* USBPD_REV30_SUPPORT */
/**
  * @}
  */

/**
  * @brief Connector Type field in ID Header
  * @{
  */

#define CONNECTOR_TYPE_RESERVED         0u /*!< Reserved, for compatibility with legacy systems. */
#define CONNECTOR_TYPE_C_RECEPTACLE     2u /*!< USB Type-C Receptacle                            */
#define CONNECTOR_TYPE_C_PLUG           3u /*!< USB Type-C Plug                                  */
typedef uint32_t USBPD_ConnectorType_TypeDef;

/**
  * @}
  */

/**
  * @brief USB Host or Device Capability field in ID Header
  * @{
  */
#define USB_NOTCAPABLE                  0u /*!< Not USB capable                                     */
#define USB_CAPABLE                     1u /*!< Capable of being enumerated as a USB host or device */

typedef uint32_t USBPD_USBCapa_TypeDef;

/**
  * @}
  */

/**
  * @brief Modal operation field in ID Header
  * @{
  */
#define MODAL_OPERATION_NONSUPP         0u /*!< Product not supports Modal Operation. */
#define MODAL_OPERATION_SUPPORTED       1u /*!< Product supports Modal Operation.     */

typedef uint32_t USBPD_ModalOp_TypeDef;

/**
  * @}
  */

/**
  * @brief Cable to USB field in Active/Passive cable
  * @{
  */
#define CABLE_TO_TYPE_A         0u /*!< USB Type-A (PD 2.0 only)  */
#define CABLE_TO_TYPE_B         1u /*!< USB Type-B (PD 2.0 only)  */
#define CABLE_TO_TYPE_C         2u /*!< USB Type-C                */
#define CABLE_CAPTIVE           3u /*!< Captive                   */

typedef uint32_t USBPD_CableToType;

/**
  * @}
  */

/**
  * @brief  cable latency values in nanoseconds (max) in Active/Passive cable
  * @{
  */
typedef enum
{
  CABLE_LATENCY_10NS      = 1U,  /*!< <10ns (~1m)        */
  CABLE_LATENCY_20NS      = 2U,  /*!< 10ns to 20ns (~2m) */
  CABLE_LATENCY_30NS      = 3U,  /*!< 20ns to 30ns (~3m) */
  CABLE_LATENCY_40NS      = 4U,  /*!< 30ns to 40ns (~4m) */
  CABLE_LATENCY_50NS      = 5U,  /*!< 40ns to 50ns (~5m) */
  CABLE_LATENCY_60NS      = 6U,  /*!< 50ns to 60ns (~6m) */
  CABLE_LATENCY_70NS      = 7U,  /*!< 60ns to 70ns (~7m) */
  CABLE_LATENCY_1000NS    = 8u   /*!< > 70ns (>~7m) for P2.0 or 1000ns  (~100m) for P3.0    */
#if defined(USBPD_REV30_SUPPORT)
  , CABLE_LATENCY_2000NS    = 9U, /*!< 2000ns (~200m)     */
  CABLE_LATENCY_3000NS    = 10u  /*!< 3000ns (~300m)     */
#endif /* USBPD_REV30_SUPPORT */
} USBPD_CableLatency;

/**
  * @}
  */

/**
  * @brief  Cable maximum VBUS voltage
  * @{
  */
#define VBUS_MAX_20V                    0u
#define VBUS_MAX_30V                    1u
#define VBUS_MAX_40V                    2u
#define VBUS_MAX_50V                    3u

typedef uint32_t USBPD_CableMaxVoltage;

/**
  * @}
  */

/**
  * @brief  Cable Termination Type in Active/Passive cable
  * @{
  */
#define CABLE_TERM_BOTH_PASSIVE_NO_VCONN        0u  /*!< VCONN not required (PD2.0 only) */
#define CABLE_TERM_BOTH_PASSIVE_VCONN           1u   /*!< VCONN required (PD2.0 only)     */
#if defined(USBPD_REV30_SUPPORT)
#define CABLE_TERM_ONE_EACH_VCONN               2u  /*!< One end Active, one end passive, VCONN required */
#define CABLE_TERM_BOTH_ACTIVE_VCONN            3u   /*!< Both ends Active, VCONN required  */
#endif /* USBPD_REV30_SUPPORT */

typedef uint32_t USBPD_CableTermType;

/**
  * @}
  */

#if defined(USBPD_REV30_SUPPORT)
/**
  * @brief  Maximum Cable VBUS Voltage in Active/Passive cable
  * @{
  */
#define VBUS_20V                0u /*!< Maximum Cable VBUS Voltage 20V */
#define VBUS_30V                1u /*!< Maximum Cable VBUS Voltage 30V */
#define VBUS_40V                2u /*!< Maximum Cable VBUS Voltage 40V */
#define VBUS_50V                3u /*!< Maximum Cable VBUS Voltage 50V */

typedef uint32_t USBPD_VBUSMaxVoltage;

/**
  * @}
  */

/**
  * @brief  Active cable - SBU Supported
  * @{
  */
#define ACTIVE_CABLE_SBU_NOT_SUPPORTED 0u /*!< SBUs connections not supported */
#define ACTIVE_CABLE_SBU_SUPPORTED     1u /*!< SBUs connections supported */

typedef uint32_t USBPD_ActiveCableSBUSupported;

/**
  * @}
  */

/**
  * @brief  Active cable - SBU Type (valid only if SBU Connection is supported @ref ACTIVE_CABLE_SBU_SUPPORTED)
  * @{
  */
#define ACTIVE_CABLE_SBU_TYPE_PASSIVE 0u /*!< SBU is passive */
#define ACTIVE_CABLE_SBU_TYPE_ACTIVE  1u /*!< SBU is active */

typedef uint32_t USBPD_ActiveCableSBUType;

/**
  * @}
  */

/**
  * @brief UFP VDO - Device capability
  * @{
  */
#define DEVICE_CAPABILITY_USB2P0            0u /*!< [USB 2.0] Device Capable */
#define DEVICE_CAPABILITY_USB2P0_BILLBOARD  1u /*!< [USB 2.0] Device Capable (Billboard only) */
#define DEVICE_CAPABILITY_USB3P2            2u /*!< [USB 3.2] Device Capable */
#define DEVICE_CAPABILITY_USB4              3u /*!< [USB4] Device Capable */

typedef uint32_t USBPD_VDO_UFP_DeviceCapability;

/**
  * @}
  */

/**
  * @brief UFP VDO - Alternate Modes
  * @{
  */
#define ALTERNATE_MODES_TBT3                  0u /*!< Supports [TBT3] Alternate Mode */
#define ALTERNATE_MODES_RECONFIG_TYPEC_2P0    1u /*!< Supports Alternate Modes that reconfigure the signals
                                                      on the [USB Type-C 2.0] connector - except for [TBT3]. */
#define ALTERNATE_MODES_NO_RECONFIG_TYPEC_2P0 2u /*!< Supports Alternate Modes that do not reconfigure the signals
                                                      on the [USB Type-C 2.0] connector */

typedef uint32_t USBPD_VDO_UFP_AlternateModes;

/**
  * @}
  */

/**
  * @brief DFP VDO - Host capability
  * @{
  */
#define HOST_CAPABILITY_USB2P0            0u /*!< [USB 2.0] Host Capable  */
#define HOST_CAPABILITY_USB3P2            1u /*!< [USB 3.2] Host Capable  */
#define HOST_CAPABILITY_USB4              2u /*!< [USB4] Host Capable     */

typedef uint32_t USBPD_VDO_DFP_HostCapability;

/**
  * @}
  */

#endif /* USBPD_REV30_SUPPORT */

/**
  * @brief  configurability of SS Directionality in Active/Passive cable and AMA VDO (PD2.0 only)
  * @{
  */
#define SS_DIR_FIXED            0u /*!< SSTX Directionality Support Fixed        */
#define SS_DIR_CONFIGURABLE     1u /*!< SSTX Directionality Support Configurable */

typedef uint32_t USBPD_SsDirectionality;

/**
  * @}
  */

/**
  * @brief SVDM commands definition
  */
#define SVDM_RESERVEDCOMMAND    0x00u
#define SVDM_DISCOVER_IDENTITY  0x01u
#define SVDM_DISCOVER_SVIDS     0x02u
#define SVDM_DISCOVER_MODES     0x03u
#define SVDM_ENTER_MODE         0x04u
#define SVDM_EXIT_MODE          0x05u
#define SVDM_ATTENTION          0x06u
#define SVDM_SPECIFIC_1         0x10u
#define SVDM_SPECIFIC_2         0x11u
#define SVDM_SPECIFIC_3         0x12u
#define SVDM_SPECIFIC_4         0x13u
#define SVDM_SPECIFIC_5         0x14u
#define SVDM_SPECIFIC_6         0x15u
#define SVDM_SPECIFIC_7         0x16u
#define SVDM_SPECIFIC_8         0x17u
#define SVDM_SPECIFIC_9         0x18u
#define SVDM_SPECIFIC_10        0x19u
#define SVDM_SPECIFIC_11        0x1Au
#define SVDM_SPECIFIC_12        0x1Bu
#define SVDM_SPECIFIC_13        0x1Cu
#define SVDM_SPECIFIC_14        0x1Du
#define SVDM_SPECIFIC_15        0x1Eu
#define SVDM_SPECIFIC_16        0x1Fu

typedef uint32_t USBPD_VDM_Command_Typedef;

/**
  * @brief  VBUS Current Handling Capability in Active/Passive cable VDO
  * @{
  */
#define VBUS_DEFAULT            0u /*!< USB Type-C Default Current */
#define VBUS_3A                 1u /*!< VBUS  Current Handling Capability 3A */
#define VBUS_5A                 2u /*!< VBUS  Current Handling Capability 5A */

typedef uint32_t USBPD_VBUSCurrentHandCap;

/**
  * @}
  */

/**
  * @brief  USB Superspeed Signaling Support in Active/Passive cable VDO
  * @{
  */
#define USB2P0_ONLY             0u /*!< [USB 2.0] only, no SuperSpeed support */
#define USB3P2_GEN1             1u /*!< [USB 3.2] Gen1 */
#define USB3P2_USB4_GEN2        2u /*!< [USB 3.2]/[USB4] Gen2 */
#if defined(USBPD_REV30_SUPPORT)
#define USB4_GEN3               3u /*!< [USB4] Gen3 */
#endif /* USBPD_REV30_SUPPORT */

/* Used for legacy */
#define USB3P1_GEN1             USB3P2_GEN1 /*!< USB3.1 Gen1 and USB2.0 */
#define USB3P1_GEN1N2           USB3P2_USB4_GEN2 /*!< USB3.1 Gen1, Gen2 and USB2.0*/

typedef uint32_t USBPD_UsbSsSupport;
/**
  * @}
  */

/**
  * @brief  Power needed by adapter for full functionality in AMA VDO header
  * @{
  */
typedef enum
{
  VCONN_1W    = 0U, /*!< VCONN  power 1W   */
  VCONN_1P5W  = 1U, /*!< VCONN  power 1.5W */
  VCONN_2W    = 2U, /*!< VCONN  power 2W   */
  VCONN_3W    = 3U, /*!< VCONN  power 3W   */
  VCONN_4W    = 4U, /*!< VCONN  power 4W   */
  VCONN_5W    = 5U, /*!< VCONN  power 5W   */
  VCONN_6W    = 6U, /*!< VCONN  power 6W   */
} USBPD_VConnPower;

/**
  * @}
  */

/**
  * @brief  VCONN being required by an adapter in AMA VDO header
  * @{
  */
typedef enum
{
  VCONN_NOT_REQUIRED =  0U, /*!< VCONN not required  */
  VCONN_REQUIRED     =  1U, /*!< VCONN required      */
} USBPD_VConnRequirement;

/**
  * @}
  */

/**
  * @brief  VBUS being required by an adapter in AMA VDO header
  * @{
  */
typedef enum
{
  VBUS_NOT_REQUIRED = 0U, /*!< VBUS not required */
  VBUS_REQUIRED     = 1U, /*!< VBUS required     */
} USBPD_VBusRequirement;

/**
  * @}
  */

#if defined(USBPD_REV30_SUPPORT)
#if defined(USBPDCORE_VPD)
/**
  * @brief  Vconn Powered USB Device VDO - Charge Through Current Support
  * @{
  */
#define VPD_CHARGE_THROUGH_NOT_SUPPORTED  0u /*!< the VPD does not support Charge Through */
#define VPD_CHARGE_THROUGH_SUPPORTED      1u /*!< the VPD supports Charge Through */

typedef uint32_t USBPD_VDO_VPD_ChargeSupport;
/**
  * @}
  */

/**
  * @brief  Vconn Powered USB Device VDO - Charge Through Support
  * @{
  */
#define VPD_CHARGE_CURRENT_3A            0u /*!< 3A capable */
#define VPD_CHARGE_CURRENT_5A            1u /*!< 5A capable */

typedef uint32_t USBPD_VDO_VPD_ChargeCurrent;
/**
  * @}
  */
#endif /* USBPDCORE_VPD */
#endif /* USBPD_REV30_SUPPORT */

#define SVDM_INITIATOR            0x00u
#define SVDM_RESPONDER_ACK        0x01u
#define SVDM_RESPONDER_NAK        0x02u
#define SVDM_RESPONDER_BUSY       0x03u
#define SVDM_CABLE_TIMEOUT        0x04u /*!< Indication that cable is PD capable but no answer to VDM Discovery identity  */
#define SVDM_CABLE_NO_PD_CAPABLE  0x05u /*!< Indication that cable is not PD capable (no goodCRC to VDM Discovery identity  */

typedef uint32_t USBPD_VDM_CommandType_Typedef;

/**
  * @brief  AMA USB Superspeed Signaling Support in AMA VDO header
  * @{
  */

#define AMA_USB2P0_ONLY       0u /*!< [USB 2.0] only, no SuperSpeed support */
#define AMA_USB3P2_GEN1       1u /*!< [USB 3.2] Gen1 and USB 2.0 */
#define AMA_USB3P2_GEN1N2     2u /*!< [USB 3.2] Gen1, Gen2 and USB 2.0 */
#define AMA_USB2P0_BILLBOARD  3u /*!< [USB 2.0] billboard only         */

/* Keep for legacy reasons */
#define AMA_USB3P1_GEN1       USB3P2_GEN1       /*!< USB3.1 Gen1 and USB2.0        */
#define AMA_USB3P1_GEN1N2     AMA_USB3P2_GEN1N2 /*!< USB3.1 Gen1, Gen2 and USB2.0  */

typedef uint32_t USBPD_AmaUsbSsSupport;

/**
  * @}
  */

/**
  * @}
  */

#if defined(USBPDCORE_FWUPD)
/**
  * @brief  USBPD Firmware Update Status Information
  * @{
  */
typedef enum
{
  USBPD_FWUPD_STATUS_OK                         = 0x00U,     /*!< Request completed successfully or delayed           */
  USBPD_FWUPD_STATUS_ERR_TARGET                 = 0x01U,     /*!< FW not targeted for this device                     */
  USBPD_FWUPD_STATUS_ERR_FILE                   = 0x02U,     /*!< Fails vendor-specific verification test             */
  USBPD_FWUPD_STATUS_ERR_WRITE                  = 0x03U,     /*!< Unable to write memory                              */
  USBPD_FWUPD_STATUS_ERR_ERASE                  = 0x04U,     /*!< Memory erase function failed                        */
  USBPD_FWUPD_STATUS_ERR_CHECK_ERASED           = 0x05U,     /*!< Memory erase check failed                           */
  USBPD_FWUPD_STATUS_ERR_PROG                   = 0x06U,     /*!< Program memory function failed                      */
  USBPD_FWUPD_STATUS_ERR_VERIFY                 = 0x07U,     /*!< Program memory failed verification                  */
  USBPD_FWUPD_STATUS_ERR_ADDRESS                = 0x08U,     /*!< Received address is out of range                    */
  USBPD_FWUPD_STATUS_ERR_NOTDONE                = 0x09U,     /*!< Received PDFU_DATA Request with a zero length Data
                                                                  Block, but the PDFU Responder expects more data     */
  USBPD_FWUPD_STATUS_ERR_FIRMWARE               = 0x0AU,     /*!< Device's firmware is corrupt.
                                                                  It cannot return to normal operations.              */
  USBPD_FWUPD_STATUS_ERR_POR                    = 0x0DU,     /*!< Unexpected power on reset                           */
  USBPD_FWUPD_STATUS_ERR_UNKNOWN                = 0x0EU,     /*!< Something went wrong                                */
  USBPD_FWUPD_STATUS_ERR_UNEXPECTED_HARD_RESET  = 0x80U,     /*!< Used when firmware update starts after a hard reset
                                                                 (enumeration, etc.) that occurred in the middle
                                                                  of firmware update                                  */
  USBPD_FWUPD_STATUS_ERR_UNEXPECTED_SOFT_RESET  = 0x81U,     /*!< Used when firmware update starts after soft reset
                                                                  (new power contract, etc.)
                                                                  that occurred in the middle of firmware update      */
  USBPD_FWUPD_STATUS_ERR_UNEXPECTED_REQUEST     = 0x82U,     /*!< PDFU Responder received a request that is not
                                                                  appropriate for the current Phase                   */
  USBPD_FWUPD_STATUS_ERR_REJECT_PAUSE           = 0x83U,     /*!< PDFU Responder is unable or unwilling to pause
                                                                  a firmware image transfer                           */
} USBPD_FWUPD_Status_TypeDef;
/**
  * @}
  */
#endif /* USBPDCORE_FWUPD */

/** @defgroup USBPD_CORE_DataInfoType_TypeDef USB CORE Data information type
  * @brief Data Info types used in PE callbacks (USBPD_PE_GetDataInfo and USBPD_PE_SetDataInfo)
  * @{
  */
typedef enum
{
  USBPD_CORE_DATATYPE_SRC_PDO          = 0x00U,      /*!< Handling of port Source PDO (SRC or DRP configuration used only in USBPD_PE_GetDataInfo) */
  USBPD_CORE_DATATYPE_SNK_PDO          = 0x01U,      /*!< Handling of port Sink PDO, requested by get sink capa (SNK or DRP configuration used only in USBPD_PE_GetDataInfo) */
  USBPD_CORE_DATATYPE_RDO_POSITION     = 0x02U,      /*!< Reset the PDO position selected by the sink only (used only in USBPD_PE_SetDataInfo)                          */
  USBPD_CORE_DATATYPE_REQ_VOLTAGE      = 0x03U,      /*!< Get voltage value requested for BIST tests, expect 5V (used only in USBPD_PE_GetDataInfo)                          */
  USBPD_CORE_DATATYPE_RCV_SRC_PDO      = 0x04U,      /*!< Storage of Received Source PDO values (used only in USBPD_PE_SetDataInfo)                          */
  USBPD_CORE_DATATYPE_RCV_SNK_PDO      = 0x05U,      /*!< Storage of Received Sink PDO values (used only in USBPD_PE_SetDataInfo)                          */
  USBPD_CORE_DATATYPE_RCV_REQ_PDO      = 0x06U,      /*!< Storage of Received Sink Request PDO value (SRC or DRP configuration used in USBPD_PE_SetDataInfo)      */
  USBPD_CORE_DATATYPE_REQUEST_DO       = 0x07U,      /*!< Not used - keep for legacy reason                           */
  USBPD_CORE_EXTENDED_CAPA             = 0x08U,      /*!< Source Extended capability message content (used in USBPD_PE_GetDataInfo and USBPD_PE_SetDataInfo) */
  USBPD_CORE_INFO_STATUS               = 0x09U,      /*!< Information status message content (used in USBPD_PE_GetDataInfo and USBPD_PE_SetDataInfo) */
  USBPD_CORE_PPS_STATUS                = 0x0AU,      /*!< PPS Status message content (used in USBPD_PE_GetDataInfo and USBPD_PE_SetDataInfo) */
  USBPD_CORE_ALERT,                       /*!< Storing of received Alert message content (used only in USBPD_PE_SetDataInfo)                          */
  USBPD_CORE_GET_MANUFACTURER_INFO,       /*!< Storing of received Get Manufacturer info message content (used only in USBPD_PE_SetDataInfo)                          */
  USBPD_CORE_MANUFACTURER_INFO,           /*!< Retrieve of Manufacturer info message content (used in USBPD_PE_GetDataInfo and USBPD_PE_SetDataInfo)                          */
  USBPD_CORE_GET_BATTERY_STATUS,          /*!< Storing of received Get Battery status message content (used only in USBPD_PE_SetDataInfo)                          */
  USBPD_CORE_BATTERY_STATUS,              /*!< Retrieve of Battery status message content (used in USBPD_PE_GetDataInfo and USBPD_PE_SetDataInfo)                          */
  USBPD_CORE_GET_BATTERY_CAPABILITY,      /*!< Storing of received Get Battery capability message content (used only in USBPD_PE_SetDataInfo)                          */
  USBPD_CORE_BATTERY_CAPABILITY,          /*!< Retrieve of Battery capability message content (used in USBPD_PE_GetDataInfo and USBPD_PE_SetDataInfo)                          */
  USBPD_CORE_UNSTRUCTURED_VDM,            /*!< Not used - keep for legacy reason                                 */
#if defined(USBPDCORE_SNK_CAPA_EXT)
  USBPD_CORE_SNK_EXTENDED_CAPA,           /*!< Storing and retrieve of Sink Extended capability message content (used in USBPD_PE_GetDataInfo and USBPD_PE_SetDataInfo) */
#endif /* USBPDCORE_SNK_CAPA_EXT */
#if defined(USBPDCORE_PECABLE)
  USBPD_CORE_CABLE_GETIDENTITY,       /*!< get the cable identity information (used in USBPD_PE_GetDataInfo) */
  USBPD_CORE_CABLE_GETSTATUS,         /*!< get the cable status information (used in USBPD_PE_GetDataInfo)   */
#endif /* USBPDCORE_PECABLE */
#if defined(USBPDCORE_USBDATA)
  USBPD_CORE_DATATYPE_ENTERUSB,       /*!< get info to send an ENTER_USB message */
#endif /* USBPDCORE_USBDATA */
  USBPD_CORE_REVISION,                 /*!< get/set revision info */
} USBPD_CORE_DataInfoType_TypeDef;
/**
  * @}
  */
#if defined(USBPDCORE_USBDATA)
/** @defgroup USBPD_CORE_ActionType_TypeDef USB CORE Action type
  * @brief Data Info types used in PE callbacks (USBPD_PE_GetDataInfo and USBPD_PE_SetDataInfo)
  * @{
  */
typedef enum
{
  USBPD_ACTION_REPLY_ENTER_USB          = 0x01U,      /*!< Get DPM reply to a ENTER_USB message  */
  USBPD_ACTION_REPLY_DATA_RESET         = 0x02U,      /*!< Get DPM reply to a DATA_RESET message */
} USBPD_CORE_ActionType_TypeDef;
#endif /* USBPDCORE_USBDATA */
/**
  * @}
  */

/**
  * @}
  */

/* Exported typedef ----------------------------------------------------------*/

/** @defgroup USBPD_CORE_DEF_Exported_TypeDef USBPD CORE DEF Exported TypeDef
  * @{
  */

#if defined(USBPD_REV30_SUPPORT) && defined(USBPDCORE_PPS)
/**
  * @brief  USB PD Programmable Power Supply APDO Structure definition (same for SRC and SNK)
  *
  */
typedef struct
{
  uint32_t MaxCurrentIn50mAunits  : 7u;  /*!< Maximum Current in 50mA increments       */
  uint32_t Reserved1              : 1u;  /*!< Reserved  - Shall be set to zero         */
  uint32_t MinVoltageIn100mV      : 8u;  /*!< Minimum Voltage in 100mV increments      */
  uint32_t Reserved2              : 1u;  /*!< Reserved  - Shall be set to zero         */
  uint32_t MaxVoltageIn100mV      : 8u;  /*!< Maximum Voltage in 100mV increments      */
  uint32_t Reserved3              : 2u;  /*!< Reserved  - Shall be set to zero         */
  uint32_t PPSPowerLimited        : 1u;  /*!< PPS Power Limited Bit                    */
  uint32_t ProgrammablePowerSupply: 2u;  /*!< 00b - Programmable Power Supply          */
  uint32_t PPS_APDO               : 2u;  /*!< 11b - Augmented Power Data Object (APDO) */
} USBPD_ProgrammablePowerSupplyAPDO_TypeDef;

#endif /*_USBPD_REV30_SUPPORT && PPS*/

/**
  * @brief  USB PD Source Fixed Supply Power Data Object Structure definition
  *
  */
typedef struct
{
  uint32_t MaxCurrentIn10mAunits :                                  10u;
  uint32_t VoltageIn50mVunits :                                     10u;
  USBPD_CORE_PDO_PeakCurr_TypeDef PeakCurrent :                     2u;
#if defined(USBPD_REV30_SUPPORT)
  uint32_t Reserved22_23 :                                          2u;
  uint32_t UnchunkedExtendedMessage :                               1u;
#else
  uint32_t Reserved22_24 :                                          3u;
#endif /* USBPD_REV30_SUPPORT */
  USBPD_CORE_PDO_DRDataSupport_TypeDef DataRoleSwap :               1u;
  USBPD_CORE_PDO_USBCommCapable_TypeDef USBCommunicationsCapable :  1u;
  USBPD_CORE_PDO_ExtPowered_TypeDef ExternallyPowered :             1u;
  USBPD_CORE_PDO_USBSuspendSupport_TypeDef USBSuspendSupported :    1u;
  USBPD_CORE_PDO_DRPowerSupport_TypeDef DualRolePower :             1u;
  USBPD_CORE_PDO_Type_TypeDef FixedSupply :                         2u;

} USBPD_SRCFixedSupplyPDO_TypeDef;

/**
  * @brief  USB PD Source Variable Supply Power Data Object Structure definition
  *
  */
typedef struct
{
uint32_t MaxCurrentIn10mAunits :
  10u;
uint32_t MinVoltageIn50mVunits :
  10u;
uint32_t MaxVoltageIn50mVunits :
  10u;
uint32_t VariableSupply :
  2u;
} USBPD_SRCVariableSupplyPDO_TypeDef;

/**
  * @brief  USB PD Source Battery Supply Power Data Object Structure definition
  *
  */
typedef struct
{
uint32_t MaxAllowablePowerIn250mWunits :
  10u;
uint32_t MinVoltageIn50mVunits :
  10u;
uint32_t MaxVoltageIn50mVunits :
  10u;
uint32_t Battery :
  2u;
} USBPD_SRCBatterySupplyPDO_TypeDef;

/**
  * @brief  USB PD Sink Fixed Supply Power Data Object Structure definition
  *
  */
typedef struct
{
uint32_t OperationalCurrentIn10mAunits :
  10u;
uint32_t VoltageIn50mVunits :
  10u;
#if defined(USBPD_REV30_SUPPORT)
uint32_t FastRoleSwapRequiredCurrent :
  2u;
uint32_t Reserved20_22 :
  3u;
#else
uint32_t Reserved20_24 :
  5u;
#endif /* USBPD_REV30_SUPPORT */
uint32_t DataRoleSwap :
  1u;
uint32_t USBCommunicationsCapable :
  1;
uint32_t ExternallyPowered :
  1u;
uint32_t HigherCapability :
  1u;
uint32_t DualRolePower :
  1u;
uint32_t FixedSupply :
  2u;
} USBPD_SNKFixedSupplyPDO_TypeDef;

/**
  * @brief  USB PD Sink Variable Supply Power Data Object Structure definition
  *
  */
typedef struct
{
uint32_t OperationalCurrentIn10mAunits :
  10u;
uint32_t MinVoltageIn50mVunits :
  10u;
uint32_t MaxVoltageIn50mVunits :
  10u;
uint32_t VariableSupply :
  2u;
} USBPD_SNKVariableSupplyPDO_TypeDef;

/**
  * @brief  USB PD Sink Battery Supply Power Data Object Structure definition
  *
  */
typedef struct
{
uint32_t OperationalPowerIn250mWunits :
  10u;
uint32_t MinVoltageIn50mVunits :
  10u;
uint32_t MaxVoltageIn50mVunits :
  10u;
uint32_t Battery :
  2u;
} USBPD_SNKBatterySupplyPDO_TypeDef;

/**
  * @brief  USB PD Sink Generic Power Data Object Structure definition
  *
  */
typedef struct
{
  uint32_t Bits_0_10                      : 10u; /*!< Specific Power Capabilities are described by the (A)PDOs in the following sections. */
  uint32_t VoltageIn50mVunits             : 10u; /*!< Maximum Voltage in 50mV units valid for all PDO (not APDO) */
  uint32_t Bits_20_29                     : 10u; /*!< Specific Power Capabilities are described by the (A)PDOs in the following sections. */
  USBPD_CORE_PDO_Type_TypeDef PowerObject : 2u;  /*!< (A) Power Data Object  */
} USBPD_GenericPDO_TypeDef;

/**
  * @brief  USB PD Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;

  USBPD_GenericPDO_TypeDef            GenericPDO;       /*!< Generic Power Data Object Structure            */

  USBPD_SRCFixedSupplyPDO_TypeDef     SRCFixedPDO;      /*!< Fixed Supply PDO - Source                      */

  USBPD_SRCVariableSupplyPDO_TypeDef  SRCVariablePDO;   /*!< Variable Supply (non-Battery) PDO - Source     */

  USBPD_SRCBatterySupplyPDO_TypeDef   SRCBatteryPDO;    /*!< Battery Supply PDO - Source                    */

  USBPD_SNKFixedSupplyPDO_TypeDef     SNKFixedPDO;      /*!< Fixed Supply PDO - Sink                        */

  USBPD_SNKVariableSupplyPDO_TypeDef  SNKVariablePDO;   /*!< Variable Supply (non-Battery) PDO - Sink       */

  USBPD_SNKBatterySupplyPDO_TypeDef   SNKBatteryPDO;    /*!< Battery Supply PDO - Sink                      */

#if defined(USBPD_REV30_SUPPORT)
#ifdef USBPDCORE_PPS
  USBPD_ProgrammablePowerSupplyAPDO_TypeDef  SRCSNKAPDO;/*!< Programmable Power Supply APDO - Source / Sink */
#endif /* USBPDCORE_PPS */
#endif /* USBPD_REV30_SUPPORT */

} USBPD_PDO_TypeDef;

/**
  * @brief  USB PD Sink Fixed and Variable Request Data Object Structure definition
  *
  */
typedef struct
{
uint32_t MaxOperatingCurrent10mAunits : /*!< Corresponding to min if GiveBackFlag = 1 */
  10u;
uint32_t OperatingCurrentIn10mAunits :
  10u;
#if defined(USBPD_REV30_SUPPORT)
uint32_t Reserved20_22 :
  3u;
uint32_t UnchunkedExtendedMessage :
  1u;
#else
uint32_t Reserved20_23 :
  4u;
#endif /* USBPD_REV30_SUPPORT */
uint32_t NoUSBSuspend :
  1u;
uint32_t USBCommunicationsCapable :
  1u;
uint32_t CapabilityMismatch :
  1u;
uint32_t GiveBackFlag :
  1u;
uint32_t ObjectPosition :
  3u;
uint32_t Reserved31 :
  1u;
} USBPD_SNKFixedVariableRDO_TypeDef;

/**
  * @brief  USB PD Sink Battery Request Data Object Structure definition
  *
  */
typedef struct
{
uint32_t MaxOperatingPowerIn250mWunits :
  10u;
uint32_t OperatingPowerIn250mWunits :
  10u;
#if defined(USBPD_REV30_SUPPORT)
uint32_t Reserved20_22 :
  3u;
uint32_t UnchunkedExtendedMessage :
  1u;                                      /*!< Unchunked Extended Messages Supported                    */
#else
uint32_t Reserved20_23 :
  4u;
#endif /* USBPD_REV30_SUPPORT */
uint32_t NoUSBSuspend :
  1u;
uint32_t USBCommunicationsCapable :
  1u;
uint32_t CapabilityMismatch :
  1u;
uint32_t GiveBackFlag :
  1u;
uint32_t ObjectPosition :
  3u;
uint32_t Reserved31 :
  1u;
} USBPD_SNKBatteryRDO_TypeDef;

#if defined(USBPD_REV30_SUPPORT)
/**
  * @brief  USB PD Sink Programmable Request Data Object Structure definition
  *
  */
typedef struct
{
  uint32_t OperatingCurrentIn50mAunits  : 7u;  /*!< Operating Current 50mA units                             */
  uint32_t Reserved1                    : 2u;  /*!< Reserved  - Shall be set to zero                         */
  uint32_t OutputVoltageIn20mV          : 11;  /*!< Output Voltage in 20mV units                             */
  uint32_t Reserved2                    : 3u;  /*!< Reserved  - Shall be set to zero                         */
  uint32_t UnchunkedExtendedMessage     : 1u;  /*!< Unchunked Extended Messages Supported                    */
  uint32_t NoUSBSuspend                 : 1u;  /*!< No USB Suspend                                           */
  uint32_t USBCommunicationsCapable     : 1u;  /*!< USB Communications Capable                               */
  uint32_t CapabilityMismatch           : 1u;  /*!< Capability Mismatch                                      */
  uint32_t Reserved3                    : 1u;  /*!< Reserved  - Shall be set to zero                         */
  uint32_t ObjectPosition               : 3u;  /*!< Object position (000b is Reserved and Shall Not be used) */
  uint32_t Reserved4                    : 1u;  /*!< USB Communications Capable                               */
} USBPD_SNKProgrammableRDO_TypeDef;
#endif /* USBPD_REV30_SUPPORT */


/**
  * @brief  USB PD Sink Generic Request Data Object Structure definition
  *
  */
typedef struct
{
#if defined(USBPD_REV30_SUPPORT)
  uint32_t Bits_0_22                    : 23u; /*!< Bits 0 to 22 of RDO                                      */
  uint32_t UnchunkedExtendedMessage     : 1u;  /*!< Unchunked Extended Messages Supported                    */
#else
  uint32_t Bits_0_23                    : 24u;  /*!< Bits 0 to 23 of RDO                                     */
#endif /* USBPD_REV30_SUPPORT */
  uint32_t NoUSBSuspend                 : 1u;  /*!< No USB Suspend                                           */
  uint32_t USBCommunicationsCapable     : 1u;  /*!< USB Communications Capable                               */
  uint32_t CapabilityMismatch           : 1u;  /*!< Capability Mismatch                                      */
  uint32_t Bit_27                       : 1u;  /*!< Reserved  - Shall be set to zero                         */
  uint32_t ObjectPosition               : 3u;  /*!< Object position (000b is Reserved and Shall Not be used) */
  uint32_t Bit_31                       : 1u;  /*!< USB Communications Capable                               */
} USBPD_SNKGenericRDO_TypeDef;

/**
  * @brief  USB PD Sink Request Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;

  USBPD_SNKGenericRDO_TypeDef       GenericRDO;       /*!<  Generic Request Data Object Structure           */

  USBPD_SNKFixedVariableRDO_TypeDef FixedVariableRDO; /*!< Fixed and Variable Request Data Object Structure */

  USBPD_SNKBatteryRDO_TypeDef       BatteryRDO;       /*!< Battery Request Data Object Structure            */

#if defined(USBPD_REV30_SUPPORT)
  USBPD_SNKProgrammableRDO_TypeDef  ProgRDO;          /*!< Programmable Request Data Object Structure       */
#endif /* USBPD_REV30_SUPPORT */

} USBPD_SNKRDO_TypeDef;

#if defined(USBPD_REV30_SUPPORT) && defined(USBPDCORE_PPS)
/**
  * @brief  USBPD Port APDO Structure definition
  *
  */
typedef struct
{
  uint32_t *ListOfAPDO;                          /*!< Pointer on Augmented Power Data Objects list, defining
                                                     port capabilities */
  uint8_t  NumberOfAPDO;                         /*!< Number of Augmented Power Data Objects defined in ListOfAPDO
                                                     This parameter must be set at max to @ref USBPD_MAX_NB_PDO value */
} USBPD_PortAPDO_TypeDef;
#endif /* USBPD_REV30_SUPPORT && USBPDCORE_PPS */

/**
  * @brief  USB PD BIST Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
uint32_t BistErrorCounter :
    16u;
uint32_t Reserved16_27 :
    12u;
uint32_t BistMode :
    4u;
  }
  b;
} USBPD_BISTDataObject_TypeDef;

/** @brief  Sink requested power profile Structure definition
  *
  */
typedef struct
{
  uint32_t MaxOperatingCurrentInmAunits;           /*!< Sink board Max operating current in mA units   */
  uint32_t OperatingVoltageInmVunits;              /*!< Sink board operating voltage in mV units       */
  uint32_t MaxOperatingVoltageInmVunits;           /*!< Sink board Max operating voltage in mV units   */
  uint32_t MinOperatingVoltageInmVunits;           /*!< Sink board Min operating voltage in mV units   */
  uint32_t OperatingPowerInmWunits;                /*!< Sink board operating power in mW units         */
  uint32_t MaxOperatingPowerInmWunits;             /*!< Sink board Max operating power in mW units     */
} USBPD_SNKPowerRequest_TypeDef;


/** @defgroup USBPD_CORE_VDM_Exported_Structures USBPD CORE VDM Exported Structures
  * @{
  */

/** @defgroup USBPD_ProductVdo_TypeDef USB PD VDM Product VDO
  * @brief USB PD Product VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
uint32_t bcdDevice :      /*!< Device version             */
    16u;
uint32_t USBProductId :   /*!< USB Product ID             */
    16u;
  }
  b;
} USBPD_ProductVdo_TypeDef;

/**
  * @}
  */

/** @defgroup USBPD_IDHeaderVDOStructure_definition USB SVDM ID header VDO Structure definition
  * @brief USB SVDM ID header VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t VID                                  : 16u;  /*!< SVDM Header's SVDM Version                                                     */
    uint32_t Reserved                             : 10u;  /*!< Reserved                                                                       */
    USBPD_ModalOp_TypeDef ModalOperation          : 1u;   /*!< Modal Operation Supported based on @ref USBPD_ModalOp_TypeDef                  */
    USBPD_ProductType_TypeDef ProductTypeUFPorCP  : 3u;   /*!< Product Type (UFP or Cable Plug)based on @ref USBPD_ProductType_TypeDef        */
    USBPD_USBCapa_TypeDef USBDevCapability        : 1u;   /*!< USB Communications Capable as a USB Device based on @ref USBPD_USBCapa_TypeDef */
    USBPD_USBCapa_TypeDef USBHostCapability       : 1u;   /*!< USB Communications Capable as USB Host based on @ref USBPD_USBCapa_TypeDef     */
  } b20;
#if defined(USBPD_REV30_SUPPORT)
  struct
  {
    uint32_t VID                                  : 16u;  /*!< SVDM Header's SVDM Version                                                     */
    uint32_t Reserved                             : 5u;   /*!< Reserved                                                                       */
    uint32_t ConnectorType                        : 2u;   /*!< Connector Type based on @ref USBPD_ConnectorType_TypeDef                       */
    uint32_t ProductTypeDFP                       : 3u;   /*!< Product Type (DFP) based on @ref USBPD_ProductType_TypeDef                     */
    USBPD_ModalOp_TypeDef ModalOperation          : 1u;   /*!< Modal Operation Supported based on @ref USBPD_ModalOp_TypeDef                  */
    USBPD_ProductType_TypeDef ProductTypeUFPorCP  : 3u;   /*!< Product Type (UFP or Cable Plug)based on @ref USBPD_ProductType_TypeDef        */
    USBPD_USBCapa_TypeDef USBDevCapability        : 1u;   /*!< USB Communications Capable as a USB Device based on @ref USBPD_USBCapa_TypeDef */
    USBPD_USBCapa_TypeDef USBHostCapability       : 1u;   /*!< USB Communications Capable as USB Host based on @ref USBPD_USBCapa_TypeDef     */
  } b30;
#endif /* USBPD_REV30_SUPPORT */
} USBPD_IDHeaderVDO_TypeDef;
/**
  * @}
  */

typedef union
{
  struct /* PD 2.0*/
  {
    USBPD_SsDirectionality    SSRX2_DirSupport    : 1u;  /*!< SSRX2 Directionality Support (PD2.0)     */
    USBPD_SsDirectionality    SSRX1_DirSupport    : 1u;  /*!< SSRX1 Directionality Support (PD2.0)     */
    USBPD_SsDirectionality    SSTX2_DirSupport    : 1u;  /*!< SSTX2 Directionality Support (PD2.0)     */
    USBPD_SsDirectionality    SSTX1_DirSupport    : 1u;  /*!< SSTX1 Directionality Support (PD2.0)     */
  }
  pd_v20;
#if defined(USBPD_REV30_SUPPORT)
  struct /* PD 3.0*/
  {
    uint8_t                   Reserved            : 2u;  /*!< Reserved                                 */
    USBPD_VBUSMaxVoltage      MaxVBUS_Voltage     : 2u;  /*!< Maximum Cable VBUS Voltage               */
  }
  pd_v30;
#endif /* USBPD_REV30_SUPPORT */
} USBPD_CableVdo_Field1TypeDef;

/** @defgroup USBPD_AttentionInfo_TypeDef USB PD Attention Info object Structure definition
  * @brief USB PD Attention Info object Structure definition
  * @{
  */
typedef struct
{
  uint32_t  VDO;
  uint16_t  SVID;
  USBPD_VDM_Command_Typedef Command;
  uint8_t   ModeIndex;
} USBPD_AttentionInfo_TypeDef;

/**
  * @}
  */

typedef union
{
  struct /* PD 2.0*/
  {
    uint8_t                   Reserved            : 4u;  /*!< Reserved                               */
  }
  pd_v20;
#if defined(USBPD_REV30_SUPPORT)
  struct /* PD 3.0*/
  {
    uint8_t                   VDOVersion          : 3u;  /*!< Version Number of the VDO              */
    uint8_t                   Reserved            : 1u;  /*!< Reserved                               */
  }
  pd_v30;
#endif /* USBPD_REV30_SUPPORT */
} USBPD_CableVdo_Field2TypeDef;

#define VDM_UNSTRUCTUREDVDM_TYPE        0x0u
#define VDM_STRUCTUREDVDM_TYPE          0x1u

typedef uint32_t USBPD_VDM_VDMType_Typedef;

/** @defgroup USBPD_SVDMHeaderStructure_definition USB SVDM Message header Structure definition
  * @brief USB SVDM Message header Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
USBPD_VDM_Command_Typedef Command :         /*!< SVDM Header's Command          */
    5u;
uint32_t Reserved5 :                        /*!< Reserved                       */
    1u;
USBPD_VDM_CommandType_Typedef CommandType : /*!< SVDM Header's Command Type     */
    2u;
uint32_t ObjectPosition :                   /*!< SVDM Header's Object Position  */
    3u;
uint32_t Reserved11 :                       /*!< Reserved                       */
    2u;
uint32_t SVDMVersion :                      /*!< SVDM Header's SVDM Version     */
    2u;
USBPD_VDM_VDMType_Typedef VDMType :         /*!< SVDM Header's VDM Type         */
    1u;
uint32_t SVID :                             /*!< SVDM Header's SVID             */
    16u;
  }
  b;
} USBPD_SVDMHeader_TypeDef;

/**
  * @}
  */

#ifdef USBPDCORE_UVDM
/** @defgroup USBPD_UVDMHeaderStructure_definition USB UVDM Message header Structure definition
  * @brief USB UVDM Message header Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t VendorUse                : 15u;  /*!< Content of this field is defined by the vendor.  */
    USBPD_VDM_VDMType_Typedef VDMType : 1u;   /*!< VDM Type                                         */
    uint32_t VID                      : 16u;  /*!< Vendor ID (VID)                                  */
  }
  b;
} USBPD_UVDMHeader_TypeDef;

/**
  * @}
  */
#endif /* USBPDCORE_UVDM */

/** @defgroup USBPD_CableVdo_TypeDef USB PD VDM Passive Cable VDO
  * @brief USB PD Passive Cable VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
    USBPD_UsbSsSupport        USB_SS_Support      : 3u;  /*!< USB SuperSpeed Signaling Support           */
    uint32_t                  reserved            : 2u;
    USBPD_VBUSCurrentHandCap  VBUS_CurrentHandCap : 2u;  /*!< VBUS Current Handling Capability           */
    uint32_t                  Fields1             : 2u;  /*!< Based on @ref USBPD_CableVdo_Field1TypeDef */
    USBPD_CableMaxVoltage     CableMaxVoltage     : 2u;  /*!< Cable maximum voltage                      */
    USBPD_CableTermType       CableTermType       : 2u;  /*!< Cable Termination Type                     */
    uint32_t                  CableLatency        : 4u;  /*!< Cable Latency                              */
    uint32_t                  Reserved            : 1u;  /*!< Reserved                                   */
    USBPD_CableToType         CableToType         : 2u;  /*!< USB Type-C plug to USB Type-A/B/C/Captive (PD 2.0)
                                                             USB Type-C plug to USB Type-C/Captive (PD 3.0) */
    uint32_t                  Fields2             : 1u;  /*!< Based on @ref USBPD_CableVdo_Field2TypeDef */
    USBPD_VDM_VDO_PassiveCable_Version_TypeDef VDO_Version : 3u;  /*!< Version number  of the VDO                 */
    uint32_t                  CableFWVersion      : 4u;  /*!< Cable FW version number (vendor defined)   */
    uint32_t                  CableHWVersion      : 4u;  /*!< Cable HW version number (vendor defined)   */
  }
  b;
} USBPD_CableVdo_TypeDef;

#if defined(USBPD_REV30_SUPPORT)
/** @defgroup USBPD_ActiveCableVdo1_TypeDef USB PD VDM Active Cable VDO
  * @brief USB PD Active Cable VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
    USBPD_UsbSsSupport        USB_HighestSpeed    : 3u;  /*!< USB Highest Speed Support           */
    uint32_t SOPSecondControllerPresent           : 1u;  /*!< SOP'' controller present Support          */
    uint32_t                  VBUS_ThroughCable   : 1u;  /*!< VBUS Through Cable Support          */
    USBPD_VBUSCurrentHandCap  VBUS_CurrentHandCap : 2u;  /*!< VBUS Current Handling Capability           */
    USBPD_ActiveCableSBUType SBUType              : 1u;  /*!< SBU Type */
    USBPD_ActiveCableSBUSupported SBUSupported    : 1u;  /*!< SBUs connections supported */
    USBPD_CableMaxVoltage     CableMaxVoltage     : 2u;  /*!< Cable maximum voltage                      */
    USBPD_CableTermType       CableTermType       : 2u;  /*!< Cable Termination Type (@ref CABLE_TERM_ONE_EACH_VCONN or @ref CABLE_TERM_BOTH_ACTIVE_VCONN */
    uint32_t                  CableLatency        : 4u;  /*!< Cable Latency                              */
    uint32_t                                      : 1u;  /*!< B17 Reserved bit                                  */
    USBPD_CableToType         ConnectorType       : 2u;  /*!< Connector Type (@ref CABLE_TO_TYPE_C or @ref CABLE_CAPTIVE) */
    uint32_t                                      : 1u;  /*!< B20 Reserved bit                                  */
    USBPD_VDM_VDO_ActiveCable_Version_TypeDef VDO_Version : 3u;  /*!< Version number  of the Active Cable VDO                 */
    uint32_t                  CableFWVersion      : 4u;  /*!< Cable FW version number (vendor defined)   */
    uint32_t                  CableHWVersion      : 4u;  /*!< Cable HW version number (vendor defined)   */
  }
  b;
} USBPD_ActiveCableVdo1_TypeDef;

/**
  * @}
  */
#endif /* USBPD_REV30_SUPPORT */

/** @defgroup USBPD_CertStatVdo_TypeDef USB PD VDM Cert stat VDO
  * @brief USB PD Cert stat VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
uint32_t XID :          /*!< USB-IF assigned XID */
    32;
  }
  b;
} USBPD_CertStatVdo_TypeDef;

/**
  * @}
  */

/** @defgroup USBPD_AMAVdo_TypeDef USB PD VDM Alternate Mode Adapter VDO (NO MORE USED IN PD3.0)
  * @brief USB PD Alternate Mode Adapter VDO Structure definition
  * @{
  */
/* #### Keep for PD2.0 legacy reasons but should NOT more used. #### */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t                AMA_USB_SS_Support  : 3u;  /*!< AMA USB SuperSpeed Signaling Support based on
                                                           @ref USBPD_AmaUsbSsSupport               */
    uint32_t                VBUSRequirement     : 1u;  /*!< VBUS  required  based on
                                                           @ref USBPD_VBusRequirement               */
    uint32_t                VCONNRequirement    : 1u;  /*!< VCONN  required  based on
                                                           @ref USBPD_VConnRequirement              */
    uint32_t                VCONNPower          : 3u;  /*!< VCONN  power  based on
                                                           @ref USBPD_VConnPower                    */
#if defined(USBPD_REV30_SUPPORT)
    uint32_t                Reserved            : 13; /*!< Reserved                                 */
    uint32_t                VDO_Version         : 3u;  /*!< Version Number of the VDO                */
#else
    uint32_t                SSRX2_DirSupport    : 1u;  /*!< SSRX2 Directionality Support (PD2.0) based on
                                                           @ref USBPD_SsDirectionality              */
    uint32_t                SSRX1_DirSupport    : 1u;  /*!< SSRX1 Directionality Support (PD2.0) based on
                                                           @ref USBPD_SsDirectionality              */
    uint32_t                SSTX2_DirSupport    : 1u;  /*!< SSTX2 Directionality Support (PD2.0) based on
                                                           @ref USBPD_SsDirectionality              */
    uint32_t                SSTX1_DirSupport    : 1u;  /*!< SSTX1 Directionality Support (PD2.0) based on
                                                           @ref USBPD_SsDirectionality              */
    uint32_t                Reserved            : 12u; /*!< Reserved                                 */
#endif /* USBPD_REV30_SUPPORT */
    uint32_t                AMAFWVersion        : 4u;  /*!< AMA FW version number (vendor defined)   */
    uint32_t                AMAHWVersion        : 4u;  /*!< AMA HW version number (vendor defined)   */
  }
  b;
} USBPD_AMAVdo_TypeDef;
/* #### Keep for PD2.0 legacy reasons but should NOT more used. #### */
/**
  * @}
  */

#if defined(USBPD_REV30_SUPPORT)
/** @defgroup USBPD_UFPVdo_TypeDef USB PD VDM UFP VDO
  * @brief USB PD UFP VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
    USBPD_UsbSsSupport USB_HighestSpeed             : 3u;   /*!< USB Highest Speed Support                                 */
    USBPD_VDO_UFP_AlternateModes AlternateModes     : 3u;   /*!< Alternate Modes based                                     */
    uint32_t                                        : 16u;  /*!< B21_6 Reserved bit                                        */
    USBPD_ConnectorType_TypeDef ConnectorType       : 2u;   /*!< Connector Type                                            */
    USBPD_VDO_UFP_DeviceCapability DeviceCapability : 4u;   /*!< Device Capability                                         */
    uint32_t                                        : 1u;   /*!< B28 Reserved bit                                          */
    USBPD_VDM_VDO_UFP_Version_TypeDef UFPVDOVersion : 3u;   /*!< Version Number of the VDO (should be set to Version1.1)   */
  }
  b;
} USBPD_UFPVdo_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_DFPVdo_TypeDef USB PD VDM DFP VDO
  * @brief USB PD DFP VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t                      PortNumber        : 5u;   /*!< Unique port number to identify a specific port on a multi-port device  */
    uint32_t                                        : 17u;  /*!< B21-5 Reserved bits                                                    */
    USBPD_ConnectorType_TypeDef   ConnectorType     : 2u;   /*!< Connector Type                                                         */
    USBPD_VDO_DFP_HostCapability  HostCapability    : 3u;   /*!< Host Capability                                                        */
    uint32_t                                        : 2u;   /*!< B28-27 Reserved bits                                                   */
    USBPD_VDM_VDO_DFP_Version_TypeDef DFPVDOVersion : 3u;   /*!< Version Number of the VDO (should be set to Version1.1)                */
  }
  b;
} USBPD_DFPVdo_TypeDef;
/**
  * @}
  */

#if defined(USBPDCORE_VPD)
/** @defgroup USBPD_VPDVdo_TypeDef USB PD VDM Vconn Powered USB Device VDO
  * @brief USB PD Vconn Powered USB Device VDO Structure definition
  * @{
  */
typedef union
{
  uint32_t d32;
  struct
  {
    USBPD_VDO_VPD_ChargeSupport ChargeThroughSupport : 1u;  /*!< Charge Through Support           */
    uint32_t   GroundImpedance     : 6u;  /*!< Charge Through Support bit = 1b: Ground impedance through the VPD in 1 mOhm increments.
                                               Values less than 10 mOhm are Reserved and Shall Not be used.
                                               Charge Through Support bit = 0b: Reserved, Shall be set to zero */
    uint32_t   VBUS_Impedance      : 6u;  /*!< Charge Through Support bit = 1b: Vbus impedance through the VPD in 2 mOhm increments.
                                               Values less than 10 mOhm are Reserved and Shall Not be used.
                                                 Charge Through Support bit = 0b: Reserved, Shall be set to zero   */
    uint32_t                       : 2u;  /*!< B14-13 Reserved bit                           */
    USBPD_VDO_VPD_ChargeCurrent  ChargeThroughCurrent : 1u;  /*!< Charge Through Current Support            */
    USBPD_CableMaxVoltage     CableMaxVoltage         : 2u;  /*!< Cable maximum voltage                         */
    uint32_t                                          : 4u;  /*!< B20-17 Reserved bit                           */
    USBPD_VDM_VDO_VPD_Version_TypeDef VDO_Version     : 3u;  /*!< Version number  of the VPD VDO                */
    uint32_t                  FWVersion               : 4u;  /*!< Cable FW version number (vendor defined)           */
    uint32_t                  HWVersion               : 4u;  /*!< Cable HW version number (vendor defined)           */
  }
  b;
} USBPD_VPDVdo_TypeDef;
/**
  * @}
  */
#endif /* USBPDCORE_VPD */
#endif /* USBPD_REV30_SUPPORT */

/** @defgroup USBPD_DiscoveryIdentity_TypeDef USB PD Discovery identity Structure definition
  * @brief Data received from Discover Identity messages
  * @{
  */
typedef struct
{
  USBPD_IDHeaderVDO_TypeDef IDHeader;               /*!< ID Header VDO                                        */
  USBPD_CertStatVdo_TypeDef CertStatVDO;            /*!< Cert Stat VDO                                        */
  USBPD_ProductVdo_TypeDef  ProductVDO;             /*!< Product VDO                                          */
#if defined(USBPDCORE_VCONN_SUPPORT)
  USBPD_CableVdo_TypeDef    CableVDO;               /*!< Passive Cable VDO                                    */
#endif /* USBPDCORE_VCONN_SUPPORT */
  USBPD_AMAVdo_TypeDef      AMA_VDO;                /*!< Alternate Mode Adapter VDO                           */
#if defined(USBPD_REV30_SUPPORT)
#if defined(USBPDCORE_VCONN_SUPPORT)
  USBPD_ActiveCableVdo1_TypeDef ActiveCableVDO1;    /*!< Active Cable VDO 1                                   */
#endif /* USBPDCORE_VCONN_SUPPORT */
  USBPD_UFPVdo_TypeDef      UFP_VDO;                /*!< UFP VDO                                             */
  USBPD_DFPVdo_TypeDef      DFP_VDO;                /*!< DFP VDO                                              */
#if defined(USBPDCORE_VPD)
  USBPD_VPDVdo_TypeDef      VPD_VDO;                /*!< VPD VDO                                              */
#endif /* USBPDCORE_VPD */
#endif /* USBPD_REV30_SUPPORT */
#if defined(USBPDCORE_VCONN_SUPPORT)
  uint8_t                   CableVDO_Presence : 1U; /*!< Indicate Passive Cable VDO presence or not           */
#endif /* USBPDCORE_VCONN_SUPPORT */
  uint8_t                   AMA_VDO_Presence  : 1U; /*!< Indicate Alternate Mode Adapter VDO presence or not  */
#if defined(USBPD_REV30_SUPPORT)
#if defined(USBPDCORE_VCONN_SUPPORT)
  uint8_t                   ActiveCableVDO1_Presence: 1U; /*!< indicate active cable vdo 1 presence or not    */
#endif /* USBPDCORE_VCONN_SUPPORT */
  uint8_t                   UFP_VDO_Presence  : 1U; /*!< Indicate UFP VDO presence or not                     */
  uint8_t                   DFP_VDO_Presence  : 1U; /*!< Indicate DFP VDO presence or not                     */
#if defined(USBPDCORE_VPD)
  uint8_t                   VPD_VDO_Presence  : 1U; /*!< Indicate VPD VDO presence or not                     */
#if defined(USBPDCORE_VCONN_SUPPORT)
  uint8_t                   Reserved          : 2U; /*!< Reserved bits                                        */
#else
  uint8_t                   Reserved          : 3U; /*!< Reserved bits                                        */
#endif /* USBPDCORE_VCONN_SUPPORT */
#else
#if defined(USBPDCORE_VCONN_SUPPORT)
  uint8_t                   Reserved          : 3U; /*!< Reserved bits                                        */
#else
  uint8_t                   Reserved          : 4U; /*!< Reserved bits                                        */
#endif /* USBPDCORE_VCONN_SUPPORT */
#endif /* USBPDCORE_VPD */
#else
#if defined(USBPDCORE_VCONN_SUPPORT)
  uint8_t                   Reserved          : 6U; /*!< Reserved bits                                        */
#else
  uint8_t                   Reserved          : 7U; /*!< Reserved bits                                        */
#endif /* USBPDCORE_VCONN_SUPPORT */
#endif /* USBPD_REV30_SUPPORT */
} USBPD_DiscoveryIdentity_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_ModeInfo_TypeDef USB PD Mode Info object Structure definition
  * @brief USB PD Mode Info object Structure definition
  * @{
  */
typedef struct
{
  uint32_t  NumModes;
  uint32_t  Modes[MAX_MODES_PER_SVID];
  uint16_t  SVID;
} USBPD_ModeInfo_TypeDef;

/**
  * @}
  */

/** @defgroup USBPD_SVID_TypeDef USB PD Discovery SVID Structure definition
  * @brief Data received from Discover Identity messages
  * @{
  */
/*
 * Structure to SVID supported by the devices
 */
typedef struct
{
  uint16_t  SVIDs[12u];
  uint8_t  NumSVIDs;
  uint8_t  AllSVID_Received; /*!< Flag to indicate that all the SVIDs have been received.
                                No need to send new SVDM Discovery SVID message */
} USBPD_SVIDInfo_TypeDef;
/**
  * @}
  */

#if defined(USBPD_REV30_SUPPORT)
/**
  * @brief  USBPD Alert Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t Reserved             : 16u; /*!< Reserved */
    uint32_t HotSwappableBatteries: 4u;  /*!< Hot Swappable Batteries is a combination of @ref USBPD_ADO_HOT_SWAP_BATT */
    uint32_t FixedBatteries       : 4u;  /*!< Fixed Batteries is a combination of @ref USBPD_ADO_FIXED_BATT */
    uint32_t TypeAlert            : 8u;  /*!< Type of Alert is a combination of @ref USBPD_ADO_TYPE_ALERT */
  }
  b;
} USBPD_ADO_TypeDef;

/**
  * @brief  USBPD revision Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t Revision_major    : 4u; /*!< revision major */
    uint32_t Revision_minor    : 4u; /*!< revision minor */
    uint32_t Version_major     : 4u; /*!< version major */
    uint32_t Version_minor     : 4u; /*!< version minor */
    uint32_t                   :16u;  /*!< reserved */
  }
  b;
} USBPD_RevisionDO_TypeDef;



/**
  * @brief  USBPD Battery Status Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t Reserved: 8u;
    uint32_t BatteryInfo: 8u; /*!< Based on @ref USBPD_BSDO_BATT_INFO */
    uint32_t BatteryPC: 16u;
  }
  b;
} USBPD_BSDO_TypeDef;

/**
  * @brief  USBPD Source Capabilities Extended Message Structure definition
  *
  */
typedef struct USBPD_SCEDB_TypeDef
{
  uint16_t VID;                 /*!< Vendor ID (assigned by the USB-IF)                   */
  uint16_t PID;                 /*!< Product ID (assigned by the manufacturer)            */
  uint32_t XID;                 /*!< Value provided by the USB-IF assigned to the product */
  uint8_t  FW_revision;         /*!< Firmware version number                              */
  uint8_t  HW_revision;         /*!< Hardware version number                              */
  uint8_t  Voltage_regulation;  /*!< Voltage Regulation                                   */
  uint8_t  Holdup_time;         /*!< Holdup Time                                          */
  uint8_t  Compliance;          /*!< Compliance                                           */
  uint8_t  TouchCurrent;        /*!< Touch Current                                        */
  uint16_t PeakCurrent1;        /*!< Peak Current1                                        */
  uint16_t PeakCurrent2;        /*!< Peak Current2                                        */
  uint16_t PeakCurrent3;        /*!< Peak Current3                                        */
  uint8_t  Touchtemp;           /*!< Touch Temp                                           */
  uint8_t  Source_inputs;       /*!< Source Inputs                                        */
  uint8_t  NbBatteries;         /*!< Number of Batteries/Battery Slots                    */
  uint8_t  SourcePDP;           /*!< Source PDP                                           */
} USBPD_SCEDB_TypeDef;

#if defined(USBPDCORE_SNK_CAPA_EXT)
/**
  * @brief  SKEDB -  Sink Load Characteristics structure definition
  *
  */
typedef union
{
  uint16_t Value;
  struct
  {
    uint16_t PercentOverload  : 5u; /*!< Percent overload in 10% increments Values higher than 25 (11001b) are clipped to 250%.
                                         00000b is the default.                                   */
    uint16_t OverloadPeriod   : 6u; /*!< Overload period in 20ms when bits 0-4 non-zero.          */
    uint16_t DutyCycle        : 4u; /*!< Duty cycle in 5% increments when bits 0-4 are non-zero.  */
    uint16_t VBusVoltageDrop  : 1u; /*!< Can tolerate VBUS Voltage drop.                          */
  } b;
} USBPD_SKEDB_SinkLoadCharac_TypeDef;
/**
  * @brief  USBPD Sink Capabilities Extended Message Structure definition
  *
  */
typedef struct USBPD_SKEDB_TypeDef
{
  uint16_t VID;                 /*!< Vendor ID (assigned by the USB-IF)                             */
  uint16_t PID;                 /*!< Product ID (assigned by the manufacturer)                      */
  uint32_t XID;                 /*!< Value provided by the USB-IF assigned to the product           */
  uint8_t  FW_revision;         /*!< Firmware version number                                        */
  uint8_t  HW_revision;         /*!< Hardware version number                                        */
  uint8_t  SKEDB_Version;       /*!< SKEDB Version (not the specification Version) based on
                                     @ref USBPD_SKEDB_VERSION                                       */
  uint8_t  LoadStep;            /*!< Load Step based on @ref USBPD_SKEDB_LOADSTEP                   */
  USBPD_SKEDB_SinkLoadCharac_TypeDef SinkLoadCharac;  /*!< Sink Load Characteristics                */
  uint8_t  Compliance;          /*!< Compliance based on combination of @ref USBPD_SKEDB_COMPLIANCE */
  uint8_t  Touchtemp;           /*!< Touch Temp based on @ref USBPD_SKEDB_TOUCHTEMP                 */
  uint8_t  BatteryInfo;         /*!< Battery info                                                   */
  uint8_t  SinkModes;           /*!< Sink Modes based on combination of @ref USBPD_SKEDB_SINKMODES  */
  uint8_t  SinkMinimumPDP;      /*!< The Minimum PDP required by the Sink to operate without
                                     consuming any power from its Battery(s) should it have one     */
  uint8_t  SinkOperationalPDP;  /*!< The PDP the Sink requires to operate normally. For Sinks with
                                     a Battery, it is the PDP Rating of the charger supplied with
                                     it or recommended for it.                                      */
  uint8_t  SinkMaximumPDP;      /*!< The Maximum PDP the Sink can consume to operate and
                                     charge its Battery(s) should it have one.                      */
} USBPD_SKEDB_TypeDef;
#endif /* USBPDCORE_SNK_CAPA_EXT */

/**
  * @brief  USBPD Source Status Extended Message Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  uint8_t InternalTemp;          /*!< Source or Sink internal temperature in degrees centigrade         */
  uint8_t PresentInput;          /*!< Present Input based on @ref USBPD_SDB_PRESENT_INPUT               */
  uint8_t PresentBatteryInput;   /*!< Present Battery Input                                             */
  uint8_t EventFlags;            /*!< Event Flags based on @ref USBPD_SDB_EVENT_FLAGS                   */
  uint8_t TemperatureStatus;     /*!< Temperature based on @ref USBPD_SDB_TEMP_STATUS                   */
  uint8_t PowerStatus;           /*!< Power Status based on combination of @ref USBPD_SDB_POWER_STATUS  */
  uint8_t PowerStateChange;      /*!< The Power state change status byte indicates a power state change
                                      based on @ref USBPD_SDB_PWR_STATE                                 */
} USBPD_SDB_TypeDef;

/**
  * @brief  USBPD Get Battery Capabilities Data Block Extended Message Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  uint8_t BatteryCapRef;     /*!< Number of the Battery indexed from zero    */
} USBPD_GBCDB_TypeDef;

/**
  * @brief  USBPD Get Battery Status Data Block Extended Message Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  uint8_t BatteryStatusRef;     /*!< Number of the Battery indexed from zero  */
} USBPD_GBSDB_TypeDef;

/**
  * @brief  USBPD  Battery Capability Data Block Extended Message Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  uint16_t VID;                       /*!< Vendor ID (assigned by the USB-IF)         */
  uint16_t PID;                       /*!< Product ID (assigned by the manufacturer)  */
  uint16_t BatteryDesignCapa;         /*!< Battery Design Capacity                    */
  uint16_t BatteryLastFullChargeCapa; /*!< Battery last full charge capacity        */
  uint8_t  BatteryType;               /*!< Battery Type                               */
} USBPD_BCDB_TypeDef;

/**
  * @brief  USBPD Get Manufacturer Info Info Data Block Extended Message Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  uint8_t ManufacturerInfoTarget;     /*!< Manufacturer Info Target based on @ref USBPD_MANUFINFO_TARGET                      */
  uint8_t ManufacturerInfoRef;        /*!< Manufacturer Info Ref between Min_Data=0 and Max_Data=7 (@ref USBPD_MANUFINFO_REF) */
} USBPD_GMIDB_TypeDef;

/**
  * @brief  USBPD Manufacturer Info Data Block Extended Message Structure definition
  *
  */
typedef struct
{
  uint16_t VID;                       /*!< Vendor ID (assigned by the USB-IF)        */
  uint16_t PID;                       /*!< Product ID (assigned by the manufacturer) */
  uint8_t ManuString[22];             /*!< Vendor defined byte array                 */
} USBPD_MIDB_TypeDef;

#if defined(USBPDCORE_FWUPD)
/**
  * @brief  USBPD Firmware Update GET_FW_ID Response Payload Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  USBPD_FWUPD_Status_TypeDef   Status;  /*!< Status Information during Firmware Update      */
  uint16_t  VID;         /*!< USB-IF assigned Vendor ID                                     */
  uint16_t  PID;         /*!< USB-IF assigned Product ID                                    */
  uint8_t   HWVersion;   /*!< Hardware Version                                              */
  uint8_t   SiVersion;   /*!< Silicon Version                                               */
  uint16_t  FWVersion1;  /*!< Most significant component of the firmware version            */
  uint16_t  FWVersion2;  /*!< Second-most significant component of the firmware version     */
  uint16_t  FWVersion3;  /*!< Third-most significant component of the firmware version      */
  uint16_t  FWVersion4;  /*!< Least significant component of the firmware version           */
  uint8_t   ImageBank;   /*!< Image bank for which firmware is requested                    */
  uint8_t   Flags1;      /*!< Flags1                                                        */
  uint8_t   Flags2;      /*!< Flags2                                                        */
  uint8_t   Flags3;      /*!< Flags3                                                        */
  uint8_t   Flags4;      /*!< Flags4                                                        */
} USBPD_FWUPD_GetFwIDRspPayload_TypeDef;

/**
  * @brief  USBPD Firmware Update PDFU_INITIATE Request Payload Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  uint16_t  FWVersion1;  /*!< Most significant component of the firmware version            */
  uint16_t  FWVersion2;  /*!< Second-most significant component of the firmware version     */
  uint16_t  FWVersion3;  /*!< Third-most significant component of the firmware version      */
  uint16_t  FWVersion4;  /*!< Least significant component of the firmware version           */
} USBPD_FWUPD_PdfuInitReqPayload_TypeDef;

/**
  * @brief  USBPD Firmware Update PDFU_INITIATE Response Payload Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  USBPD_FWUPD_Status_TypeDef   Status;  /*!< Status Information during Firmware Update      */
  uint8_t   WaitTime;         /*!< Wait time                                                */
  uint8_t   MaxImageSize[3u];  /*!< Max image size                                           */
} USBPD_FWUPD_PdfuInitRspPayload_TypeDef;

/**
  * @brief  USBPD Firmware Update PDFU_DATA Response Payload Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  USBPD_FWUPD_Status_TypeDef   Status;  /*!< Status Information during Firmware Update      */
  uint8_t   WaitTime;         /*!< Wait time                                                */
  uint8_t   NumDataNR;        /*!< Number of PDFU_DATA_NR Requests                          */
  uint16_t  DataBlockNum;     /*!< Data Block Number of the next PDFU_DATA or PDFU_DATA_NR  */
} USBPD_FWUPD_PdfuDataRspPayload_TypeDef;

/**
  * @brief  USBPD Firmware Update PDFU_VALIDATE Response Payload Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  USBPD_FWUPD_Status_TypeDef   Status;  /*!< Status Information during Firmware Update      */
  uint8_t   WaitTime;         /*!< Wait time                                                */
  uint8_t   Flags;            /*!< Flags                                                    */
} USBPD_FWUPD_PdfuValidateRspPayload_TypeDef;

/**
  * @brief  USBPD Firmware Update PDFU_DATA_PAUSE Response Payload Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  USBPD_FWUPD_Status_TypeDef   Status;  /*!< Status Information during Firmware Update      */
} USBPD_FWUPD_PdfuDataPauseRspPayload_TypeDef;

/**
  * @brief  USBPD Firmware Update VENDOR_SPECIFIC Request Payload Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  uint16_t  VID;                /*!< USB-IF assigned Vendor ID                              */
  uint8_t   VendorDefined[256]; /*!< Vendor defined                                         */
} USBPD_FWUPD_VendorSpecificReqPayload_TypeDef;

/**
  * @brief  USBPD Firmware Update VENDOR_SPECIFIC Response Payload Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  USBPD_FWUPD_Status_TypeDef   Status;  /*!< Status Information during Firmware Update      */
  uint16_t  VID;                /*!< USB-IF assigned Vendor ID                              */
  uint8_t   VendorDefined[255]; /*!< Vendor defined                                         */
} USBPD_FWUPD_VendorSpecificRspPayload_TypeDef;

/**
  * @brief  USBPD Firmware Update Request Data Block Extended Message Structure definition
  *
  */
typedef __PACKED_STRUCT
{
  uint8_t   ProtocolVersion;  /*!< Protocol Version (@ref USBPD_FWUPD_PROT_VER)             */
  uint8_t   MessageType;      /*!< Firmware Update Message type (@ref USBPD_FWUPD_MSGTYPE)  */
  uint8_t   Payload[258];     /*!< Payload                                         */
} USBPD_FRQDB_TypeDef;

#endif /* USBPDCORE_FWUPD */

#ifdef USBPDCORE_PPS
/**
  * @brief  USBPD PPS Status Data Block Extended Message Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint16_t OutputVoltageIn20mVunits;  /*!< Source output voltage in 20mV units.
                                             When set to 0xFFFF, the Source does not support this field.          */
    uint8_t  OutputCurrentIn50mAunits;  /*!< Source output current in 50mA units.
                                             When set to 0xFF, the Source does not support this field.            */
    uint8_t  RealTimeFlags;             /*!< Real Time Flags, combination of @ref USBPD_CORE_DEF_REAL_TIME_FLAGS  */
  } fields;
} USBPD_PPSSDB_TypeDef;
#endif /* USBPDCORE_PPS */

/**
  * @brief  USBPD Country Code Data Block Extended Message Structure definition
  *
  */
typedef struct
{
  uint32_t Length;            /*!< Number of country codes in the message */
  uint16_t *PtrCountryCode;   /*!< Pointer of the country codes (1 to n)  */
} USBPD_CCDB_TypeDef;

/**
  * @brief  USBPD Country Info Data Block Extended Message Structure definition
  *
  */
typedef struct
{
  uint16_t CountryCode;             /*!< 1st and 2nd character of the Alpha-2 Country Code defined by [ISO 3166]  */
  uint32_t Reserved;                /*!< Reserved - Shall be set to 0.   */
  uint8_t  PtrCountrySpecificData;  /*!< Pointer on Content defined by the country authority (0 t 256 bytes). */
} USBPD_CIDB_TypeDef;

#endif /* USBPD_REV30_SUPPORT */

/**
  * @}
  */

/** @defgroup USBPD_CORE_SETTINGS_Exported_Structures USBPD CORE Settings Exported Structures
  * @brief  USBPD Settings Structure definition
  * @{
  */
#if defined(USBPD_REV30_SUPPORT)
typedef union
{
  uint16_t PD3_Support;
  struct
  {
    uint16_t PE_UnchunkSupport                : 1u; /*!< Unchunked support                                                          */
    uint16_t PE_FastRoleSwapSupport           : 1u; /*!< Fast role swap support (not yet implemented)                               */
    uint16_t Is_GetPPSStatus_Supported        : 1u; /*!< Get PPS status message supported by PE                                     */
    uint16_t Is_SrcCapaExt_Supported          : 1u; /*!< Source_Capabilities_Extended message supported by PE                       */
    uint16_t Is_Alert_Supported               : 1u; /*!< Alert message supported by PE                                              */
    uint16_t Is_GetStatus_Supported           : 1u; /*!< Get_Status message supported by PE (Is_Alert_Supported should be enabled)  */
    uint16_t Is_GetManufacturerInfo_Supported : 1u; /*!< Manufacturer_Info message supported by PE                                  */
    uint16_t Is_GetCountryCodes_Supported     : 1u; /*!< Get_Country_Codes message supported by PE                                  */
    uint16_t Is_GetCountryInfo_Supported      : 1u; /*!< Get_Country_Info message supported by PE                                   */
    uint16_t Is_SecurityRequest_Supported     : 1u; /*!< Security_Response message supported by PE                                  */
    uint16_t Is_FirmUpdateRequest_Supported   : 1u; /*!< Firmware update response message supported by PE                           */
    uint16_t Reserved2                        : 1u; /*!< Reserved bits: old Is_SnkCapaExt_Supported (SNK_CAPA_EXT msg mandatory)    */
    uint16_t Is_GetBattery_Supported          : 1u; /*!< Get Battery Capabitity and Status messages supported by PE                 */
    uint16_t reserved                         : 3u; /*!< Reserved bits                                                              */
  } d;
} USBPD_PD3SupportTypeDef;
#endif /* USBPD_REV30_SUPPORT */

typedef struct
{
#if defined(USBPDCORE_LIB_NO_PD)
  USBPD_PortPowerRole_TypeDef PE_DefaultRole: 1u; /*!< Default port role  based on @ref USBPD_PortPowerRole_TypeDef                    */
  CAD_RP_Source_Current_Adv_Typedef CAD_DefaultResistor  : 2u; /*!< Default RP resistor based on @ref CAD_RP_Source_Current_Adv_Typedef */
  uint8_t                                   : 5u; /*!< Reserved bits */
#else
  USBPD_SupportedSOP_TypeDef PE_SupportedSOP; /*!<  Corresponds to the message managed by the stack and this should be set depending if you want discuss with the cable
                                                    So if VconnSupport is enabling this field must be set to
                                                      @ref USBPD_SUPPORTED_SOP_SOP |
                                                      @ref USBPD_SUPPORTED_SOP_SOP1 |
                                                      @ref USBPD_SUPPORTED_SOP_SOP2
                                                    else
                                                      @ref USBPD_SUPPORTED_SOP_SOP
                                              */
  USBPD_SpecRev_TypeDef PE_SpecRevision     : 2u; /*!< Spec revision value based on @ref USBPD_SpecRev_TypeDef                         */
  USBPD_PortPowerRole_TypeDef PE_DefaultRole: 1u; /*!< Default port role  based on @ref USBPD_PortPowerRole_TypeDef                    */
  uint32_t PE_RoleSwap                      : 1u; /*!< If enabled, allows the port to have DRP behavior                                */
#if defined(USBPDCORE_VPD)
  uint32_t VPDSupport                       : 1u; /*!< support of the CTVPD device                                                     */
#else
  uint32_t _empty1                          : 1u; /*!< Reserved bit                                                                    */
#endif /* USBPDCORE_VPD */
  uint32_t PE_VDMSupport                    : 1u; /*!< Support VDM: If not enabled any VDM message received is replied "not supported" */
  uint32_t PE_PingSupport                   : 1u; /*!< support Ping (only for PD3.0): If enabled allows DPM to send ping message       */
  uint32_t PE_CapscounterSupport            : 1u; /*!< If enabled after an amount of message source capabilities not replied, the stack stop the message send.*/
  uint32_t PE_RespondsToDiscovSOP           : 1u; /*!< Can respond successfully to a Discover Identity */
  uint32_t PE_AttemptsDiscovSOP             : 1u; /*!< Can send a Discover Identity */
  uint32_t CAD_TryFeature                   : 2u; /*!< Not yet implemented                                                              */
  uint32_t CAD_AccesorySupport              : 1u; /*!< Not yet implemented                                                              */
  uint32_t CAD_RoleToggle                   : 1u; /*!< If enabled allows the detection state machine switch Rp/Rd means toggle the presented role between source and sink */
  CAD_RP_Source_Current_Adv_Typedef CAD_DefaultResistor  : 2u; /*!< Default RP resistor based on @ref CAD_RP_Source_Current_Adv_Typedef */
  uint32_t CAD_SNKToggleTime                : 8u; /*!< Sink toggle time in ms                                                           */
  uint32_t CAD_SRCToggleTime                : 8u; /*!< Source toggle time in ms                                                         */
#if defined(USBPD_REV30_SUPPORT)
  USBPD_PD3SupportTypeDef PE_PD3_Support;         /*!< PD3 structure support flags based on @ref USBPD_PD3SupportTypeDef                */
#else
  uint16_t reserved                         : 16u; /*!< Reserved bits */
#endif /* USBPD_REV30_SUPPORT */
#endif /*USBPDCORE_LIB_NO_PD*/
} USBPD_SettingsTypeDef;

/**
  * @}
  */

/** @defgroup USBPD_CORE_PARAMS_Exported_Structures USBPD CORE Params Exported Structures
  * @brief  USBPD Params Structure definition
  * @{
  */
typedef struct
{
  USBPD_SpecRev_TypeDef               PE_SpecRevision : 2u;  /*!< PE Specification revision                                */
  USBPD_PortPowerRole_TypeDef         PE_PowerRole    : 1u;  /*!< PE Power role                                            */
  USBPD_PortDataRole_TypeDef          PE_DataRole     : 1u;  /*!< PE Data role                                             */
  uint32_t                            PE_SwapOngoing  : 1u;  /*!< Power role swap ongoing flag                             */
  USBPD_VDMVersion_TypeDef            VDM_Version     : 1u;  /*!< VDM version                                              */
  CCxPin_TypeDef                      ActiveCCIs      : 2u;  /*!< Active CC line based on @ref CCxPin_TypeDef              */
  USBPD_POWER_StateTypedef            PE_Power        : 3u;  /*!< Power status based on @ref USBPD_POWER_StateTypedef      */
  uint32_t                            DPM_Initialized : 1u;  /*!< DPM initialized flag                                     */
  uint32_t                            PE_IsConnected  : 1u;  /*!< USB-PD PE stack is connected to CC line                  */
  CCxPin_TypeDef                      VconnCCIs       : 2u;  /*!< VConn  CC line based on @ref CCxPin_TypeDef              */
  uint32_t                            VconnStatus     : 1u;  /*!< VConnStatus USBP_TRUE = vconn on USBPD_FALSE = vconn off */
  CAD_RP_Source_Current_Adv_Typedef   RpResistor      : 2u;  /*!< RpResistor presented                                     */
  CAD_SNK_Source_Current_Adv_Typedef  SNKExposedRP_AtAttach : 2u; /*!< Exposed resistance from source at attach            */

#if defined(USBPDCORE_VPD)
  uint32_t                            VPDflag         : 1u;  /*!< VPD detection flag                                       */
  uint32_t                            CAD_VPDStatus   : 2u;  /*!< CAD VPD status used between CAD and PE                   */
  uint32_t                            PE_VPDStatus    : 2u;  /*!< CAD VPD status used between CAD and PE                   */
#else
  uint32_t                            Reserved1       : 5u;  /*!< Reserved bits                                            */
#endif /* USBPDCORE_VPD */


#if defined(USBPD_REV30_SUPPORT) && defined(USBPDCORE_UNCHUNCKED_MODE)
  uint32_t                            PE_UnchunkSupport: 1u; /*!< Unchunked support                                        */
#else
  uint32_t                            Reserved2        : 1u; /*!< Reserved bits                                            */
#endif /* defined(USBPD_REV30_SUPPORT) && defined(USBPDCORE_UNCHUNCKED_MODE) */

#if defined(USBPDCORE_VCONN_SUPPORT)
  USBPD_SpecRev_TypeDef               CBL_SpecRevision: 2u;  /*!< Cable Specification revision                             */
#else
  uint32_t                            Reserved3        : 2u;  /*!< Reserved bits                                           */
#endif /* USBPDCORE_VCONN_SUPPORT */

#if defined(USBPDCORE_PECABLE)
  uint32_t                            IsCableConnected: 1u;  /*!< Is Cable connected                                       */
  uint32_t                            Reserved4       : 3u;  /*!< Reserved bits                                            */
#else
  uint32_t                            Reserved4        : 4u;  /*!< Reserved bits                                            */
#endif /* USBPDCORE_PECABLE */
} USBPD_ParamsTypeDef;

#if defined(USBPDCORE_USBDATA)
/**
  * @brief  Enter USB Data object
  *
  */
typedef union
{
  uint32_t d32;
  struct {
  uint32_t Reserved1               :13u;  /*!< Reserved  - Shall be set to zero                               */
  uint32_t HostPresent             : 1u;  /*!< Connected to a Host.                                           */
  uint32_t TBTSupport              : 1u;  /*!< [TBT3] is supported by the host's USB4 Connection Manager      */
  uint32_t DPSupport               : 1u;  /*!< [USB4] DP tunneling supported by the host                      */
  uint32_t PCISupport              : 1u;  /*!< [USB4] PCIe tunneling supported by the hosts                   */
  uint32_t CableCurrent            : 2u;  /*!< 00b = VBUS is not supported 01b = Reserved 10b = 3A 11b = 5A   */
  uint32_t CableType               : 2u;  /*!< 00b: Passive,
                                               01b: Active Re-timer
                                               10b: Active Re-driver
                                               11b: Optically Isolatedt */
  uint32_t CableSpeed              : 3u;  /*!< 000b: [USB 2.0] only, no SuperSpeed support,
                                               001b: [USB 3.2] Gen1,
                                               010b: [USB 3.2] Gen2 and [USB4] Gen2
                                               011b: [USB4] Gen3          */
  uint32_t Reserved2               : 1u;  /*!< Reserved  - Shall be set to zero                               */
  uint32_t USB3DRD                 : 1u;  /*!<  0b: Not capable of operating as a [USB 3.2] Device
                                                1b: Capable of operating as a [USB 3.2] Device                */
  uint32_t USB4DRD                 : 1u;  /*!<  0b: Not capable of operating as a [USB 4] Device
                                                1b: Capable of operating as a [USB 4] Device                  */
  uint32_t Reserved3               : 1u;
  uint32_t USBMode                 : 3u;  /*! <000b: [USB 2.0], 001b: [USB 3.2] 010b: [USB4]                  */
  uint32_t Reserved4               : 1u;
  }b;
} USBPD_EnterUSBData_TypeDef;
#endif /* USBPDCORE_USBDATA */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* USBPD_DEF_H_ */

