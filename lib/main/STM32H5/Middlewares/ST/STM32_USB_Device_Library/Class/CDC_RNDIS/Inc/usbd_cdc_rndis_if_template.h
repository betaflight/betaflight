/**
  ******************************************************************************
  * @file    usbd_cdc_rndis_if_template.h
  * @author  MCD Application Team
  * @brief   Header for usbd_cdc_rndis_if.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_RNDIS_IF_H
#define __USBD_CDC_RNDIS_IF_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_rndis.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Ensure this MAC address value is same as MAC_ADDRx declared in STM32xxx_conf.h */
#define CDC_RNDIS_MAC_STR_DESC                 (uint8_t *)"000202030000"
#define CDC_RNDIS_MAC_ADDR0                    0x00U /* 01 */
#define CDC_RNDIS_MAC_ADDR1                    0x02U /* 02 */
#define CDC_RNDIS_MAC_ADDR2                    0x02U /* 03 */
#define CDC_RNDIS_MAC_ADDR3                    0x03U /* 00 */
#define CDC_RNDIS_MAC_ADDR4                    0x00U /* 00 */
#define CDC_RNDIS_MAC_ADDR5                    0x00U /* 00 */

#define USBD_CDC_RNDIS_VENDOR_DESC             "STMicroelectronics"
#define USBD_CDC_RNDIS_LINK_SPEED              100000U /* 10Mbps */
#define USBD_CDC_RNDIS_VID                     0x0483U

/* Max Number of Trials waiting for Tx ready */
#define CDC_RNDIS_MAX_TX_WAIT_TRIALS           1000000U

/* Ethernet Maximum Segment size, typically 1514 bytes */
#define CDC_RNDIS_ETH_MAX_SEGSZE                            1514U

#define CDC_RNDIS_CONNECT_SPEED_UPSTREAM                    0x1E000000U
#define CDC_RNDIS_CONNECT_SPEED_DOWNSTREAM                  0x1E000000U


extern USBD_CDC_RNDIS_ItfTypeDef                    USBD_CDC_RNDIS_fops;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __USBD_CDC_RNDIS_IF_H */

