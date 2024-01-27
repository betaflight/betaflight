/**
  ******************************************************************************
  * @file    Inc/usbd_cdc_ecm_if_template.h
  * @author  MCD Application Team
  * @brief   Header for usbd_cdc_ecm_if_template.c file.
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
#ifndef __USBD_CDC_ECM_IF_H
#define __USBD_CDC_ECM_IF_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_ecm.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Ensure this MAC address value is same as MAC_ADDRx declared in STM32xxx_conf.h */
#define CDC_ECM_MAC_STR_DESC                 (uint8_t *)"000202030000"
#define CDC_ECM_MAC_ADDR0                    0x00U /* 01 */
#define CDC_ECM_MAC_ADDR1                    0x02U /* 02 */
#define CDC_ECM_MAC_ADDR2                    0x02U /* 03 */
#define CDC_ECM_MAC_ADDR3                    0x03U /* 00 */
#define CDC_ECM_MAC_ADDR4                    0x00U /* 00 */
#define CDC_ECM_MAC_ADDR5                    0x00U /* 00 */

/* Max Number of Trials waiting for Tx ready */
#define CDC_ECM_MAX_TX_WAIT_TRIALS           1000000U

#define CDC_ECM_ETH_STATS_BYTE0                                 0U
/*(uint8_t)(CDC_ECM_XMIT_OK_VAL | CDC_ECM_RVC_OK_VAL | CDC_ECM_XMIT_ERROR_VAL | \
            CDC_ECM_RCV_ERROR_VAL | CDC_ECM_RCV_NO_BUFFER_VAL | CDC_ECM_DIRECTED_BYTES_XMIT_VAL | \
            CDC_ECM_DIRECTED_FRAMES_XMIT_VAL | CDC_ECM_MULTICAST_BYTES_XMIT_VAL) */

#define CDC_ECM_ETH_STATS_BYTE1                                 0U
/*(uint8_t)(CDC_ECM_MULTICAST_FRAMES_XMIT_VAL | CDC_ECM_BROADCAST_BYTES_XMIT_VAL | \
            CDC_ECM_BROADCAST_FRAMES_XMIT_VAL | CDC_ECM_DIRECTED_BYTES_RCV_VAL | \
            CDC_ECM_DIRECTED_FRAMES_RCV_VAL | CDC_ECM_MULTICAST_BYTES_RCV_VAL | \
            CDC_ECM_MULTICAST_FRAMES_RCV_VAL | CDC_ECM_BROADCAST_BYTES_RCV_VAL) */

#define CDC_ECM_ETH_STATS_BYTE2                                 0U
/*(uint8_t)(CDC_ECM_BROADCAST_FRAMES_RCV_VAL | CDC_ECM_RCV_CRC_ERROR_VAL | \
            CDC_ECM_TRANSMIT_QUEUE_LENGTH_VAL | CDC_ECM_RCV_ERROR_ALIGNMENT_VAL | \
            CDC_ECM_XMIT_ONE_COLLISION_VAL | CDC_ECM_XMIT_MORE_COLLISIONS_VAL | \
            CDC_ECM_XMIT_DEFERRED_VAL | CDC_ECM_XMIT_MAX_COLLISIONS_VAL) */

#define CDC_ECM_ETH_STATS_BYTE3                                 0U
/*(uint8_t)(CDC_ECM_RCV_OVERRUN_VAL | CDC_ECM_XMIT_UNDERRUN_VAL | CDC_ECM_XMIT_HEARTBEAT_FAILURE_VAL | \
            CDC_ECM_XMIT_TIMES_CRS_LOST_VAL | CDC_ECM_XMIT_LATE_COLLISIONS_VAL | CDC_ECM_ETH_STATS_RESERVED) */

/* Ethernet Maximum Segment size, typically 1514 bytes */
#define CDC_ECM_ETH_MAX_SEGSZE                                  1514U

/* Number of Ethernet multicast filters */
#define CDC_ECM_ETH_NBR_MACFILTERS                              0U

/* Number of wakeup power filters */
#define CDC_ECM_ETH_NBR_PWRFILTERS                              0U


#define CDC_ECM_CONNECT_SPEED_UPSTREAM                          0x004C4B40U /* 5Mbps */
#define CDC_ECM_CONNECT_SPEED_DOWNSTREAM                        0x004C4B40U /* 5Mbps */

extern USBD_CDC_ECM_ItfTypeDef                          USBD_CDC_ECM_fops;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#endif /* __USBD_CDC_ECM_IF_H */

