/*!
 * @file        usbh_core.c
 *
 * @brief       USB host core function
 *
 * @version     V1.0.0
 *
 * @date        2023-01-16
 *
 * @attention
 *
 *  Copyright (C) 2023 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/* Includes */
#include "usbh_channel.h"
#include "usbh_core.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_Core
  @{
  */

/** @defgroup USBH_Core_Functions Functions
  @{
  */

/*!
 * @brief     Clear all of the pipe data buffer.
 *
 * @param     usbInfo
 *
 * @retval    None
 */
void USBH_CH_Clear(USBH_INFO_T* usbInfo)
{
    uint8_t i = 0;

    for (i = 0; i < USBH_CHANNEL_MAX_NUM; i++)
    {
        usbInfo->xferChannel[i] = 0;
    }
}

/*!
 * @brief     Get the free channel.
 *
 * @param     usbInfo
 *
 * @retval    None.
 */
uint16_t USBH_CH_ReadFreeChannel(USBH_INFO_T* usbInfo)
{
    uint8_t i;

    for (i = 0; i < USBH_CHANNEL_MAX_NUM; i++)
    {
        if ((usbInfo->xferChannel[i] & 0x8000) == 0)
        {
            return i;
        }
    }

    return 0xFFFF;
}

/*!
 * @brief     Free the channel.
 *
 * @param     usbInfo
 *
 * @param     chNum : channel to be free.
 *
 * @retval    None.
 */
void USBH_CH_FreeChannel(USBH_INFO_T* usbInfo, uint8_t chNum)
{
    if (chNum < USBH_CHANNEL_MAX_NUM)
    {
        usbInfo->xferChannel[chNum] &= 0x7FFF;
    }
}

/*!
 * @brief     Alloc a channel for a endpoint by address.
 *
 * @param     usbInfo
 *
 * @param     epAddr : endpoint address to be alloc channel.
 *
 * @retval    channel number.
 */
uint8_t USBH_CH_AllocChannel(USBH_INFO_T* usbInfo, uint8_t epAddr)
{
    uint16_t channelNum;

    channelNum = USBH_CH_ReadFreeChannel(usbInfo);

    if (channelNum != 0xFFFF)
    {
        usbInfo->xferChannel[channelNum & 0x0F] = (uint32_t)(0x8000 | epAddr);
    }

    return (uint8_t)channelNum;
}

/**@} end of group USBH_Core_Functions */
/**@} end of group USBH_Core */
/**@} end of group APM32_USB_Library */
