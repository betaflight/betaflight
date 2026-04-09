/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * porting for ch32h41x by Temperslee
 */
#pragma once
#include <stdint.h>
#include "drivers/resource.h"
#include "platform.h"

#if defined(USE_CHBSP_DRIVER)
#define PLATFORM_TRAIT_DMA_MUX_REQUIRED 1
#endif

#define DMA_ARCH_TYPE DMA_Channel_TypeDef

// typedef enum {
//   DMA_NONE = 0,
//   DMA_FIRST_HANDLER = 1,
//   DMA1_CH1_HANDLER = DMA_FIRST_HANDLER,
//   DMA1_CH2_HANDLER,
//   DMA1_CH3_HANDLER,
//   DMA1_CH4_HANDLER,
//   DMA1_CH5_HANDLER,
//   DMA1_CH6_HANDLER,
//   DMA1_CH7_HANDLER,
//   DMA1_CH8_HANDLER,
//   DMA2_CH1_HANDLER,
//   DMA2_CH2_HANDLER,
//   DMA2_CH3_HANDLER,
//   DMA2_CH4_HANDLER,
//   DMA2_CH5_HANDLER,
//   DMA2_CH6_HANDLER,
//   DMA2_CH7_HANDLER,
//   DMA2_CH8_HANDLER,
//   DMA_LAST_HANDLER = DMA2_CH8_HANDLER
// } dmaIdentifier_e;

  #define DMA1_CH1_HANDLER    (DMA_FIRST_HANDLER + 0)
  #define DMA1_CH2_HANDLER    (DMA_FIRST_HANDLER + 1)
  #define DMA1_CH3_HANDLER    (DMA_FIRST_HANDLER + 2)
  #define DMA1_CH4_HANDLER    (DMA_FIRST_HANDLER + 3)
  #define DMA1_CH5_HANDLER    (DMA_FIRST_HANDLER + 4)
  #define DMA1_CH6_HANDLER    (DMA_FIRST_HANDLER + 5)
  #define DMA1_CH7_HANDLER    (DMA_FIRST_HANDLER + 6)
  #define DMA1_CH8_HANDLER    (DMA_FIRST_HANDLER + 7)
  #define DMA2_CH1_HANDLER    (DMA_FIRST_HANDLER + 8)
  #define DMA2_CH2_HANDLER    (DMA_FIRST_HANDLER + 9)
  #define DMA2_CH3_HANDLER    (DMA_FIRST_HANDLER + 10)
  #define DMA2_CH4_HANDLER    (DMA_FIRST_HANDLER + 11)
  #define DMA2_CH5_HANDLER    (DMA_FIRST_HANDLER + 12)
  #define DMA2_CH6_HANDLER    (DMA_FIRST_HANDLER + 13)
  #define DMA2_CH7_HANDLER    (DMA_FIRST_HANDLER + 14)
  #define DMA2_CH8_HANDLER    (DMA_FIRST_HANDLER + 15)
  #define DMA_LAST_HANDLER    (DMA2_CH8_HANDLER)




#define DMA_DEVICE_NO(x) ((((x) - 1) / 8) + 1)
#define DMA_DEVICE_INDEX(x) ((((x) - 1) % 8) + 1)

uint32_t dmaGetChannel(const uint8_t channel);

#define DMA_OUTPUT_INDEX 0
#define DMA_OUTPUT_STRING "DMA%d Channel %d:"
#define DMA_INPUT_STRING "DMA%d_CH%d"

#define DEFINE_DMA_CHANNEL(d, c, f)                                            \
  {.dma = d,                                                                   \
   .ref = (dmaResource_t *)d##_Channel##c##_BASE,                              \
   .irqHandlerCallback = NULL,                                                 \
   .flagsShift = f,                                                            \
   .irqN = d##_Channel##c##_IRQn,                                              \
   .userParam = 0,                                                             \
   .resourceOwner.owner = 0,                                                   \
   .resourceOwner.index = 0,                                                   \
   .dmamux = (dmamux_channel_type *)d##MUX_CHANNEL##c}

#define DMA_HANDLER_CODE __attribute__((interrupt("WCH-Interrupt-fast")))

#define DEFINE_DMA_IRQ_HANDLER(d, c, i)                                        \
  DMA_HANDLER_CODE void DMA##d##_Channel##c##_IRQHandler(void) {               \
    const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i);                          \
    dmaCallbackHandlerFuncPtr handler =                                        \
        dmaDescriptors[index].irqHandlerCallback;                              \
    if (handler)                                                               \
      handler(&dmaDescriptors[index]);                                         \
  }

#define DMA_CLEAR_FLAG(d, flag)  ((DMA_TypeDef *)(d)->dma)->INTFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (((DMA_TypeDef *)(d)->dma)->INTFR & (flag << d->flagsShift))
#define DMA_IT_GLOB ((uint32_t)0x00000001) // channel global interput flag
#define DMA_IT_TCIF ((uint32_t)0x00000002) // channel full transport flag
#define DMA_IT_HTIF ((uint32_t)0x00000004) // channel half transport flag
#define DMA_IT_TEIF ((uint32_t)0x00000008) // channel transport error flag


#define xDMA_Init(dmaResource, initStruct)                                     \
  DMA_Init((DMA_ARCH_TYPE *)(dmaResource), initStruct)
#define xDMA_DeInit(dmaResource) DMA_DeInit((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_Cmd(dmaResource, newState)                                        \
  DMA_Cmd((DMA_ARCH_TYPE *)(dmaResource), newState)
#define xDMA_ITConfig(dmaResource, flags, newState)                            \
  DMA_ITConfig((DMA_ARCH_TYPE *)(dmaResource), flags, newState)
#define xDMA_GetCurrDataCounter(dmaResource)                                   \
  DMA_GetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_SetCurrDataCounter(dmaResource, count)                            \
  DMA_SetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource), count)
#define xDMA_GetFlagStatus(dmaResource, flags)                                 \
  DMA_GetFlagStatus((DMA_ARCH_TYPE *)(dmaResource), flags)
#define xDMA_ClearFlag(dmaResource, flags)                                     \
  DMA_ClearFlag((DMA_ARCH_TYPE *)(dmaResource), flags)

#define DMA_CCR_EN 1U
#define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->CFGR & DMA_CCR_EN)
