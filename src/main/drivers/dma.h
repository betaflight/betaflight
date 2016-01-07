/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

typedef void (*dmaCallbackHandlerFuncPtr)(DMA_Channel_TypeDef *channel);

typedef enum {
    DMA1_CH2_HANDLER = 0,
    DMA1_CH3_HANDLER,
    DMA1_CH6_HANDLER,
    DMA1_CH7_HANDLER,
} dmaHandlerIdentifier_e;

typedef struct dmaHandlers_s {
    dmaCallbackHandlerFuncPtr dma1Channel2IRQHandler;
    dmaCallbackHandlerFuncPtr dma1Channel3IRQHandler;
    dmaCallbackHandlerFuncPtr dma1Channel6IRQHandler;
    dmaCallbackHandlerFuncPtr dma1Channel7IRQHandler;
} dmaHandlers_t;

void dmaInit(void);
void dmaSetHandler(dmaHandlerIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback);
