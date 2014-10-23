
#pragma once

#define NVIC_PRIORITY_GROUPING NVIC_PriorityGroup_2

// can't use 0 
#define MAX_IRQ_PRIORITY                      0
#define MAX_IRQ_SUBPRIORITY                   1

#define TIMER_IRQ_PRIORITY                    1
#define TIMER_IRQ_SUBPRIORITY                 1

#define BARO_EXTIRQ_PRIORITY               0x0f
#define BARO_EXTIRQ_SUBPRIORITY            0x0f

#define WS2811_DMA_IRQ_PRIORITY               1             // TODO - is there some reason to use high priority? (or to use DMA IRQ at all?)
#define WS2811_DMA_IRQ_SUBPRIORITY            2

#define SERIALUART1_TXDMA_IRQ_PRIORITY        1
#define SERIALUART1_TXDMA_IRQ_SUBPRIORITY     1

#define SERIALUART1_RXDMA_IRQ_PRIORITY        1
#define SERIALUART1_RXDMA_IRQ_SUBPRIORITY     1

#define SERIALUART1_IRQ_PRIORITY              1
#define SERIALUART1_IRQ_SUBPRIORITY           1

#define SERIALUART2_TXDMA_IRQ_PRIORITY        1
#define SERIALUART2_TXDMA_IRQ_SUBPRIORITY     0

#define SERIALUART2_RXDMA_IRQ_PRIORITY        1
#define SERIALUART2_RXDMA_IRQ_SUBPRIORITY     1

#define SERIALUART2_IRQ_PRIORITY              1
#define SERIALUART2_IRQ_SUBPRIORITY           2

#define SERIALUART3_IRQ_PRIORITY              1
#define SERIALUART3_IRQ_SUBPRIORITY           2

#define I2C_ER_IRQ_PRIORITY                   0
#define I2C_ER_IRQ_SUBPRIORITY                0
#define I2C_EV_IRQ_PRIORITY                   0
#define I2C_EV_IRQ_SUBPRIORITY                0

#define USB_IRQ_PRIORITY                      2
#define USB_IRQ_SUBPRIORITY                   0
#define USB_WUP_IRQ_PRIORITY                  1
#define USB_WUP_IRQ_SUBPRIORITY               0

#define CALLBACK_IRQ_PRIORITY 0x0f
#define CALLBACK_IRQ_SUBPRIORITY 0x0f

// utility macros to join/split priority
#define NVIC_BUILD_PRIORITY(base,sub) ((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING>>8))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8)))))<<4)
#define NVIC_SPLIT_PRIORITY_BASE(prio) (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
#define NVIC_SPLIT_PRIORITY_SUB(prio) (((prio)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
