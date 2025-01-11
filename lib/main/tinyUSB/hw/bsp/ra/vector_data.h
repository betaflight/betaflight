/* vector numbers are configurable/dynamic, hence this, it will be used inside the port */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(BSP_MCU_GROUP_RA6M5) || defined(BSP_MCU_GROUP_RA6M3) || (BSP_CFG_MCU_PART_SERIES == 8)
#define BOARD_HAS_USB_HIGHSPEED
#endif

/* ISR prototypes */
void usbfs_interrupt_handler(void);
void usbfs_resume_handler(void);

#ifndef BSP_MCU_GROUP_RA2A1
void usbfs_d0fifo_handler(void);
void usbfs_d1fifo_handler(void);
#endif

#ifdef BOARD_HAS_USB_HIGHSPEED
void usbhs_interrupt_handler(void);
void usbhs_d0fifo_handler(void);
void usbhs_d1fifo_handler(void);
#endif

/* Vector table allocations */
#define USBFS_INT_IRQn    0
#define USBFS_RESUME_IRQn 1
#define USBFS_FIFO_0_IRQn 2
#define USBFS_FIFO_1_IRQn 3

#define USBHS_USB_INT_RESUME_IRQn  4 /* USBHS USB INT RESUME (USBHS interrupt) */
#define USBHS_FIFO_0_IRQn          5 /* USBHS FIFO 0 (DMA transfer request 0) */
#define USBHS_FIFO_1_IRQn          6 /* USBHS FIFO 1 (DMA transfer request 1) */


#ifdef __cplusplus
}
#endif

#endif
