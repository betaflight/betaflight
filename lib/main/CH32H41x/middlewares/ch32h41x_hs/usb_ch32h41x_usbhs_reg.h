/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

/**********************************/
/*********USB high speed**********/
/**********************************/
typedef volatile unsigned short *PUINT16V;
typedef volatile unsigned long *PUINT32V;
typedef volatile unsigned char *PUINT8V;

/* USB high speed device register */
#define R8_USB_CTRL             (*((PUINT8V)0x40030000)) // RW, USB_high_speed control register
#define USBHS_UD_LPM_EN         0x80                     // RW, enable LPM
#define USBHS_UD_DEV_EN         0x20                     // RW, enable USB equipment
#define USBHS_UD_DMA_EN         0x10                     // RW, enable DMA transmit
#define USBHS_UD_PHY_SUSPENDM   0x08                     // RW, suspeng USB PHY
#define USBHS_UD_CLR_ALL        0x04                     // RW, clear all interupt flag
#define USBHS_UD_RST_SIE        0x02                     // RW, reset USB protocol processor,including end point register
#define USBHS_UD_RST_LINK       0x01                     // RW, enable LNK layer reset
#define R8_USB_BASE_MODE        (*((PUINT8V)0x40030001)) // RW, USB_high_speed mode control register
#define USBHS_UD_SPEED_FULL     0x00
#define USBHS_UD_SPEED_HIGH     0x01
#define USBHS_UD_SPEED_LOW      0x02
#define USBHS_UD_SPEED_TYPE     0x03                      // RW, speed mode excpeted by the equipment,00:full speed, 01:high speed, 10:low speed
#define R8_USB_INT_EN           (*((PUINT8V)0x40030002))  // RW, USB_high_speed intreurpt enable register
#define USBHS_UDIE_FIFO_OVER    0x80                      // RW, enable fifo overflow interupt
#define USBHS_UDIE_LINK_RDY     0x40                      // RW, enable USB conect interupt
#define USBHS_UDIE_SOF_ACT      0x20                      // RW, enable SOF package received interupt
#define USBHS_UDIE_TRANSFER     0x10                      // RW, enable USB transmit end interupt
#define USBHS_UDIE_LPM_ACT      0x08                      // RW, enable lpm transmit end interupt
#define USBHS_UDIE_BUS_SLEEP    0x04                      // RW, enable usb bus sleep interupt
#define USBHS_UDIE_SUSPEND      0x02                      // RW, enable usb bus suspend interupt
#define USBHS_UDIE_BUS_RST      0x01                      // RW, enable usb bus reset interupt
#define R8_USB_DEV_AD           (*((PUINT8V)0x40030003))  // RW, USB_high_speed device adress register
#define USBHS_UD_DEV_ADDR       0x7F                      // RW, adress of usb equipment
#define R8_USB_WAKE_CTRL        (*((PUINT8V)0x40030004))  // RW, USB_high_speed wake up remotely register
#define USBHS_UD_UD_REMOTE_WKUP 0x01                      // RW1, wake up remotely and auto reset hardware
#define R8_USB_TEST_MODE        (*((PUINT8V)0x40030005))  // RW, USB_high_speed test mode register
#define USBHS_UD_TEST_EN        0x80                      // RW, enable test mode
#define USBHS_UD_TEST_SE0NAK    0x08                      // RW, output SE0 when in test mode
#define USBHS_UD_TEST_PKT       0x04                      // RW, output one package(including DATA0,data and length of end pont4) when in test mode,not work on virtual equipment
#define USBHS_UD_TEST_K         0x02                      // RW, output K when in test mode
#define USBHS_UD_TEST_J         0x01                      // RW, output J when in test mode
#define R16_USB_LPM_DATA        (*((PUINT16V)0x40030006)) // RW, USB_high_speed power control register
#define USBHS_UD_LPM_BUSY       0x8000                    // RW, power control busy
#define USBHS_UD_LPM_DATA       0x07FF                    // RO, power control data
#define R8_USB_INT_FG           (*((PUINT8V)0x40030008))  // RW, USB_high_speed interupt flag register
#define USBHS_UDIF_FIFO_OV      0x80                      // RW1, clear fifo overflow interupt flag
#define USBHS_UDIF_LINK_RDY     0x40                      // RW1, clear USB conect interupt flag
#define USBHS_UDIF_RX_SOF       0x20                      // RW1, clear SOF package received interupt flag
#define USBHS_UDIF_TRANSFER     0x10                      // RO,  USB transmit end interupt flag,cleared by USBHS_UDMS_HS_MOD
#define USBHS_UDIF_LPM_ACT      0x08                      // RW1, clear lpm transmit end interupt flag
#define USBHS_UDIF_BUS_SLEEP    0x04                      // RW1, clear usb bus sleep interupt flag
#define USBHS_UDIF_SUSPEND      0x02                      // RW1, clear usb bus suspend interupt flag
#define USBHS_UDIF_BUS_RST      0x01                      // RW1, clear usb bus reset interupt flag
#define R8_USB_INT_ST           (*((PUINT8V)0x40030009))  // RW, USB_high_speed interupt status register
#define USBHS_UDIS_EP_DIR       0x10                      // RO, end point tranfer diector of data
#define USBHS_UDIS_EP_ID_MASK   0x07                      // RO, number of end point which data transmission occured
#define R8_USB_MIS_ST           (*((PUINT8V)0x4003000A))  // RW, USB_high_speed miscellaneous register
#define USBHS_UDMS_HS_MOD       0x80                      // RO, host with high speed
#define USBHS_UDMS_SUSP_REQ     0x10                      // RO, requirment of suspending USB
#define USBHS_UDMS_SIE_FREE     0x08                      // RO, USB free state
#define USBHS_UDMS_SLEEP        0x04                      // RO, USB sleep state
#define USBHS_UDMS_SUSPEND      0x02                      // RO, USB in suspend state
#define USBHS_UDMS_READY        0x01                      // RO, USB in connected state
#define R16_USB_FRAME_NO        (*((PUINT16V)0x4003000C)) // RW, USB_high_speed frame number register
#define USBHS_UD_MFRAME_NO      0xE000
#define USBHS_UD_FRAME_NO       0x07FF
#define R16_USB_BUS             (*((PUINT16V)0x4003000E)) // RW, USB_high_speed bus status register
#define USBHS_USB_DM_ST         0x08
#define USBHS_USB_DP_ST         0x04
#define USB_WAKEUP              0x01
#define R16_UEP_TX_EN           (*((PUINT16V)0x40030010)) // RW, USB_high_speed end point transmit enable register
/* Bit definition for R16_U2EP_TX_EN & R16_U2EP_RX_EN register */
#define RB_EP0_EN  0x0001
#define RB_EP1_EN  0x0002
#define RB_EP2_EN  0x0004
#define RB_EP3_EN  0x0008
#define RB_EP4_EN  0x0010
#define RB_EP5_EN  0x0020
#define RB_EP6_EN  0x0040
#define RB_EP7_EN  0x0080
#define RB_EP8_EN  0x0100
#define RB_EP9_EN  0x0200
#define RB_EP10_EN 0x0400
#define RB_EP11_EN 0x0800
#define RB_EP12_EN 0x1000
#define RB_EP13_EN 0x2000
#define RB_EP14_EN 0x4000
#define RB_EP15_EN 0x8000

#define R16_UEP_RX_EN         (*((PUINT16V)0x40030012)) // RW, USB_high_speed end point receive enableregister
#define USBHS_UEP_RX_EN        0xFFFF
#define R16_UEP_T_TOG_AUTO    (*((PUINT16V)0x40030014)) // RW, USB_high_speed end point transmit auto toggle enable register
#define USBHS_UEP_T_TOG_AUTO   0xFF
#define R16_UEP_R_TOG_AUTO    (*((PUINT16V)0x40030016)) // RW, USB_high_speed end point receive auto toggle enable register
#define USBHS_UEP_R_TOG_AUTO   0xFF
#define R8_UEP_T_BURST        (*((PUINT8V)0x40030018)) // RW, USB_high_speed end point transmit burst register
#define USBHS_UEP_T_BURST_EN   0xFF
#define R8_UEP_T_BURST_MODE   (*((PUINT8V)0x40030019)) // RW, USB_high_speed end point transmit burst mode register
#define USBHS_UEP_T_BURST_MODE 0xFF
#define R8_UEP_R_BURST        (*((PUINT8V)0x4003001A)) // RW, USB_high_speed end point receive burst register
#define USBHS_UEP_R_BURST_EN   0xFF
#define R8_UEP_R_RES_MODE     (*((PUINT8V)0x4003001B)) // RW, USB_high_speed end point transmit reply mode register
#define USBHS_UEP_R_RES_MODE   0xFF
#define R32_UEP_AF_MODE       (*((PUINT32V)0x4003001C)) // RW, USB_high_speed end point multiplexing register
#define USBHS_UEP_T_AF         0xFE

#define R32_UEP0_DMA          (*((PUINT32V)0x40030020)) // RW, USB_high_speed end point0 begin adress of DMA buffer register
#define UEPn_DMA               0xFFFFFF
#define R32_UEP1_RX_DMA       (*((PUINT32V)0x40030024)) // RW, USB_high_speed end point1 begin adress of DMA receive buffer register
#define R32_UEP2_RX_DMA       (*((PUINT32V)0x40030028)) // RW, USB_high_speed end point2 begin adress of DMA receive buffer register
#define R32_UEP3_RX_DMA       (*((PUINT32V)0x4003002C)) // RW, USB_high_speed end point3 begin adress of DMA receive buffer register
#define R32_UEP4_RX_DMA       (*((PUINT32V)0x40030030)) // RW, USB_high_speed end point4 begin adress of DMA receive buffer register
#define R32_UEP5_RX_DMA       (*((PUINT32V)0x40030034)) // RW, USB_high_speed end point5 begin adress of DMA receive buffer register
#define R32_UEP6_RX_DMA       (*((PUINT32V)0x40030038)) // RW, USB_high_speed end point6 begin adress of DMA receive buffer register
#define R32_UEP7_RX_DMA       (*((PUINT32V)0x4003003C)) // RW, USB_high_speed end point7 begin adress of DMA receive buffer register
#define UEPn_RX_DMA            0xFFFFFF
#define R32_UEP1_TX_DMA       (*((PUINT32V)0x40030040)) // RW, USB_high_speed end point1 begin adress of DMA transmit buffer register
#define R32_UEP2_TX_DMA       (*((PUINT32V)0x40030044)) // RW, USB_high_speed end point2 begin adress of DMA transmit buffer register
#define R32_UEP3_TX_DMA       (*((PUINT32V)0x40030048)) // RW, USB_high_speed end point3 begin adress of DMA transmit buffer register
#define R32_UEP4_TX_DMA       (*((PUINT32V)0x4003004C)) // RW, USB_high_speed end point4 begin adress of DMA transmit buffer register
#define R32_UEP5_TX_DMA       (*((PUINT32V)0x40030050)) // RW, USB_high_speed end point5 begin adress of DMA transmit buffer register
#define R32_UEP6_TX_DMA       (*((PUINT32V)0x40030054)) // RW, USB_high_speed end point6 begin adress of DMA transmit buffer register
#define R32_UEP7_TX_DMA       (*((PUINT32V)0x40030058)) // RW, USB_high_speed end point7 begin adress of DMA transmit buffer register
#define UEPn_TX_DMA            0xFFFFFF

#define R32_UEP0_MAX_LEN      (*((PUINT32V)0x4003005C)) // RW, USB_high_speed end point0 max length package register
#define R32_UEP1_MAX_LEN      (*((PUINT32V)0x40030060)) // RW, USB_high_speed end point1 max length package register
#define R32_UEP2_MAX_LEN      (*((PUINT32V)0x40030064)) // RW, USB_high_speed end point2 max length package register
#define R32_UEP3_MAX_LEN      (*((PUINT32V)0x40030068)) // RW, USB_high_speed end point3 max length package register
#define R32_UEP4_MAX_LEN      (*((PUINT32V)0x4003006C)) // RW, USB_high_speed end point4 max length package register
#define R32_UEP5_MAX_LEN      (*((PUINT32V)0x40030070)) // RW, USB_high_speed end point5 max length package register
#define R32_UEP6_MAX_LEN      (*((PUINT32V)0x40030074)) // RW, USB_high_speed end point6 max length package register
#define R32_UEP7_MAX_LEN      (*((PUINT32V)0x40030078)) // RW, USB_high_speed end point7 max length package register
#define UEPn_MAX_LEN           0x007F

#define R16_UEP0_RX_LEN       (*((PUINT16V)0x4003007C)) // RW, USB_high_speed end point0 length of receive register
#define UEP0_RX_LEN            0x007F
#define R16_UEP1_RX_LEN       (*((PUINT16V)0x40030080)) // RW, USB_high_speed end point1 single received length register
#define R16_UEP1_R_SIZE       (*((PUINT16V)0x40030082)) // RW, USB_high_speed end point1 total received length register
#define R16_UEP2_RX_LEN       (*((PUINT16V)0x40030084)) // RW, USB_high_speed end point2 single received length register
#define R16_UEP2_R_SIZE       (*((PUINT16V)0x40030086)) // RW, USB_high_speed end point2 total received length register
#define R16_UEP3_RX_LEN       (*((PUINT16V)0x40030088)) // RW, USB_high_speed end point3 single received length register
#define R16_UEP3_R_SIZE       (*((PUINT16V)0x4003008A)) // RW, USB_high_speed end point3 total received length register
#define R16_UEP4_RX_LEN       (*((PUINT16V)0x4003008C)) // RW, USB_high_speed end point4 single received length register
#define R16_UEP4_R_SIZE       (*((PUINT16V)0x4003008E)) // RW, USB_high_speed end point4 total received length register
#define R16_UEP5_RX_LEN       (*((PUINT16V)0x40030090)) // RW, USB_high_speed end point5 single received length register
#define R16_UEP5_R_SIZE       (*((PUINT16V)0x40030092)) // RW, USB_high_speed end point5 total received length register
#define R16_UEP6_RX_LEN       (*((PUINT16V)0x40030094)) // RW, USB_high_speed end point6 single received length register
#define R16_UEP6_R_SIZE       (*((PUINT16V)0x40030096)) // RW, USB_high_speed end point6 total received length register
#define R16_UEP7_RX_LEN       (*((PUINT16V)0x40030098)) // RW, USB_high_speed end point7 single received length register
#define R16_UEP7_R_SIZE       (*((PUINT16V)0x4003009A)) // RW, USB_high_speed end point7 total received length register
#define UEPn_RX_LEN            0xFFFF
#define UEPn_R_SIZE            0xFFFF

#define R16_UEP0_T_LEN        (*((PUINT16V)0x4003009C)) // RW, USB_high_speed end point0 length of transmission register
#define UEP0_T_LEN             0x7F
#define R8_UEP0_TX_CTRL       (*((PUINT8V)0x4003009E))  // RW, USB_high_speed end point0 transmit control register
#define R8_UEP0_RX_CTRL       (*((PUINT8V)0x4003009F))  // RW, USB_high_speed end point0 receive control register

#define R16_UEP1_T_LEN       (*((PUINT16V)0x400300A0)) // RW, USB_high_speed end point1 length of transmission register
#define R8_UEP1_TX_CTRL       (*((PUINT8V)0x400300A2))  // RW, USB_high_speed end point1 transmit control register
#define R8_UEP1_RX_CTRL       (*((PUINT8V)0x400300A3))  // RW, USB_high_speed end point1 receive control register
#define R16_UEP2_T_LEN       (*((PUINT16V)0x400300A4)) // RW, USB_high_speed end point2 length of transmission register
#define R8_UEP2_TX_CTRL       (*((PUINT8V)0x400300A6))  // RW, USB_high_speed end point2 transmit control register
#define R8_UEP2_RX_CTRL       (*((PUINT8V)0x400300A7))  // RW, USB_high_speed end point2 receive control register
#define R16_UEP3_T_LEN       (*((PUINT16V)0x400300A8)) // RW, USB_high_speed end point3 length of transmission register
#define R8_UEP3_TX_CTRL       (*((PUINT8V)0x400300AA))  // RW, USB_high_speed end point3 transmit control register
#define R8_UEP3_RX_CTRL       (*((PUINT8V)0x400300AB))  // RW, USB_high_speed end point3 receive control register
#define R16_UEP4_T_LEN       (*((PUINT16V)0x400300AC)) // RW, USB_high_speed end point4 length of transmission register
#define R8_UEP4_TX_CTRL       (*((PUINT8V)0x400300AE))  // RW, USB_high_speed end point4 transmit control register
#define R8_UEP4_RX_CTRL       (*((PUINT8V)0x400300AF))  // RW, USB_high_speed end point4 receive control register
#define R16_UEP5_T_LEN       (*((PUINT16V)0x400300B0)) // RW, USB_high_speed end point5 length of transmission register
#define R8_UEP5_TX_CTRL       (*((PUINT8V)0x400300B2))  // RW, USB_high_speed end point5 transmit control register
#define R8_UEP5_RX_CTRL       (*((PUINT8V)0x400300B3))  // RW, USB_high_speed end point5 receive control register
#define R16_UEP6_T_LEN       (*((PUINT16V)0x400300B4)) // RW, USB_high_speed end point6 length of transmission register
#define R8_UEP6_TX_CTRL       (*((PUINT8V)0x400300B6))  // RW, USB_high_speed end point6 transmit control register
#define R8_UEP6_RX_CTRL       (*((PUINT8V)0x400300B7))  // RW, USB_high_speed end point6 receive control register
#define R16_UEP7_T_LEN       (*((PUINT16V)0x400300B8)) // RW, USB_high_speed end point7 length of transmission register
#define R8_UEP7_TX_CTRL       (*((PUINT8V)0x400300BA))  // RW, USB_high_speed end point7 transmit control register
#define R8_UEP7_RX_CTRL       (*((PUINT8V)0x400300BB))  // RW, USB_high_speed end point7 receive control register
/**R16_UEPn_T_LEN**/
#define UEPn_T_LEN 0xFFFF
/**R8_UEPn_TX_CTRL**/
#define USBHS_UEP_T_DONE      0x80
#define USBHS_UEP_T_NAK_ACT   0x40
#define USBHS_UEP_T_TOG_MASK  0x0C
#define USBHS_UEP_T_TOG_MDATA 0x0C
#define USBHS_UEP_T_TOG_DATA2 0x08
#define USBHS_UEP_T_TOG_DATA1 0x04
#define USBHS_UEP_T_TOG_DATA0 0x00
#define USBHS_UEP_T_RES_MASK  0x03
#define USBHS_UEP_T_RES_ACK   0x02
#define USBHS_UEP_T_RES_STALL 0x01
#define USBHS_UEP_T_RES_NAK   0x00

/**R8_UEPn_RX_CTRL**/
#define USBHS_UEP_R_DONE      0x80
#define USBHS_UEP_R_NAK_ACT   0x40
#define USBHS_UEP_R_NAK_TOG   0x20
#define USBHS_UEP_R_TOG_MATCH 0x10
#define USBHS_UEP_R_SETUP_IS  0x08
#define USBHS_UEP_R_TOG_MASK  0x0C
#define USBHS_UEP_R_TOG_MDATA 0x0C
#define USBHS_UEP_R_TOG_DATA2 0x08
#define USBHS_UEP_R_TOG_DATA1 0x04
#define USBHS_UEP_R_TOG_DATA0 0x00
#define USBHS_UEP_R_RES_MASK  0x03
#define USBHS_UEP_R_RES_ACK   0x02
#define USBHS_UEP_R_RES_STALL 0x01
#define USBHS_UEP_R_RES_NAK   0x00

#define R16_UEP_T_ISO      (*((PUINT16V)0x400300BC)) // RW, USB_high_speed end point transmit sync mode register
#define USBHS_UEP1_T_ISO_EN 0x02
#define USBHS_UEP2_T_ISO_EN 0x04
#define USBHS_UEP3_T_ISO_EN 0x08
#define USBHS_UEP4_T_ISO_EN 0x10
#define USBHS_UEP5_T_ISO_EN 0x20
#define USBHS_UEP6_T_ISO_EN 0x40
#define USBHS_UEP7_T_ISO_EN 0x80
#define R16_UEP_R_ISO      (*((PUINT16V)0x400300BE)) // RW, USB_high_speed end point receive sync mode register
#define USBHS_UEP1_R_ISO_EN 0x02
#define USBHS_UEP2_R_ISO_EN 0x04
#define USBHS_UEP3_R_ISO_EN 0x08
#define USBHS_UEP4_R_ISO_EN 0x10
#define USBHS_UEP5_R_ISO_EN 0x20
#define USBHS_UEP6_R_ISO_EN 0x40
#define USBHS_UEP7_R_ISO_EN 0x80

#define R32_UEP1_RX_FIFO   (*((PUINT32V)0x400300C0))  
#define R32_UEP2_RX_FIFO   (*((PUINT32V)0x400300C4))  
#define R32_UEP3_RX_FIFO   (*((PUINT32V)0x400300C8))  
#define R32_UEP4_RX_FIFO   (*((PUINT32V)0x400300CC))  
#define R32_UEP5_RX_FIFO   (*((PUINT32V)0x400300D0))  
#define R32_UEP6_RX_FIFO   (*((PUINT32V)0x400300D4))  
#define R32_UEP7_RX_FIFO   (*((PUINT32V)0x400300D8))  
#define RB_UEPn_RX_FIFO_END_ADDR   0xFFFF0000
#define RB_UEPn_RX_FIFO_START_ADDR   0x0000FFFF

#define R32_UEP1_TX_FIFO   (*((PUINT32V)0x400300DC))  
#define R32_UEP2_TX_FIFO   (*((PUINT32V)0x400300E0))  
#define R32_UEP3_TX_FIFO   (*((PUINT32V)0x400300E4))  
#define R32_UEP4_TX_FIFO   (*((PUINT32V)0x400300E8))  
#define R32_UEP5_TX_FIFO   (*((PUINT32V)0x400300EC))  
#define R32_UEP6_TX_FIFO   (*((PUINT32V)0x400300F0))  
#define R32_UEP7_TX_FIFO   (*((PUINT32V)0x400300F4)) 
#define RB_UEPn_TX_FIFO_END_ADDR   0xFFFF0000
#define RB_UEPn_TX_FIFO_START_ADDR   0x0000FFFF

/* USB high speed host register  */
#define R8_UH_CFG            (*((PUINT8V)0x40030100)) // RW, USB_high_speed register
#define USBHS_UH_LPM_EN       0x80
#define USBHS_UH_FORCE_FS     0x40
#define USBHS_UH_SOF_EN       0x20
#define USBHS_UH_DMA_EN       0x10
#define USBHS_UH_PHY_SUSPENDM 0x08
#define USBHS_UH_CLR_ALL      0x04
#define USBHS_RST_SIE         0x02
#define USBHS_RST_LINK        0x01
#define R8_UH_INT_EN          (*((PUINT8V)0x40030102)) // RW, USB_high_speed register
#define USBHS_UHIE_FIFO_OVER  0x80
#define USBHS_UHIE_TX_HALT    0x40
#define USBHS_UHIE_SOF_ACT    0x20
#define USBHS_UHIE_TRANSFER   0x10
#define USBHS_UHIE_RESUME_ACT 0x08
#define USBHS_UHIE_WKUP_ACT   0x04
#define R8_UH_DEV_AD         (*((PUINT8V)0x40030103)) // RW, USB_high_speed register
#define USBHS_UH_DEV_ADDR     0xFF
#define R32_UH_CONTROL       (*((PUINT32V)0x40030104)) // RW, USB_high_speed register
#define USBHS_UH_RX_NO_RES    0x800000
#define USBHS_UH_TX_NO_RES    0x400000
#define USBHS_UH_RX_NO_DATA   0x200000
#define USBHS_UH_TX_NO_DATA   0x100000
#define USBHS_UH_PRE_PID_EN   0x080000
#define USBHS_UH_SPLIT_VALID  0x040000
#define USBHS_UH_LPM_VALID    0x020000
#define USBHS_UH_HOST_ACTION  0x010000
#define USBHS_UH_BUF_MODE     0x0400
#define USBHS_UH_T_TOG_MASK   0x0300
#define USBHS_UH_T_TOG_MDATA  0x0300
#define USBHS_UH_T_TOG_DATA2  0x0200
#define USBHS_UH_T_TOG_DATA1  0x0100
#define USBHS_UH_T_TOG_DATA0  0x0000
#define USBHS_UH_T_ENDP_MASK  0xF0
#define USBHS_UH_T_TOKEN_MASK 0x0F

#define R8_UH_INT_FLAG            (*((PUINT8V)0x40030108)) // RW, USB_high_speed register
#define USBHS_UHIF_FIFO_OVER       0x80
#define USBHS_UHIF_TX_HALT         0x40
#define USBHS_UHIF_SOF_ACT         0x20
#define USBHS_UHIF_TRANSFER        0x10
#define USBHS_UHIF_RESUME_ACT      0x08
#define USBHS_UHIF_WKUP_ACT        0x04
#define R8_UH_INT_ST              (*((PUINT8V)0x40030109)) // RW, USB_high_speed register
#define USBHS_UHIF_PORT_RX_RESUME  0x10
#define USBHS_UH_R_TOKEN_MASK      0x0F
#define R8_UH_MIS_ST              (*((PUINT8V)0x4003010A)) // RW, USB_high_speed register
#define USBHS_UHMS_BUS_SE0         0x80
#define USBHS_UHMS_BUS_J           0x40
#define USBHS_UHMS_LINESTATE       0x30
#define USBHS_UHMS_USB_WAKEUP      0x08
#define USBHS_UHMS_SOF_ACT         0x04
#define USBHS_UHMS_SOF_PRE         0x02
#define USBHS_UHMS_SOF_FREE        0x01
#define R32_UH_LPM_DATA           (*((PUINT32V)0x4003010C)) // RW, USB_high_speed register
#define USBHS_UH_LPM_DATA          0x07FF
#define R32_UH_SPLIT_DATA         (*((PUINT32V)0x40030110)) // RW, USB_high_speed register
#define USBHS_UH_SPLIT_DATA        0x07FFFF
#define R32_UH_FRAME              (*((PUINT32V)0x40030114)) // RW, USB_high_speed register
#define USBHS_UH_SOF_CNT_CLR       0x02000000
#define USBHS_UH_SOF_CNT_EN        0x01000000
#define USBHS_UH_MFRAME_NO         0x070000
#define USBHS_UH_FRAME_NO          0x07FF
#define R32_UH_TX_LEN             (*((PUINT32V)0x40030118)) // RW, USB_high_speed register
#define USBHS_UH_TX_LEN            0x07FF
#define R32_UH_RX_LEN             (*((PUINT32V)0x4003011C)) // RW, USB_high_speed register
#define USBHS_UH_RX_LEN            0x07FF
#define R32_UH_RX_MAX_LEN         (*((PUINT32V)0x40030120)) // RW, USB_high_speed register
#define USBHS_UH_RX_MAX_LEN        0x07FF
#define R32_UH_RX_DMA             (*((PUINT32V)0x40030124)) // RW, USB_high_speed register
#define USBHS_R32_UH_RX_DMA        0x01FFFF
#define R32_UH_TX_DMA             (*((PUINT32V)0x40030128)) // RW, USB_high_speed register
#define USBHS_R32_UH_TX_DMA        0x01FFFF
#define R32_UH_PORT_CTRL          (*((PUINT32V)0x4003012C)) // RW, USB_high_speed register
#define USBHS_UH_BUS_RST_LONG      0x010000
#define USBHS_UH_PORT_SLEEP_BESL   0xF000
#define USBHS_UH_CLR_PORT_SLEEP    0x0100
#define USBHS_UH_CLR_PORT_CONNECT  0x20
#define USBHS_UH_CLR_PORT_EN       0x10
#define USBHS_UH_SET_PORT_SLEEP    0x08
#define USBHS_UH_CLR_PORT_SUSP     0x04
#define USBHS_UH_SET_PORT_SUSP     0x02
#define USBHS_UH_SET_PORT_RESET    0x01
#define R8_UH_PORT_CFG            (*((PUINT8V)0x40030130)) // RW, USB_high_speed register
#define USBHS_UH_PD_EN             0x80
#define USBHS_UH_HOST_EN           0x01
#define R8_UH_PORT_INT_EN         (*((PUINT8V)0x40030132)) // RW, USB_high_speed register
#define USBHS_UHIE_PORT_SLP        0x20
#define USBHS_UHIE_PORT_RESET      0x10
#define USBHS_UHIE_PORT_SUSP       0x04
#define USBHS_UHIE_PORT_EN         0x02
#define USBHS_UHIE_PORT_CONNECT    0x01
#define R8_UH_PORT_TEST_CT        (*((PUINT8V)0x40030133)) // RW, USB_high_speed register
#define USBHS_UH_TEST_FORCE_EN     0x04
#define USBHS_UH_TEST_K            0x02
#define USBHS_UH_TEST_J            0x01
#define R16_UH_PORT_ST            (*((PUINT16V)0x40030134)) // RW, USB_high_speed register
#define USBHS_UHIS_PORT_TEST       0x0800
#define USBHS_UHIS_PORT_SPEED_MASK 0x0600
#define USBHS_UHIS_PORT_HS         0x0400
#define USBHS_UHIS_PORT_LS         0x0200
#define USBHS_UHIS_PORT_FS         0x0000
#define USBHS_UHIS_PORT_SLP        0x20
#define USBHS_UHIS_PORT_RST        0x10
#define USBHS_UHIS_PORT_SUSP       0x04
#define USBHS_UHIS_PORT_EN         0x02
#define USBHS_UHIS_PORT_CONNECT    0x01
#define R8_UH_PORT_CHG            (*((PUINT8V)0x40030136))
#define USBHS_UHIF_PORT_SLP        0x20
#define USBHS_UHIF_PORT_RESET      0x10
#define USBHS_UHIF_PORT_SUSP       0x04
#define USBHS_UHIF_PORT_EN         0x02
#define USBHS_UHIF_PORT_CONNECT    0x01
#define R32_UH_BC_CTRL            (*((PUINT32V)0x4003013C))
#define UDM_VSRC_ACT               0x0400
#define UDM_BC_VSRC                0x0200
#define UDP_BC_VSRC                0x0100
#define BC_AUTO_MODE               0x40
#define UDM_BC_CMPE                0x20
#define UDP_BC_CMPE                0x10
#define UDM_BC_CMPO                0x02
#define UDP_BC_CMPO                0x01


#define __IO volatile /* defines 'read / write' permissions */

typedef struct
{
    __IO uint8_t CONTROL;     /* 0x40030000 */
    __IO uint8_t BASE_MODE;   /* 0x40030001 */
    __IO uint8_t INT_EN;      /* 0x40030002 */
    __IO uint8_t DEV_AD;      /* 0x40030003 */
    __IO uint8_t WAKE_CTRL;   /* 0x40030004 */
    __IO uint8_t TEST_MODE;   /* 0x40030005 */
    __IO uint16_t LPM_DATA;   /* 0x40030006 */

    __IO uint8_t INT_FG;      /* 0x40030008 */
    __IO uint8_t INT_ST;      /* 0x40030009 */
    __IO uint8_t MIS_ST;      /* 0x4003000a */
    __IO uint8_t RESERVE0;    /* 0x4003000b */

    __IO uint16_t FRAME_NO;   /* 0x4003000c */
    __IO uint16_t USB_BUS;    /* 0x4003000e */

    __IO uint16_t UEP_TX_EN;  /* 0x40030010 */
    __IO uint16_t UEP_RX_EN;  /* 0x40030012 */
    __IO uint16_t UEP_T_TOG_AUTO;  /* 0x40030014 */
    __IO uint16_t UEP_R_TOG_AUTO;  /* 0x40030016 */

    __IO uint8_t UEP_T_BURST;      /* 0x40030018 */
    __IO uint8_t UEP_T_BURST_MODE; /* 0x40030019 */
    __IO uint8_t UEP_R_BURST;      /* 0x4003001a */
    __IO uint8_t UEP_R_RES_MODE;   /* 0x4003001b */

    __IO uint32_t UEP_AF_MODE;     /* 0x4003001c */
    __IO uint32_t UEP0_DMA;    /* 0x40030020 */
    __IO uint32_t UEP1_RX_DMA; /* 0x40030024 */
    __IO uint32_t UEP2_RX_DMA; /* 0x40030028 */
    __IO uint32_t UEP3_RX_DMA; /* 0x4003002c */
    __IO uint32_t UEP4_RX_DMA; /* 0x40030030 */
    __IO uint32_t UEP5_RX_DMA; /* 0x40030034 */
    __IO uint32_t UEP6_RX_DMA; /* 0x40030038 */
    __IO uint32_t UEP7_RX_DMA; /* 0x4003003c */
    __IO uint32_t UEP1_TX_DMA; /* 0x40030040 */
    __IO uint32_t UEP2_TX_DMA; /* 0x40030044 */
    __IO uint32_t UEP3_TX_DMA; /* 0x40030048 */
    __IO uint32_t UEP4_TX_DMA; /* 0x4003004c */
    __IO uint32_t UEP5_TX_DMA; /* 0x40030050 */
    __IO uint32_t UEP6_TX_DMA; /* 0x40030054 */
    __IO uint32_t UEP7_TX_DMA; /* 0x40030058 */
    __IO uint32_t UEP0_MAX_LEN; /* 0x4003005c */
    __IO uint32_t UEP1_MAX_LEN; /* 0x40030060 */
    __IO uint32_t UEP2_MAX_LEN; /* 0x40030064 */
    __IO uint32_t UEP3_MAX_LEN; /* 0x40030068 */
    __IO uint32_t UEP4_MAX_LEN; /* 0x4003006c */
    __IO uint32_t UEP5_MAX_LEN; /* 0x40030070 */
    __IO uint32_t UEP6_MAX_LEN; /* 0x40030074 */
    __IO uint32_t UEP7_MAX_LEN; /* 0x40030078 */

    __IO uint16_t USB_EP0_RX_LEN; /* 0x4003007c */
    __IO uint16_t RESERVE1;       /* 0x4003007e */
    __IO uint16_t UEP1_RX_LEN;    /* 0x40030080 */
    __IO uint16_t UEP1_R_SIZE;    /* 0x40030082 */
    __IO uint16_t UEP2_RX_LEN;    /* 0x40030084 */
    __IO uint16_t UEP2_R_SIZE;    /* 0x40030086 */
    __IO uint16_t UEP3_RX_LEN;    /* 0x40030088 */
    __IO uint16_t UEP3_R_SIZE;    /* 0x4003008a */
    __IO uint16_t UEP4_RX_LEN;    /* 0x4003008c */
    __IO uint16_t UEP4_R_SIZE;    /* 0x4003008e */
    __IO uint16_t UEP5_RX_LEN;    /* 0x40030090 */
    __IO uint16_t UEP5_R_SIZE;    /* 0x40030092 */
    __IO uint16_t UEP6_RX_LEN;    /* 0x40030094 */
    __IO uint16_t UEP6_R_SIZE;    /* 0x40030096 */
    __IO uint16_t UEP7_RX_LEN;    /* 0x40030098 */
    __IO uint16_t UEP7_R_SIZE;    /* 0x4003009a */

    __IO uint16_t UEP0_TX_LEN; /* 0x4003009c */
    __IO uint8_t UEP0_TX_CTRL; /* 0x4003009e */
    __IO uint8_t UEP0_RX_CTRL; /* 0x4003009f */
    __IO uint16_t UEP1_TX_LEN; /* 0x400300a0 */
    __IO uint8_t UEP1_TX_CTRL; /* 0x400300a2 */
    __IO uint8_t UEP1_RX_CTRL; /* 0x400300a3 */
    __IO uint16_t UEP2_TX_LEN; /* 0x400300a4 */
    __IO uint8_t UEP2_TX_CTRL; /* 0x400300a6 */
    __IO uint8_t UEP2_RX_CTRL; /* 0x400300a7 */
    __IO uint16_t UEP3_TX_LEN; /* 0x400300a8 */
    __IO uint8_t UEP3_TX_CTRL; /* 0x400300aa */
    __IO uint8_t UEP3_RX_CTRL; /* 0x400300ab */
    __IO uint16_t UEP4_TX_LEN; /* 0x400300ac */
    __IO uint8_t UEP4_TX_CTRL; /* 0x400300ae */
    __IO uint8_t UEP4_RX_CTRL; /* 0x400300af */
    __IO uint16_t UEP5_TX_LEN; /* 0x400300b0 */
    __IO uint8_t UEP5_TX_CTRL; /* 0x400300b2 */
    __IO uint8_t UEP5_RX_CTRL; /* 0x400300b3 */
    __IO uint16_t UEP6_TX_LEN; /* 0x400300b4 */
    __IO uint8_t UEP6_TX_CTRL; /* 0x400300b6 */
    __IO uint8_t UEP6_RX_CTRL; /* 0x400300b7 */
    __IO uint16_t UEP7_TX_LEN; /* 0x400300b8 */
    __IO uint8_t UEP7_TX_CTRL; /* 0x400300ba */
    __IO uint8_t UEP7_RX_CTRL; /* 0x400300bb */

    __IO uint16_t UEP_TX_ISO;  /* 0x400300bc */
    __IO uint16_t UEP_RX_ISO;  /* 0x400300be */

    __IO uint32_t UEP1_RX_FIFO; /* 0x400300C0 */
    __IO uint32_t UEP2_RX_FIFO; /* 0x400300C4 */
    __IO uint32_t UEP3_RX_FIFO; /* 0x400300C8 */
    __IO uint32_t UEP4_RX_FIFO; /* 0x400300CC */
    __IO uint32_t UEP5_RX_FIFO; /* 0x400300D0 */
    __IO uint32_t UEP6_RX_FIFO; /* 0x400300D4 */
    __IO uint32_t UEP7_RX_FIFO; /* 0x400300D8 */
    __IO uint32_t UEP1_TX_FIFO; /* 0x400300DC */
    __IO uint32_t UEP2_TX_FIFO; /* 0x400300E0 */
    __IO uint32_t UEP3_TX_FIFO; /* 0x400300E4 */
    __IO uint32_t UEP4_TX_FIFO; /* 0x400300E8 */
    __IO uint32_t UEP5_TX_FIFO; /* 0x400300EC */
    __IO uint32_t UEP6_TX_FIFO; /* 0x400300F0 */
    __IO uint32_t UEP7_TX_FIFO; /* 0x400300F4 */

} USBHSD_TypeDef;



/* USBHS Host Registers */
typedef struct  __attribute__((packed))
{
  __IO uint8_t  CFG;
  uint8_t  RESERVED0;
  __IO uint8_t  INT_EN;
  __IO uint8_t  DEV_ADDR;
  __IO uint32_t CONTROL;

  __IO uint8_t  INT_FLAG;
  __IO uint8_t  INT_ST;
  __IO uint8_t  MIS_ST;
  uint8_t  RESERVED1;

  __IO uint32_t LPM;
  __IO uint32_t SPLIT;
  __IO uint32_t FRAME;
  __IO uint32_t TX_LEN;
  __IO uint32_t RX_LEN;
  __IO uint32_t RX_MAX_LEN;
  __IO uint32_t RX_DMA;
  __IO uint32_t TX_DMA;
  __IO uint32_t PORT_CTRL;
  __IO uint8_t  PORT_CFG;
  uint8_t  RESERVED2;
  __IO uint8_t  PORT_INT_EN;
  __IO uint8_t  PORT_TEST_CT;

  __IO uint16_t PORT_STATUS;
  __IO uint8_t  PORT_STATUS_CHG;
  uint8_t  RESERVED3[5];
  __IO uint32_t ROOT_BC_CTRL;
} USBHSH_TypeDef;

typedef void (*usb_rxsof_handler_t)(void);

extern usb_rxsof_handler_t usb_rxsof_handler;


