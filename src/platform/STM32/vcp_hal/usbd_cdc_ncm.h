/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * minimal cdc-ncm (network control model) usb device class for the st usb device library, written for
 * betaflight's phone-config mode so an iphone enumerates the fc as usb ethernet. ntb structures and
 * parameter values come from the ncm 1.0 spec and tinyusb's ncm.h.
 */

#pragma once

#include "usbd_ioreq.h"

// endpoints, same numbering as the normal vcp (only one usb class is active at a time)
#define NCM_IN_EP                    0x81U   // bulk in  (datagrams device to host)
#define NCM_OUT_EP                   0x01U   // bulk out (datagrams host to device)
#define NCM_NOTIF_EP                 0x82U   // interrupt in (notifications)

#define NCM_NOTIF_ITF                0x00U   // communication (control) interface number
#define NCM_DATA_ITF                 0x01U   // data interface number

#define NCM_NOTIF_PACKET_SIZE        16U
#define NCM_DATA_FS_PACKET_SIZE      64U
#define NCM_FS_BINTERVAL             0x10U

#define NCM_MAC_STRING_INDEX         6U

// ntb sizing, kept small since this is a low-bandwidth cli link not a nic
#define NCM_NTB_OUT_MAX_SIZE         2048U
#define NCM_NTB_IN_MAX_SIZE          2048U
#define NCM_OUT_MAX_DATAGRAMS        4U

// cdc class-specific notification codes
#define NCM_NOTIFY_NETWORK_CONNECTION   0x00U
#define NCM_NOTIFY_CONNECTION_SPEED     0x2AU

#define NCM_NET_DISCONNECTED            0x00U
#define NCM_NET_CONNECTED               0x01U

extern USBD_ClassTypeDef USBD_CDC_NCM;
#define USBD_CDC_NCM_CLASS &USBD_CDC_NCM

// interface callbacks supplied by the platform/glue layer (the network side, e.g. lwip)
typedef struct {
    int8_t (*Init)(void);
    int8_t (*DeInit)(void);
    int8_t (*Receive)(uint8_t *frame, uint32_t length);   // one ethernet datagram received from the host
    void   (*LinkChange)(uint8_t up);
    const uint8_t *macStringDescriptor;                   // 12 upper-case hex chars, e.g. "020000000001"
} USBD_CDC_NCM_ItfTypeDef;

uint8_t USBD_CDC_NCM_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_CDC_NCM_ItfTypeDef *fops);

// queue one ethernet datagram for transmission to the host (wrapped in an ntb). returns USBD_OK / USBD_BUSY.
uint8_t USBD_CDC_NCM_TransmitFrame(USBD_HandleTypeDef *pdev, const uint8_t *frame, uint16_t length);

// non-zero while a datagram (and its zlp) is still going out. lets the tx side send one frame at a time
uint8_t USBD_CDC_NCM_IsTxBusy(void);

// set the link state and (re)send the connection notifications
void USBD_CDC_NCM_SetLinkState(USBD_HandleTypeDef *pdev, uint8_t up);

// process deferred rx and link-change from the main loop (lwip must not be touched from the irq)
void USBD_CDC_NCM_Process(USBD_HandleTypeDef *pdev);
