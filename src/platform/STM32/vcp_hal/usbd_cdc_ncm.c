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
 * minimal cdc-ncm device class for the st usb device library, used by phone-config mode. the
 * descriptors are standards-compliant (iad, data interface alt0 empty / alt1 bulk, ncm functional
 * descriptor, mac string) and the ntb framing follows ncm 1.0 (cross-checked against tinyusb's ncm.h).
 * a single static instance is enough since only one usb device is ever active.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// UNUSED comes from the hal (the h7 hal defines it unguarded, so common/utils.h must stay out),
// MIN from common/maths.h via usbd_conf.h

#include "usbd_core.h"
#include "usbd_ctlreq.h"

#include "usbd_cdc_ncm.h"

// ncm class-specific request codes (ncm 1.0, table 6.2)
#define NCM_SET_ETHERNET_PACKET_FILTER   0x43U
#define NCM_GET_NTB_PARAMETERS           0x80U
#define NCM_GET_NTB_INPUT_SIZE           0x85U
#define NCM_SET_NTB_INPUT_SIZE           0x86U
#define NCM_GET_NTB_FORMAT               0x83U
#define NCM_SET_NTB_FORMAT               0x84U

#define NTH16_SIGNATURE        0x484D434EU   // "NCMH"
#define NDP16_SIGNATURE        0x304D434EU   // "NCM0" (datagram pointer table, no crc)
#define NDP16_SIGNATURE_NCM1   0x314D434EU   // "NCM1" (with crc), also accepted on rx

// ntb structures (ncm 1.0)
typedef struct __attribute__((packed)) {
    uint16_t wLength;
    uint16_t bmNtbFormatsSupported;
    uint32_t dwNtbInMaxSize;
    uint16_t wNdbInDivisor;
    uint16_t wNdbInPayloadRemainder;
    uint16_t wNdbInAlignment;
    uint16_t wReserved;
    uint32_t dwNtbOutMaxSize;
    uint16_t wNdbOutDivisor;
    uint16_t wNdbOutPayloadRemainder;
    uint16_t wNdbOutAlignment;
    uint16_t wNtbOutMaxDatagrams;
} ntb_parameters_t;

typedef struct __attribute__((packed)) {
    uint32_t dwSignature;
    uint16_t wHeaderLength;
    uint16_t wSequence;
    uint16_t wBlockLength;
    uint16_t wNdpIndex;
} nth16_t;

typedef struct __attribute__((packed)) {
    uint16_t wDatagramIndex;
    uint16_t wDatagramLength;
} ndp16_datagram_t;

typedef struct __attribute__((packed)) {
    uint32_t dwSignature;
    uint16_t wLength;
    uint16_t wNextNdpIndex;
} ndp16_t;

static __ALIGN_BEGIN const ntb_parameters_t ntbParameters __ALIGN_END = {
    .wLength                 = sizeof(ntb_parameters_t),
    .bmNtbFormatsSupported   = 0x0001,        // 16-bit NTB only
    .dwNtbInMaxSize          = NCM_NTB_IN_MAX_SIZE,
    .wNdbInDivisor           = 1,
    .wNdbInPayloadRemainder  = 0,
    .wNdbInAlignment         = 4,
    .wReserved               = 0,
    .dwNtbOutMaxSize         = NCM_NTB_OUT_MAX_SIZE,
    .wNdbOutDivisor          = 1,
    .wNdbOutPayloadRemainder = 0,
    .wNdbOutAlignment        = 4,
    .wNtbOutMaxDatagrams     = NCM_OUT_MAX_DATAGRAMS,
};

// configuration descriptor (full speed)
#define NCM_CONFIG_DESC_SIZE  94U

static __ALIGN_BEGIN uint8_t ncmConfigDesc[NCM_CONFIG_DESC_SIZE] __ALIGN_END = {
    // Configuration descriptor
    0x09, USB_DESC_TYPE_CONFIGURATION, LOBYTE(NCM_CONFIG_DESC_SIZE), HIBYTE(NCM_CONFIG_DESC_SIZE),
    0x02,                       // bNumInterfaces
    0x01,                       // bConfigurationValue
    0x00,                       // iConfiguration
    0xC0,                       // bmAttributes: self powered
    0x32,                       // bMaxPower: 100 mA

    // Interface Association Descriptor (NCM function = control + data interface)
    0x08, 0x0B, NCM_NOTIF_ITF, 0x02, 0x02, 0x0D, 0x00, 0x00,

    // Communication (control) interface
    0x09, USB_DESC_TYPE_INTERFACE, NCM_NOTIF_ITF, 0x00, 0x01, 0x02, 0x0D, 0x00, 0x00,
    // CDC Header functional descriptor
    0x05, 0x24, 0x00, 0x10, 0x01,
    // CDC Union functional descriptor (master = control itf, slave = data itf)
    0x05, 0x24, 0x06, NCM_NOTIF_ITF, NCM_DATA_ITF,
    // CDC Ethernet Networking functional descriptor
    0x0D, 0x24, 0x0F, NCM_MAC_STRING_INDEX, 0x00, 0x00, 0x00, 0x00,
    LOBYTE(1514), HIBYTE(1514),  // wMaxSegmentSize = 1514
    0x00, 0x00,                  // wNumberMCFilters
    0x00,                        // bNumberPowerFilters
    // CDC NCM functional descriptor
    0x06, 0x24, 0x1A, 0x00, 0x01, 0x00,   // bcdNcmVersion 1.00, bmNetworkCapabilities 0

    // Notification endpoint (interrupt IN)
    0x07, USB_DESC_TYPE_ENDPOINT, NCM_NOTIF_EP, 0x03,
    LOBYTE(NCM_NOTIF_PACKET_SIZE), HIBYTE(NCM_NOTIF_PACKET_SIZE), NCM_FS_BINTERVAL,

    // Data interface, alternate setting 0 (no endpoints, network idle)
    0x09, USB_DESC_TYPE_INTERFACE, NCM_DATA_ITF, 0x00, 0x00, 0x0A, 0x00, 0x01, 0x00,

    // Data interface, alternate setting 1 (two bulk endpoints, network active)
    0x09, USB_DESC_TYPE_INTERFACE, NCM_DATA_ITF, 0x01, 0x02, 0x0A, 0x00, 0x01, 0x00,
    // Bulk OUT endpoint
    0x07, USB_DESC_TYPE_ENDPOINT, NCM_OUT_EP, 0x02,
    LOBYTE(NCM_DATA_FS_PACKET_SIZE), HIBYTE(NCM_DATA_FS_PACKET_SIZE), 0x00,
    // Bulk IN endpoint
    0x07, USB_DESC_TYPE_ENDPOINT, NCM_IN_EP, 0x02,
    LOBYTE(NCM_DATA_FS_PACKET_SIZE), HIBYTE(NCM_DATA_FS_PACKET_SIZE), 0x00,
};

static __ALIGN_BEGIN uint8_t ncmDeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
    USB_LEN_DEV_QUALIFIER_DESC, USB_DESC_TYPE_DEVICE_QUALIFIER, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x40, 0x01, 0x00,
};

// class instance state
typedef enum { NOTIFY_IDLE = 0, NOTIFY_SPEED, NOTIFY_CONNECTION, NOTIFY_DONE } ncmNotifyState_e;

typedef struct {
    uint8_t  dataAltSet;        // 0 = idle, 1 = active
    uint8_t  linkUp;
    ncmNotifyState_e notifyState;
    uint8_t  notifyBusy;
    uint8_t  cmdOpCode;         // pending EP0 OUT class request (0xFF = none)
    uint16_t cmdLength;
    uint8_t  txBusy;
    uint16_t txSeq;
    uint16_t txLastLen;              // length of the last bulk-in transfer, for zlp termination
    volatile uint8_t  rxReady;       // an ntb arrived, set in irq and cleared in the main loop
    volatile uint32_t rxLen;
    volatile uint8_t  linkPending;   // deferred link-state change for the glue layer
    volatile uint8_t  notifyPending; // deferred link-up notification (sent from the main loop)
    USBD_CDC_NCM_ItfTypeDef *fops;
    __ALIGN_BEGIN uint8_t ctrlBuf[64] __ALIGN_END;
    __ALIGN_BEGIN uint8_t notifBuf[16] __ALIGN_END;
    __ALIGN_BEGIN uint8_t rxNtb[NCM_NTB_OUT_MAX_SIZE] __ALIGN_END;
    __ALIGN_BEGIN uint8_t txNtb[NCM_NTB_IN_MAX_SIZE] __ALIGN_END;
} ncmHandle_t;

static ncmHandle_t ncm;

// forward declarations of the class callbacks
static uint8_t ncmInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t ncmDeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t ncmSetup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t ncmEP0RxReady(USBD_HandleTypeDef *pdev);
static uint8_t ncmDataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t ncmDataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t *ncmGetCfgDesc(uint16_t *length);
static uint8_t *ncmGetDeviceQualifierDesc(uint16_t *length);
#if (USBD_SUPPORT_USER_STRING == 1U)
static uint8_t *ncmGetUsrStrDesc(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length);
#endif

USBD_ClassTypeDef USBD_CDC_NCM = {
    ncmInit,
    ncmDeInit,
    ncmSetup,
    NULL,               // EP0_TxSent
    ncmEP0RxReady,
    ncmDataIn,
    ncmDataOut,
    NULL,               // SOF
    NULL,               // IsoINIncomplete
    NULL,               // IsoOUTIncomplete
    ncmGetCfgDesc,      // GetHSConfigDescriptor
    ncmGetCfgDesc,      // GetFSConfigDescriptor
    ncmGetCfgDesc,      // GetOtherSpeedConfigDescriptor
    ncmGetDeviceQualifierDesc,
#if (USBD_SUPPORT_USER_STRING == 1U)
    ncmGetUsrStrDesc,
#endif
};

// notification chain (speed change then network connection)
static void ncmSendNextNotification(USBD_HandleTypeDef *pdev, bool forceNext)
{
    if (!forceNext && ncm.notifyBusy) {
        return;
    }

    if (ncm.notifyState == NOTIFY_SPEED) {
        // CONNECTION_SPEED_CHANGE: 8-byte header + 8-byte payload (down/up bit/s)
        uint8_t *b = ncm.notifBuf;
        b[0] = 0xA1; b[1] = NCM_NOTIFY_CONNECTION_SPEED;
        b[2] = 0x00; b[3] = 0x00;                 // wValue
        b[4] = NCM_NOTIF_ITF; b[5] = 0x00;        // wIndex
        b[6] = 0x08; b[7] = 0x00;                 // wLength = 8
        const uint32_t speed = 12000000U;         // 12 Mbit/s (full speed)
        memcpy(&b[8], &speed, 4);
        memcpy(&b[12], &speed, 4);
        if (USBD_LL_Transmit(pdev, NCM_NOTIF_EP, ncm.notifBuf, 16) == USBD_OK) {
            ncm.notifyState = NOTIFY_CONNECTION;
            ncm.notifyBusy = 1;
        } else {
            ncm.notifyBusy = 0;   // failed submit means no completion irq, don't wedge the chain
        }
    } else if (ncm.notifyState == NOTIFY_CONNECTION) {
        // NETWORK_CONNECTION: 8-byte header, wValue = link state
        uint8_t *b = ncm.notifBuf;
        b[0] = 0xA1; b[1] = NCM_NOTIFY_NETWORK_CONNECTION;
        b[2] = ncm.linkUp ? 0x01 : 0x00; b[3] = 0x00;
        b[4] = NCM_NOTIF_ITF; b[5] = 0x00;
        b[6] = 0x00; b[7] = 0x00;
        if (USBD_LL_Transmit(pdev, NCM_NOTIF_EP, ncm.notifBuf, 8) == USBD_OK) {
            ncm.notifyState = NOTIFY_DONE;
            ncm.notifyBusy = 1;
        } else {
            ncm.notifyBusy = 0;
        }
    } else {
        ncm.notifyBusy = 0;
    }
}

void USBD_CDC_NCM_SetLinkState(USBD_HandleTypeDef *pdev, uint8_t up)
{
    ncm.linkUp = up ? 1 : 0;
    ncm.notifyState = NOTIFY_SPEED;
    ncmSendNextNotification(pdev, false);
    ncm.linkPending = 1;   // defer the lwip link-change to the main loop
}

// class callbacks
static uint8_t ncmInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    UNUSED(cfgidx);

    USBD_LL_OpenEP(pdev, NCM_NOTIF_EP, USBD_EP_TYPE_INTR, NCM_NOTIF_PACKET_SIZE);
    pdev->ep_in[NCM_NOTIF_EP & 0xFU].is_used = 1U;

    ncm.dataAltSet = 0;
    ncm.linkUp = 0;
    ncm.notifyState = NOTIFY_IDLE;
    ncm.notifyBusy = 0;
    ncm.notifyPending = 0;
    ncm.linkPending = 0;
    ncm.txBusy = 0;
    ncm.txSeq = 0;
    ncm.txLastLen = 0;
    ncm.rxReady = 0;
    ncm.rxLen = 0;
    ncm.cmdOpCode = 0xFF;
    ncm.cmdLength = 0;

    if (ncm.fops && ncm.fops->Init) {
        ncm.fops->Init();
    }
    return USBD_OK;
}

static uint8_t ncmDeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    UNUSED(cfgidx);

    USBD_LL_CloseEP(pdev, NCM_NOTIF_EP);
    pdev->ep_in[NCM_NOTIF_EP & 0xFU].is_used = 0U;

    if (ncm.dataAltSet) {
        USBD_LL_CloseEP(pdev, NCM_IN_EP);
        USBD_LL_CloseEP(pdev, NCM_OUT_EP);
        pdev->ep_in[NCM_IN_EP & 0xFU].is_used = 0U;
        pdev->ep_out[NCM_OUT_EP & 0xFU].is_used = 0U;
        ncm.dataAltSet = 0;
    }

    if (ncm.fops && ncm.fops->DeInit) {
        ncm.fops->DeInit();
    }
    return USBD_OK;
}

static void ncmSetDataInterface(USBD_HandleTypeDef *pdev, uint8_t alt)
{
    if (alt == ncm.dataAltSet) {
        return;
    }
    if (alt == 1) {
        USBD_LL_OpenEP(pdev, NCM_IN_EP, USBD_EP_TYPE_BULK, NCM_DATA_FS_PACKET_SIZE);
        USBD_LL_OpenEP(pdev, NCM_OUT_EP, USBD_EP_TYPE_BULK, NCM_DATA_FS_PACKET_SIZE);
        pdev->ep_in[NCM_IN_EP & 0xFU].is_used = 1U;
        pdev->ep_out[NCM_OUT_EP & 0xFU].is_used = 1U;
        USBD_LL_PrepareReceive(pdev, NCM_OUT_EP, ncm.rxNtb, NCM_NTB_OUT_MAX_SIZE);
    } else {
        USBD_LL_CloseEP(pdev, NCM_IN_EP);
        USBD_LL_CloseEP(pdev, NCM_OUT_EP);
        pdev->ep_in[NCM_IN_EP & 0xFU].is_used = 0U;
        pdev->ep_out[NCM_OUT_EP & 0xFU].is_used = 0U;
        ncm.txBusy = 0;
        ncm.rxReady = 0;
    }
    ncm.dataAltSet = alt;

    if (alt == 1) {
        // defer the link-up notification to the main loop. sending it here, on the interrupt endpoint
        // mid control-transfer in the usb irq, corrupts the st usb core and hard-faults.
        if (!ncm.linkUp) {
            ncm.notifyPending = 1;
        }
    } else {
        ncm.notifyPending = 0;
        if (ncm.linkUp) {
            ncm.linkUp = 0;      // re-activation must re-notify
            ncm.linkPending = 1; // tell the glue layer the link went down
        }
    }
}

static uint8_t ncmSetup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_StatusTypeDef ret = USBD_OK;
    uint16_t len;

    // control replies are served from ncm.ctrlBuf, not a stack local: USBD_CtlSendData only stores the
    // pointer, the fifo copy is deferred to the TXFE irq, by which time a stack buffer is gone
    switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    case USB_REQ_TYPE_CLASS:
        switch (req->bRequest) {
        case NCM_GET_NTB_PARAMETERS:
            len = MIN(sizeof(ntbParameters), req->wLength);
            USBD_CtlSendData(pdev, (uint8_t *)&ntbParameters, len);
            break;

        case NCM_GET_NTB_INPUT_SIZE:
            ncm.ctrlBuf[0] = LOBYTE(NCM_NTB_IN_MAX_SIZE); ncm.ctrlBuf[1] = HIBYTE(NCM_NTB_IN_MAX_SIZE);
            ncm.ctrlBuf[2] = 0; ncm.ctrlBuf[3] = 0;
            USBD_CtlSendData(pdev, ncm.ctrlBuf, MIN(4, req->wLength));
            break;

        case NCM_GET_NTB_FORMAT:
            ncm.ctrlBuf[0] = 0x00; ncm.ctrlBuf[1] = 0x00;   // 16-bit NTB
            USBD_CtlSendData(pdev, ncm.ctrlBuf, MIN(2, req->wLength));
            break;

        case NCM_SET_NTB_INPUT_SIZE:
        case NCM_SET_NTB_FORMAT:
        case NCM_SET_ETHERNET_PACKET_FILTER:
            if (req->wLength != 0 && (req->bmRequest & 0x80U) == 0) {
                // data stage follows on EP0 OUT
                ncm.cmdOpCode = (uint8_t)req->bRequest;
                ncm.cmdLength = MIN(req->wLength, sizeof(ncm.ctrlBuf));
                USBD_CtlPrepareRx(pdev, ncm.ctrlBuf, ncm.cmdLength);
            } else {
                if (req->bRequest == NCM_SET_ETHERNET_PACKET_FILTER && !ncm.linkUp) {
                    ncm.notifyPending = 1;
                }
            }
            break;

        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
        }
        break;

    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest) {
        case USB_REQ_GET_STATUS:
            if (pdev->dev_state == USBD_STATE_CONFIGURED) {
                ncm.ctrlBuf[0] = 0; ncm.ctrlBuf[1] = 0;
                USBD_CtlSendData(pdev, ncm.ctrlBuf, 2);
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_GET_INTERFACE:
            if (pdev->dev_state == USBD_STATE_CONFIGURED) {
                ncm.ctrlBuf[0] = (req->wIndex == NCM_DATA_ITF) ? ncm.dataAltSet : 0;
                USBD_CtlSendData(pdev, ncm.ctrlBuf, 1);
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_SET_INTERFACE:
            if (pdev->dev_state == USBD_STATE_CONFIGURED) {
                if (req->wIndex == NCM_DATA_ITF) {
                    if (req->wValue <= 1) {   // the data interface only has alternates 0 and 1
                        ncmSetDataInterface(pdev, (uint8_t)req->wValue);
                    } else {
                        USBD_CtlError(pdev, req);
                        ret = USBD_FAIL;
                    }
                }
            } else {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_CLEAR_FEATURE:
            break;

        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
        }
        break;

    default:
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
        break;
    }

    return (uint8_t)ret;
}

static uint8_t ncmEP0RxReady(USBD_HandleTypeDef *pdev)
{
    UNUSED(pdev);
    if (ncm.cmdOpCode != 0xFF) {
        // the only out class requests we take are the SET_NTB_* / packet-filter ones, just ack them
        if (ncm.cmdOpCode == NCM_SET_ETHERNET_PACKET_FILTER && !ncm.linkUp) {
            ncm.notifyPending = 1;
        }
        ncm.cmdOpCode = 0xFF;
    }
    return USBD_OK;
}

static uint8_t ncmDataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if (epnum == (NCM_NOTIF_EP & 0x7FU)) {
        ncm.notifyBusy = 0;
        ncmSendNextNotification(pdev, true);
    } else if (epnum == (NCM_IN_EP & 0x7FU)) {
        // a bulk-in transfer that is a non-zero multiple of the max packet size needs a terminating
        // zlp or the host waits forever. the st usb layer doesn't auto-zlp, so issue it here.
        if (ncm.txLastLen != 0 && (ncm.txLastLen % NCM_DATA_FS_PACKET_SIZE) == 0) {
            ncm.txLastLen = 0;
            if (USBD_LL_Transmit(pdev, NCM_IN_EP, NULL, 0) != USBD_OK) {   // length 0, NULL buffer is safe
                ncm.txBusy = 0;
            }
        } else {
            ncm.txLastLen = 0;
            ncm.txBusy = 0;
        }
    }
    return USBD_OK;
}

static uint8_t ncmDataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if (epnum == NCM_OUT_EP) {
        // defer to the main loop, lwip (NO_SYS) must not be touched from the irq. the out endpoint is
        // not re-armed until USBD_CDC_NCM_Process() consumes this ntb, so the host is flow-controlled.
        ncm.rxLen = USBD_LL_GetRxDataSize(pdev, epnum);
        ncm.rxReady = 1;
    }
    return USBD_OK;
}

// called from the main loop. delivers rx datagrams to the glue layer and runs deferred link changes
void USBD_CDC_NCM_Process(USBD_HandleTypeDef *pdev)
{
    if (ncm.notifyPending) {
        ncm.notifyPending = 0;
        USBD_CDC_NCM_SetLinkState(pdev, 1);   // send link-up from the main loop, not the irq
    }

    if (ncm.linkPending) {
        ncm.linkPending = 0;
        if (ncm.fops && ncm.fops->LinkChange) {
            ncm.fops->LinkChange(ncm.linkUp);
        }
    }

    if (ncm.rxReady) {
        const uint32_t rxLen = ncm.rxLen;
        const nth16_t *nth = (const nth16_t *)ncm.rxNtb;
        // validate the whole ntb before walking it so a malformed header can't read past rxNtb.
        // all offsets are bounded by the declared block length, itself bounded by the transfer
        const uint32_t minNtb = sizeof(nth16_t) + sizeof(ndp16_t) + 2U * sizeof(ndp16_datagram_t);
        if (ncm.fops && ncm.fops->Receive &&
            rxLen >= minNtb &&
            nth->wHeaderLength == sizeof(nth16_t) &&
            nth->dwSignature == NTH16_SIGNATURE &&
            nth->wBlockLength >= minNtb &&
            nth->wBlockLength <= rxLen &&
            nth->wNdpIndex >= sizeof(nth16_t) &&
            (uint32_t)nth->wNdpIndex + sizeof(ndp16_t) + 2U * sizeof(ndp16_datagram_t) <= nth->wBlockLength) {

            const uint32_t blockLen = nth->wBlockLength;
            const ndp16_t *ndp = (const ndp16_t *)(ncm.rxNtb + nth->wNdpIndex);
            if ((ndp->dwSignature == NDP16_SIGNATURE || ndp->dwSignature == NDP16_SIGNATURE_NCM1) &&
                ndp->wLength >= sizeof(ndp16_t) + 2U * sizeof(ndp16_datagram_t) &&
                (uint32_t)nth->wNdpIndex + ndp->wLength <= blockLen) {
                // walk only the datagram pointers the ndp declares, not everything up to rxLen
                uint32_t entries = (uint32_t)(ndp->wLength - sizeof(ndp16_t)) / sizeof(ndp16_datagram_t);
                if (entries > NCM_OUT_MAX_DATAGRAMS) {
                    entries = NCM_OUT_MAX_DATAGRAMS;
                }
                const ndp16_datagram_t *dg = (const ndp16_datagram_t *)(ncm.rxNtb + nth->wNdpIndex + sizeof(ndp16_t));
                for (uint32_t i = 0; i < entries && dg[i].wDatagramIndex != 0 && dg[i].wDatagramLength != 0; i++) {
                    if ((uint32_t)dg[i].wDatagramIndex + dg[i].wDatagramLength <= blockLen) {
                        ncm.fops->Receive(ncm.rxNtb + dg[i].wDatagramIndex, dg[i].wDatagramLength);
                    }
                }
            }
        }
        ncm.rxReady = 0;
        USBD_LL_PrepareReceive(pdev, NCM_OUT_EP, ncm.rxNtb, NCM_NTB_OUT_MAX_SIZE);
    }
}

uint8_t USBD_CDC_NCM_TransmitFrame(USBD_HandleTypeDef *pdev, const uint8_t *frame, uint16_t length)
{
    if (ncm.txBusy || !ncm.dataAltSet) {
        return USBD_BUSY;
    }

    // build a single-datagram ntb: nth16 | datagram | ndp16 | datagram pointers.
    // size math in 32 bits, the offsets must not wrap before the bounds check
    const uint32_t dgOffset = sizeof(nth16_t);
    const uint32_t ndpOffset = (dgOffset + length + 3U) & ~3U;
    const uint32_t total = ndpOffset + sizeof(ndp16_t) + 2U * sizeof(ndp16_datagram_t);
    if (total > NCM_NTB_IN_MAX_SIZE) {
        return USBD_FAIL;
    }

    memset(ncm.txNtb, 0, total);

    nth16_t *nth = (nth16_t *)ncm.txNtb;
    nth->dwSignature = NTH16_SIGNATURE;
    nth->wHeaderLength = sizeof(nth16_t);
    nth->wSequence = ncm.txSeq++;
    nth->wBlockLength = (uint16_t)total;
    nth->wNdpIndex = (uint16_t)ndpOffset;

    memcpy(ncm.txNtb + dgOffset, frame, length);

    ndp16_t *ndp = (ndp16_t *)(ncm.txNtb + ndpOffset);
    ndp->dwSignature = NDP16_SIGNATURE;
    ndp->wLength = sizeof(ndp16_t) + 2U * sizeof(ndp16_datagram_t);
    ndp->wNextNdpIndex = 0;

    ndp16_datagram_t *dg = (ndp16_datagram_t *)(ncm.txNtb + ndpOffset + sizeof(ndp16_t));
    dg[0].wDatagramIndex = (uint16_t)dgOffset;
    dg[0].wDatagramLength = length;
    dg[1].wDatagramIndex = 0;
    dg[1].wDatagramLength = 0;

    ncm.txBusy = 1;
    ncm.txLastLen = (uint16_t)total;
    if (USBD_LL_Transmit(pdev, NCM_IN_EP, ncm.txNtb, (uint16_t)total) != USBD_OK) {
        ncm.txBusy = 0;      // don't wedge the tx path on a failed submit
        ncm.txLastLen = 0;
        return USBD_FAIL;
    }
    return USBD_OK;
}

uint8_t USBD_CDC_NCM_IsTxBusy(void)
{
    return ncm.txBusy;
}

static uint8_t *ncmGetCfgDesc(uint16_t *length)
{
    *length = sizeof(ncmConfigDesc);
    return ncmConfigDesc;
}

static uint8_t *ncmGetDeviceQualifierDesc(uint16_t *length)
{
    *length = sizeof(ncmDeviceQualifierDesc);
    return ncmDeviceQualifierDesc;
}

#if (USBD_SUPPORT_USER_STRING == 1U)
static uint8_t *ncmGetUsrStrDesc(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length)
{
    UNUSED(pdev);
    static __ALIGN_BEGIN uint8_t strDesc[64] __ALIGN_END;
    if (index == NCM_MAC_STRING_INDEX && ncm.fops && ncm.fops->macStringDescriptor) {
        USBD_GetString((uint8_t *)ncm.fops->macStringDescriptor, strDesc, length);
        return strDesc;
    }
    return NULL;
}
#endif

uint8_t USBD_CDC_NCM_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_CDC_NCM_ItfTypeDef *fops)
{
    UNUSED(pdev);
    if (fops == NULL) {
        return USBD_FAIL;
    }
    ncm.fops = fops;
    return USBD_OK;
}
