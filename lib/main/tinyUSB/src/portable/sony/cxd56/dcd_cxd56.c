/*
 * The MIT License (MIT)
 *
 * Copyright 2019 Sony Semiconductor Solutions Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_CXD56

#include <errno.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/arch.h>

#include "device/dcd.h"
#include "device/usbd_pvt.h"

#define CXD56_EPNUM (7)
#define CXD56_SETUP_QUEUE_DEPTH (4)
#define CXD56_MAX_DATA_OUT_SIZE (64)

OSAL_QUEUE_DEF(usbd_int_set, _setup_queue_def, CXD56_SETUP_QUEUE_DEPTH, struct usb_ctrlreq_s);

struct usbdcd_driver_s
{
  struct usbdevclass_driver_s usbdevclass_driver;
  FAR struct usbdev_ep_s *ep[CXD56_EPNUM];
  FAR struct usbdev_req_s *req[CXD56_EPNUM];
  osal_queue_t setup_queue;
  bool setup_processed;
  FAR uint8_t dataout[CXD56_MAX_DATA_OUT_SIZE];
  size_t outlen;
};

static struct usbdcd_driver_s usbdcd_driver;
static struct usbdev_s *usbdev;

static int  _dcd_bind       (FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev);
static void _dcd_unbind     (FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev);
static int  _dcd_setup      (FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev,
                             FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout, size_t outlen);
static void _dcd_disconnect (FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev);
static void _dcd_suspend    (FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev);
static void _dcd_resume     (FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev);

static const struct usbdevclass_driverops_s g_driverops =
{
  _dcd_bind,       /* bind */
  _dcd_unbind,     /* unbind */
  _dcd_setup,      /* setup */
  _dcd_disconnect, /* disconnect */
  _dcd_suspend,    /* suspend */
  _dcd_resume,     /* resume */
};

static void usbdcd_ep0incomplete(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  (void) ep;

  uint8_t ep_addr = (uint32_t)req->priv;

  if (req->result || req->xfrd != req->len)
  {
    if (req->len)
    {
      dcd_event_xfer_complete(0, ep_addr, req->xfrd, XFER_RESULT_SUCCESS, true);
    }
  }
  else
  {
    if (req->xfrd)
    {
      dcd_event_xfer_complete(0, ep_addr, req->xfrd, XFER_RESULT_SUCCESS, true);
    }
  }
}

static int _dcd_bind(FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev)
{
  (void) driver;

  usbdev = dev;
  usbdcd_driver.ep[0] = dev->ep0;

  #ifdef EP_ALLOCREQ
  // SDK v2
  usbdcd_driver.req[0] = EP_ALLOCREQ(usbdcd_driver.ep[0]);
  if (usbdcd_driver.req[0] != NULL) {
    usbdcd_driver.req[0]->len = 64;
    usbdcd_driver.req[0]->buf = EP_ALLOCBUFFER(usbdcd_driver.ep[0], 64);
    if (!usbdcd_driver.req[0]->buf) {
      EP_FREEREQ(usbdcd_driver.ep[0], usbdcd_driver.req[0]);
      usbdcd_driver.req[0] = NULL;
      return ENOMEM;
    }
  }
  #else
  // SDK v3
  usbdcd_driver.req[0] = usbdev_allocreq(usbdcd_driver.ep[0], 64);
  if (usbdcd_driver.req[0] == NULL) {
    return ENOMEM;
  }
  #endif

  usbdcd_driver.req[0]->callback = usbdcd_ep0incomplete;

  DEV_CONNECT(dev);
  return 0;
}

static void _dcd_unbind(FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev)
{
  (void) driver;
  (void) dev;
}

static int _dcd_setup(FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev,
                      FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout, size_t outlen)
{
  (void) driver;
  (void) dev;

  if (usbdcd_driver.setup_processed)
  {
    usbdcd_driver.setup_processed = false;
    dcd_event_setup_received(0, (uint8_t const *) ctrl, true);
  }
  else
  {
    osal_queue_send(usbdcd_driver.setup_queue, ctrl, true);
  }

  if (outlen > 0 && outlen <= CXD56_MAX_DATA_OUT_SIZE)
  {
    memcpy(usbdcd_driver.dataout, dataout, outlen);
    usbdcd_driver.outlen = outlen;
  }

  return 0;
}

static void _dcd_disconnect(FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev)
{
  (void) driver;

  tusb_speed_t speed;

  switch (dev->speed)
  {
    case USB_SPEED_LOW:
      speed = TUSB_SPEED_LOW;
      break;
    case USB_SPEED_FULL:
      speed = TUSB_SPEED_FULL;
      break;
    case USB_SPEED_HIGH:
      speed = TUSB_SPEED_HIGH;
      break;
    default:
      speed = TUSB_SPEED_HIGH;
      break;
  }

  dcd_event_bus_reset(0, speed, true);
  DEV_CONNECT(dev);
}

static void _dcd_suspend(FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev)
{
  (void) driver;
  (void) dev;

  dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
}

static void _dcd_resume(FAR struct usbdevclass_driver_s *driver, FAR struct usbdev_s *dev)
{
  (void) driver;
  (void) dev;

  dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
}

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;

  usbdcd_driver.usbdevclass_driver.speed = USB_SPEED_HIGH;
  usbdcd_driver.usbdevclass_driver.ops = &g_driverops;
  usbdcd_driver.setup_processed = true;
  usbdcd_driver.setup_queue = osal_queue_create(&_setup_queue_def);

  usbdev_register(&usbdcd_driver.usbdevclass_driver);

  return true;
}

// Enable device interrupt
void dcd_int_enable(uint8_t rhport)
{
  (void) rhport;

  up_enable_irq(CXD56_IRQ_USB_INT);
}

// Disable device interrupt
void dcd_int_disable(uint8_t rhport)
{
  (void) rhport;

  up_disable_irq(CXD56_IRQ_USB_INT);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;
  (void) dev_addr;
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;

  DEV_WAKEUP(usbdev);
}

void dcd_connect(uint8_t rhport)
{
  (void) rhport;
  DEV_CONNECT(usbdev);
}

void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  DEV_DISCONNECT(usbdev);
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *p_endpoint_desc)
{
  (void) rhport;

  uint8_t epnum = tu_edpt_number(p_endpoint_desc->bEndpointAddress);
  uint8_t const dir = tu_edpt_dir(p_endpoint_desc->bEndpointAddress);
  uint8_t xfrtype = 0;
  uint16_t const ep_mps = tu_edpt_packet_size(p_endpoint_desc);

  struct usb_epdesc_s epdesc;

  if (epnum >= CXD56_EPNUM)
  {
    return false;
  }

  switch (p_endpoint_desc->bmAttributes.xfer)
  {
  case 1:
    xfrtype = USB_EP_ATTR_XFER_ISOC;
    break;
  case 2:
    xfrtype = USB_EP_ATTR_XFER_BULK;
    break;
  case 3:
    xfrtype = USB_EP_ATTR_XFER_INT;
    break;
  }

  usbdcd_driver.ep[epnum] = DEV_ALLOCEP(usbdev, epnum, dir == TUSB_DIR_IN, xfrtype);
  if (usbdcd_driver.ep[epnum] == NULL)
  {
    return false;
  }

  usbdcd_driver.req[epnum] = NULL;

  #ifdef EP_ALLOCREQ
  // sdk v2
  usbdcd_driver.req[epnum] = EP_ALLOCREQ(usbdcd_driver.ep[epnum]);
  if (usbdcd_driver.req[epnum] != NULL) {
    usbdcd_driver.req[epnum]->len = ep_mps;
  }
  #else
  // sdk v3
  usbdcd_driver.req[epnum] = usbdev_allocreq(usbdcd_driver.ep[epnum], ep_mps);
  #endif

  if(usbdcd_driver.req[epnum] == NULL) {
    return false;
  }

  usbdcd_driver.req[epnum]->callback = usbdcd_ep0incomplete;

  epdesc.len = p_endpoint_desc->bLength;
  epdesc.type = p_endpoint_desc->bDescriptorType;
  epdesc.addr = p_endpoint_desc->bEndpointAddress;
  epdesc.attr = xfrtype;
  epdesc.mxpacketsize[0] = LSBYTE(ep_mps);
  epdesc.mxpacketsize[1] = MSBYTE(ep_mps);
  epdesc.interval = p_endpoint_desc->bInterval;

  if (EP_CONFIGURE(usbdcd_driver.ep[epnum], &epdesc, false) < 0)
  {
    return false;
  }

  return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  // TODO implement dcd_edpt_close_all()
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
  (void) rhport;

  bool ret = true;
  uint8_t epnum = tu_edpt_number(ep_addr);

  if (epnum >= CXD56_EPNUM)
  {
    return false;
  }

  if (epnum == 0)
  {
    if (total_bytes == 0)
    {
      usbdcd_driver.setup_processed = true;
      dcd_event_xfer_complete(0, ep_addr, 0, XFER_RESULT_SUCCESS, false);
    }
    else if (ep_addr == 0x00 && total_bytes == usbdcd_driver.outlen)
    {
      memcpy(buffer, usbdcd_driver.dataout, usbdcd_driver.outlen);
      dcd_event_xfer_complete(0, ep_addr, total_bytes, XFER_RESULT_SUCCESS, false);
      usbdcd_driver.outlen = 0;
    }
    else
    {
      usbdcd_driver.req[epnum]->len = total_bytes;
      usbdcd_driver.req[epnum]->priv = (void *)((uint32_t)ep_addr);
      usbdcd_driver.req[epnum]->flags = total_bytes < usbdcd_driver.ep[epnum]->maxpacket ? USBDEV_REQFLAGS_NULLPKT : 0;
      usbdcd_driver.req[epnum]->buf = buffer;

      if (EP_SUBMIT(usbdcd_driver.ep[epnum], usbdcd_driver.req[epnum]) < 0)
      {
        ret = false;
      }
    }

    struct usb_ctrlreq_s ctrl;

    if (usbdcd_driver.setup_processed)
    {
      if (osal_queue_receive(usbdcd_driver.setup_queue, &ctrl, 100))
      {
        usbdcd_driver.setup_processed = false;
        dcd_event_setup_received(0, (uint8_t *)&ctrl, false);
      }
    }
  }
  else
  {
    usbdcd_driver.req[epnum]->len = total_bytes;
    usbdcd_driver.req[epnum]->priv = (void *)((uint32_t)ep_addr);
    usbdcd_driver.req[epnum]->flags = total_bytes < usbdcd_driver.ep[epnum]->maxpacket ? USBDEV_REQFLAGS_NULLPKT : 0;
    usbdcd_driver.req[epnum]->buf = buffer;

    if (EP_SUBMIT(usbdcd_driver.ep[epnum], usbdcd_driver.req[epnum]) < 0)
    {
      ret = false;
    }
  }

  return ret;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t epnum = tu_edpt_number(ep_addr);

  if (epnum >= CXD56_EPNUM)
  {
    return;
  }

  EP_STALL(usbdcd_driver.ep[epnum]);
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t epnum = tu_edpt_number(ep_addr);

  if (epnum >= CXD56_EPNUM)
  {
    return;
  }

  EP_RESUME(usbdcd_driver.ep[epnum]);
}

#endif
