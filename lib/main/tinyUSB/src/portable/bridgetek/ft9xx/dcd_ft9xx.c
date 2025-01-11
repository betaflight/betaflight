/*
 * The MIT License (MIT)
 *
 * Copyright 2021 Bridgetek Pte Ltd
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

/*
 * Contains code adapted from Bridgetek Pte Ltd via license terms stated
 * in https://brtchip.com/BRTSourceCodeLicenseAgreement
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && \
  (CFG_TUSB_MCU == OPT_MCU_FT90X || CFG_TUSB_MCU == OPT_MCU_FT93X)

#include <stdint.h>
#include <ft900.h>
#include <registers/ft900_registers.h>

#define USBD_USE_STREAMS

#include "device/dcd.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

// Board code will determine the state of VBUS from USB host.
extern int8_t board_ft9xx_vbus(void);
extern int board_uart_write(void const *buf, int len);

// Static array to store an incoming SETUP request for processing by tinyusb.
CFG_TUD_MEM_SECTION CFG_TUSB_MEM_ALIGN
static uint8_t _ft9xx_setup_packet[8];

struct ft9xx_xfer_state
{
  volatile uint8_t ready; // OUT Transfer has been received and waiting for transfer.
  volatile uint8_t valid; // Transfer is pending and total_size, remain_size, and buff_ptr are valid.

  int16_t total_size; // Total transfer size in bytes for this transfer.
  int16_t remain_size; // Total remaining in transfer.
  uint8_t *buff_ptr; // Pointer to buffer to transmit from or receive to.

  uint8_t type; // Endpoint type. Of type USBD_ENDPOINT_TYPE from endpoint descriptor.
  uint8_t dir; // Endpoint direction. TUSB_DIR_OUT or TUSB_DIR_IN. For control endpoint this is the current direction.
  uint16_t buff_size; // Actual size of buffer RAM used by endpoint.
  uint16_t size; // Max packet size for endpoint from endpoint descriptor.
};
// Endpoint description array for each endpoint.
static struct ft9xx_xfer_state ep_xfer[USBD_MAX_ENDPOINT_COUNT];
// USB speed.
static tusb_speed_t _speed;

// Interrupt handlers.
void _ft9xx_usbd_ISR(void); // Interrupt handler for USB device.
void ft9xx_usbd_pm_ISR(void); // Interrupt handler for USB device for power management (called by board).

// Internal functions forward declarations.
static uint16_t _ft9xx_edpt_xfer_out(uint8_t ep_number, uint8_t *buffer, uint16_t xfer_bytes);
static uint16_t _ft9xx_edpt_xfer_in(uint8_t ep_number, uint8_t *buffer, uint16_t xfer_bytes);
static void _ft9xx_reset_edpts(void);
static inline void _ft9xx_phy_enable(bool en);
static void _ft9xx_usb_speed(void);
static void _dcd_ft9xx_attach(void);
static void _dcd_ft9xx_detach(void) __attribute__((unused));
static uint16_t _ft9xx_dusb_in(uint8_t ep_number, const uint8_t *buffer, uint16_t length);
static uint16_t _ft9xx_dusb_out(uint8_t ep_number, uint8_t *buffer, uint16_t length);

// Internal functions.

// Manage an OUT transfer from the host.
// This can be up-to the maximum packet size of the endpoint.
// Continuation of a transfer beyond the maximum packet size is performed
// by the interrupt handler.
static uint16_t _ft9xx_edpt_xfer_out(uint8_t ep_number, uint8_t *buffer, uint16_t xfer_bytes)
{
  //Note: this is called from only the interrupt handler when an OUT transfer is called.
  uint16_t ep_size = ep_xfer[ep_number].size;
  (void)ep_size;
  if (xfer_bytes > ep_size)
  {
    xfer_bytes = ep_size;
  }

  // Wait until the endpoint has finished - it should be complete!
  //while (!(USBD_EP_SR_REG(ep_number) & MASK_USBD_EPxSR_OPRDY))
    //;

  // Send the first packet of max packet size
  xfer_bytes = _ft9xx_dusb_out(ep_number, (uint8_t *)buffer, xfer_bytes);
  if (ep_number == USBD_EP_0)
  {
    // Set flags to indicate data ready.
    USBD_EP_SR_REG(USBD_EP_0) = (MASK_USBD_EP0SR_OPRDY);
  }
  else
  {
    USBD_EP_SR_REG(ep_number) = (MASK_USBD_EPxSR_OPRDY);
  }

  return xfer_bytes;
}

// Manage an IN transfer to the host.
// This can be up-to the maximum packet size of the endpoint.
// Continuation of a transfer beyond the maximum packet size is performed
// by the interrupt handler.
static uint16_t _ft9xx_edpt_xfer_in(uint8_t ep_number, uint8_t *buffer, uint16_t xfer_bytes)
{
  //Note: this may be called from the interrupt handler or from normal code.
  uint8_t end = 0;
  uint16_t ep_size = ep_xfer[ep_number].size;
  (void)ep_size;

  if ((xfer_bytes == 0) || (xfer_bytes < ep_size))
  {
    end = 1;
  }
  else
  {
    xfer_bytes = ep_size;
  }

  if (ep_number == USBD_EP_0)
  {
    // An IN direction SETUP can be interrupted by an OUT packet.
    // This will result in a STALL generated by the silicon.
    while (USBD_EP_SR_REG(USBD_EP_0) & MASK_USBD_EP0SR_STALL)
    {
      // Clear the STALL and finish the transaction.
      USBD_EP_SR_REG(USBD_EP_0) = (MASK_USBD_EP0SR_STALL);
    }
  }
  else
  {
    // If there is data to transmit then wait until the IN buffer
    // for the endpoint is empty.
    // This does not apply to interrupt endpoints.
    if (ep_xfer[ep_number].type != TUSB_XFER_INTERRUPT)
    {
      uint8_t sr_reg;
      do
      {
        sr_reg = USBD_EP_SR_REG(ep_number);
      } while (sr_reg & MASK_USBD_EPxSR_INPRDY);
    }
  }

  // Do not send a ZLP for interrupt endpoints.
  if ((ep_xfer[ep_number].type != TUSB_XFER_INTERRUPT) || (xfer_bytes > 0))
  {
    xfer_bytes = _ft9xx_dusb_in(ep_number, (uint8_t *)buffer, xfer_bytes);
  }

  if (ep_number == USBD_EP_0)
  {
    if (end)
    {
      // Set flags to indicate data ready and transfer complete.
      USBD_EP_SR_REG(USBD_EP_0) = MASK_USBD_EP0SR_INPRDY | MASK_USBD_EP0SR_DATAEND;
    }
    else
    {
      // Set flags to indicate data ready.
      USBD_EP_SR_REG(USBD_EP_0) = (MASK_USBD_EP0SR_INPRDY);
    }
  }
  else
  {
    // Set flags to indicate data ready.
    USBD_EP_SR_REG(ep_number) = (MASK_USBD_EPxSR_INPRDY);
  }

  return xfer_bytes;
}

// Reset all non-control endpoints to a default state.
// Control endpoint is always enabled and ready. All others disabled.
static void _ft9xx_reset_edpts(void)
{
  // Disable all endpoints and remove configuration values.
  for (int i = 1; i < USBD_MAX_ENDPOINT_COUNT; i++)
  {
    // Clear settings.
    tu_memclr(&ep_xfer[i], sizeof(struct ft9xx_xfer_state));
    // Disable hardware.
    USBD_EP_CR_REG(i) = 0;
  }

  // Enable interrupts from USB device control.
  USBD_REG(cmie) = MASK_USBD_CMIE_ALL;
}

// Enable or disable the USB PHY.
static inline void _ft9xx_phy_enable(bool en)
{
  if (en)
    SYS->PMCFG_L |= MASK_SYS_PMCFG_DEV_PHY_EN;
  else
    SYS->PMCFG_L &= ~MASK_SYS_PMCFG_DEV_PHY_EN;
}

// Safely connect to the USB.
static void _dcd_ft9xx_attach(void)
{
  uint8_t reg;

  CRITICAL_SECTION_BEGIN
  // Disable device responses.
  USBD_REG(faddr) = 0;

  // Reset USB Device.
  SYS->MSC0CFG = SYS->MSC0CFG | MASK_SYS_MSC0CFG_DEV_RESET_ALL;
  // Disable device connect/disconnect/host reset detection.
  SYS->PMCFG_H = MASK_SYS_PMCFG_DEV_DIS_DEV;
  SYS->PMCFG_H = MASK_SYS_PMCFG_DEV_CONN_DEV;
  SYS->PMCFG_L = SYS->PMCFG_L & (~MASK_SYS_PMCFG_DEV_DETECT_EN);

  // Enable Chip USB device clock/PM configuration.
  sys_enable(sys_device_usb_device);
  CRITICAL_SECTION_END;

  // Wait a short time to get started.
  delayms(1);

  CRITICAL_SECTION_BEGIN
  // Turn off the device enable bit.
#if BOARD_TUD_MAX_SPEED == OPT_MODE_HIGH_SPEED
  USBD_REG(fctrl) = 0;
#else // BOARD_TUD_MAX_SPEED == OPT_MODE_FULL_SPEED
  //Set the full speed only bit if required.
  USBD_REG(fctrl) = MASK_USBD_FCTRL_MODE_FS_ONLY;
#endif // BOARD_TUD_MAX_SPEED

  // Clear first reset and suspend interrupts.
  do
  {
    reg = USBD_REG(cmif);
    USBD_REG(cmif) = reg;
  } while (reg);
  // Clear any endpoint interrupts.
  reg = USBD_REG(epif);
  USBD_REG(epif) = reg;

  // Disable all interrupts from USB device control before attaching interrupt.
  USBD_REG(cmie) = 0;
  CRITICAL_SECTION_END;

  // Enable device connect/disconnect/host reset detection.
  // Set device detect and remote wakeup enable interrupt enables.
  SYS->PMCFG_L = SYS->PMCFG_L | MASK_SYS_PMCFG_DEV_DETECT_EN;

#if defined(__FT930__)
  // Setup VBUS detect
  SYS->MSC0CFG = SYS->MSC0CFG | MASK_SYS_MSC0CFG_USB_VBUS_EN;
#endif
}

// Gracefully disconnect from the USB.
static void _dcd_ft9xx_detach(void)
{
  // Disable device connect/disconnect/host reset detection.
  SYS->PMCFG_L = SYS->PMCFG_L & (~MASK_SYS_PMCFG_DEV_DETECT_EN);

#if defined(__FT930__)
  // Disable VBUS detection.
  SYS->MSC0CFG = SYS->MSC0CFG & (~MASK_SYS_MSC0CFG_USB_VBUS_EN);
#endif
  CRITICAL_SECTION_BEGIN
  // Disable interrupts from USB.
  USBD_REG(epie) = 0;
  USBD_REG(cmie) = 0;
  // Turn off the device enable bit.
  USBD_REG(fctrl) = 0;
  CRITICAL_SECTION_END;

  delayms(1);

  // Disable USB PHY
  dcd_disconnect(BOARD_TUD_RHPORT);
  delayms(1);

  // Disable Chip USB device clock/PM configuration.
  sys_disable(sys_device_usb_device);

  // Reset USB Device... Needed for Back voltage D+ to be <400mV
  SYS->MSC0CFG = SYS->MSC0CFG | MASK_SYS_MSC0CFG_DEV_RESET_ALL;

  delayms(1);
  // Set device detect and remote wakeup enable interrupt enables.
  SYS->PMCFG_L = SYS->PMCFG_L | MASK_SYS_PMCFG_DEV_DETECT_EN;

#if defined(__FT930__)
  // Setup VBUS detect
  SYS->MSC0CFG = SYS->MSC0CFG | MASK_SYS_MSC0CFG_USB_VBUS_EN;
#endif
}

// Determine the speed of the USB to which we are connected.
// Set the speed of the PHY accordingly.
// High speed can be disabled through CFG_TUSB_RHPORT0_MODE or CFG_TUD_MAX_SPEED settings.
static void _ft9xx_usb_speed(void)
{
	uint8_t  fctrl_val;

	// If USB device function is already enabled then disable it.
	if (USBD_REG(fctrl) & MASK_USBD_FCTRL_USB_DEV_EN) {
		USBD_REG(fctrl) = (USBD_REG(fctrl) & (~MASK_USBD_FCTRL_USB_DEV_EN));
		delayus(200);
	}

#if BOARD_TUD_MAX_SPEED == OPT_MODE_HIGH_SPEED

	/* Detect high or full speed */
	fctrl_val = MASK_USBD_FCTRL_USB_DEV_EN;
#if defined(__FT900__)
	if (!sys_check_ft900_revB())//if 90x series is rev C
	{
		fctrl_val |= MASK_USBD_FCTRL_IMP_PERF;
	}
#endif
	USBD_REG(fctrl) = fctrl_val;

#if defined(__FT930__)
	delayus(200);

	_speed = (SYS->MSC0CFG & MASK_SYS_MSC0CFG_HIGH_SPED_MODE) ?
		TUSB_SPEED_HIGH : TUSB_SPEED_FULL;
#else /* __FT930__ */
	/* Detection by SOF */
	while (!(USBD_REG(cmif) & MASK_USBD_CMIF_SOFIRQ));
	USBD_REG(cmif) = MASK_USBD_CMIF_SOFIRQ;
	delayus(125 + 5);
	_speed = (USBD_REG(cmif) & MASK_USBD_CMIF_SOFIRQ) ?
		TUSB_SPEED_HIGH : TUSB_SPEED_FULL;
  dcd_event_bus_reset(BOARD_TUD_RHPORT, _speed, true);

#endif /* !__FT930__ */

#else // BOARD_TUD_MAX_SPEED == OPT_MODE_FULL_SPEED

	/* User force set to full speed */
  _speed = TUSB_SPEED_FULL;
  fctrl_val =
			MASK_USBD_FCTRL_USB_DEV_EN | MASK_USBD_FCTRL_MODE_FS_ONLY;
#if defined(__FT900__)
	if (!sys_check_ft900_revB())//if 90x series is rev C
	{
			fctrl_val |= MASK_USBD_FCTRL_IMP_PERF;
	}
#endif
	USBD_REG(fctrl) = fctrl_val;
  dcd_event_bus_reset(BOARD_TUD_RHPORT, _speed, true);
	return;

#endif // BOARD_TUD_MAX_SPEED
}

// Send a buffer to the USB IN FIFO.
// When the macro USBD_USE_STREAMS is defined this will stream a buffer of data
// to the FIFO using the most efficient MCU streamout combination.
// If streaming is disabled then it will send each byte of the buffer in turn
// to the FIFO. The is no reason to not stream.
// The total number of bytes sent to the FIFO is returned.
static uint16_t _ft9xx_dusb_in(uint8_t ep_number, const uint8_t *buffer, uint16_t length)
{
  uint16_t bytes_read = 0;
  uint16_t buff_size = length;

#ifdef USBD_USE_STREAMS
  volatile uint8_t *data_reg;

  data_reg = (volatile uint8_t *)&(USBD->ep[ep_number].epxfifo);
  if (buff_size)
  {
    if (((uint32_t)buffer) % 4 == 0)
    {
      uint16_t aligned = buff_size & (~3);
      uint16_t left = buff_size & 3;

      if (aligned)
      {
        __asm__ volatile("streamout.l %0,%1,%2"
                         :
                         : "r"(data_reg), "r"(buffer), "r"(aligned));
        buffer += aligned;
      }
      if (left)
      {
        __asm__ volatile("streamout.b %0,%1,%2"
                         :
                         : "r"(data_reg), "r"(buffer), "r"(left));
      }
    }
    else
    {
      __asm__ volatile("streamout.b %0,%1,%2"
                       :
                       : "r"(data_reg), "r"(buffer), "r"(buff_size));
    }
    bytes_read = buff_size;
  }
#else // USBD_USE_STREAMS

  bytes_read = buff_size;
  while (buff_size--)
  {
    USBD_EP_FIFO_REG(ep_number) = *buffer++;
  };

#endif // USBD_USE_STREAMS

  return bytes_read;
}

// Receive a buffer from the USB OUT FIFO.
// When the macro USBD_USE_STREAMS is defined this will stream from the FIFO
// to a buffer of data using the most efficient MCU streamin combination.
// If streaming is disabled then it will receive each byte from the FIFO in turn
// to the buffer. The is no reason to not stream.
// The total number of bytes received from the FIFO is returned.
static uint16_t _ft9xx_dusb_out(uint8_t ep_number, uint8_t *buffer, uint16_t length)
{
#ifdef USBD_USE_STREAMS
  volatile uint8_t *data_reg;
#endif // USBD_USE_STREAMS
  uint16_t bytes_read = 0;
  uint16_t buff_size = length;

  if (length > 0)
  {
    if (ep_number == USBD_EP_0)
    {
      buff_size = USBD_EP_CNT_REG(USBD_EP_0);
    }
    else
    {
      if (USBD_EP_SR_REG(ep_number) & (MASK_USBD_EPxSR_OPRDY))
      {
        buff_size = USBD_EP_CNT_REG(ep_number);
      }
    }
  }

  // Only read as many bytes as we have space for.
  if (buff_size > length)
    buff_size = length;

#ifdef USBD_USE_STREAMS
  data_reg = (volatile uint8_t *)&(USBD->ep[ep_number].epxfifo);
  if (buff_size)
  {
    if ((uint32_t)buffer % 4 == 0)
    {
      uint16_t aligned = buff_size & (~3);
      uint16_t left = buff_size & 3;

      if (aligned)
      {
        __asm__ volatile("streamin.l %0,%1,%2"
                         :
                         : "r"(buffer), "r"(data_reg), "r"(aligned));
        buffer += aligned;
      }
      if (left)
      {
        __asm__ volatile("streamin.b %0,%1,%2"
                         :
                         : "r"(buffer), "r"(data_reg), "r"(left));
      }
    }
    else
    {
      __asm__ volatile("streamin.b %0,%1,%2"
                       :
                       : "r"(buffer), "r"(data_reg), "r"(buff_size));
    }
    bytes_read = buff_size;
  }
#else // USBD_USE_STREAMS

  bytes_read = buff_size;
  while (buff_size--)
  {
    *buffer++ = USBD_EP_FIFO_REG(ep_number);
  }

#endif // USBD_USE_STREAMS

  return bytes_read;
}

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

// Initialize controller to device mode
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  TU_LOG2("FT9xx initialisation\r\n");

  _dcd_ft9xx_attach();

  interrupt_attach(interrupt_usb_device, (int8_t)interrupt_usb_device, _ft9xx_usbd_ISR);

  dcd_connect(rhport);
  return true;
}

// Enable device interrupt
void dcd_int_enable(uint8_t rhport)
{
  (void)rhport;
  TU_LOG3("FT9xx int enable\r\n");

  // Peripheral devices interrupt enable.
  interrupt_enable_globally();
}

// Disable device interrupt
void dcd_int_disable(uint8_t rhport)
{
  (void)rhport;
  TU_LOG3("FT9xx int disable\r\n");

  // Peripheral devices interrupt disable.
  interrupt_disable_globally();
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  (void)rhport;
  (void)dev_addr;

  // Respond with status. There is no checking that the address is in range.
  dcd_edpt_xfer(rhport, tu_edpt_addr(USBD_EP_0, TUSB_DIR_IN), NULL, 0);

  // Set the update bit for the address register.
  dev_addr |= 0x80;

  // Modify the address register within a critical section.
  CRITICAL_SECTION_BEGIN
  {
    USBD_REG(faddr) = dev_addr;
  }
  CRITICAL_SECTION_END;
}

// Invoked when a control transfer's status stage is complete.
// May help DCD to prepare for next control transfer, this API is optional.
#if 0 // never called
void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
{
  (void) rhport;

  if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
      request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD )
  {
    if (request->bRequest == TUSB_REQ_SET_ADDRESS)
    {
    }
    else if (request->bRequest == TUSB_REQ_SET_CONFIGURATION)
    {
    }
  }
}
#endif // 0

// Wake up host
void dcd_remote_wakeup(uint8_t rhport)
{
  (void)rhport;

  SYS->MSC0CFG = SYS->MSC0CFG | MASK_SYS_MSC0CFG_DEV_RMWAKEUP;

  // At least 2 ms of delay needed for RESUME Data K state.
  delayms(2);

  SYS->MSC0CFG &= ~MASK_SYS_MSC0CFG_DEV_RMWAKEUP;

  // Enable USB PHY and determine current bus speed.
  dcd_connect(rhport);
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
  (void)rhport;
  TU_LOG2("FT9xx connect\r\n");

  CRITICAL_SECTION_BEGIN
  // Is device connected?
  if (board_ft9xx_vbus())
  {
    // Clear/disable address register.
    USBD_REG(faddr) = 0;
    _ft9xx_phy_enable(true);

    // Determine bus speed and signal speed to tusb.
    _ft9xx_usb_speed();
  }

  // Setup the control endpoint only.
#if CFG_TUD_ENDPOINT0_SIZE == 64
  USBD_EP_CR_REG(USBD_EP_0) = (USBD_EP0_MAX_SIZE_64 << BIT_USBD_EP0_MAX_SIZE);
#elif CFG_TUD_ENDPOINT0_SIZE == 32
  USBD_EP_CR_REG(USBD_EP_0) = (USBD_EP0_MAX_SIZE_32 << BIT_USBD_EP0_MAX_SIZE);
#elif CFG_TUD_ENDPOINT0_SIZE == 16
  USBD_EP_CR_REG(USBD_EP_0) = (USBD_EP0_MAX_SIZE_16 << BIT_USBD_EP0_MAX_SIZE);
#elif CFG_TUD_ENDPOINT0_SIZE == 8
  USBD_EP_CR_REG(USBD_EP_0) = (USBD_EP0_MAX_SIZE_8 << BIT_USBD_EP0_MAX_SIZE);
#else
#error "CFG_TUD_ENDPOINT0_SIZE must be defined with a value of 8, 16, 32 or 64."
#endif
  CRITICAL_SECTION_END;

  // Configure the control endpoint.
  ep_xfer[USBD_EP_0].size = CFG_TUD_ENDPOINT0_SIZE;
  ep_xfer[USBD_EP_0].type = TUSB_XFER_CONTROL;

  // Enable interrupts on EP0.
  USBD_REG(epie) = (MASK_USBD_EPIE_EP0IE);

  // Restore default endpoint state.
  _ft9xx_reset_edpts();
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
  (void)rhport;
  TU_LOG2("FT9xx disconnect\r\n");

  // Disable the USB PHY.
  _ft9xx_phy_enable(false);
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

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *ep_desc)
{
  (void)rhport;
  uint8_t const ep_number = tu_edpt_number(ep_desc->bEndpointAddress);
  uint8_t const ep_dir = tu_edpt_dir(ep_desc->bEndpointAddress);
  uint8_t const ep_type = ep_desc->bmAttributes.xfer;
  uint16_t const ep_size = tu_edpt_packet_size(ep_desc); // Mask size per packet, bits 10..0.
  uint16_t ep_buff_size;
  uint8_t ep_reg_size = USBD_EP_MAX_SIZE_8;
  uint8_t ep_reg_data = 0;
  int16_t total_ram;

  TU_LOG2("FT9xx endpoint open %d %c\r\n", ep_number, ep_dir?'I':'O');

  // Check that the requested endpoint number is allowable.
  if (ep_number >= USBD_MAX_ENDPOINT_COUNT)
  {
    TU_LOG1("FT9xx endpoint not valid: requested %d max %d\r\n", ep_number, USBD_MAX_ENDPOINT_COUNT);
    return false;
  }

  // Calculate the physical size of the endpoint as a power of 2. This may be more than
  // the requested size.
  while (ep_size > (8 * (1 << ep_reg_size)))
  {
    ep_reg_size++;
  }
  if (ep_reg_size > USBD_EP_MAX_SIZE_1024)
  {
    TU_LOG1("FT9xx endpoint size not valid: requested %d max 1024\r\n", ep_size);
    return false;
  }
  // Calculate actual amount of buffer RAM used by this endpoint. This may be more than the
  // requested size.
  ep_buff_size = 8 << ep_reg_size;

  if (ep_number > 0)
  {
    // Set EP cmd parameters...
    ep_reg_data |= (ep_reg_size << BIT_USBD_EP_MAX_SIZE);

    if (ep_xfer[ep_number].type != USBD_EP_TYPE_DISABLED)
    {
      // This could be because an endpoint has been assigned with the same number.
      // On FT9xx, IN and OUT endpoints may not have the same number. e.g. There
      // cannot been an 0x81 and 0x01 endpoint.
      TU_LOG1("FT9xx endpoint %d already assigned\r\n", ep_number);
      return false;
    }

    // Check that there is enough buffer RAM to allocate to this new endpoint.
    // Available buffer RAM depends on the device revision.
    // The IN and OUT buffer RAM should be the same size.
    if (ep_dir == USBD_DIR_IN)
      total_ram = USBD_RAMTOTAL_IN;
    else
      total_ram = USBD_RAMTOTAL_OUT;
    // Work out how much has been allocated to existing endpoints.
    // The total RAM allocated should always be a positive number as this
    // algorithm should not let it go below zero.
    for (int i = 1; i < USBD_MAX_ENDPOINT_COUNT; i++)
    {
      if (ep_xfer[i].type != USBD_EP_TYPE_DISABLED)
      {
        if (ep_xfer[i].dir == ep_dir)
        {
          total_ram -= ep_xfer[i].buff_size;
        }
      }
    }

    if (sys_check_ft900_revB())
    {
      // The control endpoint is taken into account as well on RevB silicon.
      total_ram -= ep_xfer[0].buff_size;
    }

    // Make sure we have enough space. The corner case is having zero bytes
    // free which means that total_ram must be signed as zero bytes free is
    // allowable.
    if (total_ram < ep_buff_size)
    {
      TU_LOG1("FT9xx insufficient buffer RAM for endpoint %d\r\n", ep_number);
      return false;
    }

    // Set the type of this endpoint in the control register.
    if (ep_type == TUSB_XFER_BULK)
      ep_reg_data |= (USBD_EP_DIS_BULK << BIT_USBD_EP_CONTROL_DIS);
    else if (ep_type == TUSB_XFER_INTERRUPT)
      ep_reg_data |= (USBD_EP_DIS_INT << BIT_USBD_EP_CONTROL_DIS);
    else if (ep_type == TUSB_XFER_ISOCHRONOUS)
      ep_reg_data |= (USBD_EP_DIS_ISO << BIT_USBD_EP_CONTROL_DIS);
    // Set the direction of this endpoint in the control register.
    if (ep_dir == USBD_DIR_IN)
      ep_reg_data |= MASK_USBD_EPxCR_DIR;
    // Do not perform double buffering.
    //if (<double buffering flag> != USBD_DB_OFF)
    //ep_reg_data |= MASK_USBD_EPxCR_DB;
    // Set the control register for this endpoint.
    USBD_EP_CR_REG(ep_number) = ep_reg_data;
    TU_LOG2("FT9xx endpoint setting %x\r\n", ep_reg_data);
  }
  else
  {
    // Set the control register for endpoint zero.
    USBD_EP_CR_REG(USBD_EP_0) = (ep_reg_size << BIT_USBD_EP0_MAX_SIZE);
  }

  CRITICAL_SECTION_BEGIN
  // Store the endpoint characteristics for later reference.
  ep_xfer[ep_number].dir = ep_dir;
  ep_xfer[ep_number].type = ep_type;
  ep_xfer[ep_number].size = ep_size;
  ep_xfer[ep_number].buff_size = ep_buff_size;

  // Clear register transaction continuation and signalling state.
  ep_xfer[ep_number].ready = 0;
  ep_xfer[ep_number].valid = 0;
  ep_xfer[ep_number].buff_ptr = NULL;
  ep_xfer[ep_number].total_size = 0;
  ep_xfer[ep_number].remain_size = 0;
  CRITICAL_SECTION_END

  return true;
}

// Close all endpoints.
void dcd_edpt_close_all(uint8_t rhport)
{
  (void)rhport;
  // Reset the endpoint configurations.
  _ft9xx_reset_edpts();
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
  (void)rhport;
  uint8_t ep_number = tu_edpt_number(ep_addr);
  uint8_t ep_dir = tu_edpt_dir(ep_addr);
  uint16_t xfer_bytes;
  bool status = false;

  // We will attempt to transfer the buffer. If it is less than or equal to the  endpoint
  // maximum packet size then the whole buffer will be transferred. If it is larger then
  // the interrupt handler will transfer the remainder.
  // ep_xfer is used to tell the interrupt handler what to do.
  // ep_xfer can be used at interrupt level to continue transfers.
  CRITICAL_SECTION_BEGIN

  // Transfer currently in progress.
  if (ep_xfer[ep_number].valid == 0)
  {
    ep_xfer[ep_number].total_size = total_bytes;
    ep_xfer[ep_number].remain_size = total_bytes;
    ep_xfer[ep_number].buff_ptr = buffer;

    if (ep_number == USBD_EP_0)
    {
      ep_xfer[USBD_EP_0].dir = ep_dir;
    }
    else
    {
      // Enable the interrupt for this endpoint allowing the interrupt handler to report
      // continue the transfer and signal completion.
      USBD_REG(epie) = USBD_REG(epie) | (1 << ep_number);
    }

    if (ep_dir == TUSB_DIR_IN)
    {
      // For IN transfers send the first packet as a starter. Interrupt handler to complete
      // this if it is larger than one packet.
      xfer_bytes = _ft9xx_edpt_xfer_in(ep_number, buffer, total_bytes);

      ep_xfer[ep_number].buff_ptr += xfer_bytes;
      ep_xfer[ep_number].remain_size -= xfer_bytes;

      // Tell the interrupt handler to signal dcd_event_xfer_complete on completion.
      ep_xfer[ep_number].valid = 1;
    }
    else // (dir == TUSB_DIR_OUT)
    {
      // For OUT transfers on the control endpoint.
      // The host may already have performed the first data transfer after the SETUP packet
      // before the transfer is setup for it.
      if (ep_xfer[ep_number].ready)
      {
        // We have received a data packet on the endpoint without a transfer
        // being initialised. This can be because the host has sent this packet before
        // a new transfer has been initiated on the endpoint.
        // We will now stream the data from the FIFO.
        ep_xfer[ep_number].ready = 0;

        // Transfer incoming data from an OUT packet to the buffer.
        xfer_bytes = _ft9xx_edpt_xfer_out(ep_number, buffer, total_bytes);

        // Report completion of the transfer.
        dcd_event_xfer_complete(BOARD_TUD_RHPORT, ep_number /*| TUSB_DIR_OUT_MASK */, xfer_bytes, XFER_RESULT_SUCCESS, false);
      }
      else
      {
        // Tell the interrupt handler to wait for the packet to be received and
        // then report the transfer complete with dcd_event_xfer_complete.
        ep_xfer[ep_number].valid = 1;
      }
    }
    status = true;
  }
  else
  {
    // Note: should not arrive here.
  }

  CRITICAL_SECTION_END

  return status;
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack - optional, however, must be listed in usbd.c
bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t *ff, uint16_t total_bytes)
{
  (void)rhport;
  (void)ep_addr;
  (void)ff;
  (void)total_bytes;
  bool status = false;
  return status;
}

// Stall endpoint (non-control endpoint)
void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  uint8_t ep_number = tu_edpt_number(ep_addr);
  (void)rhport;

  CRITICAL_SECTION_BEGIN
  if (ep_number == USBD_EP_0)
  {
    USBD_EP_CR_REG(USBD_EP_0) = USBD_EP_CR_REG(USBD_EP_0) |
                              MASK_USBD_EP0CR_SDSTL;
  }
  else
  {
    USBD_EP_CR_REG(ep_number) = USBD_EP_CR_REG(ep_number) |
                              MASK_USBD_EPxCR_SDSTL;
    USBD_EP_SR_REG(ep_number) = MASK_USBD_EPxSR_CLR_TOGGLE |
                              MASK_USBD_EPxSR_FIFO_FLUSH;
  }
  CRITICAL_SECTION_END
}

// Clear stall (non-control endpoint), data toggle is also reset to DATA0
void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  uint8_t ep_number = tu_edpt_number(ep_addr);
  (void)rhport;

  if (ep_number > USBD_EP_0)
  {
    CRITICAL_SECTION_BEGIN
    USBD_EP_CR_REG(ep_number) = USBD_EP_CR_REG(ep_number) &
                              (~MASK_USBD_EPxCR_SDSTL);
    USBD_EP_SR_REG(ep_number) = MASK_USBD_EPxSR_CLR_TOGGLE;

    // Allow transfers to restart.
    ep_xfer[ep_number].ready = 0;
    ep_xfer[ep_number].valid = 0;
    ep_xfer[ep_number].remain_size = 0;
    CRITICAL_SECTION_END
  }
}

// Interrupt handling.

void _ft9xx_usbd_ISR(void)
{
  dcd_int_handler(BOARD_TUD_RHPORT);
}

void dcd_int_handler(uint8_t rhport)
{
  (void)rhport;

  // Read the Common Interrupt Flag Register.
  uint8_t cmif = USBD_REG(cmif);
  // Read the Endpoint Interrupt Flag Register.
#if defined(__FT930__)
  // This is 16 bits on FT93x.
  uint16_t epif = USBD_REG(epif);
#else
  // This is 8 bits on FT90x.
  uint8_t epif = USBD_REG(epif);
#endif

  if (cmif & MASK_USBD_CMIF_ALL)
  {
    // Clear all CMIF bits.
    USBD_REG(cmif) = MASK_USBD_CMIF_ALL;
    if (cmif & MASK_USBD_CMIF_PHYIRQ) //Handle PHY interrupt
    {
    }
    if (cmif & MASK_USBD_CMIF_PIDIRQ) //Handle PIDIRQ interrupt
    {
    }
    if (cmif & MASK_USBD_CMIF_CRC16IRQ) //Handle CRC16IRQ interrupt
    {
    }
    if (cmif & MASK_USBD_CMIF_CRC5IRQ) //Handle CRC5 interrupt
    {
    }
    if (cmif & MASK_USBD_CMIF_RSTIRQ) //Handle Reset interrupt
    {
      // Reset endpoints to default state.
      _ft9xx_reset_edpts();
      dcd_event_bus_reset(BOARD_TUD_RHPORT, _speed, true);
    }
    if (cmif & MASK_USBD_CMIF_SUSIRQ) //Handle Suspend interrupt
    {
      dcd_event_bus_signal(BOARD_TUD_RHPORT, DCD_EVENT_SUSPEND, true);
    }
    if (cmif & MASK_USBD_CMIF_RESIRQ) //Handle Resume interrupt
    {
      dcd_event_bus_signal(BOARD_TUD_RHPORT, DCD_EVENT_RESUME, true);
    }
    if (cmif & MASK_USBD_CMIF_SOFIRQ) //Handle SOF interrupt
    {
      dcd_event_bus_signal(BOARD_TUD_RHPORT, DCD_EVENT_SOF, true);
    }
  }
  // Handle endpoint interrupts.
  if (epif)
  {
    uint16_t xfer_bytes;

    // Check for EP0 interrupts pending.
    if (epif & MASK_USBD_EPIF_EP0IRQ)
    {
      // Clear interrupt register.
      USBD_REG(epif) = MASK_USBD_EPIF_EP0IRQ;
      // Test for an incoming SETUP request on the control endpoint.
      if (USBD_EP_SR_REG(USBD_EP_0) & MASK_USBD_EP0SR_SETUP)
      {
        // If protocol STALL, End the STALL signalling.
        if (USBD_EP_CR_REG(USBD_EP_0) & MASK_USBD_EP0CR_SDSTL)
        {
          // STALL end.
          USBD_EP_CR_REG(USBD_EP_0) = USBD_EP_CR_REG(USBD_EP_0) &
                                      (~MASK_USBD_EP0CR_SDSTL);
          // Clear STALL send.
          USBD_EP_SR_REG(USBD_EP_0) = MASK_USBD_EP0SR_STALL;
        }

        // Host has sent a SETUP packet. Receive this into the SETUP packet store.
        _ft9xx_dusb_out(USBD_EP_0, (uint8_t *)_ft9xx_setup_packet, sizeof(USB_device_request));

        // Send the packet to tinyusb.
        dcd_event_setup_received(BOARD_TUD_RHPORT, _ft9xx_setup_packet, true);

        // Clear the interrupt that signals a SETUP packet is received.
        USBD_EP_SR_REG(USBD_EP_0) = (MASK_USBD_EP0SR_SETUP);

        // Any SETUP packet will clear the incoming FIFO.
        ep_xfer[USBD_EP_0].ready = 0;

        // Allow new DATA and ACK transfers on the control endpoint.
        ep_xfer[USBD_EP_0].valid = 0;
        return;
      }
      else
      {
        // Check for a complete or partially complete transfers on EP0.
        if (ep_xfer[USBD_EP_0].valid)
        {
          xfer_bytes = (uint16_t)ep_xfer[USBD_EP_0].total_size;

          // Transfer incoming data from an OUT packet to the buffer supplied.
          if (ep_xfer[USBD_EP_0].dir == TUSB_DIR_OUT)
          {
            xfer_bytes = _ft9xx_edpt_xfer_out(USBD_EP_0, ep_xfer[USBD_EP_0].buff_ptr, xfer_bytes);
          }
          // Now signal completion of data packet.
          dcd_event_xfer_complete(BOARD_TUD_RHPORT, USBD_EP_0 | (ep_xfer[USBD_EP_0].dir ? TUSB_DIR_IN_MASK : 0),
            xfer_bytes, XFER_RESULT_SUCCESS, true);

          // Incoming FIFO has been cleared.
          ep_xfer[USBD_EP_0].ready = 0;

          // Allow new transfers on the control endpoint.
          ep_xfer[USBD_EP_0].valid = 0;
        }
        // No transfer is in flight for EP0.
        else
        {
          // We have received a data packet on the control endpoint without a transfer
          // being initialised. This can be because the host has sent this packet before
          // a new transfer has been initiated on the control endpoint.
          // We will record that there is data in the FIFO for dcd_edpt_xfer to obtain
          // once the transfer is initiated.
          ep_xfer[USBD_EP_0].ready = 1;
        }
      }
    }
    else // !(epif & MASK_USBD_EPIF_EP0IRQ)
    {
      // Mask out currently disabled endpoints.
      epif &= USBD_REG(epie);

      // Handle complete and partially complete transfers for each endpoint.
      for (uint8_t ep_number = 1; ep_number < USBD_MAX_ENDPOINT_COUNT; ep_number++)
      {
        if ((epif & MASK_USBD_EPIF_IRQ(ep_number)) == 0)
        {
          // No pending interrupt for this endpoint.
          continue;
        }

        if (ep_xfer[ep_number].valid)
        {
          xfer_bytes = 0;

          // Clear interrupt register for this endpoint.
          USBD_REG(epif) = MASK_USBD_EPIF_IRQ(ep_number);

          // Start or continue an OUT transfer.
          if (ep_xfer[ep_number].dir == TUSB_DIR_OUT)
          {
            xfer_bytes = _ft9xx_edpt_xfer_out(ep_number,
                            ep_xfer[ep_number].buff_ptr,
                            (uint16_t)ep_xfer[ep_number].remain_size);

            // Report each OUT packet received to the stack.
            dcd_event_xfer_complete(BOARD_TUD_RHPORT,
                                      ep_number /* | TUSB_DIR_OUT_MASK */,
                                      xfer_bytes, XFER_RESULT_SUCCESS, true);

            ep_xfer[ep_number].buff_ptr += xfer_bytes;
            ep_xfer[ep_number].remain_size -= xfer_bytes;
          }
          // continue an IN transfer
          else // if (ep_xfer[ep_number].dir == TUSB_DIR_IN)
          {
            if (ep_xfer[ep_number].remain_size > 0)
            {
              xfer_bytes = _ft9xx_edpt_xfer_in(ep_number,
                            ep_xfer[ep_number].buff_ptr,
                            (uint16_t)ep_xfer[ep_number].remain_size);

              ep_xfer[ep_number].buff_ptr += xfer_bytes;
              ep_xfer[ep_number].remain_size -= xfer_bytes;
            }

            if (ep_xfer[ep_number].remain_size == 0)
            {
              dcd_event_xfer_complete(BOARD_TUD_RHPORT,
                                      ep_number | TUSB_DIR_IN_MASK,
                                      ep_xfer[ep_number].total_size, XFER_RESULT_SUCCESS, true);
            }
          }

          // When the transfer is complete...
          if (ep_xfer[ep_number].remain_size == 0)
          {
            // Finish this transfer and allow new transfers on this endpoint.
            ep_xfer[ep_number].valid = 0;

            // Disable the interrupt for this endpoint now it is complete.
            USBD_REG(epie) = USBD_REG(epie) & (~(1 << ep_number));
          }

          ep_xfer[ep_number].ready = 0;
        }
        // No OUT transfer is in flight for this endpoint.
        else
        {
          if (ep_xfer[ep_number].dir == TUSB_DIR_OUT)
          {
            // We will record that there is data in the FIFO for dcd_edpt_xfer to obtain
            // once the transfer is initiated.
            // Strictly this should not happen for a non-control endpoint. Interrupts
            // are disabled when there are no transfers setup for an endpoint.
            ep_xfer[ep_number].ready = 1;
          }
        }
      }
    }
  }
}

// Power management interrupt handler.
// This handles USB device related power management interrupts only.
void ft9xx_usbd_pm_ISR(void)
{
    uint16_t pmcfg = SYS->PMCFG_H;

  // Main interrupt handler is responible for
  if (pmcfg & MASK_SYS_PMCFG_DEV_CONN_DEV)
  {
      // Signal connection interrupt
      SYS->PMCFG_H = MASK_SYS_PMCFG_PM_GPIO_IRQ_PEND;
      dcd_event_bus_signal(BOARD_TUD_RHPORT, DCD_EVENT_RESUME, true);
  }

  if (pmcfg & MASK_SYS_PMCFG_DEV_DIS_DEV)
  {
      // Signal disconnection interrupt
      SYS->PMCFG_H = MASK_SYS_PMCFG_PM_GPIO_IRQ_PEND;
      dcd_event_bus_signal(BOARD_TUD_RHPORT, DCD_EVENT_UNPLUGGED, true);
  }

  if (pmcfg & MASK_SYS_PMCFG_HOST_RST_DEV)
  {
      // Signal Host Reset interrupt
      SYS->PMCFG_H = MASK_SYS_PMCFG_PM_GPIO_IRQ_PEND;
      dcd_event_bus_signal(BOARD_TUD_RHPORT, DCD_EVENT_BUS_RESET, true);
  }

  if (pmcfg & MASK_SYS_PMCFG_HOST_RESUME_DEV)
  {
      // Signal Host Resume interrupt
      SYS->PMCFG_H = MASK_SYS_PMCFG_PM_GPIO_IRQ_PEND;
      if (!(SYS->MSC0CFG & MASK_SYS_MSC0CFG_DEV_RMWAKEUP))
      {
          // If we are driving K-state on Device USB port;
          // We must maintain the 1ms requirement before resuming the phy
          dcd_event_bus_signal(BOARD_TUD_RHPORT, DCD_EVENT_RESUME, true);
      }
  }
}

#endif
