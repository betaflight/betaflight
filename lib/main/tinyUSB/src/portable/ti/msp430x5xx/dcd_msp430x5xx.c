/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019-2020 William D. Jones
 * Copyright (c) 2019-2020 Ha Thach (tinyusb.org)
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

#if CFG_TUD_ENABLED && ( CFG_TUSB_MCU == OPT_MCU_MSP430x5xx )

#include "msp430.h"
#include "device/dcd.h"

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/
// usbpllir_mirror and usbmaintl_mirror can be added later if needed.
static volatile uint16_t usbiepie_mirror = 0;
static volatile uint16_t usboepie_mirror = 0;
static volatile uint8_t usbie_mirror = 0;
static volatile uint16_t usbpwrctl_mirror = 0;
static bool in_isr = false;

uint8_t _setup_packet[8];

// Xfer control
typedef struct
{
  uint8_t * buffer;
  // tu_fifo_t * ff; // TODO support dcd_edpt_xfer_fifo API
  uint16_t total_len;
  uint16_t queued_len;
  uint16_t max_size;
  bool short_packet;
} xfer_ctl_t;

xfer_ctl_t xfer_status[8][2];
#define XFER_CTL_BASE(_ep, _dir) &xfer_status[_ep][_dir]

// Accessing endpoint regs
typedef volatile uint8_t * ep_regs_t;

typedef enum
{
  CNF = 0,
  BBAX = 1,
  BCTX = 2,
  BBAY = 5,
  BCTY = 6,
  SIZXY = 7
} ep_regs_index_t;

#define EP_REGS(epnum, dir) ((ep_regs_t) ((uintptr_t)&USBOEPCNF_1 + 64*dir + 8*(epnum - 1)))

static void bus_reset(void)
{
  // Hardcoded into the USB core.
  xfer_status[0][TUSB_DIR_OUT].max_size = 8;
  xfer_status[0][TUSB_DIR_IN].max_size = 8;

  USBKEYPID = USBKEY;

  // Enable the control EP 0. Also enable Indication Enable- a guard flag
  // separate from the Interrupt Enable mask.
  USBOEPCNF_0 |= (UBME | USBIIE);
  USBIEPCNF_0 |= (UBME | USBIIE);

  // Enable interrupts for this endpoint.
  USBOEPIE |= BIT0;
  USBIEPIE |= BIT0;

  // Clear NAK until a setup packet is received.
  USBOEPCNT_0 &= ~NAK;
  USBIEPCNT_0 &= ~NAK;

  // Enable responding to packets.
  USBCTL |= FEN;

  // Dedicated buffers in hardware for SETUP and EP0, no setup needed.
  // Now safe to respond to SETUP packets.
  USBIE |= SETUPIE;

  USBKEYPID = 0;
}

// Controls reset behavior of the USB module on receipt of a bus reset event.
// - enable: When true, bus reset events will cause a reset the USB module.
static void enable_functional_reset(const bool enable)
{
  // Check whether or not the USB configuration registers were
  // locked prior to this function being called so that, if
  // necessary, the lock state can be restored on exit.
  bool unlocked = (USBKEYPID == 0xA528) ? true : false;

  if(!unlocked) USBKEYPID = USBKEY;

  if(enable)
  {
    USBCTL |= FRSTE;
  }
  else
  {
    USBCTL &= ~FRSTE;
  }

  if(!unlocked) USBKEYPID = 0;
}

/*------------------------------------------------------------------*/
/* Controller API
 *------------------------------------------------------------------*/
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport; (void) rh_init;

  USBKEYPID = USBKEY;

  // Enable the module (required to write config regs)!
  USBCNF |= USB_EN;

  // Reset used interrupts
  USBOEPIE = 0;
  USBIEPIE = 0;
  USBIE = 0;
  USBOEPIFG = 0;
  USBIEPIFG = 0;
  USBIFG = 0;
  USBPWRCTL &= ~(VUOVLIE | VBONIE | VBOFFIE | VUOVLIFG | VBONIFG | VBOFFIFG);
  usboepie_mirror = 0;
  usbiepie_mirror = 0;
  usbie_mirror = 0;
  usbpwrctl_mirror = 0;

  USBVECINT = 0;

  if(USBPWRCTL & USBBGVBV) {// Bus power detected?
    USBPWRCTL |= VBOFFIE;   // Enable bus-power-removed interrupt.
    USBIE |= RSTRIE;        // Enable reset and wait for it before continuing.
    USBCNF |= PUR_EN;       // Enable pullup.
  } else {
    USBPWRCTL |= VBONIE;    // Enable bus-power-applied interrupt.
    USBCNF &= ~USB_EN;      // Disable USB module until bus power is detected.
  }

  USBKEYPID = 0;

  return true;
}

// There is no "USB peripheral interrupt disable" bit on MSP430, so we have
// to save the relevant registers individually.
// WARNING: Unlike the ARM/NVIC routines, these functions are _not_ idempotent
// if you modified the registers saved in between calls so they don't match
// the mirrors; mirrors will be updated to reflect most recent register
// contents.
void dcd_int_enable (uint8_t rhport)
{
  (void) rhport;

  __bic_SR_register(GIE); // Unlikely to be called in ISR, but let's be safe.
                          // Also, this cleanly disables all USB interrupts
                          // atomically from application's POV.

  // This guard is required because tinyusb can enable interrupts without
  // having disabled them first.
  if(in_isr)
  {
    USBOEPIE = usboepie_mirror;
    USBIEPIE = usbiepie_mirror;
    USBIE = usbie_mirror;
    USBPWRCTL |= usbpwrctl_mirror;
  }

  in_isr = false;
  __bis_SR_register(GIE);
}

void dcd_int_disable (uint8_t rhport)
{
  (void) rhport;

  __bic_SR_register(GIE);
  usboepie_mirror = USBOEPIE;
  usbiepie_mirror = USBIEPIE;
  usbie_mirror = USBIE;
  usbpwrctl_mirror = (USBPWRCTL & (VUOVLIE | VBONIE | VBOFFIE));
  USBOEPIE = 0;
  USBIEPIE = 0;
  USBIE = 0;
  USBPWRCTL &= ~(VUOVLIE | VBONIE | VBOFFIE);
  in_isr = true;
  __bis_SR_register(GIE);
}

void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;

  USBFUNADR = dev_addr;

  // Response with status after changing device address
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;
}

void dcd_connect(uint8_t rhport)
{
  dcd_int_disable(rhport);

  USBKEYPID = USBKEY;
  USBCNF |= PUR_EN; // Enable pullup.
  USBKEYPID = 0;

  dcd_int_enable(rhport);
}

void dcd_disconnect(uint8_t rhport)
{
  dcd_int_disable(rhport);

  USBKEYPID = USBKEY;
  USBCNF &= ~PUR_EN; // Disable pullup.
  USBKEYPID = 0;

  dcd_int_enable(rhport);
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

/*------------------------------------------------------------------*/
/* DCD Endpoint port
 *------------------------------------------------------------------*/

bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * desc_edpt)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(desc_edpt->bEndpointAddress);
  uint8_t const dir   = tu_edpt_dir(desc_edpt->bEndpointAddress);

  // Unsupported endpoint numbers or type (Iso not supported. Control
  // not supported on nonzero endpoints).
  if( (epnum > 7) || \
      (desc_edpt->bmAttributes.xfer == 0) || \
      (desc_edpt->bmAttributes.xfer == 1)) {
    return false;
  }

  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, dir);
  xfer->max_size = tu_edpt_packet_size(desc_edpt);

  // Buffer allocation scheme:
  // For simplicity, only single buffer for now, since tinyusb currently waits
  // for an xfer to complete before scheduling another one. This means only
  // the X buffer is used.
  //
  // 1904 bytes are available, the max endpoint size supported on msp430 is
  // 64 bytes. This is enough RAM for all 14 endpoints enabled _with_ double
  // bufferring (64*14*2 = 1792 bytes). Extra RAM exists for triple and higher
  // order bufferring, which must be maintained in software.
  //
  // For simplicity, each endpoint gets a hardcoded 64 byte chunk (regardless
  // of actual wMaxPacketSize) whose start address is the following:
  // addr = 128 * (epnum - 1) + 64 * dir.
  //
  // Double buffering equation:
  // x_addr = 256 * (epnum - 1) + 128 * dir
  // y_addr = x_addr + 64
  // Address is right-shifted by 3 to fit into 8 bits.

  uint8_t buf_base = (128 * (epnum - 1) + 64 * dir) >> 3;

  // IN and OUT EP registers have the same structure.
  ep_regs_t ep_regs = EP_REGS(epnum, dir);

  // FIXME: I was able to get into a situation where OUT EP 3 would stall
  // while debugging, despite stall code never being called. It appears
  // these registers don't get cleared on reset, being part of RAM.
  // Investigate and see if I can duplicate.
  // Also, DBUF got set on OUT EP 2 while debugging. Only OUT EPs seem to be
  // affected at this time. USB RAM directly precedes main RAM; perhaps I'm
  // overwriting registers via buffer overflow w/ my debugging code?
  ep_regs[SIZXY] = tu_edpt_packet_size(desc_edpt);
  ep_regs[BCTX] |= NAK;
  ep_regs[BBAX] = buf_base;
  ep_regs[CNF] &= ~(TOGGLE | STALL | DBUF); // ISO xfers not supported on
                           // MSP430, so no need to gate DATA0/1 and frame
                           // behavior. Clear stall and double buffer bit as
                           // well- see above comment.
  ep_regs[CNF] |= (UBME | USBIIE);

  USBKEYPID = USBKEY;
  if(dir == TUSB_DIR_OUT)
  {
    USBOEPIE |= (1 << epnum);
  }
  else
  {
    USBIEPIE |= (1 << epnum);
  }
  USBKEYPID = 0;

  return true;
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport; (void) ep_addr;
  // TODO implement dcd_edpt_close()
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  // TODO implement dcd_edpt_close_all()
}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, dir);
  xfer->buffer = buffer;
  // xfer->ff     = NULL; // TODO support dcd_edpt_xfer_fifo API
  xfer->total_len = total_bytes;
  xfer->queued_len = 0;
  xfer->short_packet = false;

  if(epnum == 0)
  {
    if(dir == TUSB_DIR_OUT)
    {
      // Interrupt will notify us when data was received.
      USBCTL &= ~DIR;
      USBOEPCNT_0 &= ~NAK;
    }
    else
    {
      // Kickstart the IN packet handler by queuing initial data and calling
      // the ISR to transmit the first packet.
      // Interrupt only fires on completed xfer.
      USBCTL |= DIR;
      USBIEPIFG |= BIT0;
    }
  }
  else
  {
    ep_regs_t ep_regs = EP_REGS(epnum, dir);

    if(dir == TUSB_DIR_OUT)
    {
      ep_regs[BCTX] &= ~NAK;
    }
    else
    {
      USBIEPIFG |= (1 << epnum);
    }
  }

  return true;
}

#if 0 // TODO support dcd_edpt_xfer_fifo API
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, dir);
  xfer->buffer = NULL;
  xfer->ff     = ff;
  xfer->total_len = total_bytes;
  xfer->queued_len = 0;
  xfer->short_packet = false;

  ep_regs_t ep_regs = EP_REGS(epnum, dir);

  if(dir == TUSB_DIR_OUT)
  {
    ep_regs[BCTX] &= ~NAK;
  }
  else
  {
    USBIEPIFG |= (1 << epnum);
  }

  return true;
}
#endif

void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  if(epnum == 0)
  {
    if(dir == TUSB_DIR_OUT)
    {
      USBOEPCNT_0 |= NAK;
      USBOEPCNF_0 |= STALL;
    }
    else
    {
      USBIEPCNT_0 |= NAK;
      USBIEPCNF_0 |= STALL;
    }
  }
  else
  {
    ep_regs_t ep_regs = EP_REGS(epnum, dir);
    ep_regs[CNF] |= STALL;
  }
}

void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  if(epnum == 0)
  {
    if(dir == TUSB_DIR_OUT)
    {
      USBOEPCNF_0 &= ~STALL;
    }
    else
    {
      USBIEPCNF_0 &= ~STALL;
    }
  }
  else
  {
    ep_regs_t ep_regs = EP_REGS(epnum, dir);
    // Required by USB spec to reset DATA toggle bit to DATA0 on interrupt
    // and bulk endpoints.
    ep_regs[CNF] &= ~(STALL + TOGGLE);
  }
}

void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
{
  (void) rhport;
  (void) request;

  // FIXME: Per manual, we should be clearing the NAK bits of EP0 after the
  // Status Phase of a control xfer is done, in preparation of another possible
  // SETUP packet. However, from my own testing, SETUP packets _are_ correctly
  // handled by the USB core without clearing the NAKs.
  //
  // Right now, clearing NAKs in this callbacks causes a direction mismatch
  // between host and device on EP0. Figure out why and come back to this.
  // USBOEPCNT_0 &= ~NAK;
  // USBIEPCNT_0 &= ~NAK;
}

/*------------------------------------------------------------------*/

static void receive_packet(uint8_t ep_num)
{
  xfer_ctl_t * xfer = XFER_CTL_BASE(ep_num, TUSB_DIR_OUT);
  ep_regs_t ep_regs = EP_REGS(ep_num, TUSB_DIR_OUT);
  uint8_t xfer_size;

  if(ep_num == 0)
  {
    xfer_size = USBOEPCNT_0 & 0x0F;
  }
  else
  {
    xfer_size = ep_regs[BCTX] & 0x7F;
  }

  uint16_t remaining = xfer->total_len - xfer->queued_len;
  uint16_t to_recv_size;

  if(remaining <= xfer->max_size) {
    // Avoid buffer overflow.
    to_recv_size = (xfer_size > remaining) ? remaining : xfer_size;
  } else {
    // Room for full packet, choose recv_size based on what the microcontroller
    // claims.
    to_recv_size = (xfer_size > xfer->max_size) ? xfer->max_size : xfer_size;
  }

#if 0 // TODO support dcd_edpt_xfer_fifo API
  if (xfer->ff)
  {
    volatile uint8_t * ep_buf = (ep_num == 0) ? &USBOEP0BUF : (&USBSTABUFF + (ep_regs[BBAX] << 3));
    tu_fifo_write_n(xfer->ff, (const void *) ep_buf, to_recv_size);
  }
  else
#endif
  {
    uint8_t * base = (xfer->buffer + xfer->queued_len);

    if(ep_num == 0)
    {
      volatile uint8_t * ep0out_buf = &USBOEP0BUF;
      for(uint16_t i = 0; i < to_recv_size; i++)
      {
        base[i] = ep0out_buf[i];
      }
    }
    else
    {
      volatile uint8_t * ep_buf = &USBSTABUFF + (ep_regs[BBAX] << 3);
      for(uint16_t i = 0; i < to_recv_size ; i++)
      {
        base[i] = ep_buf[i];
      }
    }
  }

  xfer->queued_len += xfer_size;

  xfer->short_packet = (xfer_size < xfer->max_size);
  if((xfer->total_len == xfer->queued_len) || xfer->short_packet)
  {
    dcd_event_xfer_complete(0, ep_num, xfer->queued_len, XFER_RESULT_SUCCESS, true);
  }
  else
  {
    // Schedule to receive another packet.
    if(ep_num == 0)
    {
      USBOEPCNT_0 &= ~NAK;
    }
    else
    {
      ep_regs[BCTX] &= ~NAK;
    }
  }
}

static void transmit_packet(uint8_t ep_num)
{
  xfer_ctl_t * xfer = XFER_CTL_BASE(ep_num, TUSB_DIR_IN);

  // First, determine whether we should even send a packet or finish
  // up the xfer.
  bool zlp = (xfer->total_len == 0); // By necessity, xfer->total_len will
                                     // equal xfer->queued_len for ZLPs.
                                     // Of course a ZLP is a short packet.
  if((!zlp && (xfer->total_len == xfer->queued_len)) || xfer->short_packet)
  {
    dcd_event_xfer_complete(0, ep_num | TUSB_DIR_IN_MASK, xfer->queued_len, XFER_RESULT_SUCCESS, true);
    return;
  }

  // Then actually commit to transmit a packet.
  uint8_t * base = (xfer->buffer + xfer->queued_len);
  uint16_t remaining = xfer->total_len - xfer->queued_len;
  uint8_t xfer_size = (xfer->max_size < xfer->total_len) ? xfer->max_size : remaining;

  xfer->queued_len += xfer_size;
  if(xfer_size < xfer->max_size)
  {
    // Next "xfer complete interrupt", the transfer will end.
    xfer->short_packet = true;
  }

  if(ep_num == 0)
  {
    volatile uint8_t * ep0in_buf = &USBIEP0BUF;
    for(uint16_t i = 0; i < xfer_size; i++)
    {
      ep0in_buf[i] = base[i];
    }

    USBIEPCNT_0 = (USBIEPCNT_0 & 0xF0) + xfer_size;
    USBIEPCNT_0 &= ~NAK;
  }
  else
  {
    ep_regs_t ep_regs = EP_REGS(ep_num, TUSB_DIR_IN);
    volatile uint8_t * ep_buf = &USBSTABUFF + (ep_regs[BBAX] << 3);

#if 0 // TODO support dcd_edpt_xfer_fifo API
    if (xfer->ff)
    {
      tu_fifo_read_n(xfer->ff, (void *) ep_buf, xfer_size);
    }
    else
#endif
    {
      for(int i = 0; i < xfer_size; i++)
      {
        ep_buf[i] = base[i];
      }
    }

    ep_regs[BCTX] = (ep_regs[BCTX] & 0x80) + (xfer_size & 0x7F);
    ep_regs[BCTX] &= ~NAK;
  }
}

static void handle_setup_packet(void)
{
  volatile uint8_t * setup_buf = &USBSUBLK;

  for(int i = 0; i < 8; i++)
  {
    _setup_packet[i] = setup_buf[i];
  }

  // Force NAKs until tinyusb can handle the SETUP packet and prepare for a new xfer.
  USBIEPCNT_0 |= NAK;
  USBOEPCNT_0 |= NAK;

  // Clear SETUPIFG to avoid handling in the USBVECINT switch statement.
  // When handled there the NAKs applied to the endpoints above are
  // cleared by hardware and the host will receive stale/duplicate data.
  //
  // Excerpt from MSP430x5xx and MSP430x6xx Family User's Guide:
  //
  // "...the SETUPIFG is cleared upon reading USBIV. In addition, the NAK on
  // input endpoint 0 and output endpoint 0 is also cleared."
  USBIEPCNF_0 &= ~UBME; // Errata USB10 workaround.
  USBOEPCNF_0 &= ~UBME; // Errata USB10 workaround.
  USBIFG &= ~SETUPIFG;
  USBIEPCNF_0 |= UBME;  // Errata USB10 workaround.
  USBOEPCNF_0 |= UBME;  // Errata USB10 workaround.
  dcd_event_setup_received(0, (uint8_t*) &_setup_packet[0], true);
}

#if CFG_TUSB_OS == OPT_OS_NONE
TU_ATTR_ALWAYS_INLINE static inline void tu_delay(uint32_t ms) {
  // msp430 can run up to 25Mhz -> 40ns per cycle. 1 ms = 25000 cycles
  // each loop need 4 cycle: 1 sub, 1 cmp, 1 jump, 1 nop
  volatile uint32_t cycles = (25000 * ms) >> 2;
  while (cycles > 0) {
    cycles--;
    asm("nop");
  }
}
#else
#define tu_delay(ms) osal_task_delay(ms)
#endif

static void handle_bus_power_event(void *param) {
  (void) param;

  tu_delay(5);                 // Bus power settling delay.

  USBKEYPID = USBKEY;

  if(USBPWRCTL & USBBGVBV) {          // Event caused by application of bus power.
    USBPWRCTL |= VBOFFIE;             // Enable bus-power-removed interrupt.
    USBPLLDIVB = USBPLLDIVB;          // For some reason the PLL will *NOT* lock unless the divider
                                      // register is re-written. The assumption here is that this
                                      // register was already properly configured during board-level
                                      // initialization.
    USBPLLCTL |= (UPLLEN | UPFDEN);   // Enable the PLL.

    uint16_t attempts = 0;
    do {                              // Poll the PLL, checking for a successful lock.
      USBPLLIR = 0;
      tu_delay(1);
      attempts++;
    } while ((attempts < 10) && (USBPLLIR != 0));

    // A successful lock is indicated by all PLL-related interrupt flags being cleared.
    if(!USBPLLIR) {
      const tusb_rhport_init_t rhport_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL
      };
      dcd_init(0, &rhport_init);         // Re-initialize the USB module.
    }
  } else {                            // Event caused by removal of bus power.
    USBPWRCTL |= VBONIE;              // Enable bus-power-applied interrupt.
    USBPLLCTL &= ~(UPLLEN | UPFDEN);  // Disable the PLL.
    USBCNF = 0;                       // Disable the USB module.
    dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, false);
  }

  USBKEYPID = 0;
}

void dcd_int_handler(uint8_t rhport)
{
  (void) rhport;

  // Setup is special- reading USBVECINT to handle setup packets is done to
  // stop hardware-generated NAKs on EP0.
  uint8_t setup_status = USBIFG & SETUPIFG;

  if(setup_status)
  {
    enable_functional_reset(true);
    handle_setup_packet();
  }

  // Workaround possible bug in MSP430 GCC 9.3.0 where volatile variable
  // USBVECINT is read from twice when only once is intended. The second
  // (garbage) read seems to be triggered by certain switch statement
  // configurations.
  uint16_t curr_vector;
  #if __GNUC__ > 9 || (__GNUC__ == 9 && __GNUC_MINOR__ > 2)
    asm volatile ("mov %1, %0"
                  : "=r" (curr_vector)
                  : "m" (USBVECINT));
  #else
    curr_vector = USBVECINT;
  #endif

  switch(curr_vector)
  {
    case USBVECINT_NONE:
      break;

    case USBVECINT_RSTR:
      enable_functional_reset(false); // Errata USB4 workaround.
      bus_reset();
      dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
      break;

    case USBVECINT_PWR_VBUSOn:
    case USBVECINT_PWR_VBUSOff: {
      USBKEYPID = USBKEY;
      // Prevent (possibly) unstable power from generating spurious interrupts.
      USBPWRCTL &= ~(VBONIE | VBOFFIE);
      USBKEYPID = 0;

      dcd_event_t event;

      event.rhport = 0;
      event.event_id = USBD_EVENT_FUNC_CALL;
      event.func_call.func = handle_bus_power_event;

      dcd_event_handler(&event, true);
      }
      break;

    // Clear the (hardware-enforced) NAK on EP 0 after a SETUP packet
    // is received. At this point, even though the hardware is no longer
    // forcing NAKs, the EP0 NAK bits should still be set to avoid
    // sending/receiving data before tinyusb is ready.
    //
    // Furthermore, it's possible for the hardware to STALL in the middle of
    // a control xfer if the EP0 NAK bits aren't set properly.
    // See: https://e2e.ti.com/support/microcontrollers/msp430/f/166/t/845259
    // From my testing, if all of the following hold:
    // * OUT EP0 NAK is cleared.
    // * IN EP0 NAK is set.
    // * DIR bit in USBCTL is clear.
    // and an IN packet is received on EP0, the USB core will STALL. Setting
    // both EP0 NAKs manually when a SETUP packet is received, as is done
    // in handle_setup_packet(), avoids meeting STALL conditions.
    //
    // TODO: Figure out/explain why the STALL condition can be reached in the
    // first place. When I first noticed the STALL, the only two places I
    // touched the NAK bits were in dcd_edpt_xfer() and to _set_ (sic) them in
    // bus_reset(). SETUP packet handling should've been unaffected.
    case USBVECINT_SETUP_PACKET_RECEIVED:
      break;

    case USBVECINT_INPUT_ENDPOINT0:
      enable_functional_reset(true);
      transmit_packet(0);
      break;

    case USBVECINT_OUTPUT_ENDPOINT0:
      enable_functional_reset(true);
      receive_packet(0);
      break;

    case USBVECINT_INPUT_ENDPOINT1:
    case USBVECINT_INPUT_ENDPOINT2:
    case USBVECINT_INPUT_ENDPOINT3:
    case USBVECINT_INPUT_ENDPOINT4:
    case USBVECINT_INPUT_ENDPOINT5:
    case USBVECINT_INPUT_ENDPOINT6:
    case USBVECINT_INPUT_ENDPOINT7:
    {
      uint8_t ep = ((curr_vector - USBVECINT_INPUT_ENDPOINT1) >> 1) + 1;
      transmit_packet(ep);
    }
    break;

    case USBVECINT_OUTPUT_ENDPOINT1:
    case USBVECINT_OUTPUT_ENDPOINT2:
    case USBVECINT_OUTPUT_ENDPOINT3:
    case USBVECINT_OUTPUT_ENDPOINT4:
    case USBVECINT_OUTPUT_ENDPOINT5:
    case USBVECINT_OUTPUT_ENDPOINT6:
    case USBVECINT_OUTPUT_ENDPOINT7:
    {
      uint8_t ep = ((curr_vector - USBVECINT_OUTPUT_ENDPOINT1) >> 1) + 1;
      receive_packet(ep);
    }
    break;

    default:
      while(true);
  }

}

#endif
