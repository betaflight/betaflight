#include "cdc_vcp_ch32h41x.h"
#include "ch32h417.h"

struct cdc_line_coding cdc_vcp_line_coding = {
    .dwDTERate = 115200, .bDataBits = 8, .bCharFormat = 0, .bParityType = 0};

static const uint8_t device_descriptor[] = {USB_DEVICE_DESCRIPTOR_INIT(
    USB_2_0, 0x02, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01)};

static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01,
                               USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS,
                        0x04)};

static const uint8_t device_quality_descriptor[] = {
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00, 0x02,
    0x00, 0x00,
    0x00, 0x40,
    0x01, 0x00,
};

static const char *string_descriptors[] = {
    (const char[]){0x09, 0x04}, /* Langid */
    "Betaflight",               /* Manufacturer */
    "Betaflight CH32H415",      /* Product */
    "2025123456",               /* Serial Number */
    "WCH HS Virtual COM Port",  /* interface */
};

static const uint8_t *device_descriptor_callback(uint8_t speed) {
  UNUSED(speed);
  return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed) {
  UNUSED(speed);
  return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed) {
  UNUSED(speed);
  return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index) {
  if (index > 4) {
    return NULL;
  }
  UNUSED(speed);
  return string_descriptors[index];
}

const struct usb_descriptor cdc_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback};

#define RX_BUFFER_SIZE 2048
#define TX_BUFFER_SIZE 512
// ep DMA addr
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t usb_drivers_buffer[CDC_MAX_MPS];

// ep DMA addr
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[RX_BUFFER_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[TX_BUFFER_SIZE];

volatile uint8_t ep_tx_busy_flag = 0;
volatile uint8_t usbd_cdc_info = 0;
volatile uint8_t ep_rx_finish = 0;
volatile uint16_t ep_rx_length = 0;

static volatile uint16_t rx_head = 0; // write p
static volatile uint16_t rx_tail = 0; // read p

static void usbd_event_handler(uint8_t busid, uint8_t event) {
  switch (event) {
  case USBD_EVENT_RESET:
    usbd_cdc_info = (uint8_t)USBD_EVENT_RESET;
    break;
  case USBD_EVENT_CONNECTED:
    usbd_cdc_info = (uint8_t)USBD_EVENT_CONNECTED;
    break;
  case USBD_EVENT_DISCONNECTED:
    usbd_cdc_info = (uint8_t)USBD_EVENT_DISCONNECTED;
    break;
  case USBD_EVENT_RESUME:
    usbd_cdc_info = (uint8_t)USBD_EVENT_RESUME;
    break;
  case USBD_EVENT_SUSPEND:
    usbd_cdc_info = (uint8_t)USBD_EVENT_SUSPEND;
    break;
  case USBD_EVENT_CONFIGURED:
    usbd_cdc_info = (uint8_t)USBD_EVENT_CONFIGURED;
    ep_tx_busy_flag = 0;
    ep_rx_finish = 0;
    /* setup first out ep read transfer */
    usbd_ep_start_read(busid, CDC_OUT_EP, usb_drivers_buffer, CDC_MAX_MPS);
    break;
  case USBD_EVENT_SET_REMOTE_WAKEUP:
    // usbd_cdc_info = (uint8_t)USBD_EVENT_SET_REMOTE_WAKEUP;
    break;
  case USBD_EVENT_CLR_REMOTE_WAKEUP:
    // usbd_cdc_info = (uint8_t)USBD_EVENT_CLR_REMOTE_WAKEUP;
    break;

  default:
    break;
  }
}

// ep receive call
void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes) {
#if 0
    ep_rx_finish = 1;
    ep_rx_length = nbytes;
#else
  for (uint32_t i = 0; i < nbytes; i++) {
    uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
    if (next_head == rx_tail) {
      // if full ,disard the neww data
      break;
    }
    read_buffer[rx_head] = usb_drivers_buffer[i];
    rx_head = next_head;
  }
  usbd_ep_start_read(busid, CDC_OUT_EP, usb_drivers_buffer,
                     CDC_MAX_MPS); // resume receive
  UNUSED(ep);
#endif
}

// ep send call
void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes) {
  // USB_LOG_RAW("actual in len:%d\r\n", (unsigned int)nbytes);

  // zero length control by app
  //  if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
  //      /* send zlp */
  //      usbd_ep_start_write(busid, CDC_IN_EP, NULL, 0);
  //  } else
  UNUSED(busid);
  UNUSED(ep);
  UNUSED(nbytes);
  {
    ep_tx_busy_flag = 0; // send finish
  }
}

/*!< endpoint call back */
struct usbd_endpoint cdc_out_ep = {.ep_addr = CDC_OUT_EP,
                                   .ep_cb = usbd_cdc_acm_bulk_out};

struct usbd_endpoint cdc_in_ep = {.ep_addr = CDC_IN_EP,
                                  .ep_cb = usbd_cdc_acm_bulk_in};

static struct usbd_interface intf0;

void cdc_acm_init(uint8_t busid, uintptr_t reg_base) {
#ifdef CONFIG_USBDEV_ADVANCE_DESC
  usbd_desc_register(busid, &cdc_descriptor);
#endif
  usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
  // usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
  usbd_add_endpoint(busid, &cdc_out_ep);
  usbd_add_endpoint(busid, &cdc_in_ep);
  usbd_initialize(busid, reg_base, usbd_event_handler);
}

void usbd_cdc_acm_set_line_coding(uint8_t busid, uint8_t intf,
                                  struct cdc_line_coding *line_coding) {
  (void)busid;
  cdc_vcp_line_coding.bCharFormat = line_coding->bCharFormat;
  cdc_vcp_line_coding.bDataBits = line_coding->bDataBits;
  cdc_vcp_line_coding.bParityType = line_coding->bParityType;
  cdc_vcp_line_coding.dwDTERate = line_coding->dwDTERate;
}

void usbd_cdc_acm_get_line_coding(uint8_t busid, uint8_t intf,
                                  struct cdc_line_coding *line_coding) {
  (void)busid;
  (void)intf;

  line_coding->dwDTERate = cdc_vcp_line_coding.dwDTERate;
  line_coding->bDataBits = cdc_vcp_line_coding.bDataBits;
  line_coding->bParityType = cdc_vcp_line_coding.bParityType;
  line_coding->bCharFormat = cdc_vcp_line_coding.bCharFormat;
}

uint16_t usb_vcp_rx_available(void) {
  if (rx_head >= rx_tail)
    return rx_head - rx_tail;
  else
    return RX_BUFFER_SIZE - rx_tail + rx_head;
}

// uint16_t usb_vcp_get_rx_data(uint8_t busid,uint8_t *buffer)
// {
//     //receiving
//     uint16_t tmp_len = 0;
//     if(ep_rx_finish == 0)  return 0;
//     ep_rx_finish = 0;
//     tmp_len = ep_rx_length;
//     memcpy(buffer,read_buffer,tmp_len);
//     usbd_ep_start_read(busid, CDC_OUT_EP, read_buffer, CDC_MAX_MPS); //resume
//     receive return tmp_len;
// }

uint16_t usb_vcp_get_rx_data(uint8_t busid, uint8_t *buffer, uint32_t len) {
  uint32_t count = 0;

  count = usb_vcp_rx_available();

  if (count > len)
    count = len;

  for (uint32_t i = 0; i < count; i++) {
    buffer[i] = read_buffer[rx_tail];
    rx_tail++;
    if (rx_tail >= RX_BUFFER_SIZE)
      rx_tail = 0;
  }
  return count;
}

uint16_t usb_vcp_send_data(uint8_t busid, uint8_t *buffer, uint16_t len) {
  if (ep_tx_busy_flag == 0) {
    ep_tx_busy_flag = 1;
    memcpy(write_buffer, buffer, len);
    usbd_ep_start_write(busid, CDC_IN_EP, write_buffer, len);
  } else {
    return 1;
  }
  return 0;
}
