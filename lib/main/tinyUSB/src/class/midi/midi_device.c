/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#if (CFG_TUD_ENABLED && CFG_TUD_MIDI)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "midi_device.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

typedef struct {
  uint8_t buffer[4];
  uint8_t index;
  uint8_t total;
} midid_stream_t;

typedef struct {
  uint8_t itf_num;
  uint8_t ep_in;
  uint8_t ep_out;

  // For Stream read()/write() API
  // Messages are always 4 bytes long, queue them for reading and writing so the
  // callers can use the Stream interface with single-byte read/write calls.
  midid_stream_t stream_write;
  midid_stream_t stream_read;

  /*------------- From this point, data is not cleared by bus reset -------------*/
  // FIFO
  tu_fifo_t rx_ff;
  tu_fifo_t tx_ff;
  uint8_t rx_ff_buf[CFG_TUD_MIDI_RX_BUFSIZE];
  uint8_t tx_ff_buf[CFG_TUD_MIDI_TX_BUFSIZE];

  #if CFG_FIFO_MUTEX
  osal_mutex_def_t rx_ff_mutex;
  osal_mutex_def_t tx_ff_mutex;
  #endif
} midid_interface_t;

#define ITF_MEM_RESET_SIZE   offsetof(midid_interface_t, rx_ff)

// Endpoint Transfer buffer
CFG_TUD_MEM_SECTION static struct {
  TUD_EPBUF_DEF(epin, CFG_TUD_MIDI_EP_BUFSIZE);
  TUD_EPBUF_DEF(epout, CFG_TUD_MIDI_EP_BUFSIZE);
} _midid_epbuf[CFG_TUD_MIDI];

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
static midid_interface_t _midid_itf[CFG_TUD_MIDI];

bool tud_midi_n_mounted (uint8_t itf) {
  midid_interface_t* midi = &_midid_itf[itf];
  return midi->ep_in && midi->ep_out;
}

static void _prep_out_transaction(uint8_t idx) {
  const uint8_t rhport = 0;
  midid_interface_t* p_midi = &_midid_itf[idx];
  uint16_t available = tu_fifo_remaining(&p_midi->rx_ff);

  // Prepare for incoming data but only allow what we can store in the ring buffer.
  // TODO Actually we can still carry out the transfer, keeping count of received bytes
  // and slowly move it to the FIFO when read().
  // This pre-check reduces endpoint claiming
  TU_VERIFY(available >= CFG_TUD_MIDI_EP_BUFSIZE, );

  // claim endpoint
  TU_VERIFY(usbd_edpt_claim(rhport, p_midi->ep_out), );

  // fifo can be changed before endpoint is claimed
  available = tu_fifo_remaining(&p_midi->rx_ff);

  if ( available >= CFG_TUD_MIDI_EP_BUFSIZE )  {
    usbd_edpt_xfer(rhport, p_midi->ep_out, _midid_epbuf[idx].epout, CFG_TUD_MIDI_EP_BUFSIZE);
  }else
  {
    // Release endpoint since we don't make any transfer
    usbd_edpt_release(rhport, p_midi->ep_out);
  }
}

//--------------------------------------------------------------------+
// READ API
//--------------------------------------------------------------------+
uint32_t tud_midi_n_available(uint8_t itf, uint8_t cable_num)
{
  (void) cable_num;

  midid_interface_t* midi = &_midid_itf[itf];
  const midid_stream_t* stream = &midi->stream_read;

  // when using with packet API stream total & index are both zero
  return tu_fifo_count(&midi->rx_ff) + (uint8_t) (stream->total - stream->index);
}

uint32_t tud_midi_n_stream_read(uint8_t itf, uint8_t cable_num, void* buffer, uint32_t bufsize)
{
  (void) cable_num;
  TU_VERIFY(bufsize, 0);

  uint8_t* buf8 = (uint8_t*) buffer;

  midid_interface_t* midi = &_midid_itf[itf];
  midid_stream_t* stream = &midi->stream_read;

  uint32_t total_read = 0;
  while( bufsize )
  {
    // Get new packet from fifo, then set packet expected bytes
    if ( stream->total == 0 )
    {
      // return if there is no more data from fifo
      if ( !tud_midi_n_packet_read(itf, stream->buffer) ) return total_read;

      uint8_t const code_index = stream->buffer[0] & 0x0f;

      // MIDI 1.0 Table 4-1: Code Index Number Classifications
      switch(code_index)
      {
        case MIDI_CIN_MISC:
        case MIDI_CIN_CABLE_EVENT:
          // These are reserved and unused, possibly issue somewhere, skip this packet
          return 0;
        break;

        case MIDI_CIN_SYSEX_END_1BYTE:
        case MIDI_CIN_1BYTE_DATA:
          stream->total = 1;
        break;

        case MIDI_CIN_SYSCOM_2BYTE     :
        case MIDI_CIN_SYSEX_END_2BYTE  :
        case MIDI_CIN_PROGRAM_CHANGE   :
        case MIDI_CIN_CHANNEL_PRESSURE :
          stream->total = 2;
        break;

        default:
          stream->total = 3;
        break;
      }
    }

    // Copy data up to bufsize
    uint8_t const count = (uint8_t) tu_min32(stream->total - stream->index, bufsize);

    // Skip the header (1st byte) in the buffer
    TU_VERIFY(0 == tu_memcpy_s(buf8, bufsize, stream->buffer + 1 + stream->index, count));

    total_read += count;
    stream->index += count;
    buf8 += count;
    bufsize -= count;

    // complete current event packet, reset stream
    if ( stream->total == stream->index )
    {
      stream->index = 0;
      stream->total = 0;
    }
  }

  return total_read;
}

bool tud_midi_n_packet_read (uint8_t itf, uint8_t packet[4])
{
  midid_interface_t* midi = &_midid_itf[itf];
  TU_VERIFY(midi->ep_out);

  const uint32_t num_read = tu_fifo_read_n(&midi->rx_ff, packet, 4);
  _prep_out_transaction(itf);
  return (num_read == 4);
}

//--------------------------------------------------------------------+
// WRITE API
//--------------------------------------------------------------------+

static uint32_t write_flush(uint8_t idx) {
  midid_interface_t* midi = &_midid_itf[idx];

  if (!tu_fifo_count(&midi->tx_ff)) {
    return 0; // No data to send
  }

  const uint8_t rhport = 0;

  // skip if previous transfer not complete
  TU_VERIFY( usbd_edpt_claim(rhport, midi->ep_in), 0 );

  uint16_t count = tu_fifo_read_n(&midi->tx_ff, _midid_epbuf[idx].epin, CFG_TUD_MIDI_EP_BUFSIZE);

  if (count) {
    TU_ASSERT( usbd_edpt_xfer(rhport, midi->ep_in, _midid_epbuf[idx].epin, count), 0 );
    return count;
  }else {
    // Release endpoint since we don't make any transfer
    usbd_edpt_release(rhport, midi->ep_in);
    return 0;
  }
}

uint32_t tud_midi_n_stream_write(uint8_t itf, uint8_t cable_num, const uint8_t* buffer, uint32_t bufsize)
{
  midid_interface_t* midi = &_midid_itf[itf];
  TU_VERIFY(midi->ep_in, 0);

  midid_stream_t* stream = &midi->stream_write;

  uint32_t i = 0;
  while ( (i < bufsize) && (tu_fifo_remaining(&midi->tx_ff) >= 4) )
  {
    const uint8_t data = buffer[i];
    i++;

    if ( stream->index == 0 )
    {
      //------------- New event packet -------------//
      const uint8_t msg = data >> 4;

      stream->index = 2;
      stream->buffer[1] = data;

      // Check to see if we're still in a SysEx transmit.
      if ( ((stream->buffer[0]) & 0xF) == MIDI_CIN_SYSEX_START )
      {
        if ( data == MIDI_STATUS_SYSEX_END )
        {
          stream->buffer[0] = (uint8_t) ((cable_num << 4) | MIDI_CIN_SYSEX_END_1BYTE);
          stream->total = 2;
        }
        else
        {
          stream->total = 4;
        }
      }
      else if ( (msg >= 0x8 && msg <= 0xB) || msg == 0xE )
      {
        // Channel Voice Messages
        stream->buffer[0] = (uint8_t) ((cable_num << 4) | msg);
        stream->total = 4;
      }
      else if ( msg == 0xC || msg == 0xD)
      {
        // Channel Voice Messages, two-byte variants (Program Change and Channel Pressure)
        stream->buffer[0] = (uint8_t) ((cable_num << 4) | msg);
        stream->total = 3;
      }
      else if ( msg == 0xf )
      {
        // System message
        if ( data == MIDI_STATUS_SYSEX_START )
        {
          stream->buffer[0] = MIDI_CIN_SYSEX_START;
          stream->total = 4;
        }
        else if ( data == MIDI_STATUS_SYSCOM_TIME_CODE_QUARTER_FRAME || data == MIDI_STATUS_SYSCOM_SONG_SELECT )
        {
          stream->buffer[0] = MIDI_CIN_SYSCOM_2BYTE;
          stream->total = 3;
        }
        else if ( data == MIDI_STATUS_SYSCOM_SONG_POSITION_POINTER )
        {
          stream->buffer[0] = MIDI_CIN_SYSCOM_3BYTE;
          stream->total = 4;
        }
        else
        {
          stream->buffer[0] = MIDI_CIN_SYSEX_END_1BYTE;
          stream->total = 2;
        }
        stream->buffer[0] |= (uint8_t)(cable_num << 4);
      }
      else
      {
        // Pack individual bytes if we don't support packing them into words.
        stream->buffer[0] = (uint8_t) (cable_num << 4 | 0xf);
        stream->buffer[2] = 0;
        stream->buffer[3] = 0;
        stream->index = 2;
        stream->total = 2;
      }
    }
    else
    {
      //------------- On-going (buffering) packet -------------//

      TU_ASSERT(stream->index < 4, i);
      stream->buffer[stream->index] = data;
      stream->index++;

      // See if this byte ends a SysEx.
      if ( (stream->buffer[0] & 0xF) == MIDI_CIN_SYSEX_START && data == MIDI_STATUS_SYSEX_END )
      {
        stream->buffer[0] = (uint8_t) ((cable_num << 4) | (MIDI_CIN_SYSEX_START + (stream->index - 1)));
        stream->total = stream->index;
      }
    }

    // Send out packet
    if ( stream->index == stream->total )
    {
      // zeroes unused bytes
      for (uint8_t idx = stream->total; idx < 4; idx++) {
        stream->buffer[idx] = 0;
      }

      const uint16_t count = tu_fifo_write_n(&midi->tx_ff, stream->buffer, 4);

      // complete current event packet, reset stream
      stream->index = stream->total = 0;

      // FIFO overflown, since we already check fifo remaining. It is probably race condition
      TU_ASSERT(count == 4, i);
    }
  }

  write_flush(itf);

  return i;
}

bool tud_midi_n_packet_write (uint8_t itf, const uint8_t packet[4]) {
  midid_interface_t* midi = &_midid_itf[itf];
  TU_VERIFY(midi->ep_in);

  if (tu_fifo_remaining(&midi->tx_ff) < 4) {
    return false;
  }

  tu_fifo_write_n(&midi->tx_ff, packet, 4);
  write_flush(itf);

  return true;
}

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void midid_init(void) {
  tu_memclr(_midid_itf, sizeof(_midid_itf));

  for (uint8_t i = 0; i < CFG_TUD_MIDI; i++) {
    midid_interface_t* midi = &_midid_itf[i];

    // config fifo
    tu_fifo_config(&midi->rx_ff, midi->rx_ff_buf, CFG_TUD_MIDI_RX_BUFSIZE, 1, false); // true, true
    tu_fifo_config(&midi->tx_ff, midi->tx_ff_buf, CFG_TUD_MIDI_TX_BUFSIZE, 1, false); // OBVS.

    #if CFG_FIFO_MUTEX
    osal_mutex_t mutex_rd = osal_mutex_create(&midi->rx_ff_mutex);
    osal_mutex_t mutex_wr = osal_mutex_create(&midi->tx_ff_mutex);
    TU_ASSERT(mutex_wr != NULL && mutex_wr != NULL, );

    tu_fifo_config_mutex(&midi->rx_ff, NULL, mutex_rd);
    tu_fifo_config_mutex(&midi->tx_ff, mutex_wr, NULL);
    #endif
  }
}

bool midid_deinit(void) {
  #if CFG_FIFO_MUTEX
  for(uint8_t i=0; i<CFG_TUD_MIDI; i++) {
    midid_interface_t* midi = &_midid_itf[i];
    osal_mutex_t mutex_rd = midi->rx_ff.mutex_rd;
    osal_mutex_t mutex_wr = midi->tx_ff.mutex_wr;

    if (mutex_rd) {
      osal_mutex_delete(mutex_rd);
      tu_fifo_config_mutex(&midi->rx_ff, NULL, NULL);
    }

    if (mutex_wr) {
      osal_mutex_delete(mutex_wr);
      tu_fifo_config_mutex(&midi->tx_ff, NULL, NULL);
    }
  }
  #endif

  return true;
}

void midid_reset(uint8_t rhport)
{
  (void) rhport;

  for(uint8_t i=0; i<CFG_TUD_MIDI; i++)
  {
    midid_interface_t* midi = &_midid_itf[i];
    tu_memclr(midi, ITF_MEM_RESET_SIZE);
    tu_fifo_clear(&midi->rx_ff);
    tu_fifo_clear(&midi->tx_ff);
  }
}

uint16_t midid_open(uint8_t rhport, const tusb_desc_interface_t* desc_itf, uint16_t max_len)
{
  // 1st Interface is Audio Control v1
  TU_VERIFY(TUSB_CLASS_AUDIO               == desc_itf->bInterfaceClass    &&
            AUDIO_SUBCLASS_CONTROL         == desc_itf->bInterfaceSubClass &&
            AUDIO_FUNC_PROTOCOL_CODE_UNDEF == desc_itf->bInterfaceProtocol, 0);

  uint16_t drv_len = tu_desc_len(desc_itf);
  const uint8_t* p_desc = tu_desc_next(desc_itf);

  // Skip Class Specific descriptors
  while ( TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len )
  {
    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  // 2nd Interface is MIDI Streaming
  TU_VERIFY(TUSB_DESC_INTERFACE == tu_desc_type(p_desc), 0);
  const tusb_desc_interface_t* desc_midi = (const tusb_desc_interface_t*) p_desc;

  TU_VERIFY(TUSB_CLASS_AUDIO               == desc_midi->bInterfaceClass    &&
            AUDIO_SUBCLASS_MIDI_STREAMING  == desc_midi->bInterfaceSubClass &&
            AUDIO_FUNC_PROTOCOL_CODE_UNDEF == desc_midi->bInterfaceProtocol, 0);

  // Find available interface
  midid_interface_t * p_midi = NULL;
  uint8_t idx;
  for(idx=0; idx<CFG_TUD_MIDI; idx++) {
    if ( _midid_itf[idx].ep_in == 0 && _midid_itf[idx].ep_out == 0 ) {
      p_midi = &_midid_itf[idx];
      break;
    }
  }
  TU_ASSERT(p_midi);

  p_midi->itf_num = desc_midi->bInterfaceNumber;
  (void) p_midi->itf_num;

  // next descriptor
  drv_len += tu_desc_len(p_desc);
  p_desc   = tu_desc_next(p_desc);

  // Find and open endpoint descriptors
  uint8_t found_endpoints = 0;
  while ( (found_endpoints < desc_midi->bNumEndpoints) && (drv_len <= max_len)  )
  {
    if ( TUSB_DESC_ENDPOINT == tu_desc_type(p_desc) )
    {
      TU_ASSERT(usbd_edpt_open(rhport, (const tusb_desc_endpoint_t*) p_desc), 0);
      uint8_t ep_addr = ((const tusb_desc_endpoint_t*) p_desc)->bEndpointAddress;

      if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN)
      {
        p_midi->ep_in = ep_addr;
      } else {
        p_midi->ep_out = ep_addr;
      }

      // Class Specific MIDI Stream endpoint descriptor
      drv_len += tu_desc_len(p_desc);
      p_desc   = tu_desc_next(p_desc);

      found_endpoints += 1;
    }

    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  // Prepare for incoming data
  _prep_out_transaction(idx);

  return drv_len;
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool midid_control_xfer_cb(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
  (void) rhport; (void) stage; (void) request;
  return false; // driver doesn't support any request yet
}

bool midid_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void) result;
  (void) rhport;

  uint8_t idx;
  midid_interface_t* p_midi;

  // Identify which interface to use
  for (idx = 0; idx < CFG_TUD_MIDI; idx++) {
    p_midi = &_midid_itf[idx];
    if ((ep_addr == p_midi->ep_out) || (ep_addr == p_midi->ep_in)) {
      break;
    }
  }
  TU_ASSERT(idx < CFG_TUD_MIDI);

  // receive new data
  if (ep_addr == p_midi->ep_out) {
    tu_fifo_write_n(&p_midi->rx_ff, _midid_epbuf[idx].epout, (uint16_t)xferred_bytes);

    // invoke receive callback if available
    if (tud_midi_rx_cb) {
      tud_midi_rx_cb(idx);
    }

    // prepare for next
    // TODO for now ep_out is not used by public API therefore there is no race condition,
    // and does not need to claim like ep_in
    _prep_out_transaction(idx);
  } else if (ep_addr == p_midi->ep_in) {
    if (0 == write_flush(idx)) {
      // If there is no data left, a ZLP should be sent if
      // xferred_bytes is multiple of EP size and not zero
      if (!tu_fifo_count(&p_midi->tx_ff) && xferred_bytes && (0 == (xferred_bytes % CFG_TUD_MIDI_EP_BUFSIZE))) {
        if (usbd_edpt_claim(rhport, p_midi->ep_in)) {
          usbd_edpt_xfer(rhport, p_midi->ep_in, NULL, 0);
        }
      }
    }
  }

  return true;
}

#endif
