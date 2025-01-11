/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Koji KITAYAMA
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

#if (CFG_TUD_ENABLED && CFG_TUD_VIDEO && CFG_TUD_VIDEO_STREAMING)

#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "video_device.h"

// Level where CFG_TUSB_DEBUG must be at least for this driver is logged
#ifndef CFG_TUD_VIDEO_LOG_LEVEL
  #define CFG_TUD_VIDEO_LOG_LEVEL   CFG_TUD_LOG_LEVEL
#endif

#define TU_LOG_DRV(...)   TU_LOG(CFG_TUD_VIDEO_LOG_LEVEL, __VA_ARGS__)

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
#define VS_STATE_PROBING      0     /* Configuration in progress */
#define VS_STATE_COMMITTED    1     /* Ready for streaming or Streaming via bulk endpoint */
#define VS_STATE_STREAMING    2     /* Streaming via isochronous endpoint */

typedef struct {
  tusb_desc_interface_t            std;
  tusb_desc_video_control_header_t ctl;
} tusb_desc_vc_itf_t;

typedef struct {
  tusb_desc_interface_t            std;
  tusb_desc_video_streaming_inout_header_t stm;
} tusb_desc_vs_itf_t;

typedef union {
  tusb_desc_video_control_header_t ctl;
  tusb_desc_video_streaming_inout_header_t stm;
} tusb_desc_video_itf_hdr_t;

typedef struct TU_ATTR_PACKED {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bEntityId;
} tusb_desc_cs_video_entity_itf_t;

typedef union {
  struct TU_ATTR_PACKED {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bFormatIndex;
    uint8_t bNumFrameDescriptors;
  };
  tusb_desc_video_format_uncompressed_t uncompressed;
  tusb_desc_video_format_mjpeg_t        mjpeg;
  tusb_desc_video_format_framebased_t  frame_based;
} tusb_desc_cs_video_fmt_t;

typedef union {
  struct TU_ATTR_PACKED {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint8_t  bFrameIndex;
    uint8_t  bmCapabilities;
    uint16_t wWidth;
    uint16_t wHeight;
  };
  tusb_desc_video_frame_uncompressed_t uncompressed;
  tusb_desc_video_frame_mjpeg_t        mjpeg;
  tusb_desc_video_frame_framebased_t  frame_based;
} tusb_desc_cs_video_frm_t;

/* video streaming interface */
typedef struct TU_ATTR_PACKED {
  uint8_t index_vc;  /* index of bound video control interface */
  uint8_t index_vs;  /* index from the video control interface */
  struct {
    uint16_t beg;    /* Offset of the begging of video streaming interface descriptor */
    uint16_t end;    /* Offset of the end of video streaming interface descriptor */
    uint16_t cur;    /* Offset of the current settings */
    uint16_t ep[2];  /* Offset of endpoint descriptors. 0: streaming, 1: still capture */
  } desc;
  uint8_t *buffer;   /* frame buffer. assume linear buffer. no support for stride access */
  uint32_t bufsize;  /* frame buffer size */
  uint32_t offset;   /* offset for the next payload transfer */
  uint32_t max_payload_transfer_size;
  uint8_t  error_code;/* error code */
  uint8_t  state;    /* 0:probing 1:committed 2:streaming */

  video_probe_and_commit_control_t probe_commit_payload; /* Probe and Commit control */
} videod_streaming_interface_t;

typedef struct {
  TUD_EPBUF_DEF(buf, CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE);
} videod_streaming_epbuf_t;

/* video control interface */
typedef struct TU_ATTR_PACKED {
  const uint8_t*beg;                     /* The head of the first video control interface descriptor */
  uint16_t len;                          /* Byte length of the descriptors */
  uint16_t cur;                          /* offset for current video control interface */
  uint8_t  stm[CFG_TUD_VIDEO_STREAMING]; /* Indices of streaming interface */
  uint8_t  error_code;                   /* error code */
  uint8_t  power_mode;
} videod_interface_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
static videod_interface_t _videod_itf[CFG_TUD_VIDEO];

static videod_streaming_interface_t _videod_streaming_itf[CFG_TUD_VIDEO_STREAMING];
CFG_TUD_MEM_SECTION static videod_streaming_epbuf_t _videod_streaming_epbuf[CFG_TUD_VIDEO_STREAMING];

static uint8_t const _cap_get     = 0x1u; /* support for GET */
static uint8_t const _cap_get_set = 0x3u; /* support for GET and SET */

//--------------------------------------------------------------------+
// Debug
//--------------------------------------------------------------------+
#if CFG_TUSB_DEBUG >= CFG_TUD_VIDEO_LOG_LEVEL

static tu_lookup_entry_t const tu_lookup_video_request[] = {
    {.key = VIDEO_REQUEST_UNDEFINED, .data = "Undefined"},
    {.key = VIDEO_REQUEST_SET_CUR, .data = "SetCur"},
    {.key = VIDEO_REQUEST_SET_CUR_ALL, .data = "SetCurAll"},
    {.key = VIDEO_REQUEST_GET_CUR, .data = "GetCur"},
    {.key = VIDEO_REQUEST_GET_MIN, .data = "GetMin"},
    {.key = VIDEO_REQUEST_GET_MAX, .data = "GetMax"},
    {.key = VIDEO_REQUEST_GET_RES, .data = "GetRes"},
    {.key = VIDEO_REQUEST_GET_LEN, .data = "GetLen"},
    {.key = VIDEO_REQUEST_GET_INFO, .data = "GetInfo"},
    {.key = VIDEO_REQUEST_GET_DEF, .data = "GetDef"},
    {.key = VIDEO_REQUEST_GET_CUR_ALL, .data = "GetCurAll"},
    {.key = VIDEO_REQUEST_GET_MIN_ALL, .data = "GetMinAll"},
    {.key = VIDEO_REQUEST_GET_MAX_ALL, .data = "GetMaxAll"},
    {.key = VIDEO_REQUEST_GET_RES_ALL, .data = "GetResAll"},
    {.key = VIDEO_REQUEST_GET_DEF_ALL, .data = "GetDefAll"},
};

static tu_lookup_table_t const tu_table_video_request = {
    .count = TU_ARRAY_SIZE(tu_lookup_video_request),
    .items = tu_lookup_video_request
};

static char const* const tu_str_video_vc_control_selector[] = {
    "Undefined",
    "Video Power Mode",
    "Request Error Code",
};

static char const* const tu_str_video_vs_control_selector[] = {
    "Undefined",
    "Probe",
    "Commit",
    "Still Probe",
    "Still Commit",
    "Still Image Trigger",
    "Stream Error Code",
    "Generate Key Frame",
    "Update Frame Segment",
    "Sync Delay",
};

#endif

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

/** Get interface number from the interface descriptor
 *
 * @param[in] desc    interface descriptor
 *
 * @return bInterfaceNumber */
static inline uint8_t _desc_itfnum(void const *desc) {
  return ((uint8_t const*)desc)[2];
}

/** Get endpoint address from the endpoint descriptor
 *
 * @param[in] desc    endpoint descriptor
 *
 * @return bEndpointAddress */
static inline uint8_t _desc_ep_addr(void const *desc) {
  return ((uint8_t const*)desc)[2];
}

/** Get instance of streaming interface
 *
 * @param[in] ctl_idx    instance number of video control
 * @param[in] stm_idx    index number of streaming interface
 *
 * @return instance */
static videod_streaming_interface_t* _get_instance_streaming(uint_fast8_t ctl_idx, uint_fast8_t stm_idx) {
  videod_interface_t *ctl = &_videod_itf[ctl_idx];
  if (!ctl->beg) return NULL;
  videod_streaming_interface_t *stm = &_videod_streaming_itf[ctl->stm[stm_idx]];
  if (!stm->desc.beg) return NULL;
  return stm;
}

static tusb_desc_vc_itf_t const* _get_desc_vc(videod_interface_t const *self) {
  return (tusb_desc_vc_itf_t const *)(self->beg + self->cur);
}

static tusb_desc_vs_itf_t const* _get_desc_vs(videod_streaming_interface_t const *self) {
  if (!self->desc.cur) return NULL;
  uint8_t const *desc = _videod_itf[self->index_vc].beg;
  return (tusb_desc_vs_itf_t const*)(desc + self->desc.cur);
}

/** Find the first descriptor of a given type
 *
 * @param[in] beg        The head of descriptor byte array.
 * @param[in] end        The tail of descriptor byte array.
 * @param[in] desc_type  The target descriptor type.
 *
 * @return The pointer for interface descriptor.
 * @retval end   did not found interface descriptor */
static void const* _find_desc(void const *beg, void const *end, uint_fast8_t desc_type) {
  void const *cur = beg;
  while ((cur < end) && (desc_type != tu_desc_type(cur))) {
    cur = tu_desc_next(cur);
  }
  return cur;
}

/** Find the first descriptor of two given types
 *
 * @param[in] beg        The head of descriptor byte array.
 * @param[in] end        The tail of descriptor byte array.
 * @param[in] desc_type_0 The first target descriptor type.
 * @param[in] desc_type_1 The second target descriptor type.
 *
 * @return The pointer for interface descriptor.
 * @retval end   did not found interface descriptor */
static void const* _find_desc_2_type(void const *beg, void const *end, uint_fast8_t desc_type_0, uint_fast8_t desc_type_1)
{
  void const *cur = beg;
  while ((cur < end) && (desc_type_0 != tu_desc_type(cur)) && (desc_type_1 != tu_desc_type(cur))) {
    cur = tu_desc_next(cur);
  }
  return cur;
}

/** Find the first descriptor specified by the arguments
 *
 * @param[in] beg        The head of descriptor byte array.
 * @param[in] end        The tail of descriptor byte array.
 * @param[in] desc_type  The target descriptor type
 * @param[in] element_0  The target element following the desc_type
 * @param[in] element_1  The target element following the element_0
 *
 * @return The pointer for interface descriptor.
 * @retval end   did not found interface descriptor */
static void const* _find_desc_3(void const *beg, void const *end,
                                uint_fast8_t desc_type,
                                uint_fast8_t element_0,
                                uint_fast8_t element_1) {
  for (void const *cur = beg; cur < end; cur = _find_desc(cur, end, desc_type)) {
    uint8_t const *p = (uint8_t const *)cur;
    if ((p[2] == element_0) && (p[3] == element_1)) {
      return cur;
    }
    cur = tu_desc_next(cur);
  }
  return end;
}

/** Return the next interface descriptor which has another interface number.
 *  If there are multiple VC interfaces, there will be an IAD descriptor before
 *  the next interface descriptor. Check both the IAD descriptor and the interface
 *  descriptor.
 *  3.1 Descriptor Layout Overview
 *
 * @param[in] beg     The head of descriptor byte array.
 * @param[in] end     The tail of descriptor byte array.
 *
 * @return The pointer for interface descriptor.
 * @retval end   did not found interface descriptor */
static void const* _next_desc_itf(void const *beg, void const *end) {
  void const *cur = beg;
  uint_fast8_t itfnum = ((tusb_desc_interface_t const*)cur)->bInterfaceNumber;
  while ((cur < end) &&
         (itfnum == ((tusb_desc_interface_t const*)cur)->bInterfaceNumber)) {
    cur = _find_desc_2_type(tu_desc_next(cur), end, TUSB_DESC_INTERFACE, TUSB_DESC_INTERFACE_ASSOCIATION);
  }
  return cur;
}

/** Find the first interface descriptor with the specified interface number and alternate setting number.
 *
 * @param[in] beg     The head of descriptor byte array.
 * @param[in] end     The tail of descriptor byte array.
 * @param[in] itfnum  The target interface number.
 * @param[in] altnum  The target alternate setting number.
 *
 * @return The pointer for interface descriptor.
 * @retval end   did not found interface descriptor */
static inline uint8_t const* _find_desc_itf(void const *beg, void const *end, uint_fast8_t itfnum, uint_fast8_t altnum)
{
  return (uint8_t const*) _find_desc_3(beg, end, TUSB_DESC_INTERFACE, itfnum, altnum);
}

/** Find the first endpoint descriptor belonging to the current interface descriptor.
 *
 * The search range is from `beg` to `end` or the next interface descriptor.
 *
 * @param[in] beg     The head of descriptor byte array.
 * @param[in] end     The tail of descriptor byte array.
 *
 * @return The pointer for endpoint descriptor.
 * @retval end   did not found endpoint descriptor */
static void const* _find_desc_ep(void const *beg, void const *end)
{
  for (void const *cur = beg; cur < end; cur = tu_desc_next(cur)) {
    uint_fast8_t desc_type = tu_desc_type(cur);
    if (TUSB_DESC_ENDPOINT == desc_type) return cur;
    if (TUSB_DESC_INTERFACE == desc_type) break;
  }
  return end;
}

/** Return the end of the video control descriptor. */
static inline void const* _end_of_control_descriptor(void const *desc)
{
  tusb_desc_vc_itf_t const *vc = (tusb_desc_vc_itf_t const *)desc;
  return ((uint8_t const*) desc) + vc->std.bLength + tu_le16toh(vc->ctl.wTotalLength);
}

/** Find the first entity descriptor with the entity ID
 *  specified by the argument belonging to the current video control descriptor.
 *
 * @param[in] desc      The video control interface descriptor.
 * @param[in] entityid  The target entity id.
 *
 * @return The pointer for interface descriptor.
 * @retval end   did not found interface descriptor */
static void const* _find_desc_entity(void const *desc, uint_fast8_t entityid)
{
  void const *end = _end_of_control_descriptor(desc);
  for (void const *cur = desc; cur < end; cur = _find_desc(cur, end, TUSB_DESC_CS_INTERFACE)) {
    tusb_desc_cs_video_entity_itf_t const *itf = (tusb_desc_cs_video_entity_itf_t const *)cur;
    if ((VIDEO_CS_ITF_VC_INPUT_TERMINAL  <= itf->bDescriptorSubtype
         && itf->bDescriptorSubtype < VIDEO_CS_ITF_VC_MAX)
        && itf->bEntityId == entityid) {
      return itf;
    }
    cur = tu_desc_next(cur);
  }
  return end;
}

/** Return the end of the video streaming descriptor. */
static inline void const* _end_of_streaming_descriptor(void const *desc)
{
  tusb_desc_vs_itf_t const *vs = (tusb_desc_vs_itf_t const *)desc;
  return ((uint8_t const*) desc) + vs->std.bLength + tu_le16toh(vs->stm.wTotalLength);
}

/** Find the first format descriptor with the specified format number. */
static inline void const *_find_desc_format(void const *beg, void const *end, uint_fast8_t fmtnum)
{
  for (void const *cur = beg; cur < end; cur = _find_desc(cur, end, TUSB_DESC_CS_INTERFACE)) {
    uint8_t const *p = (uint8_t const *)cur;
    uint_fast8_t fmt = p[2];
    if ((fmt == VIDEO_CS_ITF_VS_FORMAT_UNCOMPRESSED ||
         fmt == VIDEO_CS_ITF_VS_FORMAT_MJPEG ||
         fmt == VIDEO_CS_ITF_VS_FORMAT_DV ||
         fmt == VIDEO_CS_ITF_VS_FORMAT_FRAME_BASED) &&
        fmtnum == p[3]) {
      return cur;
    }
    cur = tu_desc_next(cur);
  }
  return end;
}

/** Find the first frame descriptor with the specified format number. */
static inline void const *_find_desc_frame(void const *beg, void const *end, uint_fast8_t frmnum)
{
  for (void const *cur = beg; cur < end; cur = _find_desc(cur, end, TUSB_DESC_CS_INTERFACE)) {
    uint8_t const *p = (uint8_t const *)cur;
    uint_fast8_t frm = p[2];
    if ((frm == VIDEO_CS_ITF_VS_FRAME_UNCOMPRESSED ||
         frm == VIDEO_CS_ITF_VS_FRAME_MJPEG ||
         frm == VIDEO_CS_ITF_VS_FRAME_FRAME_BASED) &&
        frmnum == p[3]) {
      return cur;
    }
    cur = tu_desc_next(cur);
  }
  return end;
}

/** Set uniquely determined values to variables that have not been set
 *
 * @param[in,out] param       Target */
static bool _update_streaming_parameters(videod_streaming_interface_t const *stm,
                                         video_probe_and_commit_control_t *param)
{
  tusb_desc_vs_itf_t const *vs = _get_desc_vs(stm);
  uint_fast8_t fmtnum = param->bFormatIndex;
  TU_ASSERT(vs && fmtnum <= vs->stm.bNumFormats);
  if (!fmtnum) {
    if (1 < vs->stm.bNumFormats) return true; /* Need to negotiate all variables. */
    fmtnum = 1;
    param->bFormatIndex = 1;
  }

  /* Set the parameters determined by the format  */
  param->wKeyFrameRate    = 1;
  param->wPFrameRate      = 0;
  param->wCompWindowSize  = 1; /* GOP size? */
  param->wDelay           = 0; /* milliseconds */
  param->dwClockFrequency = 27000000; /* same as MPEG-2 system time clock  */
  param->bmFramingInfo    = 0x3; /* enables FrameID and EndOfFrame */
  param->bPreferedVersion = 1;
  param->bMinVersion      = 1;
  param->bMaxVersion      = 1;
  param->bUsage           = 0;
  param->bBitDepthLuma    = 8;

  void const *end = _end_of_streaming_descriptor(vs);
  tusb_desc_cs_video_fmt_t const *fmt = _find_desc_format(tu_desc_next(vs), end, fmtnum);
  TU_ASSERT(fmt != end);

  switch (fmt->bDescriptorSubType) {
    case VIDEO_CS_ITF_VS_FORMAT_UNCOMPRESSED:
      param->wCompQuality = 1; /* 1 to 10000 */
      break;

    case VIDEO_CS_ITF_VS_FORMAT_MJPEG:
      break;

    case VIDEO_CS_ITF_VS_FORMAT_FRAME_BASED:
      break;

    default: return false;
  }

  uint_fast8_t frmnum = param->bFrameIndex;
  TU_ASSERT(frmnum <= fmt->bNumFrameDescriptors);
  if (!frmnum) {
    if (1 < fmt->bNumFrameDescriptors) return true;
    frmnum = 1;
    param->bFrameIndex = 1;
  }
  tusb_desc_cs_video_frm_t const *frm = _find_desc_frame(tu_desc_next(fmt), end, frmnum);
  TU_ASSERT(frm != end);

  /* Set the parameters determined by the frame  */
  uint_fast32_t frame_size = param->dwMaxVideoFrameSize;
  if (!frame_size) {
    switch (fmt->bDescriptorSubType) {
      case VIDEO_CS_ITF_VS_FORMAT_UNCOMPRESSED:
        frame_size = (uint_fast32_t)frm->wWidth * frm->wHeight * fmt->uncompressed.bBitsPerPixel / 8;
        break;

      case VIDEO_CS_ITF_VS_FORMAT_MJPEG:
        frame_size = (uint_fast32_t)frm->wWidth * frm->wHeight * 16 / 8; /* YUV422 */
        break;

      case VIDEO_CS_ITF_VS_FORMAT_FRAME_BASED:
        frame_size = (uint_fast32_t)frm->wWidth * frm->wHeight * 16 / 8; /* YUV422 */
        break;

      default: break;
    }
    param->dwMaxVideoFrameSize = frame_size;
  }

  uint_fast32_t interval = param->dwFrameInterval;
  if (!interval) {
    if ((1 < frm->uncompressed.bFrameIntervalType) ||
        ((0 == frm->uncompressed.bFrameIntervalType) &&
         (frm->uncompressed.dwFrameInterval[1] != frm->uncompressed.dwFrameInterval[0]))) {
      return true;
    }
    interval = frm->uncompressed.dwFrameInterval[0];
    param->dwFrameInterval = interval;
  }
  uint_fast32_t interval_ms = interval / 10000;
  TU_ASSERT(interval_ms);
  uint_fast32_t payload_size = (frame_size + interval_ms - 1) / interval_ms + 2;
  if (CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE < payload_size) {
    payload_size = CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE;
  }
  param->dwMaxPayloadTransferSize = payload_size;
  return true;
}

/** Set the minimum, maximum, default values or resolutions to variables which need to negotiate with the host
 *
 * @param[in]     request     GET_MAX, GET_MIN, GET_RES or GET_DEF
 * @param[in,out] param       Target
 */
static bool _negotiate_streaming_parameters(videod_streaming_interface_t const *stm, uint_fast8_t request,
                                            video_probe_and_commit_control_t *param)
{
  uint_fast8_t const fmtnum = param->bFormatIndex;
  if (!fmtnum) {
    switch (request) {
      case VIDEO_REQUEST_GET_MAX:
        if (_get_desc_vs(stm))
          param->bFormatIndex = _get_desc_vs(stm)->stm.bNumFormats;
        break;

      case VIDEO_REQUEST_GET_MIN:
      case VIDEO_REQUEST_GET_DEF:
        param->bFormatIndex = 1;
        break;

      default: return false;
    }
    /* Set the parameters determined by the format  */
    param->wKeyFrameRate    = 1;
    param->wPFrameRate      = 0;
    param->wCompQuality     = 1; /* 1 to 10000 */
    param->wCompWindowSize  = 1; /* GOP size? */
    param->wDelay           = 0; /* milliseconds */
    param->dwClockFrequency = 27000000; /* same as MPEG-2 system time clock  */
    param->bmFramingInfo    = 0x3; /* enables FrameID and EndOfFrame */
    param->bPreferedVersion = 1;
    param->bMinVersion      = 1;
    param->bMaxVersion      = 1;
    param->bUsage           = 0;
    param->bBitDepthLuma    = 8;
    return true;
  }

  uint_fast8_t frmnum = param->bFrameIndex;
  if (!frmnum) {
    tusb_desc_vs_itf_t const *vs = _get_desc_vs(stm);
    TU_ASSERT(vs);
    void const *end = _end_of_streaming_descriptor(vs);
    tusb_desc_cs_video_fmt_t const *fmt = _find_desc_format(tu_desc_next(vs), end, fmtnum);
    switch (request) {
      case VIDEO_REQUEST_GET_MAX:
        frmnum = fmt->bNumFrameDescriptors;
        break;

      case VIDEO_REQUEST_GET_MIN:
        frmnum = 1;
        break;

      case VIDEO_REQUEST_GET_DEF:
        switch (fmt->bDescriptorSubType) {
          case VIDEO_CS_ITF_VS_FORMAT_UNCOMPRESSED:
            frmnum = fmt->uncompressed.bDefaultFrameIndex;
            break;

          case VIDEO_CS_ITF_VS_FORMAT_MJPEG:
            frmnum = fmt->mjpeg.bDefaultFrameIndex;
            break;

          case VIDEO_CS_ITF_VS_FORMAT_FRAME_BASED:
            frmnum = fmt->frame_based.bDefaultFrameIndex;
            break;

          default: return false;
        }
        break;
      default: return false;
    }
    param->bFrameIndex = (uint8_t)frmnum;
    /* Set the parameters determined by the frame */
    tusb_desc_cs_video_frm_t const *frm = _find_desc_frame(tu_desc_next(fmt), end, frmnum);
    uint_fast32_t frame_size;
    switch (fmt->bDescriptorSubType) {
      case VIDEO_CS_ITF_VS_FORMAT_UNCOMPRESSED:
        frame_size = (uint_fast32_t)frm->wWidth * frm->wHeight * fmt->uncompressed.bBitsPerPixel / 8;
        break;

      case VIDEO_CS_ITF_VS_FORMAT_MJPEG:
        frame_size = (uint_fast32_t)frm->wWidth * frm->wHeight * 16 / 8; /* YUV422 */
        break;

      case VIDEO_CS_ITF_VS_FORMAT_FRAME_BASED:
        frame_size = (uint_fast32_t)frm->wWidth * frm->wHeight * 16 / 8; /* YUV422 */
        break;

      default: return false;
    }
    param->dwMaxVideoFrameSize = frame_size;
    return true;
  }

  if (!param->dwFrameInterval) {
    tusb_desc_vs_itf_t const *vs = _get_desc_vs(stm);
    TU_ASSERT(vs);
    void const *end = _end_of_streaming_descriptor(vs);
    tusb_desc_cs_video_fmt_t const *fmt = _find_desc_format(tu_desc_next(vs), end, fmtnum);
    tusb_desc_cs_video_frm_t const *frm = _find_desc_frame(tu_desc_next(fmt), end, frmnum);

    uint_fast32_t interval, interval_ms;
    switch (request) {
      case VIDEO_REQUEST_GET_MAX: {
        uint_fast32_t min_interval, max_interval;
        uint_fast8_t num_intervals = frm->uncompressed.bFrameIntervalType;
        max_interval = num_intervals ? frm->uncompressed.dwFrameInterval[num_intervals - 1]: frm->uncompressed.dwFrameInterval[1];
        min_interval = frm->uncompressed.dwFrameInterval[0];
        interval = max_interval;
        interval_ms = min_interval / 10000;
        break;
      }

      case VIDEO_REQUEST_GET_MIN: {
        uint_fast32_t min_interval, max_interval;
        uint_fast8_t num_intervals = frm->uncompressed.bFrameIntervalType;
        max_interval = num_intervals ? frm->uncompressed.dwFrameInterval[num_intervals - 1]: frm->uncompressed.dwFrameInterval[1];
        min_interval = frm->uncompressed.dwFrameInterval[0];
        interval = min_interval;
        interval_ms = max_interval / 10000;
        break;
      }

      case VIDEO_REQUEST_GET_DEF:
        interval = frm->uncompressed.dwDefaultFrameInterval;
        interval_ms = interval / 10000;
        break;

      case VIDEO_REQUEST_GET_RES: {
        uint_fast8_t num_intervals = frm->uncompressed.bFrameIntervalType;
        if (num_intervals) {
          interval = 0;
          interval_ms = 0;
        } else {
          interval = frm->uncompressed.dwFrameInterval[2];
          interval_ms = interval / 10000;
        }
        break;
      }

      default: return false;
    }
    param->dwFrameInterval = interval;
    if (!interval) {
      param->dwMaxPayloadTransferSize = 0;
    } else {
      uint_fast32_t frame_size = param->dwMaxVideoFrameSize;
      uint_fast32_t payload_size;
      if (!interval_ms) {
        payload_size = frame_size + 2;
      } else {
        payload_size = (frame_size + interval_ms - 1) / interval_ms + 2;
      }
      if (CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE < payload_size) {
        payload_size = CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE;
      }
      param->dwMaxPayloadTransferSize = payload_size;
    }
    return true;
  }
  return true;
}

/** Close current video control interface.
 *
 * @param[in,out] self     Video control interface context.
 * @param[in]     altnum   The target alternate setting number. */
static bool _close_vc_itf(uint8_t rhport, videod_interface_t *self)
{
  tusb_desc_vc_itf_t const *vc = _get_desc_vc(self);

  /* The next descriptor after the class-specific VC interface header descriptor. */
  void const *cur = (uint8_t const*)vc + vc->std.bLength + vc->ctl.bLength;

  /* The end of the video control interface descriptor. */
  void const *end = _end_of_control_descriptor(vc);
  if (vc->std.bNumEndpoints) {
    /* Find the notification endpoint descriptor. */
    cur = _find_desc(cur, end, TUSB_DESC_ENDPOINT);
    TU_ASSERT(cur < end);
    tusb_desc_endpoint_t const *notif = (tusb_desc_endpoint_t const *)cur;
    usbd_edpt_close(rhport, notif->bEndpointAddress);
  }
  self->cur = 0;
  return true;
}

/** Set the alternate setting to own video control interface.
 *
 * @param[in,out] self     Video control interface context.
 * @param[in]     altnum   The target alternate setting number. */
static bool _open_vc_itf(uint8_t rhport, videod_interface_t *self, uint_fast8_t altnum)
{
  TU_LOG_DRV("    open VC %d\r\n", altnum);
  uint8_t const *beg = self->beg;
  uint8_t const *end = beg + self->len;

  /* The first descriptor is a video control interface descriptor. */
  uint8_t const *cur = _find_desc_itf(beg, end, _desc_itfnum(beg), altnum);
  TU_LOG_DRV("    cur %" PRId32 "\r\n", (int32_t) (cur - beg));
  TU_VERIFY(cur < end);

  tusb_desc_vc_itf_t const *vc = (tusb_desc_vc_itf_t const *)cur;
  TU_LOG_DRV("    bInCollection %d\r\n", vc->ctl.bInCollection);
  /* Support for up to 2 streaming interfaces only. */
  TU_ASSERT(vc->ctl.bInCollection <= CFG_TUD_VIDEO_STREAMING);

  /* Update to point the end of the video control interface descriptor. */
  end = _end_of_control_descriptor(cur);

  /* Advance to the next descriptor after the class-specific VC interface header descriptor. */
  cur += vc->std.bLength + vc->ctl.bLength;
  TU_LOG_DRV("    bNumEndpoints %d\r\n", vc->std.bNumEndpoints);
  /* Open the notification endpoint if it exist. */
  if (vc->std.bNumEndpoints) {
    /* Support for 1 endpoint only. */
    TU_VERIFY(1 == vc->std.bNumEndpoints);
    /* Find the notification endpoint descriptor. */
    cur = _find_desc(cur, end, TUSB_DESC_ENDPOINT);
    TU_VERIFY(cur < end);
    tusb_desc_endpoint_t const *notif = (tusb_desc_endpoint_t const *)cur;
    /* Open the notification endpoint */
    TU_ASSERT(usbd_edpt_open(rhport, notif));
  }
  self->cur = (uint16_t) ((uint8_t const*)vc - beg);
  return true;
}

static bool _init_vs_configuration(videod_streaming_interface_t *stm) {
  /* initialize streaming settings */
  stm->state = VS_STATE_PROBING;
  stm->max_payload_transfer_size = 0;
  video_probe_and_commit_control_t *param = &stm->probe_commit_payload;
  tu_memclr(param, sizeof(*param));
  return _update_streaming_parameters(stm, param);
}

/** Set the alternate setting to own video streaming interface.
 *
 * @param[in,out] stm      Streaming interface context.
 * @param[in]     altnum   The target alternate setting number. */
static bool _open_vs_itf(uint8_t rhport, videod_streaming_interface_t *stm, uint_fast8_t altnum)
{
  uint_fast8_t i;
  TU_LOG_DRV("    reopen VS %d\r\n", altnum);
  uint8_t const *desc = _videod_itf[stm->index_vc].beg;

#ifndef TUP_DCD_EDPT_ISO_ALLOC
  /* Close endpoints of previous settings. */
  for (i = 0; i < TU_ARRAY_SIZE(stm->desc.ep); ++i) {
    uint_fast16_t ofs_ep = stm->desc.ep[i];
    if (!ofs_ep) break;
    tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const*)(desc + ofs_ep);
    /* Only ISO endpoints needs to be closed */
    if(ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
      stm->desc.ep[i] = 0;
      usbd_edpt_close(rhport, ep->bEndpointAddress);
      TU_LOG_DRV("    close EP%02x\r\n", ep->bEndpointAddress);
    }
  }
#endif

  /* clear transfer management information */
  stm->buffer  = NULL;
  stm->bufsize = 0;
  stm->offset  = 0;

  /* Find a alternate interface */
  uint8_t const *beg = desc + stm->desc.beg;
  uint8_t const *end = desc + stm->desc.end;
  uint8_t const *cur = _find_desc_itf(beg, end, _desc_itfnum(beg), altnum);
  TU_VERIFY(cur < end);

  uint_fast8_t numeps = ((tusb_desc_interface_t const *)cur)->bNumEndpoints;
  TU_ASSERT(numeps <= TU_ARRAY_SIZE(stm->desc.ep));
  stm->desc.cur = (uint16_t)(cur - desc); /* Save the offset of the new settings */
  if (!altnum && (VS_STATE_COMMITTED != stm->state)) {
    TU_VERIFY(_init_vs_configuration(stm));
  }
  /* Open bulk or isochronous endpoints of the new settings. */
  for (i = 0, cur = tu_desc_next(cur); i < numeps; ++i, cur = tu_desc_next(cur)) {
    cur = _find_desc_ep(cur, end);
    TU_ASSERT(cur < end);
    tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const*)cur;
    uint_fast32_t max_size = stm->max_payload_transfer_size;
    if (altnum && (TUSB_XFER_ISOCHRONOUS == ep->bmAttributes.xfer)) {
      /* FS must be less than or equal to max packet size */
      TU_VERIFY (tu_edpt_packet_size(ep) >= max_size);
#ifdef TUP_DCD_EDPT_ISO_ALLOC
      usbd_edpt_iso_activate(rhport, ep);
#else
      TU_ASSERT(usbd_edpt_open(rhport, ep));
#endif
    } else {
      TU_VERIFY(TUSB_XFER_BULK == ep->bmAttributes.xfer);
      TU_ASSERT(usbd_edpt_open(rhport, ep));
    }
    stm->desc.ep[i] = (uint16_t) (cur - desc);
    TU_LOG_DRV("    open EP%02x\r\n", _desc_ep_addr(cur));
  }
  if (altnum) {
    stm->state = VS_STATE_STREAMING;
  }
  TU_LOG_DRV("    done\r\n");
  return true;
}

/** Prepare the next packet payload. */
static uint_fast16_t _prepare_in_payload(videod_streaming_interface_t *stm, uint8_t* ep_buf) {
  uint_fast16_t remaining = stm->bufsize - stm->offset;
  uint_fast16_t hdr_len   = ep_buf[0];
  uint_fast16_t pkt_len   = stm->max_payload_transfer_size;
  if (hdr_len + remaining < pkt_len) {
    pkt_len = hdr_len + remaining;
  }
  TU_ASSERT(pkt_len >= hdr_len);
  uint_fast16_t data_len = pkt_len - hdr_len;
  memcpy(&ep_buf[hdr_len], stm->buffer + stm->offset, data_len);
  stm->offset += data_len;
  remaining -= data_len;
  if (!remaining) {
    tusb_video_payload_header_t *hdr = (tusb_video_payload_header_t*) ep_buf;
    hdr->EndOfFrame = 1;
  }
  return hdr_len + data_len;
}

/** Handle a standard request to the video control interface. */
static int handle_video_ctl_std_req(uint8_t rhport, uint8_t stage,
                                    tusb_control_request_t const *request,
                                    uint_fast8_t ctl_idx)
{
  TU_LOG_DRV("\r\n");
  switch (request->bRequest) {
    case TUSB_REQ_GET_INTERFACE:
      if (stage == CONTROL_STAGE_SETUP)
      {
        TU_VERIFY(1 == request->wLength, VIDEO_ERROR_UNKNOWN);
        tusb_desc_vc_itf_t const *vc = _get_desc_vc(&_videod_itf[ctl_idx]);
        TU_VERIFY(vc, VIDEO_ERROR_UNKNOWN);

        uint8_t alt_num = vc->std.bAlternateSetting;

        TU_VERIFY(tud_control_xfer(rhport, request, &alt_num, sizeof(alt_num)), VIDEO_ERROR_UNKNOWN);
      }
      return VIDEO_ERROR_NONE;

    case TUSB_REQ_SET_INTERFACE:
      if (stage == CONTROL_STAGE_SETUP)
      {
        TU_VERIFY(0 == request->wLength, VIDEO_ERROR_UNKNOWN);
        TU_VERIFY(_close_vc_itf(rhport, &_videod_itf[ctl_idx]), VIDEO_ERROR_UNKNOWN);
        TU_VERIFY(_open_vc_itf(rhport, &_videod_itf[ctl_idx], request->wValue), VIDEO_ERROR_UNKNOWN);
        tud_control_status(rhport, request);
      }
      return VIDEO_ERROR_NONE;

    default: /* Unknown/Unsupported request */
      TU_BREAKPOINT();
      return VIDEO_ERROR_INVALID_REQUEST;
  }
}

static int handle_video_ctl_cs_req(uint8_t rhport, uint8_t stage,
                                   tusb_control_request_t const *request,
                                   uint_fast8_t ctl_idx)
{
  videod_interface_t *self = &_videod_itf[ctl_idx];

  /* 4.2.1 Interface Control Request */
  uint8_t const ctrl_sel = TU_U16_HIGH(request->wValue);
  TU_LOG_DRV("%s_Control(%s)\r\n",  tu_str_video_vc_control_selector[ctrl_sel], tu_lookup_find(&tu_table_video_request, request->bRequest));

  switch (ctrl_sel) {
    case VIDEO_VC_CTL_VIDEO_POWER_MODE:
      switch (request->bRequest) {
        case VIDEO_REQUEST_SET_CUR:
          if (stage == CONTROL_STAGE_SETUP) {
            TU_VERIFY(1 == request->wLength, VIDEO_ERROR_UNKNOWN);
            TU_VERIFY(tud_control_xfer(rhport, request, &self->power_mode, sizeof(self->power_mode)), VIDEO_ERROR_UNKNOWN);
          } else if (stage == CONTROL_STAGE_DATA) {
            if (tud_video_power_mode_cb) return tud_video_power_mode_cb(ctl_idx, self->power_mode);
          }
          return VIDEO_ERROR_NONE;

        case VIDEO_REQUEST_GET_CUR:
          if (stage == CONTROL_STAGE_SETUP)
          {
            TU_VERIFY(1 == request->wLength, VIDEO_ERROR_UNKNOWN);
            TU_VERIFY(tud_control_xfer(rhport, request, &self->power_mode, sizeof(self->power_mode)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        case VIDEO_REQUEST_GET_INFO:
          if (stage == CONTROL_STAGE_SETUP)
          {
            TU_VERIFY(1 == request->wLength, VIDEO_ERROR_UNKNOWN);
            TU_VERIFY(tud_control_xfer(rhport, request, (uint8_t*)(uintptr_t) &_cap_get_set, sizeof(_cap_get_set)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        default: break;
      }
      break;

    case VIDEO_VC_CTL_REQUEST_ERROR_CODE:
      switch (request->bRequest) {
        case VIDEO_REQUEST_GET_CUR:
          if (stage == CONTROL_STAGE_SETUP)
          {
            TU_VERIFY(tud_control_xfer(rhport, request, &self->error_code, sizeof(uint8_t)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        case VIDEO_REQUEST_GET_INFO:
          if (stage == CONTROL_STAGE_SETUP)
          {
            TU_VERIFY(tud_control_xfer(rhport, request, (uint8_t*)(uintptr_t) &_cap_get, sizeof(_cap_get)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        default: break;
      }
      break;

    default: break;
  }

  /* Unknown/Unsupported request */
  TU_BREAKPOINT();
  return VIDEO_ERROR_INVALID_REQUEST;
}

static int handle_video_ctl_req(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request,
                                uint_fast8_t ctl_idx)
{
  switch (request->bmRequestType_bit.type) {
    case TUSB_REQ_TYPE_STANDARD:
      return handle_video_ctl_std_req(rhport, stage, request, ctl_idx);

    case TUSB_REQ_TYPE_CLASS: {
      uint_fast8_t entity_id = TU_U16_HIGH(request->wIndex);
      if (!entity_id) {
        return handle_video_ctl_cs_req(rhport, stage, request, ctl_idx);
      } else {
        TU_VERIFY(_find_desc_entity(_get_desc_vc(&_videod_itf[ctl_idx]), entity_id), VIDEO_ERROR_INVALID_REQUEST);
        return VIDEO_ERROR_NONE;
      }
    }

    default:
      return VIDEO_ERROR_INVALID_REQUEST;
  }
}

static int handle_video_stm_std_req(uint8_t rhport, uint8_t stage,
                                    tusb_control_request_t const *request,
                                    uint_fast8_t stm_idx)
{
  TU_LOG_DRV("\r\n");
  videod_streaming_interface_t *self = &_videod_streaming_itf[stm_idx];
  switch (request->bRequest) {
    case TUSB_REQ_GET_INTERFACE:
      if (stage == CONTROL_STAGE_SETUP)
      {
        TU_VERIFY(1 == request->wLength, VIDEO_ERROR_UNKNOWN);
        tusb_desc_vs_itf_t const *vs = _get_desc_vs(self);
        TU_VERIFY(vs, VIDEO_ERROR_UNKNOWN);
        uint8_t alt_num = vs->std.bAlternateSetting;

        TU_VERIFY(tud_control_xfer(rhport, request, &alt_num, sizeof(alt_num)), VIDEO_ERROR_UNKNOWN);
      }
      return VIDEO_ERROR_NONE;

    case TUSB_REQ_SET_INTERFACE:
      if (stage == CONTROL_STAGE_SETUP) {
        TU_VERIFY(_open_vs_itf(rhport, self, request->wValue), VIDEO_ERROR_UNKNOWN);
        tud_control_status(rhport, request);
      }
      return VIDEO_ERROR_NONE;

    default: /* Unknown/Unsupported request */
      TU_BREAKPOINT();
      return VIDEO_ERROR_INVALID_REQUEST;
  }
}

static int handle_video_stm_cs_req(uint8_t rhport, uint8_t stage,
                                   tusb_control_request_t const *request,
                                   uint_fast8_t stm_idx) {
  (void)rhport;
  videod_streaming_interface_t *stm = &_videod_streaming_itf[stm_idx];
  videod_streaming_epbuf_t *stm_epbuf = &_videod_streaming_epbuf[stm_idx];

  uint8_t const ctrl_sel = TU_U16_HIGH(request->wValue);
  TU_LOG_DRV("%s_Control(%s)\r\n", tu_str_video_vs_control_selector[ctrl_sel], tu_lookup_find(&tu_table_video_request, request->bRequest));

  /* 4.2.1 Interface Control Request */
  switch (ctrl_sel) {
    case VIDEO_VS_CTL_STREAM_ERROR_CODE:
      switch (request->bRequest) {
        case VIDEO_REQUEST_GET_CUR:
          if (stage == CONTROL_STAGE_SETUP) {
            /* TODO */
            TU_VERIFY(tud_control_xfer(rhport, request, &stm->error_code, sizeof(uint8_t)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        case VIDEO_REQUEST_GET_INFO:
          if (stage == CONTROL_STAGE_SETUP) {
            TU_VERIFY(tud_control_xfer(rhport, request, (uint8_t*)(uintptr_t) &_cap_get, sizeof(_cap_get)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        default: break;
      }
      break;

    case VIDEO_VS_CTL_PROBE:
      if (stm->state != VS_STATE_PROBING) {
        stm->state = VS_STATE_PROBING;
      }

      switch (request->bRequest) {
        case VIDEO_REQUEST_SET_CUR:
          if (stage == CONTROL_STAGE_SETUP) {
            TU_VERIFY(tud_control_xfer(rhport, request, &stm->probe_commit_payload, sizeof(video_probe_and_commit_control_t)),
                      VIDEO_ERROR_UNKNOWN);
          } else if (stage == CONTROL_STAGE_DATA) {
            TU_VERIFY(_update_streaming_parameters(stm, &stm->probe_commit_payload),
                      VIDEO_ERROR_INVALID_VALUE_WITHIN_RANGE);
          }
          return VIDEO_ERROR_NONE;

        case VIDEO_REQUEST_GET_CUR:
          if (stage == CONTROL_STAGE_SETUP) {
            TU_VERIFY(request->wLength, VIDEO_ERROR_UNKNOWN);
            TU_VERIFY(tud_control_xfer(rhport, request, &stm->probe_commit_payload, sizeof(video_probe_and_commit_control_t)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        case VIDEO_REQUEST_GET_MIN:
        case VIDEO_REQUEST_GET_MAX:
        case VIDEO_REQUEST_GET_RES:
        case VIDEO_REQUEST_GET_DEF:
          if (stage == CONTROL_STAGE_SETUP) {
            TU_VERIFY(request->wLength, VIDEO_ERROR_UNKNOWN);
            video_probe_and_commit_control_t tmp = stm->probe_commit_payload;
            TU_VERIFY(_negotiate_streaming_parameters(stm, request->bRequest, &tmp), VIDEO_ERROR_INVALID_VALUE_WITHIN_RANGE);
            TU_VERIFY(tud_control_xfer(rhport, request, &tmp, sizeof(tmp)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        case VIDEO_REQUEST_GET_LEN:
          if (stage == CONTROL_STAGE_SETUP) {
            TU_VERIFY(2 == request->wLength, VIDEO_ERROR_UNKNOWN);
            uint16_t len = sizeof(video_probe_and_commit_control_t);
            TU_VERIFY(tud_control_xfer(rhport, request, (uint8_t*)&len, sizeof(len)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        case VIDEO_REQUEST_GET_INFO:
          if (stage == CONTROL_STAGE_SETUP) {
            TU_VERIFY(1 == request->wLength, VIDEO_ERROR_UNKNOWN);
            TU_VERIFY(tud_control_xfer(rhport, request, (uint8_t*)(uintptr_t)&_cap_get_set, sizeof(_cap_get_set)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        default: break;
      }
      break;

    case VIDEO_VS_CTL_COMMIT:
      switch (request->bRequest) {
        case VIDEO_REQUEST_SET_CUR:
          if (stage == CONTROL_STAGE_SETUP) {
            TU_VERIFY(tud_control_xfer(rhport, request, &stm->probe_commit_payload, sizeof(video_probe_and_commit_control_t)), VIDEO_ERROR_UNKNOWN);
          } else if (stage == CONTROL_STAGE_DATA) {
            video_probe_and_commit_control_t *param = &stm->probe_commit_payload;
            TU_VERIFY(_update_streaming_parameters(stm, param), VIDEO_ERROR_INVALID_VALUE_WITHIN_RANGE);
            /* Set the negotiated value */
            stm->max_payload_transfer_size = param->dwMaxPayloadTransferSize;
            int ret = VIDEO_ERROR_NONE;
            if (tud_video_commit_cb) {
              ret = tud_video_commit_cb(stm->index_vc, stm->index_vs, param);
            }
            if (VIDEO_ERROR_NONE == ret) {
              stm->state   = VS_STATE_COMMITTED;
              stm->buffer  = NULL;
              stm->bufsize = 0;
              stm->offset  = 0;
              /* initialize payload header */
              tusb_video_payload_header_t *hdr = (tusb_video_payload_header_t*)stm_epbuf->buf;
              hdr->bHeaderLength = sizeof(*hdr);
              hdr->bmHeaderInfo  = 0;
            }
          }
          return VIDEO_ERROR_NONE;

        case VIDEO_REQUEST_GET_CUR:
          if (stage == CONTROL_STAGE_SETUP) {
            TU_VERIFY(request->wLength, VIDEO_ERROR_UNKNOWN);
            TU_VERIFY(tud_control_xfer(rhport, request, &stm->probe_commit_payload, sizeof(video_probe_and_commit_control_t)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        case VIDEO_REQUEST_GET_LEN:
          if (stage == CONTROL_STAGE_SETUP) {
            TU_VERIFY(2 == request->wLength, VIDEO_ERROR_UNKNOWN);
            uint16_t len = sizeof(video_probe_and_commit_control_t);
            TU_VERIFY(tud_control_xfer(rhport, request, (uint8_t*)&len, sizeof(len)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        case VIDEO_REQUEST_GET_INFO:
          if (stage == CONTROL_STAGE_SETUP) {
            TU_VERIFY(1 == request->wLength, VIDEO_ERROR_UNKNOWN);
            TU_VERIFY(tud_control_xfer(rhport, request, (uint8_t*)(uintptr_t) &_cap_get_set, sizeof(_cap_get_set)), VIDEO_ERROR_UNKNOWN);
          }
          return VIDEO_ERROR_NONE;

        default: break;
      }
      break;

    case VIDEO_VS_CTL_STILL_PROBE:
    case VIDEO_VS_CTL_STILL_COMMIT:
    case VIDEO_VS_CTL_STILL_IMAGE_TRIGGER:
    case VIDEO_VS_CTL_GENERATE_KEY_FRAME:
    case VIDEO_VS_CTL_UPDATE_FRAME_SEGMENT:
    case VIDEO_VS_CTL_SYNCH_DELAY_CONTROL:
      /* TODO */
      break;

    default: break;
  }

  /* Unknown/Unsupported request */
  TU_BREAKPOINT();
  return VIDEO_ERROR_INVALID_REQUEST;
}

static int handle_video_stm_req(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request,
                                uint_fast8_t stm_idx)
{
  switch (request->bmRequestType_bit.type) {
    case TUSB_REQ_TYPE_STANDARD:
      return handle_video_stm_std_req(rhport, stage, request, stm_idx);

    case TUSB_REQ_TYPE_CLASS:
      if (TU_U16_HIGH(request->wIndex)) return VIDEO_ERROR_INVALID_REQUEST;
      return handle_video_stm_cs_req(rhport, stage, request, stm_idx);

    default: return VIDEO_ERROR_INVALID_REQUEST;
  }
}

//--------------------------------------------------------------------+
// APPLICATION API
//--------------------------------------------------------------------+

bool tud_video_n_connected(uint_fast8_t ctl_idx)
{
  TU_ASSERT(ctl_idx < CFG_TUD_VIDEO);
  videod_streaming_interface_t *stm = _get_instance_streaming(ctl_idx, 0);
  if (stm) return true;
  return false;
}

bool tud_video_n_streaming(uint_fast8_t ctl_idx, uint_fast8_t stm_idx)
{
  TU_ASSERT(ctl_idx < CFG_TUD_VIDEO);
  TU_ASSERT(stm_idx < CFG_TUD_VIDEO_STREAMING);
  videod_streaming_interface_t *stm = _get_instance_streaming(ctl_idx, stm_idx);
  if (!stm || !stm->desc.ep[0]) return false;
  if (stm->state == VS_STATE_PROBING) return false;

#ifdef TUP_DCD_EDPT_ISO_ALLOC
  uint8_t const *desc = _videod_itf[stm->index_vc].beg;
  uint_fast16_t ofs_ep = stm->desc.ep[0];
  tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const*)(desc + ofs_ep);
  if (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
    if (stm->state == VS_STATE_COMMITTED) return false;
  }
#endif

  return true;
}

bool tud_video_n_frame_xfer(uint_fast8_t ctl_idx, uint_fast8_t stm_idx, void *buffer, size_t bufsize) {
  TU_ASSERT(ctl_idx < CFG_TUD_VIDEO);
  TU_ASSERT(stm_idx < CFG_TUD_VIDEO_STREAMING);

  if (!buffer || !bufsize) return false;
  videod_streaming_interface_t *stm = _get_instance_streaming(ctl_idx, stm_idx);
  videod_streaming_epbuf_t *stm_epbuf = &_videod_streaming_epbuf[ctl_idx];

  if (!stm || !stm->desc.ep[0] || stm->buffer) return false;
  if (stm->state == VS_STATE_PROBING) return false;

  /* Find EP address */
  uint8_t const *desc = _videod_itf[stm->index_vc].beg;
  uint8_t ep_addr = 0;
  for (uint_fast8_t i = 0; i < CFG_TUD_VIDEO_STREAMING; ++i) {
    uint_fast16_t ofs_ep = stm->desc.ep[i];
    if (!ofs_ep) continue;
    ep_addr = _desc_ep_addr(desc + ofs_ep);
    break;
  }
  if (!ep_addr) return false;

  TU_VERIFY( usbd_edpt_claim(0, ep_addr) );
  /* update the packet header */
  tusb_video_payload_header_t *hdr = (tusb_video_payload_header_t*)stm_epbuf->buf;
  hdr->FrameID   ^= 1;
  hdr->EndOfFrame = 0;
  /* update the packet data */
  stm->buffer     = (uint8_t*)buffer;
  stm->bufsize    = bufsize;
  uint_fast16_t pkt_len = _prepare_in_payload(stm, stm_epbuf->buf);
  TU_ASSERT( usbd_edpt_xfer(0, ep_addr, stm_epbuf->buf, (uint16_t) pkt_len), 0);
  return true;
}

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void videod_init(void) {
  for (uint_fast8_t i = 0; i < CFG_TUD_VIDEO; ++i) {
    videod_interface_t* ctl = &_videod_itf[i];
    tu_memclr(ctl, sizeof(*ctl));
  }
  for (uint_fast8_t i = 0; i < CFG_TUD_VIDEO_STREAMING; ++i) {
    videod_streaming_interface_t *stm = &_videod_streaming_itf[i];
    tu_memclr(stm, sizeof(videod_streaming_interface_t));
  }
}

bool videod_deinit(void) {
  return true;
}

void videod_reset(uint8_t rhport) {
  (void) rhport;
  for (uint_fast8_t i = 0; i < CFG_TUD_VIDEO; ++i) {
    videod_interface_t* ctl = &_videod_itf[i];
    tu_memclr(ctl, sizeof(*ctl));
  }
  for (uint_fast8_t i = 0; i < CFG_TUD_VIDEO_STREAMING; ++i) {
    videod_streaming_interface_t *stm = &_videod_streaming_itf[i];
    tu_memclr(stm, sizeof(videod_streaming_interface_t));
  }
}

uint16_t videod_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len) {
  TU_VERIFY((TUSB_CLASS_VIDEO       == itf_desc->bInterfaceClass) &&
            (VIDEO_SUBCLASS_CONTROL == itf_desc->bInterfaceSubClass) &&
            (VIDEO_ITF_PROTOCOL_15  == itf_desc->bInterfaceProtocol), 0);

  /* Find available interface */
  videod_interface_t *self = NULL;
  uint8_t ctl_idx;
  for (ctl_idx = 0; ctl_idx < CFG_TUD_VIDEO; ++ctl_idx) {
    if (_videod_itf[ctl_idx].beg) continue;
    self = &_videod_itf[ctl_idx];
    break;
  }
  TU_ASSERT(ctl_idx < CFG_TUD_VIDEO, 0);

  uint8_t const *end = (uint8_t const*)itf_desc + max_len;
  self->beg = (uint8_t const*) itf_desc;
  self->len = max_len;

  /*------------- Video Control Interface -------------*/
  TU_VERIFY(_open_vc_itf(rhport, self, 0), 0);
  tusb_desc_vc_itf_t const *vc = _get_desc_vc(self);
  uint_fast8_t bInCollection   = vc->ctl.bInCollection;

  /* Find the end of the video interface descriptor */
  void const *cur = _next_desc_itf(itf_desc, end);
  for (uint8_t stm_idx = 0; stm_idx < bInCollection; ++stm_idx) {
    videod_streaming_interface_t *stm = NULL;
    /* find free streaming interface handle */
    for (uint8_t i = 0; i < CFG_TUD_VIDEO_STREAMING; ++i) {
      if (_videod_streaming_itf[i].desc.beg) continue;
      stm = &_videod_streaming_itf[i];
      self->stm[stm_idx] = i;
      break;
    }
    TU_ASSERT(stm, 0);
    stm->index_vc = ctl_idx;
    stm->index_vs = stm_idx;
    stm->desc.beg = (uint16_t) ((uintptr_t)cur - (uintptr_t)itf_desc);
    cur = _next_desc_itf(cur, end);
    stm->desc.end = (uint16_t) ((uintptr_t)cur - (uintptr_t)itf_desc);
    stm->state = VS_STATE_PROBING;
#ifdef TUP_DCD_EDPT_ISO_ALLOC
    /* Allocate ISO endpoints */
    uint16_t ep_size = 0;
    uint8_t ep_addr = 0;
    uint8_t const *p_desc = (uint8_t const*)itf_desc + stm->desc.beg;
    uint8_t const *p_desc_end = (uint8_t const*)itf_desc + stm->desc.end;
    while (p_desc < p_desc_end) {
      if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
        tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *) p_desc;
        if (desc_ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
              ep_addr = desc_ep->bEndpointAddress;
              ep_size = TU_MAX(tu_edpt_packet_size(desc_ep), ep_size);
        }
      }
      p_desc = tu_desc_next(p_desc);
    }
    if(ep_addr > 0 && ep_size > 0) usbd_edpt_iso_alloc(rhport, ep_addr, ep_size);
#endif
    if (0 == stm_idx && 1 == bInCollection) {
      /* If there is only one streaming interface and no alternate settings,
       * host may not issue set_interface so open the streaming interface here. */
      uint8_t const *sbeg = (uint8_t const*)itf_desc + stm->desc.beg;
      uint8_t const *send = (uint8_t const*)itf_desc + stm->desc.end;
      if (send == _find_desc_itf(sbeg, send, _desc_itfnum(sbeg), 1)) {
        TU_VERIFY(_open_vs_itf(rhport, stm, 0), 0);
      }
    }
  }
  self->len = (uint16_t) ((uintptr_t)cur - (uintptr_t)itf_desc);
  return (uint16_t) ((uintptr_t)cur - (uintptr_t)itf_desc);
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool videod_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) {
  int err;
  TU_VERIFY(request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE);
  uint_fast8_t itfnum = tu_u16_low(request->wIndex);
  /* Identify which control interface to use */
  uint_fast8_t itf;
  for (itf = 0; itf < CFG_TUD_VIDEO; ++itf) {
    void const *desc = _videod_itf[itf].beg;
    if (!desc) continue;
    if (itfnum == _desc_itfnum(desc)) break;
  }

  if (itf < CFG_TUD_VIDEO) {
    TU_LOG_DRV("  VC[%d]: ", itf);
    err = handle_video_ctl_req(rhport, stage, request, itf);
    _videod_itf[itf].error_code = (uint8_t)err;
    if (err) return false;
    return true;
  }

  /* Identify which streaming interface to use */
  for (itf = 0; itf < CFG_TUD_VIDEO_STREAMING; ++itf) {
    videod_streaming_interface_t *stm = &_videod_streaming_itf[itf];
    if (!stm->desc.beg) continue;
    uint8_t const *desc = _videod_itf[stm->index_vc].beg;
    if (itfnum == _desc_itfnum(desc + stm->desc.beg)) break;
  }

  if (itf < CFG_TUD_VIDEO_STREAMING) {
    TU_LOG_DRV("  VS[%d]: ", itf);
    err = handle_video_stm_req(rhport, stage, request, itf);
    _videod_streaming_itf[itf].error_code = (uint8_t)err;
    if (err) return false;
    return true;
  }
  return false;
}

bool videod_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
  (void)result; (void)xferred_bytes;

  /* find streaming handle */
  uint_fast8_t itf;
  videod_interface_t *ctl;
  videod_streaming_interface_t *stm;
  for (itf = 0; itf < CFG_TUD_VIDEO_STREAMING; ++itf) {
    stm = &_videod_streaming_itf[itf];
    uint_fast16_t const ep_ofs = stm->desc.ep[0];
    if (!ep_ofs) continue;
    ctl = &_videod_itf[stm->index_vc];
    uint8_t const *desc = ctl->beg;
    if (ep_addr == _desc_ep_addr(desc + ep_ofs)) break;
  }
  TU_ASSERT(itf < CFG_TUD_VIDEO_STREAMING);
  videod_streaming_epbuf_t *stm_epbuf = &_videod_streaming_epbuf[itf];

  if (stm->offset < stm->bufsize) {
    /* Claim the endpoint */
    TU_VERIFY( usbd_edpt_claim(rhport, ep_addr), 0);
    uint_fast16_t pkt_len = _prepare_in_payload(stm, stm_epbuf->buf);
    TU_ASSERT( usbd_edpt_xfer(rhport, ep_addr, stm_epbuf->buf, (uint16_t) pkt_len), 0);
  } else {
    stm->buffer  = NULL;
    stm->bufsize = 0;
    stm->offset  = 0;
    if (tud_video_frame_xfer_complete_cb) {
      tud_video_frame_xfer_complete_cb(stm->index_vc, stm->index_vs);
    }
  }
  return true;
}

#endif
