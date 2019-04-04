#include "platform.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "rx/srxlv2.h"
#include "rx/srxlv2_types.h"
#include "io/spektrum_vtx_control.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include <string.h>

#ifndef SRXLv2_DEBUG
#define SRXLv2_DEBUG 0
#endif

#if SRXLv2_DEBUG
void cliPrintf(const char *format, ...);
#define DEBUG(format, ...) cliPrintf(format, __VA_ARGS__)
#else
#define DEBUG(...)
#endif

#define USE_SERIALRX_SRXLv2
#ifdef USE_SERIALRX_SRXLv2

#define SRXLv2_MAX_CHANNELS             32
#define SRXLv2_TIME_BETWEEN_FRAMES_US   11000 // 5500 for DSMR
#define SRXLv2_CHANNEL_SHIFT            5
#define SRXLv2_CHANNEL_CENTER           0x8000

#define SRXLv2_PORT_BAUDRATE_DEFAULT    115200
#define SRXLv2_PORT_BAUDRATE_HIGH       400000
#define SRXLv2_PORT_OPTIONS             (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_BIDIR)
#define SRXLv2_PORT_MODE                MODE_RXTX

#define SRXLv2_REPLY_QUIESCENCE         (2 * 10 * 1000000 / SRXLv2_PORT_BAUDRATE_DEFAULT) /*2 * (last_idle_timestamp - last_receive_timestamp)*/

#define SRXLv2_ID                       0xA6
#define SRXLv2_MAX_PACKET_LENGTH        80
#define SRXLv2_DEVICE_ID_BROADCAST      0xFF

#define SRXLv2_FRAME_TIMEOUT_US         50000

#define SRXLv2_LISTEN_FOR_ACTIVITY_TIMEOUT 50000
#define SRXLv2_SEND_HANDSHAKE_TIMEOUT 50000
#define SRXLv2_LISTEN_FOR_HANDSHAKE_TIMEOUT 200000


static uint8_t unit_id = 0;
static uint8_t baud_rate = 0;

static srxlv2State state = Disabled;
static volatile uint32_t timeout_timestamp = 0;
static volatile uint32_t full_timeout_timestamp = 0;
static volatile uint32_t last_receive_timestamp = 0;
static volatile uint32_t last_valid_packet_timestamp = 0;

static volatile uint8_t read_buffer[2 * SRXLv2_MAX_PACKET_LENGTH];
static volatile uint8_t read_buffer_idx = 0;
static volatile uint8_t read_buffer_read_idx = 0;
static uint8_t write_buffer[SRXLv2_MAX_PACKET_LENGTH];
static uint8_t write_buffer_idx = 0;

static serialPort_t *serialPort;

static uint8_t bus_master_device_id = 0xFF;
static bool telemetry_requested = false;

static uint8_t telemetryFrame[22];

uint8_t global_result = 0;
bool srxlv2ProcessHandshake(const srxlv2Header* header, const srxlv2HandshakeSubHeader* handshake)
{
    if (handshake->destination_device_id == Broadcast) {
        DEBUG("broadcast handshake from %x\r\n", handshake->source_device_id);
        bus_master_device_id = handshake->source_device_id;

        if (handshake->baud_supported == 1) {
            serialPort->vTable->serialSetBaudRate(serialPort, SRXLv2_PORT_BAUDRATE_HIGH);
            DEBUG("switching to %d baud\r\n", SRXLv2_PORT_BAUDRATE_HIGH);
        }

        state = Running;

        return true;
    }

    if (((handshake->destination_device_id >> 4) & 0xF) != FlightController ||
        (handshake->destination_device_id & 0xF) != unit_id) {
        return true;
    }

    DEBUG("FC handshake from %x\r\n", handshake->source_device_id);

    srxlv2HandshakeFrame response = {
        .header = *header,
        .payload = {
            handshake->destination_device_id,
            handshake->source_device_id,
            /* priority */ 10,
            /* baud_supported*/ baud_rate,
            /* info */ 0,
            U_ID_2
        }
    };

    const uint16_t crc = crc16_ccitt_update(0, &response, sizeof(response) - 2);
    response.crc_high = ((uint8_t *) &crc)[1];
    response.crc_low = ((uint8_t *) &crc)[0];

    srxlv2RxWriteData(&response, sizeof(response));

    return true;
}

void srxlv2ProcessChannelData(const srxlv2ChannelDataHeader* channel_data, rxRuntimeConfig_t *rxRuntimeConfig) {
    if (channel_data->rssi >= 0) {
        const int rssi_percent = channel_data->rssi;
        setRssi(scaleRange(rssi_percent, 0, 100, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_PROTOCOL);
    } else {
        const int rssi_dbm = channel_data->rssi;
        (void) rssi_dbm;
        // @todo come up with a scheme to scale power values to 0-1023 range
        setRssi(RSSI_MAX_VALUE / 2, RSSI_SOURCE_RX_PROTOCOL);
    }

    if (channel_data->rssi == 0) {
        global_result = RX_FRAME_FAILSAFE;
    } else {
        global_result = RX_FRAME_COMPLETE;
    }

    const uint16_t *frame_channels = (const uint16_t *) (channel_data + 1);

    if (channel_data->channel_mask.u8.channels_0_7) {
        for (size_t i = 0; i < 8; ++i) {
            if (channel_data->channel_mask.u8.channels_0_7 >> i & 1) {
                rxRuntimeConfig->channelData[i] = *frame_channels++;
            }
        }
    }

    if (channel_data->channel_mask.u8.channels_8_15) {
        for (size_t i = 0; i < 8; ++i) {
            if (channel_data->channel_mask.u8.channels_8_15 >> i & 1) {
                rxRuntimeConfig->channelData[8 + i] = *frame_channels++;
            }
        }
    }

    if (channel_data->channel_mask.u8.channels_16_23) {
        for (size_t i = 0; i < 8; ++i) {
            if (channel_data->channel_mask.u8.channels_16_23 >> i & 1) {
                rxRuntimeConfig->channelData[16 + i] = *frame_channels++;
            }
        }
    }

    if (channel_data->channel_mask.u8.channels_24_31) {
        for (size_t i = 0; i < 8; ++i) {
            if (channel_data->channel_mask.u8.channels_24_31 >> i & 1) {
                rxRuntimeConfig->channelData[24 + i] = *frame_channels++;
            }
        }
    }

    // DEBUG("channel data: %d %d %x\r\n",
    //     channel_data_header->rssi, channel_data_header->frame_losses, channel_data_header->channel_mask.u32);
}

bool srxlv2ProcessControlData(const srxlv2ControlDataSubHeader* control_data, rxRuntimeConfig_t *rxRuntimeConfig)
{
    const uint8_t own_id = FlightController << 4 | unit_id;
    if (control_data->reply_id == own_id) {
        telemetry_requested = true;
//        DEBUG("command: %x reply_id: %x own_id: %x\r\n", control_data->command, control_data->reply_id, own_id);
    }

    switch (control_data->command) {
        case ChannelData: {
            srxlv2ProcessChannelData((const srxlv2ChannelDataHeader *) (control_data + 1), rxRuntimeConfig);
        } break;

        case FailsafeChannelData: {
            srxlv2ProcessChannelData((const srxlv2ChannelDataHeader *) (control_data + 1), rxRuntimeConfig);
            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
            // DEBUG("fs channel data\r\n");
        } break;

        case VTXData: {
            #if defined(USE_SPEKTRUM_VTX_CONTROL) && defined(USE_VTX_COMMON)
            //DEBUG("vtx data\r\n");
            srxlv2VtxData *vtxData = (srxlv2VtxData*)(control_data + 1);
            //DEBUG("vtx band: %x\r\n", vtxData->band);
            //DEBUG("vtx channel: %x\r\n", vtxData->channel);
            //DEBUG("vtx pit: %x\r\n", vtxData->pit);
            //DEBUG("vtx power: %x\r\n", vtxData->power);
            //DEBUG("vtx powerDec: %x\r\n", vtxData->powerDec);
            //DEBUG("vtx region: %x\r\n", vtxData->region);
            // Pack data as it was used before srxlv2 to use existing functions.
            // Get the VTX control bytes in a frame
            uint32_t vtxControl =   (0xE0 << 24) | (0xE0 << 8) |
                                    ((vtxData->band & 0x07) << 21) |
                                    ((vtxData->channel & 0x0F) << 16) |
                                    ((vtxData->pit & 0x01) << 4) |
                                    ((vtxData->region & 0x01) << 3) |
                                    ((vtxData->power & 0x07));
            spektrumHandleVtxControl(vtxControl);
            #endif
        } break;
    }

    return true;
}

bool srxlv2ProcessPacket(const srxlv2Header* header, rxRuntimeConfig_t *rxRuntimeConfig)
{
    switch (header->packet_type) {
    case Handshake: return srxlv2ProcessHandshake(header, (const srxlv2HandshakeSubHeader *) (header + 1));
    case ControlData: return srxlv2ProcessControlData((const srxlv2ControlDataSubHeader *) (header + 1), rxRuntimeConfig);
    default: break;
    }

    return false;
}

// @note assumes packet is fully there
void srxlv2Process(rxRuntimeConfig_t *rxRuntimeConfig)
{
    union {
        uint8_t local_buffer[SRXLv2_MAX_PACKET_LENGTH];
        srxlv2Header header;
    } anonymous;

    uint32_t bytes_copied = 0;
    while (bytes_copied < sizeof(srxlv2Header)) {
        anonymous.local_buffer[bytes_copied++] = read_buffer[read_buffer_read_idx++];

        if (read_buffer_read_idx == sizeof(read_buffer)) {
            read_buffer_read_idx = 0;
        }
    }

    // @todo find out why it happens
    if (anonymous.header.id != SRXLv2_ID) {
      read_buffer_read_idx = read_buffer_idx;
      global_result = RX_FRAME_DROPPED;
      return;
    }

    //    cliPrintf("> %d %d %x\r\n", read_buffer_idx, read_buffer_read_idx, anonymous.header.id);

    const uint8_t bytes_to_read = anonymous.header.length;
    if (bytes_to_read > sizeof(anonymous)) {
      read_buffer_read_idx = read_buffer_idx;
      global_result = RX_FRAME_DROPPED;
      return;
    }

    while (bytes_copied < bytes_to_read) {
        anonymous.local_buffer[bytes_copied++] = read_buffer[read_buffer_read_idx++];

        if (read_buffer_read_idx == sizeof(read_buffer)) {
            read_buffer_read_idx = 0;
        }
    }

    //    cliPrintf("- %d %d %x\r\n", read_buffer_idx, read_buffer_read_idx, anonymous.header.id);

    if (anonymous.header.id != SRXLv2_ID ||
        bytes_copied != anonymous.header.length) {
        read_buffer_read_idx = read_buffer_idx;
        global_result = RX_FRAME_DROPPED;
        return;
    }

    const uint16_t calculated_crc = crc16_ccitt_update(0, anonymous.local_buffer, anonymous.header.length - 2);
    //    cliPrintf("< %d %d %x %x\r\n", read_buffer_idx, read_buffer_read_idx, anonymous.header.id, calculated_crc);

    const uint16_t received_crc =
        (anonymous.local_buffer[anonymous.header.length - 2] << 8) |
        (anonymous.local_buffer[anonymous.header.length - 1]);

    if (calculated_crc != received_crc) {
        read_buffer_read_idx = read_buffer_idx;
        global_result = RX_FRAME_DROPPED;
        //        DEBUG("crc mismatch %x vs %x\r\n", calculated_crc, received_crc);
        return;
    }

    //Packet is valid only after ID and CRC check out
    last_valid_packet_timestamp = micros();

    if (srxlv2ProcessPacket(&anonymous.header, rxRuntimeConfig)) {
        return;
    }

    // @todo reset
    //    DEBUG("could not parse packet: %x\r\n", anonymous.header.packet_type);
    global_result = RX_FRAME_DROPPED;
}


static void srxlv2DataReceive(uint16_t character, void *data)
{
    UNUSED(data);

    last_receive_timestamp = microsISR();

    // circular buffer depleted and waiting for sync character
    if (read_buffer_read_idx == read_buffer_idx && character != SRXLv2_ID) {
        return;
    }

    read_buffer[read_buffer_idx] = character;
    read_buffer_idx = read_buffer_idx + 1;
    if (read_buffer_idx == sizeof(read_buffer)) {
      read_buffer_idx = 0;
    }
}

static volatile uint32_t last_idle_timestamp = 0;

static void srxlv2Idle()
{
    last_idle_timestamp = microsISR();
}

uint32_t autobaud_timeout = 0;
static uint8_t srxlv2FrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    global_result = RX_FRAME_PENDING;

    const uint32_t bytes_available =
        read_buffer_read_idx > read_buffer_idx
            ? read_buffer_idx + (uint8_t) sizeof(read_buffer) - read_buffer_read_idx
            : read_buffer_idx - read_buffer_read_idx;

    if (bytes_available >= sizeof(srxlv2Header)) {
        union {
            uint8_t local_buffer[sizeof(srxlv2Header)];
            srxlv2Header header;
        } anonymous;

        uint32_t bytes_copied = 0;
        uint32_t index = read_buffer_read_idx;
        while (bytes_copied < sizeof(srxlv2Header)) {
            anonymous.local_buffer[bytes_copied++] = read_buffer[index++];

            if (index == sizeof(read_buffer)) {
                index = 0;
            }
        }

        if (anonymous.header.id != SRXLv2_ID) {
            // @todo reset
          //            DEBUG("invalid header id: %x\r\n", anonymous.header.id);
            // @todo maybe scan data for SRXLv2_ID
            read_buffer_read_idx = read_buffer_idx;
            return RX_FRAME_DROPPED;
        }

        if (bytes_available < anonymous.header.length) {
            return RX_FRAME_PENDING;
        }

        //        cliPrintf("id %x %x %x\r\n", anonymous.header.id, anonymous.header.packet_type, anonymous.header.length);

        srxlv2Process(rxRuntimeConfig);
    }

    uint8_t result = global_result;

    const uint32_t now = micros();

    switch (state) {
    case Disabled: break;

    case ListenForActivity: {
        // activity detected
        if (last_valid_packet_timestamp != 0) {
            // as ListenForActivity is done at default baud-rate, we don't need to change anything
            // @todo if there were non-handshake packets - go to running,
            // if there were - go to either Send Handshake or Listen For Handshake
            state = Running;
        } else if (last_idle_timestamp > last_receive_timestamp) {
            if (baud_rate != 0) {
                uint32_t currentBaud = serialGetBaudRate(serialPort);

                if(currentBaud == SRXLv2_PORT_BAUDRATE_DEFAULT)
                    serialPort->vTable->serialSetBaudRate(serialPort, SRXLv2_PORT_BAUDRATE_HIGH);
                else
                    serialPort->vTable->serialSetBaudRate(serialPort, SRXLv2_PORT_BAUDRATE_DEFAULT);
            }
        } else if (now >= timeout_timestamp) {
            // @todo if there was activity - detect baudrate and ListenForHandshake

            if (unit_id == 0) {
                state = SendHandshake;
                timeout_timestamp = now + SRXLv2_SEND_HANDSHAKE_TIMEOUT;
                full_timeout_timestamp = now + SRXLv2_LISTEN_FOR_HANDSHAKE_TIMEOUT;
            } else {
                state = ListenForHandshake;
                timeout_timestamp = now + SRXLv2_LISTEN_FOR_HANDSHAKE_TIMEOUT;
            }
        }
    } break;

    case SendHandshake: {
        if (now >= timeout_timestamp) {
            // @todo set another timeout for 50ms tries
            // fill write buffer with handshake frame
            result |= RX_FRAME_PROCESSING_REQUIRED;
        }
        // else if (handshake_received) {
        //  set baud rate accordingly
        //  state = Running
        // }

        if (now >= full_timeout_timestamp) {
            serialPort->vTable->serialSetBaudRate(serialPort, SRXLv2_PORT_BAUDRATE_DEFAULT);
            //DEBUG("case SendHandshake: switching to %d baud\r\n", SRXLv2_PORT_BAUDRATE_DEFAULT);
            timeout_timestamp = now + SRXLv2_LISTEN_FOR_ACTIVITY_TIMEOUT;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            state = ListenForActivity;
            last_receive_timestamp = 0;
        }
    } break;

    case ListenForHandshake: {
        if (now >= timeout_timestamp)  {
            serialPort->vTable->serialSetBaudRate(serialPort, SRXLv2_PORT_BAUDRATE_DEFAULT);
            //DEBUG("case ListenForHandshake: switching to %d baud\r\n", SRXLv2_PORT_BAUDRATE_DEFAULT);
            timeout_timestamp = now + SRXLv2_LISTEN_FOR_ACTIVITY_TIMEOUT;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            state = ListenForActivity;
            last_receive_timestamp = 0;
        }
    } break;

    case Running: {
        // frame timed out, reset state
        if (last_valid_packet_timestamp < now && now - last_valid_packet_timestamp >= SRXLv2_FRAME_TIMEOUT_US) {
            serialPort->vTable->serialSetBaudRate(serialPort, SRXLv2_PORT_BAUDRATE_DEFAULT);
            //DEBUG("case Running: switching to %d baud: %d %d\r\n", SRXLv2_PORT_BAUDRATE_DEFAULT, now, last_valid_packet_timestamp);
            timeout_timestamp = now + SRXLv2_LISTEN_FOR_ACTIVITY_TIMEOUT;
            result = (result & ~RX_FRAME_PENDING) | RX_FRAME_FAILSAFE;

            state = ListenForActivity;
            last_receive_timestamp = 0;
            last_valid_packet_timestamp = 0;
        }
    } break;
    };

    if (write_buffer_idx) {
        result |= RX_FRAME_PROCESSING_REQUIRED;
    }

    return result;
}

static bool srxlv2ProcessFrame(const rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    if (write_buffer_idx == 0) {
        return true;
    }

    const uint32_t now = micros();

    if (last_idle_timestamp > last_receive_timestamp) {
        // time sufficient for at least 2 characters has passed
        if (now - last_receive_timestamp > SRXLv2_REPLY_QUIESCENCE) {
            serialWriteBuf(serialPort, write_buffer, write_buffer_idx);
            write_buffer_idx = 0;
        } else {
          //            DEBUG("not enough time to send 2 characters passed yet, %d us since last receive, %d required\r\n", now - last_receive_timestamp, SRXLv2_REPLY_QUIESCENCE);
        }
    } else {
        // DEBUG("still receiving a frame, %d %d\r\n", last_idle_timestamp, last_receive_timestamp);
    }

    return true;
}

static uint16_t srxlv2ReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel_idx)
{
    if (channel_idx >= rxRuntimeConfig->channelCount) {
        return 0;
    }

    return 988 + ((rxRuntimeConfig->channelData[channel_idx] >> SRXLv2_CHANNEL_SHIFT) >> 1);
}

void srxlv2RxWriteData(const void *data, int len)
{
    len = MIN(len, (int)sizeof(write_buffer));
    memcpy(write_buffer, data, len);
    write_buffer_idx = len;
}

bool srxlv2RxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    static uint16_t channelData[SRXLv2_MAX_CHANNELS];
    for (size_t i = 0; i < SRXLv2_MAX_CHANNELS; ++i) {
        channelData[i] = SRXLv2_CHANNEL_CENTER;
    }

    unit_id = rxConfig->srxlv2_unit_id;
    baud_rate = rxConfig->srxlv2_baud_rate;

    rxRuntimeConfig->channelData = channelData;
    rxRuntimeConfig->channelCount = SRXLv2_MAX_CHANNELS;
    rxRuntimeConfig->rxRefreshRate = SRXLv2_TIME_BETWEEN_FRAMES_US;

    rxRuntimeConfig->rcReadRawFn = srxlv2ReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = srxlv2FrameStatus;
    rxRuntimeConfig->rcProcessFrameFn = srxlv2ProcessFrame;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    portOptions_e options = SRXLv2_PORT_OPTIONS;
    if (rxConfig->serialrx_inverted) {
        options |= SERIAL_INVERTED;
    }
    if (rxConfig->halfDuplex) {
        options |= SERIAL_BIDIR;
    }
    if (rxConfig->halfDuplex == 2) {
        options |= SERIAL_SWAP_RX_TX;
    }

    serialPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, srxlv2DataReceive,
        NULL, SRXLv2_PORT_BAUDRATE_DEFAULT, SRXLv2_PORT_MODE, options);

    if (serialPort) {
        serialPort->idleCallback = srxlv2Idle;

        state = ListenForActivity;
        timeout_timestamp = micros() + SRXLv2_LISTEN_FOR_ACTIVITY_TIMEOUT;

        if (rssiSource == RSSI_SOURCE_NONE) {
            rssiSource = RSSI_SOURCE_RX_PROTOCOL;
        }
    }

    /* handshake protocol
    1. listen for 50ms for serial activity and go to State::Running if found, autobaud may be necessary
    2. if srxlv2_unit_id = 0:
            send a Handshake with destination_device_id = 0 every 50ms for at least 200ms
        else:
            listen for Handshake for at least 200ms
    3.  respond to Handshake as currently implemented in process if rePst received
    4.  respond to broadcast Handshake
    */

    // if 50ms with not activity, go to default baudrate and to step 1

    return serialPort;
}

bool srxlv2RxIsActive(void)
{
    return serialPort;
}

bool srxlv2TelemetryRequested(void)
{
    return telemetry_requested;
}

void srxlv2InitializeFrame(sbuf_t *dst)
{
    dst->ptr = telemetryFrame;
    dst->end = ARRAYEND(telemetryFrame);

    sbufWriteU8(dst, SRXLv2_ID);
    sbufWriteU8(dst, TelemetrySensorData);
    sbufWriteU8(dst, ARRAYLEN(telemetryFrame));
    sbufWriteU8(dst, bus_master_device_id);
}

void srxlv2FinalizeFrame(sbuf_t *dst)
{
  const uint16_t crc = crc16_ccitt_update(0, telemetryFrame, sbufPtr(dst) - telemetryFrame);
  sbufWriteU16BigEndian(dst, crc);

  sbufSwitchToReader(dst, telemetryFrame);
  srxlv2RxWriteData(sbufPtr(dst), sbufBytesRemaining(dst));
  telemetry_requested = false;
}

void srxlv2Bind(void)
{
    const size_t length = sizeof(srxlv2BindInfoFrame);

    srxlv2BindInfoFrame bind = {
        .header = {
            .id = SRXLv2_ID,
            .packet_type = BindInfo,
            .length = length
        },
        .payload = {
            .request = EnterBindMode,
            .device_id = bus_master_device_id,
            .bind_type = DMSX_11ms,
            .options = SRXL_BIND_OPT_TELEM_TX_ENABLE | SRXL_BIND_OPT_BIND_TX_ENABLE,
        }
    };

    const uint16_t crc = crc16_ccitt_update(0, &bind, length - 2);
    bind.crc_high = ((uint8_t *) &crc)[1];
    bind.crc_low = ((uint8_t *) &crc)[0];

    srxlv2RxWriteData(&bind, length);
}

#endif
