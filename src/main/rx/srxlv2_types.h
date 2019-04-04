#pragma once

#define PACKED __attribute__((packed))

typedef enum {
    Disabled,
    ListenForActivity,
    SendHandshake,
    ListenForHandshake,
    Running
} srxlv2State;

typedef enum {
    Handshake = 0x21,
    BindInfo = 0x41,
    ParameterConfiguration = 0x50,
    SignalQuality = 0x55,
    TelemetrySensorData = 0x80,
    ControlData = 0xCD,
} srxlv2PacketType;

typedef struct {
    uint8_t id;
    uint8_t packet_type;
    uint8_t length;
} PACKED srxlv2Header;

typedef struct {
    uint8_t source_device_id;
    uint8_t destination_device_id;
    uint8_t priority;
    uint8_t baud_supported;
    uint8_t info;
    uint32_t unique_id;
} PACKED srxlv2HandshakeSubHeader;

typedef struct {
    uint8_t command;
    uint8_t reply_id;
} PACKED srxlv2ControlDataSubHeader;

typedef enum {
    ChannelData = 0x00,
    FailsafeChannelData = 0x01,
    VTXData = 0x02,
} srxlv2ControlDataCommand;

typedef struct {
    int8_t rssi;
    uint16_t frame_losses;
    union {
        struct {
            uint8_t channels_0_7;
            uint8_t channels_8_15;
            uint8_t channels_16_23;
            uint8_t channels_24_31;
        } u8;
        uint32_t u32;
    } channel_mask;
} PACKED srxlv2ChannelDataHeader;

typedef enum {
    NoDevice = 0,
    RemoteReceiver = 1,
    Receiver = 2,
    FlightController = 3,
    ESC = 4,
    Reserved = 5,
    SRXLServo = 6,
    SRXLServo_2 = 7,
    VTX = 8,
} srxlV2DeviceType;

typedef enum {
    FlightControllerDefault = 0x30,
    FlightControllerMax = 0x3F,
    Broadcast = 0xFF,
} srxlv2DeviceId;

typedef struct {
    srxlv2Header header;
    srxlv2HandshakeSubHeader payload;
    uint8_t crc_high;
    uint8_t crc_low;
} PACKED srxlv2HandshakeFrame;

typedef enum {
    EnterBindMode = 0xEB,
    RequestBindStatus = 0xB5,
    BoundDataReport = 0xDB,
    SetBindInfo = 0x5B,
} srxlv2BindRequest;

typedef enum {
    NotBound = 0x0,
    DSM2_1024_22ms = 0x01,
    DSM2_1024_MC24 = 0x02,
    DMS2_2048_11ms = 0x12,
    DMSX_22ms = 0xA2,
    DMSX_11ms = 0xB2,
    Surface_DSM2_16_5ms = 0x63,
    DSMR_11ms_22ms = 0xE2,
    DSMR_5_5ms = 0xE4,
} srxlv2BindType;

// Bit masks for Options byte
#define SRXL_BIND_OPT_NONE              (0x00)
#define SRXL_BIND_OPT_TELEM_TX_ENABLE   (0x01)  // Set if this device should be enabled as the current telemetry device to tx over RF
#define SRXL_BIND_OPT_BIND_TX_ENABLE    (0x02)  // Set if this device should reply to a bind request with a Discover packet over RF

typedef struct {
  uint8_t request;
  uint8_t device_id;
  uint8_t bind_type;
  uint8_t options;
  uint64_t guid;
  uint32_t uid;
} PACKED srxlv2BindInfoPayload;

typedef struct {
  srxlv2Header header;
  srxlv2BindInfoPayload payload;
  uint8_t crc_high;
  uint8_t crc_low;
} PACKED srxlv2BindInfoFrame;

// VTX Data
typedef struct
{
  uint8_t   band;     // VTX Band (0 = Fatshark, 1 = Raceband, 2 = E, 3 = B, 4 = A)
  uint8_t   channel;  // VTX Channel (0-7)
  uint8_t   pit;      // Pit/Race mode (0 = Race, 1 = Pit). Race = (normal operating) mode. Pit = (reduced power) mode.
  uint8_t   power;    // VTX Power (0 = Off, 1 = 1mw to 14mW, 2 = 15mW to 25mW, 3 = 26mW to 99mW, 4 = 100mW to 299mW, 5 = 300mW to 600mW, 6 = 601mW+, 7 = manual control)
  uint16_t  powerDec; // VTX Power as a decimal 1mw/unit
  uint8_t   region;   // Region (0 = USA, 1 = EU)
} PACKED srxlv2VtxData;

#undef PACKED
