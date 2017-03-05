/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

/*
 * This file contains definitions for the CRTP protocol, the data transport
 * protocol used for OTA communication to the crazyflie (via either Nordic ESB or
 * Bluetooth LE) and transmitted from the NRF51 to the STM32 via the syslink protocol
 *
 * For more details, see https://wiki.bitcraze.io/projects:crazyflie:crtp
 */

#define CRTP_MAX_DATA_SIZE 30

typedef enum {
  CRTP_PORT_CONSOLE          = 0x00,
  CRTP_PORT_PARAM            = 0x02,
  CRTP_PORT_SETPOINT         = 0x03,
  CRTP_PORT_MEM              = 0x04,
  CRTP_PORT_LOG              = 0x05,
  CRTP_PORT_LOCALIZATION     = 0x06,
  CRTP_PORT_SETPOINT_GENERIC = 0x07,
  CRTP_PORT_PLATFORM         = 0x0D,
  CRTP_PORT_LINK             = 0x0F,
} crtpPort_e;

typedef struct crtpPacket_s
{
    struct{
        uint8_t chan : 2;
        uint8_t link : 2;
        uint8_t port : 4;
    } header;
    uint8_t data[CRTP_MAX_DATA_SIZE];
} __attribute__((packed)) crtpPacket_t;

typedef enum {
  stopType          = 0,
  velocityWorldType = 1,
  zDistanceType     = 2,
  cppmEmuType       = 3,
} crtpCommanderPacketType_e;

// Legacy RPYT data type for supporting existing clients
// See https://wiki.bitcraze.io/projects:crazyflie:crtp:commander
typedef struct crtpCommanderRPYT_s
{
    float roll;       // deg
    float pitch;      // deg
    float yaw;        // deg
    uint16_t thrust;
} __attribute__((packed)) crtpCommanderRPYT_t;

// Commander packet type for emulating CPPM-style setpoints
// Corresponds to crtpCommanderPacketType_e::cppmEmuType
#define CRTP_CPPM_EMU_MAX_AUX_CHANNELS 10
typedef struct crtpCommanderCPPMEmuPacket_s
{
    struct {
        uint8_t numAuxChannels : 4;   // Set to 0 through CRTP_CPPM_EMU_MAX_AUX_CHANNELS
        uint8_t reserved : 4;
    } hdr;
    uint16_t channelRoll;
    uint16_t channelPitch;
    uint16_t channelYaw;
    uint16_t channelThrust;
    uint16_t channelAux[CRTP_CPPM_EMU_MAX_AUX_CHANNELS];
} __attribute__((packed)) crtpCommanderCPPMEmuPacket_t;

//typedef struct crtpCommander_s
//{
//    struct{
//        uint8_t chan : 2;
//        uint8_t link : 2;
//        uint8_t port : 4;
//    }hdr;
//    uint8_t type;
//    uint8_t numChannels;
//    uint16_t channels[14];
//
//} __attribute__((packed)) crtpCommander_t;
