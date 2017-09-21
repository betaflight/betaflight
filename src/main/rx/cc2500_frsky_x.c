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

#include <string.h>
#include <sys/_stdint.h>

#include "platform.h"

#ifdef USE_RX_FRSKY_SPI_X

#include "build/build_config.h"
#include "build/debug.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/adc.h"
#include "drivers/cc2500.h"
#include "drivers/io.h"
#include "drivers/io_def.h"
#include "drivers/io_types.h"
#include "drivers/resource.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/config.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_frsky_shared.h"
#include "rx/cc2500_frsky_x.h"

#include "sensors/battery.h"

#include "telemetry/smartport.h"

#define RC_CHANNEL_COUNT 16

const uint16_t CRCTable[] = {
        0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
        0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
        0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,
        0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
        0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,
        0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
        0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,
        0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
        0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,
        0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
        0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,
        0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
        0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,
        0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
        0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,
        0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
        0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,
        0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
        0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,
        0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
        0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,
        0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
        0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,
        0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
        0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,
        0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
        0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,
        0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
        0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,
        0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
        0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,
        0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78
};

#define TELEMETRY_OUT_BUFFER_SIZE  64

#define TELEMETRY_SEQUENCE_LENGTH 4

typedef struct telemetrySequenceMarkerData_s {
    unsigned int packetSequenceId: 2;
    unsigned int unused: 1;
    unsigned int initRequest: 1;
    unsigned int ackSequenceId: 2;
    unsigned int retransmissionRequested: 1;
    unsigned int initResponse: 1;
} __attribute__ ((__packed__)) telemetrySequenceMarkerData_t;

typedef union telemetrySequenceMarker_s {
    uint8_t raw;
    telemetrySequenceMarkerData_t data;
} __attribute__ ((__packed__)) telemetrySequenceMarker_t;

#define SEQUENCE_MARKER_REMOTE_PART 0xf0

#define TELEMETRY_DATA_SIZE 5

typedef struct telemetryData_s {
    uint8_t dataLength;
    uint8_t data[TELEMETRY_DATA_SIZE];
} __attribute__ ((__packed__)) telemetryData_t;

typedef struct telemetryBuffer_s {
    telemetryData_t data;
    uint8_t needsProcessing;
} telemetryBuffer_t;

#define TELEMETRY_FRAME_SIZE  sizeof(telemetryData_t)

typedef struct telemetryPayload_s {
    uint8_t packetConst;
    uint8_t rssiA1;
    telemetrySequenceMarker_t sequence;
    telemetryData_t data;
    uint8_t crc[2];
} __attribute__ ((__packed__)) telemetryPayload_t;

static telemetryBuffer_t telemetryRxBuffer[TELEMETRY_SEQUENCE_LENGTH];
static telemetryData_t telemetryTxBuffer[TELEMETRY_SEQUENCE_LENGTH];

static uint8_t remoteProcessedId = 0;
static uint8_t remoteAckId = 0;

static uint8_t remoteToProcessIndex = 0;

static uint8_t localPacketId;

static telemetrySequenceMarker_t responseToSend;

static uint8_t ccLen;
static uint32_t missingPackets;
static uint8_t calData[255][3];
static uint8_t cnt;
static timeDelta_t t_out;
static timeUs_t packet_timer;
static uint8_t protocolState;
static int16_t word_temp;
static uint32_t start_time;

static bool frame_received;
static uint8_t one_time=1;
static uint8_t chanskip=1;

#if defined(USE_RX_FRSKY_SPI_PA_LNA)
static uint8_t pass;
#endif
static timeDelta_t  t_received;
#ifdef USE_RX_FRSKY_SPI_TELEMETRY
static uint8_t frame[20];
static uint8_t telemetryRX;
#if defined(USE_TELEMETRY_SMARTPORT)
static uint8_t telemetryOutReader = 0;
static uint8_t telemetryOutWriter;

static uint8_t telemetryOutBuffer[TELEMETRY_OUT_BUFFER_SIZE];

static bool telemetryEnabled = false;
#endif
#endif


bool frskySpiDetect(void)//debug CC2500 spi
{
    uint8_t tmp[2];
    tmp[0] = cc2500ReadReg(CC2500_30_PARTNUM | CC2500_READ_BURST);//Cc2500 read registers chip part num
    tmp[1] = cc2500ReadReg(CC2500_31_VERSION | CC2500_READ_BURST);//Cc2500 read registers chip version
    if (tmp[0] == 0x80 && tmp[1]==0x03){
        return true;
    }
    return false;
}


static uint16_t crc(uint8_t *data, uint8_t len) {
    uint16_t crc = 0;
    for(uint8_t i=0; i < len; i++)
        crc = (crc<<8) ^ (CRCTable[((uint8_t)(crc>>8) ^ *data++) & 0xFF]);
    return crc;
}

#if defined(USE_TELEMETRY_SMARTPORT)
static uint8_t frsky_append_sport_data(uint8_t *buf)
{
    uint8_t index;
    for (index = 0; index < TELEMETRY_DATA_SIZE; index++) { //max 5 bytes in a frame
        if(telemetryOutReader == telemetryOutWriter){ //no new data
            break;
        }
        buf[index] = telemetryOutBuffer[telemetryOutReader];
        telemetryOutReader = (telemetryOutReader + 1) % TELEMETRY_OUT_BUFFER_SIZE;
    }

    return index;
}
#endif

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
static void telemetry_build_frame(uint8_t *packet)
{
    frame[0]=0x0E;//length
    frame[1]=rxFrSkySpiConfig()->bindTxId[0];
    frame[2]=rxFrSkySpiConfig()->bindTxId[1];
    frame[3]=packet[3];

    static bool evenRun = false;
    if (evenRun) {
        frame[4]=(uint8_t)RSSI_dBm|0x80;
    } else {
        const uint16_t adcExternal1Sample = adcGetChannel(ADC_EXTERNAL1);
        frame[4]=(uint8_t)((adcExternal1Sample & 0xfe0) >> 5); // A1;
    }
    evenRun = !evenRun;

    telemetrySequenceMarker_t *inFrameMarker = (telemetrySequenceMarker_t *)&packet[21];
    telemetrySequenceMarker_t *outFrameMarker = (telemetrySequenceMarker_t *)&frame[5];
    if (inFrameMarker->data.initRequest) {//check syncronization at startup ok if not no sport telemetry
        outFrameMarker-> raw = 0;
        outFrameMarker->data.initRequest = 1;
        outFrameMarker->data.initResponse = 1;

        localPacketId = 0;
    } else {
        if (inFrameMarker->data.retransmissionRequested) {
            uint8_t retransmissionFrameId = inFrameMarker->data.ackSequenceId;
            outFrameMarker->raw = responseToSend.raw & SEQUENCE_MARKER_REMOTE_PART;
            outFrameMarker->data.packetSequenceId = retransmissionFrameId;

            memcpy(&frame[6], &telemetryTxBuffer[retransmissionFrameId], TELEMETRY_FRAME_SIZE);
        } else {
            uint8_t localAckId = inFrameMarker->data.ackSequenceId;
            if (localPacketId != (localAckId + 1) % TELEMETRY_SEQUENCE_LENGTH) {
                outFrameMarker->raw = responseToSend.raw & SEQUENCE_MARKER_REMOTE_PART;
                outFrameMarker->data.packetSequenceId = localPacketId;
\
                frame[6] = frsky_append_sport_data(&frame[7]);
                memcpy(&telemetryTxBuffer[localPacketId], &frame[6], TELEMETRY_FRAME_SIZE);

                localPacketId = (localPacketId + 1) % TELEMETRY_SEQUENCE_LENGTH;
            }
        }
    }

    uint16_t lcrc = crc(&frame[3], 10);
    frame[13]=lcrc>>8;
    frame[14]=lcrc;
}

static bool frSkyXCheckQueueEmpty(void)
{
    return true;
}

#if defined(USE_TELEMETRY_SMARTPORT)
static void frSkyXTelemetrySendByte(uint8_t c) {
    if (c == FSSP_DLE || c == FSSP_START_STOP) {
        telemetryOutBuffer[telemetryOutWriter] = FSSP_DLE;
        telemetryOutWriter = (telemetryOutWriter + 1) % TELEMETRY_OUT_BUFFER_SIZE;
        telemetryOutBuffer[telemetryOutWriter] = c ^ FSSP_DLE_XOR;
        telemetryOutWriter = (telemetryOutWriter + 1) % TELEMETRY_OUT_BUFFER_SIZE;
    } else {
        telemetryOutBuffer[telemetryOutWriter] = c;
        telemetryOutWriter = (telemetryOutWriter + 1) % TELEMETRY_OUT_BUFFER_SIZE;
    }
}

static void frSkyXTelemetryWriteFrame(const smartPortPayload_t *payload)
{
    telemetryOutBuffer[telemetryOutWriter] = FSSP_START_STOP;
    telemetryOutWriter = (telemetryOutWriter + 1) % TELEMETRY_OUT_BUFFER_SIZE;
    telemetryOutBuffer[telemetryOutWriter] = FSSP_SENSOR_ID1 & 0x1f;
    telemetryOutWriter = (telemetryOutWriter + 1) % TELEMETRY_OUT_BUFFER_SIZE;
    uint8_t *data = (uint8_t *)payload;
    for (unsigned i = 0; i < sizeof(smartPortPayload_t); i++) {
        frSkyXTelemetrySendByte(*data++);
    }
}
#endif
#endif // USE_RX_FRSKY_SPI_TELEMETRY


static void initialize() {
    cc2500Reset();
    cc2500WriteReg(CC2500_02_IOCFG0,   0x01);
    cc2500WriteReg(CC2500_17_MCSM1,    0x0C);
    cc2500WriteReg(CC2500_18_MCSM0,    0x18);
    cc2500WriteReg(CC2500_06_PKTLEN,   0x1E);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x04);
    cc2500WriteReg(CC2500_08_PKTCTRL0, 0x01);
    cc2500WriteReg(CC2500_3E_PATABLE,  0xFF);
    cc2500WriteReg(CC2500_0B_FSCTRL1,  0x0A);
    cc2500WriteReg(CC2500_0C_FSCTRL0,  0x00);
    cc2500WriteReg(CC2500_0D_FREQ2,    0x5C);
    cc2500WriteReg(CC2500_0E_FREQ1,    0x76);
    cc2500WriteReg(CC2500_0F_FREQ0,    0x27);
    cc2500WriteReg(CC2500_10_MDMCFG4,  0x7B);
    cc2500WriteReg(CC2500_11_MDMCFG3,  0x61);
    cc2500WriteReg(CC2500_12_MDMCFG2,  0x13);
    cc2500WriteReg(CC2500_13_MDMCFG1,  0x23);
    cc2500WriteReg(CC2500_14_MDMCFG0,  0x7A);
    cc2500WriteReg(CC2500_15_DEVIATN,  0x51);
    cc2500WriteReg(CC2500_19_FOCCFG,   0x16);
    cc2500WriteReg(CC2500_1A_BSCFG,    0x6C);
    cc2500WriteReg(CC2500_1B_AGCCTRL2, 0x03);
    cc2500WriteReg(CC2500_1C_AGCCTRL1, 0x40);
    cc2500WriteReg(CC2500_1D_AGCCTRL0, 0x91);
    cc2500WriteReg(CC2500_21_FREND1,   0x56);
    cc2500WriteReg(CC2500_22_FREND0,   0x10);
    cc2500WriteReg(CC2500_23_FSCAL3,   0xA9);
    cc2500WriteReg(CC2500_24_FSCAL2,   0x0A);
    cc2500WriteReg(CC2500_25_FSCAL1,   0x00);
    cc2500WriteReg(CC2500_26_FSCAL0,   0x11);
    cc2500WriteReg(CC2500_29_FSTEST,   0x59);
    cc2500WriteReg(CC2500_2C_TEST2,    0x88);
    cc2500WriteReg(CC2500_2D_TEST1,    0x31);
    cc2500WriteReg(CC2500_2E_TEST0,    0x0B);
    cc2500WriteReg(CC2500_03_FIFOTHR,  0x07);
    cc2500WriteReg(CC2500_09_ADDR,     0x00);
    cc2500Strobe(CC2500_SIDLE);	// Go to idle...

    for(uint8_t c=0;c<0xFF;c++)
    {//calibrate all channels
        cc2500Strobe(CC2500_SIDLE);
        cc2500WriteReg(CC2500_0A_CHANNR, c);
        cc2500Strobe(CC2500_SCAL);
        delayMicroseconds(900);	//
        calData[c][0] = cc2500ReadReg(CC2500_23_FSCAL3);
        calData[c][1] = cc2500ReadReg(CC2500_24_FSCAL2);
        calData[c][2] = cc2500ReadReg(CC2500_25_FSCAL1);
    }
    //#######END INIT########
}


void frSkyXSetRcData(uint16_t *rcData, const uint8_t *packet)
{
    uint16_t c[8];

    c[0] = (uint16_t)((packet[10] <<8)& 0xF00) | packet[9];
    c[1] = (uint16_t)((packet[11]<<4)&0xFF0) | (packet[10]>>4);
    c[2] = (uint16_t)((packet[13] <<8)& 0xF00) | packet[12];
    c[3] = (uint16_t)((packet[14]<<4)&0xFF0) | (packet[13]>>4);
    c[4] = (uint16_t)((packet[16] <<8)& 0xF00) | packet[15];
    c[5] = (uint16_t)((packet[17]<<4)&0xFF0) | (packet[16]>>4);
    c[6] = (uint16_t)((packet[19] <<8)& 0xF00) | packet[18];
    c[7] = (uint16_t)((packet[20]<<4)&0xFF0) | (packet[19]>>4);

    uint8_t j;
    for(uint8_t i=0;i<8;i++) {
        if(c[i] > 2047)  {
            j = 8;
            c[i] = c[i] - 2048;
        } else {
            j = 0;
        }
        word_temp = (((c[i]-64)<<1)/3+860);
        if ((word_temp > 800) && (word_temp < 2200))
            rcData[i+j] = word_temp;
    }
}

rx_spi_received_e frSkyXDataReceived(uint8_t *packet)
{
    static unsigned receiveTelemetryRetryCount = 0;
    static uint32_t polling_time=0;

    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    switch (protocolState) {
    case STATE_INIT:
        if ((millis() - start_time) > 10) {
            initialize();

            protocolState = STATE_BIND;
        }

        break;
    case STATE_BIND:
    case STATE_BIND_TUNING:
    case STATE_BIND_BINDING1:
    case STATE_BIND_BINDING2:
    case STATE_BIND_COMPLETE:
        handleBinding(protocolState, packet);

        break;
    case STATE_STARTING:
        listLength = 47;
        initialiseData(0);
        protocolState = STATE_UPDATE;
        nextChannel(1, false); //
        cc2500Strobe(CC2500_SRX);
        ret = RX_SPI_RECEIVED_BIND;		
        break;
    case STATE_UPDATE:
        packet_timer = micros();
        protocolState = STATE_DATA;
        frame_received=false;//again set for receive
        t_received = 5300;
        if (checkBindRequested(false)) {
            packet_timer = 0;
            t_out = 50;
            missingPackets = 0;
            protocolState = STATE_INIT;
            break;
        }
        // here FS code could be
    case STATE_DATA:	
        if ((IORead(gdoPin)) &&(frame_received==false)){
            ccLen =cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            ccLen =cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;//read 2 times to avoid reading errors
            if (ccLen > 32)
                ccLen = 32;
            if (ccLen) {
                cc2500ReadFifo(packet, ccLen);				
                uint16_t lcrc= crc(&packet[3],(ccLen-7));
                if((lcrc >>8)==packet[ccLen-4]&&(lcrc&0x00FF)==packet[ccLen-3]){//check crc
                    if (packet[0] == 0x1D) {
                        if ((packet[1] == rxFrSkySpiConfig()->bindTxId[0]) &&
                                (packet[2] == rxFrSkySpiConfig()->bindTxId[1]) &&
                                (packet[6]==rxFrSkySpiConfig()->rxNum)) {
                            missingPackets = 0;
                            t_out = 1;
                            t_received = 0;
                            IOHi(frSkyLedPin);
                            if(one_time){
                                chanskip=packet[5]<<2;
                                if(packet[4]<listLength){}
                                else if(packet[4]<(64+listLength))
                                    chanskip +=1;
                                else if(packet[4]<(128+listLength))
                                    chanskip +=2;
                                else if(packet[4]<(192+listLength))
                                    chanskip +=3;
                                telemetryRX=1;//now telemetry can be sent
                                one_time=0;
                            }
#ifdef USE_RX_FRSKY_SPI_TELEMETRY
                            setRssiDbm(packet[ccLen - 2]);
#endif

                            telemetrySequenceMarker_t *inFrameMarker = (telemetrySequenceMarker_t *)&packet[21];

                            uint8_t remoteNewPacketId = inFrameMarker->data.packetSequenceId;
                            memcpy(&telemetryRxBuffer[remoteNewPacketId].data, &packet[22], TELEMETRY_FRAME_SIZE);
                            telemetryRxBuffer[remoteNewPacketId].needsProcessing = true;

                            responseToSend.raw = 0;
                            uint8_t remoteToAckId = (remoteAckId + 1) % TELEMETRY_SEQUENCE_LENGTH;
                            if (remoteNewPacketId != remoteToAckId) {
                                while (remoteToAckId != remoteNewPacketId) {
                                    if (!telemetryRxBuffer[remoteToAckId].needsProcessing) {
                                        responseToSend.data.ackSequenceId = remoteToAckId;
                                        responseToSend.data.retransmissionRequested = 1;

                                        receiveTelemetryRetryCount++;

                                        break;
                                    }

                                    remoteToAckId = (remoteToAckId + 1) % TELEMETRY_SEQUENCE_LENGTH;
                                }
                            }

                            if (!responseToSend.data.retransmissionRequested) {
                                receiveTelemetryRetryCount = 0;

                                remoteToAckId = (remoteAckId + 1) % TELEMETRY_SEQUENCE_LENGTH;
                                uint8_t remoteNextAckId;
                                while (telemetryRxBuffer[remoteToAckId].needsProcessing && remoteToAckId != remoteAckId) {
                                    remoteNextAckId = remoteToAckId;
                                    remoteToAckId = (remoteToAckId + 1) % TELEMETRY_SEQUENCE_LENGTH;
                                }
                                remoteAckId = remoteNextAckId;
                                responseToSend.data.ackSequenceId = remoteAckId;
                            }

                            if (receiveTelemetryRetryCount >= 5) {
                                 remoteProcessedId =  TELEMETRY_SEQUENCE_LENGTH - 1;
                                 remoteAckId =  TELEMETRY_SEQUENCE_LENGTH - 1;
                                 for (unsigned i = 0; i < TELEMETRY_SEQUENCE_LENGTH; i++) {
                                     telemetryRxBuffer[i].needsProcessing = false;
                                 }

                                 receiveTelemetryRetryCount = 0;
                             }

                            packet_timer=micros();
                            frame_received=true;//no need to process frame again.
                        }
                    }				
                }
            }
        }
        if (telemetryRX) {
            if(cmpTimeUs(micros(), packet_timer) > t_received) { // if received or not received in this time sent telemetry data
                protocolState=STATE_TELEMETRY;
                telemetry_build_frame(packet);
            }
        }
        if (cmpTimeUs(micros(), packet_timer) > t_out * SYNC) {
            if (cnt++ & 0x01) {
                IOLo(frSkyLedPin);
            } else {
                IOHi(frSkyLedPin);
            }
            //telemetryTime=micros();
#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
            setRssiFiltered(0, RSSI_SOURCE_RX_PROTOCOL);
#endif
            nextChannel(1, false);
            cc2500Strobe(CC2500_SRX);
            protocolState = STATE_UPDATE;
        }
        break;
#ifdef USE_RX_FRSKY_SPI_TELEMETRY
    case STATE_TELEMETRY:
        if(cmpTimeUs(micros(), packet_timer) >= t_received + 400) { // if received or not received in this time sent telemetry data
            cc2500Strobe(CC2500_SIDLE);
            cc2500SetPower(6);
            cc2500Strobe(CC2500_SFRX);
            delayMicroseconds(30);
#if defined(USE_RX_FRSKY_SPI_PA_LNA)
            TxEnable();
#endif
            cc2500Strobe(CC2500_SIDLE);
            cc2500WriteFifo(frame, frame[0] + 1);

#if defined(USE_TELEMETRY_SMARTPORT)
            if (telemetryEnabled) {
                bool clearToSend = false;
                uint32_t now = millis();
                smartPortPayload_t *payload = NULL;
                if ((now - polling_time) > 24) {
                    polling_time=now;

                    clearToSend = true;
                } else {
                    uint8_t remoteToProcessId = (remoteProcessedId + 1) % TELEMETRY_SEQUENCE_LENGTH;
                    while (telemetryRxBuffer[remoteToProcessId].needsProcessing && !payload) {
                        while (remoteToProcessIndex < telemetryRxBuffer[remoteToProcessId].data.dataLength && !payload) {
                            payload = smartPortDataReceive(telemetryRxBuffer[remoteToProcessId].data.data[remoteToProcessIndex], &clearToSend, frSkyXCheckQueueEmpty, false);
                            remoteToProcessIndex = remoteToProcessIndex + 1;
                        }

                        if (remoteToProcessIndex == telemetryRxBuffer[remoteToProcessId].data.dataLength) {
                            remoteToProcessIndex = 0;
                            telemetryRxBuffer[remoteToProcessId].needsProcessing = false;
                            remoteProcessedId = remoteToProcessId;
                            remoteToProcessId = (remoteProcessedId + 1) % TELEMETRY_SEQUENCE_LENGTH;
                        }
                    }
                }
                processSmartPortTelemetry(payload, &clearToSend, NULL);
            }
#endif
            protocolState = STATE_RESUME;
            ret = RX_SPI_RECEIVED_DATA;
        }

        break;
#endif // USE_RX_FRSKY_SPI_TELEMETRY
    case STATE_RESUME:
        if (cmpTimeUs(micros(), packet_timer) > t_received + 3700) {
            packet_timer = micros();
            t_received=5300;
            frame_received=false;//again set for receive
            nextChannel(chanskip, false);
            cc2500Strobe(CC2500_SRX);
#ifdef USE_RX_FRSKY_SPI_PA_LNA
            RxEnable();
#ifdef USE_RX_FRSKY_SPI_DIVERSITY // SE4311 chip
            if (missingPackets >= 2) {
                if (pass & 0x01)
                {
                    IOHi(antSelPin);
                }
                else
                {
                    IOLo(antSelPin);
                }
                pass++;
            }
#endif
#endif // USE_RX_FRSKY_SPI_PA_LNA
            if (missingPackets > MAX_MISSING_PKT)
            {
                t_out = 50;
                one_time=1;
                telemetryRX=0;
                protocolState = STATE_UPDATE;
                break;
            }
            missingPackets++;
            protocolState = STATE_DATA;
        }
        break;
    }
    return ret;
}

void frSkyXInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT;

    frskySpiRxSetup();

#if defined(USE_TELEMETRY_SMARTPORT)
     telemetryEnabled = initSmartPortTelemetryExternal(frSkyXTelemetryWriteFrame);
#endif
}

#endif
