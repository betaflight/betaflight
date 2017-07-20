/*
 This file is part of eLeReS (by Michal Maciakowski - Cyberdrones.com).
 Adapted for MultiWii by Mis (Romuald Bialy)
 Ported to STM by Marbalon (Marcin Baliniak)
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>
#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "config/config_master.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/sensor.h"
#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/rx_spi.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/cli.h"
#include "flight/imu.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/eleres.h"
#include "navigation/navigation.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "scheduler/scheduler.h"

#ifdef USE_RX_ELERES

/*****************    eLeReS compatibile reciver on RFM22B module  ********************/

#define RED_LED_ON {if (!redLed) {rfmSpiWrite(0x0e, 0x04);redLed=1;}}
#define RED_LED_OFF {if (redLed) {rfmSpiWrite(0x0e, 0x00);redLed=0;}}

#define RC_CHANS 12
#define TRANSMIT    1
#define TRANSMITTED 2
#define RECEIVE     4
#define RECEIVED    8
#define PREAMBLE    16

#define DATA_FLAG 1
#define LOCALIZER_FLAG 2
#define RELAY_FLAG 4

#define RF22B_RX_PACKET_RECEIVED_INTERRUPT  0x02
#define RF22B_PACKET_SENT_INTERRUPT         0x04
#define RF22B_VALID_PREAMBLE_INTERRUPT      0x40
#define RF22B_VALID_SYNCWORD_INTERRUPT      0x80
#define DATA_PACKAGE_SIZE 22
#define BIN_OFF_VALUE   1150
#define BIN_ON_VALUE    1850

PG_REGISTER_WITH_RESET_TEMPLATE(eleresConfig_t, eleresConfig, PG_ELERES_CONFIG, 0);

PG_RESET_TEMPLATE(eleresConfig_t, eleresConfig,
                  .eleresFreq = 435,
                  .eleresTelemetryEn = 0,
                  .eleresTelemetryPower = 7,
                  .eleresLocEn = 0,
                  .eleresLocPower = 7,
                  .eleresLocDelay = 240
                 );

static uint8_t hoppingChannel = 1;
static uint8_t redLed=0;
static timeMs_t lastPackTime, nextPackTime, localizerTime;
static uint8_t dataReady, channelHoppingTime, firstRun = 1, locForce=0;
static uint8_t holList[16];
static uint8_t localRssi = 0, quality;
static uint16_t goodFrames;
static volatile uint8_t rfMode;
static uint8_t bkgLocEnable = 0;
static uint8_t bkgLocChlist;
static uint8_t bkgLocBuf[3][10];
static uint8_t bkgLocCnt;
static uint8_t rfTxBuffer[10];
static uint8_t rfRxBuffer[DATA_PACKAGE_SIZE];
static uint8_t txFull = 0;
static uint8_t statusRegisters[2];
static uint8_t *eleresSignaturePtr;

static uint8_t rfmSpiRead(uint8_t address)
{
    return rxSpiReadCommand(address & 0x7f, 0x00);
}

static void rfmSpiWrite(uint8_t address, uint8_t data)
{
    rxSpiWriteCommand(address | 0x80, data);
}

uint16_t eleresRssi(void)
{
    return (localRssi - 18)*1024/106;
}

static void rxReset(void)
{
    rfmSpiWrite(0x07, 1);
    rfmSpiWrite(0x08, 0x03);
    rfmSpiWrite(0x08, 0x00);
    rfmSpiWrite(0x07, 5);
    rfmSpiWrite(0x05, RF22B_RX_PACKET_RECEIVED_INTERRUPT);
    rfmSpiWrite(0x06, RF22B_VALID_SYNCWORD_INTERRUPT);
    rfmSpiRead(0x03);
    rfmSpiRead(0x04);
}


static void toReadyMode(void)
{
    rfmSpiWrite(0x07, 1);
    rfmSpiWrite(0x05, 0);
    rfmSpiWrite(0x06, 0);
    rfmSpiRead(0x03);
    rfmSpiRead(0x04);
    rfMode = 0;
}

static void toRxMode(void)
{
    toReadyMode();
    rxReset();
    rfMode = RECEIVE;
}


static void toTxMode(uint8_t bytes_to_send)
{
    uint8_t i;

    toReadyMode();

    rfmSpiWrite(0x08, 0x03);
    rfmSpiWrite(0x08, 0x00);

    rfmSpiWrite(0x3e, bytes_to_send);
    for (i = 0; i<bytes_to_send; i++)
        rfmSpiWrite(0x7f, rfTxBuffer[i]);

    rfmSpiWrite(0x05, RF22B_PACKET_SENT_INTERRUPT);
    rfmSpiWrite(0x06, 0);
    rfmSpiRead(0x03);
    rfmSpiRead(0x04);

    rfmSpiWrite(0x07, 0x09);
    rfMode = TRANSMIT;
    txFull = 0;
}

static void frequencyConfigurator(uint32_t frequency)
{
    uint8_t band;

    if (frequency<48000) {
        frequency -= 24000;
        band = frequency/1000;
        if (band>23) band = 23;
        frequency -= (1000*(uint32_t)band);
        frequency *= 64;
        band |= 0x40;
    } else {
        frequency -= 48000;
        band = frequency/2000;
        if (band>22) band = 22;
        frequency -= (2000*(uint32_t)band);
        frequency *= 32;
        band |= 0x60;
    }
    rfmSpiWrite(0x75, band);
    rfmSpiWrite(0x76, (uint16_t)frequency >> 8);
    rfmSpiWrite(0x77, (uint8_t)frequency);
    rfmSpiWrite(0x79, 0);
}

static void rfm22bInitParameter(void)
{
    int8_t i;
    static uint8_t first_init = 1;

    static uint8_t cf1[9] = {0x00,0x01,0x00,0x7f,0x07,0x52,0x55,0xCA};
    static uint8_t cf2[6] = {0x68,0x01,0x3a,0x93,0x02,0x6b};
    static uint8_t cf3[8] = {0x0f,0x42,0x07,0x20,0x2d,0xd4,0x00,0x00};
    rfmSpiRead(0x03);
    rfmSpiRead(0x04);
    for (i = 0; i < 8; i++)
        rfmSpiWrite(0x06+i, cf1[i]);
    if (first_init) {
        first_init = 0;
        rfmSpiWrite(0x0e, 0);
    }

    rfmSpiWrite(0x6e, 0x09);
    rfmSpiWrite(0x6f, 0xD5);
    rfmSpiWrite(0x1c, 0x02);
    rfmSpiWrite(0x70, 0x00);
    for (i=0; i<6; i++) rfmSpiWrite(0x20+i, cf2[i]);
    rfmSpiWrite(0x2a, 0x1e);
    rfmSpiWrite(0x72, 0x1F);
    rfmSpiWrite(0x30, 0x8c);
    rfmSpiWrite(0x3e, 22);
    for (i=0; i<8; i++) rfmSpiWrite(0x32+i, cf3[i]);
    for (i=0; i<4; i++) rfmSpiWrite(0x43+i, 0xff);
    rfmSpiWrite(0x6d, eleresConfig()->eleresTelemetryPower | 0x18);
    rfmSpiWrite(0x79, 0x00);
    rfmSpiWrite(0x7a, 0x04);
    rfmSpiWrite(0x71, 0x23);
    rfmSpiWrite(0x73, 0x00);
    rfmSpiWrite(0x74, 0x00);
    for (i=0; i<4; i++) {
        rfmSpiWrite(0x3a+i, eleresSignaturePtr[i]);
        rfmSpiWrite(0x3f+i, eleresSignaturePtr[i]);
    }

    frequencyConfigurator((uint32_t)(eleresConfig()->eleresFreq * 100));
    rfmSpiRead(0x03);
    rfmSpiRead(0x04);
}

static void channelHopping(uint8_t hops)
{
    hoppingChannel += hops;
    while (hoppingChannel >= 16) hoppingChannel -= 16;

    if (eleresConfig()->eleresTelemetryEn && eleresConfig()->eleresLocEn) {
        if (bkgLocEnable && (hoppingChannel==bkgLocChlist || hoppingChannel==(bkgLocChlist+1)%16)) {
            rfmSpiWrite(0x79,0);
            bkgLocEnable = 2;
            return;
        }
        if (bkgLocEnable == 2) bkgLocEnable = 1;
    }

    rfmSpiWrite(0x79, holList[hoppingChannel]);
}

static void telemetryRX(void)
{
    static uint8_t telem_state;
    static int32_t presfil;
    static int16_t thempfil;
    uint8_t i, themp=90, wii_flymode=0;
    uint16_t pres,curr = abs(amperage) / 10;
    union {
        int32_t val;
        uint8_t b[4];
    } cnv;

    if (txFull)
        return;

    memset(rfTxBuffer,0,9);

    presfil  -= presfil/4;
    presfil  += baro.baroPressure;
    thempfil -= thempfil/8;
    thempfil += baro.baroTemperature/10;

    switch (telem_state++) {
    case 0:

        if (presfil>200000) pres = presfil/4 - 50000;
        else pres = 1;

        themp = (uint8_t)(thempfil/80 + 86);

        if (FLIGHT_MODE(FAILSAFE_MODE))    wii_flymode = 7;
        else if (FLIGHT_MODE(PASSTHRU_MODE))  wii_flymode = 8;
        else if (FLIGHT_MODE(NAV_RTH_MODE))  wii_flymode = 6;
        else if (FLIGHT_MODE(NAV_POSHOLD_MODE))  wii_flymode = 5;
        else if (FLIGHT_MODE(HEADFREE_MODE))  wii_flymode = 4;
        else if (FLIGHT_MODE(NAV_ALTHOLD_MODE))      wii_flymode = 3;
        else if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))       wii_flymode = 2;
        else                      wii_flymode = 1;
        if (ARMING_FLAG(ARMED)) wii_flymode |= 0x10;
        rfTxBuffer[0] = 0x54;
        rfTxBuffer[1] = localRssi;
        rfTxBuffer[2] = quality;
        rfTxBuffer[3] = vbat;
        rfTxBuffer[4] = themp;
        rfTxBuffer[5] = curr & 0xff;
        rfTxBuffer[6] = pres>>8;
        rfTxBuffer[7] = pres&0xFF;
        rfTxBuffer[8] = wii_flymode;
        rfTxBuffer[8] |= ((curr>>2) & 0xC0);
        break;

    case 1:
        if (gpsSol.llh.lat) {
            uint16_t hdop = gpsSol.hdop / 10;

            rfTxBuffer[0] = 0x50;
            cnv.val = gpsSol.llh.lat/10;
            for (i=0; i<4; i++) rfTxBuffer[i+1] = cnv.b[i];
            cnv.val = gpsSol.llh.lon/10;
            for (i=0; i<4; i++) rfTxBuffer[i+5] = cnv.b[i];

            rfTxBuffer[4] &= 0x0F;
            rfTxBuffer[4] |= (hdop << 4);
            rfTxBuffer[8] &= 0x1F;
            rfTxBuffer[8] |= ((hdop<<1) & 0xE0);
            break;
        }
        telem_state++;
    case 2:
        if (sensors(SENSOR_GPS)) {
            uint16_t gpsspeed =  (gpsSol.groundSpeed*9L)/250L;
            int16_t course = (gpsSol.groundCourse+360)%360;
#ifdef NAV
            int32_t alt = getEstimatedActualPosition(Z);
#else
            int32_t alt = baro.BaroAlt;
#endif
            uint16_t tim = 0;

            rfTxBuffer[0] = 0x47;
            rfTxBuffer[1] = (STATE(GPS_FIX)<<4) | (gpsSol.numSat & 0x0F);
            if (gpsSol.numSat > 15) rfTxBuffer[1] |= 0x80;
            rfTxBuffer[2] = ((course>>8) & 0x0F) | ((gpsspeed>>4) & 0xF0);
            rfTxBuffer[3] = course & 0xFF;
            rfTxBuffer[4] = gpsspeed & 0xFF;

            rfTxBuffer[5] = (alt/100) >>8;
            rfTxBuffer[6] = (alt/100) & 0xFF;

            rfTxBuffer[7] = tim>>8;
            rfTxBuffer[8] = tim&0xFF;
            break;
        }
        telem_state++;
    default:
        rfTxBuffer[0] = 'D';
        memcpy(rfTxBuffer+1,&debug[0],2);
        memcpy(rfTxBuffer+3,&debug[1],2);
        memcpy(rfTxBuffer+5,&debug[2],2);
        memcpy(rfTxBuffer+7,&debug[3],2);
        telem_state = 0;
        break;
    }
    txFull = 1;
}

//because we shared SPI bus I can't read data in real IRQ, so need to probe this pin from main idle
rx_spi_received_e eleresDataReceived(uint8_t *payload)
{
    UNUSED(payload);

    statusRegisters[0] = 0;
    statusRegisters[1] = 0;

    if (rxSpiCheckIrq())
    {
        statusRegisters[0] = rfmSpiRead(0x03);
        statusRegisters[1] = rfmSpiRead(0x04);
        return RX_SPI_RECEIVED_DATA;
    }

    eleresSetRcDataFromPayload(NULL,NULL);

    return RX_SPI_RECEIVED_NONE;
}

static void parseStatusRegister(const uint8_t *payload)
{
    UNUSED(payload);
    static uint16_t rssifil;
    const timeMs_t irq_time = millis();

    if ((rfMode & RECEIVE) && (statusRegisters[0] & RF22B_RX_PACKET_RECEIVED_INTERRUPT))
        rfMode |= RECEIVED;
    if ((rfMode & TRANSMIT) && (statusRegisters[0] & RF22B_PACKET_SENT_INTERRUPT))
        rfMode |= TRANSMITTED;
    if ((rfMode & RECEIVE) && (statusRegisters[1] & RF22B_VALID_SYNCWORD_INTERRUPT))
        rfMode |= PREAMBLE;

    if (rfMode & RECEIVED) {
        if (bkgLocEnable < 2) {
            lastPackTime = irq_time;
            nextPackTime = irq_time + channelHoppingTime;
        }
        rxSpiReadCommandMulti(0x7f,0x00,rfRxBuffer,DATA_PACKAGE_SIZE);
        rxReset();
        rfMode = RECEIVE;
        if ((rfRxBuffer[0] & 127) == 'S') {
            firstRun = 0;
            goodFrames++;

            if ((rfRxBuffer[21] & 0xF0) == 0x20)
                hoppingChannel = rfRxBuffer[21] & 0x0F;

            channelHoppingTime = (rfRxBuffer[20] & 0x0F)+18;
            dataReady |= DATA_FLAG;
        } else if (eleresConfig()->eleresLocEn && eleresConfig()->eleresTelemetryEn && bkgLocEnable==2) {
            if ((rfRxBuffer[0] == 'H' && rfRxBuffer[2] == 'L') ||
                    rfRxBuffer[0]=='T' || rfRxBuffer[0]=='P' || rfRxBuffer[0]=='G') {
                if (bkgLocCnt==0) bkgLocCnt = 200;
                toReadyMode();
                bkgLocEnable = 0;
                channelHopping(0);
                bkgLocEnable = 2;
                dataReady |= RELAY_FLAG;
            }
        } else if (eleresConfig()->eleresLocEn) {
            if (rfRxBuffer[0] == 0x4c && rfRxBuffer[1] == 0x4f && rfRxBuffer[2] == 0x43) {
                localizerTime = irq_time;
                locForce = 1;
                localRssi = 0x18;
            }
        }

        if ((dataReady & LOCALIZER_FLAG)==0) {
            if (eleresConfig()->eleresTelemetryEn)
                toTxMode(9);
            else
                channelHopping(1);
        }
    }

    if (rfMode & TRANSMITTED) {
        toReadyMode();
        if (dataReady & LOCALIZER_FLAG) {
            rfmSpiWrite(0x79, holList[0]);
        } else if (irq_time-lastPackTime <= 1500 && bkgLocEnable<2)
            channelHopping(1);
        toRxMode();
    }

    if (rfMode & PREAMBLE) {
        uint8_t rssitmp = rfmSpiRead(0x26);
        if (eleresConfig()->eleresLocEn && eleresConfig()->eleresTelemetryEn && bkgLocEnable==2) {
            if (rssitmp>124) rssitmp = 124;
            if (rssitmp<18) rssitmp = 18;
            bkgLocBuf[0][1] = rssitmp + 128;
        } else {
            rssifil -= rssifil/8;
            rssifil += rssitmp;
            localRssi = (rssifil/8 * quality / 100)+10;
            if (localRssi>124) localRssi = 124;
            if (localRssi<18) localRssi = 18;
        }
        rfMode &= ~PREAMBLE;
    }
}

void eleresSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    UNUSED(payload);
    uint16_t rcData4Values[RC_CHANS];
    static uint8_t rx_frames, loc_cnt;
    uint8_t channel_count;
    uint8_t i,n;
    uint16_t temp_int;
    static timeMs_t qtest_time, guard_time, led_time;
    timeMs_t cr_time = millis();
    int red_led_local = 0;
    //uint8_t res = RX_FRAME_PENDING;

    parseStatusRegister(statusRegisters);

    if (cr_time < (led_time + 500))
        red_led_local = 0;
    else if (cr_time < (led_time + 1000))
        red_led_local = 1;
    else
        led_time = cr_time;

    if ((dataReady & LOCALIZER_FLAG) == 0) {
        if (cr_time > nextPackTime+2) {
            if ((cr_time-lastPackTime > 1500) || firstRun) {
                localRssi = 18;
                rfm22bInitParameter();
                toRxMode();
                channelHopping(15);
                nextPackTime += 17L*channelHoppingTime;
                if (eleresConfig()->eleresTelemetryEn) {
                    telemetryRX();
                    toTxMode(9);
                }
                // res = RX_FRAME_FAILSAFE;
            } else {

                if (cr_time-lastPackTime > 3*channelHoppingTime) {
                    red_led_local=1;
                    if (localRssi > 0x18)
                        localRssi--;
                }
                toRxMode();
                channelHopping(1);
                nextPackTime += channelHoppingTime;
            }
        }
        if (cr_time > qtest_time) {
            qtest_time = cr_time + 500;
            quality = goodFrames * 100 / (500/channelHoppingTime);
            if (quality > 100) quality = 100;
            goodFrames = 0;
        }
    }

    if ((dataReady & 0x03) == DATA_FLAG && rcData != NULL) {
        if ((dataReady & RELAY_FLAG)==0) {
            channel_count = rfRxBuffer[20] >> 4;
            if (channel_count < 4)  channel_count = 4;
            if (channel_count > RC_CHANS) channel_count = 12;
            for (i = 0; i<channel_count; i++) {
                temp_int = rfRxBuffer[i+1];
                if (i%2 == 0)
                    temp_int |= ((unsigned int)rfRxBuffer[i/2 + 13] << 4) & 0x0F00;
                else
                    temp_int |= ((unsigned int)rfRxBuffer[i/2 + 13] << 8) & 0x0F00;
                if ((temp_int>799) && (temp_int<2201)) rcData4Values[i] = temp_int;
            }
            n = rfRxBuffer[19];
            for (i=channel_count; i < channel_count+5; i++) {
                if (i > 11) break;
                if (n & 0x01) temp_int = BIN_ON_VALUE;
                else temp_int = BIN_OFF_VALUE;
                rcData4Values[i] = temp_int;
                n >>= 1;
            }
            for (; i<RC_CHANS; i++) rcData4Values[i]=1500;
            for (i=0; i<RC_CHANS; i++) {
                temp_int = rcData4Values[i];
                if (temp_int < rcData[i] -3)  rcData[i] = temp_int+2;
                if (temp_int > rcData[i] +3)  rcData[i] = temp_int-2;
            }

            localizerTime = cr_time + (1000L*eleresConfig()->eleresLocDelay);

            if (eleresConfig()->eleresTelemetryEn) {
                if (eleresConfig()->eleresLocEn) {
                    if (bkgLocEnable == 0) bkgLocCnt=0;
                    if (bkgLocCnt) bkgLocCnt--;

                    if (bkgLocCnt<128)
                        telemetryRX();
                    else
                        memcpy(rfTxBuffer, bkgLocBuf[bkgLocCnt%3], 9);
                } else
                    telemetryRX();
            }

            if (eleresConfig()->eleresTelemetryEn)
                telemetryRX();
        }
        dataReady &= 0xFC;
        led_time = cr_time;
        //res = RX_FRAME_COMPLETE;
    }

    if (eleresConfig()->eleresLocEn) {
        if ((dataReady & 0x03)==(DATA_FLAG | LOCALIZER_FLAG) && rfRxBuffer[19]<128) {
            if (rx_frames == 0) guard_time = lastPackTime;
            if (rx_frames < 250) {
                rx_frames++;
            }
            if (rx_frames > 20 && cr_time-guard_time > (locForce?5000:20000)) {
                dataReady = 0;
                localizerTime = cr_time + (1000L*eleresConfig()->eleresLocDelay);
                rfm22bInitParameter();
                channelHopping(1);
                rx_frames = 0;

                if (locForce && eleresConfig()->eleresTelemetryEn) {
                    bkgLocEnable = 1;
                    temp_int = 0;
                    for (i=0; i<16; i++) {
                        uint16_t mult = holList[i] * holList[(i+1)%16];
                        if (mult > temp_int) {
                            temp_int = mult;
                            bkgLocChlist = i;
                        }
                    }
                }
            }
        }

        if (cr_time-lastPackTime > 8000) {
            rx_frames = 0;
        }
        if (!ARMING_FLAG(ARMED) && cr_time > localizerTime) {
            if ((dataReady & LOCALIZER_FLAG)==0) {
                rfm22bInitParameter();
                rfmSpiWrite(0x6d, eleresConfig()->eleresLocPower);
            }
            dataReady &= 0xFB;
            dataReady |= LOCALIZER_FLAG;
            localizerTime = cr_time+35;
            rfmSpiWrite(0x79, 0);
            red_led_local = 1;
            led_time = cr_time;

            bkgLocEnable = 0;
            if (!(++loc_cnt & 1) && eleresConfig()->eleresTelemetryEn) {
                telemetryRX();
                toTxMode(9);
            } else {
                rfTxBuffer[0] = 0x48;
                rfTxBuffer[1] = 0x45;
                rfTxBuffer[2] = 0x4c;
                rfTxBuffer[3] = 0x50;
                rfTxBuffer[4] = 0x21;
                toTxMode(5);
            }
        }

        if ((ARMING_FLAG(ARMED) || firstRun) && (dataReady & LOCALIZER_FLAG)==0)
            localizerTime = cr_time + (1000L*eleresConfig()->eleresLocDelay);

        if (eleresConfig()->eleresTelemetryEn)
            if (dataReady & RELAY_FLAG) {
                if (rfRxBuffer[0]=='H') bkgLocBuf[0][0]='T';
                if (rfRxBuffer[0]=='T') {
                    bkgLocBuf[0][0]='T';
                    memcpy(bkgLocBuf[0]+2, rfRxBuffer+2, 7);
                }
                if (rfRxBuffer[0]=='P') memcpy(bkgLocBuf[1], rfRxBuffer, 9);
                if (rfRxBuffer[0]=='G') memcpy(bkgLocBuf[2], rfRxBuffer, 9);
                dataReady = 0;
            }
    }

    if (red_led_local) {
        RED_LED_ON;
    } else {
        RED_LED_OFF;
    }
    //return res;
}

static uint8_t checkChannel(uint8_t channel, uint8_t *hop_lst)
{
    uint8_t new_channel, count = 0, high = 0, i;
    for (i=0; i<16; i++) {
        if (high<hop_lst[i]) high = hop_lst[i];
        if (channel==hop_lst[i]) count++;
    }
    if (count>0) new_channel = high+2;
    else new_channel = channel;
    return new_channel%255;
}

static void bindChannels(const uint8_t* RF_HEAD, uint8_t* hop_lst)
{
    uint8_t n;

    memset(hop_lst, 0x00, 16);

    for (int j=0; j<4; j++) {
        for (int i=0; i<4; i++) {
            n = RF_HEAD[i]%128;
            if (j==3) n /= 5;
            else n /= j+1;
            hop_lst[4*j+i] = checkChannel(n,hop_lst);
        }
    }
}

void eleresInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);
    rxConfigMutable()->rx_spi_protocol = RFM22_ELERES;
    rxRuntimeConfig->channelCount = RC_CHANS;

    rfmSpiWrite(0x07, 0x80);
    delay(100);

    eleresSignaturePtr = (uint8_t*)&eleresConfigMutable()->eleresSignature;

    rfm22bInitParameter();
    bindChannels(eleresSignaturePtr,holList);
    channelHoppingTime = 33;
    toRxMode();
    channelHopping(1);
    rfMode = RECEIVE;
    localizerTime = millis() + (1000L * eleresConfig()->eleresLocDelay);

    return true;
}

uint8_t eleresBind(void)
{
    static uint8_t eleres_signature_old[4];
    static uint8_t eleres_signature_OK_count = 0;
    uint16_t timeout = 10000;
    uint8_t i;

    eleresSignaturePtr[0] = 0x42;
    eleresSignaturePtr[1] = 0x49;
    eleresSignaturePtr[2] = 0x4e;
    eleresSignaturePtr[3] = 0x44;

    rfm22bInitParameter();
    bindChannels(eleresSignaturePtr,holList);
    channelHoppingTime = 33;
    RED_LED_OFF;
    while (timeout--) {
        eleresDataReceived(NULL);
        eleresSetRcDataFromPayload(NULL,NULL);
        if (rfRxBuffer[0]==0x42) {
            for (i=0; i<4; i++) {
                if (rfRxBuffer[i+1]==eleres_signature_old[i]) eleres_signature_OK_count++;
                else eleres_signature_OK_count = 0;
            }
            for (i=0; i<4; i++) eleres_signature_old[i] = rfRxBuffer[i+1];
            if (eleres_signature_OK_count>200) {
                for (i=0; i<4; i++)
                    eleresSignaturePtr[i] = eleres_signature_old[i];
                RED_LED_OFF;
                saveConfigAndNotify();
                rfm22bInitParameter();
                return 0;
            }
            rfRxBuffer[0] = 0;
        }
        delay(1);
    }
    rfm22bInitParameter();
    return 1;
}

#endif //RX_ELERES

