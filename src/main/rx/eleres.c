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

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/gpio.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/compass/compass.h"
#include "drivers/timer.h"
#include "drivers/time.h"
#include "drivers/pwm_rx.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/io.h"
#include "drivers/rx_spi.h"

#include "fc/rc_controls.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "fc/cli.h"
#include "io/statusindicator.h"
#include "io/osd.h"

#include "rx/rx.h"
#include "rx/msp.h"
#include "rx/rx_spi.h"
#include "rx/eleres.h"

#include "telemetry/telemetry.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "navigation/navigation.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/barometer.h"

#include "io/beeper.h"
#include "drivers/bus_spi.h"
#include "fc/runtime_config.h"

#include "config/config_master.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "scheduler/scheduler.h"

#ifdef USE_RX_ELERES


PG_REGISTER_WITH_RESET_TEMPLATE(eleresConfig_t, eleresConfig, PG_ELERES_CONFIG, 0);

PG_RESET_TEMPLATE(eleresConfig_t, eleresConfig,
                  .eleres_freq = 435,
                  .eleres_telemetry_en = 0,
                  .eleres_telemetry_power = 7,
                  .eleres_loc_en = 0,
                  .eleres_loc_power = 7,
                  .eleres_loc_delay = 240
                 );

/*****************    eLeReS compatibile reciver on RFM22B module  ********************/
rx_spi_received_e Rfm_IRQ(void);
static uint8_t red_led=0;
#define RED_LED_ON {if (!red_led) {rfm_spi_write(0x0e, 0x04);red_led=1;}}
#define RED_LED_OFF {if (red_led) {rfm_spi_write(0x0e, 0x00);red_led=0;}}

#define RC_CHANS 12

static uint8_t hopping_channel = 1;

//int16_t eleresData[RC_CHANS] = { 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

void RF22B_init_parameter(void);
void to_rx_mode(void);
void ChannelHopping(uint8_t hops);
void telemetry_RX(void);
void rx_reset(void);
void to_tx_mode(uint8_t bytes_to_send);
void to_ready_mode(void);
void frequency_configurator(uint32_t frequency);

static timeMs_t last_pack_time, next_pack_time, localizer_time;
static uint8_t DataReady, ch_hopping_time, first_run = 1, loc_force=0;
static uint8_t hop_list[16];
uint8_t RSSI = 0, QUALITY;
static uint16_t good_frames;
static volatile uint8_t RF_Mode;

uint8_t BkgLoc_enable = 0;
uint8_t BkgLoc_chlist;
uint8_t BkgLoc_buf[3][10];
uint8_t BkgLoc_cnt;

#define Transmit	1
#define Transmitted	2
#define Receive 	4
#define Received 	8
#define Preamble	16

#define RF22B_Rx_packet_received_interrupt  0x02
#define RF22B_PACKET_SENT_INTERRUPT         0x04
#define RF22B_VALID_PREAMBLE_INTERRUPT      0x40
#define RF22B_VALID_SYNCWORD_INTERRUPT  		0x80
#define DATA_PACKAGE_SIZE 22
#define BIN_OFF_VALUE	1150
#define BIN_ON_VALUE	1850
uint8_t RF_Tx_Buffer[10];
uint8_t RF_Rx_Buffer[DATA_PACKAGE_SIZE];

uint8_t tx_full = 0;


uint8_t rfm_spi_read(uint8_t address)
{
    return rxSpiReadCommand(address & 0x7f, 0x00);
}

void rfm_spi_write(uint8_t address, uint8_t data)
{
    rxSpiWriteCommand(address | 0x80, data);
}

uint8_t eleres_rssi(void)
{
    return RSSI;
}


void rx_reset(void)
{
    rfm_spi_write(0x07, 1);
    rfm_spi_write(0x08, 0x03);
    rfm_spi_write(0x08, 0x00);
    rfm_spi_write(0x07, 5);
    rfm_spi_write(0x05, RF22B_Rx_packet_received_interrupt);
    rfm_spi_write(0x06, RF22B_VALID_SYNCWORD_INTERRUPT);
    rfm_spi_read(0x03);
    rfm_spi_read(0x04);
}


void to_rx_mode(void)
{
    to_ready_mode();
    rx_reset();
    RF_Mode = Receive;
}


void to_tx_mode(uint8_t bytes_to_send)
{
    uint8_t i;

    to_ready_mode();

    rfm_spi_write(0x08, 0x03);
    rfm_spi_write(0x08, 0x00);

    rfm_spi_write(0x3e, bytes_to_send);
    for (i = 0; i<bytes_to_send; i++)
        rfm_spi_write(0x7f, RF_Tx_Buffer[i]);

    rfm_spi_write(0x05, RF22B_PACKET_SENT_INTERRUPT);
    rfm_spi_write(0x06, 0);
    rfm_spi_read(0x03);
    rfm_spi_read(0x04);

    rfm_spi_write(0x07, 0x09);
    RF_Mode = Transmit;
    tx_full = 0;
}


void to_ready_mode(void)
{
    rfm_spi_write(0x07, 1);
    rfm_spi_write(0x05, 0);
    rfm_spi_write(0x06, 0);
    rfm_spi_read(0x03);
    rfm_spi_read(0x04);
    RF_Mode = 0;
}


//because we shared SPI bus I can't read data in real IRQ, so need to probe this pin from main idle
rx_spi_received_e eLeReSDataReceived(uint8_t *payload)
{
    static timeMs_t next_loop = 0;
    UNUSED(payload);
    if (rxSpiCheckIrq())
        return Rfm_IRQ();

    if (next_loop < millis())
    {
        next_loop = millis() + 20;
        eLeReSSetRcDataFromPayload(NULL,NULL);
    }

    return RX_SPI_RECEIVED_NONE;
}

void eLeReSSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
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

    if (cr_time < (led_time + 500))
        red_led_local = 0;
    else if (cr_time < (led_time + 1000))
        red_led_local = 1;
    else
        led_time = cr_time;

    if((DataReady & 2) == 0)
    {
        if(cr_time > next_pack_time+2)
        {
            if ((cr_time-last_pack_time > 1500) || first_run)
            {
                RSSI = 18;
                RF22B_init_parameter();
                to_rx_mode();
                ChannelHopping(15);
                next_pack_time += 17L*ch_hopping_time;
                if (eleresConfig()->eleres_telemetry_en)
                {
                    telemetry_RX();
                    to_tx_mode(9);
                }
               // res = RX_FRAME_FAILSAFE;
            }
            else
            {

                if(cr_time-last_pack_time > 3*ch_hopping_time)
                {
                    red_led_local=1;
                    if (RSSI > 0x18)
                        RSSI--;
                }
                to_rx_mode();
                ChannelHopping(1);
                next_pack_time += ch_hopping_time;
            }
        }
        if(cr_time > qtest_time)
        {
            qtest_time = cr_time + 500;
            QUALITY = good_frames * 100 / (500/ch_hopping_time);
            if(QUALITY > 100) QUALITY = 100;
            good_frames = 0;
        }
    }

    if((DataReady & 3) == 1 && rcData != NULL)
    {
        if((DataReady & 4)==0)
        {
            channel_count = RF_Rx_Buffer[20] >> 4;
            if(channel_count < 4)  channel_count = 4;
            if(channel_count > RC_CHANS) channel_count = 12;
            for(i = 0; i<channel_count; i++)
            {
                temp_int = RF_Rx_Buffer[i+1];
                if(i%2 == 0)
                    temp_int |= ((unsigned int)RF_Rx_Buffer[i/2 + 13] << 4) & 0x0F00;
                else
                    temp_int |= ((unsigned int)RF_Rx_Buffer[i/2 + 13] << 8) & 0x0F00;
                if ((temp_int>799) && (temp_int<2201)) rcData4Values[i] = temp_int;
            }
            n = RF_Rx_Buffer[19];
            for(i=channel_count; i < channel_count+5; i++)
            {
                if(i > 11) break;
                if(n & 0x01) temp_int = BIN_ON_VALUE;
                else temp_int = BIN_OFF_VALUE;
                rcData4Values[i] = temp_int;
                n >>= 1;
            }
            for(; i<RC_CHANS; i++) rcData4Values[i]=1500;
            for(i=0; i<RC_CHANS; i++)
            {
                temp_int = rcData4Values[i];
                if (temp_int < rcData[i] -3)  rcData[i] = temp_int+2;
                if (temp_int > rcData[i] +3)  rcData[i] = temp_int-2;
            }

            localizer_time = cr_time + (1000L*eleresConfig()->eleres_loc_delay);

            if (eleresConfig()->eleres_telemetry_en)
            {
                if (eleresConfig()->eleres_loc_en)
                {
                    if(BkgLoc_enable == 0) BkgLoc_cnt=0;
                    if(BkgLoc_cnt) BkgLoc_cnt--;

                    if(BkgLoc_cnt<128)
                        telemetry_RX();
                    else
                        memcpy(RF_Tx_Buffer, BkgLoc_buf[BkgLoc_cnt%3], 9);
                }
                else
                    telemetry_RX();
            }

            if (eleresConfig()->eleres_telemetry_en)
                telemetry_RX();
        }
        DataReady &= 0xFC;
        led_time = cr_time;
        //res = RX_FRAME_COMPLETE;
    }

    if (eleresConfig()->eleres_loc_en)
    {
        if((DataReady & 3)==3 && RF_Rx_Buffer[19]<128)
        {
            if(rx_frames == 0)	guard_time = last_pack_time;
            if(rx_frames < 250)
            {
                rx_frames++;
            }
            if(rx_frames > 20 && cr_time-guard_time > (loc_force?5000:20000))
            {
                DataReady = 0;
                localizer_time = cr_time + (1000L*eleresConfig()->eleres_loc_delay);
                RF22B_init_parameter();
                ChannelHopping(1);
                rx_frames = 0;

                if(loc_force && eleresConfig()->eleres_telemetry_en)
                {
                    BkgLoc_enable = 1;
                    temp_int = 0;
                    for(i=0; i<16; i++)
                    {
                        uint16_t mult = hop_list[i] * hop_list[(i+1)%16];
                        if(mult > temp_int)
                        {
                            temp_int = mult;
                            BkgLoc_chlist = i;
                        }
                    }
                }
            }
        }

        if(cr_time-last_pack_time > 8000)
        {
            rx_frames = 0;
        }
        if(!ARMING_FLAG(ARMED) && cr_time > localizer_time)
        {
            if((DataReady & 2)==0)
            {
                RF22B_init_parameter();
                rfm_spi_write(0x6d, eleresConfig()->eleres_loc_power);
            }
            DataReady &= 0xFB;
            DataReady |= 2;
            localizer_time = cr_time+35;
            rfm_spi_write(0x79, 0);
            red_led_local = 1;
            led_time = cr_time;

            BkgLoc_enable = 0;
            if(!(++loc_cnt & 1) && eleresConfig()->eleres_telemetry_en)
            {
                telemetry_RX();
                to_tx_mode(9);
            }
            else
            {
                RF_Tx_Buffer[0] = 0x48;
                RF_Tx_Buffer[1] = 0x45;
                RF_Tx_Buffer[2] = 0x4c;
                RF_Tx_Buffer[3] = 0x50;
                RF_Tx_Buffer[4] = 0x21;
                to_tx_mode(5);
            }
        }

        if((ARMING_FLAG(ARMED) || first_run) && (DataReady & 2)==0)
            localizer_time = cr_time + (1000L*eleresConfig()->eleres_loc_delay);

        if (eleresConfig()->eleres_telemetry_en)
            if(DataReady & 4)
            {
                if(RF_Rx_Buffer[0]=='H') BkgLoc_buf[0][0]='T';
                if(RF_Rx_Buffer[0]=='T')
                {
                    BkgLoc_buf[0][0]='T';
                    memcpy(BkgLoc_buf[0]+2, RF_Rx_Buffer+2, 7);
                }
                if(RF_Rx_Buffer[0]=='P') memcpy(BkgLoc_buf[1], RF_Rx_Buffer, 9);
                if(RF_Rx_Buffer[0]=='G') memcpy(BkgLoc_buf[2], RF_Rx_Buffer, 9);
                DataReady = 0;
            }
    }

    if (red_led_local)
    {
        RED_LED_ON;
    }
    else
    {
        RED_LED_OFF;
    }
    //return res;
}


uint8_t CheckChannel(uint8_t channel, uint8_t *hop_lst)
{
    uint8_t new_channel, count = 0, high = 0, i;
    for (i=0; i<16; i++)
    {
        if (high<hop_lst[i]) high = hop_lst[i];
        if (channel==hop_lst[i]) count++;
    }
    if (count>0) new_channel = high+2;
    else new_channel = channel;
    return new_channel%255;
}

void Bind_Channels(const uint8_t* RF_HEAD, uint8_t* hop_lst)
{
    uint8_t n,i,j;
    for (i=0; i<16; i++) hop_lst[i] = 0;
    for (j=0; j<4; j++)
    {
        for (i=0; i<4; i++)
        {
            n = RF_HEAD[i]%128;
            if(j==3) n /= 5;
            else n /= j+1;
            hop_lst[4*j+i] = CheckChannel(n,hop_lst);
        }
    }
}

rx_spi_received_e Rfm_IRQ(void)
{
    static uint16_t rssifil;
    timeMs_t irq_time = millis();
    uint8_t St1,St2;
    rx_spi_received_e res = RX_SPI_RECEIVED_NONE;

    St1 = rfm_spi_read(0x03);
    St2 = rfm_spi_read(0x04);
    rxSpiWriteByte(0x00);

    if((RF_Mode & Receive) && (St1 & RF22B_Rx_packet_received_interrupt))
        RF_Mode |= Received;
    if((RF_Mode & Transmit) && (St1 & RF22B_PACKET_SENT_INTERRUPT))
        RF_Mode |= Transmitted;
    if((RF_Mode & Receive) && (St2 & RF22B_VALID_SYNCWORD_INTERRUPT))
        RF_Mode |= Preamble;

    if(RF_Mode & Received)
    {
        if(BkgLoc_enable < 2)
        {
            last_pack_time = irq_time;
            next_pack_time = irq_time + ch_hopping_time;
        }
        rxSpiReadCommandMulti(0x7f,0x00,RF_Rx_Buffer,DATA_PACKAGE_SIZE);
        rx_reset();
        RF_Mode = Receive;
        if ((RF_Rx_Buffer[0] & 127) == 'S')
        {
            first_run = 0;
            good_frames++;

            if ((RF_Rx_Buffer[21] & 0xF0) == 0x20)
                hopping_channel = RF_Rx_Buffer[21] & 0x0F;

            ch_hopping_time = (RF_Rx_Buffer[20] & 0x0F)+18;
            DataReady |= 1;
        }
        else if (eleresConfig()->eleres_loc_en && eleresConfig()->eleres_telemetry_en && BkgLoc_enable==2)
        {
            if((RF_Rx_Buffer[0] == 'H' && RF_Rx_Buffer[2] == 'L') ||
                    RF_Rx_Buffer[0]=='T' || RF_Rx_Buffer[0]=='P' || RF_Rx_Buffer[0]=='G')
            {
                if(BkgLoc_cnt==0) BkgLoc_cnt = 200;
                to_ready_mode();
                BkgLoc_enable = 0;
                ChannelHopping(0);
                BkgLoc_enable = 2;
                DataReady |= 4;
            }
        }
        else if (eleresConfig()->eleres_loc_en)
        {
            if (RF_Rx_Buffer[0] == 0x4c && RF_Rx_Buffer[1] == 0x4f && RF_Rx_Buffer[2] == 0x43)
            {
                localizer_time = irq_time;
                loc_force = 1;
                RSSI = 0x18;
            }
        }

        if((DataReady & 2)==0)
        {
            if (eleresConfig()->eleres_telemetry_en)
                to_tx_mode(9);
            else
                ChannelHopping(1);
        }
        res = RX_SPI_RECEIVED_DATA;
    }

    if(RF_Mode & Transmitted)
    {
        to_ready_mode();
        if(DataReady & 2)
        {
            rfm_spi_write(0x79, hop_list[0]);
        }
        else if(irq_time-last_pack_time <= 1500 && BkgLoc_enable<2)
            ChannelHopping(1);
        to_rx_mode();
    }

    if(RF_Mode & Preamble)
    {
        uint8_t rssitmp = rfm_spi_read(0x26);
        if (eleresConfig()->eleres_loc_en && eleresConfig()->eleres_telemetry_en && BkgLoc_enable==2)
        {
            if(rssitmp>124)	rssitmp = 124;
            if(rssitmp<18)	rssitmp = 18;
            BkgLoc_buf[0][1] = rssitmp + 128;
        }
        else
        {
            rssifil -= rssifil/8;
            rssifil += rssitmp;
            RSSI = (rssifil/8 * QUALITY / 100)+10;
            if(RSSI>124) RSSI = 124;
            if(RSSI<18)	RSSI = 18;
        }
        RF_Mode &= ~Preamble;
    }

    return res;
}


void RF22B_init_parameter(void)
{
    int8_t i;
    static uint8_t first_init = 1;

    static uint8_t cf1[9] = {0x00,0x01,0x00,0x7f,0x07,0x52,0x55,0xCA};
    static uint8_t cf2[6] = {0x68,0x01,0x3a,0x93,0x02,0x6b};
    static uint8_t cf3[8] = {0x0f,0x42,0x07,0x20,0x2d,0xd4,0x00,0x00};
    rfm_spi_read(0x03);
    rfm_spi_read(0x04);
    for(i = 0; i < 8; i++)
        rfm_spi_write(0x06+i, cf1[i]);
    if (first_init)
    {
        first_init = 0;
        rfm_spi_write(0x0e, 0);
    }

    rfm_spi_write(0x6e, 0x09);
    rfm_spi_write(0x6f, 0xD5);
    rfm_spi_write(0x1c, 0x02);
    rfm_spi_write(0x70, 0x00);
    for(i=0; i<6; i++) rfm_spi_write(0x20+i, cf2[i]);
    rfm_spi_write(0x2a, 0x1e);
    rfm_spi_write(0x72, 0x1F);
    rfm_spi_write(0x30, 0x8c);
    rfm_spi_write(0x3e, 22);
    for(i=0; i<8; i++) rfm_spi_write(0x32+i, cf3[i]);
    for(i=0; i<4; i++) rfm_spi_write(0x43+i, 0xff);
    rfm_spi_write(0x6d, eleresConfig()->eleres_telemetry_power | 0x18);
    rfm_spi_write(0x79, 0x00);
    rfm_spi_write(0x7a, 0x04);
    rfm_spi_write(0x71, 0x23);
    rfm_spi_write(0x73, 0x00);
    rfm_spi_write(0x74, 0x00);
    for(i=0; i<4; i++)
    {
        rfm_spi_write(0x3a+i, eleresConfig()->eleres_signature[i]);
        rfm_spi_write(0x3f+i, eleresConfig()->eleres_signature[i]);
    }

    frequency_configurator((uint32_t)(eleresConfig()->eleres_freq * 100));
    rfm_spi_read(0x03);
    rfm_spi_read(0x04);
}

void frequency_configurator(uint32_t frequency)
{
    uint8_t band;

    if(frequency<48000)
    {
        frequency -= 24000;
        band = frequency/1000;
        if(band>23) band = 23;
        frequency -= (1000*(uint32_t)band);
        frequency *= 64;
        band |= 0x40;
    }
    else
    {
        frequency -= 48000;
        band = frequency/2000;
        if(band>22) band = 22;
        frequency -= (2000*(uint32_t)band);
        frequency *= 32;
        band |= 0x60;
    }
    rfm_spi_write(0x75, band);
    rfm_spi_write(0x76, (uint16_t)frequency >> 8);
    rfm_spi_write(0x77, (uint8_t)frequency);
    rfm_spi_write(0x79, 0);
}


void ChannelHopping(uint8_t hops)
{
    hopping_channel += hops;
    while(hopping_channel >= 16) hopping_channel -= 16;

    if (eleresConfig()->eleres_telemetry_en && eleresConfig()->eleres_loc_en)
    {
        if(BkgLoc_enable && (hopping_channel==BkgLoc_chlist || hopping_channel==(BkgLoc_chlist+1)%16))
        {
            rfm_spi_write(0x79,0);
            BkgLoc_enable = 2;
            return;
        }
        if(BkgLoc_enable == 2) BkgLoc_enable = 1;
    }

    rfm_spi_write(0x79, hop_list[hopping_channel]);
}


void telemetry_RX(void)
{
    static uint8_t telem_state;
    static int32_t presfil;
    static int16_t thempfil;
    uint8_t i, themp=90, wii_flymode=0;
    uint16_t pres,curr = abs(amperage) / 10;
    union
    {
        int32_t val;
        uint8_t b[4];
    } cnv;

    if (tx_full)
        return;

    memset(RF_Tx_Buffer,0,9);

    presfil  -= presfil/4;
    presfil  += baro.baroPressure;
    thempfil -= thempfil/8;
    thempfil += baro.baroTemperature/10;

    switch (telem_state++)
    {
    case 0:

        if(presfil>200000) pres = presfil/4 - 50000;
        else pres = 1;

        themp = (uint8_t)(thempfil/80 + 86);

        if (FLIGHT_MODE(FAILSAFE_MODE))    wii_flymode = 7;
        else if(FLIGHT_MODE(PASSTHRU_MODE))  wii_flymode = 8;
        else if(FLIGHT_MODE(NAV_RTH_MODE))  wii_flymode = 6;
        else if(FLIGHT_MODE(NAV_POSHOLD_MODE))  wii_flymode = 5;
        else if(FLIGHT_MODE(HEADFREE_MODE))  wii_flymode = 4;
        else if(FLIGHT_MODE(NAV_ALTHOLD_MODE))      wii_flymode = 3;
        else if(FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))       wii_flymode = 2;
        else                      wii_flymode = 1;
        if(ARMING_FLAG(ARMED)) wii_flymode |= 0x10;
        RF_Tx_Buffer[0] = 0x54;
        RF_Tx_Buffer[1] = RSSI;
        RF_Tx_Buffer[2] = QUALITY;
        RF_Tx_Buffer[3] = vbat;
        RF_Tx_Buffer[4] = themp;
        RF_Tx_Buffer[5] = curr & 0xff;
        RF_Tx_Buffer[6] = pres>>8;
        RF_Tx_Buffer[7] = pres&0xFF;
        RF_Tx_Buffer[8] = wii_flymode;
        RF_Tx_Buffer[8]	|= ((curr>>2) & 0xC0);
        break;

    case 1:
        if (gpsSol.llh.lat)
        {
            uint16_t hdop = gpsSol.hdop / 10;

            RF_Tx_Buffer[0] = 0x50;
            cnv.val = gpsSol.llh.lat/10;
            for(i=0; i<4; i++) RF_Tx_Buffer[i+1] = cnv.b[i];
            cnv.val = gpsSol.llh.lon/10;
            for(i=0; i<4; i++) RF_Tx_Buffer[i+5] = cnv.b[i];

            RF_Tx_Buffer[4] &= 0x0F;
            RF_Tx_Buffer[4] |= (hdop << 4);
            RF_Tx_Buffer[8] &= 0x1F;
            RF_Tx_Buffer[8] |= ((hdop<<1) & 0xE0);
            break;
        }
        telem_state++;
    case 2:
        if (sensors(SENSOR_GPS))
        {
            uint16_t gpsspeed =  (gpsSol.groundSpeed*9L)/250L;
            int16_t course = (gpsSol.groundCourse+360)%360;
#ifdef NAV
            int32_t alt = getEstimatedActualPosition(Z);
#else
            int32_t alt = baro.BaroAlt;
#endif
            uint16_t tim = 0;

            RF_Tx_Buffer[0] = 0x47;
            RF_Tx_Buffer[1] = (STATE(GPS_FIX)<<4) | (gpsSol.numSat & 0x0F);
            if(gpsSol.numSat > 15) RF_Tx_Buffer[1] |= 0x80;
            RF_Tx_Buffer[2] = ((course>>8) & 0x0F) | ((gpsspeed>>4) & 0xF0);
            RF_Tx_Buffer[3] = course & 0xFF;
            RF_Tx_Buffer[4] = gpsspeed & 0xFF;

            RF_Tx_Buffer[5] = (alt/100) >>8;
            RF_Tx_Buffer[6] = (alt/100) & 0xFF;

            RF_Tx_Buffer[7] = tim>>8;
            RF_Tx_Buffer[8] = tim&0xFF;
            break;
        }
        telem_state++;
    default:
        RF_Tx_Buffer[0] = 'D';
        memcpy(RF_Tx_Buffer+1,&debug[0],2);
        memcpy(RF_Tx_Buffer+3,&debug[1],2);
        memcpy(RF_Tx_Buffer+5,&debug[2],2);
        memcpy(RF_Tx_Buffer+7,&debug[3],2);
        telem_state = 0;
        break;
    }
    tx_full = 1;
}

void eleresInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = RC_CHANS;

    rfm_spi_write(0x07, 0x80);
    delay(100);

    RF22B_init_parameter();
    Bind_Channels(eleresConfig()->eleres_signature,hop_list);
    ch_hopping_time = 33;
    to_rx_mode();
    ChannelHopping(1);
    RF_Mode = Receive;
    localizer_time = millis() + (1000L * eleresConfig()->eleres_loc_delay);

    return true;
}

uint8_t eLeReS_Bind(void)
{
    static uint8_t eleres_signature_old[4];
    static uint8_t eleres_signature_OK_count = 0;
    uint16_t timeout = 10000;
    uint8_t i;

    eleresConfigMutable()->eleres_signature[0] = 0x42;
    eleresConfigMutable()->eleres_signature[1] = 0x49;
    eleresConfigMutable()->eleres_signature[2] = 0x4e;
    eleresConfigMutable()->eleres_signature[3] = 0x44;

    RF22B_init_parameter();
    Bind_Channels(eleresConfig()->eleres_signature,hop_list);
    ch_hopping_time = 33;
    RED_LED_OFF;
    while(timeout--)
    {
        eLeReSDataReceived(NULL);
        eLeReSSetRcDataFromPayload(NULL,NULL);
        if (RF_Rx_Buffer[0]==0x42)
        {
            for(i=0; i<4; i++)
            {
                if (RF_Rx_Buffer[i+1]==eleres_signature_old[i]) eleres_signature_OK_count++;
                else eleres_signature_OK_count = 0;
            }
            for(i=0; i<4; i++) eleres_signature_old[i] = RF_Rx_Buffer[i+1];
            if (eleres_signature_OK_count>200)
            {
                for(i=0; i<4; i++)
                    eleresConfigMutable()->eleres_signature[i] = eleres_signature_old[i];
                RED_LED_OFF;
                saveConfigAndNotify();
                RF22B_init_parameter();
                return 0;
            }
            RF_Rx_Buffer[0] = 0;
        }
        delay(1);
    }
    RF22B_init_parameter();
    return 1;
}

#endif //RX_ELERES

