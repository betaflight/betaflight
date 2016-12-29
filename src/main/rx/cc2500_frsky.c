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

// This file is copied with modifications from OpenSky,
// see https://github.com/fishpepper/OpenSky

// See also in particular
// https://github.com/fishpepper/OpenSky/blob/master/frsky.c

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RX_FRSKY

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/rx_cc2500.h"
#include "drivers/system.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/cc2500_frsky.h"

#define DEFAULT_FSCAL_VALUE (-69)

#define FRSKY_CHANNEL_COUNT 8
#define FRSKY_HOPTABLE_SIZE 47

#define FRSKY_COUNT_RXSTATS 50
#define FRSKY_PACKET_LENGTH 17
#define FRSKY_PACKET_BUFFER_SIZE (FRSKY_PACKET_LENGTH+3)

//packet data example:
//BIND:   [11 03 01 16 68 14 7E BF 15 56 97 00 00 00 00 00 00 0B F8 AF ]
//NORMAL: [11 16 68 ... ]
//TX:                 11 16 68 7A 1B 0B CA CB CF C4 88 85 CB CB CB 92 8B 78 21 AF
//TELEMETRY WITH HUB: 11 16 68 60 64 5B 00 00 5E 3B 09 00 5E 5E 3B 09 00 5E 48 B1
#define FRSKY_VALID_FRAMELENGTH(_b) (_b[0] == 0x11)
#define FRSKY_VALID_CRC(_b)     (_b[19] & 0x80)
#define FRSKY_VALID_TXID(_b) ((_b[1] == storage.frsky_txid[0]) && (_b[2] == storage.frsky_txid[1]))
#define FRSKY_VALID_PACKET_BIND(_b) (FRSKY_VALID_FRAMELENGTH(_b) && FRSKY_VALID_CRC(_b) && (_b[2] == 0x01))
#define FRSKY_VALID_PACKET(_b)      (FRSKY_VALID_FRAMELENGTH(_b) && FRSKY_VALID_CRC(_b) && FRSKY_VALID_TXID(_b) )


#define debug(x) {}
#define debug_flush() {}
#define debug_put_hex8(x) {}
#define debug_put_int8(x) {}
#define debug_put_uint8(x) {}
#define debug_putc(x) {}
#define debug_put_newline() {}

static void frsky_do_bind(void);
static void frsky_configure_address(void);
static void frsky_autotune(void);
static void frsky_fetch_txid_and_hoptable(void);
static void frsky_tune_channel(uint8_t ch);
static void frsky_show_partinfo(void);
static void frsky_enter_rxmode(uint8_t channel);
static void frsky_handle_overflows(void);
static void frsky_increment_channel(int8_t cnt);


typedef enum {
    STATE_BIND = 0,
    STATE_DATA
} protocol_state_t;

static protocol_state_t protocolState;

static const uint8_t storage_default_hoptable[FRSKY_HOPTABLE_SIZE] = {
    0x01, 0x42, 0x83, 0xC4, 0x1A, 0x5B, 0x9C, 0xDD, 0x33, 0x74, 0xB5, 0x0B,
    0x4C, 0x8D, 0xCE, 0x24, 0x65, 0xA6, 0xE7, 0x3D, 0x7E, 0xBF, 0x15, 0x56,
    0x97, 0xD8, 0x2E, 0x6F, 0xB0, 0x06, 0x47, 0x88, 0xC9, 0x1F, 0x60, 0xA1,
    0xE2, 0x38, 0x79, 0xBA, 0x10, 0x51, 0x92, 0xD3, 0x29, 0x6A, 0xAB
};

static uint32_t timeOfLastHopUs;
static const uint32_t hopTimeoutUs = 5000; // 5ms

#define STORAGE_VERSION_ID 0x01
typedef struct storage_s {
    //version id
    uint8_t version;
    //persistent storage for frsky
    uint8_t frsky_txid[2];
    uint8_t frsky_hop_table[FRSKY_HOPTABLE_SIZE];
    int8_t  frsky_freq_offset;
    //add further data here...
} storage_t;

static storage_t storage;

static uint8_t frsky_current_ch_idx;

//diversity counter
static uint8_t frsky_diversity_count;

//rssi
static uint8_t frsky_rssi;
static uint8_t frsky_link_quality;

//pll calibration
static uint8_t frsky_calib_fscal1_table[FRSKY_HOPTABLE_SIZE];
static uint8_t frsky_calib_fscal2;
static uint8_t frsky_calib_fscal3;
//static int16_t storage.frsky_freq_offset_acc;

//rf rxtx buffer
static volatile uint8_t frsky_packet_buffer[FRSKY_PACKET_BUFFER_SIZE];
static volatile uint8_t frsky_packet_received;
static volatile uint8_t frsky_packet_sent;

static void storageInit(void)
{
    storage.version = STORAGE_VERSION_ID;

    //hard coded config for debugging:
    storage.frsky_txid[0] = 0x16;
    storage.frsky_txid[1] = 0x68;
    storage.frsky_freq_offset = DEFAULT_FSCAL_VALUE;

    for (int i = 0; i < FRSKY_HOPTABLE_SIZE; i++) {
        storage.frsky_hop_table[i] = storage_default_hoptable[i];
    }
}

static bool frskyCheckBindPacket(const uint8_t *payload)
{
#define BIND_PAYLOAD0       0x11
#define BIND_PAYLOAD2       0x01
    bool bindPacket = false;
    if (FRSKY_VALID_PACKET_BIND(payload)) {
        bindPacket = true;
        //!!TODO process bind packet to set txid and hopping channels
    }
    return bindPacket;
}

void frskySetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    // see frsky_update_ppm() in https://github.com/fishpepper/OpenSky/blob/master/frsky.c
    rcData[0] = (uint16_t)(((payload[10] & 0x0F)<<8 | payload[6]));
    rcData[1] = (uint16_t)(((payload[10] & 0xF0)<<4 | payload[7]));
    rcData[2] = (uint16_t)(((payload[11] & 0x0F)<<8 | payload[8]));
    rcData[3] = (uint16_t)(((payload[11] & 0xF0)<<4 | payload[9]));
    rcData[4] = (uint16_t)(((payload[16] & 0x0F)<<8 | payload[12]));
    rcData[5] = (uint16_t)(((payload[16] & 0xF0)<<4 | payload[13]));
    rcData[6] = (uint16_t)(((payload[17] & 0x0F)<<8 | payload[14]));
    rcData[7] = (uint16_t)(((payload[17] & 0xF0)<<4 | payload[15]));
}

static void frskyHopToNextChannel(void)
{
    frsky_increment_channel(1);
}

static void frskySetHoppingChannels(void)
{

}

static void frskySetBound(void)
{
    protocolState = STATE_DATA;

    timeOfLastHopUs = micros();
    frskySetHoppingChannels();
}

/*
 * This is called periodically by the scheduler.
 * Returns RX_SPI_RECEIVED_DATA if a data packet was received.
 */
rx_spi_received_e frskyDataReceived(uint8_t *payload)
{
    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;
    uint32_t timeNowUs;
    uint8_t packetReceived;
    switch (protocolState) {
    case STATE_BIND:
        cc25xx_process_packet(&packetReceived, payload, FRSKY_PACKET_BUFFER_SIZE);
        if (packetReceived) {
            const bool bindPacket = frskyCheckBindPacket(payload);
            if (bindPacket) {
                ret = RX_SPI_RECEIVED_BIND;
                //writeBindAckPayload(payload);
                // got a bind packet, so set the hopping channels and the rxTxAddr and start listening for data
                frskySetBound();
            }
        }
        break;
    case STATE_DATA:
        timeNowUs = micros();
        // read the payload, processing of payload is deferred
        cc25xx_process_packet(&packetReceived, payload, FRSKY_PACKET_BUFFER_SIZE);
        if (packetReceived) {
            const bool bindPacket = frskyCheckBindPacket(payload);
            if (bindPacket) {
                // transmitter may still continue to transmit bind packets after we have switched to data mode
                ret = RX_SPI_RECEIVED_BIND;
                //writeBindAckPayload(payload);
            } else {
                ret = RX_SPI_RECEIVED_DATA;
                //writeTelemetryAckPayload();
            }
        }
        if ((ret == RX_SPI_RECEIVED_DATA) || (timeNowUs > timeOfLastHopUs + hopTimeoutUs)) {
            frskyHopToNextChannel();
            timeOfLastHopUs = timeNowUs;
        }
        break;
    }
    return ret;
}

static void frsky_show_partinfo(void)
{
    uint8_t partnum, version;
    //start idle
    cc25xx_strobe(RFST_SIDLE);

    //check version:
    debug("frsky: cc25xx partnum 0x");
    partnum = cc25xx_get_register_burst(PARTNUM);
    debug_put_hex8(partnum);

    debug(" version 0x");
    version = cc25xx_get_register_burst(VERSION);
    debug_put_hex8(version);
    debug_put_newline();

    if (cc25xx_partnum_valid(partnum, version)){
        debug("frsky: got valid part and version info\n");
    }else{
        debug("frsky: got INVALID part and version info?!\n");
    }
    debug_flush();
}


static void frsky_configure(void)
{
    debug("frsky: configure\n"); debug_flush();

    //start idle
    cc25xx_strobe(RFST_SIDLE);
    
    // IOCFG0,1,2 is set in hal code (it is specific to the board used)
    cc25xx_set_gdo_mode();

    //normal config
    cc25xx_set_register(MCSM1    ,0x0F); //go back to rx after transmission completed //0x0C = stay idle;
    cc25xx_set_register(MCSM0    ,0x18);
    cc25xx_set_register(PKTLEN   ,FRSKY_PACKET_LENGTH); //on 251x this has to be exactly our size
    cc25xx_set_register(PKTCTRL0 ,0x05);
    cc25xx_set_register(PA_TABLE0,0xFF);
    cc25xx_set_register(FSCTRL1  ,0x08); //D4R-II seems to set 0x68 here ?! instead of 0x08
    cc25xx_set_register(FSCTRL0  ,0x00);
    //set base freq 2404 mhz
    cc25xx_set_register(FREQ2    ,0x5C);
    cc25xx_set_register(FREQ1    ,0x76);
    cc25xx_set_register(FREQ0    ,0x27);
    cc25xx_set_register(MDMCFG4  ,0xAA);
    cc25xx_set_register(MDMCFG3  ,0x39);
    cc25xx_set_register(MDMCFG2  ,0x11);
    cc25xx_set_register(MDMCFG1  ,0x23);
    cc25xx_set_register(MDMCFG0  ,0x7A);
    cc25xx_set_register(DEVIATN  ,0x42);
    cc25xx_set_register(FOCCFG   ,0x16);
    cc25xx_set_register(BSCFG    ,0x6C);
    cc25xx_set_register(AGCCTRL2 ,0x03);
    cc25xx_set_register(AGCCTRL1 ,0x40); //D4R uses 46 instead of 0x40);
    cc25xx_set_register(AGCCTRL0 ,0x91);
    cc25xx_set_register(FREND1   ,0x56);
    cc25xx_set_register(FREND0   ,0x10);
    cc25xx_set_register(FSCAL3   ,0xA9);
    cc25xx_set_register(FSCAL2   ,0x05);
    cc25xx_set_register(FSCAL1   ,0x00);
    cc25xx_set_register(FSCAL0   ,0x11);
    //???FSTEST   , 0x59);
    cc25xx_set_register(TEST2    ,0x88);
    cc25xx_set_register(TEST1    ,0x31);
    cc25xx_set_register(TEST0    ,0x0B);
    //???FIFOTHR  = 0x07);
    cc25xx_set_register(ADDR     ,0x00);

    //for now just append status
    cc25xx_set_register(PKTCTRL1, CC2500_PKTCTRL1_APPEND_STATUS);
    debug("frsky: configure done\n"); debug_flush();
}


void wdt_reset(void)
{
    IWDG_ReloadCounter();
}

static uint32_t timeoutMs;
static void timeout_set(uint32_t ms)
{
    timeoutMs = millis() + ms;
}

static bool timeout_timed_out(void)
{
    return cmp32(millis(), timeoutMs) > 0 ? true : false;
}


static uint8_t frsky_bind_jumper_set(void)
{
#if 0
    debug("frsky: BIND jumper set = "); debug_flush();
    if (io_bind_request()){
        debug("YES -> binding\n");
        return 1;
    }else{
        debug("NO -> no binding\n");
        return 0;
    }
#endif
    return 0;
}

static void led_red_off(void) {}
static void led_red_on(void) {}
static void led_green_on(void) {}

static void frskyBind(void)
{
    //set txid to bind channel
    storage.frsky_txid[0] = 0x03;

    //frequency offset to zero (will do auto tune later on)
    storage.frsky_freq_offset = 0;

    //init txid matching
    frsky_configure_address();

    //start autotune:
    frsky_autotune();

    //now run the actual binding:
    frsky_fetch_txid_and_hoptable();
}

static void frsky_do_bind(void)
{
    debug("frsky: do bind\n"); debug_flush();

    //set up leds:frsky_txid
    led_red_off();
    led_green_on();

    frskyBind();

    //important: stop RF interrupts:
    cc25xx_disable_rf_interrupt();

    //save to persistant storage:
    //storage_write_to_flash();

    //done, end up in fancy blink code
    debug("frsky: finished binding. please reset\n");
    led_green_on();

    while(1){
        led_red_on();
        delay(500);
        wdt_reset();

        led_red_off();
        delay(500);
        wdt_reset();
    }
}


static void frsky_autotune(void)
{
    uint8_t done = 0;
    uint8_t received_packet = 0;
    uint8_t state = 0;
    int8_t fscal0_min=127;
    int8_t fscal0_max=-127;
    int16_t fscal0_calc;

    debug("frsky: autotune\n"); debug_flush();

    //enter RX mode
    frsky_enter_rxmode(0);

    //find best offset:
    storage.frsky_freq_offset = 0;

    debug("frsky: entering bind loop\n"); debug_flush();

    led_red_off();

    //search for best fscal 0 match
    while(state != 5){
        //reset wdt
        wdt_reset();

        //handle any ovf conditions
        frsky_handle_overflows();

        //search full range quickly using binary search
        switch(state){
        default:
        case(0):
            //init left search:
            storage.frsky_freq_offset = -127;
            state = 1;
            break;

        case(1):
            //first search quickly through the full range:
            if (storage.frsky_freq_offset < 127-10){
                storage.frsky_freq_offset += 9;
            }else{
                //done one search, did we receive anything?
                if (received_packet){
                    //finished, go to slow search
                    storage.frsky_freq_offset = fscal0_min - 9;
                    state = 2;
                }else{
                    //no success, lets try again
                    state = 0;
                }
            }
            break;

        case(2):
            if (storage.frsky_freq_offset < fscal0_max+9){
                storage.frsky_freq_offset++;
            }else{
                //done!
                state = 5;
            }
            break;
        }

        //go to idle
        cc25xx_strobe(RFST_SIDLE);

        //set freq offset
        cc25xx_set_register(FSCTRL0, storage.frsky_freq_offset);

        led_red_off();

        //go back to RX:
        delay(1);
        cc25xx_strobe(RFST_SRX);

        //set timeout
        timeout_set(50);
        done = 0;

        led_green_on();
        led_red_off();

        //debug("tune "); debug_put_int8(storage.frsky_freq_offset); debug_put_newline(); debug_flush();

        while((!timeout_timed_out()) && (!done)){
            //handle any ovf conditions
            frsky_handle_overflows();

            cc25xx_process_packet(&frsky_packet_received, (volatile uint8_t *)&frsky_packet_buffer, FRSKY_PACKET_BUFFER_SIZE);

            if (frsky_packet_received){
                //prepare for next packet:
                frsky_packet_received = 0;
                cc25xx_enable_receive();
                cc25xx_strobe(RFST_SRX);

                //valid packet?
                if (FRSKY_VALID_PACKET_BIND(frsky_packet_buffer)){
                    //bind packet!
                    debug_putc('B');

                    //packet received
                    received_packet = 1;

                    //this fscal value is done
                    done = 1;

                    //update min/max
                    fscal0_min = MIN(fscal0_min, storage.frsky_freq_offset);
                    fscal0_max = MAX(fscal0_max, storage.frsky_freq_offset);

                    //make sure we never read the same packet twice by invalidating packet
                    frsky_packet_buffer[0] = 0x00;
                }

                /*debug("[");debug_flush();
        uint8_t cnt;
                for(cnt=0; cnt<FRSKY_PACKET_BUFFER_SIZE; cnt++){
                    debug_put_hex8(frsky_packet_buffer[cnt]);
                    debug_putc(' ');
                    debug_flush();
                }
                debug("]\n"); debug_flush();*/

            }
        }
        if (!done){
            debug_putc('-');
        }
    }

    //set offset to what we found out to be the best:
    fscal0_calc = (fscal0_max + fscal0_min)/2;

    debug("\nfrsky: fscal0 ");
    debug_put_int8(fscal0_min);
    debug(" - ");
    debug_put_int8(fscal0_max);
    debug_put_newline();
    debug_flush();

    //store new value
    storage.frsky_freq_offset = fscal0_calc;

    cc25xx_strobe(RFST_SIDLE);

    //set freq offset
    cc25xx_set_register(FSCTRL0, storage.frsky_freq_offset);

    //go back to RX:
    delay(1);
    cc25xx_strobe(RFST_SRX);

    debug("frsky: autotune done. offset=");
    debug_put_int8(storage.frsky_freq_offset);
    debug_put_newline();
    debug_flush();
}

static void frsky_enter_rxmode(uint8_t channel)
{
    cc25xx_strobe(RFST_SIDLE);

    cc25xx_enter_rxmode();

    //set & do a manual tuning for the given channel
    frsky_tune_channel(channel);

    cc25xx_enable_receive();

    //go back to rx mode
    cc25xx_strobe(RFST_SRX);
}

static void frsky_configure_address(void)
{
    // start idle
    cc25xx_strobe(RFST_SIDLE);

    //freq offset
    cc25xx_set_register(FSCTRL0, storage.frsky_freq_offset);

    //never automatically calibrate, po_timeout count = 64
    //no autotune as (we use our pll map)
    cc25xx_set_register(MCSM0,0x08);

    //set address
    cc25xx_set_register(ADDR, storage.frsky_txid[0]);

    //append status, filter by address, autoflush on bad crc, PQT=0
    cc25xx_set_register(PKTCTRL1, CC2500_PKTCTRL1_APPEND_STATUS | CC2500_PKTCTRL1_CRC_AUTOFLUSH | CC2500_PKTCTRL1_FLAG_ADR_CHECK_01);
}

static void frsky_tune_channel(uint8_t ch)
{
    //start idle
    cc25xx_strobe(RFST_SIDLE);

    //set channel number
    cc25xx_set_register(CHANNR, ch);

    //start Self calib:
    cc25xx_strobe(RFST_SCAL);

    //wait for scal end
    //either delay_us(800) or check MARCSTATE:
    while(cc25xx_get_register(MARCSTATE) != 0x01);

    //now FSCAL3..1 shold be set up correctly! yay!
}

static void frsky_handle_overflows(void)
{
    uint8_t marc_state;

    //fetch marc status
    marc_state = cc25xx_get_register(MARCSTATE) & 0x1F;
    if (marc_state == 0x11){
        debug("frsky: RXOVF\n");
        //flush rx buf
        cc25xx_strobe(RFST_SFRX);
        //cc25xx_strobe(RFST_SIDLE);
    }else if (marc_state == 0x16){
        debug("frsky: TXOVF\n");
        //flush tx buf
        cc25xx_strobe(RFST_SFTX);
        //cc25xx_strobe(RFST_SIDLE);
    }
}

static void frsky_fetch_txid_and_hoptable(void)
{
    uint16_t hopdata_received = 0;
    uint8_t index;
    uint8_t i;

    //enter RX mode
    frsky_enter_rxmode(0);

#define MAX_BIND_PACKET_COUNT 10
    //DONE when n times a one:
#define HOPDATA_RECEIVE_DONE ((1<<(MAX_BIND_PACKET_COUNT))-1)

    //clear txid:
    storage.frsky_txid[0] = 0;
    storage.frsky_txid[1] = 0;

    //timeout to wait for packets
    timeout_set(9*3+1);

    //fetch hopdata array
    while(hopdata_received != HOPDATA_RECEIVE_DONE){
        //reset wdt
        wdt_reset();

        //handle any ovf conditions
        frsky_handle_overflows();

        //FIXME: this should be handled in a cleaner way.
        //as this is just for binding, stay with this fix for now...
        if (timeout_timed_out()){
            //do diversity
            cc25xx_switch_antenna();

            debug_putc('m');

            //next packet should be there ein 9ms
            //if no packet for 3*9ms -> reset rx chain:
            timeout_set(3*9+1);

            //re-prepare for next packet:
            cc25xx_strobe(RFST_SIDLE);
            //TESTME: moved to rx_sleep....
            //delay(1);
            frsky_packet_received = 0;
            cc25xx_rx_sleep();
            cc25xx_enable_receive();
            cc25xx_strobe(RFST_SRX);
        }

        //process incoming data
        cc25xx_process_packet(&frsky_packet_received, (volatile uint8_t *)&frsky_packet_buffer, FRSKY_PACKET_BUFFER_SIZE);

        if (frsky_packet_received){
            debug_putc('p');

            //prepare for next packet:
            frsky_packet_received = 0;
            cc25xx_enable_receive();
            cc25xx_strobe(RFST_SRX);


#if FRSKY_DEBUG_BIND_DATA
            if (FRSKY_VALID_FRAMELENGTH(frsky_packet_buffer)){
                debug("frsky: RX ");
                debug_flush();
                for(i=0; i<FRSKY_PACKET_BUFFER_SIZE; i++){
                    debug_put_hex8(frsky_packet_buffer[i]);
                    debug_putc(' ');
                }
                debug_put_newline();
            }
#endif


            //do we know our txid yet?
            if (FRSKY_VALID_PACKET_BIND(frsky_packet_buffer)){
                //next packet should be ther ein 9ms
                //if no packet for 3*9ms -> reset rx chain:
                timeout_set(3*9+1);

                debug_putc('B');
                if ((storage.frsky_txid[0] == 0) && (storage.frsky_txid[1] == 0)){
                    //no! extract this
                    storage.frsky_txid[0] = frsky_packet_buffer[3];
                    storage.frsky_txid[1] = frsky_packet_buffer[4];
                    //debug
                    debug("frsky: got txid 0x");
                    debug_put_hex8(storage.frsky_txid[0]);
                    debug_put_hex8(storage.frsky_txid[1]);
                    debug_put_newline();
                }

                //this is actually for us
                index = frsky_packet_buffer[5];

                //valid bind index?
                if (index/5 < MAX_BIND_PACKET_COUNT){
                    //copy data to our hop list:
                    for(i=0; i<5; i++){
                        if ((index+i) < FRSKY_HOPTABLE_SIZE){
                            storage.frsky_hop_table[index+i] = frsky_packet_buffer[6+i];
                        }
                    }
                    //mark as done: set bit flag for index
                    hopdata_received |= (1<<(index/5));
                }else{
                    debug("frsky: invalid bind idx");
                    debug_put_uint8(index/5);
                    debug_put_newline();
                }

                //make sure we never read the same packet twice by crc flag
                frsky_packet_buffer[FRSKY_PACKET_BUFFER_SIZE-1] = 0x00;
            }
        }
    }

#if FRSKY_DEBUG_BIND_DATA
    debug("frsky: hop[] = ");
    for(i=0; i<FRSKY_HOPTABLE_SIZE; i++){
        debug_put_hex8(storage.frsky_hop_table[i]);
        debug_putc(' ');
        debug_flush();
    }
    debug_putc('\n');
#endif

    //idle
    cc25xx_strobe(RFST_SIDLE);
}

static void frsky_calib_pll(void)
{
    uint8_t i;
    uint8_t ch;

    debug("frsky: calib pll\n");

    //fine tune offset
    cc25xx_set_register(FSCTRL0, storage.frsky_freq_offset);

    debug("frsky: tuning hop[] =");

    //calibrate pll for all channels
    for(i=0; i<FRSKY_HOPTABLE_SIZE; i++){
        //reset wdt
        wdt_reset();

        //fetch channel from hop_table:
        ch = storage.frsky_hop_table[i];

        //debug info
        debug_putc(' ');
        debug_put_hex8(ch);

        //set channel number
        frsky_tune_channel(ch);

        //store pll calibration:
        frsky_calib_fscal1_table[i] = cc25xx_get_register(FSCAL1);
    }
    debug_put_newline();

    //only needed once:
    frsky_calib_fscal3 = cc25xx_get_register(FSCAL3);
    frsky_calib_fscal2 = cc25xx_get_register(FSCAL2);

    //return to idle
    cc25xx_strobe(RFST_SIDLE);

    debug("frsky: calib fscal0 = ");
    debug_put_int8(storage.frsky_freq_offset);
    debug("\nfrsky: calib fscal1 = ");
    for(i=0; i<FRSKY_HOPTABLE_SIZE; i++){
        debug_put_hex8(frsky_calib_fscal1_table[i]);
        debug_putc(' ');
        debug_flush();
    }
    debug("\nfrsky: calib fscal2 = 0x");
    debug_put_hex8(frsky_calib_fscal2);
    debug("\nfrsky: calib fscal3 = 0x");
    debug_put_hex8(frsky_calib_fscal3);
    debug_put_newline();
    debug_flush();

    debug("frsky: calib pll done\n");
}

static void frsky_set_channel(uint8_t hop_index)
{
    const uint8_t ch = storage.frsky_hop_table[hop_index];
    //debug_putc('S'); debug_put_hex8(ch);

    //go to idle
    cc25xx_strobe(RFST_SIDLE);

    //fetch and set our stored pll calib data:
    cc25xx_set_register(FSCAL3, frsky_calib_fscal3);
    cc25xx_set_register(FSCAL2, frsky_calib_fscal2);
    cc25xx_set_register(FSCAL1, frsky_calib_fscal1_table[hop_index]);

    //set channel
    cc25xx_set_register(CHANNR, ch);
}

static void frsky_increment_channel(int8_t cnt)
{
    int8_t next = frsky_current_ch_idx;
    //add increment
    next+=cnt;
    //convert to a safe unsigned number:
    if(next<0){
        next += FRSKY_HOPTABLE_SIZE;
    }
    if (next >= FRSKY_HOPTABLE_SIZE){
        next -= FRSKY_HOPTABLE_SIZE;
    }

    frsky_current_ch_idx = next;
    frsky_set_channel(frsky_current_ch_idx);
}

#if 0
static uint8_t frsky_extract_rssi(uint8_t rssi_raw)
{
#define FRSKY_RSSI_OFFSET 70
    if (rssi_raw >= 128){
        //adapted to fit better to the original values... FIXME: find real formula
        //return (rssi_raw * 18)/32 - 82;
        return ((((uint16_t)rssi_raw) * 18)>>5) - 82;
    }else{
        return ((((uint16_t)rssi_raw) * 18)>>5) + 65;
    }
}
#endif

void frsky_init(void)
{
    //uint8_t i;
    debug("frsky: init\n"); debug_flush();

    cc25xx_init();

    frsky_link_quality = 0;
    frsky_diversity_count = 0;
    frsky_packet_received = 0;
    frsky_packet_sent = 0;

    frsky_rssi = 100;

    // check if spi is working properly
    frsky_show_partinfo();

    // init frsky registersttings for cc2500
    frsky_configure();

    if (frsky_bind_jumper_set()){
        //do binding
        frsky_do_bind();
        //binding will never return/continue
    }

    //show info:
    debug("frsky: using txid 0x"); debug_flush();
    debug_put_hex8(storage.frsky_txid[0]);
    debug_put_hex8(storage.frsky_txid[1]);
    debug_put_newline();

    //init txid matching
    frsky_configure_address();

    //tune cc2500 pll and save the values to ram
    frsky_calib_pll();

    debug("frsky: init done\n");debug_flush();
}

static void frskySetup(const uint32_t *rxSpiId)
{
    storageInit();
    frsky_init();
    rxSpiId = NULL; // !!TODO remove this once  configurator supports setting rx_id
    if (rxSpiId == NULL || *rxSpiId == 0) {
        //rxSpiIdPtr = NULL;
        protocolState = STATE_BIND;
        frskyBind();
        //!!TODO - change back into data mode and continue
    } else {
        //rxSpiIdPtr = (uint32_t*)rxSpiId;
        // use the rxTxAddr provided and go straight into DATA_STATE
        //!!TODO set storage data, ie frsky_txid[2], frsky_hop_table[FRSKY_HOPTABLE_SIZE], frsky_freq_offset;
    }
}

void frskyInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);
    rxRuntimeConfig->channelCount = FRSKY_CHANNEL_COUNT;
    frskySetup(&rxConfig->rx_spi_id);
}
#endif

