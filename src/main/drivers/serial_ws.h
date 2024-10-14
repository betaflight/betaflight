/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define PRE_LWS_BUFFER 40
#define RX_BUFFER_SIZE 1400
#define TX_BUFFER_SIZE 1400

typedef struct {
    serialPort_t port;
    // prepend with allocated bytes, so ws transmission can prepend protocols stuff without reallocs
    uint8_t rxBufferPre[PRE_LWS_BUFFER];
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    uint8_t txBufferPre[PRE_LWS_BUFFER];
    uint8_t txBuffer[TX_BUFFER_SIZE];

    //server context
    struct lws_context *context;
    //client ?
    struct lws *wsi;

    //dyad_Stream *serv;
    //dyad_Stream *conn;

    bool connected;
    uint16_t clientCount;
    uint8_t id;
} wsPort_t;

serialPort_t *serialWsOpen( int id,
                            serialReceiveCallbackPtr rxCallback,
                            void *rxCallbackData,
                            uint32_t baudRate,
                            portMode_e mode,
                            portOptions_e options);

// wsPort API
void wsDataIn(wsPort_t *instance, uint8_t *ch, int size);
void wsDataOut(wsPort_t *instance);

void wsUpdate(void);

