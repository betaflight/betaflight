/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_OSD) && defined(USE_OSD_CUSTOM_TEXT)

#include "common/time.h"
#include "drivers/serial.h"
#include "io/serial.h"
#include "osd/osd_custom_text.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#define INPUT_BUFFER_SIZE 64
#define DISPLAY_BUFFER_SIZE 32

PG_REGISTER_WITH_RESET_FN(osdCustomTextConfig_t, osdCustomTextConfig, PG_OSD_CUSTOM_TEXT_CONFIG, 0);

void pgResetFn_osdCustomTextConfig(osdCustomTextConfig_t *config)
{
    config->terminator = OSD_CUSTOM_TEXT_TERMINATOR_LF;
}

static serialPort_t *osdCustomTextSerialPort = NULL;
static char inputBuffer[INPUT_BUFFER_SIZE];
static uint8_t inputPos = 0;
static char displayBuffer[DISPLAY_BUFFER_SIZE];

bool osdCustomTextInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_OSD_CUSTOM_TEXT);
    if (!portConfig) {
        return false;
    }

    // Use telemetry baud rate index for OSD custom text
    const uint32_t baudrate = baudRates[portConfig->telemetry_baudrateIndex];

    osdCustomTextSerialPort = openSerialPort(
        portConfig->identifier,
        FUNCTION_OSD_CUSTOM_TEXT,
        NULL,  // No RX callback, we'll poll in update function
        NULL,
        baudrate,
        MODE_RX,
        SERIAL_NOT_INVERTED
    );

    if (!osdCustomTextSerialPort) {
        return false;
    }

    // Initialize buffers
    memset(inputBuffer, 0, INPUT_BUFFER_SIZE);
    memset(displayBuffer, 0, DISPLAY_BUFFER_SIZE);
    inputPos = 0;

    return true;
}

void osdCustomTextUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!osdCustomTextSerialPort) {
        return;
    }

    // Determine termination character based on config
    const uint8_t termChar = (osdCustomTextConfig()->terminator == OSD_CUSTOM_TEXT_TERMINATOR_NULL) ? 0x00 : '\n';

    // Read all available bytes
    while (serialRxBytesWaiting(osdCustomTextSerialPort)) {
        const uint8_t c = serialRead(osdCustomTextSerialPort);

        // Check for configured termination character
        if (c == termChar) {
            if (inputPos > 0) {
                // Copy input buffer to display buffer, truncate to fit
                const uint8_t copyLen = (inputPos < (DISPLAY_BUFFER_SIZE - 1)) ? inputPos : (DISPLAY_BUFFER_SIZE - 1);
                memcpy(displayBuffer, inputBuffer, copyLen);
                displayBuffer[copyLen] = '\0';
            } else {
                // Empty message clears the display
                displayBuffer[0] = '\0';
            }
            // Reset input buffer
            inputPos = 0;
        }
        // Ignore carriage return
        else if (c == '\r') {
            // Do nothing, just skip
        }
        // Buffer regular characters (skip null bytes when not used as terminator)
        else if (c != 0x00 && inputPos < (INPUT_BUFFER_SIZE - 1)) {
            inputBuffer[inputPos++] = c;
        }
        // If buffer is full, keep overwriting the last position until we get a terminator
        // This prevents buffer overflow while waiting for terminator
    }
}

const char* osdCustomTextGet(void)
{
    return displayBuffer;
}

#endif // USE_OSD && USE_OSD_CUSTOM_TEXT
