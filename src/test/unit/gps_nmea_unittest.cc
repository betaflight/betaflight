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

#include <string.h>

extern "C" {
    #include "platform.h"

    #include "drivers/serial.h"
    #include "io/serial.h"

    #include "io/gps_utils.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static serialPort_t *gpsPort;
static int serialWritePos = 0;

#define SERIAL_BUFFER_SIZE 256
static uint8_t serialWriteBuffer[SERIAL_BUFFER_SIZE];

serialPort_t serialTestInstance;

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    EXPECT_EQ(instance, &serialTestInstance);
    EXPECT_LT(serialWritePos, sizeof(serialWriteBuffer));
    serialWriteBuffer[serialWritePos++] = ch;
}

void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count)
{
    while(count--)
        serialWrite(instance, *data++);
}

void serialTestResetBuffers()
{
    gpsPort = &serialTestInstance;
    memset(&serialWriteBuffer, 0, sizeof(serialWriteBuffer));
    serialWritePos = 0;
}


// list of valid NMEA commands
const char * nmeaSamples[] = {
    "$PUBX,41,1,0003,0001,38400,0*26\r\n",
    "$PSRF103,00,6,00,0*23\r\n",
    "$PCAS02,100*1E\r\n",
    "$PSRF103,03,00,00,01*27\r\n",
};

TEST(nmeaUtilsTest, TestNmeaPrintf)
{
    for (unsigned i = 0; i < ARRAYLEN(nmeaSamples); i++) {
        serialTestResetBuffers();
        const char* sample = nmeaSamples[i];
        int starPos = strcspn(sample, "*");
        EXPECT_EQ(strlen(sample) - strlen("*XX\r\n"), starPos);
        const char* tx = strndup(sample, starPos);
        nmeaPrintf(gpsPort, "%s", tx);
        serialWrite(gpsPort, 0);  // terminate string for simplicity
        free((void*)tx);
        ASSERT_STREQ((const char*)serialWriteBuffer, sample);
    }
}
