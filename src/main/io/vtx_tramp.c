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

/* Created by jflyper */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

#include "platform.h"

#ifdef VTX_TRAMP

#include "build/debug.h"

#include "common/utils.h"

#include "io/serial.h"
#include "drivers/serial.h"
#include "drivers/vtx_var.h"
#include "io/vtx_tramp.h"

static serialPort_t *trampSerialPort = NULL;

static uint8_t trampCmdBuffer[16];

void trampClearBuffer(uint8_t *buf)
{
    for (int i = 0 ; i < 16 ; i++) {
        buf[i] = 0;
    }
}

void trampPrepareBuffer(uint8_t *buf, uint8_t cmd)
{
    trampClearBuffer(buf);

    trampCmdBuffer[0] = 15;
    trampCmdBuffer[1] = cmd;
}

void trampWriteBuf(uint8_t *buf)
{
    serialWriteBuf(trampSerialPort, buf, 16);
}

void trampChecksum(uint8_t *buf)
{
    uint8_t cksum = 0;

    for (int i = 0 ; i < 14 ; i++)
        cksum += buf[i];

    buf[14] = cksum;
}

void trampSetFreq(uint16_t freq)
{
    if (!trampSerialPort)
        return;

    trampPrepareBuffer(trampCmdBuffer, 'F');
    trampCmdBuffer[2] = freq & 0xff;
    trampCmdBuffer[3] = (freq >> 8) & 0xff;
    trampChecksum(trampCmdBuffer);
    trampWriteBuf(trampCmdBuffer);
}

void trampSetBandChan(uint8_t band, uint8_t chan)
{
    if (!trampSerialPort)
        return;

debug[0] = band;
debug[1] = chan;
debug[2] = vtx58FreqTable[band - 1][chan - 1];

    trampSetFreq(vtx58FreqTable[band - 1][chan - 1]);
}

void trampSetRFPower(uint8_t level)
{
    if (!trampSerialPort)
        return;

    trampPrepareBuffer(trampCmdBuffer, 'F');
    trampCmdBuffer[2] = level;
    trampChecksum(trampCmdBuffer);
    trampWriteBuf(trampCmdBuffer);
}

void trampSetPitmode(uint8_t onoff)
{
    if (!trampSerialPort)
        return;

    trampPrepareBuffer(trampCmdBuffer, 'I');
    trampCmdBuffer[2] = onoff;
    trampChecksum(trampCmdBuffer);
    trampWriteBuf(trampCmdBuffer);
}

bool trampInit()
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_CONTROL);

    if (portConfig) {
        trampSerialPort = openSerialPort(portConfig->identifier, FUNCTION_VTX_CONTROL, NULL, 9600, MODE_RXTX, 0); // MODE_TX possible?
    }

    if (!trampSerialPort) {
        return false;
    }

    return true;
}

#ifdef CMS
#include "cms/cms.h"
#include "cms/cms_types.h"

uint8_t trampCmsBand = 1;
uint8_t trampCmsChan = 1;

static OSD_TAB_t trampCmsEntBand = { &trampCmsBand, 5, vtx58BandNames, NULL };

static OSD_TAB_t trampCmsEntChan = { &trampCmsChan, 8, vtx58ChanNames, NULL };

static long trampCmsCommence(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    trampSetBandChan(trampCmsBand, trampCmsChan);

    return MENU_CHAIN_BACK;
}

static OSD_Entry trampCmsMenuCommenceEntries[] = {
    { "CONFIRM", OME_Label,   NULL,          NULL, 0 },
    { "YES",     OME_Funcall, trampCmsCommence, NULL, 0 },
    { "BACK",    OME_Back, NULL, NULL, 0 },
    { NULL,      OME_END, NULL, NULL, 0 }
};

static CMS_Menu trampCmsMenuCommence = {
    .GUARD_text = "XVTXTRC",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = trampCmsMenuCommenceEntries,
};

static OSD_Entry trampMenuEntries[] =
{
    { "- TRAMP -", OME_Label, NULL, NULL, 0 },

    //{ "",       OME_Label,   NULL,                   saCmsStatusString,  DYNAMIC },
    { "BAND",   OME_TAB,     NULL,                   &trampCmsEntBand,      0 },
    { "CHAN",   OME_TAB,     NULL,                   &trampCmsEntChan,      0 },
    //{ "(FREQ)", OME_UINT16,  NULL,                   &trampCmsEntFreqRef,   DYNAMIC },
    //{ "POWER",  OME_TAB,     saCmsConfigPowerByGvar, &saCmsEntPower,     0 },
    { "SET",    OME_Submenu, cmsMenuChange,          &trampCmsMenuCommence, 0 },
    //{ "CONFIG", OME_Submenu, cmsMenuChange,          &saCmsMenuConfig,   0 },

    { "BACK",   OME_Back, NULL, NULL, 0 },
    { NULL,     OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuVtxTramp = {
    .GUARD_text = "XVTXTR",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = trampMenuEntries,
};
#endif

#endif // VTX_TRAMP
