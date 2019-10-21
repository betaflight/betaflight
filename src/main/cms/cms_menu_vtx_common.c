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

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

#include "platform.h"

#if defined(USE_CMS) && defined(USE_VTX_CONTROL) && (defined(USE_VTX_TRAMP) || defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_RTC6705))

#include "common/printf.h"

#include "cms/cms.h"
#include "cms/cms_menu_vtx_rtc6705.h"
#include "cms/cms_menu_vtx_smartaudio.h"
#include "cms/cms_menu_vtx_tramp.h"
#include "cms/cms_types.h"

#include "drivers/vtx_common.h"

#include "cms_menu_vtx_common.h"

#define MAX_STATUS_LINE_LENGTH 21

static char statusLine1[MAX_STATUS_LINE_LENGTH] = "";
static char statusLine2[MAX_STATUS_LINE_LENGTH] = "";

static long setStatusMessage(void)
{
    vtxDevice_t *device = vtxCommonDevice();

    statusLine1[0] = 0;
    statusLine2[0] = 0;

    if (!device) {
        tfp_sprintf(&statusLine1[0], "VTX NOT RESPONDING");
        tfp_sprintf(&statusLine2[0], "OR NOT CONFIGURED");
    } else {
        vtxDevType_e vtxType = vtxCommonGetDeviceType(device);
        if (vtxType == VTXDEV_UNSUPPORTED) {
            tfp_sprintf(&statusLine1[0], "UNSUPPORTED VTX TYPE");
        } else {
            tfp_sprintf(&statusLine1[0], "UNKNOWN VTX TYPE");
        }
    }
    return 0;
}

// Redirect to the proper menu based on the vtx device type
// If device isn't valid or not a supported type then don't
// redirect and instead display a local informational menu.
static const void *vtxMenuRedirect(void)
{
    vtxDevice_t *device = vtxCommonDevice();

    if (device) {
        vtxDevType_e vtxType = vtxCommonGetDeviceType(device);

        switch (vtxType) {
        
#if defined(USE_VTX_RTC6705)
        case VTXDEV_RTC6705:
            return &cmsx_menuVtxRTC6705;
#endif
#if defined(USE_VTX_SMARTAUDIO)
        case VTXDEV_SMARTAUDIO:
            return &cmsx_menuVtxSmartAudio;
#endif
#if defined(USE_VTX_TRAMP)
        case VTXDEV_TRAMP:
            return &cmsx_menuVtxTramp;
#endif

        default:
            return NULL;
        }
    }

    return NULL;
}

static const OSD_Entry vtxRedirectMenuEntries[] =
{
    { "",     OME_Label, NULL, statusLine1,  DYNAMIC },
    { "",     OME_Label, NULL, statusLine2,  DYNAMIC },
    { "",     OME_Label, NULL, NULL, 0 },
    { "BACK", OME_Back,  NULL, NULL, 0 },
    { NULL,   OME_END,   NULL, NULL, 0 }
};

CMS_Menu cmsx_menuVtxRedirect = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XVTXREDIRECT",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = setStatusMessage,
    .onExit = NULL,
    .checkRedirect = vtxMenuRedirect,
    .entries = vtxRedirectMenuEntries,
};

#endif
