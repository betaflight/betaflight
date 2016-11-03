//
// CMS things for blackbox and flashfs.
// Should be part of blackbox.c (or new blackbox/blackbox_cms.c) and io/flashfs.c
//
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#include "build/version.h"

#ifdef CMS

#include "drivers/system.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#include "io/cms.h"
#include "io/cms_types.h"
#include "io/cms_blackbox.h"

#include "io/flashfs.h"

#ifdef USE_FLASHFS
long cmsx_EraseFlash(displayPort_t *pDisplay, void *ptr)
{
    UNUSED(ptr);

    displayClear(pDisplay);
    displayWrite(pDisplay, 5, 3, "ERASING FLASH...");
    displayResync(pDisplay); // Was max7456RefreshAll(); Why at this timing?

    flashfsEraseCompletely();
    while (!flashfsIsReady()) {
        delay(100);
    }

    displayClear(pDisplay);
    displayResync(pDisplay); // Was max7456RefreshAll(); wedges during heavy SPI?

    return 0;
}
#endif // USE_FLASHFS

uint8_t cmsx_FeatureBlackbox;

OSD_UINT8_t entryBlackboxRateDenom = {&masterConfig.blackbox_rate_denom,1,32,1};

OSD_Entry cmsx_menuBlackbox[] =
{
    {"--- BLACKBOX ---", OME_Label, NULL, NULL, 0},
    {"ENABLED", OME_Bool, NULL, &cmsx_FeatureBlackbox, 0},
    {"RATE DENOM", OME_UINT8, NULL, &entryBlackboxRateDenom, 0},
#ifdef USE_FLASHFS
    {"ERASE FLASH", OME_Submenu, cmsx_EraseFlash, NULL, 0},
#endif // USE_FLASHFS
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

void cmsx_Blackbox_FeatureRead(void)
{
    cmsx_FeatureBlackbox = feature(FEATURE_BLACKBOX) ? 1 : 0;
}

void cmsx_Blackbox_FeatureWriteback(void)
{
    if (cmsx_FeatureBlackbox)
        featureSet(FEATURE_BLACKBOX);
    else
        featureClear(FEATURE_BLACKBOX);
}
#endif
