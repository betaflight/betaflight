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

#ifdef LED_STRIP

static bool featureRead = false;
static uint8_t cmsx_FeatureLedstrip;

static long cmsx_Ledstrip_FeatureRead(void)
{
    if (!featureRead) {
        cmsx_FeatureLedstrip = feature(FEATURE_LED_STRIP) ? 1 : 0;
        featureRead = true;
    }

    return 0;
}

static long cmsx_Ledstrip_FeatureWriteback(void)
{
    if (cmsx_FeatureLedstrip)
        featureSet(FEATURE_LED_STRIP);
    else
        featureClear(FEATURE_LED_STRIP);

    return 0;
}

static OSD_Entry cmsx_menuLedstripEntries[] =
{
    {"--- LED STRIP ---", OME_Label, NULL, NULL, 0},
    {"ENABLED", OME_Bool, NULL, &cmsx_FeatureLedstrip, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuLedstrip = {
    "MENULED",
    OME_MENU,
    cmsx_Ledstrip_FeatureRead,
    NULL,
    cmsx_Ledstrip_FeatureWriteback,
    cmsx_menuLedstripEntries,
};
#endif // LED_STRIP
#endif // CMS
