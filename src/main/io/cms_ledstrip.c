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

//local variable to keep color value
uint8_t ledColor;

static const char * const LED_COLOR_NAMES[] = {
    "BLACK   ",
    "WHITE   ",
    "RED     ",
    "ORANGE  ",
    "YELLOW  ",
    "LIME GRN",
    "GREEN   ",
    "MINT GRN",
    "CYAN    ",
    "LT BLUE ",
    "BLUE    ",
    "DK VIOLT",
    "MAGENTA ",
    "DEEP PNK"
};

//find first led with color flag and restore color index
//after saving all leds with flags color will have color set in OSD
void cmsx_GetLedColor(void)
{
    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        const ledConfig_t *ledConfig = &masterConfig.ledConfigs[ledIndex];

        int fn = ledGetFunction(ledConfig);

        if (fn == LED_FUNCTION_COLOR) {
            ledColor = ledGetColor(ledConfig);
            break;
        }
    }
}

//udate all leds with flag color
static long applyLedColor(displayPort_t *pDisplay, void *ptr)
{
    UNUSED(ptr);
    UNUSED(pDisplay); // Arrgh

    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        ledConfig_t *ledConfig = &masterConfig.ledConfigs[ledIndex];
        if (ledGetFunction(ledConfig) == LED_FUNCTION_COLOR)
            *ledConfig = DEFINE_LED(ledGetX(ledConfig), ledGetY(ledConfig), ledColor, ledGetDirection(ledConfig), ledGetFunction(ledConfig), ledGetOverlay(ledConfig), 0);
    }

    return 0;
}

uint8_t cmsx_FeatureLedstrip;

OSD_TAB_t entryLed = {&ledColor, 13, &LED_COLOR_NAMES[0]};

long cmsx_Ledstrip_FeatureRead(void)
{
    cmsx_FeatureLedstrip = feature(FEATURE_LED_STRIP) ? 1 : 0;

    return 0;
}

long cmsx_Ledstrip_FeatureWriteback(void)
{
    if (cmsx_FeatureLedstrip)
        featureSet(FEATURE_LED_STRIP);
    else
        featureClear(FEATURE_LED_STRIP);

    return 0;
}

long cmsx_Ledstrip_ConfigRead(void)
{
    cmsx_GetLedColor();

    return 0;
}

long cmsx_Ledstrip_onEnter(void)
{
    cmsx_Ledstrip_FeatureRead();
    cmsx_Ledstrip_ConfigRead();

    return 0;
}

OSD_Entry cmsx_menuLedstripEntries[] =
{
    {"--- LED STRIP ---", OME_Label, NULL, NULL, 0},
    {"ENABLED", OME_Bool, NULL, &cmsx_FeatureLedstrip, 0},
    {"LED COLOR", OME_TAB, applyLedColor, &entryLed, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuLedstrip = {
    "MENULED",
    OME_MENU,
    cmsx_Ledstrip_onEnter,
    NULL,
    cmsx_Ledstrip_FeatureWriteback,
    cmsx_menuLedstripEntries,
};
#endif // LED_STRIP
#endif // CMS
