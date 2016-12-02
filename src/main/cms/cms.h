#pragma once

#include "drivers/display.h"

#include "common/time.h"

// Device management
bool cmsDisplayPortRegister(displayPort_t *pDisplay);

// For main.c and scheduler
void cmsInit(void);
void cmsHandler(timeUs_t currentTimeUs);

long cmsMenuChange(displayPort_t *pPort, const void *ptr);
long cmsMenuExit(displayPort_t *pPort, const void *ptr);

#define CMS_STARTUP_HELP_TEXT1 "MENU: THR MID"
#define CMS_STARTUP_HELP_TEXT2     "+ YAW LEFT"
#define CMS_STARTUP_HELP_TEXT3     "+ PITCH UP"
