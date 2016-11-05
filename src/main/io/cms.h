#pragma once

#include "drivers/display.h"

// Device management
typedef void (*cmsDeviceInitFuncPtr)(displayPort_t *pPort);
bool cmsDeviceRegister(cmsDeviceInitFuncPtr);

// For main.c and scheduler
void cmsInit(void);
void cmsHandler(uint32_t currentTime);

long cmsMenuChange(displayPort_t *pPort, void *ptr);
//long cmsMenuExit(displayPort_t *pPort, void *ptr);

#define CMS_STARTUP_HELP_TEXT1 "MENU: THR MID"
#define CMS_STARTUP_HELP_TEXT2     "+ YAW LEFT"
#define CMS_STARTUP_HELP_TEXT3     "+ PITCH UP"
