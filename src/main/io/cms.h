#pragma once

void cmsInit(void);
void cmsHandler(uint32_t);

void cmsOpenMenu();
void cmsUpdate(uint32_t);
void cmsScreenResync(void);

// Required for external CMS tables

void cmsChangeScreen(void * ptr);
void cmsExitMenu(void * ptr);

#define STARTUP_HELP_TEXT1 "MENU: THR MID"
#define STARTUP_HELP_TEXT2     "+ YAW LEFT"
#define STARTUP_HELP_TEXT3     "+ PITCH UP"
