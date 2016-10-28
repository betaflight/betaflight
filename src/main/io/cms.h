#pragma once

typedef struct screenFnVTable_s {
    int (*begin)(void);
    int (*end)(void);
    int (*clear)(void);
    int (*write)(uint8_t, uint8_t, char *);
    int (*heartbeat)(void);
    void (*resync)(void);
} screenFnVTable_t;

typedef struct displayPort_s {
    uint8_t rows;
    uint8_t cols;
    uint16_t buftime;
    uint16_t bufsize;
    uint16_t batchsize;         // Computed by CMS
    screenFnVTable_t *VTable;

    // CMS state
    bool cleared;
} displayPort_t;

// Device management
typedef void (*cmsDeviceInitFuncPtr)(displayPort_t *);
bool cmsDeviceRegister(cmsDeviceInitFuncPtr);

// For main.c and scheduler
void cmsInit(void);
void cmsHandler(uint32_t);


#if 0
void cmsOpenMenu();
void cmsUpdate(uint32_t);
void cmsScreenResync(displayPort_t *);
#endif

// Required for external CMS tables

void cmsChangeScreen(displayPort_t *, void *);
void cmsExitMenu(displayPort_t *, void *);

#define STARTUP_HELP_TEXT1 "MENU: THR MID"
#define STARTUP_HELP_TEXT2     "+ YAW LEFT"
#define STARTUP_HELP_TEXT3     "+ PITCH UP"

