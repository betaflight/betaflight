#pragma once

struct displayPort_s;

typedef struct displayPortVTable_s {
    int (*begin)(void);
    int (*end)(void);
    int (*clear)(void);
    int (*write)(uint8_t col, uint8_t row, char *text);
    int (*heartbeat)(void);
    void (*resync)(struct displayPort_s *pPort);
    uint32_t (*txBytesFree)(void);
} displayPortVTable_t;

typedef struct displayPort_s {
    displayPortVTable_t *vTable;
    uint8_t rows;
    uint8_t cols;
    uint16_t buftime;
    uint16_t bufsize;

    // CMS state
    bool cleared;
} displayPort_t;

// Device management
typedef void (*cmsDeviceInitFuncPtr)(displayPort_t *pPort);
bool cmsDeviceRegister(cmsDeviceInitFuncPtr);

// For main.c and scheduler
void cmsInit(void);
void cmsHandler(uint32_t currentTime);

// Required for external CMS tables
void cmsScreenClear(displayPort_t *pPort);
void cmsScreenResync(displayPort_t *pPort);
int cmsScreenWrite(displayPort_t *pPort, uint8_t x, uint8_t y, char *s);

long cmsMenuChange(displayPort_t *pPort, void *ptr);
long cmsMenuExit(displayPort_t *pPort, void *ptr);

#define CMS_STARTUP_HELP_TEXT1 "MENU: THR MID"
#define CMS_STARTUP_HELP_TEXT2     "+ YAW LEFT"
#define CMS_STARTUP_HELP_TEXT3     "+ PITCH UP"
