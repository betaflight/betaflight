
#pragma once

//type of elements
typedef enum
{
    OME_Label,
    OME_Back,
    OME_OSD_Exit,
    OME_Submenu,
    OME_Bool,
    OME_INT8,
    OME_UINT8,
    OME_UINT16,
    OME_INT16,
    OME_FLOAT, //only up to 255 value and cant be 2.55 or 25.5, just for PID's
    //wlasciwosci elementow
    OME_VISIBLE,
    OME_POS,
    OME_TAB,
    OME_END,
} OSD_MenuElement;

typedef void (* OSDMenuFuncPtr)(void *data);
#define OME_READONLY ((OSDMenuFuncPtr)-1)

typedef struct
{
    char *text;
    OSD_MenuElement type;
    OSDMenuFuncPtr func;
    void *data;
} OSD_Entry;

typedef struct
{
    uint8_t *val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
} OSD_UINT8_t;

typedef struct
{
    int8_t *val;
    int8_t min;
    int8_t max;
    int8_t step;
} OSD_INT8_t;

typedef struct
{
    int16_t *val;
    int16_t min;
    int16_t max;
    int16_t step;
} OSD_INT16_t;

typedef struct
{
    uint16_t *val;
    uint16_t min;
    uint16_t max;
    uint16_t step;
} OSD_UINT16_t;

typedef struct
{
    uint8_t *val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
    uint16_t multipler;
} OSD_FLOAT_t;

typedef struct
{
    uint8_t *val;
    uint8_t max;
    const char * const *names;
} OSD_TAB_t;

typedef struct
{
    uint8_t *val;
    uint8_t max;
    char **names;
} OSD_dynaTAB_t;
