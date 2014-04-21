#pragma once

extern uint16_t acc_1G;

typedef void (* sensorInitFuncPtr)(void);   // sensor init prototype
typedef void (* sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype

typedef struct gyro_s
{
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    sensorReadFuncPtr temperature;                          // read temperature if available
    float scale;                                            // scalefactor
} gyro_t;

typedef struct acc_s
{
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    char revisionCode;                                      // a revision code for the sensor, if known
} acc_t;
