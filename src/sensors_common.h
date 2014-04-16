#pragma once

typedef void (* sensorInitFuncPtr)(sensor_align_e align);   // sensor init prototype
typedef void (* sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype

typedef struct sensor_t
{
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    sensorReadFuncPtr temperature;                          // read temperature if available
    float scale;                                            // scalefactor (currently used for gyro only, todo for accel)
} sensor_t;

typedef enum {
    X = 0,
    Y,
    Z
} sensor_axis_e;
