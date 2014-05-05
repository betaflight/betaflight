#pragma once


enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PID_ITEM_COUNT
};

typedef struct pidProfile_s {
    uint8_t P8[PID_ITEM_COUNT];
    uint8_t I8[PID_ITEM_COUNT];
    uint8_t D8[PID_ITEM_COUNT];
} pidProfile_t;

enum {
    AI_ROLL = 0,
    AI_PITCH,
} angle_index_t;

#define ANGLE_INDEX_COUNT 2

// See http://en.wikipedia.org/wiki/Flight_dynamics
enum {
    FD_ROLL = 0,
    FD_PITCH,
    FD_YAW
} flight_dynamics_index_t;

#define FLIGHT_DYNAMICS_INDEX_COUNT 3

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

typedef struct fp_angles {
    float roll;
    float pitch;
    float yaw;
} fp_angles_def;

typedef union {
    float raw[3];
    fp_angles_def angles;
} fp_angles_t;

typedef struct int16_flightDynamicsTrims_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} int16_flightDynamicsTrims_def_t;

typedef union {
    int16_t raw[3];
    int16_flightDynamicsTrims_def_t trims;
} int16_flightDynamicsTrims_t;

typedef struct rollAndPitchTrims_s {
    int16_t roll;
    int16_t pitch;
} rollAndPitchTrims_t_def;

typedef union {
    int16_t raw[2];
    rollAndPitchTrims_t_def trims;
} rollAndPitchTrims_t;

typedef struct rollAndPitchInclination_s {
    // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
} rollAndPitchInclination_t_def;

typedef union {
    int16_t rawAngles[ANGLE_INDEX_COUNT];
    rollAndPitchInclination_t_def angle;
} rollAndPitchInclination_t;


extern rollAndPitchInclination_t inclination;

extern int16_t gyroData[FLIGHT_DYNAMICS_INDEX_COUNT];
extern int16_t gyroZero[FLIGHT_DYNAMICS_INDEX_COUNT];

extern int16_t gyroADC[XYZ_AXIS_COUNT], accADC[XYZ_AXIS_COUNT], accSmooth[XYZ_AXIS_COUNT];
extern int32_t accSum[XYZ_AXIS_COUNT];
extern int16_t axisPID[XYZ_AXIS_COUNT];

extern int16_t heading, magHold;

extern int32_t EstAlt;
extern int32_t AltHold;
extern int32_t EstAlt;
extern int32_t vario;

void mwDisarm(void);
void setPIDController(int type);
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims);
void resetErrorAngle(void);
void resetErrorGyro(void);


