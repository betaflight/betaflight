#include "board.h"
#include "mw.h"

static bool standardBoardAlignment = true;     // board orientation correction
static float boardRotation[3][3];              // matrix

int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

void initBoardAlignment(void)
{
    float roll, pitch, yaw;

    // standard alignment, nothing to calculate
    if (!mcfg.board_align_roll && !mcfg.board_align_pitch && !mcfg.board_align_yaw)
        return;

    standardBoardAlignment = false;

    // deg2rad
    roll = mcfg.board_align_roll * M_PI / 180.0f;
    pitch = mcfg.board_align_pitch * M_PI / 180.0f;
    yaw = mcfg.board_align_yaw * M_PI / 180.0f;

    // define rotation matrix
    boardRotation[0][0] = cosf(roll) * cosf(pitch);
    boardRotation[0][1] = cosf(roll) * sinf(pitch) * sinf(yaw) - sinf(roll) * cosf(yaw);
    boardRotation[0][2] = cosf(roll) * sinf(pitch) * cosf(yaw) + sinf(roll) * sinf(yaw);

    boardRotation[1][0] = sinf(roll) * cosf(pitch);
    boardRotation[1][1] = sinf(roll) * sinf(pitch) * sinf(yaw) + cosf(roll) * cosf(yaw);
    boardRotation[1][2] = sinf(roll) * sinf(pitch) * cosf(yaw) - cosf(roll) * sinf(yaw);

    boardRotation[2][0] = -sinf(pitch);
    boardRotation[2][1] = cosf(pitch) * sinf(yaw);
    boardRotation[2][2] = cosf(pitch) * cosf(yaw);
}

void alignBoard(int16_t *vec)
{
    int16_t x = vec[X];
    int16_t y = vec[Y];
    int16_t z = vec[Z];

    vec[X] = boardRotation[0][0] * x + boardRotation[0][1] * y + boardRotation[0][2] * z;
    vec[Y] = boardRotation[1][0] * x + boardRotation[1][1] * y + boardRotation[1][2] * z;
    vec[Z] = boardRotation[2][0] * x + boardRotation[2][1] * y + boardRotation[2][2] * z;
}

void alignSensors(int16_t *src, int16_t *dest, uint8_t rotation)
{
    switch (rotation) {
        case CW0_DEG:
            dest[X] = src[X];
            dest[Y] = src[Y];
            dest[Z] = src[Z];
            break;
        case CW90_DEG:
            dest[X] = src[Y];
            dest[Y] = -src[X];
            dest[Z] = src[Z];
            break;
        case CW180_DEG:
            dest[X] = -src[X];
            dest[Y] = -src[Y];
            dest[Z] = src[Z];
            break;
        case CW270_DEG:
            dest[X] = -src[Y];
            dest[Y] = src[X];
            dest[Z] = src[Z];
            break;
        case CW0_DEG_FLIP:
            dest[X] = -src[X];
            dest[Y] = src[Y];
            dest[Z] = -src[Z];
            break;
        case CW90_DEG_FLIP:
            dest[X] = src[Y];
            dest[Y] = src[X];
            dest[Z] = -src[Z];
            break;
        case CW180_DEG_FLIP:
            dest[X] = src[X];
            dest[Y] = -src[Y];
            dest[Z] = -src[Z];
            break;
        case CW270_DEG_FLIP:
            dest[X] = -src[Y];
            dest[Y] = -src[X];
            dest[Z] = -src[Z];
            break;
        default:
            break;
    }

    if (!standardBoardAlignment)
        alignBoard(dest);
}
