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
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    // standard alignment, nothing to calculate
    if (!mcfg.board_align_roll && !mcfg.board_align_pitch && !mcfg.board_align_yaw)
        return;

    standardBoardAlignment = false;

    // deg2rad
    roll = mcfg.board_align_roll * M_PI / 180.0f;
    pitch = mcfg.board_align_pitch * M_PI / 180.0f;
    yaw = mcfg.board_align_yaw * M_PI / 180.0f;

    cosx = cosf(roll);
    sinx = sinf(roll);
    cosy = cosf(pitch);
    siny = sinf(pitch);
    cosz = cosf(yaw);
    sinz = sinf(yaw);

    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    // define rotation matrix
    boardRotation[0][0] = coszcosy;
    boardRotation[0][1] = -cosy * sinz;
    boardRotation[0][2] = siny;

    boardRotation[1][0] = sinzcosx + (coszsinx * siny);
    boardRotation[1][1] = coszcosx - (sinzsinx * siny);
    boardRotation[1][2] = -sinx * cosy;

    boardRotation[2][0] = (sinzsinx) - (coszcosx * siny);
    boardRotation[2][1] = (coszsinx) + (sinzcosx * siny);
    boardRotation[2][2] = cosy * cosx;
}

void alignBoard(int16_t *vec)
{
    int16_t x = vec[X];
    int16_t y = vec[Y];
    int16_t z = vec[Z];

    vec[X] = lrintf(boardRotation[0][0] * x + boardRotation[1][0] * y + boardRotation[2][0] * z);
    vec[Y] = lrintf(boardRotation[0][1] * x + boardRotation[1][1] * y + boardRotation[2][1] * z);
    vec[Z] = lrintf(boardRotation[0][2] * x + boardRotation[1][2] * y + boardRotation[2][2] * z);
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

#ifdef PROD_DEBUG
void productionDebug(void)
{
    gpio_config_t gpio;

    // remap PB6 to USART1_TX
    gpio.pin = Pin_6;
    gpio.mode = Mode_AF_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(GPIOB, &gpio);
    gpioPinRemapConfig(AFIO_MAPR_USART1_REMAP, true);
    serialInit(mcfg.serial_baudrate);
    delay(25);
    serialPrint(core.mainport, "DBG ");
    printf("%08x%08x%08x OK\n", U_ID_0, U_ID_1, U_ID_2);
    serialPrint(core.mainport, "EOF");
    delay(25);
    gpioPinRemapConfig(AFIO_MAPR_USART1_REMAP, false);
}
#endif
