#include "board.h"
#include "mw.h"

// Driver for DFRobot I2C Led Ring
#define LED_RING_ADDRESS    0x6D

bool ledringDetect(void)
{
    bool ack = false;
    uint8_t sig = 'e';
    
    ack = i2cWrite(LED_RING_ADDRESS, 0xFF, sig);
    if (!ack)
        return false;
    return true;
}

void ledringState(void)
{
    uint8_t b[10];
    static uint8_t state;

    if (state == 0) {
        b[0] = 'z';
        b[1] = (180 - heading) / 2;	// 1 unit = 2 degrees;
        i2cWriteBuffer(LED_RING_ADDRESS, 0xFF, 2, b);
        state = 1;
    } else if (state == 1) {
        b[0] = 'y';
        b[1] = constrain(angle[ROLL] / 10 + 90, 0, 180);
        b[2] = constrain(angle[PITCH] / 10 + 90, 0, 180);
        i2cWriteBuffer(LED_RING_ADDRESS, 0xFF, 3, b);
        state = 2;
    } else if (state == 2) {
        b[0] = 'd';		// all unicolor GREEN 
        b[1] = 1;
        if (f.ARMED)
            b[2] = 1;
        else
            b[2] = 0;
        i2cWriteBuffer(LED_RING_ADDRESS, 0xFF, 3, b);
        state = 0;
    }
}

void ledringBlink(void)
{
    uint8_t b[3];
    b[0] = 'k';
    b[1] = 10;
    b[2] = 10;
    i2cWriteBuffer(LED_RING_ADDRESS, 0xFF, 3, b);
}
