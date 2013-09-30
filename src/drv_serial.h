#pragma once

typedef enum portMode_t {
    MODE_RX = 1,
    MODE_TX = 2,
    MODE_RXTX = MODE_RX | MODE_TX
} portMode_t;

typedef struct serialPort {
    
    const struct serialPortVTable *vTable;
    
    portMode_t mode;
    uint32_t baudRate;

    uint32_t rxBufferSize;
    uint32_t txBufferSize;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    uint32_t rxBufferHead;
    uint32_t rxBufferTail;
    uint32_t txBufferHead;
    uint32_t txBufferTail;

    // FIXME rename member to rxCallback
    serialReceiveCallbackPtr callback;
} serialPort_t;

struct serialPortVTable {
    void (*serialWrite)(serialPort_t *instance, uint8_t ch);
    uint8_t (*serialTotalBytesWaiting)(serialPort_t *instance);
    uint8_t (*serialRead)(serialPort_t *instance);
};

static inline void serialWrite(serialPort_t *instance, uint8_t ch)
{
    instance->vTable->serialWrite(instance, ch);
}

static inline uint8_t serialTotalBytesWaiting(serialPort_t *instance)
{
    return instance->vTable->serialTotalBytesWaiting(instance);
}

static inline uint8_t serialRead(serialPort_t *instance)
{
    return instance->vTable->serialRead(instance);
}

void serialPrint(serialPort_t *instance, const char *str);
