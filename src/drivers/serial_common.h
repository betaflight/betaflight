#pragma once

typedef enum portMode_t {
    MODE_RX = 1  << 0,
    MODE_TX = 1 << 1,
    MODE_RXTX = MODE_RX | MODE_TX,
    MODE_SBUS = 1 << 2,
} portMode_t;

typedef void (* serialReceiveCallbackPtr)(uint16_t data);   // used by serial drivers to return frames to app

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
    
    // Specified baud rate may not be allowed by an implementation, use serialGetBaudRate to determine actual baud rate in use.
    void (*serialSetBaudRate)(serialPort_t *instance, uint32_t baudRate);
    
    bool (*isSerialTransmitBufferEmpty)(serialPort_t *instance);

    void (*setMode)(serialPort_t *instance, portMode_t mode);
};

inline void serialWrite(serialPort_t *instance, uint8_t ch);
inline uint8_t serialTotalBytesWaiting(serialPort_t *instance);
inline uint8_t serialRead(serialPort_t *instance);
inline void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate);
void serialSetMode(serialPort_t *instance, portMode_t mode);
inline bool isSerialTransmitBufferEmpty(serialPort_t *instance);
void serialPrint(serialPort_t *instance, const char *str);
uint32_t serialGetBaudRate(serialPort_t *instance);
