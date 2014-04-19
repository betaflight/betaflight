#pragma once

typedef struct serialPorts_s {
    serialPort_t *mainport;
    serialPort_t *gpsport;
    serialPort_t *telemport;
    serialPort_t *rcvrport;
} serialPorts_t;

typedef struct serialConfig_s {
    uint32_t port1_baudrate;

    uint32_t softserial_baudrate;             // shared by both soft serial ports
    uint8_t softserial_1_inverted;            // use inverted softserial input and output signals on port 1
    uint8_t softserial_2_inverted;            // use inverted softserial input and output signals on port 2
    uint8_t reboot_character;               // which byte is used to reboot. Default 'R', could be changed carefully to something else.
} serialConfig_t;

extern serialPorts_t serialPorts;

void resetMainSerialPort(void);
void openMainSerialPort(uint32_t baudrate);
void evaluateOtherData(uint8_t sr);
void handleSerial(void);
