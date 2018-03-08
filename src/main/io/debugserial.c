
#include "platform.h"

#ifdef USE_DEBUG_SERIAL

#include "io/serial.h"
#include "common/printf.h"
#include "io/debugserial.h"

#define INDENTATION_CHAR " "
#define INDENTATION_LENGTH 3
#define CLEAR_SEQUENCE "%s", "\x1b[2J\x0"
#define INITIALIZATION_MESSAGE "DebugSerial_Initialize: OK\r\n"
#define CHECK_ENABLED if(!debugSerial.enabled) return

typedef struct debugSerial_s {
    serialPort_t *port;
    uint8_t tabs;
    bool enabled;
    uint16_t count;
    uint16_t rate;
} debugSerial_t;

debugSerial_t debugSerial = {0};

void dbg_enable(void);
inline void dbg_disable(void);
static void dbg_indent(void);
static inline void dbg_addIndent(void);
static inline void dbg_subIndent(void);

void dbg_initialize(serialPortIdentifier_e port_id, uint32_t baudrate) {
    debugSerial.port = openSerialPort(port_id, FUNCTION_NONE, NULL, NULL, baudrate, MODE_RXTX, 0);
    if (debugSerial.port) {
        setPrintfSerialPort(debugSerial.port);
        debugSerial.enabled = true;
        __printf((CLEAR_SEQUENCE));
        __printf((INITIALIZATION_MESSAGE));
    }
}

void dbg_setRate(uint16_t rate) {
    // When called from periodic tasks, only first hit will set the rate, then it will be no op.
    if(debugSerial.rate == 0) {
        debugSerial.rate = rate;
        debugSerial.count = 0;
        debugSerial.enabled = false;
    }
}

void dbg_tick(void) {
    debugSerial.count++;
    if(debugSerial.count == debugSerial.rate) {
        debugSerial.enabled = true;
    }

    if(debugSerial.count > debugSerial.rate) {
        debugSerial.count = 0;
        debugSerial.enabled = false;
    }
}

void dbg_enable(void) { 
    debugSerial.rate = 0;
    debugSerial.count = 0;
    debugSerial.enabled = true;
}

inline void dbg_disable(void) {
    debugSerial.enabled = false;
}

void dbg_clear(void) {
    CHECK_ENABLED;
    __printf((CLEAR_SEQUENCE));
}

void dbg_write(const char* str) {
    CHECK_ENABLED;
    __printf((str));
}

void dbg_writeLine(const char* str) {
    CHECK_ENABLED;
    dbg_indent();
    __printf(("%s\r\n", str));
}

void dbg_valuef(const char* label, float val) {
    CHECK_ENABLED;
    int i = (int)val;
    int d = (int)((val - i) * 1000);
    dbg_indent();
    __printf(("%s: %d.%d\r\n", label, i, d));
}

void dbg_enterFunction(const char* str) {
    CHECK_ENABLED;
    dbg_indent();
    __printf(("+%s\r\n", str));
    dbg_addIndent();
}

void dbg_leaveFunction(const char* str) {
    CHECK_ENABLED;
    dbg_subIndent();
    dbg_indent();
    __printf(("-%s\r\n", str));
}

void dbg_indent(void) {
    for(uint8_t i=0; i< debugSerial.tabs; i++) {
        __printf((INDENTATION_CHAR));
    }
}

inline void dbg_addIndent(void) {
    debugSerial.tabs += INDENTATION_LENGTH; 
}

inline void dbg_subIndent(void) {
    debugSerial.tabs -= INDENTATION_LENGTH; 
}

#endif // USE_DEBUG_SERIAL
