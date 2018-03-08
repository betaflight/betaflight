#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#define DBG_DEFAULT_BAUDRATE 115200
#define DBG_DEFAULT_PORT SERIAL_PORT_USART3

#ifdef USE_DEBUG_SERIAL
	#define __DBG_SETRATE(x)    dbg_setRate(x)
	#define __DBG_TICK()		dbg_tick()
	#define __DBG_ENABLE()      dbg_enable()
	#define __DBG_DISABLE()		dbg_disable()
	#define __DBG_CLS()			dbg_clear()
	#define	__DBG_ENTER()		dbg_enterFunction(__func__)
	#define	__DBG_LEAVE()		dbg_leaveFunction(__func__)
	#define __DBG_TEXT(x)		dbg_write(x)
    #define __DBG_LINE(x)		dbg_writeLine(x)
	#define __DBG_VALUEF(x, y)  dbg_valuef(x, y)
	#define __printf(x) 		tfp_printf x
	#define __DBG_EVERY(x,y) do {	\
	        __DBG_SETRATE(x); 		\
			y               		\
			__DBG_TICK();     		\
		} while(0);
#else
	#define __DBG_SETRATE(x)
	#define __DBG_TICK()
	#define __DBG_ENABLE()
	#define __DBG_DISABLE()
	#define __DBG_CLS()
	#define	__DBG_ENTER(x)
	#define	__DBG_LEAVE()
    #define __DBG_TEXT(x)
	#define __DBG_LINE(x)
	#define __DBG_VALUEF(x, y)
	#define __printf(x)
	#define __DBG_EVERY(x,y) y
#endif  // USE_DEBUG_SERIAL

void dbg_initialize(serialPortIdentifier_e port_id, uint32_t baudrate);
void dbg_setRate(uint16_t rate);
void dbg_tick(void);
void dbg_clear(void);
void dbg_write(const char* str);
void dbg_writeLine(const char* str);
void dbg_valuef(const char* label, float val);
void dbg_enterFunction(const char* str);
void dbg_leaveFunction(const char* str);
