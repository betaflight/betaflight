# Segger J-Link RTT Debugging

Betaflight supports the use of Segger J-Link based debugging.

Real Time Terminal (RTT) support is only enabled if `SEGGER_RTT=YES` is included on the make line.
    
There are two serial streams, numerically defined by `RTT_DEBUG_CHANNEL` for debug in _src/main/build/debug.h_ and `RTT_SERIAL_CHANNEL` for serial comms in _src/main/drivers/serial_rtt.h_.

By default these are both set to 0, however if `RTT_DEBUG_CHANNEL` is set to 1-3, the `JLinkRTTLogger` console application may be used on the host to log the stream independant of any serial driver activity on the `RTT_SERIAL_CHANNEL` stream.
    
If `SEGGER_RTT_CLI=YES` is also defined on the make invocation, the betaflight CLI will be enabled over the `RTT_SERIAL_CHANNEL`. This may be accessed on the host by using `telnet localhost 19021`.
    
The serial channel may be opened using `openRttSerial()` and then treated just like a local UART channel.
    
The debug stream supports both masking of debug sources and control of debug level. Calls to `dbgPrintf()`
take the debug source (taken from `dbgSrc_e`) as the source, and an integer level. A higher level number should be used for more detailed debug. The source may be filtered by setting a suitable logical OR of the required sources using `dbgMask()` and the `DBG_MSK` macro, and the max level of message detail to display may be set using `dbgLevel()`.
    
If `SEGGER_RTT=YES` is not defined, the debug routines are #define'd to be empty so debug statements may be left in the source code without impacting binary size.
