# Spektrum bind support

Spektrum bind with hardware bind plug support.
 
## Hardware

The hardware bind plug will be enabled via defining HARDWARE_BIND_PLUG during building of the firmware. This is done automatically if the AlienWii32 firmware is build. The bind plug is expected between pin 41 (PB5 STM32F103CBT6) and ground. The bind code will actually work for USART2 (PA3 STM32F103CBT6).

## Function

If the bind plug is set the bind mode will be permanently activated during the firmware start-up. The spektrum_sat_bind CLI parameter is defining the number of bind impulses (1-10) send to the satellite receiver. Please refer to the table below. If there is no hardware bind plug present and the spektrum_sat_bind paremeter is set the bind code will be executed one time only after the next start of the firmware (hardware reset or power on). The spektrum_sat_bind parameter will be reset to 0 when done. The bind code will not be executed during an soft rest of the MCU. Please refer to the sattelite receiver documentation for more details of the specific receiver in bind mode. Usually the bind mode will be indicated with some flashing LED’s. The Bind plug should be always removed for normal flying.

## Table with spektrum_sat_bind parameter Value

| Value | Receiver mode  | Notes   |
| ----- | ---------------| --------|
| 3     | DSM2 1024/22ms |         |
| 5     | DSM2 2048/11ms | default |
| 7     | DSMX 22ms      |         |
| 9     | DSMX 11ms      |         |

More detailed information regarding the sattelite binding process can be found here:
http://wiki.openpilot.org/display/Doc/Spektrum+Satellite

### Supported Hardware

all NAZE targets (NAZE32, MW32, Flip32, AlienWii32 with hardware bind pin)
