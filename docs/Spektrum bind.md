# Spektrum bind support

Spektrum bind with hardware bind plug support.
 
The Spektrum bind code is actually enabled for the NAZE, NAZE32PRO, CJMCU, EUSTM32F103RC, SPARKY, CC3D, ALIENWIIF1, ALIENWIIF3 targets.

## Configure the bind code

The following parameters can be used to enable and configure this in the related target.h file:

    SPEKTRUM_BIND          Enables the Spektrum bind code
    BIND_PORT  GPIOA       Defines the port for the bind pin
    BIND_PIN   Pin_3       Defines the bind pin (the satellite receiver is connected to)

This is to activate the hardware bind plug feature

    HARDWARE_BIND_PLUG     Enables the hardware bind plug feature
    BINDPLUG_PORT  GPIOB   Defines the port for the hardware bind plug
    BINDPLUG_PIN   Pin_5   Defines the hardware bind plug pin

## Hardware

The hardware bind plug will be enabled via defining HARDWARE_BIND_PLUG during building of the firmware. BINDPLUG_PORT and BINDPLUG_PIN also need to be defined (please see above). This is done automatically if the AlienWii32 firmware is built. The hardware bind plug is expected between the defined bind pin and ground. 

## Function

The bind code will actually work for NAZE, NAZE32PRO, CJMCU, EUSTM32F103RC, SPARKY targets (USART2) and CC3D target (USART3, flex port). The spektrum_sat_bind CLI parameter is defining the number of bind impulses (1-10) send to the satellite receiver. Setting spektrum_sat_bind to zero will disable the bind mode in any case. The bind mode will only be activated after an power on or hard reset. Please refer to the table below for the different possible values.

If the hardware bind plug is configured the bind mode will only be activated if the plug is set during the firmware start-up. The value of the spektrum_sat_bind parameter will be permanently preserved. The bind plug should be always removed for normal flying.

If no hardware bind plug is used the spektrum_sat_bind parameter will trigger the bind process during the next hardware reset and will be automatically reset to "0" after this.

Please refer to the satellite receiver documentation for more details of the specific receiver in bind mode. Usually the bind mode will be indicated with some flashing LEDs.

## Table with spektrum_sat_bind parameter value

| Value | Receiver mode     | Notes              |
| ----- | ------------------| -------------------|
| 3     | DSM2 1024bit/22ms |                    |
| 5     | DSM2 2048bit/11ms | default AlienWii32 |
| 7     | DSMX 1024bit/22ms |                    |
| 9     | DSMX 2048bit/11ms |                    |

More detailed information regarding the satellite binding process can be found here:
http://wiki.openpilot.org/display/Doc/Spektrum+Satellite

### Supported Hardware

NAZE, NAZE32PRO, CJMCU, SPARKY, EUSTM32F103RC, CC3D targets and ALIENWIIF1, ALIENWIIF3 targets with hardware bind plug

### Connecting a Spektrum-compatible satellite to a Flip32+ flight controller

The Flip32/Flip32+ is wired in a rather strange way, i.e. the dedicated connector for the satellite module uses the same UART pins as the USB adapter. This means that you can't use that connector as it maps to UART1 which you really shouldn't assign to SERIAL_RX as that will break USB functionality. (Looks this problem is fixed in later versions of the Flip32/Flip32+)

In order to connect the satellite to a Flip32+, you have to wire the serial data pin to RC_CH4. This is the fourth pin from the top in the left column of the 3x6 header on the right side of the board. GND and +3.3V may either be obtained from the dedicated SAT connector or from any ground pin and pin 1 of the BOOT connector which also provides 3.3V.

#### Tested satellite transmitter combinations

| Satellite            | Remote         | Remark                                                   |
| -------------------- | -------------- | -------------------------------------------------------- |
| Orange R100          | Spektrum DX6i  | Bind value 3                                             |
| Lemon RX DSM2/DSMX   | Spektrum DX8   | Bind value 5                                             |
| Lemon RX DSMX        | Walkera Devo10 | Bind value 9, Deviation firmware 4.01 up to 12 channels  |
| Lemon RX DSM2        | Walkera Devo7  | Bind value 9, Deviation firmware                         |
