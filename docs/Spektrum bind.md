# Spektrum bind support

Spektrum bind with hardware bind plug support.
 
The Spektrum bind code is actually enabled for the NAZE, NAZE32PRO, CJMCU, EUSTM32F103RC, SPARKY, CC3D targets.

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

The hardware bind plug will be enabled via defining HARDWARE_BIND_PLUG during building of the firmware. BINDPLUG_PORT and BINDPLUG_PIN also need to be defined (please see above). This is done automatically if the AlienWii32 firmware is build. The hardware bind plug is expected between the defined bind pin and ground. 

## Function

The bind code will actually work for NAZE, NAZE32PRO, CJMCU, EUSTM32F103RC, SPARKY targets (USART2) and CC3D target (USART3, flex port). The spektrum_sat_bind CLI parameter is defining the number of bind impulses (1-10) send to the satellite receiver. Setting spektrum_sat_bind to zero will disable the bind mode in any case. The bind mode will only be activated after an power on or hard reset. Please refer to the table below for the different possible values.

If the hardware bind plug is configured the bind mode will only be activated if the plug is set during the firmware start-up. The value of the spektrum_sat_bind parameter will be permanently preserved. The bind plug should be always removed for normal flying.

If no hardware bind plug is used the spektrum_sat_bind parameter will trigger the bind process during the next hardware reset and will be automatically reset to "0" after this.

Please refer to the satellite receiver documentation for more details of the specific receiver in bind mode. Usually the bind mode will be indicated with some flashing LEDs.

## Table with spektrum_sat_bind parameter value

| Value | Receiver mode  | Notes              |
| ----- | ---------------| -------------------|
| 3     | DSM2 1024/22ms |                    |
| 5     | DSM2 2048/11ms | default AlienWii32 |
| 7     | DSMX 22ms      |                    |
| 9     | DSMX 11ms      |                    |

More detailed information regarding the satellite binding process can be found here:
http://wiki.openpilot.org/display/Doc/Spektrum+Satellite

### Supported Hardware

NAZE, NAZE32PRO, CJMCU, SPARKY, EUSTM32F103RC, CC3D targets (AlienWii32 with hardware bind plug)
