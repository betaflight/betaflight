# Board - RMRC Dodo

The RMRC Dodo board is made and sold by Ready Made RC. In terms of CPU pin mapping it is a clone of the SPRacingF3 board (also see the SPRacingF3 documentation). Currently there are three versions with slightly different hardware available: Rev. 1, 2, and 3.

The revision 3 board changed the CPU from one with 128KB to one with 256KB however, for compatibility reasons, Cleanflight only supports and uses 128KB of it.  The same binary can be used on all DODO boards.

## Hardware Features

* STM32 F3 ARM Cortex-M processor with 128KB or 256KB flash program memory (Rev. 1 and 2: 128KB flash, Rev 3 256K).
* 2MB external flash memory for storage.
* MPU6050 accelerometer/gyro (Rev. 2: MPU6000)
* BMP280 Barometer
* No compass sensor.
* 3 hardware UARTs (+ 2 software) for GPS, telemetry, OSD, etc., 5V compatible
* On-board 5V/0.5A BEC directly powered off 2-6S main battery pack. No external BEC/regulator necessary.
* On-board 3.3V regulator to power peripherials like Spektrum satellite receiver.
* 36x36mm board with 30.5mm mounting holes pattern (same as CC3D or Naze32, but different pin-out).

Note: Earlier revisions had issues with the buzzer circuit. 

## Serial Ports

| Name  | RX (Board) | TX (Board) | RX (MCU) | TX (MCU) | Notes                                        |
|:-----:|:----------:|:----------:|:--------:|:--------:|----------------------------------------------|
| UART1 | U1RX       | U1TX       | PA10     | PA9      | Shared with USB connection                   |
| UART2 | U2RX       | U2TX/SWCLK | PA15     | PA14     | Shared with SWD                              |
| UART3 | U3RX/3     | U3TX/4     | PB11     | PB10     | RX also connected to Spektrum satellite port |

## Pinouts

### Starboard (Front to back)

| Label                  | Notes                                   |
|:----------------------:|:---------------------------------------:|
| RC_IN_8/SOFTSERIAL2_TX | RC8 ppm input or soft-serial 2 transmit |
| RC_IN_7/SOFTSERIAL2_RX | RC7 ppm input or soft-serial 2 receive  |
| RC_IN_6/SOFTSERIAL1_TX | RC6 ppm input or soft-serial 1 transmit |
| RC_IN_5/SOFTSERIAL1_RX | RC5 ppm input or soft-serial 2 receive  |
| RC_IN_4/U3TX           | RC4 ppm input or UART3 transmit         |
| RC_IN_3/U3RX           | RC3 ppm input or UART3 receive          |
| RC_IN_2                | RC2 ppm input                           |
| RC_IN_1/PPM IN         | RC1 ppm input or SUM/CPPM               |
| 5V                     | 5V bus                                  |
| GND                    | Ground                                  |

### Back (Left to right)

| Label              | Notes                                |
|:------------------:|:------------------------------------:|
| SPEKTRUM_VCC/3V3   | 3.3V output                          |
| SPEKTRUM_GND/GND   | Ground                               |
| SPEKTRUM_DATA/U3RX | UART3 receive                        |
| USB                | Micro USB socket                     |
| BAT+               | Main battery (2S - 6S) positive lead |
| GND/BAT-           | Main battery negative lead           |

### Front (Left to right)

| Label       | Notes                                                             |
|:-----------:|:-----------------------------------------------------------------:|
| BZ_5V/BZ+   | 5V, connect to positive (+) terminal of buzzer                    |
| BZ_OC/BZ-   | Open-collector output, connect to negative (-) terminal of buzzer |
| RC_OUT_1    | ESC 1 output (rear, starboard, CW on quad)                        | 
| RC_OUT_2    | ESC 2 output (front, starboard, CCW on quad)                      |
| RC_OUT_3    | ESC 3 output (front, port, CW on quad)                            |
| RC_OUT_4    | ESC 4 output (rear, port, CCW on quad)                            |
| RC_OUT_5    | PPM output 5                                                      |
| RC_OUT_6    | PPM output 6                                                      |
| RC_OUT_7    | PPM output 7                                                      |
| RC_OUT_8    | PPM output 8                                                      |
| LED_OUT     | WS2811 led output                                                 |
| LED_GND/GND | Ground                                                            |

### Starboard (Front to back)

| Label      | Column  | Notes                                                   |
|:----------:|:-------:|:-------------------------------------------------------:|
| GND        | Outside | Ground                                                  |
| U1TX       | Outside | UART1 transmit (1)                                      |
| U1RX       | Outside | UART1 receive (1)                                       |
| 5V         | Outside | 5V bus                                                  |
| 3V3        | Outside | 3.3V output (2)                                         |
| GND        | Inside  | Ground                                                  |
| SDA        | Inside  | I2C data                                                |
| SCL        | Inside  | I2C clock                                               |
| 3V3        | Inside  | 3.3V output                                             |
| PPM_RSSI   | Inside  | Low-pass filtered PPM-style RSSI input (FrSky or EzUHF) |
| U2RX       | Outside | UART2 receive                                           |
| U2TX/SWCLK | Outside | UART2 transmit or SWD clock                             |
| SWDIO      | Outside | SWD input/output                                        |
| GND        | Outside | Ground                                                  |

1. Shared with USB.
2. Can be used to power external low-power devices like Spektrum satellite receiver.

### Top pads

| Label | Notes                                                        |
|:-----:|:------------------------------------------------------------:|
| CURR  | Current sensor ADC input, unbuffered                         |
| RSSI  | RSSI ADC input, unbuffered. Use PPM_RSSI for filtered input. |
| BOOT  | BOOT jumper for recovering a bricked boot loader             |

### Bottom pads

| Label | Notes                                                                   |
|:-----:|:------------------------------------------------------------------------|
| TRIG  | Trigger for sonar (on-board 1kOhm series resistor for 5V compatibility) |
| ECHO  | Echo for sonar (on-board 1kOhm series resistor for 5V compatibility)    |




