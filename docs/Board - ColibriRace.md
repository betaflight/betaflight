# Board - Colibri RACE

The Colibri RACE is a STM32F3 based flight control designed specifically to work with the TBS POWERCUBE multi rotor stack. 

## Hardware Features:
* STM32F303 based chipset for ultimate performance
* PPM, SBUS, DSM, DSMX input (5V and 3.3V provided over internal BUS). No inverters or hacks needed.
* 6 PWM ESC output channels (autoconnect, internal BUS)
	* RGB LED strip support incl. power management
	* Extension port for GPS / external compass / pressure sensor
	* UART port for peripherals (Blackbox, FrSky telemetry etc.)
	* Choose between plug & play sockets or solder pads for R/C and buzzer
	* 5V buzzer output
	* MPU6500 new generation accelerometer/gyro
	* 3x status LED (DCDC pwr/ 3.3V pwr/ status)
	* Battery monitoring for 12V, 5V and VBat supply
	* Size: 36mmx36mm (30.5mm standard raster)
	* Weight: 4.4g

	For more details please visit:
	http://www.team-blacksheep.com/powercube

## Serial Ports

	| Value | Identifier   | Board Markings | Notes                                     |
	| ----- | ------------ | -------------- | ------------------------------------------|
	| 1     | VCP          | USB PORT       | Main Port For MSP                         |
	| 2     | USART1       | FREE PORT      | PC4 and PC5(Tx and Rx respectively)       |
	| 3     | USART2       | PPM Serial     | PA15                                      |
	| 4     | USART3       | GPS PORT       | PB10 and PB11(Tx and Rx respectively)     |

## Pinouts

	Full pinout details are available in the manual, here:

	http://www.team-blacksheep.com/colibri_race


### SWD - ICSP

	| Pin | Function       | Notes                                        |
	| --- | -------------- | -------------------------------------------- |
	| 1   | VCC_IN         | 3.3 Volt                                     |
	| 2   | SWDIO          |                                              |
	| 3   | nRESET         |                                              |
	| 4   | SWCLK          |                                              |
	| 5   | Ground         |                                              |
	| 6   | SWO/TDO        |                                              |

### Internal Bus

	| Pin | Function       | Notes                                        |
	| --- | -------------- | -------------------------------------------- |
	| 1   | PWM1           | MOTOR 1                                      |
	| 2   | PWM2           | MOTOR 2                                      |
	| 3   | PWM3           | MOTOR 3                                      |
	| 4   | PWM4           | MOTOR 4                                      |
	| 5   | PWM5           | MOTOR 5 (For Y6 or Hex X)                    |
	| 6   | PWM6           | MOTOR 6 (For Y6 or Hex X)                    |
	| 7   | BST SDA        | Use For TBS CorePro Control Device           |
	| 8   | BST SCL        | Use For TBS CorePro Control Device           |
	| 9   | PWM7           | Can be a normal GPIO (PA2) or PWM            |
	| 10  | PWM8           | Can be a normal GPIO (PA2) or PWM            |
	| 11  | 12.2V DCDC     | If 12v is detected, the Blue LED will turn on|
	| 12  | 5.1V DCDC      | Voltage for MCU                              |

### Servo

	| Pin | Function       | Notes                                        |
	| --- | -------------- | -------------------------------------------- |
	| 1   | Ground         |                                              |
	| 2   | VCC_OUT        | 5.1 Volt output to LCD Strip                 |
	| 3   | PWM Servo      | PB14 - PWM10                                 |

### IO_1 - LED Strip

	| Pin | Function          | Notes                                        |
	| --- | ----------------- | -------------------------------------------- |
	| 1   | LED_STRIP         | Enable `feature LED_STRIP`                   |
	| 2   | VCC_OUT           | 5.1 Volt output to LCD Strip                 |
	| 3   | Ground            |                                              |

### IO_2 - Sensor Interface

	| Pin | Function          | Notes                                        |
	| --- | ----------------- | -------------------------------------------- |
	| 1   | VCC_OUT           | 4.7 Volt output to the device                |
	| 2   | Ground            |                                              |
	| 3   | UART3 TX          | GPS                                          |
	| 4   | UART3 RX          | GPS                                          |
	| 5   | SDA               | mag, pressure, or other i2c device           |
	| 6   | SCL               | mag, pressure, or other i2c device           |

### IO_3 - RC input

	IO_3 is used for RX_PPM/RX_SERIAL. Under the `PORT` tab, set RX_SERIAL to UART2 when using RX_SERIAL.

	| Pin | Function          | Notes                                        |
	| --- | ----------------- | -------------------------------------------- |
	| 1   | PPM/Serial        | Can PPM or Serial input                      |
	| 2   | VCC_OUT           | 3.3 Volt output to the device                |
	| 3   | Ground            |                                              |
	| 4   | VCC_OUT           | 5.1 Volt output to the device                |

### IO_4 - Buzzer

	| Pin | Function          | Notes                                        |
	| --- | ----------------- | -------------------------------------------- |
	| 1   | BUZZER            | Normal high (5.1v)                           |
	| 2   | VCC_OUT           | 5.1 Volt output to the device                |

### IO_5 - Free UART

	| Pin | Function          | Notes                                        |
	| --- | ----------------- | -------------------------------------------- |
	| 1   | UART1 TX          | Free UART                                    |
	| 2   | UART1 RX          | Free UART                                    |
	| 3   | Ground            |                                              |
	| 4   | VCC_OUT           | 4.7 Volt output to the device                |

### IO_6 - IR TX (extension)

	| Pin | Function          | Notes                                        |
	| --- | ----------------- | -------------------------------------------- |
	| 1   | IR TX             |                                              |
	| 2   | Ground            |                                              |
