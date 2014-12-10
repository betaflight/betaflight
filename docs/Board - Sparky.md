# Board - Sparky

The Sparky is a very low cost and very powerful board.

* 3 hardware serial ports.
* USB (can be used at the same time as the serial ports).
* 10 PWM outputs.
* Dedicated PPM/SerialRX input pin.
* MPU9150 I2C Acc/Gyro/Mag
* Baro

# Status

Flyable!

Tested with revision 1 board. 

## TODO
* Mag
* Baro
* Led Strip
* ADC
* Display
* Softserial - though having 3 hardware serial ports makes it a little redundant.

# Flashing

## Via USART1

Short the bootloader pads and flash using configurator or the st flashloader tool via USART1.
Unshort bootloader pads after flashing.

## Via SWD

On the bottom of the board there is an SWD header socket onto switch a JST-SH connector can be soldered.
Once you have SWD connected you can use the st-link or j-link tools to flash a binary.

See Sparky schematic for CONN2 pinouts.

## TauLabs bootloader

Flashing cleanflight will erase the TauLabs bootloader, this is not a problem and can easily be restored using the st flashloader tool.

# Serial Ports

| Value | Identifier   | RX        | TX         | Notes                                                                                       |
| ----- | ------------ | --------- | ---------- | ------------------------------------------------------------------------------------------- |
| 1     | USB VCP      | RX (USB)  | TX (USB)   |  |
| 2     | USART1       | RX / PB7  | TX / PB6   | Conn1 / Flexi Port |
| 3     | USART2       | RX / PA3  | PWM6 / PA2 | On RX is on INPUT header.  Best port for Serial RX input. |
| 4     | USART3       | RX / PB11 | TX / PB10  | RX/TX is on one end of the 6-pin header about the PWM outputs. |

USB VSP *can* be used at the same time as other serial ports (unlike Naze32).

