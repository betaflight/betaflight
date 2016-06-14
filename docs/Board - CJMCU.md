# Board - CJMCU

The CJMCU is a tiny (80mm) board running a STM32F103, which contains a 3-Axis Compass (HMC5883L)
and an Accelerometer/Gyro (MPU6050).

This board does not have an onboard USB-Serial converter, so an external adapter is needed.

# Hardware revisions

| Revision | Notes |
| -------- | ----- |
| 1        | No boot jumper pads by LED1. Uses blue and red LEDs |
| 2        | Boot jumper pads presoldered with pins and a jumper by LED1. Uses green and red LEDs. |

Version 2 boards are supported from firmware v1.4.0 onwards, do NOT flash earlier versions to version 2 boards.

# Pins

## RX Connections

| Pin Label | Description              |
| --------- | ------------------------ |
| PA0       | RC Channel 1             |
| PA1       | RC Channel 2             |
| PA2       | RC Channel 3 / USART2 TX |
| PA3       | RC Channel 4 / USART2 RX |
| VCC       | Power (See note)         |
| GND       | Ground                   |

NOTE: The VCC RX Pin is not regulated and will supply what ever voltage is provided to the board, this will mean it'll provide 5v if a 5v serial connection is used. Be careful if you are using a voltage sensitive RX. A regulated 3.3v supply can be found on the top pin of column 1, just below the RX GND pin.

## Serial Connections

USART1 (along with power) is on the following pins.

| Pin Label | Description     |
| --------- | --------------- |
| TX1       | UART1 TX        |
| RX1       | UART2 RX        |
| GND       | Ground          |
| 3V3       | Power +3.3v     |
| 5V        | Power +5v       |

USART2 is the following pins.

| Pin Label | Description |
| --------- | ----------- |
| PA2       | USART2 TX   |
| PA3       | USART2 RX   |


## Power Connections

| Pin Label | Description             |
| --------- | ----------------------- |
| Power +   | Power - 1 Cell 3.7v Max |
| Power -   | Ground                  |

## Motor Connections

In standard QUADX configuration, the motors are mapped:

| INAV | CJMCU  |
| ----------- | ------ |
| Motor 1     | Motor3 |
| Motor 2     | Motor2 |
| Motor 3     | Motor4 |
| Motor 4     | Motor1 |

It is therefore simplest to wire the motors:
 * Motor 1 -> Clockwise
 * Motor 2 -> Anti-Clockwise
 * Motor 3 -> Clockwise
 * Motor 4 -> Anti-Clockwise

If you are using the Hubsan x4/Ladybird motors, clockwise are Blue (GND) / Red (VCC) wires, anticlockwise
are Black (GND) / White (VCC).
i.e. there is one wire on each motor out of the standard RED/BLACK VCC/GND polarity colors that can be used to identify polarity.

If you have wired as above, Motor1/Motor2 on the board will be forward.

# Connecting a Serial-USB Adapter

You will need a USB -> Serial UART adapter. Connect:

| Adapter           | CJMCU                      |
| ----------------- | -------------------------- |
| Either 3.3v OR 5v | The correct 3.3v OR 5v pin |
| RX                | TX                         |
| TX                | RX                         |

When first connected this should power up the board, and will be in bootloader mode. If this does not happen, check
the charge switch is set to POW.
After the flashing process has been completed, this will allow access via the INAV configurator to change 
settings or flash a new firmware.

WARNING: If the motors are connected and the board boots into the bootloader, they will start
to spin after around 20 seconds, it is recommended not to connect the motors until the board
is flashed.

# Flashing

To flash the board:
 * Open INAV Configurator
 * Choose the latest CJMCU firmware from the list.
 * Select "Load Firmware [Online]" and wait for the firmware to download.
 * Tick "No Reboot Sequence" and "Full Chip Erase"
 * Connect the USB->Serial adapter to the board
 * Select the USB-UART adapter from the top left box
 * Click "Flash Firmware"
 * You should see "Programming: SUCCESSFUL" in the log box
 * Click "Connect" -> This should open the "Initial Setup" tab and you should see sensor data from the quad shown
 * Unplug the quad and short the 2 "BOOT0" pins. Revision 1 boards require this to be soldered, revision 2 boards can connect the included jumper to the two pre-soldered pins - This prevents the board from going into bootloader mode on next
   boot, if anything goes wrong, simply disconnect these two pins and the bootloader will start, allowing you to reflash. You cannot
   overwrite the bootloader.

# Charging

The CJMCU has on it a TP4056 Lithium battery charging IC that can charge a 1S battery at 1A using a provided 5v supply attached to the 5v serial pin.

To charge an attached battery:
 * Set the power switch to OFF
 * Set the charge switch to CHG
 * Plug in a 1S battery to the battery pins
 * Plug in a 5v supply to the 5v serial pins

The charger will finish when either the battery reaches 4.2v, or the battery's voltage is greater than the charger's input voltage.

The two nearby LEDs will show the status of charging:

| Status             | Green LED | Red LED   |
|--------------------|-----------|-----------|
| Charging           | On        | Off       |
| Finished           | Off       | On        |
| 5v not connected   | Off       | Off       |
| Batt not connected | Flashing  | On        |


# Helpful Hints

 * If you are only using a 4 channel RX, in the auxiliary configuration tab, you can add a "Horizon" mode range around 1500
 for one of the the AUX channels which will result in it being always on
 * Enabling the feature MOTOR_STOP helps with crashes so it doesn't try to keep spinning on its back
 * When the power runs low, the quad will start jumping around a bit, if the flight behaviour seems strange, check your batteries charge
