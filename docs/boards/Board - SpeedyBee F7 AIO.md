# Board - Speedy Bee F7 AIO

## Description
The Speedy Bee F7 AIO is an AIO flight controller that integrates PDB, Bluetooth chip, Barometer, 32MB OnBoard Flash (Used for BlackBox), 
and users can adjust the parameters of the flight controller using the Speedy Bee App with an integrated Bluetooth chip.
### Hardware Features
* MCU: STM32F722
* IMU: ICM20689
* OSD: BetaFlight OSD w/ AT7456E chip
* BLE Module: inner connect to UART3 for remote setting with SpeedyBee App or other similar apps
* BlackBox: 32MB onboard dataflash
* Current Sensor: 200A(Scale 102)
* BetaFlight Camera Control Pad: Yes
* Power input: 3s - 6s Lipo
* Power output: 5V *5(Including BZ+), the maximum load current is 2.5A. 9V * 1, the maximum load current is 2.5A.
* ESC power output: 4 * VCC output
* UART: UART Pads * 4(UART1, UART2, UART4, UART5)
* RSSI input: RSSI input solder pad
* SmartPort: Use Softserial1 to support SmartPort
* I2C: Used for external Magnetometer, Sonar, etc.
* Buzzer: BZ+ and BZ- pad used for 5V Buzzer
* ESC signal: S1 - S5
* LED pin: Used for WS2812 LED
* Boot button: Used to easy enter DFU mode
* BetaFlight Target: SPEEDYBEEF7