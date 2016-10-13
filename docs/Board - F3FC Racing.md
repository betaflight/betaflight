# Board - F3FC Racing

Designed by RCExplorer.se

Full details available on the website, here:

http://rcexplorer.se/product/f3fc-racing/

## Hardware Features

* STM32F303CC Processor
* MPU6000 connected via SPI (With interrupt pin connected)
* 3 UARTs
* VCP USB
* 6 PWM channels (PWM 6 used as PPM input when selected)
* Supports SBus, SumH, SumD, Spektrum1024/2048, XBus and PPM receivers. No external inverters required.
* Dedicated RGB LED control pin.
* Dedicated I2C port
* Integrated battery monitoring
* Current sensor input pin
* Buzzer/Beeper port
* Built in 5V BEC (500mA) 
* Push button for entering DFU mode
* 3.3V pad available for spectrum satellite receivers.

## Serial Ports

| Value | Identifier   | RX           | TX           |  Notes                                                                                       |
| ----- | ------------ | ------------ | ------------ | ------------------------------------------------------------------------------------------- |
| 1     | VCP	        | PA11         | PA12         | Mini USB connector |
| 2     | USART1        | PB7          | PB6          | Marked as R1 and T1 on the board |
| 3     | USART2        | PA3          | PA2          | Marked as R2 and T2 on the board  |
| 4     | USART3        | PB11         | PB10         | Marked as R3 and T3 on the board  |

## Pinouts

Full pinout details are available in the manual, here:

http://rcexplorer.se/product/f3fc-racing/

### IOs

| Pad    | Signal         | Notes                                        |
| ------ | -------------- | -------------------------------------------- |
| Isense | PB2            | Analog sensor                                |
| Vbat   | PA5            | Voltage supplied to the built in BEC         |
| LED    | PB8            | RGB LED control                              |
| FB     | PA6            | Servo position FeedBack / RSSI input         |
| BZ-    | PA0            | Buzzer output                                |
| 6      | PA1            | Used as PPM input when selected              |
| SCLK   | PA9            | SCLK I2C                                     |
| SDA    | PA10           | SDA I2C                                      |


PWM’s are marked as 1-6

## Voltage and current monitoring

Voltage is measured on the “Vbat” (which also powers the built in BEC) pads through a voltage divider of 10k/1k connected to PA5.

Isense pad is connected straight to PB2 and expects an analog voltage between 0-3.3V