# YuPiF4 by RcNet

An high quality flight controller for the most demanding pilotes.

## Description

The YuPiF4 is a 36x36mm (30.5x30.5 mounting holes) board with an F4 microcontroller.

## MCU, Sensors and Features
### Hardware  
- MCU: STM32F405RTG6  
- IMU: not yet defined  
- IMU Interrupt: Yes  
- BARO: No  
- VCP: Yes  
- Hardware UARTS: 3 with an inverter for SBus and a Smart-Port Pad to solder directly your SPort wire  
- Blackbox: SD card slot  
- PPM : A specific pad is available for PPM input  
- Battery Voltage Sensor: Yes, directly connected, no wiring necessary  
- Integrated Voltage Regulator: the board can be powered by your lipo (2S to 6S)  
- Brushed Motor Mosfets: No  
- Motor outputs : can drive up to 6 motors  
- Buttons: Boot0 to enter in DFU mode### Features  
- Current Sensor: Not implemented  
- BlHeli passthrough: Yes   
- WS2811 Led Strip: Yes (on motor output Pin 5)  
- Transponder: No

### Extension Board
- An extension board with GPS and Baro will be available

## Designers and Maintainers

[RcNet](https://github.com/ted-rcnet) and [FaduF](https://github.com/Faduf)

## Manufacturers and Distributors

This board is currently in the prototype phase. Only small batches were manufactured for testing.Website : http://www.yupif4.com/

![YuPiF4 - Logo](http://www.yupif4.com/imgs/YuPiF4.jpg)
