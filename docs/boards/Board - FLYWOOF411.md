# Board - FLYWOOF411

This board use the STM32F411CEU6 microcontroller and have the following features:

* The 16M byte SPI flash for data logging
* USB VCP and boot select button on board(for DFU)
* Stable voltage regulation,9V/1.5A DCDC BEC for VTX/camera etc.And could select 5v/9v with pad
* Serial LED interface(LED_STRIP)
* VBAT/CURR/RSSI sensors input
* Suppose IRC Tramp/smart audio/FPV Camera Control/FPORT/telemetry
* Supports SBus, Spektrum1024/2048, PPM. No external inverters required (built-in).
* Supports I2C device extend(baro/compass/OLED etc)
* Supports GPS 

### All uarts have pad on board 
| Value | Identifier   | RX   | TX   | Notes                                                                                       |
| ----- | ------------ | -----| -----| ------------------------------------------------------------------------------------------- |
| 1     | USART1       | PB7 |  PB6 | FOR SBUS IN(inverter build in)                                                      |
| 2     | USART2       | PA3 |  PA2|  FOR VTX SM/IRC ETC                                                                                    |


### I2C with GPS port together.Use for BARO or compass etc 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | I2C1         |    SDA   |  PB9   | with GPS outlet
| 2     | I2C1         |    SCL   |  PB8   | with GPS outlet


### Buzzer/LED output 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | LED0         |    LED   |  PC13  | 
| 2     | BEEPER       |    BEE   |  PC14  | 


### VBAT input with 1/10 divider ratio,Current signal input,Analog/digit RSSI input
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | ADC1         |    VBAT   |  PA0  |  DMA2_Stream0
| 2     | ADC1         |    CURR   |  PA1  |  DMA2_Stream0
| 3     | ADC1         |    RSSI   |  PB1  |  DMA2_Stream0


### 8 Outputs, 1 PPM input 
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | TIM9_CH1     |    PPM    |  PA2  |  PPM
| 2     | TIM1_CH1     |    OUPUT1 |  PA8  |  DMA2_Stream1
| 3     | TIM1_CH2     |    OUPUT2 |  PA9  |  DMA2_Stream2
| 4     | TIM1_CH3     |    OUPUT3 |  PA10 |  DMA2_Stream6
| 5     | TIM3_CH3     |    OUPUT4 |  PB0  |  DMA1_Stream7
| 6     | TIM3_CH1     |    OUPUT5 |  PB4  |  DMA1_Stream4
| 7     | TIM3_CH4     |    ANY    |  PB1  |  DMA1_Stream2
| 8     | TIM5_CH4     |    ANY    |  PA3  |  DMA1_Stream3   
| 9     | TIM2_CH3     |    CAM_C  |  PB10 |  DMA1_Stream1   
| 10    | TIM2_CH4     |    LED    |  PB11 |  DMA1_Stream6


### Gyro & ACC  ICM20689
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI1         |    SCK   |  PA5   | 
| 2     | SPI1         |    MISO  |  PA6   | 
| 3     | SPI1         |    MOSI  |  PA7   | 
| 4     | SPI1         |    CS    |  PA4   | 

### OSD MAX7456
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI3         |    SCK   |  PB13  | 
| 2     | SPI3         |    MISO  |  PB14  | 
| 3     | SPI3         |    MOSI  |  PB15   | 
| 4     | SPI3         |    CS    |  PB12  |

### 16Mbyte flash
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI3         |    SCK   |  PB13  | 
| 2     | SPI3         |    MISO  |  PB14  | 
| 3     | SPI3         |    MOSI  |  PB15   | 
| 4     | SPI3         |    CS    |  PB2  | 

### SWD
| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | SWCLK          | PAD                                          |
| 2   | Ground         | PAD                                          |
| 3   | SWDIO          | PAD                                          |
| 4   | 3V3            | PAD                                          |

* FLYWOO TECH





