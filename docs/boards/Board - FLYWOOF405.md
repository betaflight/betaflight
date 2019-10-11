# Board - FLYWOOF405

This board use the STM32F405RGT6 microcontroller and have the following features:
* 1024K bytes of flash memory,192K bytes RAM,168 MHz CPU/210 DMIPS

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
| 1     | USART1       | PA10 |  PB6 | USE smartport/FPORT/TEL etc                                                      |
| 2     | USART3       | PB11 |  PB10| FOR SBUS IN(inverter build in)                                                          |
| 3     | USART4       | PA1  |  PA0 | PAD USE FOR TRAMP/smart audio                                                                           |
| 4     | USART5       | PD2  |   /  | PAD ESC sensor                                                            |
| 5     | USART6       | PC7  |  PC6 | PAD USE FOR GPS/BLE etc                                                                                        |


### I2C with GPS port together.Use for BARO or compass etc 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | I2C1         |    SDA   |  PB9   | with GPS outlet
| 2     | I2C1         |    SCL   |  PB8   | with GPS outlet


### Buzzer/LED output 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | LED0         |    LED   |  PC14  | 
| 2     | BEEPER       |    BEE   |  PC13  | 


### VBAT input with 1/10 divider ratio,Current signal input,Analog/digit RSSI input
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | ADC1         |    VBAT   |  PC3  |  DMA2_Stream0
| 2     | ADC1         |    CURR   |  PC2  |  DMA2_Stream0
| 3     | ADC1         |    RSSI   |  PC1  |  DMA2_Stream0


### 8 Outputs, 1 PPM input 
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | TIM10_CH1    |    PPM    |  PB7  |  PPM
| 2     | TIM3_CH3     |    OUPUT1 |  PB0  |  DMA1_Stream7
| 3     | TIM3_CH4     |    OUPUT2 |  PB1  |  DMA1_Stream2
| 4     | TIM2_CH4     |    OUPUT3 |  PA3  |  DMA1_Stream6
| 5     | TIM2_CH3     |    OUPUT4 |  PA2  |  DMA1_Stream1
| 6     | TIM3_CH2     |    OUPUT5 |  PB5  |  DMA1_Stream5
| 7     | TIM4_CH2     |    OUPUT6 |  PB7  |  DMA1_Stream3
| 8     | TIM8_CH4     |    OUPUT7 |  PC9  |  DMA2_Stream7   
| 9     | TIM3_CH1     |    OUPUT8 |  PB4  |  DMA1_Stream4   
| 10    | TIM8_CH3     |    LED    |  PC8  |  DMA2_Stream2   LED_STRIP
| 11    | TIM1_CH2     |    PWM    |  PA9  |  FPV Camera Control(FCAM)


### Gyro & ACC  ICM20689
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI1         |    SCK   |  PA5   | 
| 2     | SPI1         |    MISO  |  PA6   | 
| 3     | SPI1         |    MOSI  |  PA7   | 
| 4     | SPI1         |    CS    |  PC4   | 

### OSD MAX7456
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI3         |    SCK   |  PC10  | 
| 2     | SPI3         |    MISO  |  PC11  | 
| 3     | SPI3         |    MOSI  |  PC12   | 
| 4     | SPI3         |    CS    |  PB14  |

### 16Mbyte flash
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI3         |    SCK   |  PC10  | 
| 2     | SPI3         |    MISO  |  PC11  | 
| 3     | SPI3         |    MOSI  |  PC12   | 
| 4     | SPI3         |    CS    |  PB3  | 

### SWD
| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | SWCLK          | PAD                                          |
| 2   | Ground         | PAD                                          |
| 3   | SWDIO          | PAD                                          |
| 4   | 3V3            | PAD                                          |

* FLYWOO TECH





