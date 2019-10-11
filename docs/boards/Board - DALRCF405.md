# Board - DALRCF405

The DALRCF405  described here:
http://www.dalrcmodel.com/DALRC/plus/view.php?aid=184

This board use the STM32F405RGT6 microcontroller and have the following features:
* 1024K bytes of flash memory,192K bytes RAM,168 MHz CPU/210 DMIPS

* The 16M byte SPI flash for data logging
* USB VCP and boot select button on board(for DFU)
* Stable voltage regulation,9V/2A DCDC BEC for VTX/camera etc.And could select 5v/9v with pad
* Serial LED interface(LED_STRIP)
* VBAT/CURR/RSSI sensors input
* Suppose IRC Tramp/smart audio/FPV Camera Control/FPORT/telemetry
* Supports SBus, Spektrum1024/2048, PPM. No external inverters required (built-in).
* Supports I2C device extend(baro/compass/OLED etc)
* Supports GPS 

### All uarts have pad on board 
| Value | Identifier   | RX   | TX   | Notes                                                                                       |
| ----- | ------------ | -----| -----| ------------------------------------------------------------------------------------------- |
| 1     | USART1       | PB7  |  PA9 |  PB7 FOR SBUS IN(inverter build in)                                                         |
| 2     | USART2       | PA3  |  PA2 |  PAD USE FOR TRAMP/smart audio                                                              |
| 3     | USART3       | PB11 |  PB10|  USE FOR GPS                                                                                |
| 4     | USART4       | PA1  |  PA0 |  PA0 FOR RSSI/FPORT/TEL etc                                                                 |
| 5     | USART5       | PD2  |  PC12|  PAD                                                                                        |


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
| 1     | ADC1         |    VBAT   |  PC2  |  DMA2_Stream0
| 2     | ADC1         |    CURR   |  PC1  |  DMA2_Stream0
| 3     | ADC1         |    RSSI   |  PA0  |  DMA2_Stream0
| 4     | ADC1         |    extend |  PC0  |  DMA2_Stream0 extend for other senser(PAD)


### 8 Outputs, 1 PPM input 
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | TIM12_CH2    |    PPM    |  PB15 |  PPM
| 2     | TIM3_CH3     |    OUPUT1 |  PB0  |  DMA1_Stream7
| 3     | TIM8_CH1     |    OUPUT2 |  PC6  |  DMA2_Stream2
| 4     | TIM1_CH3     |    OUPUT3 |  PA10 |  DMA2_Stream6
| 5     | TIM1_CH1     |    OUPUT4 |  PA8  |  DMA2_Stream1
| 6     | TIM8_CH3     |    OUPUT5 |  PC8  |  DMA2_Stream4
| 7     | TIM3_CH4     |    OUPUT6 |  PB1  |  DMA1_Stream2
| 8     | TIM3_CH2     |    OUPUT7 |  PC7  |  DMA1_Stream5   NO PAD
| 9     | TIM8_CH4     |    OUPUT8 |  PC9  |  DMA2_Stream7   NO PAD
| 10    | TIM4_CH1     |    PWM    |  PB6  |  DMA1_Stream0   LED_STRIP
| 11    | TIM2_CH1     |    PWM    |  PA5  |  FPV Camera Control(FCAM)


### Gyro & ACC ,suppose ICM20689/MPU6000
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI1         |    SCK   |  PB3   | 
| 2     | SPI1         |    MISO  |  PA6   | 
| 3     | SPI1         |    MOSI  |  PA7   | 
| 4     | SPI1         |    CS    |  PC4   | 

### OSD MAX7456
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI3         |    SCK   |  PC10  | 
| 2     | SPI3         |    MISO  |  PC11  | 
| 3     | SPI3         |    MOSI  |  PB5   | 
| 4     | SPI3         |    CS    |  PA15  |

### 16Mbyte flash
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI2         |    SCK   |  PB13  | 
| 2     | SPI2         |    MISO  |  PB14  | 
| 3     | SPI2         |    MOSI  |  PC3   | 
| 4     | SPI2         |    CS    |  PB12  | 

### SWD
| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | SWCLK          | PAD                                          |
| 2   | Ground         | PAD                                          |
| 3   | SWDIO          | PAD                                          |
| 4   | 3V3            | PAD                                          |

###Designers
* nyway &DALRC





