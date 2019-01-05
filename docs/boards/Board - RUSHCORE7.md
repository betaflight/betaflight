# Board - RUSHCORE7

The RUSHCORE7 described here:

This board use the STM32F722RET6 microcontroller and have the following features:
* High-performance and DSP with FPU, ARM Cortex-M7 MCU with 512 Kbytes Flash 
* 216 MHz CPU,462 DMIPS/2.14 DMIPS/MHz (Dhrystone 2.1) and DSP instructions, Art Accelerator, L1 cache, SDRAM
* Support MPU6000 or ICM20602
* OSD on board
* The 16M byte SPI flash on board for data logging
* USB VCP and boot button on board(for DFU)
* BEC 5v/2A on board
* Serial LED (LED_STRIP)
* VBAT/CURR/RSSI sensors input
* Suppose IRC Tramp/smart audio/FPV Camera Control/FPORT/telemetry
* Supports SBus, Spektrum1024/2048, PPM etc
* Supports I2C device extend(baro/compass/OLED etc)
* Supports GPS 
* More about: www.rushfpv.com

### Uarts
| Value | Identifier   | RX   | TX   | Notes                                                                                       |
| ----- | ------------ | -----| -----| ------------------------------------------------------------------------------------------- |
| 1     | USART1       | PB7  |  PB6 |                                                                                             |
| 2     | USART2       | PA3  |  PA2 | FOR SBUS IN(inverter build in)/PPM                                                          |
| 3     | USART3       | PC11 |  PC10|                                                                                             |
| 4     | USART4       | PA1  |  PA0 | PA0 FOR RSSI/FPORT/TEL etc                                                                  |
| 5     | USART5       | PD2  |  PC12| PC12 TRAMP/smart audio                                                                      |


### I2C  
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | I2C1         |    SDA   |  PB9   | 
| 2     | I2C1         |    SCL   |  PB8   | 


### Buzzer/LED output 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | LED0         |    LED   |  PC13  |On board
| 2     | Buzzer       |    BEE   |  PB1   | 


### VBAT input with 1/10 divider ratio,Current signal input,Analog/digit RSSI input
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | ADC1         |    VBAT   |  PC1  |  
| 2     | ADC1         |    CURR   |  PC3  |  
| 3     | ADC1         |    RSSI   |  PA0  |   


### 9 Outputs 
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | TIM2_CH3     |    PPM    |  PA2  |  PPM/SBUS
| 2     | TIM8_CH3     |    OUPUT1 |  PC8  |  DMA
| 3     | TIM8_CH1     |    OUPUT2 |  PC6  |  DMA
| 4     | TIM8_CH4     |    OUPUT3 |  PC9  |  DMA
| 5     | TIM8_CH2     |    OUPUT4 |  PC7  |  DMA
| 6     | TIM1_CH1     |    OUPUT5 |  PA8  |  DMA
| 7     | TIM1_CH2     |    OUPUT6 |  PA9  |  DMA  
| 8     | TIM2_CH4     |    PWM    |  PB11 |  DMA  LED_STRIP
| 9     | TIM3_CH3     |    PWM    |  PB0  |  FPV Camera Control(FCAM)


### Gyro & ACC ,support ICM20602 and MPU6000
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI1         |    SCK   |  PA5   | MPU6000 & ICM20602
| 2     | SPI1         |    MISO  |  PA6   | MPU6000 & ICM20602
| 3     | SPI1         |    MOSI  |  PA7   | MPU6000 & ICM20602
| 4     | SPI1         |    CS    |  PA4   | MPU6000 & ICM20602
| 5     | SPI1         |    INT   |  PC4   | MPU6000 & ICM20602

### OSD MAX7456
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI2         |    SCK   |  PB13  | 
| 2     | SPI2         |    MISO  |  PB14  | 
| 3     | SPI2         |    MOSI  |  PB15  | 
| 4     | SPI2         |    CS    |  PB12  |

### 16Mbyte flash
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI3         |    SCK   |  PB3   | 
| 2     | SPI3         |    MISO  |  PB4   | 
| 3     | SPI3         |    MOSI  |  PB5   | 
| 4     | SPI3         |    CS    |  PC15  | 

### SWD
| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | SWCLK          | PAD                                          |
| 2   | Ground         | PAD                                          |
| 3   | SWDIO          | PAD                                          |
| 4   | 3V3            | PAD                                          |



