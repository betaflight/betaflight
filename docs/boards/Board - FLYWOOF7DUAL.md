# Board - FLYWOOF7DUAL

This board use the STM32F722RET6 microcontroller and have the following features:
* High-performance and DSP with FPU, ARM Cortex-M7 MCU with 512 Kbytes Flash 
* 216 MHz CPU,462 DMIPS/2.14 DMIPS/MHz (Dhrystone 2.1) and DSP instructions, Art Accelerator, L1 cache, SDRAM
DUAL gyro MPU6000 and ICM20689,could choose mpu6000,more stable and smooth.Or choose ICM20689,higher rate(32K/32K).Choose on CLI mode,experience different feature gyro on same board
* The 16M byte SPI flash for data logging
* USB VCP and boot select button on board(for DFU)
* Stable voltage regulation,9V/2A DCDC BEC for VTX/camera etc.And could select 5v/9v with pad
* Serial LED interface(LED_STRIP)
* VBAT/CURR/RSSI sensors input
* Suppose IRC Tramp/smart audio/FPV Camera Control/FPORT/telemetry
* Supports SBus, Spektrum1024/2048, PPM. No external inverters required (built-in).
* Supports I2C device extend(baro/compass/OLED etc)(socket)
* Supports GPS (socket)

### All uarts have pad on board 
| Value | Identifier   | RX   | TX   | Notes                                                                                       |
| ----- | ------------ | -----| -----| ------------------------------------------------------------------------------------------- |
| 1     | USART1       | PA10 |  PA9 | FOR SBUS IN(inverter build in)     
| 2     | USART2       | PA3  |  PA2 | USE FOR TRAMP/smart audio                                                                   |
| 3     | USART3       | PB11 |  PB10| USE FOR GPS                                                                                 |
| 4     | USART4       | PA1  |  PA0 | PAD USE FOR TRAMP/smart audio                                                               |
| 5     | USART5       | PD2  |  PC12| PAD ESC sensor                                                                              |
| 6     | USART6       | PC7  |  PC6 | PAD                                                                                         |


### I2C with GPS port together.Use for BARO or compass etc 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | I2C1         |    SDA   |  PB7   | with GPS outlet
| 2     | I2C1         |    SCL   |  PB6   | with GPS outlet


### Buzzer/LED output 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | LED0         |    LED   |  PC15  | 
| 2     | BEEPER       |    BEE   |  PC14  | 


### VBAT input with 1/10 divider ratio,Current signal input,Analog/digit RSSI input
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | ADC1         |    VBAT   |  PC1  |  DMA2_Stream0
| 2     | ADC1         |    CURR   |  PC0  |  DMA2_Stream0
| 3     | ADC1         |    RSSI   |  PC2  |  DMA2_Stream0


### 6 Outputs, 1 PPM input 
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | TIM11_CH1    |    PPM    |  PB9  |  PPM
| 2     | TIM3_CH4     |    OUPUT1 |  PB1  |  DMA1_Stream2
| 3     | TIM3_CH1     |    OUPUT2 |  PB4  |  DMA1_Stream4
| 4     | TIM2_CH2     |    OUPUT3 |  PB3  |  DMA1_Stream6
| 5     | TIM2_CH1     |    OUPUT4 |  PA15 |  DMA1_Stream5
| 6     | TIM8_CH3     |    OUPUT5 |  PC8  |  DMA2_Stream4
| 7     | TIM8_CH4     |    OUPUT6 |  PC9  |  DMA2_Stream1
| 8     | TIM1_CH1     |    LED    |  PA8  |  LED STRIP   
| 9     | TIM4_CH3     |    PWM    |  PB8  |  FC CAM   

| 10    | TIM8_CH3     |    LED    |  PC8  |  DMA2_Stream2   LED_STRIP
| 11    | TIM1_CH2     |    PWM    |  PA9  |  FPV Camera Control(FCAM)


### Gyro & ACC  ICM20689
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI1         |    SCK   |  PA5   | MPU6000 & ICM20689
| 2     | SPI1         |    MISO  |  PA6   | MPU6000 & ICM20689
| 3     | SPI1         |    MOSI  |  PA7   | MPU6000 & ICM20689
| 4     | SPI1         |    CS1   |  PA4   | MPU6000
| 5     | SPI1         |    CS2   |  PB2   | ICM20689
| 6     | SPI1         |    INT1  |  PC3   | MPU6000
| 7     | SPI1         |    INT2  |  PC4   | ICM20689

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
| 1     | SPI3         |    SCK   |  PC10  | 
| 2     | SPI3         |    MISO  |  PC11  | 
| 3     | SPI3         |    MOSI  |  PB5   | 
| 4     | SPI3         |    CS    |  PC13  | 

### SWD
| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | SWCLK          | PAD                                          |
| 2   | Ground         | PAD                                          |
| 3   | SWDIO          | PAD                                          |
| 4   | 3V3            | PAD                                          |

* FLYWOO TECH 
* www.flywoo.net




