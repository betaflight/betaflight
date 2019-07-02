# Board - VIVAF4AIO

The VIVAF4AIO described here:
https://team-blacksheep.com/products/prod:viva_f4_fc

* STM32 F4 Processor
* ICM20602 Gyro, BMP280 Baro
* 5V 3A, 3.3V 0.5A BEC
* 2-6S battery input
* Betaflight OSD
* 16MB BlackBox memory
* Plug & Play connector for VivaFPV 4in1 ESC with telemetry & current sensor
* Plug & Play connector for TBS Unify Pro HV (7pin)
* Direct solder compatible with TBS Crossfire Nano & TBS Crossfire Nano Diversity Rx
* 5X UART (UART1 = RX, UART2 = VTX)
* Currentsensor & Camera Control included
* 30.5x30.5 mounting holes
* USB-Micro socket for firmware upgrades

### All uarts have pad on board 
| Value | Identifier   | RX   | TX   | Notes                                                                                       |
| ----- | ------------ | -----| -----| ------------------------------------------------------------------------------------------- |
| 1     | USART1       | PB7  |  PA9 |  PB7 FOR SBUS IN(inverter build in)                                                         |
| 2     | USART2       | PA3  |  PA2 |  PAD USE FOR TRAMP/smart audio                                                              |
| 3     | USART3       | PB11 |  PB10|  USE FOR GPS                                                                                |
| 4     | USART4       | PA1  |  PA0 |  PA0 FOR RSSI/FPORT/TEL etc                                                                 |
| 5     | USART5       | PD2  |  PC12|  PAD                                                                                        |

### I2C with GPS port together, also utilised for BARO or compass etc 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |
| 1     | I2C1         |    SDA   |  PB9   | with GPS outlet
| 2     | I2C1         |    SCL   |  PB8   | with GPS outlet

### Buzzer/LED output 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |
| 1     | LED0         |    LED   |  PC14  | 
| 2     | BEEPER       |    BEE   |  PC13  | 

### 6 Outputs, 1 PPM input 
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |
| 1     | TIM12_CH2    |    PPM    |  PB15 |  PPM
| 2     | TIM3_CH3     |    OUPUT1 |  PB0  |  DMA1_Stream7
| 3     | TIM8_CH1     |    OUPUT2 |  PC6  |  DMA2_Stream2
| 4     | TIM1_CH3     |    OUPUT3 |  PA10 |  DMA2_Stream6
| 5     | TIM1_CH1     |    OUPUT4 |  PA8  |  DMA2_Stream1
| 6     | TIM8_CH3     |    OUPUT5 |  PC8  |  DMA2_Stream4
| 7     | TIM3_CH4     |    OUPUT6 |  PB1  |  DMA1_Stream2
| 10    | TIM4_CH1     |    PWM    |  PB6  |  DMA1_Stream0   LED_STRIP
| 11    | TIM2_CH1     |    PWM    |  PA5  |  FPV Camera Control(FCAM)


### Gyro & ACC ,suppose ICM20602/MPU6000
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
