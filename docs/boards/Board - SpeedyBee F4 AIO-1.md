# Board - SpeedyBee F4 AIO

### Descriptions
This SpeedyBee F4 AIO board integrates the Bluetooth module, with SpeedyBee App, we can set the FC parameter via mobile device.It runs Betaflight, with Built-in LC-Filter for 5V and 9V, and  no pinned-connectors – all direct solder for optimal durability on top side. It has the current sensor and ESC power output. 

### Hardware Features
* MCU: STM32F405
* IMU: MPU6000
* OSD: BetaFlight OSD w/ AT7456E chip
* BLE Module: inner connect to UART5 for remote setting with SpeedyBee App or other similar apps
* BlackBox: 16Mb onboard dataflash
* Current Sensor: 200A(Scale 608)
* Power input: 3s - 6s Lipo
* Power output: 4.5V * 1 , 3.3V * 1, 5V * 3 , 9V * 1
* Cam And VTX power output: Built-in LC-Filter for 5V and 9V
* ESC power output: 4 * VCC output
* UART: UART Pads * 3(UART1, UART3, UART4)
* UART2 for Receiver: Built in inverter of RX2 for SBUS input through SBUS pad, RX2 pad used for PPM, DSM2, DSMX, IBUS
* RSSI input: RSSI input solder pad
* I2C: Used for external GPS, Barometer module
* Buzzer: Buz- and 5V pad used for 5V Buzzer
* ESC signal: s1 - s7
* LED pin: Used for WS2812 LED
* Boot button: Used to easy enter DFU mode



### All uarts have pad on board 
| Value | Identifier   | RX   | TX   | Notes                                                                                       |
| ----- | ------------ | -----| -----| ------------------------------------------------------------------------------------------- |
| 1     | USART1       | PA10  |  PA9 |  PB7 FOR SBUS IN(inverter built-in)                                                         |
| 2     | USART2       | PA3  |  PA2 |  PA3 for SBUS signal through SBUS pad,                                                               and normally connect to RX2 pad for other types receiver |
| 3     | USART3       | PC11 |  PC10|  USE FOR GPS                                                                                |
| 4     | USART4       | PA1  |  PA0 |  PA0 FOR RSSI/FPORT/TEL etc                                                                 |
| 5     | USART5       | PD2  |  PC12|  PAD                                                                                        |


### I2C, use for Barometer or compass
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | I2C1         |    SDA   |  PB7   | 
| 2     | I2C1         |    SCL   |  PB6   | 


### Buzzer/LED output 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | LED0         |    LED   |  PB9  | 
| 2     | BEEPER       |    BEE   |  PC13  | 


### Analog signal input
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | ADC1         |    VBAT   |  PC2  | 
| 2     | ADC1         |    CURR   |  PC1  | 
| 3     | ADC1         |    RSSI   |  PC3  | 


### 7 PWM Outputs
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                        
| 1     | TIM8_CH1     |    OUPUT1 |  PC6  |  DMA1_Stream7
| 2     | TIM8_CH2     |    OUPUT2 |  PC7  |  DMA2_Stream2
| 3     | TIM8_CH3     |    OUPUT3 |  PC8  |  DMA2_Stream6
| 4     | TIM8_CH4     |    OUPUT4 |  PC9  |  DMA2_Stream1
| 5     | TIM2_CH1     |    OUPUT5 |  PA15 |  DMA2_Stream4
| 6     | TIM1_CH1     |    OUPUT6 |  PA8  |  DMA1_Stream2
| 7     | TIM4_CH3     |    OUPUT7 |  PB8  |  DMA1_Stream5

### LED & PPM Input
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                        
| 1     | TIM5_CH4     |    PPM |  PA3  | 
| 2     | TIM4_CH1     |    LED Strip Signal Input |  PB6  | 


### Gyro & ACC(MPU6000)
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI1         |    SCK   |  PA5   | 
| 2     | SPI1         |    MISO  |  PA6   | 
| 3     | SPI1         |    MOSI  |  PA7   | 
| 4     | SPI1         |    CS    |  PB11   | 

### OSD(AT7456E)
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI2         |    SCK   |  PB13  | 
| 2     | SPI2         |    MISO  |  PB14  | 
| 3     | SPI2         |    MOSI  |  PB15   | 
| 4     | SPI2         |    CS    |  PB10  |

### 16Mbyte onboard flash
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI3         |    SCK   |  PB3  | 
| 2     | SPI3         |    MISO  |  PB4  | 
| 3     | SPI3         |    MOSI  |  PB5   | 
| 4     | SPI3         |    CS    |  PC0  | 




