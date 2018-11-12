# Board - VGOODF722DUAL

The VGOODF722DUAL  described here:

This board use the STM32F722RET6 microcontroller and have the following features:
* MCU
	* * High-performance and DSP with FPU, ARM Cortex-M7 MCU with 512 Kbytes Flash 
	* * 216 MHz CPU,462 DMIPS/2.14 DMIPS/MHz (Dhrystone 2.1) and DSP instructions, Art Accelerator, L1 cache, SDRAM
* DUAL gyro MPU6000 and ICM20602,could choose mpu6000,more stable and smooth.Or choose ICM20602,higher rate(32K/32K).Choose on CLI mode,experience different feature gyro on same board
* The 16M byte SPI flash on board for data logging
* USB VCP and boot select button on board(for DFU)
* Stable voltage regulation,DUAL BEC  5v/2A and 10v/2A for VTX/camera etc.And could select 5v/10v with pad
* Serial LED interface(LED_STRIP)
* VBAT/CURR/RSSI sensors input
	* *Current Sensor:* Rated for 184A (*Suggested scale value `179`*)
	* *Voltage Sensor:* 1:10 signal output ratio (*Suggested scale value `110`*)
* Suppose IRC Tramp/smart audio/FPV Camera Control/FPORT/telemetry
* Supports SBus(built-in inverters), Spektrum1024/2048, PPM
* Supports BMP280 (connected via I2C1)
* BetaFlight OSD (AT7456E connected via SPI1)
* Supports GPS (socket)
* 5 UARTs (1,2,3,4,6)
* 6 Dshot outputs


## Status LEDs
| Value | Identifier	| Color	| Color Codes																					   | 
| ----- | ------------- | ------| ------------------------------------------------------------------------------------------------ | 
| 1		| LED0			| Blue	| FC Status																					   	   |
| 2		| LED1			| Green	| FC Status																					   	   |
| 3		| LED3.3		| Red	| 3v3 Status																					   |


### All uarts have pad on board 
| Value | Identifier   | TX   | RX   | Notes                                                                                       |
| ----- | ------------ | -----| -----| ------------------------------------------------------------------------------------------- |
| 1     | USART1       | PA9  |  PA10|  PAD USE FOR TRAMP/smart audio	                                                           |
| 2     | USART2       | PA2  |  PA3 |  PB7 FOR SBUS IN(inverter build in)/PPM                                                     |         |
| 3     | USART3       | PB10 |  PB11|  USE FOR GPS                                                                                |
| 4     | USART4       | PA0  |  PA1 |  PA0 FOR RSSI/FPORT/TEL etc                                                                 |
| 5     | USART6       | PC6  |  PC7 |  PAD                                                                                        |


### I2C use for BARO BMP280 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | I2C1         |    SDA   |  PB9   | 
| 2     | I2C1         |    SCL   |  PB8   | 


### Buzzer/LED output 
| Value | Identifier   | function |  pin   | Notes                                                                                 |
| ----- | ------------ | ---------| -------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | BEEPER       |    BEEP  |  PC13  |Pad 


### VBAT input with 1/10 divider ratio,Current signal input,Analog/digit RSSI input
| Value | Identifier   | function  |  pin  | Notes                                                                                 |
| ----- | ------------ | ----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | ADC1         |    VBAT   |  PC0  |  
| 2     | ADC1         |    CURR   |  PC1  |  
| 3     | ADC1         |    RSSI   |  PC2  |   


### 6 Outputs & 1 Ledstrip & 1 PPM(SBUS)
| Value | Identifier   | function	|  pin  | Notes                                                                                 |
| ----- | ------------ | -----------| ------| ------------------------------------------------------------------------------------- |                                                                                       
| 1     | TIM9_CH2     | PPM&SBUS	|  PA3  |  PPM&SBUS
| 2     | TIM3_CH1     | OUPUT1 	|  PB4  |  DMA
| 3     | TIM3_CH2     | OUPUT2 	|  PB5  |  DMA
| 4     | TIM3_CH3     | OUPUT3 	|  PB0  |  DMA
| 5     | TIM3_CH4     | OUPUT4 	|  PB1  |  DMA
| 6     | TIM4_CH1     | OUPUT5 	|  PB6  |  DMA
| 7     | TIM4_CH2     | OUPUT6 	|  PB7  |  DMA  
| 8     | TIM1_CH1     | PWM	 	|  PA8  |  DMA  LED_STRIP


### OSD MAX7456
| Value | Identifier   | function |  pin    | Notes                                                                                 |
| ----- | ------------ | ---------| --------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI1         |    SCK   |  PA5    | 
| 2     | SPI1         |    MISO  |  PA6    | 
| 3     | SPI1         |    MOSI  |  PA7    | 
| 4     | SPI1         |    CS    |  PB2    |

### 16Mbyte flash
| Value | Identifier   | function |  pin    | Notes                                                                                 |
| ----- | ------------ | ---------| --------| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI2         |    SCK   |  PB13   | 
| 2     | SPI2         |    MISO  |  PB14   | 
| 3     | SPI2         |    MOSI  |  PB15   | 
| 4     | SPI2         |    CS    |  PB12   | 


### Gyro & ACC ,support ICM20602 and MPU6000
| Value | Identifier   | function |  pin    | Notes                                                                                 |
| ----- | ------------ | ---------| -------_| ------------------------------------------------------------------------------------- |                                                                                      
| 1     | SPI3         |    SCK   |  PC10   | MPU6000 & ICM20602
| 2     | SPI3         |    MISO  |  PC11   | MPU6000 & ICM20602
| 3     | SPI3         |    MOSI  |  PC12   | MPU6000 & ICM20602
| 4     | SPI3         |    CS1   |  PA15   | MPU6000
| 5     | SPI3         |    CS2   |  PD2    | ICM20602 
| 6     | SPI3         |    INT1  |  PC3    | MPU6000
| 7     | SPI3         |    INT2  |  PC4    | ICM20602


###Designers
* zhanshenrui(zhanshenrui@yeah.net)





