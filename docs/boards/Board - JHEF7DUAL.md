# Board - JHEF7DUAL

## Features

* Processors and Sensors
  * *MCU:* STM32F722RET6
  * *IMU:* ICM20689(Gyro1) & MPU6000(Gyro2) connected via SPI1
  * *Baro:* BMP280 (connected via I2C1)
  * *OSD:* BetaFlight OSD (AT7456E connected via SPI2)
* *Blackbox:* FLASH M25P16 (connected via SPI3)
* 6 UARTs (1,2,3,4,5,6)
* 8 Dshot outputs
* 2 PINIO (VTX power switcher/user1 and 2 camera switcher/user2)
* USB VCP and boot select button on board (for DFU) 
* Serial LED interface(LED_STRIP)
* VBAT / CURR / RSSI sensors input
* Suppose IRC Tramp / Smart audio / FPV Camera Control / FPORT/telemetry
* Supports SBus, Spektrum1024/2048, PPM. No external inverters required (built-in).
* Supports I2C device extend(Compass / OLED etc)
* Supports GPS

## Pinout

### All uarts have pad on board 

| Value | Identifier |  RX  |  TX  |             Notes              |
| :---: | :--------: | :--: | :--: | :----------------------------: |
|   1   |   USART1   | PA10 | PA9  | FOR SBUS IN(inverter build in) |
|   2   |   USART2   | PA3  | PA2  |   USE FOR TRAMP/smart audio    |
|   3   |   USART3   | PB11 | PB10 |          USE FOR GPS           |
|   4   |   USART4   | PA1  | PA0  | PAD USE FOR TRAMP/smart audio  |
|   5   |   USART5   | PD2  | PC12 |         PAD ESC sensor         |
|   6   |   USART6   | PC7  | PC6  |              PAD               |

### I2C with GPS port together.Use for BARO or compass etc 

| Value | Identifier | function | pin  | Notes |
| :---: | :--------: | :------: | :--: | :---: |
|   1   |    I2C1    |   SDA    | PB7  |       |
|   2   |    I2C1    |   SCL    | PB6  |       |

### Buzzer/LED output 

| Value | Identifier | function | pin  | Notes |
| :---: | :--------: | :------: | :--: | :---: |
|   1   |    LED0    |   LED    | PA15 |       |
|   2   |   BEEPER   |   BEE    | PC15 |       |

### VBAT input, Current input, Analog RSSI input

| Value | Identifier | function | pin  |    Notes     |
| :---: | :--------: | :------: | :--: | :----------: |
|   1   |    ADC1    |   VBAT   | PC2  | DMA2_Stream0 |
|   2   |    ADC1    |   CURR   | PC1  | DMA2_Stream0 |
|   3   |    ADC1    |   RSSI   | PC0  | DMA2_Stream0 |

### PWM Input & PWM Output 

| Value | Identifier | function | pin  |    Notes     |
| :---: | :--------: | :------: | :--: | :----------: |
|   1   |  TIM9_CH2  |   PPM    | PA3  |     PPM      |
|   2   |  TIM3_CH3  |  Motor1  | PB0  | DMA1_Stream2 |
|   3   |  TIM3_CH4  |  Motor2  | PB1  | DMA1_Stream2 |
|   4   |  TIM3_CH1  |  Motor3  | PB4  | DMA1_Stream4 |
|   5   |  TIM2_CH2  |  Motor4  | PB3  | DMA1_Stream6 |
|   7   |  TIM8_CH4  |  Motor5  | PC9  | DMA2_Stream1 |
|   8   |  TIM8_CH3  |  Motor6  | PC8  | DMA2_Stream4 |
|   9   |  TIM1_CH1  |   LED    | PA8  |  LED STRIP   |
|  10   |  TIM4_CH3  |   ANY    | PB8  |    FC CAM    |

### Gyro & ACC  ICM20689

| Value | Identifier | function | pin  |       Notes        |
| :---: | :--------: | :------: | :--: | :----------------: |
|   1   |    SPI1    |   SCK    | PA5  | MPU6000 & ICM20689 |
|   2   |    SPI1    |   MISO   | PA6  | MPU6000 & ICM20689 |
|   3   |    SPI1    |   MOSI   | PA7  | MPU6000 & ICM20689 |
|   4   |    SPI1    |   CS2    | PA4  |      MPU6000       |
|   5   |    SPI1    |   CS1    | PB2  |      ICM20689      |
|   6   |    SPI1    |   INT2   | PC3  |      MPU6000       |
|   7   |    SPI1    |   INT1   | PC4  |      ICM20689      |

### OSD MAX7456

| Value | Identifier | function | pin  | Notes |
| :---: | :--------: | :------: | :--: | :---: |
|   1   |    SPI2    |   SCK    | PB13 |       |
|   2   |    SPI2    |   MISO   | PB14 |       |
|   3   |    SPI2    |   MOSI   | PB15 |       |
|   4   |    SPI2    |    CS    | PB12 |       |

### 2MB FLash

| Value | Identifier | function | pin  | Notes |
| :---: | :--------: | :------: | :--: | :---: |
|   1   |    SPI3    |   SCK    | PC10 |       |
|   2   |    SPI3    |   MISO   | PC11 |       |
|   3   |    SPI3    |   MOSI   | PB5  |       |
|   4   |    SPI3    |    CS    | PC13 |       |

### SWD

| Pin  | Function | Notes |
| :--: | :------: | :---: |
|  1   |  SWCLK   |  PAD  |
|  2   |  Ground  |  PAD  |
|  3   |  SWDIO   |  PAD  |
|  4   |   3V3    |  PAD  |

## Designers

- JHE_FPV