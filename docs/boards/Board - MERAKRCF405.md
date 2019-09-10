# Board - MERAKRCF405

MERAKRCF405 flight control is carefully optimized layout, so that users can be more concise,convenient wiring,.For ensure a good flight experience ,we use of high-performance MPU6000 gyroscope. After a long flight verification test, the flight control ensures stable flight performance. For details of flight control, please visit MERAK RC website:(http://www.merakrc.com) 

### Hardware

* Processors and Sensors
  * *MCU: STM32F405RGT6
  * IMU_1 MPU6000(0 deg)
  * *OSD:* BetaFlight OSD (AT7456E)
  * Blackbox: FLASH W25Q128 (16MB)

* 6 Dshot outputs

* 5 UARTs (UART5 support SerialRC)

* Stable voltage regulation,9V/2A DCDC BEC for VTX/camera etc.5V/2A DCDC BEC for FC/WS2812 etc.

* Independent camera control.

### Pinout

### All uarts have pad on board 

| Value | Identifier |  RX  |  TX  |  Notes   |
| :---: | :--------: | :--: | :--: | :------: |
|   1   |   USART1   | PA10 | PA9  |          |
|   2   |   USART2   | PA3  | PA2  |          |
|   3   |   USART3   | PB11 | PB10 |          |
|   4   |   USART4   | PC11 | PC10 |          |
|   5   |   USART5   | PD2  | PC12 | Rx input |

### Buzzer/LED output 

| Value | Identifier | function | pin  | Notes |
| :---: | :--------: | :------: | :--: | :---: |
|   1   |    LED0    |   LED    | PC13 |       |
|   2   |   BEEPER   |   BEEP   | PC14 |       |

### VBAT input, Current input, Analog RSSI input

| Value | Identifier | function | pin  | Notes |
| :---: | :--------: | :------: | :--: | :---: |
|   1   |    ADC1    |   VBAT   | PC2  |       |
|   2   |    ADC1    | CURRENT  | PC1  |       |
|   3   |    ADC1    | RSSI     | PC3  |       |

### PWM Input & PWM Output & LED strip & Camera control

| Value | Identifier |  function   | pin  | Notes |
| :---: | :--------: | :---------: | :--: | :---: |
|   1   |  TIM8_CH3  |     PPM     | PC8  |       |
|   2   |  TIM3_CH1  |   Motor1    | PC6  |       |
|   3   |  TIM3_CH2  |   Motor2    | PC7  |       |
|   4   |  TIM5_CH1  |   Motor3    | PA0  |       |
|   5   |  TIM5_CH2  |   Motor4    | PA1  |       |
|   6   |  TIM3_CH3  |   Motor5    | PB0  |       |
|   7   |  TIM1_CH3N |   Motor6    | PB1  |       |
|   8   |  TIM4_CH3  |  LED strip  | PB8  |       |
|   9   |  TIM11_CH1 | CAM Control | PB9  |       |

### Gyro & ACC  MPU6000

| Value | Identifier | function | pin  |     Notes      |
| :---: | :--------: | :------: | :--: | :------------: |
|   1   |    SPI2    |   SCK    | PB13  |    MPU6000    |
|   2   |    SPI2    |   MISO   | PB14  |    MPU6000    |
|   3   |    SPI2    |   MOSI   | PB15  |    MPU6000    |
|   4   |    SPI2    |   CS1    | PB12  |  MPU6000_CS   |
|   5   |     IO     |   INT2   | PC4   |  MPU6000_INT  |

### OSD MAX7456

| Value | Identifier | function | pin  | Notes |
| :---: | :--------: | :------: | :--: | :---: |
|   1   |    SPI3    |   SCK    | PB3  |       |
|   2   |    SPI3    |   MISO   | PB4  |       |
|   3   |    SPI3    |   MOSI   | PB5  |       |
|   4   |    SPI3    |    CS    | PA15 |       |

### FLash Blackbox

| Value | Identifier | function | pin  | Notes |
| :---: | :--------: | :------: | :--: | :---: |
|   1   |    SPI1    |   SCK    |  PA5 |       |
|   2   |    SPI1    |   MISO   |  PA6 |       |
|   3   |    SPI1    |   MOSI   |  PA7 |       |
|   4   |    SPI1    |    CS    |  PA4 |       |

