
#pragma once

#define MPU6000_CONFIG		    	0x1A

#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03

#define GYRO_SCALE_FACTOR  0.00053292f  // (4/131) * pi/180   (32.75 LSB = 1 DPS)

#define MPU6000_WHO_AM_I_CONST              (0x68)

typedef struct mpu6000Config_s {
#ifdef STM32F303
    uint32_t gpioAHBPeripherals;
#endif
#ifdef STM32F10X
    uint32_t gpioAPB2Peripherals;
#endif
    uint16_t gpioPin;
    GPIO_TypeDef *gpioPort;

    uint8_t exti_port_source;
    uint32_t exti_line;
    uint8_t exti_pin_source;
    IRQn_Type exti_irqn;
} mpu6000Config_t;

bool mpu6000SpiAccDetect(const mpu6000Config_t *config, acc_t *acc);
bool mpu6000SpiGyroDetect(const mpu6000Config_t *config, gyro_t *gyro, uint16_t lpf);

bool mpu6000SpiGyroRead(int16_t *gyroADC);
bool mpu6000SpiAccRead(int16_t *gyroADC);
