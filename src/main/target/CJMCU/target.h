/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "CJM1" // CJMCU
#define USE_HARDWARE_REVISION_DETECTION

#define LED0                    PC14
#define LED1                    PC13
#define LED2                    PC15

#define ACC
#define USE_ACC_MPU6050

#define GYRO
#define USE_GYRO_MPU6050

//#define MAG
//#define USE_MAG_HMC5883

#define BRUSHED_MOTORS

#define USE_UART1
#define USE_UART2

#define SERIAL_PORT_COUNT       2

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)

// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67


#if (FLASH_SIZE > 64)
#define BLACKBOX
#define USE_SERVOS
#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PIN                PA3
#else
#undef USE_CLI
#undef SERIAL_RX
#define SKIP_TASK_STATISTICS
#define SKIP_CLI_COMMAND_HELP
#define SKIP_PID_FLOAT
#endif

// Since the CJMCU PCB has holes for 4 motors in each corner we can save same flash space by disabling support for other mixers.
#define USE_QUAD_MIXER_ONLY
#define SKIP_SERIAL_PASSTHROUGH

// IO - assuming all IOs on 48pin package TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))

#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))
