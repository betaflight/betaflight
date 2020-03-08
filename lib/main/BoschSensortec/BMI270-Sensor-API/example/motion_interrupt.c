#include <stdio.h>
#include "bmi2.h"
#include "bmi270.h"

void delay_us(uint32_t period);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(int8_t rslt);

int main(void)
{
    /* Variable to define rslt */
    int8_t rslt;
    struct bmi2_dev dev;
    struct bmi2_sens_config config;

    /* Array to select sensors */
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_NO_MOTION };

    /* Variable to get any-motion status */
    uint16_t no_mot_status = 0;

    /* Select features and their pins to be mapped to */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_NO_MOTION, .hw_int_pin = BMI2_INT1 };


    /*to enable the i2c interface*/
       dev.read = i2c_reg_read;
       dev.write = i2c_reg_write;
       dev.delay_us = delay_us;
       dev.read_write_len = 128;
       dev.intf = BMI2_I2C_INTERFACE;
       dev.dev_id = BMI2_I2C_PRIM_ADDR;

       /*to enable spi interface*/
    /*
     * ac_setup_interface(AC_SPI_STD);
     * dev.read = ac_spi_read;
     * dev.write = ac_spi_write;
     * dev.delay_us = delay_us;
     * dev.read_write_len = 4096;
     * dev.intf = BMI2_SPI_INTERFACE;
     * dev.dev_id = SPI_CS;
     * dev.dummy_byte = 1;
     */

    dev.config_file_ptr = NULL;

    /*initialize bmi270 */
    rslt = bmi270_init(&dev);
    print_rslt(rslt);

    /* Enable the selected sensors */
    rslt = bmi2_sensor_enable(sens_list, 2, &dev);
    print_rslt(rslt);

    /* Configure type of feature */
    config.type = BMI2_NO_MOTION;

    /* Get default configurations for the type of feature selected */
    rslt = bmi2_get_sensor_config(&config, 1, &dev);
    print_rslt(rslt);

    /* Set the new configuration */
    rslt = bmi2_set_sensor_config(&config, 1, &dev);
    print_rslt(rslt);

    /* Set the new configuration */
    rslt = bmi2_map_feat_int(&sens_int, 1, &dev);
    print_rslt(rslt);

    printf("Do not move the board\n");

    while (1)
    {
        /* Clear buffer */
        no_mot_status = 0;

        dev.delay_us(50000);

        /* Check the interrupt status of the no-motion */
        rslt = bmi2_get_int_status(&no_mot_status, &dev);
        print_rslt(rslt);
        if (no_mot_status & BMI270_NO_MOT_STATUS_MASK)
        {
            printf("no_mot_status = %x\n", no_mot_status);
            printf("No-motion interrupt generated");
            break;
        }

        dev.delay_us(50000);
    }

    return 0;
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bma4_write_config_file",
 *      "bma4_write_regs", "bma4_set_accel_config"  and so on.
 *
 *  @param[in] period_us  : the required wait time in microseconds.
 *  @return void.
 *
 */
void delay_us(uint32_t period)
{
    /* Wait for a period amount of us*/
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length)
{

    /* Write to registers using I2C. Return 0 for a successful execution. */
    return 0;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Read from registers using I2C. Return 0 for a successful execution. */
    return 0;
}

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] cs           : Chip select to enable the sensor.
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Write to registers using SPI. Return 0 for a successful execution. */
    return 0;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] cs       : Chip select to enable the sensor.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Read from registers using SPI. Return 0 for a successful execution. */
    return 0;
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(int8_t rslt)
{
    switch (rslt)
    {
        case BMI2_OK:

            /* Do nothing */
            break;
        case BMI2_E_NULL_PTR:
            printf("Error [%d] : Null pointer\r\n", rslt);
            break;
        case BMI2_E_COM_FAIL:
            printf("Error [%d] : Communication failure\r\n", rslt);
            break;
        case BMI2_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found\r\n", rslt);
            break;
        case BMI2_E_INVALID_SENSOR:
            printf("Error [%d] : Invalid sensor\r\n", rslt);
            break;
        case BMI2_E_SELF_TEST_FAIL:
            printf("Warning [%d] : Self test failed\r\n", rslt);
            break;
        case BMI2_E_INVALID_INT_PIN:
            printf("warning [%d] : invalid int pin\r\n", rslt);
            break;
        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}
