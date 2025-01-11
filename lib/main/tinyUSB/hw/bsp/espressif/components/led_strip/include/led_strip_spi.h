/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "led_strip_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LED Strip SPI specific configuration
 */
typedef struct {
    spi_clock_source_t clk_src; /*!< SPI clock source */
    spi_host_device_t spi_bus;  /*!< SPI bus ID. Which buses are available depends on the specific chip */
    struct {
        uint32_t with_dma: 1;   /*!< Use DMA to transmit data */
    } flags;                    /*!< Extra driver flags */
} led_strip_spi_config_t;

/**
 * @brief Create LED strip based on SPI MOSI channel
 * @note Although only the MOSI line is used for generating the signal, the whole SPI bus can't be used for other purposes.
 *
 * @param led_config LED strip configuration
 * @param spi_config SPI specific configuration
 * @param ret_strip Returned LED strip handle
 * @return
 *      - ESP_OK: create LED strip handle successfully
 *      - ESP_ERR_INVALID_ARG: create LED strip handle failed because of invalid argument
 *      - ESP_ERR_NOT_SUPPORTED: create LED strip handle failed because of unsupported configuration
 *      - ESP_ERR_NO_MEM: create LED strip handle failed because of out of memory
 *      - ESP_FAIL: create LED strip handle failed because some other error
 */
esp_err_t led_strip_new_spi_device(const led_strip_config_t *led_config, const led_strip_spi_config_t *spi_config, led_strip_handle_t *ret_strip);

#ifdef __cplusplus
}
#endif
