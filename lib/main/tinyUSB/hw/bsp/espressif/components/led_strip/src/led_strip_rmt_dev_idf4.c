/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/rmt.h"
#include "led_strip.h"
#include "led_strip_interface.h"

static const char *TAG = "led_strip_rmt";

#define WS2812_T0H_NS   (300)
#define WS2812_T0L_NS   (900)
#define WS2812_T1H_NS   (900)
#define WS2812_T1L_NS   (300)

#define SK6812_T0H_NS   (300)
#define SK6812_T0L_NS   (900)
#define SK6812_T1H_NS   (600)
#define SK6812_T1L_NS   (600)

#define LED_STRIP_RESET_MS (10)

// the memory size of each RMT channel, in words (4 bytes)
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define LED_STRIP_RMT_DEFAULT_MEM_BLOCK_SYMBOLS 64
#else
#define LED_STRIP_RMT_DEFAULT_MEM_BLOCK_SYMBOLS 48
#endif

static uint32_t led_t0h_ticks = 0;
static uint32_t led_t1h_ticks = 0;
static uint32_t led_t0l_ticks = 0;
static uint32_t led_t1l_ticks = 0;

typedef struct {
    led_strip_t base;
    rmt_channel_t rmt_channel;
    uint32_t strip_len;
    uint8_t bytes_per_pixel;
    uint8_t buffer[0];
} led_strip_rmt_obj;

static void IRAM_ATTR ws2812_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
        size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL) {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ led_t0h_ticks, 1, led_t0l_ticks, 0 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ led_t1h_ticks, 1, led_t1l_ticks, 0 }}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (size < src_size && num < wanted_num) {
        for (int i = 0; i < 8; i++) {
            // MSB first
            if (*psrc & (1 << (7 - i))) {
                pdest->val =  bit1.val;
            } else {
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}

static esp_err_t led_strip_rmt_set_pixel(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
    led_strip_rmt_obj *rmt_strip = __containerof(strip, led_strip_rmt_obj, base);
    ESP_RETURN_ON_FALSE(index < rmt_strip->strip_len, ESP_ERR_INVALID_ARG, TAG, "index out of the maximum number of leds");
    uint32_t start = index * rmt_strip->bytes_per_pixel;
    // In thr order of GRB
    rmt_strip->buffer[start + 0] = green & 0xFF;
    rmt_strip->buffer[start + 1] = red & 0xFF;
    rmt_strip->buffer[start + 2] = blue & 0xFF;
    if (rmt_strip->bytes_per_pixel > 3) {
        rmt_strip->buffer[start + 3] = 0;
    }
    return ESP_OK;
}

static esp_err_t led_strip_rmt_refresh(led_strip_t *strip)
{
    led_strip_rmt_obj *rmt_strip = __containerof(strip, led_strip_rmt_obj, base);
    ESP_RETURN_ON_ERROR(rmt_write_sample(rmt_strip->rmt_channel, rmt_strip->buffer, rmt_strip->strip_len * rmt_strip->bytes_per_pixel, true), TAG,
                        "transmit RMT samples failed");
    vTaskDelay(pdMS_TO_TICKS(LED_STRIP_RESET_MS));
    return ESP_OK;
}

static esp_err_t led_strip_rmt_clear(led_strip_t *strip)
{
    led_strip_rmt_obj *rmt_strip = __containerof(strip, led_strip_rmt_obj, base);
    // Write zero to turn off all LEDs
    memset(rmt_strip->buffer, 0, rmt_strip->strip_len * rmt_strip->bytes_per_pixel);
    return led_strip_rmt_refresh(strip);
}

static esp_err_t led_strip_rmt_del(led_strip_t *strip)
{
    led_strip_rmt_obj *rmt_strip = __containerof(strip, led_strip_rmt_obj, base);
    ESP_RETURN_ON_ERROR(rmt_driver_uninstall(rmt_strip->rmt_channel), TAG, "uninstall RMT driver failed");
    free(rmt_strip);
    return ESP_OK;
}

esp_err_t led_strip_new_rmt_device(const led_strip_config_t *led_config, const led_strip_rmt_config_t *dev_config, led_strip_handle_t *ret_strip)
{
    led_strip_rmt_obj *rmt_strip = NULL;
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(led_config && dev_config && ret_strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(led_config->led_pixel_format < LED_PIXEL_FORMAT_INVALID, ESP_ERR_INVALID_ARG, TAG, "invalid led_pixel_format");
    ESP_RETURN_ON_FALSE(dev_config->flags.with_dma == 0, ESP_ERR_NOT_SUPPORTED, TAG, "DMA is not supported");

    uint8_t bytes_per_pixel = 3;
    if (led_config->led_pixel_format == LED_PIXEL_FORMAT_GRBW) {
        bytes_per_pixel = 4;
    } else if (led_config->led_pixel_format == LED_PIXEL_FORMAT_GRB) {
        bytes_per_pixel = 3;
    } else {
        assert(false);
    }

    // allocate memory for led_strip object
    rmt_strip = calloc(1, sizeof(led_strip_rmt_obj) + led_config->max_leds * bytes_per_pixel);
    ESP_RETURN_ON_FALSE(rmt_strip, ESP_ERR_NO_MEM, TAG, "request memory for les_strip failed");

    // install RMT channel driver
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(led_config->strip_gpio_num, dev_config->rmt_channel);
    // set the minimal clock division because the LED strip needs a high clock resolution
    config.clk_div = 2;

    uint8_t mem_block_num = 2;
    // override the default value if the user specify the mem block size
    if (dev_config->mem_block_symbols) {
        mem_block_num = (dev_config->mem_block_symbols + LED_STRIP_RMT_DEFAULT_MEM_BLOCK_SYMBOLS / 2) / LED_STRIP_RMT_DEFAULT_MEM_BLOCK_SYMBOLS;
    }
    config.mem_block_num = mem_block_num;

    ESP_GOTO_ON_ERROR(rmt_config(&config), err, TAG, "RMT config failed");
    ESP_GOTO_ON_ERROR(rmt_driver_install(config.channel, 0, 0), err, TAG, "RMT install failed");

    uint32_t counter_clk_hz = 0;
    rmt_get_counter_clock((rmt_channel_t)dev_config->rmt_channel, &counter_clk_hz);
    // ns -> ticks
    float ratio = (float)counter_clk_hz / 1e9;
    if (led_config->led_model == LED_MODEL_WS2812) {
        led_t0h_ticks = (uint32_t)(ratio * WS2812_T0H_NS);
        led_t0l_ticks = (uint32_t)(ratio * WS2812_T0L_NS);
        led_t1h_ticks = (uint32_t)(ratio * WS2812_T1H_NS);
        led_t1l_ticks = (uint32_t)(ratio * WS2812_T1L_NS);
    } else if (led_config->led_model == LED_MODEL_SK6812) {
        led_t0h_ticks = (uint32_t)(ratio * SK6812_T0H_NS);
        led_t0l_ticks = (uint32_t)(ratio * SK6812_T0L_NS);
        led_t1h_ticks = (uint32_t)(ratio * SK6812_T1H_NS);
        led_t1l_ticks = (uint32_t)(ratio * SK6812_T1L_NS);
    } else {
        assert(false);
    }

    // adapter to translates the LES strip date frame into RMT symbols
    rmt_translator_init((rmt_channel_t)dev_config->rmt_channel, ws2812_rmt_adapter);

    rmt_strip->bytes_per_pixel = bytes_per_pixel;
    rmt_strip->rmt_channel = (rmt_channel_t)dev_config->rmt_channel;
    rmt_strip->strip_len = led_config->max_leds;
    rmt_strip->base.set_pixel = led_strip_rmt_set_pixel;
    rmt_strip->base.refresh = led_strip_rmt_refresh;
    rmt_strip->base.clear = led_strip_rmt_clear;
    rmt_strip->base.del = led_strip_rmt_del;

    *ret_strip = &rmt_strip->base;
    return ESP_OK;

err:
    if (rmt_strip) {
        free(rmt_strip);
    }
    return ret;
}
