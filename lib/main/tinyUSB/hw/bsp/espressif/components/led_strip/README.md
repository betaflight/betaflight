# LED Strip Driver

[![Component Registry](https://components.espressif.com/components/espressif/led_strip/badge.svg)](https://components.espressif.com/components/espressif/led_strip)

This driver is designed for addressable LEDs like [WS2812](http://www.world-semi.com/Certifications/WS2812B.html), where each LED is controlled by a single data line.

## Backend Controllers

### The [RMT](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html) Peripheral

This is the most economical way to drive the LEDs because it only consumes one RMT channel, leaving other channels free to use. However, the memory usage increases dramatically with the number of LEDs. If the RMT hardware can't be assist by DMA, the driver will going into interrupt very frequently, thus result in a high CPU usage. What's worse, if the RMT interrupt is delayed or not serviced in time (e.g. if Wi-Fi interrupt happens on the same CPU core), the RMT transaction will be corrupted and the LEDs will display incorrect colors. If you want to use RMT to drive a large number of LEDs, you'd better to enable the DMA feature if possible [^1].

#### Allocate LED Strip Object with RMT Backend

```c
#define BLINK_GPIO 0

led_strip_handle_t led_strip;

/* LED strip initialization with the GPIO and pixels number*/
led_strip_config_t strip_config = {
    .strip_gpio_num = BLINK_GPIO, // The GPIO that connected to the LED strip's data line
    .max_leds = 1, // The number of LEDs in the strip,
    .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
    .led_model = LED_MODEL_WS2812, // LED strip model
    .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)
};

led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    .rmt_channel = 0,
#else
    .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
    .resolution_hz = 10 * 1000 * 1000, // 10MHz
    .flags.with_dma = false, // whether to enable the DMA feature
#endif
};
ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
```

You can create multiple LED strip objects with different GPIOs and pixel numbers. The backend driver will automatically allocate the RMT channel for you if there is more available.

### The [SPI](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html) Peripheral

SPI peripheral can also be used to generate the timing required by the LED strip. However this backend is not as economical as the RMT one, because it will take up the whole **bus**, unlike the RMT just takes one **channel**. You **CAN'T** connect other devices to the same SPI bus if it's been used by the led_strip, because the led_strip doesn't have the concept of "Chip Select".

Please note, the SPI backend has a dependency of **ESP-IDF >= 5.1**

#### Allocate LED Strip Object with SPI Backend

```c
#define BLINK_GPIO 0

led_strip_handle_t led_strip;

/* LED strip initialization with the GPIO and pixels number*/
led_strip_config_t strip_config = {
    .strip_gpio_num = BLINK_GPIO, // The GPIO that connected to the LED strip's data line
    .max_leds = 1, // The number of LEDs in the strip,
    .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
    .led_model = LED_MODEL_WS2812, // LED strip model
    .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)
};

led_strip_spi_config_t spi_config = {
    .clk_src = SPI_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
    .flags.with_dma = true, // Using DMA can improve performance and help drive more LEDs
    .spi_bus = SPI2_HOST,   // SPI bus ID
};
ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
```

The number of LED strip objects can be created depends on how many free SPI buses are free to use in your project.

## FAQ

* Which led_strip backend should I choose?
  * It depends on your application requirement and target chip's ability.

    ```mermaid
    flowchart LR
    A{Is RMT supported?}
    A --> |No| B[SPI backend]
    B --> C{Does the led strip has \n a larger number of LEDs?}
    C --> |No| D[Don't have to enable the DMA of the backend]
    C --> |Yes| E[Enable the DMA of the backend]
    A --> |Yes| F{Does the led strip has \n a larger number of LEDs?}
    F --> |Yes| G{Does RMT support DMA?}
    G --> |Yes| E
    G --> |No| B
    F --> |No| H[RMT backend] --> D
    ```

* How to set the brightness of the LED strip?
  * You can tune the brightness by scaling the value of each R-G-B element with a **same** factor. But pay attention to the overflow of the value.

[^1]: The RMT DMA feature is not available on all ESP chips. Please check the data sheet before using it.
