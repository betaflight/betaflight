# API Reference

## Header files

- [include/led_strip.h](#file-includeled_striph)
- [include/led_strip_rmt.h](#file-includeled_strip_rmth)
- [include/led_strip_spi.h](#file-includeled_strip_spih)
- [include/led_strip_types.h](#file-includeled_strip_typesh)
- [interface/led_strip_interface.h](#file-interfaceled_strip_interfaceh)

## File include/led_strip.h

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**led\_strip\_clear**](#function-led_strip_clear) ([**led\_strip\_handle\_t**](#struct-led_strip_t) strip) <br>_Clear LED strip (turn off all LEDs)_ |
|  esp\_err\_t | [**led\_strip\_del**](#function-led_strip_del) ([**led\_strip\_handle\_t**](#struct-led_strip_t) strip) <br>_Free LED strip resources._ |
|  esp\_err\_t | [**led\_strip\_refresh**](#function-led_strip_refresh) ([**led\_strip\_handle\_t**](#struct-led_strip_t) strip) <br>_Refresh memory colors to LEDs._ |
|  esp\_err\_t | [**led\_strip\_set\_pixel**](#function-led_strip_set_pixel) ([**led\_strip\_handle\_t**](#struct-led_strip_t) strip, uint32\_t index, uint32\_t red, uint32\_t green, uint32\_t blue) <br>_Set RGB for a specific pixel._ |
|  esp\_err\_t | [**led\_strip\_set\_pixel\_hsv**](#function-led_strip_set_pixel_hsv) ([**led\_strip\_handle\_t**](#struct-led_strip_t) strip, uint32\_t index, uint16\_t hue, uint8\_t saturation, uint8\_t value) <br>_Set HSV for a specific pixel._ |
|  esp\_err\_t | [**led\_strip\_set\_pixel\_rgbw**](#function-led_strip_set_pixel_rgbw) ([**led\_strip\_handle\_t**](#struct-led_strip_t) strip, uint32\_t index, uint32\_t red, uint32\_t green, uint32\_t blue, uint32\_t white) <br>_Set RGBW for a specific pixel._ |

## Functions Documentation

### function `led_strip_clear`

_Clear LED strip (turn off all LEDs)_

```c
esp_err_t led_strip_clear (
    led_strip_handle_t strip
)
```

**Parameters:**

- `strip` LED strip

**Returns:**

- ESP\_OK: Clear LEDs successfully
- ESP\_FAIL: Clear LEDs failed because some other error occurred

### function `led_strip_del`

_Free LED strip resources._

```c
esp_err_t led_strip_del (
    led_strip_handle_t strip
)
```

**Parameters:**

- `strip` LED strip

**Returns:**

- ESP\_OK: Free resources successfully
- ESP\_FAIL: Free resources failed because error occurred

### function `led_strip_refresh`

_Refresh memory colors to LEDs._

```c
esp_err_t led_strip_refresh (
    led_strip_handle_t strip
)
```

**Parameters:**

- `strip` LED strip

**Returns:**

- ESP\_OK: Refresh successfully
- ESP\_FAIL: Refresh failed because some other error occurred

**Note:**

: After updating the LED colors in the memory, a following invocation of this API is needed to flush colors to strip.

### function `led_strip_set_pixel`

_Set RGB for a specific pixel._

```c
esp_err_t led_strip_set_pixel (
    led_strip_handle_t strip,
    uint32_t index,
    uint32_t red,
    uint32_t green,
    uint32_t blue
)
```

**Parameters:**

- `strip` LED strip
- `index` index of pixel to set
- `red` red part of color
- `green` green part of color
- `blue` blue part of color

**Returns:**

- ESP\_OK: Set RGB for a specific pixel successfully
- ESP\_ERR\_INVALID\_ARG: Set RGB for a specific pixel failed because of invalid parameters
- ESP\_FAIL: Set RGB for a specific pixel failed because other error occurred

### function `led_strip_set_pixel_hsv`

_Set HSV for a specific pixel._

```c
esp_err_t led_strip_set_pixel_hsv (
    led_strip_handle_t strip,
    uint32_t index,
    uint16_t hue,
    uint8_t saturation,
    uint8_t value
)
```

**Parameters:**

- `strip` LED strip
- `index` index of pixel to set
- `hue` hue part of color (0 - 360)
- `saturation` saturation part of color (0 - 255)
- `value` value part of color (0 - 255)

**Returns:**

- ESP\_OK: Set HSV color for a specific pixel successfully
- ESP\_ERR\_INVALID\_ARG: Set HSV color for a specific pixel failed because of an invalid argument
- ESP\_FAIL: Set HSV color for a specific pixel failed because other error occurred

### function `led_strip_set_pixel_rgbw`

_Set RGBW for a specific pixel._

```c
esp_err_t led_strip_set_pixel_rgbw (
    led_strip_handle_t strip,
    uint32_t index,
    uint32_t red,
    uint32_t green,
    uint32_t blue,
    uint32_t white
)
```

**Note:**

Only call this function if your led strip does have the white component (e.g. SK6812-RGBW)

**Note:**

Also see `led_strip_set_pixel` if you only want to specify the RGB part of the color and bypass the white component

**Parameters:**

- `strip` LED strip
- `index` index of pixel to set
- `red` red part of color
- `green` green part of color
- `blue` blue part of color
- `white` separate white component

**Returns:**

- ESP\_OK: Set RGBW color for a specific pixel successfully
- ESP\_ERR\_INVALID\_ARG: Set RGBW color for a specific pixel failed because of an invalid argument
- ESP\_FAIL: Set RGBW color for a specific pixel failed because other error occurred

## File include/led_strip_rmt.h

## Structures and Types

| Type | Name |
| ---: | :--- |
| struct | [**led\_strip\_rmt\_config\_t**](#struct-led_strip_rmt_config_t) <br>_LED Strip RMT specific configuration._ |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**led\_strip\_new\_rmt\_device**](#function-led_strip_new_rmt_device) (const [**led\_strip\_config\_t**](#struct-led_strip_config_t) \*led\_config, const [**led\_strip\_rmt\_config\_t**](#struct-led_strip_rmt_config_t) \*rmt\_config, [**led\_strip\_handle\_t**](#struct-led_strip_t) \*ret\_strip) <br>_Create LED strip based on RMT TX channel._ |

## Structures and Types Documentation

### struct `led_strip_rmt_config_t`

_LED Strip RMT specific configuration._

Variables:

- rmt\_clock\_source\_t clk_src  <br>RMT clock source

- struct [**led\_strip\_rmt\_config\_t**](#struct-led_strip_rmt_config_t) flags  <br>Extra driver flags

- size\_t mem_block_symbols  <br>How many RMT symbols can one RMT channel hold at one time. Set to 0 will fallback to use the default size.

- uint32\_t resolution_hz  <br>RMT tick resolution, if set to zero, a default resolution (10MHz) will be applied

- uint32\_t with_dma  <br>Use DMA to transmit data

## Functions Documentation

### function `led_strip_new_rmt_device`

_Create LED strip based on RMT TX channel._

```c
esp_err_t led_strip_new_rmt_device (
    const led_strip_config_t *led_config,
    const led_strip_rmt_config_t *rmt_config,
    led_strip_handle_t *ret_strip
)
```

**Parameters:**

- `led_config` LED strip configuration
- `rmt_config` RMT specific configuration
- `ret_strip` Returned LED strip handle

**Returns:**

- ESP\_OK: create LED strip handle successfully
- ESP\_ERR\_INVALID\_ARG: create LED strip handle failed because of invalid argument
- ESP\_ERR\_NO\_MEM: create LED strip handle failed because of out of memory
- ESP\_FAIL: create LED strip handle failed because some other error

## File include/led_strip_spi.h

## Structures and Types

| Type | Name |
| ---: | :--- |
| struct | [**led\_strip\_spi\_config\_t**](#struct-led_strip_spi_config_t) <br>_LED Strip SPI specific configuration._ |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**led\_strip\_new\_spi\_device**](#function-led_strip_new_spi_device) (const [**led\_strip\_config\_t**](#struct-led_strip_config_t) \*led\_config, const [**led\_strip\_spi\_config\_t**](#struct-led_strip_spi_config_t) \*spi\_config, [**led\_strip\_handle\_t**](#struct-led_strip_t) \*ret\_strip) <br>_Create LED strip based on SPI MOSI channel._ |

## Structures and Types Documentation

### struct `led_strip_spi_config_t`

_LED Strip SPI specific configuration._

Variables:

- spi\_clock\_source\_t clk_src  <br>SPI clock source

- struct [**led\_strip\_spi\_config\_t**](#struct-led_strip_spi_config_t) flags  <br>Extra driver flags

- spi\_host\_device\_t spi_bus  <br>SPI bus ID. Which buses are available depends on the specific chip

- uint32\_t with_dma  <br>Use DMA to transmit data

## Functions Documentation

### function `led_strip_new_spi_device`

_Create LED strip based on SPI MOSI channel._

```c
esp_err_t led_strip_new_spi_device (
    const led_strip_config_t *led_config,
    const led_strip_spi_config_t *spi_config,
    led_strip_handle_t *ret_strip
)
```

**Note:**

Although only the MOSI line is used for generating the signal, the whole SPI bus can't be used for other purposes.

**Parameters:**

- `led_config` LED strip configuration
- `spi_config` SPI specific configuration
- `ret_strip` Returned LED strip handle

**Returns:**

- ESP\_OK: create LED strip handle successfully
- ESP\_ERR\_INVALID\_ARG: create LED strip handle failed because of invalid argument
- ESP\_ERR\_NOT\_SUPPORTED: create LED strip handle failed because of unsupported configuration
- ESP\_ERR\_NO\_MEM: create LED strip handle failed because of out of memory
- ESP\_FAIL: create LED strip handle failed because some other error

## File include/led_strip_types.h

## Structures and Types

| Type | Name |
| ---: | :--- |
| enum  | [**led\_model\_t**](#enum-led_model_t)  <br>_LED strip model._ |
| enum  | [**led\_pixel\_format\_t**](#enum-led_pixel_format_t)  <br>_LED strip pixel format._ |
| struct | [**led\_strip\_config\_t**](#struct-led_strip_config_t) <br>_LED Strip Configuration._ |
| typedef struct [**led\_strip\_t**](#struct-led_strip_t) \* | [**led\_strip\_handle\_t**](#typedef-led_strip_handle_t)  <br>_LED strip handle._ |

## Structures and Types Documentation

### enum `led_model_t`

_LED strip model._

```c
enum led_model_t {
    LED_MODEL_WS2812,
    LED_MODEL_SK6812,
    LED_MODEL_INVALID
};
```

**Note:**

Different led model may have different timing parameters, so we need to distinguish them.

### enum `led_pixel_format_t`

_LED strip pixel format._

```c
enum led_pixel_format_t {
    LED_PIXEL_FORMAT_GRB,
    LED_PIXEL_FORMAT_GRBW,
    LED_PIXEL_FORMAT_INVALID
};
```

### struct `led_strip_config_t`

_LED Strip Configuration._

Variables:

- struct [**led\_strip\_config\_t**](#struct-led_strip_config_t) flags  <br>Extra driver flags

- uint32\_t invert_out  <br>Invert output signal

- led\_model\_t led_model  <br>LED model

- led\_pixel\_format\_t led_pixel_format  <br>LED pixel format

- uint32\_t max_leds  <br>Maximum LEDs in a single strip

- int strip_gpio_num  <br>GPIO number that used by LED strip

### typedef `led_strip_handle_t`

_LED strip handle._

```c
typedef struct led_strip_t* led_strip_handle_t;
```

## File interface/led_strip_interface.h

## Structures and Types

| Type | Name |
| ---: | :--- |
| struct | [**led\_strip\_t**](#struct-led_strip_t) <br>_LED strip interface definition._ |
| typedef struct [**led\_strip\_t**](#struct-led_strip_t) | [**led\_strip\_t**](#typedef-led_strip_t)  <br> |

## Structures and Types Documentation

### struct `led_strip_t`

_LED strip interface definition._

Variables:

- esp\_err\_t(\* clear  <br>_Clear LED strip (turn off all LEDs)_<br>**Parameters:**

- `strip` LED strip
- `timeout_ms` timeout value for clearing task

**Returns:**

- ESP\_OK: Clear LEDs successfully
- ESP\_FAIL: Clear LEDs failed because some other error occurred

- esp\_err\_t(\* del  <br>_Free LED strip resources._<br>**Parameters:**

- `strip` LED strip

**Returns:**

- ESP\_OK: Free resources successfully
- ESP\_FAIL: Free resources failed because error occurred

- esp\_err\_t(\* refresh  <br>_Refresh memory colors to LEDs._<br>**Parameters:**

- `strip` LED strip
- `timeout_ms` timeout value for refreshing task

**Returns:**

- ESP\_OK: Refresh successfully
- ESP\_FAIL: Refresh failed because some other error occurred

**Note:**

: After updating the LED colors in the memory, a following invocation of this API is needed to flush colors to strip.

- esp\_err\_t(\* set_pixel  <br>_Set RGB for a specific pixel._<br>**Parameters:**

- `strip` LED strip
- `index` index of pixel to set
- `red` red part of color
- `green` green part of color
- `blue` blue part of color

**Returns:**

- ESP\_OK: Set RGB for a specific pixel successfully
- ESP\_ERR\_INVALID\_ARG: Set RGB for a specific pixel failed because of invalid parameters
- ESP\_FAIL: Set RGB for a specific pixel failed because other error occurred

- esp\_err\_t(\* set_pixel_rgbw  <br>_Set RGBW for a specific pixel. Similar to_ `set_pixel`_but also set the white component._<br>**Parameters:**

- `strip` LED strip
- `index` index of pixel to set
- `red` red part of color
- `green` green part of color
- `blue` blue part of color
- `white` separate white component

**Returns:**

- ESP\_OK: Set RGBW color for a specific pixel successfully
- ESP\_ERR\_INVALID\_ARG: Set RGBW color for a specific pixel failed because of an invalid argument
- ESP\_FAIL: Set RGBW color for a specific pixel failed because other error occurred

### typedef `led_strip_t`

```c
typedef struct led_strip_t led_strip_t;
```

Type of LED strip
