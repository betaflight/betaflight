/**
  **************************************************************************
  * @file     usbh_hid_keyboard.c
  * @brief    usb host hid keyboard type
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
 #include "usbh_hid_keyboard.h"

/** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @defgroup USBH_hid_class_keyboard
  * @brief usb host class hid keyboard
  * @{
  */

/** @defgroup USBH_hid_class_keyboard_private_functions
  * @{
  */

static const uint8_t hid_keyboard_codes[] =
{
  0,    0,     0,     0,     31,    50,    48,   33,
  19,   34,    35,    36,    24,    37,    38,   39,
  52,   51,    25,    26,    17,    20,    32,   21,
  23,   49,    18,    47,    22,    46,    2,    3,
  4,    5,     6,     7,     8,     9,     10,   11,
  43,   110,   15,    16,    61,    12,    13,   27,
  28,   29,    42,    40,    41,    1,     53,   54,
  55,   30,    112,   113,   114,   115,   116,  117,
  118,  119,   120,   121,   122,   123,   124,  125,
  126,  75,    80,    85,    76,    81,    86,   89,
  79,   84,    83,    90,    95,    100,   105,  106,
  108,  93,    98,    103,   92,    97,    102,  91,
  96,   101,   99,    104,   45,    129,   0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     107,   0,    56,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  0,    0,     0,     0,     0,     0,     0,    0,
  58,   44,    60,    127,   64,    57,    62,   128,
};

#ifdef QWERTY_KEYBOARD

static const int8_t hid_keyboard_key[] = {
  '\0',  '`',  '1',  '2',  '3',  '4',  '5',  '6',
  '7',  '8',  '9',  '0',  '-',  '=',  '\0', '\r',
  '\t',  'q',  'w',  'e',  'r',  't',  'y',  'u',
  'i',  'o',  'p',  '[',  ']',  '\\',
  '\0',  'a',  's',  'd',  'f',  'g',  'h',  'j',
  'k',  'l',  ';',  '\'', '\0', '\n',
  '\0',  '\0', 'z',  'x',  'c',  'v',  'b',  'n',
  'm',  ',',  '.',  '/',  '\0', '\0',
  '\0',  '\0', '\0', ' ',  '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0',  '\0', '\0', '\0', '\0', '\r', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0',
  '\0',  '\0', '7',  '4',  '1',
  '\0',  '/',  '8',  '5',  '2',
  '0',   '*',  '9',  '6',  '3',
  '.',   '-',  '+',  '\0', '\n', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0',  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0'
};

static const int8_t hid_keyboard_key_shift[] = {
  '\0', '~',  '!',  '@',  '#',  '$',  '%',  '^',  '&',  '*',  '(',  ')',
  '_',  '+',  '\0', '\0', '\0', 'Q',  'W',  'E',  'R',  'T',  'Y',  'U',
  'I',  'O',  'P',  '{',  '}',  '|',  '\0', 'A',  'S',  'D',  'F',  'G',
  'H',  'J',  'K',  'L',  ':',  '"',  '\0', '\n', '\0', '\0', 'Z',  'X',
  'C',  'V',  'B',  'N',  'M',  '<',  '>',  '?',  '\0', '\0',  '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0',    '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'
};

#else
static const int8_t hid_keyboard_key[] = {
  '\0',  '`',  '1',  '2',  '3',  '4',  '5',  '6',  '7',  '8',  '9',  '0',
  '-',  '=',  '\0', '\r', '\t',  'a',  'z',  'e',  'r',  't',  'y',  'u',
  'i',  'o',  'p',  '[',  ']', '\\', '\0',  'q',  's',  'd',  'f',  'g',
  'h',  'j',  'k',  'l',  'm',  '\0', '\0', '\n', '\0',  '\0', 'w',  'x',
  'c',  'v',  'b',  'n',  ',',  ';',  ':',  '!',  '\0', '\0', '\0',  '\0',
  '\0', ' ',  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0',  '\0', '\0', '\0', '\0', '\r', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0',  '\0', '7',  '4',  '1','\0',  '/',
  '8',  '5',  '2', '0',   '*',  '9',  '6',  '3', '.',   '-',  '+',  '\0',
  '\n', '\0', '\0', '\0', '\0', '\0', '\0','\0',  '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'
};

static const int8_t hid_keyboard_key_shift[] = {
  '\0', '~',  '!',  '@',  '#',  '$',  '%',  '^',  '&',  '*',  '(',  ')',  '_',
  '+',  '\0', '\0', '\0', 'A',  'Z',  'E',  'R',  'T',  'Y',  'U',  'I',  'O',
  'P',  '{',  '}',  '*', '\0', 'Q',  'S',  'D',  'F',  'G',  'H',  'J',  'K',
  'L',  'M',  '%',  '\0', '\n', '\0', '\0', 'W',  'X',  'C',  'V',  'B',  'N',
  '?',  '.',  '/',  '\0',  '\0', '\0','\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
  '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'
};
#endif

/**
  * @brief  usb host hid keyboard decode
  * @param  data: keyboard data
  * @retval none
  */
void usbh_hid_keyboard_decode(uint8_t *data)
{
  static uint8_t shift;
  static uint8_t keys[KEYBOARD_MAX_NB_PRESSED];
  static uint8_t keys_n[KEYBOARD_MAX_NB_PRESSED];
  static uint8_t keys_l[KEYBOARD_MAX_NB_PRESSED];
  static uint8_t key_newest;
  static uint8_t nb_keys;
  static uint8_t nb_keys_n;
  static uint8_t nb_keys_l;

  uint8_t idx;
  uint8_t idy;
  uint8_t err;
  uint8_t out;

  nb_keys = 0;
  nb_keys_n = 0;
  nb_keys_l = 0;
  key_newest = 0;

  if(data[0] == KEYBOARD_LEFT_SHIFT || data[0] == KEYBOARD_RIGHT_SHIFT)
  {
    shift = TRUE;
  }
  else
  {
    shift = FALSE;
  }
  err = FALSE;

  for(idx = 2; idx < 2 + KEYBOARD_MAX_NB_PRESSED; idx ++)
  {
    if((data[idx] == 0x01) ||
       (data[idx] == 0x02) ||
       (data[idx] == 0x03))
    {
      err = TRUE;
    }
  }

  if(err == TRUE)
  {
    return;
  }

  nb_keys = 0;
  nb_keys_n = 0;
  for(idx = 2; idx < 2 + KEYBOARD_MAX_NB_PRESSED; idx ++)
  {
    if(data[idx] != 0)
    {
      keys[nb_keys] = data[idx];
      nb_keys ++;
      for(idy = 0; idy < nb_keys_l; idy ++)
      {
        if(data[idx] == keys_l[idy])
        {
          break;
        }
      }

      if(idy == nb_keys_l)
      {
        keys_n[nb_keys_n] = data[idx];
        nb_keys_n ++;
      }

    }
  }

  if(nb_keys_n == 1)
  {
    key_newest = keys_n[0];

    if(shift == TRUE)
    {
      out = hid_keyboard_key_shift[hid_keyboard_codes[key_newest]];
    }
    else
    {
      out = hid_keyboard_key[hid_keyboard_codes[key_newest]];
    }
    /* callback user handler */
    USBH_DEBUG("%c", out);
  }
  else
  {
    key_newest = 0;
  }

  nb_keys_l = nb_keys;
  for(idx = 0; idx < KEYBOARD_MAX_NB_PRESSED; idx ++)
  {
    keys_l[idx] = keys[idx];
  }
  return;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

