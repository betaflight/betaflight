/**
  * @file        apm32f4xx_dal_log.h
  * @brief       Header file containing functions prototypes of LOG DAL library.
  *
  * @attention
  *
  *  Copyright (C) 2023 Geehy Semiconductor
  *
  *  You may not use this file except in compliance with the
  *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
  *
  *  The program is only for reference, which is distributed in the hope
  *  that it will be useful and instructional for customers to develop
  *  their software. Unless required by applicable law or agreed to in
  *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
  *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
  *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
  *  and limitations under the License.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4XX_DAL_LOG_H
#define APM32F4XX_DAL_LOG_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"
#include <stdarg.h>

/** @addtogroup APM32F4xx_DAL_Driver
  @{
*/

/** @defgroup LOG
  @{
*/

/* Exported macro ------------------------------------------------------------*/
/** @defgroup LOG_Exported_Macros LOG Exported Macros
  * @{
  */

#if !defined (USE_LOG_COMPONENT)
#define USE_LOG_COMPONENT           0U
#endif

#define LOG_COLOR_BLACK             "30"
#define LOG_COLOR_RED               "31"
#define LOG_COLOR_GREEN             "32"
#define LOG_COLOR_BROWN             "33"
#define LOG_COLOR_BLUE              "34"
#define LOG_COLOR_PURPLE            "35"
#define LOG_COLOR_CYAN              "36"
#define LOG_COLOR_WHITE             "37"

#define LOG_BG_COLOR_BLACK          "40"
#define LOG_BG_COLOR_RED            "41"
#define LOG_BG_COLOR_GREEN          "42"
#define LOG_BG_COLOR_BROWN          "43"
#define LOG_BG_COLOR_BLUE           "44"
#define LOG_BG_COLOR_PURPLE         "45"
#define LOG_BG_COLOR_CYAN           "46"
#define LOG_BG_COLOR_WHITE          "47"

#define LOG_COLOR(COLOR)            "\033[0;" COLOR "m"
#define LOG_BOLD(COLOR)             "\033[1;" COLOR "m"
#define LOG_BG_BLK_COLOR(COLOR)     "\033[40;" COLOR "m"
#define LOG_COLOR_RESET             "\033[0m"

#define LOG_COLOR_DEBUG             LOG_COLOR(LOG_COLOR_GREEN)
#define LOG_COLOR_INFO              LOG_COLOR(LOG_COLOR_BLUE)
#define LOG_COLOR_WARNING           LOG_COLOR(LOG_COLOR_BROWN)
#define LOG_COLOR_ERROR             LOG_COLOR(LOG_COLOR_RED)

#define LOG_FORMAT(letter, format)  LOG_COLOR_ ## letter "[%s] " format LOG_COLOR_RESET

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup LOG_Exported_Types LOG Exported Types
  * @{
  */

/**
 * @brief DAL log level
 */
typedef enum 
{
    DAL_LOG_NONE,
    DAL_LOG_DEBUG,
    DAL_LOG_INFO,
    DAL_LOG_WARNING,
    DAL_LOG_ERROR,
} DAL_LOG_LEVEL_T;
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup LOG_Exported_Functions
  * @{
  */

typedef int (*LOG_FUNC_T)(const char *, va_list);

/* Control functions */
void DAL_LOG_SetCallback(LOG_FUNC_T callback);
void DAL_LOG_Print(const char *format, ...);

#define DAL_LOG_LEVEL(level, tag, format, ...)                                      \
    do {                                                                            \
            if (level == DAL_LOG_INFO)                                              \
            {                                                                       \
                DAL_LOG_Print(LOG_FORMAT(INFO, format), tag, ##__VA_ARGS__);        \
            }                                                                       \
            else if (level == DAL_LOG_DEBUG)                                        \
            {                                                                       \
                DAL_LOG_Print(LOG_FORMAT(DEBUG, format), tag, ##__VA_ARGS__);       \
            }                                                                       \
            else if (level == DAL_LOG_WARNING)                                      \
            {                                                                       \
                DAL_LOG_Print(LOG_FORMAT(WARNING, format), tag, ##__VA_ARGS__);     \
            }                                                                       \
            else if (level == DAL_LOG_ERROR)                                        \
            {                                                                       \
                DAL_LOG_Print(LOG_FORMAT(ERROR, format), tag, ##__VA_ARGS__);       \
            }                                                                       \
    } while(0U)

#if (USE_LOG_COMPONENT == 1U)
    #define DAL_LOGI(tag, format, ...)      DAL_LOG_LEVEL(DAL_LOG_INFO, tag, format, ##__VA_ARGS__)
    #define DAL_LOGE(tag, format, ...)      DAL_LOG_LEVEL(DAL_LOG_ERROR, tag, format, ##__VA_ARGS__)
    #define DAL_LOGW(tag, format, ...)      DAL_LOG_LEVEL(DAL_LOG_WARNING, tag, format, ##__VA_ARGS__)
    #define DAL_LOGD(tag, format, ...)      DAL_LOG_LEVEL(DAL_LOG_DEBUG, tag, format, ##__VA_ARGS__)
#else
    #define DAL_LOGI(tag, format, ...)      ((void)0U)
    #define DAL_LOGE(tag, format, ...)      ((void)0U)
    #define DAL_LOGW(tag, format, ...)      ((void)0U)
    #define DAL_LOGD(tag, format, ...)      ((void)0U)
#endif /* DAL_LOG_ENABLE */

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4XX_DAL_LOG_H */
