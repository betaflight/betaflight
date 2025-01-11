/* Auto-generated config file peripheral_clk_config.h */
#ifndef PERIPHERAL_CLK_CONFIG_H
#define PERIPHERAL_CLK_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

/**
 * \def CONF_HCLK_FREQUENCY
 * \brief HCLK's Clock frequency
 */
#ifndef CONF_HCLK_FREQUENCY
#define CONF_HCLK_FREQUENCY 300000000
#endif

/**
 * \def CONF_FCLK_FREQUENCY
 * \brief FCLK's Clock frequency
 */
#ifndef CONF_FCLK_FREQUENCY
#define CONF_FCLK_FREQUENCY 300000000
#endif

/**
 * \def CONF_CPU_FREQUENCY
 * \brief CPU's Clock frequency
 */
#ifndef CONF_CPU_FREQUENCY
#define CONF_CPU_FREQUENCY 300000000
#endif

/**
 * \def CONF_SLCK_FREQUENCY
 * \brief Slow Clock frequency
 */
#define CONF_SLCK_FREQUENCY 0

/**
 * \def CONF_MCK_FREQUENCY
 * \brief Master Clock frequency
 */
#define CONF_MCK_FREQUENCY 150000000

/**
 * \def CONF_PCK6_FREQUENCY
 * \brief Programmable Clock Controller 6 frequency
 */
#define CONF_PCK6_FREQUENCY 1714285

// <h> USART Clock Settings
// <o> USART Clock source

// <0=> Master Clock (MCK)
// <1=> MCK / 8 for USART
// <2=> Programmable Clock Controller 4 (PMC_PCK4)
// <3=> External Clock
// <i> This defines the clock source for the USART
// <id> usart_clock_source
#ifndef CONF_USART1_CK_SRC
#define CONF_USART1_CK_SRC 0
#endif

// <o> USART External Clock Input on SCK <1-4294967295>
// <i> Inputs the external clock frequency on SCK
// <id> usart_clock_freq
#ifndef CONF_USART1_SCK_FREQ
#define CONF_USART1_SCK_FREQ 10000000
#endif

// </h>

/**
 * \def USART FREQUENCY
 * \brief USART's Clock frequency
 */
#ifndef CONF_USART1_FREQUENCY
#define CONF_USART1_FREQUENCY 150000000
#endif

#ifndef CONF_SRC_USB_480M
#define CONF_SRC_USB_480M 0
#endif

#ifndef CONF_SRC_USB_48M
#define CONF_SRC_USB_48M 1
#endif

// <y> USB Full/Low Speed Clock
// <CONF_SRC_USB_48M"> USB Clock Controller (USB_48M)
// <id> usb_fsls_clock_source
// <i> 48MHz clock source for low speed and full speed.
// <i> It must be available when low speed is supported by host driver.
// <i> It must be available when low power mode is selected.
#ifndef CONF_USBHS_FSLS_SRC
#define CONF_USBHS_FSLS_SRC CONF_SRC_USB_48M
#endif

// <y> USB Clock Source(Normal/Low-power Mode Selection)
// <CONF_SRC_USB_480M"> USB High Speed Clock (USB_480M)
// <CONF_SRC_USB_48M"> USB Clock Controller (USB_48M)
// <id> usb_clock_source
// <i> Select the clock source for USB.
// <i> In normal mode, use "USB High Speed Clock (USB_480M)".
// <i> In low-power mode, use "USB Clock Controller (USB_48M)".
#ifndef CONF_USBHS_SRC
#define CONF_USBHS_SRC CONF_SRC_USB_480M
#endif

/**
 * \def CONF_USBHS_FSLS_FREQUENCY
 * \brief USBHS's Full/Low Speed Clock Source frequency
 */
#ifndef CONF_USBHS_FSLS_FREQUENCY
#define CONF_USBHS_FSLS_FREQUENCY 48000000
#endif

/**
 * \def CONF_USBHS_FREQUENCY
 * \brief USBHS's Selected Clock Source frequency
 */
#ifndef CONF_USBHS_FREQUENCY
#define CONF_USBHS_FREQUENCY 480000000
#endif

// <<< end of configuration section >>>

#endif // PERIPHERAL_CLK_CONFIG_H
