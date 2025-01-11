#ifndef _CLOCK_CONFIG_H_
#define _CLOCK_CONFIG_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_XTAL0_CLK_HZ                         24000000U  /*!< Board xtal0 frequency in Hz */

#define BOARD_XTAL32K_CLK_HZ                          32768U  /*!< Board xtal32k frequency in Hz */
/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes default configuration of clocks.
 *
 */
void BOARD_InitBootClocks(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/*******************************************************************************
 * Definitions for BOARD_BootClockRUN configuration
 ******************************************************************************/
#define BOARD_BOOTCLOCKRUN_CORE_CLOCK             500000000U  /*!< Core clock frequency: 500000000Hz */

/* Clock outputs (values are in Hz): */
#define BOARD_BOOTCLOCKRUN_ADC_ALT_CLK                40000000UL     /* Clock consumers of ADC_ALT_CLK output : N/A */
#define BOARD_BOOTCLOCKRUN_CKIL_SYNC_CLK_ROOT         32768UL        /* Clock consumers of CKIL_SYNC_CLK_ROOT output : CSU, EWM, GPT1, GPT2, KPP, PIT, RTWDOG, SNVS, SPDIF, TEMPMON, USB, WDOG1, WDOG2 */
#define BOARD_BOOTCLOCKRUN_CLKO1_CLK                  0UL            /* Clock consumers of CLKO1_CLK output : N/A */
#define BOARD_BOOTCLOCKRUN_CLKO2_CLK                  0UL            /* Clock consumers of CLKO2_CLK output : N/A */
#define BOARD_BOOTCLOCKRUN_CLK_1M                     1000000UL      /* Clock consumers of CLK_1M output : EWM, RTWDOG */
#define BOARD_BOOTCLOCKRUN_CLK_24M                    24000000UL     /* Clock consumers of CLK_24M output : GPT1, GPT2 */
#define BOARD_BOOTCLOCKRUN_CORE_CLK_ROOT              500000000UL    /* Clock consumers of CORE_CLK_ROOT output : ARM, FLEXSPI */
#define BOARD_BOOTCLOCKRUN_ENET_500M_REF_CLK          500000000UL    /* Clock consumers of ENET_500M_REF_CLK output : N/A */
#define BOARD_BOOTCLOCKRUN_FLEXIO1_CLK_ROOT           30000000UL     /* Clock consumers of FLEXIO1_CLK_ROOT output : FLEXIO1 */
#define BOARD_BOOTCLOCKRUN_FLEXSPI_CLK_ROOT           132000000UL    /* Clock consumers of FLEXSPI_CLK_ROOT output : FLEXSPI */
#define BOARD_BOOTCLOCKRUN_GPT1_IPG_CLK_HIGHFREQ      62500000UL     /* Clock consumers of GPT1_ipg_clk_highfreq output : GPT1 */
#define BOARD_BOOTCLOCKRUN_GPT2_IPG_CLK_HIGHFREQ      62500000UL     /* Clock consumers of GPT2_ipg_clk_highfreq output : GPT2 */
#define BOARD_BOOTCLOCKRUN_IPG_CLK_ROOT               125000000UL    /* Clock consumers of IPG_CLK_ROOT output : ADC1, ADC_ETC, AIPSTZ1, AIPSTZ2, AOI, ARM, CCM, CSU, DCDC, DCP, DMA0, DMAMUX, EWM, FLEXIO1, FLEXRAM, FLEXSPI, GPC, GPIO1, GPIO2, GPIO5, IOMUXC, KPP, LPI2C1, LPI2C2, LPSPI1, LPSPI2, LPUART1, LPUART2, LPUART3, LPUART4, OCOTP, PWM1, RTWDOG, SAI1, SAI3, SNVS, SPDIF, SRC, TEMPMON, TRNG, USB, WDOG1, WDOG2, XBARA */
#define BOARD_BOOTCLOCKRUN_LPI2C_CLK_ROOT             60000000UL     /* Clock consumers of LPI2C_CLK_ROOT output : LPI2C1, LPI2C2 */
#define BOARD_BOOTCLOCKRUN_LPSPI_CLK_ROOT             105600000UL    /* Clock consumers of LPSPI_CLK_ROOT output : LPSPI1, LPSPI2 */
#define BOARD_BOOTCLOCKRUN_MQS_MCLK                   63529411UL     /* Clock consumers of MQS_MCLK output : N/A */
#define BOARD_BOOTCLOCKRUN_PERCLK_CLK_ROOT            62500000UL     /* Clock consumers of PERCLK_CLK_ROOT output : GPT1, GPT2, PIT */
#define BOARD_BOOTCLOCKRUN_SAI1_CLK_ROOT              63529411UL     /* Clock consumers of SAI1_CLK_ROOT output : N/A */
#define BOARD_BOOTCLOCKRUN_SAI1_MCLK1                 63529411UL     /* Clock consumers of SAI1_MCLK1 output : SAI1 */
#define BOARD_BOOTCLOCKRUN_SAI1_MCLK2                 63529411UL     /* Clock consumers of SAI1_MCLK2 output : SAI1 */
#define BOARD_BOOTCLOCKRUN_SAI1_MCLK3                 30000000UL     /* Clock consumers of SAI1_MCLK3 output : SAI1 */
#define BOARD_BOOTCLOCKRUN_SAI3_CLK_ROOT              63529411UL     /* Clock consumers of SAI3_CLK_ROOT output : N/A */
#define BOARD_BOOTCLOCKRUN_SAI3_MCLK1                 63529411UL     /* Clock consumers of SAI3_MCLK1 output : SAI3 */
#define BOARD_BOOTCLOCKRUN_SAI3_MCLK2                 0UL            /* Clock consumers of SAI3_MCLK2 output : SAI3 */
#define BOARD_BOOTCLOCKRUN_SAI3_MCLK3                 30000000UL     /* Clock consumers of SAI3_MCLK3 output : SAI3 */
#define BOARD_BOOTCLOCKRUN_SPDIF0_CLK_ROOT            30000000UL     /* Clock consumers of SPDIF0_CLK_ROOT output : SPDIF */
#define BOARD_BOOTCLOCKRUN_SPDIF0_EXTCLK_OUT          0UL            /* Clock consumers of SPDIF0_EXTCLK_OUT output : SPDIF */
#define BOARD_BOOTCLOCKRUN_TRACE_CLK_ROOT             132000000UL    /* Clock consumers of TRACE_CLK_ROOT output : ARM */
#define BOARD_BOOTCLOCKRUN_UART_CLK_ROOT              80000000UL     /* Clock consumers of UART_CLK_ROOT output : LPUART1, LPUART2, LPUART3, LPUART4 */
#define BOARD_BOOTCLOCKRUN_USBPHY_CLK                 480000000UL    /* Clock consumers of USBPHY_CLK output : TEMPMON, USB */

/*! @brief Usb1 PLL set for BOARD_BootClockRUN configuration.
 */
extern const clock_usb_pll_config_t usb1PllConfig_BOARD_BootClockRUN;
/*! @brief Sys PLL for BOARD_BootClockRUN configuration.
 */
extern const clock_sys_pll_config_t sysPllConfig_BOARD_BootClockRUN;
/*! @brief Enet PLL set for BOARD_BootClockRUN configuration.
 */
extern const clock_enet_pll_config_t enetPllConfig_BOARD_BootClockRUN;

/*******************************************************************************
 * API for BOARD_BootClockRUN configuration
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes configuration of clocks.
 *
 */
void BOARD_BootClockRUN(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

#endif /* _CLOCK_CONFIG_H_ */
