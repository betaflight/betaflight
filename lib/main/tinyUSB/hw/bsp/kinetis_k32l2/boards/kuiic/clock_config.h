#ifndef CLOCK_CONFIG_H
#define CLOCK_CONFIG_H

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SIM_OSC32KSEL_LPO_CLK         3U        /*!< OSC32KSEL select: LPO clock */
#define SOPT5_LPUART1RXSRC_LPUART_RX  0x00u     /*!<@brief LPUART1 Receive Data Source Select: LPUART_RX pin */
#define SOPT5_LPUART1TXSRC_LPUART_TX  0x00u     /*!<@brief LPUART1 Transmit Data Source Select: LPUART_TX pin */
#define BOARD_BOOTCLOCKRUN_CORE_CLOCK 48000000U /*!< Core clock frequency: 48000000Hz */

void BOARD_BootClockRUN(void);

#endif
