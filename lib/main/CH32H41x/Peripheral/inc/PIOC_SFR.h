/* Define for PIOC           */
/* Website:  http://wch.cn   */
/* Email:    tech@wch.cn     */
/* Author:   W.ch 2022.08    */
/* V1.0 SpecialFunctionRegister */

// __PIOC_SFR_H__

#ifndef __PIOC_SFR_H__
#define __PIOC_SFR_H__

#ifdef __cplusplus
extern "C" {
#endif

// Register Bit Attribute / Bit Access Type
//   RO:    Read Only (internal change)
//   RW:    Read / Write
// Attribute: master/PIOC

/* Register name rule:
   R32_* for 32 bits register (UINT32,ULONG)
   R16_* for 16 bits register (UINT16,USHORT)
   R8_*  for  8 bits register (UINT8,UCHAR)
   RB_*  for bit or bit mask of 8 bit register */

/* ********************************************************************************************************************* */

#define PIOC_SRAM_BASE      (0x50040000)        // PIOC code RAM base address

#define PIOC_SFR_BASE       PIOC_BASE                 // PIOC SFR base address

#define R32_PIOC_SFR        (*((volatile unsigned long *)(PIOC_SFR_BASE+0x04))) // RO/RW, PIOC SFR

#define R8_INDIR_ADDR       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x04))) // RO/RW, PIOC indirect address

#define R8_TMR0_COUNT       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x05))) // RO/RW, PIOC timer count

#define R8_TMR0_CTRL        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x06))) // RO/RW, PIOC timer control and GP bit
#define  RB_EN_LEVEL1       0x80                      // RO/RW, enable IO1 level change to wakeup & action interrupt flag
#define  RB_EN_LEVEL0       0x40                      // RO/RW, enable IO0 level change to wakeup & action interrupt flag
#define  RB_GP_BIT_Y        0x20                      // RO/RW, general-purpose bit 1, reset by power on, no effect if system reset or RB_MST_RESET reset
#define  RB_GP_BIT_X        0x10                      // RO/RW, general-purpose bit 0, reset by power on, no effect if system reset or RB_MST_RESET reset
#define  RB_TMR0_MODE       0x08                      // RO/RW, timer mode: 0-timer, 1-PWM
#define  RB_TMR0_FREQ2      0x04                      // RO/RW, timer clock frequency selection 2
#define  RB_TMR0_FREQ1      0x02                      // RO/RW, timer clock frequency selection 1
#define  RB_TMR0_FREQ0      0x01                      // RO/RW, timer clock frequency selection 0

#define R8_TMR0_INIT        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x07))) // RO/RW, PIOC timer initial value


#define R32_PORT_CFG        (*((volatile unsigned long *)(PIOC_SFR_BASE+0x08))) // RO/RW, port status and config

#define R8_BIT_CYCLE        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x08))) // RO/RW, encode bit cycle
#define  RB_BIT_TX_O0       0x80                      // RO/RW, bit data for IO0 port encode output
#define  RB_BIT_CYCLE       0x7F                      // RO/RW, IO0 port bit data cycle -1

#define R8_INDIR_ADDR2      (*((volatile unsigned char *)(PIOC_SFR_BASE+0x09))) // RO/RW, PIOC indirect address 2

#define R8_PORT_DIR         (*((volatile unsigned char *)(PIOC_SFR_BASE+0x0A))) // RO/RW, IO port direction and mode
//#define  RB_PORT_MOD3       0x80                      // RO/RW, IO port mode 3
//#define  RB_PORT_MOD2       0x40                      // RO/RW, IO port mode 2
//#define  RB_PORT_MOD1       0x20                      // RO/RW, IO port mode 1
//#define  RB_PORT_MOD0       0x10                      // RO/RW, IO port mode 0
//#define  RB_PORT_PU1        0x08                      // RO/RW, IO1 port pullup enable
//#define  RB_PORT_PU0        0x04                      // RO/RW, IO0 port pullup enable
#define  RB_PORT_DIR1       0x02                      // RO/RW, IO1 port direction
#define  RB_PORT_DIR0       0x01                      // RO/RW, IO0 port direction

#define R8_PORT_IO          (*((volatile unsigned char *)(PIOC_SFR_BASE+0x0B))) // RO/RW, IO port input and output
#define  RB_PORT_IN_XOR     0x80                      // RO/RO, IO0 XOR IO1 port input
#define  RB_BIT_RX_I0       0x40                      // RO/RO, decoced bit data for IO0 port received
#define  RB_PORT_IN1        0x20                      // RO/RO, IO1 port input
#define  RB_PORT_IN0        0x10                      // RO/RO, IO0 port input
#define  RB_PORT_XOR1       0x08                      // RO/RO, IO1 port output XOR input
#define  RB_PORT_XOR0       0x04                      // RO/RO, IO0 port output XOR input
#define  RB_PORT_OUT1       0x02                      // RO/RW, IO1 port output
#define  RB_PORT_OUT0       0x01                      // RO/RW, IO0 port output


#define R32_DATA_CTRL       (*((volatile unsigned long *)(PIOC_SFR_BASE+0x1C))) // RW/RW, data control

#define R8_SYS_CFG          (*((volatile unsigned char *)(PIOC_SFR_BASE+0x1C))) // RW/RW, port config
#define  RB_INT_REQ         0x80                      // RO/RW, PIOC interrupt request action, set 1/0 by PIOC, clear 0 by master write R8_CTRL_RD (no effect)
#define  RB_DATA_SW_MR      0x40                      // RO/RO, R8_CTRL_RD wait for read status, set 1 by PIOC write R8_CTRL_RD, clear 0 by master read R8_CTRL_RD
#define  RB_DATA_MW_SR      0x20                      // RO/RO, R8_CTRL_WR wait for read status, set 1 by master write R8_CTRL_WR, clear 0 by PIOC read R8_CTRL_WR
#define  RB_MST_CFG_B4      0x10                      // RW/RO, config inform bit, default 0
#define  RB_MST_IO_EN1      0x08                      // RW/RO, IO1 switch enable, default 0
#define  RB_MST_IO_EN0      0x04                      // RW/RO, IO0 switch enable, default 0
#define  RB_MST_RESET       0x02                      // RW/RO, force PIOC reset, high action, default 0
#define  RB_MST_CLK_GATE    0x01                      // RW/RO, PIOC global clock enable, high action, default 0

#define R8_CTRL_RD          (*((volatile unsigned char *)(PIOC_SFR_BASE+0x1D))) // RO/RW, data for master read only and PIOC write only

#define R8_CTRL_WR          (*((volatile unsigned char *)(PIOC_SFR_BASE+0x1E))) // RW/RO, data for master write only and PIOC read only

#define R8_DATA_EXCH        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x1F))) // RW/RW, data exchange


#define R32_DATA_REG0_3     (*((volatile unsigned long *)(PIOC_SFR_BASE+0x20))) // RW/RW, data buffer 0~3
#define R8_DATA_REG0        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x20))) // RW/RW, data buffer 0
#define R8_DATA_REG1        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x21))) // RW/RW, data buffer 1
#define R8_DATA_REG2        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x22))) // RW/RW, data buffer 2
#define R8_DATA_REG3        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x23))) // RW/RW, data buffer 3

#define R32_DATA_REG4_7     (*((volatile unsigned long *)(PIOC_SFR_BASE+0x24))) // RW/RW, data buffer 4~7
#define R8_DATA_REG4        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x24))) // RW/RW, data buffer 4
#define R8_DATA_REG5        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x25))) // RW/RW, data buffer 5
#define R8_DATA_REG6        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x26))) // RW/RW, data buffer 6
#define R8_DATA_REG7        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x27))) // RW/RW, data buffer 7

#define R32_DATA_REG8_11    (*((volatile unsigned long *)(PIOC_SFR_BASE+0x28))) // RW/RW, data buffer 8~11
#define R8_DATA_REG8        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x28))) // RW/RW, data buffer 8
#define R8_DATA_REG9        (*((volatile unsigned char *)(PIOC_SFR_BASE+0x29))) // RW/RW, data buffer 9
#define R8_DATA_REG10       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x2A))) // RW/RW, data buffer 10
#define R8_DATA_REG11       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x2B))) // RW/RW, data buffer 11

#define R32_DATA_REG12_15   (*((volatile unsigned long *)(PIOC_SFR_BASE+0x2C))) // RW/RW, data buffer 12~15
#define R8_DATA_REG12       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x2C))) // RW/RW, data buffer 12
#define R8_DATA_REG13       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x2D))) // RW/RW, data buffer 13
#define R8_DATA_REG14       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x2E))) // RW/RW, data buffer 14
#define R8_DATA_REG15       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x2F))) // RW/RW, data buffer 15

#define R32_DATA_REG16_19   (*((volatile unsigned long *)(PIOC_SFR_BASE+0x30))) // RW/RW, data buffer 16~19
#define R8_DATA_REG16       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x30))) // RW/RW, data buffer 16
#define R8_DATA_REG17       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x31))) // RW/RW, data buffer 17
#define R8_DATA_REG18       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x32))) // RW/RW, data buffer 18
#define R8_DATA_REG19       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x33))) // RW/RW, data buffer 19

#define R32_DATA_REG20_23   (*((volatile unsigned long *)(PIOC_SFR_BASE+0x34))) // RW/RW, data buffer 20~23
#define R8_DATA_REG20       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x34))) // RW/RW, data buffer 20
#define R8_DATA_REG21       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x35))) // RW/RW, data buffer 21
#define R8_DATA_REG22       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x36))) // RW/RW, data buffer 22
#define R8_DATA_REG23       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x37))) // RW/RW, data buffer 23

#define R32_DATA_REG24_27   (*((volatile unsigned long *)(PIOC_SFR_BASE+0x38))) // RW/RW, data buffer 24~27
#define R8_DATA_REG24       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x38))) // RW/RW, data buffer 24
#define R8_DATA_REG25       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x39))) // RW/RW, data buffer 25
#define R8_DATA_REG26       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x3A))) // RW/RW, data buffer 26
#define R8_DATA_REG27       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x3B))) // RW/RW, data buffer 27

#define R32_DATA_REG28_31   (*((volatile unsigned long *)(PIOC_SFR_BASE+0x3C))) // RW/RW, data buffer 28~31
#define R8_DATA_REG28       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x3C))) // RW/RW, data buffer 28
#define R8_DATA_REG29       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x3D))) // RW/RW, data buffer 29
#define R8_DATA_REG30       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x3E))) // RW/RW, data buffer 30
#define R8_DATA_REG31       (*((volatile unsigned char *)(PIOC_SFR_BASE+0x3F))) // RW/RW, data buffer 31

/* ******************************************************************************************************* */

/* PIOC Registers */
typedef struct
{
    uint32_t          RESERVED00;
    union {
      __IO uint32_t   D32_PIOC_SFR ; // RO/RW, PIOC SFR
      struct {
        __IO uint8_t  D8_INDIR_ADDR; // RO/RW, PIOC indirect address
        __IO uint8_t  D8_TMR0_COUNT; // RO/RW, PIOC timer count
        __IO uint8_t  D8_TMR0_CTRL; // RO/RW, PIOC timer control and GP bit
        __IO uint8_t  D8_TMR0_INIT; // RO/RW, PIOC timer initial value
      } ;
    } ;
    union {
      __IO uint32_t   D32_PORT_CFG ; // RO/RW, port status and config
      struct {
        __IO uint8_t  D8_BIT_CYCLE; // RO/RW, encode bit cycle
        __IO uint8_t  D8_INDIR_ADDR2; // RO/RW, PIOC indirect address 2
        __IO uint8_t  D8_PORT_DIR; // RO/RW, IO port direction and mode
        __IO uint8_t  D8_PORT_IO; // RO/RW, IO port input and output
      } ;
    } ;
    uint32_t          RESERVED0C;
    uint32_t          RESERVED10;
    uint32_t          RESERVED14;
    uint32_t          RESERVED18;
    union {
      __IO uint32_t   D32_DATA_CTRL ; // RW/RW, data control
      struct {
        __IO uint8_t  D8_SYS_CFG; // RW/RW, port config
        __IO uint8_t  D8_CTRL_RD; // RO/RW, data for master read only and PIOC write only
        __IO uint8_t  D8_CTRL_WR; // RW/RO, data for master write only and PIOC read only
        __IO uint8_t  D8_DATA_EXCH; // RW/RW, data exchange
      } ;
    } ;
    union {
      __IO uint32_t   D32_DATA_REG0_3 ; // RW/RW, data buffer 0~3
      struct {
        __IO uint8_t  D8_DATA_REG0; // RW/RW, data buffer 0
        __IO uint8_t  D8_DATA_REG1; // RW/RW, data buffer 1
        __IO uint8_t  D8_DATA_REG2; // RW/RW, data buffer 2
        __IO uint8_t  D8_DATA_REG3; // RW/RW, data buffer 3
      } ;
      __IO uint16_t   D16_DATA_REG0_1 ; // RW/RW, data buffer 0~1
    } ;
    union {
      __IO uint32_t   D32_DATA_REG4_7 ; // RW/RW, data buffer 4~7
      struct {
        __IO uint8_t  D8_DATA_REG4; // RW/RW, data buffer 4
        __IO uint8_t  D8_DATA_REG5; // RW/RW, data buffer 5
        __IO uint8_t  D8_DATA_REG6; // RW/RW, data buffer 6
        __IO uint8_t  D8_DATA_REG7; // RW/RW, data buffer 7
      } ;
    } ;
    union {
      __IO uint32_t   D32_DATA_REG8_11 ; // RW/RW, data buffer 8~11
      struct {
        __IO uint8_t  D8_DATA_REG8; // RW/RW, data buffer 8
        __IO uint8_t  D8_DATA_REG9; // RW/RW, data buffer 9
        __IO uint8_t  D8_DATA_REG10; // RW/RW, data buffer 10
        __IO uint8_t  D8_DATA_REG11; // RW/RW, data buffer 11
      } ;
    } ;
    union {
      __IO uint32_t   D32_DATA_REG12_15 ; // RW/RW, data buffer 12~15
      struct {
        __IO uint8_t  D8_DATA_REG12; // RW/RW, data buffer 12
        __IO uint8_t  D8_DATA_REG13; // RW/RW, data buffer 13
        __IO uint8_t  D8_DATA_REG14; // RW/RW, data buffer 14
        __IO uint8_t  D8_DATA_REG15; // RW/RW, data buffer 15
      } ;
    } ;
    union {
      __IO uint32_t   D32_DATA_REG16_19 ; // RW/RW, data buffer 16~19
      struct {
        __IO uint8_t  D8_DATA_REG16; // RW/RW, data buffer 16
        __IO uint8_t  D8_DATA_REG17; // RW/RW, data buffer 17
        __IO uint8_t  D8_DATA_REG18; // RW/RW, data buffer 18
        __IO uint8_t  D8_DATA_REG19; // RW/RW, data buffer 19
      } ;
    } ;
    union {
      __IO uint32_t   D32_DATA_REG20_23 ; // RW/RW, data buffer 20~23
      struct {
        __IO uint8_t  D8_DATA_REG20; // RW/RW, data buffer 20
        __IO uint8_t  D8_DATA_REG21; // RW/RW, data buffer 21
        __IO uint8_t  D8_DATA_REG22; // RW/RW, data buffer 22
        __IO uint8_t  D8_DATA_REG23; // RW/RW, data buffer 23
      } ;
    } ;
    union {
      __IO uint32_t   D32_DATA_REG24_27 ; // RW/RW, data buffer 24~27
      struct {
        __IO uint8_t  D8_DATA_REG24; // RW/RW, data buffer 24
        __IO uint8_t  D8_DATA_REG25; // RW/RW, data buffer 25
        __IO uint8_t  D8_DATA_REG26; // RW/RW, data buffer 26
        __IO uint8_t  D8_DATA_REG27; // RW/RW, data buffer 27
      } ;
    } ;
    union {
      __IO uint32_t   D32_DATA_REG28_31 ; // RW/RW, data buffer 28~31
      struct {
        __IO uint8_t  D8_DATA_REG28; // RW/RW, data buffer 28
        __IO uint8_t  D8_DATA_REG29; // RW/RW, data buffer 29
        __IO uint8_t  D8_DATA_REG30; // RW/RW, data buffer 30
        __IO uint8_t  D8_DATA_REG31; // RW/RW, data buffer 31
      } ;
    } ;
} PIOC_TypeDef;

#define PIOC          ((PIOC_TypeDef *)PIOC_BASE)

#ifdef __cplusplus
}
#endif

#endif  // __PIOC_SFR_H__
