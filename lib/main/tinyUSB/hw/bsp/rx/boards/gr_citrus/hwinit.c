/************************************************************************/
/*    File Version: V1.00                                               */
/*    Date Generated: 08/07/2013                                        */
/************************************************************************/

#include "iodefine.h"
#ifdef __cplusplus
extern "C" {
#endif
extern void HardwareSetup(void);
#ifdef __cplusplus
}
#endif

void HardwareSetup(void)
{
    SYSTEM.PRCR.WORD     = 0xA503u;
    SYSTEM.SOSCCR.BYTE   = 0x01u;
    SYSTEM.MOSCWTCR.BYTE = 0x0Du;
    SYSTEM.PLLWTCR.BYTE  = 0x0Eu;
    SYSTEM.PLLCR.WORD    = 0x0F00u;
    SYSTEM.MOSCCR.BYTE   = 0x00u;
    SYSTEM.PLLCR2.BYTE   = 0x00u;
    for (unsigned i = 0; i < 2075u; ++i) __asm("nop");
    SYSTEM.SCKCR.LONG    = 0x21021211u;
    SYSTEM.SCKCR2.WORD   = 0x0033u;
    SYSTEM.SCKCR3.WORD   = 0x0400u;
    SYSTEM.SYSCR0.WORD   = 0x5A01;
    SYSTEM.MSTPCRB.BIT.MSTPB15 = 0;
    SYSTEM.PRCR.WORD     = 0xA500u;
}
