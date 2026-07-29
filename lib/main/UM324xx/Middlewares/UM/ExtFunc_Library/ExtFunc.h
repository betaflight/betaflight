
#ifndef _EXTFUNC_H_
#define _EXTFUNC_H_

#include "um324xx_hal.h"



#define Main_ENABLE             0x0U
#define Boot_PIN_ENABLE         0x1U
#define Boot_PIN_DISABLE        0x2U

/*升压模式：提高内核电压至1.2V*/
void boost_mode_Open(void);
void boost_mode_Close(void);


/* Vref1.2V for ADC0 AIN16 enable */
void Vref_1V2_Enable(void);
/* Vref1.2V for ADC0 AIN16 disable */
void Vref_1V2_Disable(void);

/* Enable LDO02 from pin PB0 output */    
void LDO02_OUTPUT(void);
/* Disable LDO02 from pin PB0 output */    
void LDO02_OUTPUT_Dis(void);

/* Enable Vref1.2V from pin PB1 output */    
void Vref_1V2_OUTPUT(void);
/* Disable Vref1.2V from pin PB1 output */    
void Vref_1V2_OUTPUT_Dis(void);

/*芯片启动方式配置：
    1: Main_ENABLE
    2: Boot_PIN_ENABLE
    3: Boot_PIN_DISABLE
*/
void Boot_PIN_Config(uint8_t mode);

void OTP_ENABLE(void);

#endif


