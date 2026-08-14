
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324xF_HAL_PCD_EX_H_
#define __UM324xF_HAL_PCD_EX_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"


HAL_StatusTypeDef  HAL_PCDEx_PMAConfig(PCD_HandleTypeDef *hpcd, uint16_t ep_addr,
                                       uint32_t pmaadress);
void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state);
void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg);
void HAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg);

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* __UM324xF_HAL_PCD_EX_H_ */
