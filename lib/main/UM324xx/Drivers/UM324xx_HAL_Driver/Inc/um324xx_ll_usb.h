
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324xx_LL_USB_H_
#define __UM324xx_LL_USB_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/**
  * @brief  USB Instance Initialization Structure definition
  */
typedef struct
{
  uint8_t dev_endpoints;            /*!< Device Endpoints number.
                                         This parameter depends on the used USB core.
                                         This parameter must be a number between Min_Data = 1 and Max_Data = 5 */
  uint32_t speed;                /*!< USB Core speed.
                                      This parameter can be any value of @ref USB_Core_Speed_                */
  uint32_t phy_itface;           /*!< Select the used PHY interface.
                                      This parameter can be any value of @ref USB_Core_PHY_                  */

  uint8_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */

  uint8_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */
	
	uint8_t low_power_enable;        /*!< Enable or disable the low Power Mode.                                  */

  uint8_t lpm_enable;              /*!< Enable or disable Link Power Management.                               */

  uint32_t battery_charging_enable; /*!< Enable or disable Battery charging.                                 */
} USB_CfgTypeDef;

typedef struct
{
  uint8_t   num;                  /*!< Endpoint number
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 5   */

  uint8_t   is_in;                /*!< Endpoint direction
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */

  uint8_t   is_stall;             /*!< Endpoint stall condition
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 1    */

  uint8_t   type;                 /*!< Endpoint type
                                       This parameter can be any value of @ref USB_LL_EP_Type                   */

  uint32_t  pmaadress;            /*!< PMA Address                                                              */

  uint32_t  maxpacket;            /*!< Endpoint Max packet size
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 64KB */

  uint8_t   *xfer_buff;           /*!< Pointer to transfer buffer                                               */

  uint32_t  xfer_len;             /*!< Current transfer length                                                  */

  uint32_t  xfer_count;           /*!< Partial transfer length in case of multi packet transfer                 */
	
	uint32_t  xfer_count_last;
} USB_EPTypeDef;


/** @defgroup USB_LL_EP0_MPS USB Low Layer EP0 MPS
  * @{
  */
#define EP_MPS_64                              0U
#define EP_MPS_32                              1U
#define EP_MPS_16                              2U
#define EP_MPS_8                               3U
/**
  * @}
  */

/** @defgroup USB_LL_EP_Type USB Low Layer EP Type
  * @{
  */
#define EP_TYPE_CTRL                           0U
#define EP_TYPE_ISOC                           1U
#define EP_TYPE_BULK                           2U
#define EP_TYPE_INTR                           3U
#define EP_TYPE_MSK                            3U


#define EP_ADDR_MSK                            0x7U

#define EP0_IN_ADDR                            0x80
#define EP0_OUT_ADDR                           0x00
#define EP1_IN_ADDR                            0x81
#define EP1_OUT_ADDR                           0x01
#define EP2_IN_ADDR                            0x82
#define EP2_OUT_ADDR                           0x02
#define EP3_IN_ADDR                            0x83
#define EP3_OUT_ADDR                           0x03
#define EP4_IN_ADDR                            0x84
#define EP4_OUT_ADDR                           0x04


/* Exported functions --------------------------------------------------------*/
/** @addtogroup USB_LL_Exported_Functions USB Low Layer Exported Functions
  * @{
  */

HAL_StatusTypeDef USB_EnableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DevInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_ActivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPSetStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_StopDevice(USB_TypeDef *USBx);
uint32_t          USB_ReadInterrupts(USB_TypeDef const *USBx);
HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_TypeDef *USBx);
void              USB_WritePMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf,
                               uint32_t wPMABufAddr, uint16_t wNBytes);

void              USB_ReadPMA(USB_TypeDef const *USBx, uint8_t *pbUsrBuf,
                              uint32_t wPMABufAddr, uint16_t wNBytes);

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* __UM324xx_LL_USB_H_ */
