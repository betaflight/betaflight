/**
  ******************************************************************************
  * @file    usbd_dfu_core.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB DFU Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as DFU Device (and enumeration for each implemented memory interface)
  *           - Transfers to/from memory interfaces
  *           - Easy-to-customize "plug-in-like" modules for adding/removing memory interfaces.
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                DFU Class Driver Description
  *          =================================================================== 
  *           This driver manages the DFU class V1.1 following the "Device Class Specification for 
  *           Device Firmware Upgrade Version 1.1 Aug 5, 2004".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as DFU device (in DFU mode only)
  *             - Requests management (supporting ST DFU sub-protocol)
  *             - Memory operations management (Download/Upload/Erase/Detach/GetState/GetStatus)
  *             - DFU state machine implementation.
  *          
  *           @note
  *            ST DFU sub-protocol is compliant with DFU protocol and use sub-requests to manage
  *            memory addressing, commands processing, specific memories operations (ie. Erase) ...
  *            As required by the DFU specification, only endpoint 0 is used in this application.
  *            Other endpoints and functions may be added to the application (ie. DFU ...)
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *           This driver doesn't implement the following aspects of the specification 
  *           (but it is possible to manage these features with some modifications on this driver):
  *             - Manifestation Tolerant mode
  *      
  *  @endverbatim
  *                                  
  ******************************************************************************               
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_dfu_core.h"
#include "usbd_desc.h"
#include "usbd_req.h"
#include "usb_bsp.h"


/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup usbd_dfu 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup usbd_dfu_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_dfu_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_dfu_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_dfu_Private_FunctionPrototypes
  * @{
  */

/*********************************************
   DFU Device library callbacks
 *********************************************/
static uint8_t  usbd_dfu_Init     (void  *pdev, 
                                  uint8_t cfgidx);

static uint8_t  usbd_dfu_DeInit   (void  *pdev, 
                                  uint8_t cfgidx);

static uint8_t  usbd_dfu_Setup    (void  *pdev, 
                                  USB_SETUP_REQ *req);

static uint8_t  EP0_TxSent        (void  *pdev);

static uint8_t  EP0_RxReady       (void  *pdev);


static uint8_t  *USBD_DFU_GetCfgDesc (uint8_t speed, 
                                      uint16_t *length);


#ifdef USB_OTG_HS_CORE
static uint8_t  *USBD_DFU_GetOtherCfgDesc (uint8_t speed, 
                                      uint16_t *length);
#endif

static uint8_t* USBD_DFU_GetUsrStringDesc (uint8_t speed, 
                                           uint8_t index ,
                                           uint16_t *length);

/*********************************************
   DFU Requests management functions
 *********************************************/
static void DFU_Req_DETACH    (void *pdev, 
                               USB_SETUP_REQ *req);

static void DFU_Req_DNLOAD    (void *pdev,
                               USB_SETUP_REQ *req);

static void DFU_Req_UPLOAD    (void *pdev,
                               USB_SETUP_REQ *req);

static void DFU_Req_GETSTATUS (void *pdev);

static void DFU_Req_CLRSTATUS (void *pdev);

static void DFU_Req_GETSTATE  (void *pdev);

static void DFU_Req_ABORT     (void *pdev);

static void DFU_LeaveDFUMode  (void *pdev); 

/**
  * @}
  */ 

/** @defgroup usbd_dfu_Private_Variables
  * @{
  */ 
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t usbd_dfu_CfgDesc[USB_DFU_CONFIG_DESC_SIZ] __ALIGN_END ;


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t usbd_dfu_OtherCfgDesc[USB_DFU_CONFIG_DESC_SIZ] __ALIGN_END ;

/* The list of Interface String descriptor pointers is defined in usbd_dfu_mal.c 
  file. This list can be updated whenever a memory has to be added or removed */
extern const uint8_t* usbd_dfu_StringDesc[];

/* State Machine variables */
uint8_t DeviceState;
uint8_t DeviceStatus[6];
uint32_t Manifest_State = Manifest_complete;
/* Data Management variables */
static uint32_t wBlockNum = 0, wlength = 0;
static uint32_t Pointer = APP_DEFAULT_ADD;  /* Base Address to Erase, Program or Read */
static __IO uint32_t  usbd_dfu_AltSet = 0;

extern uint8_t MAL_Buffer[];

/* DFU interface class callbacks structure */
USBD_Class_cb_TypeDef  DFU_cb = 
{
  usbd_dfu_Init,
  usbd_dfu_DeInit,
  usbd_dfu_Setup,
  EP0_TxSent,
  EP0_RxReady,
  NULL, /* DataIn, */
  NULL, /* DataOut, */
  NULL, /*SOF */
  NULL,
  NULL,     
  USBD_DFU_GetCfgDesc,
#ifdef USB_OTG_HS_CORE  
  USBD_DFU_GetOtherCfgDesc, /* use same cobfig as per FS */
#endif  
  USBD_DFU_GetUsrStringDesc,
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB DFU device Configuration Descriptor */
__ALIGN_BEGIN uint8_t usbd_dfu_CfgDesc[USB_DFU_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuation Descriptor size */
  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
  USB_DFU_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x02,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xC0,         /*bmAttributes: bus powered and Supprts Remote Wakeup */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  /* 09 */
  
  /**********  Descriptor of DFU interface 0 Alternate setting 0 **************/  
  USBD_DFU_IF_DESC(0), /* This interface is mandatory for all devices */
  
#if (USBD_ITF_MAX_NUM > 1)
  /**********  Descriptor of DFU interface 0 Alternate setting 1 **************/ 
  USBD_DFU_IF_DESC(1),
#endif /* (USBD_ITF_MAX_NUM > 1) */

#if (USBD_ITF_MAX_NUM > 2)
  /**********  Descriptor of DFU interface 0 Alternate setting 2 **************/ 
  USBD_DFU_IF_DESC(2),
#endif /* (USBD_ITF_MAX_NUM > 2) */

#if (USBD_ITF_MAX_NUM > 3)
  /**********  Descriptor of DFU interface 0 Alternate setting 3 **************/ 
  USBD_DFU_IF_DESC(3),
#endif /* (USBD_ITF_MAX_NUM > 3) */

#if (USBD_ITF_MAX_NUM > 4)
  /**********  Descriptor of DFU interface 0 Alternate setting 4 **************/ 
  USBD_DFU_IF_DESC(4),
#endif /* (USBD_ITF_MAX_NUM > 4) */

#if (USBD_ITF_MAX_NUM > 5)
  /**********  Descriptor of DFU interface 0 Alternate setting 5 **************/ 
  USBD_DFU_IF_DESC(5),
#endif /* (USBD_ITF_MAX_NUM > 5) */

#if (USBD_ITF_MAX_NUM > 6)
#error "ERROR: usbd_dfu_core.c: Modify the file to support more descriptors!"
#endif /* (USBD_ITF_MAX_NUM > 6) */

  /******************** DFU Functional Descriptor********************/
  0x09,   /*blength = 9 Bytes*/
  DFU_DESCRIPTOR_TYPE,   /* DFU Functional Descriptor*/
  0x0B,   /*bmAttribute
                bitCanDnload             = 1      (bit 0)
                bitCanUpload             = 1      (bit 1)
                bitManifestationTolerant = 0      (bit 2)
                bitWillDetach            = 1      (bit 3)
                Reserved                          (bit4-6)
                bitAcceleratedST         = 0      (bit 7)*/
  0xFF,   /*DetachTimeOut= 255 ms*/
  0x00,
  /*WARNING: In DMA mode the multiple MPS packets feature is still not supported
   ==> In this case, when using DMA XFERSIZE should be set to 64 in usbd_conf.h */
  TRANSFER_SIZE_BYTES(XFERSIZE),       /* TransferSize = 1024 Byte*/         
  0x1A,                                /* bcdDFUVersion*/
  0x01
  /***********************************************************/
  /* 9*/
} ;

#ifdef USE_USB_OTG_HS
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN uint8_t usbd_dfu_OtherCfgDesc[USB_DFU_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_DFU_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x02,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xC0,         /*bmAttributes: bus powered and Supprts Remote Wakeup */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  /* 09 */
  
  /**********  Descriptor of DFU interface 0 Alternate setting 0 **************/  
  USBD_DFU_IF_DESC(0), /* This interface is mandatory for all devices */
  
#if (USBD_ITF_MAX_NUM > 1)
  /**********  Descriptor of DFU interface 0 Alternate setting 1 **************/ 
  USBD_DFU_IF_DESC(1),
#endif /* (USBD_ITF_MAX_NUM > 1) */

#if (USBD_ITF_MAX_NUM > 2)
  /**********  Descriptor of DFU interface 0 Alternate setting 2 **************/ 
  USBD_DFU_IF_DESC(2),
#endif /* (USBD_ITF_MAX_NUM > 2) */

#if (USBD_ITF_MAX_NUM > 3)
  /**********  Descriptor of DFU interface 0 Alternate setting 3 **************/ 
  USBD_DFU_IF_DESC(3),
#endif /* (USBD_ITF_MAX_NUM > 3) */

#if (USBD_ITF_MAX_NUM > 4)
  /**********  Descriptor of DFU interface 0 Alternate setting 4 **************/ 
  USBD_DFU_IF_DESC(4),
#endif /* (USBD_ITF_MAX_NUM > 4) */

#if (USBD_ITF_MAX_NUM > 5)
  /**********  Descriptor of DFU interface 0 Alternate setting 5 **************/ 
  USBD_DFU_IF_DESC(5),
#endif /* (USBD_ITF_MAX_NUM > 5) */

#if (USBD_ITF_MAX_NUM > 6)
#error "ERROR: usbd_dfu_core.c: Modify the file to support more descriptors!"
#endif /* (USBD_ITF_MAX_NUM > 6) */

  /******************** DFU Functional Descriptor********************/
  0x09,   /*blength = 9 Bytes*/
  DFU_DESCRIPTOR_TYPE,   /* DFU Functional Descriptor*/
  0x0B,   /*bmAttribute
                bitCanDnload             = 1      (bit 0)
                bitCanUpload             = 1      (bit 1)
                bitManifestationTolerant = 0      (bit 2)
                bitWillDetach            = 1      (bit 3)
                Reserved                          (bit4-6)
                bitAcceleratedST         = 0      (bit 7)*/
  0xFF,   /*DetachTimeOut= 255 ms*/
  0x00,
  /*WARNING: In DMA mode the multiple MPS packets feature is still not supported
   ==> In this case, when using DMA XFERSIZE should be set to 64 in usbd_conf.h */
  TRANSFER_SIZE_BYTES(XFERSIZE),       /* TransferSize = 1024 Byte*/         
  0x1A,                                /* bcdDFUVersion*/
  0x01
  /***********************************************************/
  /* 9*/
};
#endif /* USE_USB_OTG_HS */

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif

__ALIGN_BEGIN static uint8_t usbd_dfu_Desc[USB_DFU_DESC_SIZ] __ALIGN_END =
{
  0x09,   /*blength = 9 Bytes*/
  DFU_DESCRIPTOR_TYPE,   /* DFU Functional Descriptor*/
  0x0B,   /*bmAttribute
                bitCanDnload             = 1      (bit 0)
                bitCanUpload             = 1      (bit 1)
                bitManifestationTolerant = 0      (bit 2)
                bitWillDetach            = 1      (bit 3)
                Reserved                          (bit4-6)
                bitAcceleratedST         = 0      (bit 7)*/
  0xFF,   /*DetachTimeOut= 255 ms*/
  0x00,
  /*WARNING: In DMA mode the multiple MPS packets feature is still not supported
   ==> In this case, when using DMA XFERSIZE should be set to 64 in usbd_conf.h */
  TRANSFER_SIZE_BYTES(XFERSIZE),  /* TransferSize = 1024 Byte*/
  0x1A,                     /* bcdDFUVersion*/
  0x01
};
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/**
  * @}
  */ 

/** @defgroup usbd_dfu_Private_Functions
  * @{
  */ 

/**
  * @brief  usbd_dfu_Init
  *         Initializes the DFU interface.
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  usbd_dfu_Init (void  *pdev, 
                               uint8_t cfgidx)
{
  /* Initilialize the MAL(Media Access Layer) */
  MAL_Init();
  
  /* Initialize the state of the DFU interface */
  DeviceState = STATE_dfuIDLE;
  DeviceStatus[0] = STATUS_OK;
  DeviceStatus[4] = DeviceState;
  
  return USBD_OK;
}

/**
  * @brief  usbd_dfu_Init
  *         De-initializes the DFU layer.
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  usbd_dfu_DeInit (void  *pdev, 
                                 uint8_t cfgidx)
{
  /* Restore default state */
  DeviceState = STATE_dfuIDLE;
  DeviceStatus[0] = STATUS_OK;
  DeviceStatus[4] = DeviceState;
  wBlockNum = 0;
  wlength = 0;

  /* DeInitilialize the MAL(Media Access Layer) */
  MAL_DeInit();
  
  return USBD_OK;
}

/**
  * @brief  usbd_dfu_Setup
  *         Handles the DFU request parsing.
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  usbd_dfu_Setup (void  *pdev, 
                                USB_SETUP_REQ *req)
{
  uint16_t len = 0;
  uint8_t  *pbuf = NULL;
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    /* DFU Class Requests -------------------------------*/
  case USB_REQ_TYPE_CLASS :  
    switch (req->bRequest)
    {
    case DFU_DNLOAD:
      DFU_Req_DNLOAD(pdev, req);
      break;
      
    case DFU_UPLOAD:
      DFU_Req_UPLOAD(pdev, req);   
      break;
      
    case DFU_GETSTATUS:
      DFU_Req_GETSTATUS(pdev);
      break;
      
    case DFU_CLRSTATUS:
      DFU_Req_CLRSTATUS(pdev);
      break;      

    case DFU_GETSTATE:
      DFU_Req_GETSTATE(pdev);
      break;  

    case DFU_ABORT:
      DFU_Req_ABORT(pdev);
      break;

    case DFU_DETACH:
      DFU_Req_DETACH(pdev, req);
      break;

    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL;
    }
    break;
    
    /* Standard Requests -------------------------------*/
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( (req->wValue >> 8) == DFU_DESCRIPTOR_TYPE)
      {
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
        pbuf = usbd_dfu_Desc;   
#else
        pbuf = usbd_dfu_CfgDesc + 9 + (9 * USBD_ITF_MAX_NUM);
#endif 
        len = MIN(USB_DFU_DESC_SIZ , req->wLength);
      }
      
      USBD_CtlSendData (pdev, 
                        pbuf,
                        len);
      break;
      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        (uint8_t *)&usbd_dfu_AltSet,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM)
      {
        usbd_dfu_AltSet = (uint8_t)(req->wValue);
      }
      else
      {
        /* Call the error management function (command will be nacked */
        USBD_CtlError (pdev, req);
      }
      break;
    }
  }
  return USBD_OK;
}

/**
  * @brief  EP0_TxSent
  *         Handles the DFU control endpoint data IN stage.
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  EP0_TxSent (void  *pdev)
{
  uint32_t Addr;
  USB_SETUP_REQ req;  
  
  if (DeviceState == STATE_dfuDNBUSY)
  {
    /* Decode the Special Command*/
    if (wBlockNum == 0)   
    {
      if ((MAL_Buffer[0] ==  CMD_GETCOMMANDS) && (wlength == 1))
      {}
      else if  (( MAL_Buffer[0] ==  CMD_SETADDRESSPOINTER ) && (wlength == 5))
      {
        Pointer  = MAL_Buffer[1];
        Pointer += MAL_Buffer[2] << 8;
        Pointer += MAL_Buffer[3] << 16;
        Pointer += MAL_Buffer[4] << 24;
      }
      else if (( MAL_Buffer[0] ==  CMD_ERASE ) && (wlength == 5))
      {
        Pointer  = MAL_Buffer[1];
        Pointer += MAL_Buffer[2] << 8;
        Pointer += MAL_Buffer[3] << 16;
        Pointer += MAL_Buffer[4] << 24;
        MAL_Erase(Pointer);
      }
      else
      {
        /* Reset the global length and block number */
        wlength = 0;
        wBlockNum = 0;     
        /* Call the error management function (command will be nacked) */
        req.bmRequest = 0;
        req.wLength = 1;
        USBD_CtlError (pdev, &req);
      }
    }
    /* Regular Download Command */
    else if (wBlockNum > 1)  
    {
      /* Decode the required address */
      Addr = ((wBlockNum - 2) * XFERSIZE) + Pointer;
      
      /* Preform the write operation */
      MAL_Write(Addr, wlength);
    }
    /* Reset the global lenght and block number */
    wlength = 0;
    wBlockNum = 0;
    
    /* Update the state machine */
    DeviceState =  STATE_dfuDNLOAD_SYNC;
    DeviceStatus[4] = DeviceState;
    DeviceStatus[1] = 0;
    DeviceStatus[2] = 0;
    DeviceStatus[3] = 0;
    return USBD_OK;
  }
  else if (DeviceState == STATE_dfuMANIFEST)/* Manifestation in progress*/
  {
    /* Start leaving DFU mode */
    DFU_LeaveDFUMode(pdev);
  }
  
  return USBD_OK;
}

/**
  * @brief  EP0_RxReady
  *         Handles the DFU control endpoint data OUT stage.
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  EP0_RxReady (void  *pdev)
{ 
  return USBD_OK;
}


/******************************************************************************
     DFU Class requests management
******************************************************************************/
/**
  * @brief  DFU_Req_DETACH
  *         Handles the DFU DETACH request.
  * @param  pdev: device instance
  * @param  req: pointer to the request structure.
  * @retval None.
  */
static void DFU_Req_DETACH(void *pdev, USB_SETUP_REQ *req)
{
  if (DeviceState == STATE_dfuIDLE || DeviceState == STATE_dfuDNLOAD_SYNC
      || DeviceState == STATE_dfuDNLOAD_IDLE || DeviceState == STATE_dfuMANIFEST_SYNC
        || DeviceState == STATE_dfuUPLOAD_IDLE )
  {
    /* Update the state machine */
    DeviceState = STATE_dfuIDLE;
    DeviceStatus[0] = STATUS_OK;
    DeviceStatus[1] = 0;
    DeviceStatus[2] = 0;
    DeviceStatus[3] = 0; /*bwPollTimeout=0ms*/
    DeviceStatus[4] = DeviceState;
    DeviceStatus[5] = 0; /*iString*/
    wBlockNum = 0;
    wlength = 0;
  } 
  
  /* Check the detach capability in the DFU functional descriptor */
  if ((usbd_dfu_CfgDesc[12 + (9 * USBD_ITF_MAX_NUM)]) & DFU_DETACH_MASK)
  {
    /* Perform an Attach-Detach operation on USB bus */
    DCD_DevDisconnect (pdev);
    DCD_DevConnect (pdev);  
  }
  else
  {
    /* Wait for the period of time specified in Detach request */
    USB_OTG_BSP_mDelay (req->wValue);  
  }
}

/**
  * @brief  DFU_Req_DNLOAD
  *         Handles the DFU DNLOAD request.
  * @param  pdev: device instance
  * @param  req: pointer to the request structure
  * @retval None
  */
static void DFU_Req_DNLOAD(void *pdev, USB_SETUP_REQ *req)
{
  /* Data setup request */
  if (req->wLength > 0)
  {
    if ((DeviceState == STATE_dfuIDLE) || (DeviceState == STATE_dfuDNLOAD_IDLE))
    {
      /* Update the global length and block number */
      wBlockNum = req->wValue;
      wlength = req->wLength;
      
      /* Update the state machine */
      DeviceState = STATE_dfuDNLOAD_SYNC;
      DeviceStatus[4] = DeviceState;
      
      /* Prepare the reception of the buffer over EP0 */
      USBD_CtlPrepareRx (pdev,
                         (uint8_t*)MAL_Buffer,                                  
                         wlength);
    }
    /* Unsupported state */
    else
    {
      /* Call the error management function (command will be nacked */
      USBD_CtlError (pdev, req);
    }
  }
  /* 0 Data DNLOAD request */
  else
  {
    /* End of DNLOAD operation*/
    if (DeviceState == STATE_dfuDNLOAD_IDLE || DeviceState == STATE_dfuIDLE )
    {
      Manifest_State = Manifest_In_Progress;
      DeviceState = STATE_dfuMANIFEST_SYNC;
      DeviceStatus[1] = 0;
      DeviceStatus[2] = 0;
      DeviceStatus[3] = 0;
      DeviceStatus[4] = DeviceState;
    }
    else
    {
      /* Call the error management function (command will be nacked */
      USBD_CtlError (pdev, req);
    }
  }  
}

/**
  * @brief  DFU_Req_UPLOAD
  *         Handles the DFU UPLOAD request.
  * @param  pdev: instance
  * @param  req: pointer to the request structure
  * @retval status
  */
static void DFU_Req_UPLOAD(void *pdev, USB_SETUP_REQ *req)
{
  uint8_t *Phy_Addr = NULL;
  uint32_t Addr = 0;
  
  /* Data setup request */
  if (req->wLength > 0)
  {
    if ((DeviceState == STATE_dfuIDLE) || (DeviceState == STATE_dfuUPLOAD_IDLE))
    {
      /* Update the global langth and block number */
      wBlockNum = req->wValue;
      wlength = req->wLength;
      
      /* DFU Get Command */
      if (wBlockNum == 0)  
      {
        /* Update the state machine */
        DeviceState = (wlength > 3)? STATE_dfuIDLE:STATE_dfuUPLOAD_IDLE;        
        DeviceStatus[4] = DeviceState;
        DeviceStatus[1] = 0;
        DeviceStatus[2] = 0;
        DeviceStatus[3] = 0;
        
        /* Store the values of all supported commands */
        MAL_Buffer[0] = CMD_GETCOMMANDS;
        MAL_Buffer[1] = CMD_SETADDRESSPOINTER;
        MAL_Buffer[2] = CMD_ERASE;
        
        /* Send the status data over EP0 */
        USBD_CtlSendData (pdev,
                          (uint8_t *)(&(MAL_Buffer[0])),
                          3);
      }
      else if (wBlockNum > 1)
      {
        DeviceState = STATE_dfuUPLOAD_IDLE ;
        DeviceStatus[4] = DeviceState;
        DeviceStatus[1] = 0;
        DeviceStatus[2] = 0;
        DeviceStatus[3] = 0;
        Addr = ((wBlockNum - 2) * XFERSIZE) + Pointer;  /* Change is Accelerated*/
        
        /* Return the physical address where data are stored */
        Phy_Addr = MAL_Read(Addr, wlength);
        
        /* Send the status data over EP0 */
        USBD_CtlSendData (pdev,
                          Phy_Addr,
                          wlength);
      }
      else  /* unsupported wBlockNum */
      {
        DeviceState = STATUS_ERRSTALLEDPKT;
        DeviceStatus[4] = DeviceState;
        DeviceStatus[1] = 0;
        DeviceStatus[2] = 0;
        DeviceStatus[3] = 0;
        
        /* Call the error management function (command will be nacked */
        USBD_CtlError (pdev, req); 
      }
    }
    /* Unsupported state */
    else
    {
      wlength = 0;
      wBlockNum = 0;   
      /* Call the error management function (command will be nacked */
      USBD_CtlError (pdev, req);
    }
  }
  /* No Data setup request */
  else
  {
    DeviceState = STATE_dfuIDLE;
    DeviceStatus[1] = 0;
    DeviceStatus[2] = 0;
    DeviceStatus[3] = 0;
    DeviceStatus[4] = DeviceState;
  }
}

/**
  * @brief  DFU_Req_GETSTATUS
  *         Handles the DFU GETSTATUS request.
  * @param  pdev: instance
  * @retval status
  */
static void DFU_Req_GETSTATUS(void *pdev)
{
  switch (DeviceState)
  {
  case   STATE_dfuDNLOAD_SYNC:
    if (wlength != 0)
    {
      DeviceState = STATE_dfuDNBUSY;
      DeviceStatus[4] = DeviceState;
      if ((wBlockNum == 0) && (MAL_Buffer[0] == CMD_ERASE))
      {
        MAL_GetStatus(Pointer, 0, DeviceStatus);
      }
      else
      {
        MAL_GetStatus(Pointer, 1, DeviceStatus);
      }
    }
    else  /* (wlength==0)*/
    {
      DeviceState = STATE_dfuDNLOAD_IDLE;
      DeviceStatus[4] = DeviceState;
      DeviceStatus[1] = 0;
      DeviceStatus[2] = 0;
      DeviceStatus[3] = 0;
    }
    break;
    
  case   STATE_dfuMANIFEST_SYNC :
    if (Manifest_State == Manifest_In_Progress)
    {
      DeviceState = STATE_dfuMANIFEST;
      DeviceStatus[4] = DeviceState;
      DeviceStatus[1] = 1;             /*bwPollTimeout = 1ms*/
      DeviceStatus[2] = 0;
      DeviceStatus[3] = 0;
      //break;
    }
    else if ((Manifest_State == Manifest_complete) && \
      ((usbd_dfu_CfgDesc[(11 + (9 * USBD_ITF_MAX_NUM))]) & 0x04))
    {
      DeviceState = STATE_dfuIDLE;
      DeviceStatus[4] = DeviceState;
      DeviceStatus[1] = 0;
      DeviceStatus[2] = 0;
      DeviceStatus[3] = 0;
      //break;
    }
    break;
    
  default :
    break;
  }
  
  /* Send the status data over EP0 */
  USBD_CtlSendData (pdev,
                    (uint8_t *)(&(DeviceStatus[0])),
                    6);
}

/**
  * @brief  DFU_Req_CLRSTATUS 
  *         Handles the DFU CLRSTATUS request.
  * @param  pdev: device instance
  * @retval status
  */
static void DFU_Req_CLRSTATUS(void *pdev)
{
  if (DeviceState == STATE_dfuERROR)
  {
    DeviceState = STATE_dfuIDLE;
    DeviceStatus[0] = STATUS_OK;/*bStatus*/
    DeviceStatus[1] = 0;
    DeviceStatus[2] = 0;
    DeviceStatus[3] = 0; /*bwPollTimeout=0ms*/
    DeviceStatus[4] = DeviceState;/*bState*/
    DeviceStatus[5] = 0;/*iString*/
  }
  else
  {   /*State Error*/
    DeviceState = STATE_dfuERROR;
    DeviceStatus[0] = STATUS_ERRUNKNOWN;/*bStatus*/
    DeviceStatus[1] = 0;
    DeviceStatus[2] = 0;
    DeviceStatus[3] = 0; /*bwPollTimeout=0ms*/
    DeviceStatus[4] = DeviceState;/*bState*/
    DeviceStatus[5] = 0;/*iString*/
  }
}

/**
  * @brief  DFU_Req_GETSTATE
  *         Handles the DFU GETSTATE request.
  * @param  pdev: device instance
  * @retval None
  */
static void DFU_Req_GETSTATE(void *pdev)
{
  /* Return the current state of the DFU interface */
  USBD_CtlSendData (pdev, 
                    &DeviceState,
                    1);  
}

/**
  * @brief  DFU_Req_ABORT
  *         Handles the DFU ABORT request.
  * @param  pdev: device instance
  * @retval None
  */
static void DFU_Req_ABORT(void *pdev)
{
  if (DeviceState == STATE_dfuIDLE || DeviceState == STATE_dfuDNLOAD_SYNC
      || DeviceState == STATE_dfuDNLOAD_IDLE || DeviceState == STATE_dfuMANIFEST_SYNC
        || DeviceState == STATE_dfuUPLOAD_IDLE )
  {
    DeviceState = STATE_dfuIDLE;
    DeviceStatus[0] = STATUS_OK;
    DeviceStatus[1] = 0;
    DeviceStatus[2] = 0;
    DeviceStatus[3] = 0; /*bwPollTimeout=0ms*/
    DeviceStatus[4] = DeviceState;
    DeviceStatus[5] = 0; /*iString*/
    wBlockNum = 0;
    wlength = 0;
  }  
}

/**
  * @brief  DFU_LeaveDFUMode
  *         Handles the sub-protocol DFU leave DFU mode request (leaves DFU mode
  *         and resets device to jump to user loaded code).
  * @param  pdev: device instance
  * @retval None
  */
void DFU_LeaveDFUMode(void *pdev)
{
 Manifest_State = Manifest_complete;

  if ((usbd_dfu_CfgDesc[(11 + (9 * USBD_ITF_MAX_NUM))]) & 0x04)
  {
    DeviceState = STATE_dfuMANIFEST_SYNC;
    DeviceStatus[4] = DeviceState;
    DeviceStatus[1] = 0;
    DeviceStatus[2] = 0;
    DeviceStatus[3] = 0;
    return;
  }
  else
  {
    DeviceState = STATE_dfuMANIFEST_WAIT_RESET;
    DeviceStatus[4] = DeviceState;
    DeviceStatus[1] = 0;
    DeviceStatus[2] = 0;
    DeviceStatus[3] = 0;

    /* Disconnect the USB device */
    DCD_DevDisconnect (pdev);

    /* DeInitilialize the MAL(Media Access Layer) */
    MAL_DeInit();
    
    /* Generate system reset to allow jumping to the user code */
    NVIC_SystemReset();
   
    /* This instruction will not be reached (system reset) */
    return;
  }  
}

/**
  * @brief  USBD_DFU_GetCfgDesc 
  *         Returns configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_DFU_GetCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_dfu_CfgDesc);
  return usbd_dfu_CfgDesc;
}

#ifdef USB_OTG_HS_CORE
/**
  * @brief  USBD_DFU_GetOtherCfgDesc 
  *         Returns other speed configuration descriptor.
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_DFU_GetOtherCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_dfu_OtherCfgDesc);
  return usbd_dfu_OtherCfgDesc;
}
#endif

/**
  * @brief  USBD_DFU_GetUsrStringDesc
  *         Manages the transfer of memory interfaces string descriptors.
  * @param  speed : current device speed
  * @param  index: desciptor index
  * @param  length : pointer data length
  * @retval pointer to the descriptor table or NULL if the descriptor is not supported.
  */
static uint8_t* USBD_DFU_GetUsrStringDesc (uint8_t speed, uint8_t index , uint16_t *length)
{
  /* Check if the requested string interface is supported */
  if (index <= (USBD_IDX_INTERFACE_STR + USBD_ITF_MAX_NUM))
  {
    
    
    USBD_GetString ((uint8_t *)usbd_dfu_StringDesc[index - USBD_IDX_INTERFACE_STR - 1], USBD_StrDesc, length);
    return USBD_StrDesc;  
  }
  /* Not supported Interface Descriptor index */
  else
  {
    return NULL;
  }
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

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
