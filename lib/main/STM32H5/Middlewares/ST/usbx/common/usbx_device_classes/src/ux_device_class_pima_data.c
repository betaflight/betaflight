/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/

/**************************************************************************/
/**************************************************************************/
/**                                                                       */ 
/** USBX Component                                                        */ 
/**                                                                       */
/**   Device PIMA Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_pima.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    ux_device_class_pima_data.c                         PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This module contains all the data definition used by the PIMA       */ 
/*    device class.                                                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/

/* Define PIMA vendor extension descriptor. This is a regular string that needs to be put into unicode. */
UCHAR _ux_device_class_pima_vendor_extension_descriptor[] =     {
#ifdef UX_PIMA_WITH_MTP_SUPPORT
                                                                "microsoft.com: 1.0;  microsoft.com/WMPPD: 11.0; microsoft.com/WMPPD: 10.0; microsoft.com/WMDRMPD: 10.1;"
#else
                                                                0
#endif                                                                
                                                                };
                                                                
/* Define PIMA supported operations. The last entry MUST be a zero. The DeviceInfoSet command
   will parse this array and compute the number of functions supported and return it to the
   host.  */

USHORT _ux_device_class_pima_supported_operations[] =            {

                                                                UX_DEVICE_CLASS_PIMA_OC_GET_DEVICE_INFO,                    
                                                                UX_DEVICE_CLASS_PIMA_OC_OPEN_SESSION,                   
                                                                UX_DEVICE_CLASS_PIMA_OC_CLOSE_SESSION,                  
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_STORAGE_IDS,                    
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_STORAGE_INFO,                   
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_NUM_OBJECTS,                    
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_HANDLES,                 
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_INFO,                    
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT,                     
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_THUMB,                      
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_PARTIAL_OBJECT,
                                                                UX_DEVICE_CLASS_PIMA_OC_DELETE_OBJECT,                  
                                                                UX_DEVICE_CLASS_PIMA_OC_SEND_OBJECT_INFO,                   
                                                                UX_DEVICE_CLASS_PIMA_OC_SEND_OBJECT,                    
                                                                UX_DEVICE_CLASS_PIMA_OC_INITIATE_CAPTURE,               
                                                                UX_DEVICE_CLASS_PIMA_OC_FORMAT_STORE,                   
                                                                UX_DEVICE_CLASS_PIMA_OC_RESET_DEVICE,                   
#ifdef UX_PIMA_WITH_MTP_SUPPORT                                                        
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROPS_SUPPORTED,
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROP_DESC,
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROP_VALUE,
                                                                UX_DEVICE_CLASS_PIMA_OC_SET_OBJECT_PROP_VALUE,
                                                                UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_REFERENCES,
                                                                UX_DEVICE_CLASS_PIMA_OC_SET_OBJECT_REFERENCES,                        
#endif                                                                
                                                                0
                                                                };


/* Define PIMA supported events. The last entry MUST be a zero. The DeviceInfoSet command
   will parse this array and compute the number of functions supported and return it to the
   host.  */

USHORT _ux_device_class_pima_supported_events[] =                {

                                                                UX_DEVICE_CLASS_PIMA_EC_CANCEL_TRANSACTION,         
                                                                UX_DEVICE_CLASS_PIMA_EC_OBJECT_ADDED,               
                                                                UX_DEVICE_CLASS_PIMA_EC_OBJECT_REMOVED,             
                                                                UX_DEVICE_CLASS_PIMA_EC_STORE_ADDED,                    
                                                                UX_DEVICE_CLASS_PIMA_EC_STORE_REMOVED,              
                                                                UX_DEVICE_CLASS_PIMA_EC_DEVICE_PROP_CHANGED,            
                                                                UX_DEVICE_CLASS_PIMA_EC_OBJECT_INFO_CHANGED,            
                                                                UX_DEVICE_CLASS_PIMA_EC_DEVICE_INFO_CHANGED,            
                                                                UX_DEVICE_CLASS_PIMA_EC_REQUEST_OBJECT_TRANSFER,        
                                                                UX_DEVICE_CLASS_PIMA_EC_STORE_FULL,                 
                                                                UX_DEVICE_CLASS_PIMA_EC_DEVICE_RESET,               
                                                                UX_DEVICE_CLASS_PIMA_EC_STORAGE_INFO_CHANGED,       
                                                                UX_DEVICE_CLASS_PIMA_EC_CAPTURE_COMPLETE,           
                                                                UX_DEVICE_CLASS_PIMA_EC_UNREPORTED_STATUS,          
                                                                0
                                                                };

/* Define PIMA supported device properties. The last entry MUST be a zero. The DeviceInfoSet command
   will parse this array and compute the number of functions supported and return it to the
   host.  For each declared device property, a dataset must be created in the application.  
   This table is used is the application has not defined any device properties. */
USHORT _ux_device_class_pima_device_prop_supported[] =   {

                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_UNDEFINED,                                    
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_BATTERY_LEVEL,                                
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_FUNCTIONAL_MODE,                                  
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_IMAGE_SIZE,                                    
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_COMPRESSION_SETTING,                              
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_WHITE_BALANCE,                                
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_RGB_GAIN,                                        
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_F_NUMBER,                                        
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_FOCAL_LENGTH,                                      
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_FOCUS_DISTANCE,                                
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_FOCUS_MODE,                                    
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_EXPOSURE_METERING_MODE,                        
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_FLASH_MODE,                                    
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_EXPOSURE_TIME,                                
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_EXPOSURE_PROGRAM_MODE,                          
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_EXPOSURE_INDEX,                                
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_EXPOSURE_BIAS_COMPENSATION,                    
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_DATE_TIME,                        
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_CAPTURE_DELAY,                                
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_STILL_CAPTURE_MODE,                            
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_CONTRAST,                                        
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_SHARPNESS,                                    
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_DIGITAL_ZOOM,                                      
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_EFFECT_MODE,                                      
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_BURST_NUMBER,                                      
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_BURST_INTERVAL,                                
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_TIME_LAPSE_NUMBER,                              
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_TIME_LAPSE_INTERVAL,                            
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_FOCUS_METERING_MODE,                            
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_UPLOAD_URL,                                    
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_ARTIST,                                          
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_COPYRIGHT_INFO,                                
#ifdef UX_PIMA_WITH_MTP_SUPPORT
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_SYNCHRONIZATION_PARTNER,                        
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_DEVICE_FRIENDLY_NAME,                            
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_VOLUME,                                        
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_SUPPORTED_FORMATS_ORDERED,                    
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_DEVICE_ICON,                                    
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_PLAYBACK_RATE,                                
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_PLAYBACK_OBJECT,                                
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_PLAYBACK_CONTAINER,                            
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_SESSION_INITIATOR_VERSION_INFO,                
                                                                UX_DEVICE_CLASS_PIMA_DEV_PROP_PERCEIVED_DEVICE_TYPE,                        
#endif
                                                                0
    };
    
/* Define PIMA supported capture formats. The last entry MUST be a zero. The DeviceInfoSet command
   will parse this array and compute the number of functions supported and return it to the
   host.  
   This table is used is the application has not defined any capture formats. */
USHORT _ux_device_class_pima_supported_capture_formats[] =       {
                                                                0
                                                                };

/* Define PIMA supported image formats. The last entry MUST be a zero. The DeviceInfoSet command
   will parse this array and compute the number of formats supported and return it to the
   host.  
   This table is used is the application has not defined any capture formats. */
USHORT _ux_device_class_pima_supported_image_formats[] =        {
                                                                UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED,                            
                                                                UX_DEVICE_CLASS_PIMA_OFC_ASSOCIATION,                        
                                                                UX_DEVICE_CLASS_PIMA_OFC_SCRIPT,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_EXECUTABLE,                            
                                                                UX_DEVICE_CLASS_PIMA_OFC_TEXT,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_HTML,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_DPOF,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_AIFF,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_WAV,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_MP3,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_AVI,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_MPEG,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_ASF,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_DEFINED,                            
                                                                UX_DEVICE_CLASS_PIMA_OFC_EXIF_JPEG,                            
                                                                UX_DEVICE_CLASS_PIMA_OFC_TIFF_EP,                            
                                                                UX_DEVICE_CLASS_PIMA_OFC_FLASHPIX,                            
                                                                UX_DEVICE_CLASS_PIMA_OFC_BMP,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_CIFF,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED,                            
                                                                UX_DEVICE_CLASS_PIMA_OFC_GIF,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_JFIF,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_CD,                                    
                                                                UX_DEVICE_CLASS_PIMA_OFC_PICT,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_PNG,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED,                            
                                                                UX_DEVICE_CLASS_PIMA_OFC_TIFF,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_TIFF_IT,                            
                                                                UX_DEVICE_CLASS_PIMA_OFC_JP2,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_JPX,                                
#ifdef UX_PIMA_WITH_MTP_SUPPORT
                                                                UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_FIRMWARE,                    
                                                                UX_DEVICE_CLASS_PIMA_OFC_WINDOWS_IMAGE_FORMAT,                
                                                                UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_AUDIO,                    
                                                                UX_DEVICE_CLASS_PIMA_OFC_WMA,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_OGG,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_AAC,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_AUDIBLE,                            
                                                                UX_DEVICE_CLASS_PIMA_OFC_FLAC,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_VIDEO,                    
                                                                UX_DEVICE_CLASS_PIMA_OFC_WMV,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_MP4_CONTAINER,                        
                                                                UX_DEVICE_CLASS_PIMA_OFC_MP2,                                
                                                                UX_DEVICE_CLASS_PIMA_OFC_3GP_CONTAINER,                        
                                                                UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_COLLECTION,                
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_MULTIMEDIA_ALBUM,            
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_IMAGE_ALBUM,                
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_AUDIO_ALBUM,                
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_VIDEO_ALBUM,                
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_AUDIO_AND_VIDEO_PLAYLIST,    
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_CONTACT_GROUP,                
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_MESSAGE_FOLDER,            
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_CHAPTERED_PRODUCTION,        
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_AUDIO_PLAYLIST,            
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_VIDEO_PLAYLIST,            
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_MEDIACAST,                    
                                                                UX_DEVICE_CLASS_PIMA_OFC_WPL_PLAYLIST,                        
                                                                UX_DEVICE_CLASS_PIMA_OFC_M3U_PLAYLIST,                        
                                                                UX_DEVICE_CLASS_PIMA_OFC_MPL_PLAYLIST,                        
                                                                UX_DEVICE_CLASS_PIMA_OFC_ASX_PLAYLIST,                        
                                                                UX_DEVICE_CLASS_PIMA_OFC_PLS_PLAYLIST,                        
                                                                UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_DOCUMENT,                    
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_DOCUMENT,                    
                                                                UX_DEVICE_CLASS_PIMA_OFC_XML_DOCUMENT,                        
                                                                UX_DEVICE_CLASS_PIMA_OFC_MICROSOFT_WORD_DOCUMENT,            
                                                                UX_DEVICE_CLASS_PIMA_OFC_MHT_COMPILED_HTML_DOCUMENT,            
                                                                UX_DEVICE_CLASS_PIMA_OFC_MICROSOFT_EXCEL_SPREADSHEET,        
                                                                UX_DEVICE_CLASS_PIMA_OFC_MICROSOFT_POWERPOINT_PRESENTATION,    
                                                                UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_MESSAGE,                    
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_MESSAGE,                    
                                                                UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_CONTACT,                    
                                                                UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_CONTACT,                    
                                                                UX_DEVICE_CLASS_PIMA_OFC_VCARD2,                                
#endif
                                                                0
                                                                };
