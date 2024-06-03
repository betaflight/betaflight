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
/**   Pictbridge Application                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_pictbridge.h"
#include "ux_device_class_pima.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_dpsclient_object_info_get            PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns the information of the object.                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                   Pima instance associated     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    user application                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added no-callback handling, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_dpsclient_object_info_get(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle, 
                                                UX_SLAVE_CLASS_PIMA_OBJECT **object)
{

UX_PICTBRIDGE               *pictbridge;
UX_SLAVE_CLASS_PIMA_OBJECT   *object_info;

    /* Get the pointer to the Pictbridge instance.  */
    pictbridge = (UX_PICTBRIDGE *)pima -> ux_device_class_pima_application;
    
    /* Check the object handle. If this is handle 1 or 2 , we need to return the XML script object.
       If the handle is not 1 or 2, this is a JPEG picture or other object to be printed.  */
    if ((object_handle == UX_PICTBRIDGE_OBJECT_HANDLE_HOST_RESPONSE) || (object_handle == UX_PICTBRIDGE_OBJECT_HANDLE_CLIENT_REQUEST))
    {
        /* Check what XML object is requested. It is either a request script or a response.  */
        if (object_handle == UX_PICTBRIDGE_OBJECT_HANDLE_HOST_RESPONSE)       
            object_info = (UX_SLAVE_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_host;
        else        
            object_info = (UX_SLAVE_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_client;
    }        
    else
    {

        /* Get the object info from the application (if there is callback).  */
        if (pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_object_info_get)
            return pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_object_info_get(pima, object_handle, object);
        else
        {

            /* Get the object info from job.  */
            object_info = pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_object;

            /* Object info not available.  */
            if (object_info == UX_NULL)
            {

                /* Error trap.  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_PICTBRIDGE_ERROR_NO_VALID_OBJECT_INFO);
                return(UX_PICTBRIDGE_ERROR_NO_VALID_OBJECT_INFO);
            }
        }
    }

    /* Return the pointer to this object.  */
    *object = object_info;

    /* We are done.  */  
    return(UX_SUCCESS);
}

