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
/*    _ux_pictbridge_dpsclient_object_handles_get         PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns the handles array.                            */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_dpsclient_object_handles_get(UX_SLAVE_CLASS_PIMA *pima, 
                          ULONG object_handles_format_code, 
                          ULONG object_handles_association, 
                          ULONG *object_handles_array,
                          ULONG object_handles_max_number)
{
UX_PICTBRIDGE                   *pictbridge;
UX_SLAVE_CLASS_PIMA_OBJECT      *object_info;

    UX_PARAMETER_NOT_USED(object_handles_association);
    UX_PARAMETER_NOT_USED(object_handles_max_number);

    /* Get the pointer to the Pictbridge instance.  */
    pictbridge = (UX_PICTBRIDGE *) pima -> ux_device_class_pima_application;

    /* Set the pima pointe to the pictbridge instance.  */
    pictbridge -> ux_pictbridge_pima = (VOID *) pima;
    
    /* We say we have one object but the caller might specify different format code and associations. */
    object_info = pictbridge -> ux_pictbridge_object_client;
    
   /* Insert in the array the number of found handles so far : 0.  */
    _ux_utility_long_put((UCHAR *)object_handles_array, 0);
    
    /* Check the type demanded.  */
    if (object_handles_format_code == 0 || object_handles_format_code == 0xFFFFFFFF || 
        object_info -> ux_device_class_pima_object_format == object_handles_format_code)
    {

        /* Insert in the array the number of found handles.  This handle is for the client XML script. */
        _ux_utility_long_put((UCHAR *)object_handles_array, 1);

         /* Adjust the array to point after the number of elements.  */
        object_handles_array++;    

        /* We have a candidate.  Store the handle. */
        _ux_utility_long_put((UCHAR *)object_handles_array, object_info -> ux_device_class_pima_object_handle_id);

    }
    
    return(UX_SUCCESS);
}

