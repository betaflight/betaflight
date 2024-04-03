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
/**   Device Pima Class                                                   */
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
/*    _ux_device_class_pima_object_add                    PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function sends an event to the host to inform that an object   */ 
/*    has been added.                                                     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*    object_handle                         Handle of object to delete    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_class_pima_event_set       Add PIMA event to queue       */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device Pima Class                                                   */ 
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
UINT  _ux_device_class_pima_object_add(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle)
{

UINT                        status;
UX_SLAVE_CLASS_PIMA_EVENT   pima_event;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_OBJECT_ADD, pima, object_handle, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Set the pima code for object added.  */
    pima_event.ux_device_class_pima_event_code         = UX_DEVICE_CLASS_PIMA_EC_REQUEST_OBJECT_TRANSFER;

    /* Set the object handle.  */
    pima_event.ux_device_class_pima_event_parameter_1  = object_handle;

    /* Other parameters are not used.  */
    pima_event.ux_device_class_pima_event_parameter_2  = 0;
    pima_event.ux_device_class_pima_event_parameter_3  = 0;

    /* Add the event.  */
    status =  _ux_device_class_pima_event_set(pima, &pima_event);

    /* Return completion status.  */
    return(status);
}


