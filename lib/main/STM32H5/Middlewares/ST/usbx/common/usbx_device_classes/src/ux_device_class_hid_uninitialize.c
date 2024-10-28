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
/**                                                                       */ 
/** USBX Component                                                        */ 
/**                                                                       */
/**   Device HID Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_hid.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_hid_uninitialize                   PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function uninitializes the USB HID device.                     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                              Pointer to hid command         */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_thread_delete             Remove storage thread.         */ 
/*    _ux_utility_memory_free              Free memory used by storage    */ 
/*    _ux_utility_event_flags_delete       Remove flag event structure    */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Source Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            added interrupt OUT support,*/
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_hid_uninitialize(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_CLASS_HID                      *hid;
UX_SLAVE_CLASS                          *class_ptr;


    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    hid = (UX_SLAVE_CLASS_HID *) class_ptr -> ux_slave_class_instance;

#if !defined(UX_DEVICE_STANDALONE)

    /* Remove HID thread.  */
    _ux_device_thread_delete(&class_ptr -> ux_slave_class_thread);

    /* Remove the thread used by HID.  */
    _ux_utility_memory_free(class_ptr -> ux_slave_class_thread_stack);

    /* Delete the event flag group for the hid class.  */
    _ux_device_event_flags_delete(&hid -> ux_device_class_hid_event_flags_group);
#endif

    /* Free memory for the array. */
    _ux_utility_memory_free(hid -> ux_device_class_hid_event_array);

#if defined(UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT)

#if !defined(UX_DEVICE_STANDALONE)

    /* Free read mutex.  */
    _ux_device_mutex_delete(&hid -> ux_device_class_hid_read_mutex);
#endif

    /* Uninitialize receiver.  */
    if (hid -> ux_device_class_hid_receiver)
        hid -> ux_device_class_hid_receiver ->
            ux_device_class_hid_receiver_uninitialize(hid -> ux_device_class_hid_receiver);
#endif

    /* Free the resources.  */
    _ux_utility_memory_free(hid);

    /* Return completion status.  */
    return(UX_SUCCESS);
}

