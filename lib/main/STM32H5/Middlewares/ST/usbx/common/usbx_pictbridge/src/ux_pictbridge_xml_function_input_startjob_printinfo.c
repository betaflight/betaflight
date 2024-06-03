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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_xml_function_input_startjob_printinfo                */
/*                                                        PORTABLE C      */ 
/*                                                           6.1          */
/*                                                                        */ 
/*                                                                        */ 
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function receives a printinfo data block tag. There can be     */ 
/*    many so we should create a new printinfo block into the startjob    */ 
/*    structure for each of them as they come.                            */ 
/*                                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pictbridge                             Pictbridge instance          */ 
/*    input_variable                         Pointer to variable          */ 
/*    input_string                           Pointer to string            */ 
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
/*    _ux_pictbridge_object_parse                                         */ 
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
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo(UX_PICTBRIDGE *pictbridge, 
                            UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter)
{
UX_PICTBRIDGE_PRINTINFO     *printinfo;

    UX_PARAMETER_NOT_USED(input_string);
    UX_PARAMETER_NOT_USED(input_variable);
    UX_PARAMETER_NOT_USED(xml_parameter);

    /* Allocate some memory for the print info structure.  */
    printinfo =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, sizeof(UX_PICTBRIDGE_PRINTINFO));

    /* Check memory allocation.  */
    if (printinfo == UX_NULL)
    {
        /* Set the error flag in the jobinfo structure.  */
        pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_status = UX_ERROR;

        /* Return an error. */
        return(UX_MEMORY_INSUFFICIENT);
    }

    /* Store the current printinfo structure.  */
    pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_current = printinfo;

    /* We need to hook this instance, either it is the first or we have one or more previous instances. */
    if (pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_start == UX_NULL)
    {

        /* This is the first instance of a printinfo block.  */
        pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_start = printinfo;
        
        /* Make it the current as well.  */
        pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_current = printinfo;
        
    }
    else
    {
        /* Store the new instance in the next instance pointer of the current instance.  */
        pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_current -> ux_pictbridge_printinfo_next = printinfo;
        
        /* And the new one becomes the current.  */
        pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_printinfo_current = printinfo;
        
    }

    /* This function did not fail.  */
    return(UX_SUCCESS);
}


