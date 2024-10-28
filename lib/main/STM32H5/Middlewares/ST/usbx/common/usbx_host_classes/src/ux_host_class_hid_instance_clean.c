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
/**   HID Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_instance_clean                   PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function cleans all the components of a HID instance.          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Class                                                           */ 
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
UINT  _ux_host_class_hid_instance_clean(UX_HOST_CLASS_HID *hid)
{

UX_HOST_CLASS_HID_PARSER     *hid_parser;
UX_HOST_CLASS_HID_REPORT     *hid_report;
UX_HOST_CLASS_HID_REPORT     *hid_next_report;
UX_HOST_CLASS_HID_FIELD      *hid_field;
UX_HOST_CLASS_HID_FIELD      *hid_next_field;


    /* Get the parser structure pointer.  */
    hid_parser =  &hid -> ux_host_class_hid_parser;

    /* Each report list of the parser should be cleaned: Input report.  */
    hid_report =  hid_parser -> ux_host_class_hid_parser_input_report;
    while (hid_report != UX_NULL)
    {

        /* Get the next report before we clean the current report.  */
        hid_next_report =  hid_report -> ux_host_class_hid_report_next_report;
        
        /* Get the first field in the report.  */
        hid_field =  hid_report -> ux_host_class_hid_report_field;
        
        /* Clean all the fields attached.  */
        while (hid_field != UX_NULL)
        {

            /* Get the next field before we clean the current field.  */
            hid_next_field =  hid_field -> ux_host_class_hid_field_next_field;        
        
            /* Free the usage table.  */
            if (hid_field -> ux_host_class_hid_field_usages != UX_NULL)
                _ux_utility_memory_free(hid_field -> ux_host_class_hid_field_usages);
                
            /* Free the value table.  */
            if (hid_field -> ux_host_class_hid_field_values != UX_NULL)
                _ux_utility_memory_free(hid_field -> ux_host_class_hid_field_values);
            
            /* Now free the field memory.  */                
            _ux_utility_memory_free(hid_field);

            /* Next field.  */
            hid_field =  hid_next_field;
        }

        /* Free the report.  */
        _ux_utility_memory_free(hid_report);

        /* Next report.  */
        hid_report =  hid_next_report;
    }

    /* Each report list of the parser should be cleaned: Output report.  */
    hid_report =  hid_parser -> ux_host_class_hid_parser_output_report;
    while (hid_report != UX_NULL)
    {

        /* Get the next report before we clean the current report.  */
        hid_next_report =  hid_report -> ux_host_class_hid_report_next_report;
        
        /* Get the first field in the report.  */
        hid_field =  hid_report -> ux_host_class_hid_report_field;
        
        /* Clean all the fields attached.  */
        while (hid_field != UX_NULL)
        {

            /* Get the next field before we clean the current field.  */
            hid_next_field =  hid_field -> ux_host_class_hid_field_next_field;        
        
            /* Free the usage table.  */
            if (hid_field -> ux_host_class_hid_field_usages != UX_NULL)
                _ux_utility_memory_free(hid_field -> ux_host_class_hid_field_usages);
                
            /* Free the value table.  */
            if (hid_field -> ux_host_class_hid_field_values != UX_NULL)
                _ux_utility_memory_free(hid_field -> ux_host_class_hid_field_values);
            
            /* Now free the field memory.  */                
            _ux_utility_memory_free(hid_field);

            /* Next field.  */
            hid_field =  hid_next_field;
        }

        /* Free the report.  */
        _ux_utility_memory_free(hid_report);

        /* Next report.  */
        hid_report =  hid_next_report;
    }

    /* Each report list of the parser should be cleaned: Feature report.  */
    hid_report =  hid_parser -> ux_host_class_hid_parser_feature_report;
    while (hid_report != UX_NULL)
    {

        /* Get the next report before we clean the current report.  */
        hid_next_report =  hid_report -> ux_host_class_hid_report_next_report;
        
        /* Get the first field in the report.  */
        hid_field =  hid_report -> ux_host_class_hid_report_field;
        
        /* Clean all the fields attached.  */
        while (hid_field != UX_NULL)
        {

            /* Get the next field before we clean the current field.  */
            hid_next_field =  hid_field -> ux_host_class_hid_field_next_field;        
        
            /* Free the usage table.  */
            if (hid_field -> ux_host_class_hid_field_usages != UX_NULL)
                _ux_utility_memory_free(hid_field -> ux_host_class_hid_field_usages);
                
            /* Free the value table.  */
            if (hid_field -> ux_host_class_hid_field_values != UX_NULL)
                _ux_utility_memory_free(hid_field -> ux_host_class_hid_field_values);
            
            /* Now free the field memory.  */                
            _ux_utility_memory_free(hid_field);

            /* Next field.  */
            hid_field =  hid_next_field;
        }

        /* Free the report.  */
        _ux_utility_memory_free(hid_report);

        /* Next report.  */
        hid_report =  hid_next_report;
    }

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

