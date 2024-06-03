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
/**   Asix Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_asix.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_asix_setup                           PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function prepares the Asix chip Phy and rx and xmit registers. */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    asix                                         Pointer to asix class  */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Transfer request              */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_asix_activate                                        */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_asix_setup(UX_HOST_CLASS_ASIX *asix)
{

UCHAR                       *setup_buffer;
UX_ENDPOINT                 *control_endpoint;
UX_TRANSFER                 *transfer_request;
ULONG                        phy_register_value;
UINT                        status;

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &asix -> ux_host_class_asix_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the buffer.  */
    setup_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_ASIX_SETUP_BUFFER_SIZE);
    if (setup_buffer == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Get the Ethernet Phy Address register.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  2;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_READ_PHY_ID;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_IN | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if ((status != UX_SUCCESS) || (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS) || 
            (transfer_request -> ux_transfer_request_actual_length != 2))
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }
    
    /* Extract the  PHY IDs and the type.  */
    asix -> ux_host_class_asix_primary_phy_id = *(setup_buffer + UX_HOST_CLASS_ASIX_PHY_ID_PRIMARY) & UX_HOST_CLASS_ASIX_PHY_ID_MASK;
    asix -> ux_host_class_asix_primary_phy_type = (*(setup_buffer + UX_HOST_CLASS_ASIX_PHY_ID_PRIMARY)  >> UX_HOST_CLASS_ASIX_PHY_TYPE_SHIFT) & UX_HOST_CLASS_ASIX_PHY_TYPE_MASK;
    
    asix -> ux_host_class_asix_secondary_phy_id = *(setup_buffer + UX_HOST_CLASS_ASIX_PHY_ID_SECONDARY) & UX_HOST_CLASS_ASIX_PHY_ID_MASK;
    asix -> ux_host_class_asix_secondary_phy_type = (*(setup_buffer + UX_HOST_CLASS_ASIX_PHY_ID_SECONDARY)  >> UX_HOST_CLASS_ASIX_PHY_TYPE_SHIFT) & UX_HOST_CLASS_ASIX_PHY_TYPE_MASK;
    
    /* Set the GPIO 2 register.  */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_GPIO_STATUS;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  (UX_HOST_CLASS_ASIX_GPIO_RSE | UX_HOST_CLASS_ASIX_GPIO_GPO2EN | UX_HOST_CLASS_ASIX_GPIO_GPO_2);
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Set the Software PHY Select register.  */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_SW_PHY_SELECT_STATUS;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  1;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Perform a software reset of IPPD.  */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_SW_RESET;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_ASIX_SW_RESET_IPPD;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Perform a software reset.  */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_SW_RESET;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }


    /* Perform a software reset of IPRL and PRL.  */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_SW_RESET;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  (UX_HOST_CLASS_ASIX_SW_RESET_IPRL | UX_HOST_CLASS_ASIX_SW_RESET_PRL);
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Write the value of the Receive Control register.  */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_RX_CTL;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0;
    transfer_request -> ux_transfer_request_index               =  UX_HOST_CLASS_ASIX_RXCR_MFB_2048;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Get the Ethernet Phy Address register.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  UX_HOST_CLASS_ASIX_NODE_ID_LENGTH;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_READ_NODE_ID;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_IN | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS || 
        transfer_request -> ux_transfer_request_actual_length != UX_HOST_CLASS_ASIX_NODE_ID_LENGTH)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }


    /* Copy the node id into the Asix instance.  */
    _ux_utility_memory_copy(asix -> ux_host_class_asix_node_id, setup_buffer, UX_HOST_CLASS_ASIX_NODE_ID_LENGTH); /* Use case of memcpy is verified. */

    /* Request ownership of Serial Management Interface.  */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_OWN_SMI ;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }


    /* Get the value of the PHYIDR1 in the PHY register.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  2;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_READ_PHY_REG;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_IN | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  asix -> ux_host_class_asix_primary_phy_id;
    transfer_request -> ux_transfer_request_index               =  UX_HOST_CLASS_ASIX_PHY_REG_PHYIDR1;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS || 
        transfer_request -> ux_transfer_request_actual_length != 2)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Extract the Vendor model number and model revision number.  First get the value
       in proper 16 bit little endian.  */
    phy_register_value = _ux_utility_short_get(setup_buffer);

    /* Get the model revision number.  */
    asix -> ux_host_class_asix_model_revision_number = (phy_register_value >> UX_HOST_CLASS_ASIX_PHY_REG_PHYIDR1_MDL_REV_SHIFT) & UX_HOST_CLASS_ASIX_PHY_REG_PHYIDR1_MDL_REV_MASK;

    /* Get the vendor model number.  */
    asix -> ux_host_class_asix_vendor_model_number   = (phy_register_value >> UX_HOST_CLASS_ASIX_PHY_REG_PHYIDR1_VNDR_REV_SHIFT) & UX_HOST_CLASS_ASIX_PHY_REG_PHYIDR1_VNDR_REV_MASK ;


    /* Perform a software reset.  External Phy Reset Pin level. */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_SW_RESET;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_ASIX_SW_RESET_PRL;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Perform a software reset.  Internal Phy reset Control and external Phy Reset Pin level. */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_SW_RESET;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  (UX_HOST_CLASS_ASIX_SW_RESET_IPRL | UX_HOST_CLASS_ASIX_SW_RESET_PRL);
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }


    /* Write the value of the BMCR register in the PHY register.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  2;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_PHY_REG;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  asix -> ux_host_class_asix_primary_phy_id;
    transfer_request -> ux_transfer_request_index               =  UX_HOST_CLASS_ASIX_PHY_REG_BMCR;

    /* Set the value for the PHY reg. */
    phy_register_value =  UX_HOST_CLASS_ASIX_PHY_REG_BMCR_RESET;    

    /* Insert the value into the target buffer.  */
    _ux_utility_short_put(setup_buffer, (USHORT)phy_register_value);
    
    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS ||
        transfer_request -> ux_transfer_request_actual_length != 2)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }


    /* Read the value of the BMCR register in the PHY register.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  2;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_READ_PHY_REG;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_IN | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  asix -> ux_host_class_asix_primary_phy_id;
    transfer_request -> ux_transfer_request_index               =  UX_HOST_CLASS_ASIX_PHY_REG_BMCR;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS || transfer_request -> ux_transfer_request_actual_length != 2)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Extract the speed selected by the PHY.  */
    phy_register_value = _ux_utility_short_get(setup_buffer);

    /* Isolate the speed and memorize it.  */
    if (phy_register_value & UX_HOST_CLASS_ASIX_PHY_REG_BMCR_SPEED_100MBS)
        
        /* Select 100 MBPS as our speed.  */
        asix -> ux_host_class_asix_speed_selected = UX_HOST_CLASS_ASIX_SPEED_SELECTED_100MPBS;
        
    else
            
        /* Select 10 MBPS as our speed.  */
        asix -> ux_host_class_asix_speed_selected = UX_HOST_CLASS_ASIX_SPEED_SELECTED_10MPBS;
        
    /* Set the value of the ANAR in the PHY register.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  2;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_PHY_REG;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  asix -> ux_host_class_asix_primary_phy_id;
    transfer_request -> ux_transfer_request_index               =  UX_HOST_CLASS_ASIX_PHY_REG_ANAR;

    /* Set the value for the PHY reg. */
    phy_register_value =  (UX_HOST_CLASS_ASIX_PHY_REG_ANAR_DEFAULT_SELECTOR |
                          UX_HOST_CLASS_ASIX_PHY_REG_ANAR_10_HD             |
                          UX_HOST_CLASS_ASIX_PHY_REG_ANAR_10_FD             |
                          UX_HOST_CLASS_ASIX_PHY_REG_ANAR_TX_HD             |
                          UX_HOST_CLASS_ASIX_PHY_REG_ANAR_TX_FD             |
                          UX_HOST_CLASS_ASIX_PHY_REG_ANAR_PAUSE);           

    /* Insert the value into the target buffer.  */
    _ux_utility_short_put(setup_buffer, (USHORT)phy_register_value);
    

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS || 
        transfer_request -> ux_transfer_request_actual_length != 2)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Set the value of the BMCR in the PHY register.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  2;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_PHY_REG;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  asix -> ux_host_class_asix_primary_phy_id;
    transfer_request -> ux_transfer_request_index               =  UX_HOST_CLASS_ASIX_PHY_REG_BMCR;

    /* Check speed.  */
    if (asix -> ux_host_class_asix_speed_selected == UX_HOST_CLASS_ASIX_SPEED_SELECTED_100MPBS)
    
        /* Set speed at 100MBPS.  */
        phy_register_value =  UX_HOST_CLASS_ASIX_PHY_REG_BMCR_SPEED_100MBS;

    /* Set the value for the PHY reg. */
    phy_register_value |=  (UX_HOST_CLASS_ASIX_PHY_REG_BMCR_AUTO_NEGOTIATION | UX_HOST_CLASS_ASIX_PHY_REG_BMCR_RESTART_NEG |
                            UX_HOST_CLASS_ASIX_PHY_REG_BMCR_DUPLEX_MODE);

    /* Insert the value into the target buffer.  */
    _ux_utility_short_put(setup_buffer, (USHORT)phy_register_value);

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS || 
        transfer_request -> ux_transfer_request_actual_length != 2)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }


    /* Check speed.  */
    if (asix -> ux_host_class_asix_speed_selected == UX_HOST_CLASS_ASIX_SPEED_SELECTED_100MPBS)
    
        /* Set speed at 100MBPS.  */
        transfer_request -> ux_transfer_request_value =  UX_HOST_CLASS_ASIX_MEDIUM_PS;
    
    /* Write the value of the Medium Mode. */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_MEDIUM_MODE;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               |=  (UX_HOST_CLASS_ASIX_MEDIUM_FD | UX_HOST_CLASS_ASIX_MEDIUM_BIT2 | UX_HOST_CLASS_ASIX_MEDIUM_RFC_ENABLED |
                                                                    UX_HOST_CLASS_ASIX_MEDIUM_TFC_ENABLED | UX_HOST_CLASS_ASIX_MEDIUM_RE_ENABLED);
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }


    /* Write the value of the IPG0/IPG1/IPG2. */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_IPG012;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_ASIX_PPG0_IPG1;
    transfer_request -> ux_transfer_request_index               =  UX_HOST_CLASS_ASIX_PPG2;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Release SMI ownership. */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_RELEASE_SMI;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Set the Rx Control register value.  */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_RX_CTL;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  (UX_HOST_CLASS_ASIX_RXCR_AB | UX_HOST_CLASS_ASIX_RXCR_SO | UX_HOST_CLASS_ASIX_RXCR_MFB_2048);
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);

    }

    /* Free all used resources.  */
    _ux_utility_memory_free(setup_buffer);

    /* Return completion status.  */
    return(status);
}

