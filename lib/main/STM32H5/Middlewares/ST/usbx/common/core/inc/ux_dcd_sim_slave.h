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
/**   Slave Simulator Controller Driver                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_dcd_sim_slave.h                                  PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX slave simulator. It is designed to work ONLY with the USBX     */ 
/*    host simulator.                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added HCD connected to,     */
/*                                            supported bi-dir-endpoints, */
/*                                            resulting in version 6.1.6  */
/*  08-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DCD_SIM_SLAVE_H
#define UX_DCD_SIM_SLAVE_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  


/* Define USB slave simulator major equivalences.  */

#define UX_DCD_SIM_SLAVE_SLAVE_CONTROLLER                       98
#define UX_DCD_SIM_SLAVE_MAX_ED                                 16


/* Define USB slave simulator error code register bits.  */

#define UX_DCD_SIM_SLAVE_ERROR_TRANSMISSION_OK                  0x00000001u
#define UX_DCD_SIM_SLAVE_ERROR_CODE_MASK                        0x0000000eu
#define UX_DCD_SIM_SLAVE_ERROR_CODE_SHIFT                       0x00000001u
#define UX_DCD_SIM_SLAVE_ERROR_CODE_PID_ERROR                   0x00000001u
#define UX_DCD_SIM_SLAVE_ERROR_CODE_PID_UNKNOWN                 0x00000002u
#define UX_DCD_SIM_SLAVE_ERROR_CODE_UNEXPECTED_PACKET           0x00000003u
#define UX_DCD_SIM_SLAVE_ERROR_CODE_TOKEN_CRC                   0x00000004u
#define UX_DCD_SIM_SLAVE_ERROR_CODE_DATA_CRC                    0x00000005u
#define UX_DCD_SIM_SLAVE_ERROR_CODE_TIME_OUT                    0x00000006u
#define UX_DCD_SIM_SLAVE_ERROR_CODE_BABBLE                      0x00000007u
#define UX_DCD_SIM_SLAVE_ERROR_CODE_UNEXPECTED_EOP              0x00000008u
#define UX_DCD_SIM_SLAVE_ERROR_CODE_NAK                         0x00000009u
#define UX_DCD_SIM_SLAVE_ERROR_CODE_STALLED                     0x0000000au
#define UX_DCD_SIM_SLAVE_ERROR_CODE_OVERFLOW                    0x0000000bu
#define UX_DCD_SIM_SLAVE_ERROR_CODE_EMPTY_PACKET                0x0000000cu
#define UX_DCD_SIM_SLAVE_ERROR_CODE_BIT_STUFFING                0x0000000du
#define UX_DCD_SIM_SLAVE_ERROR_CODE_SYNC_ERROR                  0x0000000eu
#define UX_DCD_SIM_SLAVE_ERROR_CODE_DATA_TOGGLE                 0x0000000fu


/* Define USB slave simulator physical endpoint status definition.  */

#define UX_DCD_SIM_SLAVE_ED_STATUS_UNUSED                       0u
#define UX_DCD_SIM_SLAVE_ED_STATUS_USED                         1u
#define UX_DCD_SIM_SLAVE_ED_STATUS_TRANSFER                     2u
#define UX_DCD_SIM_SLAVE_ED_STATUS_STALLED                      4u
#define UX_DCD_SIM_SLAVE_ED_STATUS_DONE                         8u


/* Define USB slave simulator physical endpoint structure.  */

typedef struct UX_DCD_SIM_SLAVE_ED_STRUCT 
{

    ULONG           ux_sim_slave_ed_status;
    ULONG           ux_sim_slave_ed_index;
    ULONG           ux_sim_slave_ed_payload_length;
    ULONG           ux_sim_slave_ed_ping_pong;
    ULONG           ux_sim_slave_ed_status_register;
    ULONG           ux_sim_slave_ed_configuration_value;
    struct UX_SLAVE_ENDPOINT_STRUCT             
                    *ux_sim_slave_ed_endpoint;
} UX_DCD_SIM_SLAVE_ED;


/* Define USB slave simulator DCD structure definition.  */

typedef struct UX_DCD_SIM_SLAVE_STRUCT
{                                 

    struct UX_SLAVE_DCD_STRUCT                 
                    *ux_dcd_sim_slave_dcd_owner;
    struct UX_DCD_SIM_SLAVE_ED_STRUCT              
                    ux_dcd_sim_slave_ed[UX_DCD_SIM_SLAVE_MAX_ED];
#ifdef UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT
    struct UX_DCD_SIM_SLAVE_ED_STRUCT              
                    ux_dcd_sim_slave_ed_in[UX_DCD_SIM_SLAVE_MAX_ED];
#endif
    UINT            (*ux_dcd_sim_slave_dcd_control_request_process_hub)(UX_SLAVE_TRANSFER *transfer_request);
    VOID            *ux_dcd_sim_slave_hcd;
} UX_DCD_SIM_SLAVE;


/* Define slave simulator function prototypes.  */

UINT    _ux_dcd_sim_slave_address_set(UX_DCD_SIM_SLAVE *dcd_sim_slave, ULONG address);
UINT    _ux_dcd_sim_slave_endpoint_create(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_sim_slave_endpoint_destroy(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_sim_slave_endpoint_reset(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_sim_slave_endpoint_stall(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_sim_slave_endpoint_status(UX_DCD_SIM_SLAVE *dcd_sim_slave, ULONG endpoint_index);
UINT    _ux_dcd_sim_slave_frame_number_get(UX_DCD_SIM_SLAVE *dcd_sim_slave, ULONG *frame_number);
UINT    _ux_dcd_sim_slave_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter);
UINT    _ux_dcd_sim_slave_initialize(VOID);
UINT    _ux_dcd_sim_slave_initialize_complete(VOID);
UINT    _ux_dcd_sim_slave_state_change(UX_DCD_SIM_SLAVE *dcd_sim_slave, ULONG state);
UINT    _ux_dcd_sim_slave_transfer_request(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_TRANSFER *transfer_request);
UINT    _ux_dcd_sim_slave_transfer_run(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_TRANSFER *transfer_request);
UINT    _ux_dcd_sim_slave_transfer_abort(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_TRANSFER *transfer_request);

/* Define Device Simulator Class API prototypes.  */

#define ux_dcd_sim_slave_initialize                 _ux_dcd_sim_slave_initialize

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif

#endif

