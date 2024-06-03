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
/**   Host Simulator Controller Driver                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_hcd_sim_host.h                                   PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX host simulator. It is designed to work ONLY with the USBX      */ 
/*    device (slave) simulator.                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added controller disable,   */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added HCD uninitialize,     */
/*                                            resulting in version 6.1.2  */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added port status variable, */
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

#ifndef UX_HCD_SIM_HOST_H
#define UX_HCD_SIM_HOST_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  


/* Define simulator host generic definitions.  */

#define UX_HCD_SIM_HOST_CONTROLLER                              99
#define UX_HCD_SIM_HOST_MAX_PAYLOAD                             4096
#define UX_HCD_SIM_HOST_FRAME_DELAY                             4 
#define UX_HCD_SIM_HOST_PERIODIC_ENTRY_NB                       32    
#define UX_HCD_SIM_HOST_PERIODIC_ENTRY_MASK                     0x1f
#define UX_HCD_SIM_HOST_AVAILABLE_BANDWIDTH                     6000



/* Define simulator host completion code errors.  */

#define UX_HCD_SIM_HOST_NO_ERROR                                0x00
#define UX_HCD_SIM_HOST_ERROR_CRC                               0x01
#define UX_HCD_SIM_HOST_ERROR_BIT_STUFFING                      0x02
#define UX_HCD_SIM_HOST_ERROR_DATA_TOGGLE                       0x03
#define UX_HCD_SIM_HOST_ERROR_STALL                             0x04
#define UX_HCD_SIM_HOST_ERROR_DEVICE_NOT_RESPONDING             0x05
#define UX_HCD_SIM_HOST_ERROR_PID_FAILURE                       0x06
#define UX_HCD_SIM_HOST_ERROR_PID_UNEXPECTED                    0x07
#define UX_HCD_SIM_HOST_ERROR_DATA_OVERRRUN                     0x08
#define UX_HCD_SIM_HOST_ERROR_DATA_UNDERRUN                     0x09
#define UX_HCD_SIM_HOST_ERROR_BUFFER_OVERRRUN                   0x0c
#define UX_HCD_SIM_HOST_ERROR_BUFFER_UNDERRUN                   0x0d
#define UX_HCD_SIM_HOST_NOT_ACCESSED                            0x0e
#define UX_HCD_SIM_HOST_NAK                                     0x0f


/* Define simulator host structure.  */

typedef struct UX_HCD_SIM_HOST_STRUCT
{

    struct UX_HCD_STRUCT
                    *ux_hcd_sim_host_hcd_owner;
    ULONG           ux_hcd_sim_host_hcor;
    UINT            ux_hcd_sim_host_nb_root_hubs;
    ULONG           ux_hcd_sim_host_port_status[1];
    struct UX_HCD_SIM_HOST_ED_STRUCT         
                    *ux_hcd_sim_host_ed_list;
    struct UX_HCD_SIM_HOST_TD_STRUCT         
                    *ux_hcd_sim_host_td_list;
    struct UX_HCD_SIM_HOST_ISO_TD_STRUCT     
                    *ux_hcd_sim_host_iso_td_list;
    struct UX_HCD_SIM_HOST_ED_STRUCT         
                    *ux_hcd_sim_host_asynch_head_ed;
    struct UX_HCD_SIM_HOST_ED_STRUCT         
                    *ux_hcd_sim_host_asynch_current_ed;
    struct UX_HCD_SIM_HOST_ED_STRUCT         
                    *ux_hcd_sim_host_iso_head_ed;
    struct UX_HCD_SIM_HOST_ED_STRUCT         
                    *ux_hcd_sim_host_interrupt_ed_list[32];
    UINT            ux_hcd_sim_host_queue_empty;
    UINT            ux_hcd_sim_host_periodic_scheduler_active;
    UINT            ux_hcd_sim_host_interruptible;
    ULONG           ux_hcd_sim_host_interrupt_count;
#if !defined(UX_HOST_STANDALONE)
    UX_TIMER        ux_hcd_sim_host_timer;
#endif
} UX_HCD_SIM_HOST;


/* Define simulator host ED structure.  */

typedef struct UX_HCD_SIM_HOST_ED_STRUCT
{

    struct UX_HCD_SIM_HOST_TD_STRUCT        
                    *ux_sim_host_ed_tail_td;
    struct UX_HCD_SIM_HOST_TD_STRUCT        
                    *ux_sim_host_ed_head_td;
    struct UX_HCD_SIM_HOST_ED_STRUCT        
                    *ux_sim_host_ed_next_ed;
    struct UX_HCD_SIM_HOST_ED_STRUCT        
                    *ux_sim_host_ed_previous_ed;
    ULONG           ux_sim_host_ed_status;
    struct UX_ENDPOINT_STRUCT          
                    *ux_sim_host_ed_endpoint;
    ULONG           ux_sim_host_ed_toggle;   
    ULONG           ux_sim_host_ed_frame;    
} UX_HCD_SIM_HOST_ED;


/* Define simulator host ED bitmap.  */

#define UX_HCD_SIM_HOST_ED_STATIC                               0x80000000u
#define UX_HCD_SIM_HOST_ED_SKIP                                 0x40000000u
#define UX_HCD_SIM_HOST_ED_TRANSFER                             0x00100000u


/* Define simulator host TD structure.  */

typedef struct UX_HCD_SIM_HOST_TD_STRUCT
{                                                

    UCHAR *         ux_sim_host_td_buffer;
    ULONG           ux_sim_host_td_length;
    struct UX_HCD_SIM_HOST_TD_STRUCT        
                    *ux_sim_host_td_next_td;
    struct UX_TRANSFER_STRUCT          
                    *ux_sim_host_td_transfer_request;
    struct UX_HCD_SIM_HOST_TD_STRUCT        
                    *ux_sim_host_td_next_td_transfer_request;
    struct UX_HCD_SIM_HOST_ED_STRUCT        
                    *ux_sim_host_td_ed;
    ULONG           ux_sim_host_td_actual_length;
    ULONG           ux_sim_host_td_status;
    ULONG           ux_sim_host_td_direction;
    ULONG           ux_sim_host_td_toggle;
} UX_HCD_SIM_HOST_TD;


/* Define simulator host TD bitmap.  */

#define UX_HCD_SIM_HOST_TD_SETUP_PHASE                          0x00010000   
#define UX_HCD_SIM_HOST_TD_DATA_PHASE                           0x00020000   
#define UX_HCD_SIM_HOST_TD_STATUS_PHASE                         0x00040000   
#define UX_HCD_SIM_HOST_TD_OUT                                  0x00000800
#define UX_HCD_SIM_HOST_TD_IN                                   0x00001000
#define UX_HCD_SIM_HOST_TD_ACK_PENDING                          0x00002000       
#define UX_HCD_SIM_HOST_TD_TOGGLE_FROM_ED                       0x80000000


/* Define simulator host ISOCHRONOUS TD structure.  */

typedef struct UX_HCD_SIM_HOST_ISO_TD_STRUCT
{

    UCHAR *         ux_sim_host_iso_td_buffer;
    ULONG           ux_sim_host_iso_td_length;
    struct UX_HCD_SIM_HOST_ISO_TD_STRUCT    
                    *ux_sim_host_iso_td_next_td;
    struct UX_TRANSFER_STRUCT          
                    *ux_sim_host_iso_td_transfer_request;
    struct UX_HCD_SIM_HOST_ISO_TD_STRUCT    
                    *ux_sim_host_iso_td_next_td_transfer_request;
    struct UX_HCD_SIM_HOST_ED_STRUCT        
                    *ux_sim_host_iso_td_ed;
    ULONG           ux_sim_host_iso_td_actual_length;
    ULONG           ux_sim_host_iso_td_status;
    ULONG           ux_sim_host_iso_td_direction;
} UX_HCD_SIM_HOST_ISO_TD;


/* Define simulator host function prototypes.  */

VOID    _ux_hcd_sim_host_asynch_queue_process(UX_HCD_SIM_HOST *hcd_sim_host);
VOID    _ux_hcd_sim_host_asynch_schedule(UX_HCD_SIM_HOST *hcd_sim_host);
UINT    _ux_hcd_sim_host_asynchronous_endpoint_create(UX_HCD_SIM_HOST *hcd_sim_host, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_sim_host_asynchronous_endpoint_destroy(UX_HCD_SIM_HOST *hcd_sim_host, UX_ENDPOINT *endpoint);
UX_HCD_SIM_HOST_ED       
        *_ux_hcd_sim_host_ed_obtain(UX_HCD_SIM_HOST *hcd_sim_host);
VOID    _ux_hcd_sim_host_ed_td_clean(UX_HCD_SIM_HOST_ED *ed);
UINT    _ux_hcd_sim_host_endpoint_reset(UX_HCD_SIM_HOST *hcd_sim_host, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_sim_host_entry(UX_HCD *hcd, UINT function, VOID *parameter);
UINT    _ux_hcd_sim_host_frame_number_get(UX_HCD_SIM_HOST *hcd_sim_host, ULONG *frame_number);
VOID    _ux_hcd_sim_host_frame_number_set(UX_HCD_SIM_HOST *hcd_sim_host, ULONG frame_number);
UINT    _ux_hcd_sim_host_initialize(UX_HCD *hcd);
UINT    _ux_hcd_sim_host_uninitialize(UX_HCD_SIM_HOST *hcd);
UINT    _ux_hcd_sim_host_controller_disable(UX_HCD_SIM_HOST *hcd);
UINT    _ux_hcd_sim_host_interrupt_endpoint_create(UX_HCD_SIM_HOST *hcd_sim_host, UX_ENDPOINT *endpoint);
VOID    _ux_hcd_sim_host_iso_queue_process(UX_HCD_SIM_HOST *hcd_sim_host);
VOID    _ux_hcd_sim_host_iso_schedule(UX_HCD_SIM_HOST *hcd_sim_host);
UINT    _ux_hcd_sim_host_isochronous_endpoint_create(UX_HCD_SIM_HOST *hcd_sim_host, UX_ENDPOINT *endpoint);
UX_HCD_SIM_HOST_ISO_TD   
        *_ux_hcd_sim_host_isochronous_td_obtain(UX_HCD_SIM_HOST *hcd_sim_host);
UX_HCD_SIM_HOST_ED       
        *_ux_hcd_sim_host_least_traffic_list_get(UX_HCD_SIM_HOST *hcd_sim_host);
UINT    _ux_hcd_sim_host_periodic_endpoint_destroy(UX_HCD_SIM_HOST *hcd_sim_host, UX_ENDPOINT *endpoint);
VOID    _ux_hcd_sim_host_periodic_schedule(UX_HCD_SIM_HOST *hcd_sim_host);
UINT    _ux_hcd_sim_host_periodic_tree_create(UX_HCD_SIM_HOST *hcd_sim_host);
ULONG   _ux_hcd_sim_host_port_status_get(UX_HCD_SIM_HOST *hcd_sim_host, ULONG port_index);
UX_HCD_SIM_HOST_TD       
        *_ux_hcd_sim_host_regular_td_obtain(UX_HCD_SIM_HOST *hcd_sim_host);
UINT    _ux_hcd_sim_host_request_bulk_transfer(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_sim_host_request_control_transfer(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_sim_host_request_interrupt_transfer(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_sim_host_request_isochronous_transfer(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_sim_host_request_transfer(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request);
VOID    _ux_hcd_sim_host_timer_function(ULONG hcd_sim_host_addr);
UINT    _ux_hcd_sim_host_transaction_schedule(UX_HCD_SIM_HOST *hcd_sim_host, UX_HCD_SIM_HOST_ED *ed);
UINT    _ux_hcd_sim_host_transfer_abort(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_sim_host_port_reset(UX_HCD_SIM_HOST *hcd_sim_host, ULONG port_index);

UINT    _ux_hcd_sim_host_transfer_run(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request);

/* Define Device Simulator Class API prototypes.  */

#define ux_hcd_sim_host_initialize                 _ux_hcd_sim_host_initialize
/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif

#endif

