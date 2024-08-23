/* This is a small demo of the USBX   */

#include "ux_api.h"
#include "ux_system.h"
#include "ux_utility.h"
#include "ux_host_class_dpump.h"
#include "ux_device_class_dpump.h"


/* Define USBX demo constants.  */

#define UX_DEMO_STACK_SIZE      4096
#define UX_DEMO_MEMORY_SIZE     (28*1024)


/* Define the counters used in the demo application...  */

ULONG                           thread_0_counter;
ULONG                           thread_1_counter;
ULONG                           error_counter;


/* Define USBX demo global variables.  */

ULONG                           ux_demo_memory_buffer[(UX_DEMO_MEMORY_SIZE + UX_DEMO_STACK_SIZE * 2) / sizeof(ULONG)];

unsigned char                   host_out_buffer[UX_HOST_CLASS_DPUMP_PACKET_SIZE];
unsigned char                   host_in_buffer[UX_HOST_CLASS_DPUMP_PACKET_SIZE];
unsigned char                   device_buffer[UX_HOST_CLASS_DPUMP_PACKET_SIZE];

UX_HOST_CLASS                   *class_driver;
UX_HOST_CLASS_DPUMP             *dpump;
UX_SLAVE_CLASS_DPUMP            *dpump_device;


#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED 50
UCHAR device_framework_full_speed[] = { 

    /* Device descriptor */
        0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x08,
        0xec, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x01,                                      

    /* Configuration descriptor */
        0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0xc0,
        0x32, 

    /* Interface descriptor */
        0x09, 0x04, 0x00, 0x00, 0x02, 0x99, 0x99, 0x99,
        0x00,

    /* Endpoint descriptor (Bulk Out) */
        0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,

    /* Endpoint descriptor (Bulk In) */
        0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00 
    };
    
    
#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED 60
UCHAR device_framework_high_speed[] = { 

    /* Device descriptor */
        0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
        0x0a, 0x07, 0x25, 0x40, 0x01, 0x00, 0x01, 0x02,
        0x03, 0x01,                                      

    /* Device qualifier descriptor */
        0x0a, 0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
        0x01, 0x00,

    /* Configuration descriptor */
        0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0xc0,
        0x32, 

    /* Interface descriptor */
        0x09, 0x04, 0x00, 0x00, 0x02, 0x99, 0x99, 0x99,
        0x00,

    /* Endpoint descriptor (Bulk Out) */
        0x07, 0x05, 0x01, 0x02, 0x00, 0x02, 0x00,

    /* Endpoint descriptor (Bulk In) */
        0x07, 0x05, 0x82, 0x02, 0x00, 0x02, 0x00 
    };
    
    /* String Device Framework :
     Byte 0 and 1 : Word containing the language ID : 0x0904 for US
     Byte 2       : Byte containing the index of the descriptor
     Byte 3       : Byte containing the length of the descriptor string
    */
   
#define STRING_FRAMEWORK_LENGTH 38
UCHAR string_framework[] = { 

    /* Manufacturer string descriptor : Index 1 */
        0x09, 0x04, 0x01, 0x0c, 
        0x45, 0x78, 0x70, 0x72,0x65, 0x73, 0x20, 0x4c, 
        0x6f, 0x67, 0x69, 0x63,

    /* Product string descriptor : Index 2 */
        0x09, 0x04, 0x02, 0x0c, 
        0x44, 0x61, 0x74, 0x61, 0x50, 0x75, 0x6d, 0x70, 
        0x44, 0x65, 0x6d, 0x6f,  

    /* Serial Number string descriptor : Index 3 */
        0x09, 0x04, 0x03, 0x04, 
        0x30, 0x30, 0x30, 0x31
    };


    /* Multiple languages are supported on the device, to add
       a language besides English, the unicode language code must
       be appended to the language_id_framework array and the length
       adjusted accordingly. */
#define LANGUAGE_ID_FRAMEWORK_LENGTH 2
UCHAR language_id_framework[] = { 

    /* English. */
        0x09, 0x04
    };


/* Define prototypes for external Host Controller's (HCDs), classes and clients.  */

VOID                tx_demo_instance_activate(VOID  *dpump_instance);             
VOID                tx_demo_instance_deactivate(VOID *dpump_instance);

UINT                _ux_host_class_dpump_entry(UX_HOST_CLASS_COMMAND *command);
UINT                ux_hcd_sim_initialize(UX_HCD *hcd);
UINT                _ux_host_class_dpump_write(UX_HOST_CLASS_DPUMP *dpump, UCHAR * data_pointer, 
                                    ULONG requested_length, ULONG *actual_length);
UINT                _ux_host_class_dpump_read (UX_HOST_CLASS_DPUMP *dpump, UCHAR *data_pointer, 
                                    ULONG requested_length, ULONG *actual_length);

TX_THREAD           tx_demo_thread_host_simulation;
TX_THREAD           tx_demo_thread_device_simulation;
void                tx_demo_thread_host_simulation_entry(ULONG);
void                tx_demo_thread_device_simulation_entry(ULONG);
VOID                error_handler(void);


/* Define the entry point.  */

int  main()
{

    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();

    return(0);
}


/* Define what the initial system looks like.  */

void  tx_application_define(void *first_unused_memory)
{

CHAR                            *stack_pointer;
CHAR                            *memory_pointer;
UINT                            status;
UX_SLAVE_CLASS_DPUMP_PARAMETER  parameter;


    UX_PARAMETER_NOT_USED(first_unused_memory);

    /* Initialize the free memory pointer.  */
    stack_pointer = (CHAR *) ux_demo_memory_buffer;
    memory_pointer = stack_pointer + (UX_DEMO_STACK_SIZE * 2);
    
    /* Initialize USBX Memory.  */
    status =  ux_system_initialize(memory_pointer, UX_DEMO_MEMORY_SIZE, UX_NULL, 0);

    /* Check for error.  */
    if (status != UX_SUCCESS)
        error_handler();

    /* The code below is required for installing the host portion of USBX.  */
    status =  ux_host_stack_initialize(UX_NULL);

    /* Check for error.  */
    if (status != UX_SUCCESS)
        error_handler();

    /* Register all the host class drivers for this USBX implementation.  */
    status =  ux_host_stack_class_register(_ux_system_host_class_dpump_name, ux_host_class_dpump_entry);

    /* Check for error.  */
    if (status != UX_SUCCESS)
        error_handler();

    /* Register all the USB host controllers available in this system */
    status =  ux_host_stack_hcd_register(_ux_system_host_hcd_simulator_name, ux_hcd_sim_host_initialize,0,0);

    /* Check for error.  */
    if (status != UX_SUCCESS)
        error_handler();

    /* The code below is required for installing the device portion of USBX */
    status =  _ux_device_stack_initialize(device_framework_high_speed, DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED,
                                       device_framework_full_speed, DEVICE_FRAMEWORK_LENGTH_FULL_SPEED,
                                       string_framework, STRING_FRAMEWORK_LENGTH,
                                       language_id_framework, LANGUAGE_ID_FRAMEWORK_LENGTH, UX_NULL);

    /* Check for error.  */
    if (status != UX_SUCCESS)
        error_handler();

    /* Set the parameters for callback when insertion/extraction of a Data Pump device.  */
    parameter.ux_slave_class_dpump_instance_activate   =  tx_demo_instance_activate;
    parameter.ux_slave_class_dpump_instance_deactivate =  tx_demo_instance_deactivate;

    /* Initialize the device dpump class. The class is connected with interface 0 */
     status =  _ux_device_stack_class_register(_ux_system_slave_class_dpump_name, _ux_device_class_dpump_entry, 
                                               1, 0, &parameter);

    /* Check for error.  */
    if (status != UX_SUCCESS)
        error_handler();

    /* Initialize the simulated device controller.  */
    status =  _ux_dcd_sim_slave_initialize();

    /* Check for error.  */
    if (status != UX_SUCCESS)
        error_handler();
    
    /* Create the main host simulation thread.  */
    status =  tx_thread_create(&tx_demo_thread_host_simulation, "tx demo host simulation", tx_demo_thread_host_simulation_entry, 0,  
            stack_pointer, UX_DEMO_STACK_SIZE, 
            20, 20, 1, TX_AUTO_START);

    /* Check for error.  */
    if (status != TX_SUCCESS)
        error_handler();
            
    /* Create the main demo thread.  */
    status =  tx_thread_create(&tx_demo_thread_device_simulation, "tx demo slave simulation", tx_demo_thread_device_simulation_entry, 0,  
            stack_pointer + UX_DEMO_STACK_SIZE, UX_DEMO_STACK_SIZE, 
            20, 20, 1, TX_AUTO_START);
      
    /* Check for error.  */
    if (status != TX_SUCCESS)
        error_handler();
}


void  tx_demo_thread_host_simulation_entry(ULONG arg)
{

UINT            status;
ULONG           actual_length;
UCHAR           current_char;
UX_HOST_CLASS   *class;


    UX_PARAMETER_NOT_USED(arg);

    /* Find the main data pump container.  */
    status =  ux_host_stack_class_get(_ux_system_host_class_dpump_name, &class);

    /* Check for error.  */
    if (status != UX_SUCCESS)
        error_handler();

    /* We get the first instance of the data pump device.  */
    do
    {

        status =  ux_host_stack_class_instance_get(class, 0, (VOID **) &dpump);
        tx_thread_relinquish();
    } while (status != UX_SUCCESS);

    /* We still need to wait for the data pump status to be live.  */
    while (dpump -> ux_host_class_dpump_state != UX_HOST_CLASS_INSTANCE_LIVE)
    {

        tx_thread_relinquish();
    }

    /* At this point, the data pump class has been found.  Now use the 
       data pump to send and receive data between the host and device.  */    
    
    /* We start with a 'A' in buffer.  */
    current_char = 'A';

    while(1)
    {

        /* Increment thread counter.  */
        thread_0_counter++;

        /* Initialize the write buffer. */
        _ux_utility_memory_set(host_out_buffer, current_char, UX_HOST_CLASS_DPUMP_PACKET_SIZE); /* Use case of memset is verified. */

        /* Increment the character in buffer.  */
        current_char++;
        
        /* Check for upper alphabet limit.  */
        if (current_char > 'Z') 
            current_char =  'A';
        
        /* Write to the host Data Pump Bulk out endpoint.  */
        status =  _ux_host_class_dpump_write (dpump, host_out_buffer, UX_HOST_CLASS_DPUMP_PACKET_SIZE, &actual_length);

        /* Check for error.  */
        if (status != UX_SUCCESS)
            error_handler();

        /* Verify that the status and the amount of data is correct.  */
        if ((status != UX_SUCCESS) || actual_length != UX_HOST_CLASS_DPUMP_PACKET_SIZE)
            return;
        
        /* Read to the Data Pump Bulk out endpoint.  */
        status =  _ux_host_class_dpump_read (dpump, host_in_buffer, UX_HOST_CLASS_DPUMP_PACKET_SIZE, &actual_length);

        /* Verify that the status and the amount of data is correct.  */
        if ((status != UX_SUCCESS) || actual_length != UX_HOST_CLASS_DPUMP_PACKET_SIZE)
            error_handler();
            
        /* Relinquish to other thread.  */
        tx_thread_relinquish();
    }        
}


void  tx_demo_thread_device_simulation_entry(ULONG arg)
{

UINT    status;
ULONG   actual_length;


    UX_PARAMETER_NOT_USED(arg);

    while(1)
    {

        /* Ensure the dpump class on the device is still alive.  */
        while (dpump_device != UX_NULL)
        {

            /* Increment thread counter.  */
            thread_1_counter++;

            /* Read from the device data pump.  */
            status =  _ux_device_class_dpump_read(dpump_device, device_buffer, UX_HOST_CLASS_DPUMP_PACKET_SIZE, &actual_length); 

            /* Verify that the status and the amount of data is correct.  */
            if ((status != UX_SUCCESS) || actual_length != UX_HOST_CLASS_DPUMP_PACKET_SIZE)
                error_handler();

            /* Now write to the device data pump.  */
            status =  _ux_device_class_dpump_write(dpump_device, device_buffer, UX_HOST_CLASS_DPUMP_PACKET_SIZE, &actual_length); 

            /* Verify that the status and the amount of data is correct.  */
            if ((status != UX_SUCCESS) || actual_length != UX_HOST_CLASS_DPUMP_PACKET_SIZE)
                error_handler();
        }

        /* Relinquish to other thread.  */
        tx_thread_relinquish();
    }
}            

VOID  tx_demo_instance_activate(VOID *dpump_instance) 
{

    /* Save the DPUMP instance.  */
    dpump_device = (UX_SLAVE_CLASS_DPUMP *) dpump_instance;
}           

VOID  tx_demo_instance_deactivate(VOID *dpump_instance)
{

    UX_PARAMETER_NOT_USED(dpump_instance);

    /* Reset the DPUMP instance.  */
    dpump_device = UX_NULL;
}


VOID  error_handler(void)
{

    /* Increment error counter.  */
    error_counter++;

    while(1)
    {
    
        /* Error - just spin here!  Look at call tree in debugger 
           to see where the error occurred.  */
    }
}

