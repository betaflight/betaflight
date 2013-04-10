function tab_initialize_auxiliary_configuration() {
    // Request AUX_CONFIG names
    send_message(MSP_codes.MSP_BOXNAMES, MSP_codes.MSP_BOXNAMES);
    
    // Request AUX_CONFIG values
    send_message(MSP_codes.MSP_BOX, MSP_codes.MSP_BOX);
}