function tab_initialize_sensors() {
}

function sensor_array_pull() {
    // Update UI
    
    // Request new data
    send_message(MSP_codes.MSP_RAW_IMU, MSP_codes.MSP_RAW_IMU);
    send_message(MSP_codes.MSP_ALTITUDE, MSP_codes.MSP_ALTITUDE);
}
