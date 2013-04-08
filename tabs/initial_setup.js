function tab_initialize_initial_setup() {
    $('a.calibrateAccel').click(function() {
        send_message(MSP_codes.MSP_ACC_CALIBRATION, MSP_codes.MSP_ACC_CALIBRATION);
    });
    
    $('a.calibrateMag').click(function() {
        send_message(MSP_codes.MSP_MAG_CALIBRATION, MSP_codes.MSP_MAG_CALIBRATION);
    });
}