var yaw_fix = 0.0;

function tab_initialize_initial_setup() {
    $('a.calibrateAccel').click(function() {
        send_message(MSP_codes.MSP_ACC_CALIBRATION, MSP_codes.MSP_ACC_CALIBRATION);
    });
    
    $('a.calibrateMag').click(function() {
        send_message(MSP_codes.MSP_MAG_CALIBRATION, MSP_codes.MSP_MAG_CALIBRATION);
    });

    $('a.resetSettings').click(function() {
        send_message(MSP_codes.MSP_RESET_CONF, MSP_codes.MSP_RESET_CONF);
    });    
    

    // reset yaw button hook
    $('div#interactive_block > a.reset').click(function() {
        yaw_fix = SENSOR_DATA.kinematicsZ * - 1.0;
        console.log("YAW reset to 0");
    });  
    
    // enable kinematics data pulling
    timers.push(setInterval(kinematics_poll, 50));    
}

function kinematics_poll() {
    // Update cube
    var cube = $('div#cube');
    
    cube.css('-webkit-transform', 'rotateY(' + ((SENSOR_DATA.kinematicsZ * -1.0) - yaw_fix) + 'deg)');
    $('#cubePITCH', cube).css('-webkit-transform', 'rotateX(' + SENSOR_DATA.kinematicsY + 'deg)');
    $('#cubeROLL', cube).css('-webkit-transform', 'rotateZ(' + SENSOR_DATA.kinematicsX + 'deg)'); 
    
    // Request new data
    send_message(MSP_codes.MSP_ATTITUDE, MSP_codes.MSP_ATTITUDE); 
}