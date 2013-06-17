var yaw_fix = 0.0;

function tab_initialize_initial_setup() {
    // Fill in the accel trimms from CONFIG object
    $('input[name="pitch"]').val(CONFIG.accelerometerTrims[0]);
    $('input[name="roll"]').val(CONFIG.accelerometerTrims[1]);
    
    // UI Hooks
    $('a.calibrateAccel').click(function() {
        send_message(MSP_codes.MSP_ACC_CALIBRATION, MSP_codes.MSP_ACC_CALIBRATION);
    });
    
    $('a.calibrateMag').click(function() {
        send_message(MSP_codes.MSP_MAG_CALIBRATION, MSP_codes.MSP_MAG_CALIBRATION);
    });

    $('a.resetSettings').click(function() {
        send_message(MSP_codes.MSP_RESET_CONF, MSP_codes.MSP_RESET_CONF);
    });

    
    $('input[name="pitch"], input[name="roll"]').change(function() {
        // if any of the fields changed, unlock update button
        $('a.update').addClass('active');
    });    
    
    $('a.update').click(function() {
        CONFIG.accelerometerTrims[0] = parseInt($('input[name="pitch"]').val());
        CONFIG.accelerometerTrims[1] = parseInt($('input[name="roll"]').val());
        
        var buffer_out = new Array();
        buffer_out[0] = lowByte(CONFIG.accelerometerTrims[0]);
        buffer_out[1] = highByte(CONFIG.accelerometerTrims[0]);
        buffer_out[2] = lowByte(CONFIG.accelerometerTrims[1]);
        buffer_out[3] = highByte(CONFIG.accelerometerTrims[1]); 

        // Send over the new trims
        send_message(MSP_codes.MSP_SET_ACC_TRIM, buffer_out);
        
        // Save changes to EEPROM
        send_message(MSP_codes.MSP_EEPROM_WRITE, MSP_codes.MSP_EEPROM_WRITE);
        
        // remove the active status
        $(this).removeClass('active');
    });    

    // reset yaw button hook
    $('div#interactive_block > a.reset').click(function() {
        yaw_fix = SENSOR_DATA.kinematicsZ * - 1.0;
        console.log("YAW reset to 0");
    });  

    $('#content .backup').click(configuration_backup);
    
    $('#content .restore').click(configuration_restore);
    
    // enable data pulling
    timers.push(setInterval(data_poll, 50));    
}

function data_poll() {
    // Update cube
    var cube = $('div#cube');
    
    cube.css('-webkit-transform', 'rotateY(' + ((SENSOR_DATA.kinematicsZ * -1.0) - yaw_fix) + 'deg)');
    $('#cubePITCH', cube).css('-webkit-transform', 'rotateX(' + SENSOR_DATA.kinematicsY + 'deg)');
    $('#cubeROLL', cube).css('-webkit-transform', 'rotateZ(' + SENSOR_DATA.kinematicsX + 'deg)'); 

    // Update Compass
    $('div#compass .pointer').css('-webkit-transform', 'rotate(' + (SENSOR_DATA.kinematicsZ) + 'deg)'); 
    $('div#compass .value').html(SENSOR_DATA.kinematicsZ + '&deg;');
    
    // Update voltage indicator
    $('span.bat-voltage').html(BATTERY.voltage + ' V');
    
    // Request new data
    send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
    send_message(MSP_codes.MSP_ATTITUDE, MSP_codes.MSP_ATTITUDE); 
    send_message(MSP_codes.MSP_COMP_GPS, MSP_codes.MSP_COMP_GPS);
    send_message(MSP_codes.MSP_BAT, MSP_codes.MSP_BAT);
}