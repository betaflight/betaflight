function tab_initialize_initial_setup() {
    ga_tracker.sendAppView('Initial Setup');
    
    $('#content').load("./tabs/initial_setup.html", function() {
        GUI.active_tab = 'initial_setup';
        
        send_message(MSP_codes.MSP_IDENT, MSP_codes.MSP_IDENT, false, function() {
            send_message(MSP_codes.MSP_ACC_TRIM, MSP_codes.MSP_ACC_TRIM, false, function() {
                send_message(MSP_codes.MSP_MISC, MSP_codes.MSP_MISC, false, function() {
                    var yaw_fix = 0.0;
                    
                    // Fill in misc stuff
                    $('input[name="mincellvoltage"]').val(MISC.vbatmincellvoltage);
                    $('input[name="maxcellvoltage"]').val(MISC.vbatmaxcellvoltage);
                    $('input[name="voltagescale"]').val(MISC.vbatscale);
                    
                    $('input[name="minthrottle"]').val(MISC.minthrottle);
                    $('input[name="maxthrottle"]').val(MISC.maxthrottle);
                    $('input[name="failsafe_throttle"]').val(MISC.failsafe_throttle);
                    $('input[name="mincommand"]').val(MISC.mincommand);
                    
                    $('input[name="mag_declination"]').val(MISC.mag_declination / 10);
                    
                    // Fill in the accel trimms from CONFIG object
                    $('input[name="pitch"]').val(CONFIG.accelerometerTrims[0]);
                    $('input[name="roll"]').val(CONFIG.accelerometerTrims[1]);
                    
                    // Display multiType
                    var str = '';
                    switch (CONFIG.multiType) {
                        case 1: // TRI
                            str = 'TRI';
                            break;
                        case 2: // QUAD +
                            str = 'Quad +';
                            break;
                        case 3: // QUAD X
                            str = 'Quad X';
                            break;
                        case 4: // BI
                            str = 'BI';
                            break;
                        case 5: // GIMBAL
                            str = 'Gimbal';
                            break;
                        case 6: // Y6
                            str = 'Y6';
                            break;
                        case 7: // HEX 6
                            str = 'HEX 6';
                            break;
                        case 8: // FLYING_WING
                            str = 'Flying Wing';
                            break;
                        case 9: // Y4
                            str = 'Y4';
                            break;
                        case 10: // HEX6 X
                            str = 'HEX6 X';
                            break;
                        case 11: // OCTO X8
                        case 12:
                        case 13:
                            str = 'OCTO X8';
                            break;
                        case 14: // AIRPLANE
                            str = 'Airplane';
                            break;
                        case 15: // Heli 120 
                            str = 'Heli 120';
                            break;
                        case 16: // Heli 90 
                            str = 'Heli 90';
                            break;
                        case 17: // Vtail 
                            str = 'Vtail';
                            break;
                        case 18: // HEX6 H
                            str = 'HEX6 H';
                            break;
                        case 19: // PPM to SERVO
                            str = 'PPM to SERVO';
                            break;
                        case 20: // Dualcopter
                            str = 'Dualcopter';
                            break;
                        case 21: //
                            str = 'Singlecopter';
                            break;
                    }
                    
                    $('span.model').html('Model: ' + str);
                    
                    // UI Hooks
                    $('a.calibrateAccel').click(function() {
                        var self = $(this);
                        
                        if (!self.hasClass('calibrating')) {
                            self.addClass('calibrating');
                            
                            // During this period MCU won't be able to process any serial commands because its locked in a for/while loop
                            // until this operation finishes, sending more commands through data_poll() will result in serial buffer overflow
                            GUI.interval_pause('initial_setup_data_pull');
                            send_message(MSP_codes.MSP_ACC_CALIBRATION, MSP_codes.MSP_ACC_CALIBRATION);
                            
                            GUI.timeout_add('button_reset', function() {
                                GUI.interval_resume('initial_setup_data_pull');
                                self.removeClass('calibrating');
                            }, 2000);
                        }
                    });
                    
                    $('a.calibrateMag').click(function() {
                        var self = $(this);
                        
                        if (!self.hasClass('calibrating')) {
                            self.addClass('calibrating');
                        
                            send_message(MSP_codes.MSP_MAG_CALIBRATION, MSP_codes.MSP_MAG_CALIBRATION);
                            
                            GUI.timeout_add('button_reset', function() {
                                self.removeClass('calibrating');
                            }, 30000);
                        }
                    });

                    $('a.resetSettings').click(function() {
                        send_message(MSP_codes.MSP_RESET_CONF, MSP_codes.MSP_RESET_CONF, false, function() {
                            tab_initialize_initial_setup();
                        });
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
                        
                        MISC.vbatmincellvoltage = parseFloat($('input[name="mincellvoltage"]').val()) * 10;
                        MISC.vbatmaxcellvoltage = parseFloat($('input[name="maxcellvoltage"]').val()) * 10;
                        MISC.vbatscale = parseInt($('input[name="voltagescale"]').val());
                        
                        MISC.minthrottle = parseInt($('input[name="minthrottle"]').val());
                        MISC.maxthrottle = parseInt($('input[name="maxthrottle"]').val());
                        MISC.failsafe_throttle = parseInt($('input[name="failsafe_throttle"]').val());
                        MISC.mincommand = parseInt($('input[name="mincommand"]').val());
                        
                        MISC.mag_declination = parseFloat($('input[name="mag_declination"]').val()) * 10;
                        
                        // we also have to fill the unsupported bytes
                        var buffer_out = new Array();
                        buffer_out[0] = 0; // powerfailmeter
                        buffer_out[1] = 0;
                        buffer_out[2] = lowByte(MISC.minthrottle);
                        buffer_out[3] = highByte(MISC.minthrottle);
                        buffer_out[4] = lowByte(MISC.maxthrottle);
                        buffer_out[5] = highByte(MISC.maxthrottle);
                        buffer_out[6] = lowByte(MISC.mincommand);
                        buffer_out[7] = highByte(MISC.mincommand);
                        buffer_out[8] = lowByte(MISC.failsafe_throttle);
                        buffer_out[9] = highByte(MISC.failsafe_throttle);
                        buffer_out[10] = 0;
                        buffer_out[11] = 0;
                        buffer_out[12] = 0;
                        buffer_out[13] = 0;
                        buffer_out[14] = 0;
                        buffer_out[15] = 0;
                        buffer_out[16] = lowByte(MISC.mag_declination);
                        buffer_out[17] = highByte(MISC.mag_declination);
                        buffer_out[18] = MISC.vbatscale;
                        buffer_out[19] = MISC.vbatmincellvoltage;
                        buffer_out[20] = MISC.vbatmaxcellvoltage;
                        buffer_out[21] = 0; // vbatlevel_crit (unused)
                        
                        // Send over new misc
                        send_message(MSP_codes.MSP_SET_MISC, buffer_out);
                        
                        // Save changes to EEPROM
                        send_message(MSP_codes.MSP_EEPROM_WRITE, MSP_codes.MSP_EEPROM_WRITE, false, function() {
                            var element = $('a.update');
                            element.addClass('success');
                            
                            GUI.timeout_add('success_highlight', function() {
                                element.removeClass('success');
                            }, 2000);
                        });
                    });    

                    // reset yaw button hook
                    $('div#interactive_block > a.reset').click(function() {
                        yaw_fix = SENSOR_DATA.kinematicsZ * - 1.0;
                        console.log("YAW reset to 0");
                    });  

                    $('#content .backup').click(configuration_backup);
                    
                    $('#content .restore').click(configuration_restore);
                    
                    GUI.interval_add('initial_setup_data_pull', function() {                    
                        // Update voltage indicator
                        $('.bat-voltage').html(ANALOG.voltage + ' V');
                        
                        // Request new data, if transmission fails it doesn't matter as new transmission will be requested after 50ms
                        send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS, false, function() { // cycle time, active sensors, etc...
                            send_message(MSP_codes.MSP_ANALOG, MSP_codes.MSP_ANALOG, false, function() { // battery voltage
                                send_message(MSP_codes.MSP_ATTITUDE, MSP_codes.MSP_ATTITUDE, false, function() { // kinematics
                                    // Update cube
                                    var cube = $('div#cube');
                                    
                                    cube.css('-webkit-transform', 'rotateY(' + ((SENSOR_DATA.kinematicsZ * -1.0) - yaw_fix) + 'deg)');
                                    $('#cubePITCH', cube).css('-webkit-transform', 'rotateX(' + SENSOR_DATA.kinematicsY + 'deg)');
                                    $('#cubeROLL', cube).css('-webkit-transform', 'rotateZ(' + SENSOR_DATA.kinematicsX + 'deg)'); 

                                    // Update Compass
                                    $('div#compass .pointer').css('-webkit-transform', 'rotate(' + (SENSOR_DATA.kinematicsZ) + 'deg)'); 
                                    $('div#compass .value').html(SENSOR_DATA.kinematicsZ + '&deg;');
                                });
                            });
                        }); 
                    }, 50, true);
                });
            });
        });
    });
}