function tab_initialize_pid_tuning() {
    ga_tracker.sendAppView('PID Tuning');
    GUI.active_tab = 'pid_tuning';
    
    send_message(MSP_codes.MSP_PID, MSP_codes.MSP_PID, false, function() {
        send_message(MSP_codes.MSP_RC_TUNING, MSP_codes.MSP_RC_TUNING, false, function() {
            // Fill in the data from PIDs array
            var needle = 0;

            var i = 0;
            $('.pid_tuning .ROLL input').each(function() {
                switch (i) {
                    case 0:
                        $(this).val(PIDs[needle][i++].toFixed(1));  
                        break;
                    case 1:
                        $(this).val(PIDs[needle][i++].toFixed(3));  
                        break;
                    case 2:
                        $(this).val(PIDs[needle][i++].toFixed(0));  
                        break;
                }     
            });
            needle++;
            
            i = 0;
            $('.pid_tuning .PITCH input').each(function() {
                switch (i) {
                    case 0:
                        $(this).val(PIDs[needle][i++].toFixed(1));  
                        break;
                    case 1:
                        $(this).val(PIDs[needle][i++].toFixed(3));  
                        break;
                    case 2:
                        $(this).val(PIDs[needle][i++].toFixed(0));  
                        break;
                }     
            });
            needle++;
            
            i = 0;
            $('.pid_tuning .YAW input').each(function() {
                switch (i) {
                    case 0:
                        $(this).val(PIDs[needle][i++].toFixed(1));  
                        break;
                    case 1:
                        $(this).val(PIDs[needle][i++].toFixed(3));  
                        break;
                    case 2:
                        $(this).val(PIDs[needle][i++].toFixed(0));  
                        break;
                }      
            });
            needle++;

            i = 0;
            $('.pid_tuning .ALT input').each(function() {
                switch (i) {
                    case 0:
                        $(this).val(PIDs[needle][i++].toFixed(1));  
                        break;
                    case 1:
                        $(this).val(PIDs[needle][i++].toFixed(3));  
                        break;
                    case 2:
                        $(this).val(PIDs[needle][i++].toFixed(0));  
                        break;
                }     
            });
            needle++;

            i = 0;
            $('.pid_tuning .Pos input').each(function() {
                $(this).val(PIDs[needle][i++].toFixed(2));       
            });
            needle++;
            
            i = 0;
            $('.pid_tuning .PosR input').each(function() {
                switch (i) {
                    case 0:
                        $(this).val(PIDs[needle][i++].toFixed(1));  
                        break;
                    case 1:
                        $(this).val(PIDs[needle][i++].toFixed(2));  
                        break;
                    case 2:
                        $(this).val(PIDs[needle][i++].toFixed(3));  
                        break;
                }       
            });
            needle++;  

            i = 0;
            $('.pid_tuning .NavR input').each(function() {
                switch (i) {
                    case 0:
                        $(this).val(PIDs[needle][i++].toFixed(1));  
                        break;
                    case 1:
                        $(this).val(PIDs[needle][i++].toFixed(2));  
                        break;
                    case 2:
                        $(this).val(PIDs[needle][i++].toFixed(3));  
                        break;
                }      
            });
            needle++; 

            i = 0;
            $('.pid_tuning .LEVEL input').each(function() {
                switch (i) {
                    case 0:
                        $(this).val(PIDs[needle][i++].toFixed(1));  
                        break;
                    case 1:
                        $(this).val(PIDs[needle][i++].toFixed(2));  
                        break;
                    case 2:
                        $(this).val(PIDs[needle][i++].toFixed(0));  
                        break;
                }      
            });
            needle++; 

            i = 0;
            $('.pid_tuning .MAG input').each(function() {
                $(this).val(PIDs[needle][i++].toFixed(1));       
            });
            needle++;  

            // Fill in data from RC_tuning object
            $('.rate-tpa input[name="roll-pitch"]').val(RC_tuning.roll_pitch_rate.toFixed(2));
            $('.rate-tpa input[name="yaw"]').val(RC_tuning.yaw_rate.toFixed(2));
            $('.rate-tpa input[name="tpa"]').val(RC_tuning.dynamic_THR_PID.toFixed(2));
            
            // UI Hooks
            $('.pid_tuning input, .rate-tpa input').change(function() {
                // if any of the fields changed, unlock update button
                $('a.update').addClass('active');
            });
            
            $('a.update').click(function() {
                if ($(this).hasClass('active')) {
                    // Catch all the changes and stuff the inside PIDs array
                    var needle_main = 0;
                    var needle_secondary = 0;
                    
                    $('.pid_tuning input').each(function() {
                        PIDs[needle_main][needle_secondary] = parseFloat($(this).val());
                        needle_secondary++;
                        
                        // exceptions (required for the "shorter" PID arrays, 2 fields, 1 field, etc)
                        if (needle_main == 4) {
                            if (needle_secondary >= 2) {
                                needle_main++;
                                needle_secondary = 0;
                            }
                        } else if (needle_main == 8) {
                            if (needle_secondary >= 1) {
                                needle_main++;
                                needle_secondary = 0;
                            }
                        } else {
                            if (needle_secondary >= 3) {
                                needle_main++;
                                needle_secondary = 0;
                            }
                        }
                    });
                    
                    var PID_buffer_out = new Array();
                    var PID_buffer_needle = 0;
                    for (var i = 0; i < PIDs.length; i++) {
                        switch (i) {
                            case 0: 
                            case 1: 
                            case 2: 
                            case 3: 
                            case 7: 
                            case 8:
                            case 9:
                                PID_buffer_out[PID_buffer_needle]     = parseInt(PIDs[i][0] * 10);
                                PID_buffer_out[PID_buffer_needle + 1] = parseInt(PIDs[i][1] * 1000);
                                PID_buffer_out[PID_buffer_needle + 2] = parseInt(PIDs[i][2]);
                                break;
                            case 4:
                                PID_buffer_out[PID_buffer_needle]     = parseInt(PIDs[i][0] * 100);
                                PID_buffer_out[PID_buffer_needle + 1] = parseInt(PIDs[i][1] * 100);
                                PID_buffer_out[PID_buffer_needle + 2] = parseInt(PIDs[i][2]);
                                break;
                            case 5: 
                            case 6:
                                PID_buffer_out[PID_buffer_needle]     = parseInt(PIDs[i][0] * 10);
                                PID_buffer_out[PID_buffer_needle + 1] = parseInt(PIDs[i][1] * 100);
                                PID_buffer_out[PID_buffer_needle + 2] = parseInt(PIDs[i][2] * 1000);
                                break;                     
                        }
                        PID_buffer_needle += 3;
                    }
                    
                    // Send over the PID changes
                    send_message(MSP_codes.MSP_SET_PID, PID_buffer_out);
                    
                    // catch RC_tuning changes
                    RC_tuning.roll_pitch_rate = parseFloat($('.rate-tpa input[name="roll-pitch"]').val());
                    RC_tuning.yaw_rate = parseFloat($('.rate-tpa input[name="yaw"]').val());
                    RC_tuning.dynamic_THR_PID = parseFloat($('.rate-tpa input[name="tpa"]').val());            
                    
                    var RC_tuning_buffer_out = new Array();
                    RC_tuning_buffer_out[0] = parseInt(RC_tuning.RC_RATE * 100);
                    RC_tuning_buffer_out[1] = parseInt(RC_tuning.RC_EXPO * 100);
                    RC_tuning_buffer_out[2] = parseInt(RC_tuning.roll_pitch_rate * 100);
                    RC_tuning_buffer_out[3] = parseInt(RC_tuning.yaw_rate * 100);
                    RC_tuning_buffer_out[4] = parseInt(RC_tuning.dynamic_THR_PID * 100);
                    RC_tuning_buffer_out[5] = parseInt(RC_tuning.throttle_MID * 100);
                    RC_tuning_buffer_out[6] = parseInt(RC_tuning.throttle_EXPO * 100);
                    
                    // Send over the RC_tuning changes
                    send_message(MSP_codes.MSP_SET_RC_TUNING, RC_tuning_buffer_out);
                    
                    // Save changes to EEPROM
                    send_message(MSP_codes.MSP_EEPROM_WRITE, MSP_codes.MSP_EEPROM_WRITE);
                    
                    // remove the active status
                    $(this).removeClass('active');
                }
            });

            // enable data pulling
            GUI.interval_add('pid_data_poll', function() {
                send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
            }, 50);  
        });
    });   
}