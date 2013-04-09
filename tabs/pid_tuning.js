function tab_initialize_pid_tuning() {
    // Fill in the data from PIDs array
    var needle = 0;

    var i = 0;
    $('.pid_tuning .ROLL input').each(function() {
        $(this).val(PIDs[needle][i++].toFixed(3));       
    });
    needle++;
    
    i = 0;
    $('.pid_tuning .PITCH input').each(function() {
        $(this).val(PIDs[needle][i++].toFixed(3));       
    });
    needle++;
    
    i = 0;
    $('.pid_tuning .YAW input').each(function() {
        $(this).val(PIDs[needle][i++].toFixed(3));       
    });
    needle++;

    i = 0;
    $('.pid_tuning .ALT input').each(function() {
        $(this).val(PIDs[needle][i++].toFixed(3));       
    });
    needle++;

    i = 0;
    $('.pid_tuning .Pos input').each(function() {
        $(this).val(PIDs[needle][i++].toFixed(3));       
    });
    needle++;
    
    i = 0;
    $('.pid_tuning .PosR input').each(function() {
        $(this).val(PIDs[needle][i++].toFixed(3));       
    });
    needle++;  

    i = 0;
    $('.pid_tuning .NavR input').each(function() {
        $(this).val(PIDs[needle][i++].toFixed(2));       
    });
    needle++; 

    i = 0;
    $('.pid_tuning .LEVEL input').each(function() {
        $(this).val(PIDs[needle][i++].toFixed(2));       
    });
    needle++; 

    i = 0;
    $('.pid_tuning .MAG input').each(function() {
        $(this).val(PIDs[needle][i++].toFixed(2));       
    });
    needle++;  

    // UI Hooks
    
    $('.pid_tuning input').change(function() {
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
                
                /*
                for (var ii = 0; ii < PIDs[i].length; ii++) {
                    PID_buffer_out[PID_buffer_needle] = PIDs[i][ii];
                    PID_buffer_needle++;
                }
                */
            }
            
            //console.log(PID_buffer_out);
            // Send over the changes
            send_message(MSP_codes.MSP_SET_PID, PID_buffer_out, PID_buffer_out.length);
            
            // remove the active status
            $(this).removeClass('active');
        }
    });
    
}