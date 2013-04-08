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
            
            // Send over the changes
            
            
            // remove the active status
            $(this).removeClass('active');
        }
    });
    
}