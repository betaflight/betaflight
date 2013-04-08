function tab_initialize_pid_tuning() {
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
    
}