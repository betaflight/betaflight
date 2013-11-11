function tab_initialize_motor_outputs() {
    ga_tracker.sendAppView('Motor Outputs Page');
    GUI.active_tab = 'motor_outputs';
    
    // UI hooks
    $('div.sliders input').change(function() {
        var index = $(this).index();
        
        $('div.values li').eq(index).html($(this).val());
        
        // send data to mcu
        var buffer_out = [];
        
        for (var i = 0; i < 8; i++) {
            var val = parseInt($('div.sliders input').eq(i).val());
            
            buffer_out.push(lowByte(val));
            buffer_out.push(highByte(val));
        }
        
        send_message(MSP_codes.MSP_SET_MOTOR, buffer_out);
    });
    
    // enable Motor data pulling
    timers.push(setInterval(motorPoll, 50));
}

function motorPoll() {
    // Request New Motor data
    send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
    send_message(MSP_codes.MSP_MOTOR, MSP_codes.MSP_MOTOR);
    
    // Update UI
    for (var i = 0; i < MOTOR_DATA.length; i++) {
        MOTOR_DATA[i] -= 1000; 
        var margin_top = 220.0 - (MOTOR_DATA[i] * 0.22);
        var height = (MOTOR_DATA[i] * 0.22);
        var color = parseInt(MOTOR_DATA[i] * 0.256);
        $('.motor-' + i + ' .indicator').css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
    } 

    // Request New Servo data
    send_message(MSP_codes.MSP_SERVO, MSP_codes.MSP_SERVO);
    
    // Update UI
    for (var i = 0; i < SERVO_DATA.length; i++) {
        SERVO_DATA[i] -= 1000; 
        var margin_top = 220.0 - (SERVO_DATA[i] * 0.22);
        var height = (SERVO_DATA[i] * 0.22);
        var color = parseInt(SERVO_DATA[i] * 0.256);
        $('.servo-' + i + ' .indicator').css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
    } 
}