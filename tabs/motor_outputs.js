function tab_initialize_motor_outputs() {
    // enable Motor data pulling
    motor_poll = setInterval(motorPoll, 50);
    timers.push(motor_poll);
}

function motorPoll() {
    // Request New Motor data
    send_message(MSP_codes.MSP_MOTOR, MSP_codes.MSP_MOTOR);
    
    // Update UI
    for (var i = 0; i < MOTOR_DATA.length; i++) {
        MOTOR_DATA[i] -= 1000; 
        var margin_top = 330.0 - (MOTOR_DATA[i] * 0.33);
        var height = (MOTOR_DATA[i] * 0.33);
        var color = parseInt(MOTOR_DATA[i] * 0.256);
        $('.motor-' + i + ' .indicator').css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
    } 

    // Request New Servo data
    send_message(MSP_codes.MSP_SERVO, MSP_codes.MSP_SERVO);
    
    // Update UI
    for (var i = 0; i < SERVO_DATA.length; i++) {
        SERVO_DATA[i] -= 1000; 
        var margin_top = 330.0 - (SERVO_DATA[i] * 0.33);
        var height = (SERVO_DATA[i] * 0.33);
        var color = parseInt(SERVO_DATA[i] * 0.256);
        $('.servo-' + i + ' .indicator').css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
    } 
}