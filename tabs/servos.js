function tab_initialize_servos() {
    ga_tracker.sendAppView('Servos');
    
    // request current Servo Config
    send_message(MSP_codes.MSP_SERVO_CONF, MSP_codes.MSP_SERVO_CONF);
}