function tab_initialize_receiver() {
    receiver_poll = setInterval(receiverPoll, 100);
    timers.push(receiver_poll);
}

function receiverPoll() {
    send_message(MSP_codes.MSP_RC, MSP_codes.MSP_RC);
    
    // Update UI with latest data
    $('.tab-receiver meter:eq(0)').val(RC.throttle);
    $('.tab-receiver .value:eq(0)').html(RC.throttle);
    
    $('.tab-receiver meter:eq(1)').val(RC.pitch);
    $('.tab-receiver .value:eq(1)').html(RC.pitch);
    
    $('.tab-receiver meter:eq(2)').val(RC.roll);
    $('.tab-receiver .value:eq(2)').html(RC.roll);
    
    $('.tab-receiver meter:eq(3)').val(RC.yaw);
    $('.tab-receiver .value:eq(3)').html(RC.yaw);
    
    
    $('.tab-receiver meter:eq(4)').val(RC.AUX1);
    $('.tab-receiver .value:eq(4)').html(RC.AUX1);
    
    $('.tab-receiver meter:eq(5)').val(RC.AUX2);
    $('.tab-receiver .value:eq(5)').html(RC.AUX2);
    
    $('.tab-receiver meter:eq(6)').val(RC.AUX3);
    $('.tab-receiver .value:eq(6)').html(RC.AUX3);
    
    $('.tab-receiver meter:eq(7)').val(RC.AUX4);
    $('.tab-receiver .value:eq(7)').html(RC.AUX4);
}