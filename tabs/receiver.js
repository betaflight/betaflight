function tab_initialize_receiver() {
    receiver_poll = setInterval(receiverPoll, 100);
    timers.push(receiver_poll);
}

function receiverPoll() {
    send_message(MSP_codes.MSP_RC, MSP_codes.MSP_RC);
}

// TODO: This function / interval needs to be killed when leaving Receiver tab