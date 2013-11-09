function tab_initialize_about() {
    ga_tracker.sendAppView('About Page');
    GUI.active_tab = 'about';
    
    // enable data pulling
    timers.push(setInterval(about_data_poll, 50));   
}

function about_data_poll() {
    send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
}