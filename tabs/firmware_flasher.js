function tab_initialize_firmware_flasher() {
    ga_tracker.sendAppView('Firmware Flasher');
    GUI.active_tab = 'firmware_flasher';
    
    $('#content').load("./tabs/firmware_flasher.html", function() {
        // empty for now
    });
}