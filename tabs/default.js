function tab_initialize_default() {
    GUI.active_tab = 'default';

    $('#content').load("./tabs/default.html", function() {
        //check_usb_permissions(); // temporary enabled in dev branch, should be commented out untill DFU support goes live

        // translate to user-selected language
        localize();

        // load changelog content
        $('div.changelog.configurator .wrapper').load('./changelog.html');

        // UI Hooks
        $('a.firmware_flasher').click(tab_initialize_firmware_flasher);

        $('div.welcome a').click(function() {
            ga_tracker.sendEvent('ExternalUrls', 'Click', $(this).prop('href'));
        });
    });
}