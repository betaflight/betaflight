tabs.default = {};
tabs.default.initialize = function(callback) {
    GUI.active_tab_ref = this;
    GUI.active_tab = 'default';

    $('#content').load("./tabs/default.html", function() {
        //check_usb_permissions(); // temporary enabled in dev branch, should be commented out untill DFU support goes live

        // translate to user-selected language
        localize();

        // load changelog content
        $('div.changelog.configurator .wrapper').load('./changelog.html');

        // UI Hooks
        $('a.firmware_flasher').click(function() {
            tabs.firmware_flasher.initialize();
        });

        $('div.welcome a').click(function() {
            googleAnalytics.sendEvent('ExternalUrls', 'Click', $(this).prop('href'));
        });

        if (callback) callback();
    });
};

tabs.default.cleanup = function(callback) {
    if (callback) callback();
};