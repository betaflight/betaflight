'use strict';

TABS.landing = {};
TABS.landing.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'landing') {
        GUI.active_tab = 'landing';
        googleAnalytics.sendAppView('Landing');
    }

    $('#content').load("./tabs/landing.html", function () {
        //check_usb_permissions(); // temporary enabled in dev branch, should be commented out untill DFU support goes live

        // translate to user-selected language
        localize();

        // load changelog content
        $('div.changelog.configurator .wrapper').load('./changelog.html');

        // UI Hooks
        $('a.firmware_flasher').click(function () {
            TABS.firmware_flasher.initialize();
        });

        $('div.welcome a, div.sponsors a').click(function () {
            googleAnalytics.sendEvent('ExternalUrls', 'Click', $(this).prop('href'));
        });

        if (callback) callback();
    });
};

TABS.landing.cleanup = function (callback) {
    if (callback) callback();
};