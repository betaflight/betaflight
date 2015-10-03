'use strict';

TABS.landing = {};
TABS.landing.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'landing') {
        GUI.active_tab = 'landing';
        googleAnalytics.sendAppView('Landing');
    }

    $('#content').load("./tabs/landing.html", function () {
        check_usb_permissions(); // check for permissions require for USB DFU support

        // translate to user-selected language
        localize();

        // load changelog content
        $('div.changelog.configurator .wrapper').load('./changelog.html');

        $('div.welcome a, div.sponsors a').click(function () {
            googleAnalytics.sendEvent('ExternalUrls', 'Click', $(this).prop('href'));
        });

        if (callback) callback();
    });
};

TABS.landing.cleanup = function (callback) {
    if (callback) callback();
};