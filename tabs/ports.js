'use strict';

TABS.ports = {};

TABS.ports.initialize = function (callback, scrollPosition) {
    var self = this;

    if (GUI.active_tab != 'ports') {
        GUI.active_tab = 'ports';
        googleAnalytics.sendAppView('Ports');
    }

    function load_config() {
        MSP.send_message(MSP_codes.MSP_CONFIG, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/ports.html", process_html);
    }

    MSP.send_message(MSP_codes.MSP_IDENT, false, false, load_config);

    function process_html() {

        localize();

        $('a.save').click(function () {

            function save_to_eeprom() {
                MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, reboot);
            }

            function reboot() {
                GUI.log(chrome.i18n.getMessage('configurationEepromSaved'));

                GUI.tab_switch_cleanup(function() {
                    MSP.send_message(MSP_codes.MSP_SET_REBOOT, false, false, reinitialize);
                });
            }

            function reinitialize() {
                GUI.log(chrome.i18n.getMessage('deviceRebooting'));

                GUI.timeout_add('waiting_for_bootup', function waiting_for_bootup() {
                    MSP.send_message(MSP_codes.MSP_IDENT, false, false, function () {
                        GUI.log(chrome.i18n.getMessage('deviceReady'));
                        TABS.ports.initialize(false, $('#content').scrollTop());
                    });
                },1500); // 1500 ms seems to be just the right amount of delay to prevent data request timeouts
            }

            MSP.send_message(MSP_codes.MSP_SET_CONFIG, MSP.crunch(MSP_codes.MSP_SET_CONFIG), false, save_to_eeprom);
        });

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    }
};

TABS.ports.cleanup = function (callback) {
    if (callback) callback();
};