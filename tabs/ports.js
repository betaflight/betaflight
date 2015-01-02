'use strict';

TABS.ports = {};

TABS.ports.initialize = function (callback, scrollPosition) {
    var self = this;

    if (GUI.active_tab != 'ports') {
        GUI.active_tab = 'ports';
        googleAnalytics.sendAppView('Ports');
    }

    load_configuration_from_fc();
    
    function load_configuration_from_fc() {
        MSP.send_message(MSP_codes.MSP_IDENT, false, false, on_ident_loaded_handler);

        function on_ident_loaded_handler() {
            MSP.send_message(MSP_codes.MSP_BF_CONFIG, false, false, on_configuration_loaded_handler);
        }

        function on_configuration_loaded_handler() {
            $('#content').load("./tabs/ports.html", on_tab_loaded_handler);
        }
    }

    function on_tab_loaded_handler() {

        localize();

        $('a.save').click(on_save_handler);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    }

    function on_save_handler() {
        
        MSP.send_message(MSP_codes.MSP_SET_CONFIG, MSP.crunch(MSP_codes.MSP_SET_CONFIG), false, save_to_eeprom);

        function save_to_eeprom() {
            MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, on_saved_handler);
        }

        function on_saved_handler() {
            GUI.log(chrome.i18n.getMessage('configurationEepromSaved'));

            GUI.tab_switch_cleanup(function() {
                MSP.send_message(MSP_codes.MSP_SET_REBOOT, false, false, on_reboot_success_handler);
            });
        }

        function on_reboot_success_handler() {
            GUI.log(chrome.i18n.getMessage('deviceRebooting'));

            var rebootTimeoutDelay = 1500;  // seems to be just the right amount of delay to prevent data request timeouts
            
            GUI.timeout_add('waiting_for_bootup', function waiting_for_bootup() {
                MSP.send_message(MSP_codes.MSP_IDENT, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('deviceReady'));
                    TABS.ports.initialize(false, $('#content').scrollTop());
                });
            }, rebootTimeoutDelay); 
        }
    }
};

TABS.ports.cleanup = function (callback) {
    if (callback) callback();
};