'use strict';

TABS.configuration = {};

TABS.configuration.initialize = function (callback) {
    var self = this;
    GUI.active_tab_ref = this;
    GUI.active_tab = 'configuration';
    googleAnalytics.sendAppView('Configuration');


    function check_compatibility() {
        if (bit_check(CONFIG.capability, 30)) {
            // new stuff supported, continue fetching configuration data
            load_config();
        } else {
            // old version, deny access
            $('#content').text('We are sorry, but advanced configuration is only available for boards with latest firmware, please update');

            if (callback) callback();
        }
    }

    function load_config() {
        MSP.send_message(MSP_codes.MSP_CONFIG, false, false, load_rc_map);
    }

    function load_rc_map() {
        MSP.send_message(MSP_codes.MSP_RCMAP, false, false, load_misc);
    }

    function load_misc() {
        MSP.send_message(MSP_codes.MSP_MISC, false, false, load_acc_trim);
    }

    function load_acc_trim() {
        MSP.send_message(MSP_codes.MSP_ACC_TRIM, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/configuration.html", process_html);
    }

    MSP.send_message(MSP_codes.MSP_IDENT, false, false, check_compatibility);

    function process_html() {
        // translate to user-selected language
        localize();

        // index references
        var featureNames = [
            'PPM',
            'VBAT',
            'INFLIGHT_ACC_CAL',
            'SERIALRX',
            'MOTOR_STOP',
            'SERVO_TILT',
            'SOFTSERIAL',
            'LED_RING',
            'GPS',
            'FAILSAFE',
            'SONAR',
            'TELEMETRY',
            'POWERMETER',
            'VARIO',
            '3D'
        ];

        var RCMAPlLetters = ['A', 'E', 'R', 'T', '1', '2', '3', '4'];

        console.log('all ready');

        if (callback) callback();
    }

};

TABS.configuration.cleanup = function (callback) {
    if (callback) callback();
};