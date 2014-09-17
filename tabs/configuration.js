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
        var RCMAPlLetters = ['A', 'E', 'R', 'T', '1', '2', '3', '4'];

        var featureNames = [
            'PPM - Disable PWM input and enable PPM input',
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

        // generate features
        var features_e = $('.features');
        for (var i = 0; i < featureNames.length; i++) {
            var element = $('<dt><input id="feature-' + i + '" type="checkbox" /></dt><dd><label for="feature-' + i + '">' + featureNames[i] + '</label></dd>');
            element.find('input').attr('checked', bit_check(BF_CONFIG.features, i));

            features_e.append(element);
        }


        // UI hooks
        $('input', features_e).change(function () {
            var element = $(this),
                index = $('input', features_e).index(element),
                state = element.is(':checked');

            if (state) {
                BF_CONFIG.features = bit_set(BF_CONFIG.features, index);
            } else {
                BF_CONFIG.features = bit_clear(BF_CONFIG.features, index);
            }
        });

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

                MSP.send_message(MSP_codes.MSP_IDENT, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('deviceReady'));
                    TABS.configuration.initialize();
                });
            }

            MSP.send_message(MSP_codes.MSP_SET_CONFIG, MSP.crunch(MSP_codes.MSP_SET_CONFIG), false, save_to_eeprom);
        });

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull () {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    }
};

TABS.configuration.cleanup = function (callback) {
    if (callback) callback();
};