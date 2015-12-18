'use strict';

TABS.failsafe = {};

TABS.failsafe.initialize = function (callback, scrollPosition) {
    var self = this;

    if (GUI.active_tab != 'failsafe') {
        GUI.active_tab = 'failsafe';
        googleAnalytics.sendAppView('Failsafe');
    }

    function load_rx_config() {
        MSP.send_message(MSP_codes.MSP_RX_CONFIG, false, false, load_failssafe_config);
    }

    function load_failssafe_config() {
        MSP.send_message(MSP_codes.MSP_FAILSAFE_CONFIG, false, false, load_rxfail_config);
    }
    
    function load_rxfail_config() {
        MSP.send_message(MSP_codes.MSP_RXFAIL_CONFIG, false, false, get_box_names);
    }

    function get_box_names() {
        MSP.send_message(MSP_codes.MSP_BOXNAMES, false, false, get_mode_ranges);
    }

    function get_mode_ranges() {
        MSP.send_message(MSP_codes.MSP_MODE_RANGES, false, false, get_box_ids);
    }

    function get_box_ids() {
        MSP.send_message(MSP_codes.MSP_BOXIDS, false, false, get_rc_data);
    }

    function get_rc_data() {
        MSP.send_message(MSP_codes.MSP_RC, false, false, load_config);
    }

    // BEGIN Support for pre API version 1.15.0
    function load_config() {
        MSP.send_message(MSP_codes.MSP_BF_CONFIG, false, false, load_misc);
    }

    function load_misc() {
        MSP.send_message(MSP_codes.MSP_MISC, false, false, load_html);
    }
    // END (Support for pre API version 1.15.0

    function load_html() {
        $('#content').load("./tabs/failsafe.html", process_html);
    }

    var apiVersionGte1_15_0 = semver.gte(CONFIG.apiVersion, "1.15.0");

    // Uncomment next line for testing older functionality on newer API version
    //apiVersionGte1_15_0 = false;

    if(apiVersionGte1_15_0) {
        MSP.send_message(MSP_codes.MSP_IDENT, false, false, load_rx_config);
    } else {
        MSP.send_message(MSP_codes.MSP_IDENT, false, false, load_config);
    }

    function process_html() {
        var failsafeFeature;

        // translate to user-selected language
        localize();

        // Conditionally hide the old or the new control pane's
        if(apiVersionGte1_15_0) {
            var oldPane = $('div.oldpane');
            oldPane.prop("disabled", true);
            oldPane.hide();
        } else {
            var newPane = $('div.newpane');
            newPane.prop("disabled", true);
            newPane.hide();
        }

        if(apiVersionGte1_15_0) {
            // generate labels for assigned aux modes
            var auxAssignment = [],
                i,
                element;

            for (var channelIndex = 0; channelIndex < RC.active_channels - 4; channelIndex++) {
                auxAssignment.push("");
            }

            for (var modeIndex = 0; modeIndex < AUX_CONFIG.length; modeIndex++) {

                var modeId = AUX_CONFIG_IDS[modeIndex];

                // scan mode ranges to find assignments
                for (var modeRangeIndex = 0; modeRangeIndex < MODE_RANGES.length; modeRangeIndex++) {
                    var modeRange = MODE_RANGES[modeRangeIndex];

                    if (modeRange.id != modeId) {
                        continue;
                    }

                    var range = modeRange.range;
                    if (!(range.start < range.end)) {
                        continue; // invalid!
                    }

                    auxAssignment[modeRange.auxChannelIndex] += "<span class=\"modename\">" + AUX_CONFIG[modeIndex] + "</span>";
                }
            }

            // generate full channel list
            var channelNames = [
                    chrome.i18n.getMessage('controlAxisRoll'),
                    chrome.i18n.getMessage('controlAxisPitch'),
                    chrome.i18n.getMessage('controlAxisYaw'),
                    chrome.i18n.getMessage('controlAxisThrottle')
                ],
                fullChannels_e = $('div.activechannellist'),
                aux_index = 1,
                aux_assignment_index = 0;

            for (i = 0; i < RXFAIL_CONFIG.length; i++) {
                if (i < channelNames.length) {
                    fullChannels_e.append('\
                        <div class="number">\
                            <div class="channelprimary">\
                                <span>' + channelNames[i] + '</span>\
                            </div>\
                            <div class="cf_tip channelsetting" title="' + chrome.i18n.getMessage("failsafeChannelFallbackSettingsAuto") + '">\
                                <select class="aux_set" id="' + i + '">\
                                    <option value="0">Auto</option>\
                                    <option value="1">Hold</option>\
                                </select>\
                            </div>\
                        </div>\
                    ');
                } else {
                    fullChannels_e.append('\
                        <div class="number">\
                            <div class="channelauxiliary">\
                                <span class="channelname">' + chrome.i18n.getMessage("controlAxisAux" + (aux_index++)) + '</span>\
                                ' + auxAssignment[aux_assignment_index++] + '\
                            </div>\
                            <div class="cf_tip channelsetting" title="' + chrome.i18n.getMessage("failsafeChannelFallbackSettingsHold") + '">\
                                <select class="aux_set" id="' + i + '">\
                                    <option value="1">Hold</option>\
                                    <option value="2">Set</option>\
                                </select>\
                            </div>\
                            <div class="auxiliary"><input type="number" name="aux_value" min="750" max="2250" id="' + i + '"/></div>\
                        </div>\
                    ');
                }
            }

            var channel_mode_array = [];
            $('.number', fullChannels_e).each(function () {
                channel_mode_array.push($('select.aux_set' , this));
            });

            var channel_value_array = [];
            $('.number', fullChannels_e).each(function () {
                channel_value_array.push($('input[name="aux_value"]' , this));
            });

            var channelMode = $('select.aux_set');
            var channelValue = $('input[name="aux_value"]');

            // UI hooks
            channelMode.change(function () {
                var currentMode = parseInt($(this).val());
                var i = parseInt($(this).prop("id"));
                RXFAIL_CONFIG[i].mode = currentMode;
                if (currentMode == 2) {
                    channel_value_array[i].prop("disabled", false);
                    channel_value_array[i].show();
                } else {
                    channel_value_array[i].prop("disabled", true);
                    channel_value_array[i].hide();
                }
            });

            // UI hooks
            channelValue.change(function () {
                var i = parseInt($(this).prop("id"));
                RXFAIL_CONFIG[i].value = parseInt($(this).val());
            });

            // for some odd reason chrome 38+ changes scroll according to the touched select element
            // i am guessing this is a bug, since this wasn't happening on 37
            // code below is a temporary fix, which we will be able to remove in the future (hopefully)
            $('#content').scrollTop((scrollPosition) ? scrollPosition : 0);

            // fill stage 1 Valid Pulse Range Settings
            $('input[name="rx_min_usec"]').val(RX_CONFIG.rx_min_usec);
            $('input[name="rx_max_usec"]').val(RX_CONFIG.rx_max_usec);

            // fill fallback settings (mode and value) for all channels
            for (i = 0; i < RXFAIL_CONFIG.length; i++) {
                channel_value_array[i].val(RXFAIL_CONFIG[i].value);
                channel_mode_array[i].val(RXFAIL_CONFIG[i].mode);
                channel_mode_array[i].change();
            }

            // fill stage 2 fields
            failsafeFeature = $('input[name="failsafe_feature_new"]');
            failsafeFeature.change(function () {
                if ($(this).is(':checked')) {
                    $('div.stage2').show();
                } else {
                    $('div.stage2').hide();
                }
            });

            failsafeFeature.prop('checked', bit_check(BF_CONFIG.features, 8));
            failsafeFeature.change();

            $('input[name="failsafe_throttle"]').val(FAILSAFE_CONFIG.failsafe_throttle);
            $('input[name="failsafe_off_delay"]').val(FAILSAFE_CONFIG.failsafe_off_delay);
            $('input[name="failsafe_throttle_low_delay"]').val(FAILSAFE_CONFIG.failsafe_throttle_low_delay);
            $('input[name="failsafe_delay"]').val(FAILSAFE_CONFIG.failsafe_delay);

            // set stage 2 failsafe procedure
            $('input[type="radio"].procedure').change(function () {
                var element = $(this),
                    checked = element.is(':checked'),
                    id = element.attr('id');
                switch(id) {
                    case 'drop':
                        if (checked) {
                            $('input[name="failsafe_throttle"]').prop("disabled", true);
                            $('input[name="failsafe_off_delay"]').prop("disabled", true);
                        }
                        break;

                    case 'land':
                        if (checked) {
                            $('input[name="failsafe_throttle"]').prop("disabled", false);
                            $('input[name="failsafe_off_delay"]').prop("disabled", false);
                        }
                        break;
                }
            });

            switch(FAILSAFE_CONFIG.failsafe_procedure) {
                default:
                case 0:
                    element = $('input[id="land"]') ;
                    element.prop('checked', true);
                    element.change();
                    break;
                case 1:
                    element = $('input[id="drop"]');
                    element.prop('checked', true);
                    element.change();
                    break;
            }

            // set stage 2 kill switch option
            $('input[name="failsafe_kill_switch"]').prop('checked', FAILSAFE_CONFIG.failsafe_kill_switch);

        } else {

            // set FAILSAFE feature option (pre API 1.15.0)
            failsafeFeature = $('input[name="failsafe_feature"]');
            failsafeFeature.prop('checked', bit_check(BF_CONFIG.features, 8));

            // fill failsafe_throttle field (pre API 1.15.0)
            $('input[name="failsafe_throttle_old"]').val(MISC.failsafe_throttle);
        }

        $('a.save').click(function () {
            // gather data that doesn't have automatic change event bound
            if(apiVersionGte1_15_0) {
                RX_CONFIG.rx_min_usec = parseInt($('input[name="rx_min_usec"]').val());
                RX_CONFIG.rx_max_usec = parseInt($('input[name="rx_max_usec"]').val());

                // get FAILSAFE feature option (>= API 1.15.0)
                if ($('input[name="failsafe_feature_new"]').is(':checked')) {
                    BF_CONFIG.features = bit_set(BF_CONFIG.features, 8);
                } else {
                    BF_CONFIG.features = bit_clear(BF_CONFIG.features, 8);
                }

                FAILSAFE_CONFIG.failsafe_throttle = parseInt($('input[name="failsafe_throttle"]').val());
                FAILSAFE_CONFIG.failsafe_off_delay = parseInt($('input[name="failsafe_off_delay"]').val());
                FAILSAFE_CONFIG.failsafe_throttle_low_delay = parseInt($('input[name="failsafe_throttle_low_delay"]').val());
                FAILSAFE_CONFIG.failsafe_delay = parseInt($('input[name="failsafe_delay"]').val());

                if( $('input[id="land"]').is(':checked')) {
                    FAILSAFE_CONFIG.failsafe_procedure = 0;
                } else if( $('input[id="drop"]').is(':checked')) {
                    FAILSAFE_CONFIG.failsafe_procedure = 1;
                }

                FAILSAFE_CONFIG.failsafe_kill_switch = $('input[name="failsafe_kill_switch"]').is(':checked') ? 1 : 0;
            } else {
                // get FAILSAFE feature option (pre API 1.15.0)
                if ($('input[name="failsafe_feature"]').is(':checked')) {
                    BF_CONFIG.features = bit_set(BF_CONFIG.features, 8);
                } else {
                    BF_CONFIG.features = bit_clear(BF_CONFIG.features, 8);
                }

                // get failsafe_throttle field value (pre API 1.15.0)
                MISC.failsafe_throttle = parseInt($('input[name="failsafe_throttle_old"]').val());
            }

            function save_failssafe_config() {
                MSP.send_message(MSP_codes.MSP_SET_FAILSAFE_CONFIG, MSP.crunch(MSP_codes.MSP_SET_FAILSAFE_CONFIG), false, save_rxfail_config);
            }

            function save_rxfail_config() {
                MSP.sendRxFailConfig(save_bf_config);
            }

            function save_bf_config() {
                MSP.send_message(MSP_codes.MSP_SET_BF_CONFIG, MSP.crunch(MSP_codes.MSP_SET_BF_CONFIG), false, save_to_eeprom);
            }

            // BEGIN pre API 1.15.0 save functions
            function save_misc() {
                MSP.send_message(MSP_codes.MSP_SET_MISC, MSP.crunch(MSP_codes.MSP_SET_MISC), false, save_to_eeprom);
            }
            // END pre API 1.15.0 save functions

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

                if (BOARD.find_board_definition(CONFIG.boardIdentifier).vcp) { // VCP-based flight controls may crash old drivers, we catch and reconnect
                    $('a.connect').click();
                    GUI.timeout_add('start_connection',function start_connection() {
                        $('a.connect').click();
                    },2500);
                } else {

                    GUI.timeout_add('waiting_for_bootup', function waiting_for_bootup() {
                        MSP.send_message(MSP_codes.MSP_IDENT, false, false, function () {
                            GUI.log(chrome.i18n.getMessage('deviceReady'));
                            TABS.failsafe.initialize(false, $('#content').scrollTop());
                        });
                    },1500); // 1500 ms seems to be just the right amount of delay to prevent data request timeouts
                }
            }

            if(apiVersionGte1_15_0) {
                MSP.send_message(MSP_codes.MSP_SET_RX_CONFIG, MSP.crunch(MSP_codes.MSP_SET_RX_CONFIG), false, save_failssafe_config);
            } else {
                MSP.send_message(MSP_codes.MSP_SET_BF_CONFIG, MSP.crunch(MSP_codes.MSP_SET_BF_CONFIG), false, save_misc);
            }
        });

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        GUI.content_ready(callback);
    }
};

TABS.failsafe.cleanup = function (callback) {
    if (callback) callback();
};
