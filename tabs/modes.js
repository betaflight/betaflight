// Disabled via main.js/main.html, cleanflight does not use MSP_BOX.

'use strict';

TABS.modes = {};
TABS.modes.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'modes') {
        GUI.active_tab = 'modes';
        googleAnalytics.sendAppView('Modes');
    }

    function get_box_data() {
        MSP.send_message(MSP_codes.MSP_BOX, false, false, get_box_ids);
    }

    function get_box_ids() {
        MSP.send_message(MSP_codes.MSP_BOXIDS, false, false, get_rc_data);
    }

    function get_rc_data() {
        MSP.send_message(MSP_codes.MSP_RC, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/modes.html", process_html);
    }

    MSP.send_message(MSP_codes.MSP_BOXNAMES, false, false, get_box_data);

    function process_html() {
        // generate heads according to RC count
        var table_head = $('table.boxes .heads');
        var main_head = $('table.boxes .main');
        for (var i = 0; i < (RC.active_channels - 4); i++) {
            table_head.append('<th colspan="3">AUX ' + (i + 1) + '</th>');

            // 3 columns per aux channel (this might be requested to change to 6 in the future, so watch out)
            main_head.append('\
                <th i18n="auxiliaryLow"></th>\
                <th i18n="auxiliaryMed"></th>\
                <th i18n="auxiliaryHigh"></th>\
            ');
        }

        // translate to user-selected language
        localize();

        // generate table from the supplied AUX names and AUX data
        for (var i = 0; i < AUX_CONFIG.length; i++) {
            var line = '<tr class="switches">';
            line += '<td class="name">' + AUX_CONFIG[i] + '</td>';

            for (var j = 0; j < (RC.active_channels - 4) * 3; j++) {
                if (bit_check(AUX_CONFIG_values[i], j)) {
                    line += '<td><input type="checkbox" checked="checked" /></td>';
                } else {
                    line += '<td><input type="checkbox" /></td>';
                }
            }

            line += '</tr>';

            $('.boxes > tbody:last').append(line);
        }

        // UI Hooks
        $('a.update').click(function () {
            // catch the input changes
            var main_needle = 0,
                needle = 0;

            $('.boxes input').each(function () {
                if ($(this).is(':checked')) {
                    AUX_CONFIG_values[main_needle] = bit_set(AUX_CONFIG_values[main_needle], needle);
                } else {
                    AUX_CONFIG_values[main_needle] = bit_clear(AUX_CONFIG_values[main_needle], needle);
                }

                needle++;

                if (needle >= (RC.active_channels - 4) * 3) { // 1 aux * 3 checkboxes, 4 AUX = 12 bits per line
                    main_needle++;
                    needle = 0;
                }
            });

            function save_to_eeprom() {
                MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('auxiliaryEepromSaved'));
                });
            }

            MSP.send_message(MSP_codes.MSP_SET_BOX, MSP.crunch(MSP_codes.MSP_SET_BOX), false, save_to_eeprom);
        });

        // val = channel value
        // aux_num = position of corresponding aux channel in the html table
        var switches_e = $('table.boxes .switches');
        function box_highlight(aux_num, val) {
            var pos = 0; // < 1300

            if (val > 1300 && val < 1700) {
                pos = 1;
            } else if (val > 1700) {
                pos = 2;
            }

            var highlight_column = (aux_num * 3) + pos + 2; // +2 to skip name column and index starting on 1 instead of 0
            var erase_columns = (aux_num * 3) + 2;

            $('td:nth-child(n+' + erase_columns + '):nth-child(-n+' + (erase_columns + 2) + ')', switches_e).css('background-color', 'transparent');
            $('td:nth-child(' + highlight_column + ')', switches_e).css('background-color', 'orange');
        }

        // data pulling functions used inside interval timer
        function get_rc_data() {
            MSP.send_message(MSP_codes.MSP_RC, false, false, update_ui);
        }

        function update_ui() {
            for (var i = 0; i < AUX_CONFIG.length; i++) {
                if (bit_check(CONFIG.mode, i)) {
                    $('td.name').eq(i).addClass('on').removeClass('off');
                } else {
                    $('td.name').eq(i).removeClass('on').removeClass('off');

                    if (AUX_CONFIG_values[i] > 0) {
                        $('td.name').eq(i).addClass('off');
                    }
                }

            }

            for (var i = 0; i < (RC.active_channels - 4); i++) {
                box_highlight(i, RC.channels[i + 4]);
            }
        }

        // update ui instantly on first load
        update_ui();

        // enable data pulling
        GUI.interval_add('aux_data_pull', get_rc_data, 50);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        GUI.content_ready(callback);
    }
};

TABS.modes.cleanup = function (callback) {
    if (callback) callback();
};