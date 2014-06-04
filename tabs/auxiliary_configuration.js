function tab_initialize_auxiliary_configuration() {
    ga_tracker.sendAppView('Auxiliary Configuration');
    GUI.active_tab = 'auxiliary_configuration';

    MSP.send_message(MSP_codes.MSP_BOXNAMES, false, false, get_box_data);

    function get_box_data() {
        MSP.send_message(MSP_codes.MSP_BOX, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/auxiliary_configuration.html", process_html);
    }

    function process_html() {
        // translate to user-selected language
        localize();

        function box_check(num, pos) {
            if (bit_check(num, pos)) { // 1
                return '<td><input type="checkbox" checked="checked" /></td>';
            } else { // 0
                return '<td><input type="checkbox" /></td>';
            }
        }

        // val = channel value
        // aux_num = position of corresponding aux channel in the html table
        function box_highlight(val, aux_num) {
            var tr = $('table.boxes .switches');
            var pos = 0; // < 1300

            if (val > 1300 && val < 1700) {
                pos = 1;
            } else if (val > 1700) {
                pos = 2;
            }

            $('td:nth-child(' + aux_num + '), td:nth-child(' + (aux_num + 1) + '), td:nth-child(' + (aux_num + 2) + ')', tr).css('background-color', 'transparent');
            $('td:nth-child(' + (aux_num + pos) + ')', tr).css('background-color', 'orange');
        }

        // generate table from the supplied AUX names and AUX data
        for (var i = 0; i < AUX_CONFIG.length; i++) {
            $('.boxes > tbody:last').append(
                '<tr class="switches">' +
                    '<td class="name">' + AUX_CONFIG[i] + '</td>' +
                    box_check(AUX_CONFIG_values[i], 0) +
                    box_check(AUX_CONFIG_values[i], 1) +
                    box_check(AUX_CONFIG_values[i], 2) +

                    box_check(AUX_CONFIG_values[i], 3) +
                    box_check(AUX_CONFIG_values[i], 4) +
                    box_check(AUX_CONFIG_values[i], 5) +

                    box_check(AUX_CONFIG_values[i], 6) +
                    box_check(AUX_CONFIG_values[i], 7) +
                    box_check(AUX_CONFIG_values[i], 8) +

                    box_check(AUX_CONFIG_values[i], 9) +
                    box_check(AUX_CONFIG_values[i], 10) +
                    box_check(AUX_CONFIG_values[i], 11) +

                    box_check(AUX_CONFIG_values[i], 16) +
                    box_check(AUX_CONFIG_values[i], 17) +
                    box_check(AUX_CONFIG_values[i], 18) +

                    box_check(AUX_CONFIG_values[i], 19) +
                    box_check(AUX_CONFIG_values[i], 20) +
                    box_check(AUX_CONFIG_values[i], 21) +

                    box_check(AUX_CONFIG_values[i], 22) +
                    box_check(AUX_CONFIG_values[i], 23) +
                    box_check(AUX_CONFIG_values[i], 24) +

                    box_check(AUX_CONFIG_values[i], 25) +
                    box_check(AUX_CONFIG_values[i], 26) +
                    box_check(AUX_CONFIG_values[i], 27) +
                    '</tr>'
            );
        }

        // UI Hooks
        $('a.update').click(function() {
            // catch the input changes
            var main_needle = 0;
            var needle = 0;

            var boxCountFor4AuxChannels = 3 * 4;
            var boxCountPerLine = boxCountFor4AuxChannels;
            if (bit_check(CONFIG.capability, 5)) {
                boxCountPerLine = boxCountFor4AuxChannels * 2;
            }

            $('.boxes input').each(function() {
                var bitIndex = needle;
                if (bit_check(CONFIG.capability, 5) && needle >= boxCountFor4AuxChannels) {
                    bitIndex += 4; // 0-11 bits for aux 1-4, 16-27 for aux 5-8
                }
                
                if ($(this).is(':checked')) {
                    AUX_CONFIG_values[main_needle] = bit_set(AUX_CONFIG_values[main_needle], bitIndex);
                } else {
                    AUX_CONFIG_values[main_needle] = bit_clear(AUX_CONFIG_values[main_needle], bitIndex);
                }

                needle++;

                if (needle >= boxCountPerLine) {
                    main_needle++;
                    needle = 0;
                }
            });

            // send over the data
            var AUX_val_buffer_out = new Array();

            var needle = 0;
            for (var i = 0; i < AUX_CONFIG_values.length; i++) {
                AUX_val_buffer_out[needle++] = lowByte(AUX_CONFIG_values[i] & 0xFFF);
                AUX_val_buffer_out[needle++] = highByte(AUX_CONFIG_values[i] & 0xFFF);
            }
            if (bit_check(CONFIG.capability, 5)) {
                for (var i = 0; i < AUX_CONFIG_values.length; i++) {
                    AUX_val_buffer_out[needle++] = lowByte((AUX_CONFIG_values[i] >> 16) & 0xFFF);
                    AUX_val_buffer_out[needle++] = highByte((AUX_CONFIG_values[i] >> 16) & 0xFFF);
                }
            }
            
            MSP.send_message(MSP_codes.MSP_SET_BOX, AUX_val_buffer_out, false, save_to_eeprom);

            function save_to_eeprom() {
                MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function() {
                    GUI.log(chrome.i18n.getMessage('auxiliaryEepromSaved'));

                    var element = $('a.update');
                    element.addClass('success');

                    GUI.timeout_add('success_highlight', function() {
                        element.removeClass('success');
                    }, 2000);
                });
            }
        });

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

            box_highlight(RC.channels[4],  2);  // aux 1
            box_highlight(RC.channels[5],  5);  // aux 2
            box_highlight(RC.channels[6],  8);  // aux 3
            box_highlight(RC.channels[7],  11); // aux 4
            if (RC.active_channels > 8) {
                box_highlight(RC.channels[8],  14); // aux 5
                box_highlight(RC.channels[9],  17); // aux 6
                box_highlight(RC.channels[10], 20); // aux 7
                box_highlight(RC.channels[11], 23); // aux 8
            }
        }

        // enable data pulling
        GUI.interval_add('aux_data_pull', get_rc_data, 50, true);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);
    }
}