'use strict';

TABS.servos = {};
TABS.servos.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'servos') {
        GUI.active_tab = 'servos';
        googleAnalytics.sendAppView('Servos');
    }

    function get_servo_configurations() {
        MSP.send_message(MSP_codes.MSP_SERVO_CONFIGURATIONS, false, false, get_servo_mix_rules);
    }

    function get_servo_mix_rules() {
        MSP.send_message(MSP_codes.MSP_SERVO_MIX_RULES, false, false, get_channel_forwarding);
    }

    function get_channel_forwarding() {
        var nextFunction = get_rc_data;
        
        if (semver.lt(CONFIG.apiVersion, "1.12.0")) {
            MSP.send_message(MSP_codes.MSP_CHANNEL_FORWARDING, false, false, nextFunction);
        } else { 
            nextFunction();
        }
    }

    function get_rc_data() {
        MSP.send_message(MSP_codes.MSP_RC, false, false, get_boxnames_data);
    }

    function get_boxnames_data() {
        MSP.send_message(MSP_codes.MSP_BOXNAMES, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/servos.html", process_html);
    }
    
    MSP.send_message(MSP_codes.MSP_IDENT, false, false, get_servo_configurations);
    
    function update_ui() {
            
        if (semver.lt(CONFIG.apiVersion, "1.12.0") || SERVO_CONFIG.length == 0) {
            
            $(".tab-servos").removeClass("supported");
            return;
        }
        
        $(".tab-servos").addClass("supported");
            
        var servoCheckbox = '';
        var servoHeader = '';
        for (var i = 0; i < RC.active_channels-4; i++) {
            servoHeader = servoHeader + '\
                <th >A' + (i+1) + '</th>\
            ';
        }
        servoHeader = servoHeader + '<th style="width: 110px" i18n="servosDirectionAndRate"></th>';

        for (var i = 0; i < RC.active_channels; i++) {
            servoCheckbox = servoCheckbox + '\
                <td class="channel"><input type="checkbox"/></td>\
            ';
        }

        $('div.tab-servos table.fields tr.main').append(servoHeader);

        function process_servos(name, alternate, obj) {
        
            $('div.supported_wrapper').show();
            
            $('div.tab-servos table.fields').append('\
                <tr> \
                    <td style="text-align: center">' + name + '</td>\
                    <td class="middle"><input type="number" min="500" max="2500" value="' + SERVO_CONFIG[obj].middle + '" /></td>\
                    <td class="min"><input type="number" min="500" max="2500" value="' + SERVO_CONFIG[obj].min +'" /></td>\
                    <td class="max"><input type="number" min="500" max="2500" value="' + SERVO_CONFIG[obj].max +'" /></td>\
                    <td class="angleAtMin"><input type="number" min="-90" max="0" value="' + (-SERVO_CONFIG[obj].angleAtMin) +'" /></td>\
                    <td class="angleAtMax"><input type="number" min="0" max="90" value="' + SERVO_CONFIG[obj].angleAtMax +'" /></td>\
                    ' + servoCheckbox + '\
                    <td class="direction">\
                    </td>\
                </tr> \
            ');

            if (SERVO_CONFIG[obj].indexOfChannelToForward >= 0) {
                $('div.tab-servos table.fields tr:last td.channel input').eq(SERVO_CONFIG[obj].indexOfChannelToForward).prop('checked', true);
            }

            // adding select box and generating options
            $('div.tab-servos table.fields tr:last td.direction').append('\
                <select class="rate" name="rate"></select>\
            ');

            var select = $('div.tab-servos table.fields tr:last td.direction select');

            for (var i = 100; i > -101; i--) {
                select.append('<option value="' + i + '">Rate: ' + i + '%</option>');
            }

            // select current rate
            select.val(SERVO_CONFIG[obj].rate);

            $('div.tab-servos table.fields tr:last').data('info', {'obj': obj});
            
            // UI hooks
            
            // only one checkbox for indicating a channel to forward can be selected at a time, perhaps a radio group would be best here.
            $('div.tab-servos table.fields tr:last td.channel input').click(function () {
                if($(this).is(':checked')) {
                    $(this).parent().parent().find('.channel input').not($(this)).prop('checked', false);
                }
            });
        }

        function servos_update(save_configuration_to_eeprom) {
            $('div.tab-servos table.fields tr:not(".main")').each(function () {
                var info = $(this).data('info');


                var selection = $('.channel input', this);
                var channelIndex = parseInt(selection.index(selection.filter(':checked')));
                if (channelIndex == -1) {
                    channelIndex = undefined;
                }
                
                SERVO_CONFIG[info.obj].indexOfChannelToForward = channelIndex;

                
                SERVO_CONFIG[info.obj].middle = parseInt($('.middle input', this).val());
                SERVO_CONFIG[info.obj].min = parseInt($('.min input', this).val());
                SERVO_CONFIG[info.obj].max = parseInt($('.max input', this).val());
                SERVO_CONFIG[info.obj].angleAtMin = -parseInt($('.angleAtMin input', this).val());
                SERVO_CONFIG[info.obj].angleAtMax = parseInt($('.angleAtMax input', this).val());

                var val = parseInt($('.direction select', this).val());
                SERVO_CONFIG[info.obj].rate = val;
            });
            
            //
            // send data to FC
            //
            MSP.sendServoConfigurations(send_servo_mixer_rules);

            function send_servo_mixer_rules() {
                MSP.sendServoConfigurations(save_to_eeprom);
            }
            
            function save_to_eeprom() {
                if (save_configuration_to_eeprom) {
                    MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                        GUI.log(chrome.i18n.getMessage('servosEepromSave'));
                    });
                }
            }

        }

        // drop previous table
        $('div.tab-servos table.fields tr:not(:first)').remove();

        for (var servoIndex = 0; servoIndex < 8; servoIndex++) {
            process_servos('Servo ' + servoIndex, '', servoIndex, false);
        }

        // UI hooks for dynamically generated elements
        $('table.directions select, table.directions input, table.fields select, table.fields input').change(function () {
            if ($('div.live input').is(':checked')) {
                // apply small delay as there seems to be some funky update business going wrong
                GUI.timeout_add('servos_update', servos_update, 10);
            }
        });
        
        $('a.update').click(function () {
            servos_update(true);
        });
        
    }

    function process_html() {
    
        update_ui();

        // translate to user-selected language
        localize();
        
        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function () {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        GUI.content_ready(callback);
    }
};

TABS.servos.cleanup = function (callback) {
    if (callback) callback();
};
