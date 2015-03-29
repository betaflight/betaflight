'use strict';

TABS.configuration = {};

TABS.configuration.initialize = function (callback, scrollPosition) {
    var self = this;

    if (GUI.active_tab != 'configuration') {
        GUI.active_tab = 'configuration';
        googleAnalytics.sendAppView('Configuration');
    }

    function load_config() {
        MSP.send_message(MSP_codes.MSP_BF_CONFIG, false, false, load_serial_config);
    }

    function load_serial_config() {
        if (CONFIG.apiVersion < 1.6) {
            MSP.send_message(MSP_codes.MSP_CF_SERIAL_CONFIG, false, false, load_rc_map);
        } else {
            load_rc_map();
        }
    }

    function load_rc_map() {
        MSP.send_message(MSP_codes.MSP_RX_MAP, false, false, load_misc);
    }

    function load_misc() {
        MSP.send_message(MSP_codes.MSP_MISC, false, false, load_acc_trim);
    }

    function load_acc_trim() {
        MSP.send_message(MSP_codes.MSP_ACC_TRIM, false, false
                        , CONFIG.apiVersion >= 1.8 ? load_arming_config : load_html);
    }

    function load_arming_config() {
        MSP.send_message(MSP_codes.MSP_ARMING_CONFIG, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/configuration.html", process_html);
    }

    MSP.send_message(MSP_codes.MSP_IDENT, false, false, load_config);

    function process_html() {
        // translate to user-selected language
        localize();

        var mixer_list_e = $('select.mixerList');
        for (var i = 0; i < mixerList.length; i++) {
            mixer_list_e.append('<option value="' + (i + 1) + '">' + mixerList[i].name + '</option>');
        }

        mixer_list_e.change(function () {
            var val = parseInt($(this).val());

            BF_CONFIG.mixerConfiguration = val;

            $('.mixerPreview img').attr('src', './resources/motor_order/' + mixerList[val - 1].image + '.svg');
        });

        // select current mixer configuration
        mixer_list_e.val(BF_CONFIG.mixerConfiguration).change();

        // generate features
        var features = [
            {bit: 0, group: 'rxMode', mode: 'group', name: 'RX_PPM', description: 'PPM RX input'},
            {bit: 1, group: 'batteryVoltage', name: 'VBAT', description: 'Battery voltage monitoring'},
            {bit: 2, group: 'other', name: 'INFLIGHT_ACC_CAL', description: 'In-flight level calibration'},
            {bit: 3, group: 'rxMode', mode: 'group', name: 'RX_SERIAL', description: 'Serial-based receiver (SPEKSAT, SBUS, SUMD)'},
            {bit: 4, group: 'esc', name: 'MOTOR_STOP', description: 'Don\'t spin the motors when armed'},
            {bit: 5, group: 'other', name: 'SERVO_TILT', description: 'Servo gimbal'},
            {bit: 6, group: 'other', name: 'SOFTSERIAL', description: 'Enable CPU based serial ports (configure port scenario first)'},
            {bit: 7, group: 'gps', name: 'GPS', description: 'GPS (configure port scenario first)'},
            {bit: 8, group: 'rxFailsafe', name: 'FAILSAFE', description: 'Failsafe settings on RX signal loss'},
            {bit: 9, group: 'other', name: 'SONAR', description: 'Sonar'},
            {bit: 10, group: 'other', name: 'TELEMETRY', description: 'Telemetry output'},
            {bit: 11, group: 'batteryCurrent', name: 'CURRENT_METER', description: 'Battery current monitoring'},
            {bit: 12, group: 'other', name: '3D', description: '3D mode (for use with reversible ESCs)'},
            {bit: 13, group: 'rxMode', mode: 'group', name: 'RX_PARALLEL_PWM', description: 'PWM RX input'},
            {bit: 14, group: 'rxMode', mode: 'group', name: 'RX_MSP', description: 'MSP RX input'},
            {bit: 15, group: 'rssi', name: 'RSSI_ADC', description: 'Analog RSSI input'},
            {bit: 16, group: 'other', name: 'LED_STRIP', description: 'Addressable RGB LED strip support'},
            {bit: 17, group: 'other', name: 'DISPLAY', description: 'OLED Screen Display'},
            {bit: 18, group: 'esc', name: 'ONESHOT125', description: 'ONESHOT ESC support (disconnect ESCs, remove props)'},
            {bit: 19, group: 'other', name: 'BLACKBOX', description: 'Blackbox flight data recorder'}
        ];

        var radioGroups = [];
        
        var features_e = $('.features');
        for (var i = 0; i < features.length; i++) {
            var row_e;
            
            if (features[i].mode === 'group') {
                row_e = $('<tr><td><input class="feature" id="feature-'
                        + i
                        + '" value="'
                        + features[i].bit
                        + '" title="'
                        + features[i].name
                        + '" type="radio" name="'
                        + features[i].group
                        + '" /></td><td><label for="feature-'
                        + i
                        + '">'
                        + features[i].name
                        + '</label></td><td>'
                        + features[i].description
                        + '</td>');
                radioGroups.push(features[i].group);
            } else {
                row_e = $('<tr><td><input class="feature" id="feature-'
                        + i
                        + '" name="'
                        + features[i].name
                        + '" title="'
                        + features[i].name
                        + '" type="checkbox" /></td><td><label for="feature-'
                        + i
                        + '">'
                        + features[i].name
                        + '</label></td><td>'
                        + features[i].description
                        + '</td>');
                
                var feature_e = row_e.find('input.feature');

                feature_e.prop('checked', bit_check(BF_CONFIG.features, features[i].bit));
                feature_e.data('bit', features[i].bit);
            }

            features_e.each(function () {
                if ($(this).hasClass(features[i].group)) {
                    $(this).append(row_e);
                }
            });
        }

        for (var i = 0; i < radioGroups.length; i++) {
            var group = radioGroups[i];
            var controls_e = $('input[name="' + group + '"].feature');
            
            
            controls_e.each(function() {
                var bit = parseInt($(this).attr('value'));
                var state = bit_check(BF_CONFIG.features, bit);
                
                $(this).prop('checked', state);
            });
        }

        // generate GPS
        var gpsProtocols = [
            'NMEA',
            'UBLOX'
        ];

        var gpsBaudRates = [
            '115200',
            '57600',
            '38400',
            '19200',
            '9600'
        ];

        var gpsSbas = [
            'Disabled',
            'Auto-detect',
            'European EGNOS',
            'North American WAAS',
            'Japanese MSAS',
            'Indian GAGAN'
        ];

        var gps_protocol_e = $('select.gps_protocol');
        for (var i = 0; i < gpsProtocols.length; i++) {
            gps_protocol_e.append('<option value="' + i + '">' + gpsProtocols[i] + '</option>');
        }

        gps_protocol_e.change(function () {
            MISC.gps_type = parseInt($(this).val());
        });

        gps_protocol_e.val(MISC.gps_type);
        
        
        var gps_baudrate_e = $('select.gps_baudrate');
        for (var i = 0; i < gpsBaudRates.length; i++) {
            gps_baudrate_e.append('<option value="' + gpsBaudRates[i] + '">' + gpsBaudRates[i] + '</option>');
        }
    
        if (CONFIG.apiVersion < 1.6) {
            gps_baudrate_e.change(function () {
                SERIAL_CONFIG.gpsBaudRate = parseInt($(this).val());
            });
            gps_baudrate_e.val(SERIAL_CONFIG.gpsBaudRate);
        } else {
            gps_baudrate_e.prop("disabled", true);
            gps_baudrate_e.parent().hide();
        }
        
        
        var gps_ubx_sbas_e = $('select.gps_ubx_sbas');
        for (var i = 0; i < gpsSbas.length; i++) {
            gps_ubx_sbas_e.append('<option value="' + (i - 1) + '">' + gpsSbas[i] + '</option>');
        }

        gps_ubx_sbas_e.change(function () {
            MISC.gps_ubx_sbas = parseInt($(this).val());
        });

        gps_ubx_sbas_e.val(MISC.gps_ubx_sbas);


        // generate serial RX
        var serialRXtypes = [
            'SPEKTRUM1024',
            'SPEKTRUM2048',
            'SBUS',
            'SUMD',
            'SUMH',
            'XBUS_MODE_B',
            'XBUS_MODE_B_RJ01'
        ];

        var serialRX_e = $('select.serialRX');
        for (var i = 0; i < serialRXtypes.length; i++) {
            serialRX_e.append('<option value="' + i + '">' + serialRXtypes[i] + '</option>');
        }

        serialRX_e.change(function () {
            BF_CONFIG.serialrx_type = parseInt($(this).val());
        });

        // select current serial RX type
        serialRX_e.val(BF_CONFIG.serialrx_type);

        // for some odd reason chrome 38+ changes scroll according to the touched select element
        // i am guessing this is a bug, since this wasn't happening on 37
        // code below is a temporary fix, which we will be able to remove in the future (hopefully)
        $('#content').scrollTop((scrollPosition) ? scrollPosition : 0);

        // fill board alignment
        $('input[name="board_align_roll"]').val(BF_CONFIG.board_align_roll);
        $('input[name="board_align_pitch"]').val(BF_CONFIG.board_align_pitch);
        $('input[name="board_align_yaw"]').val(BF_CONFIG.board_align_yaw);

        // fill accel trims
        $('input[name="roll"]').val(CONFIG.accelerometerTrims[1]);
        $('input[name="pitch"]').val(CONFIG.accelerometerTrims[0]);

        // fill magnetometer
        $('input[name="mag_declination"]').val(MISC.mag_declination);

        //fill motor disarm params        
        if(CONFIG.apiVersion >= 1.8) {
            $('input[name="autodisarmdelay"]').val(ARMING_CONFIG.auto_disarm_delay);
            $('input[name="disarmkillswitch"]').prop('checked', ARMING_CONFIG.disarm_kill_switch);
            if(bit_check(BF_CONFIG.features, 4 + 1))//MOTOR_STOP
                $('div.disarmdelay').show();
        }
        else       
            $('div.disarm').hide();
        
        // fill throttle
        $('input[name="minthrottle"]').val(MISC.minthrottle);
        $('input[name="midthrottle"]').val(MISC.midrc);
        $('input[name="maxthrottle"]').val(MISC.maxthrottle);
        $('input[name="failsafe_throttle"]').val(MISC.failsafe_throttle);
        $('input[name="mincommand"]').val(MISC.mincommand);

        // fill battery
        $('input[name="mincellvoltage"]').val(MISC.vbatmincellvoltage);
        $('input[name="maxcellvoltage"]').val(MISC.vbatmaxcellvoltage);
        $('input[name="warningcellvoltage"]').val(MISC.vbatwarningcellvoltage);
        $('input[name="voltagescale"]').val(MISC.vbatscale);

        // fill current
        $('input[name="currentscale"]').val(BF_CONFIG.currentscale);
        $('input[name="currentoffset"]').val(BF_CONFIG.currentoffset);
        $('input[name="multiwiicurrentoutput"]').prop('checked', MISC.multiwiicurrentoutput);


        // UI hooks
        $('input[type="checkbox"].feature', features_e).change(function () {
            var element = $(this),
                index = element.data('bit'),
                state = element.is(':checked');

            if (state) {
                BF_CONFIG.features = bit_set(BF_CONFIG.features, index);
                if(element.attr('name') === 'MOTOR_STOP')                    
                    $('div.disarmdelay').show();
            } else {
                BF_CONFIG.features = bit_clear(BF_CONFIG.features, index);
                if(element.attr('name') === 'MOTOR_STOP')
                    $('div.disarmdelay').hide();
            }
        });

        // UI hooks
        $('input[type="radio"].feature', features_e).change(function () {
            var element = $(this),
                group = element.attr('name');

            var controls_e = $('input[name="' + group + '"]');
            var selected_bit = controls_e.filter(':checked').val();
            
            controls_e.each(function() {
                var bit = $(this).attr('value');
                
                var selected = (selected_bit == bit);
                if (selected) {
                    BF_CONFIG.features = bit_set(BF_CONFIG.features, bit);
                } else {
                    BF_CONFIG.features = bit_clear(BF_CONFIG.features, bit);
                }

            });
        });

        $('a.save').click(function () {
            // gather data that doesn't have automatic change event bound
            BF_CONFIG.board_align_roll = parseInt($('input[name="board_align_roll"]').val());
            BF_CONFIG.board_align_pitch = parseInt($('input[name="board_align_pitch"]').val());
            BF_CONFIG.board_align_yaw = parseInt($('input[name="board_align_yaw"]').val());

            CONFIG.accelerometerTrims[1] = parseInt($('input[name="roll"]').val());
            CONFIG.accelerometerTrims[0] = parseInt($('input[name="pitch"]').val());
            MISC.mag_declination = parseFloat($('input[name="mag_declination"]').val());
            
            // motor disarm
            if(CONFIG.apiVersion >= 1.8) {
                ARMING_CONFIG.auto_disarm_delay = parseInt($('input[name="autodisarmdelay"]').val());
                ARMING_CONFIG.disarm_kill_switch = ~~$('input[name="disarmkillswitch"]').is(':checked'); // ~~ boolean to decimal conversion
            }
            
            MISC.minthrottle = parseInt($('input[name="minthrottle"]').val());
            MISC.midrc = parseInt($('input[name="midthrottle"]').val());
            MISC.maxthrottle = parseInt($('input[name="maxthrottle"]').val());
            MISC.failsafe_throttle = parseInt($('input[name="failsafe_throttle"]').val());
            MISC.mincommand = parseInt($('input[name="mincommand"]').val());

            MISC.vbatmincellvoltage = parseFloat($('input[name="mincellvoltage"]').val());
            MISC.vbatmaxcellvoltage = parseFloat($('input[name="maxcellvoltage"]').val());
            MISC.vbatwarningcellvoltage = parseFloat($('input[name="warningcellvoltage"]').val());
            MISC.vbatscale = parseInt($('input[name="voltagescale"]').val());

            BF_CONFIG.currentscale = parseInt($('input[name="currentscale"]').val());
            BF_CONFIG.currentoffset = parseInt($('input[name="currentoffset"]').val());
            MISC.multiwiicurrentoutput = ~~$('input[name="multiwiicurrentoutput"]').is(':checked'); // ~~ boolean to decimal conversion

            function save_serial_config() {
                if (CONFIG.apiVersion < 1.6) {
                    MSP.send_message(MSP_codes.MSP_SET_CF_SERIAL_CONFIG, MSP.crunch(MSP_codes.MSP_SET_CF_SERIAL_CONFIG), false, save_misc);
                } else {
                    save_misc();
                }
            }

            function save_misc() {
                MSP.send_message(MSP_codes.MSP_SET_MISC, MSP.crunch(MSP_codes.MSP_SET_MISC), false, save_acc_trim);
            }

            function save_acc_trim() {
                MSP.send_message(MSP_codes.MSP_SET_ACC_TRIM, MSP.crunch(MSP_codes.MSP_SET_ACC_TRIM), false
                                , CONFIG.apiVersion >= 1.8 ? save_arming_config : save_to_eeprom);
            }

            function save_arming_config() {
                MSP.send_message(MSP_codes.MSP_SET_ARMING_CONFIG, MSP.crunch(MSP_codes.MSP_SET_ARMING_CONFIG), false, save_to_eeprom);
            }

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
                        TABS.configuration.initialize(false, $('#content').scrollTop());
                    });
                },1500); // 1500 ms seems to be just the right amount of delay to prevent data request timeouts
            }

            MSP.send_message(MSP_codes.MSP_SET_BF_CONFIG, MSP.crunch(MSP_codes.MSP_SET_BF_CONFIG), false, save_serial_config);
        });

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    }
};

TABS.configuration.cleanup = function (callback) {
    if (callback) callback();
};
