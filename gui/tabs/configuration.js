'use strict';

TABS.configuration = {};

TABS.configuration.initialize = function (callback, scrollPosition) {
    var self = this;

    if (GUI.active_tab != 'configuration') {
        GUI.active_tab = 'configuration';
    }

    function load_config() {
        MSP.send_message(MSPCodes.MSP_BF_CONFIG, false, false, load_serial_config);
    }

    function load_serial_config() {
        var next_callback = load_rc_map;
        if (semver.lt(CONFIG.apiVersion, "1.6.0")) {
            MSP.send_message(MSPCodes.MSP_CF_SERIAL_CONFIG, false, false, next_callback);
        } else {
            next_callback();
        }
    }

    function load_rc_map() {
        MSP.send_message(MSPCodes.MSP_RX_MAP, false, false, load_misc);
    }

    function load_misc() {
        MSP.send_message(MSPCodes.MSP_MISC, false, false, load_acc_trim);
    }
    
    function load_acc_trim() {
        MSP.send_message(MSPCodes.MSP_ACC_TRIM, false, false, load_arming_config);
    }

    function load_arming_config() {
        var next_callback = load_loop_time;
        if (semver.gte(CONFIG.apiVersion, "1.8.0")) {
            MSP.send_message(MSPCodes.MSP_ARMING_CONFIG, false, false, next_callback);
        } else {
            next_callback();
        }
    }
    
    function load_loop_time() {
        var next_callback = load_3d;
        if (semver.gte(CONFIG.apiVersion, "1.8.0")) {
            MSP.send_message(MSPCodes.MSP_LOOP_TIME, false, false, next_callback);
        } else {
            next_callback();
        }
    }

    function load_3d() {
        var next_callback = load_rc_deadband;
        if (semver.gte(CONFIG.apiVersion, "1.14.0")) {
            MSP.send_message(MSPCodes.MSP_3D, false, false, next_callback);
        } else {
            next_callback();
        }
    }

    function load_rc_deadband() {
        var next_callback = esc_protocol;
        if (semver.gte(CONFIG.apiVersion, "1.17.0")) {
            MSP.send_message(MSPCodes.MSP_RC_DEADBAND, false, false, next_callback);
        } else {
            next_callback();
        }
    }

    function esc_protocol() {
        var next_callback = sensor_config;
        if (CONFIG.flightControllerIdentifier == "BTFL" && semver.gte(CONFIG.flightControllerVersion, "2.8.1")) {
            MSP.send_message(MSPCodes.MSP_ADVANCED_CONFIG, false, false, next_callback);
        } else {
            next_callback();
        }        
    }
    
    function sensor_config() {
        var next_callback = load_sensor_alignment;
        if (CONFIG.flightControllerIdentifier == "BTFL" && semver.gte(CONFIG.flightControllerVersion, "2.8.2")) {
            MSP.send_message(MSPCodes.MSP_SENSOR_CONFIG, false, false, next_callback);
        } else {
            next_callback();
        }
    }
    
    function load_sensor_alignment() {
        var next_callback = load_name;
        if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
            MSP.send_message(MSPCodes.MSP_SENSOR_ALIGNMENT, false, false, next_callback);
        } else {
            next_callback();
        }
    }
    
    function load_name() {
        var next_callback = load_html;
        if (CONFIG.flightControllerIdentifier == "BTFL" && semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
            MSP.send_message(MSPCodes.MSP_NAME, false, false, next_callback);
        } else {
            next_callback();
        }
    }

    
    //Update Analog/Battery Data
    function load_analog() {
        MSP.send_message(MSPCodes.MSP_ANALOG, false, false, function () {
	    $('input[name="batteryvoltage"]').val([ANALOG.voltage.toFixed(1)]);
	    $('input[name="batterycurrent"]').val([ANALOG.amperage.toFixed(2)]);
            });
    }

    function load_html() {
        $('#content').load("./tabs/configuration.html", process_html);
    }

    load_config();
    
    function process_html() {
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

        var radioGroups = [];
        var features_e = $('.tab-configuration .features');

        BF_CONFIG.features.generateElements(features_e);
        
        // translate to user-selected language
        localize();

        var alignments = [
            'CW 0°',
            'CW 90°',
            'CW 180°',
            'CW 270°',
            'CW 0° flip',
            'CW 90° flip',
            'CW 180° flip',
            'CW 270° flip'
        ];
        
        
        
        var orientation_gyro_e = $('select.gyroalign');
        var orientation_acc_e = $('select.accalign');
        var orientation_mag_e = $('select.magalign');
        

        if (semver.lt(CONFIG.apiVersion, "1.15.0")) {
            $('.tab-configuration .sensoralignment').hide();
        } else {
            for (var i = 0; i < alignments.length; i++) {
                orientation_gyro_e.append('<option value="' + (i+1) + '">'+ alignments[i] + '</option>');
                orientation_acc_e.append('<option value="' + (i+1) + '">'+ alignments[i] + '</option>');
                orientation_mag_e.append('<option value="' + (i+1) + '">'+ alignments[i] + '</option>');
            }
            orientation_gyro_e.val(SENSOR_ALIGNMENT.align_gyro);
            orientation_acc_e.val(SENSOR_ALIGNMENT.align_acc);
            orientation_mag_e.val(SENSOR_ALIGNMENT.align_mag);
        }
        
        // ESC protocols 
        
        var escprotocols = [
            'PWM',
            'ONESHOT125',
            'ONESHOT42',
            'MULTISHOT'
        ];
        
        if (CONFIG.flightControllerIdentifier == "BTFL" && semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
            escprotocols.push('BRUSHED');
        }

        var esc_protocol_e = $('select.escprotocol');

        for (var i = 0; i < escprotocols.length; i++) {
            esc_protocol_e.append('<option value="' + (i+1) + '">'+ escprotocols[i] + '</option>');
        }
        esc_protocol_e.val(PID_ADVANCED_CONFIG.fast_pwm_protocol+1);
        
        
        $('input[id="unsyncedPWMSwitch"]').prop('checked', PID_ADVANCED_CONFIG.use_unsyncedPwm !== 0);
        $('input[name="unsyncedpwmfreq"]').val(PID_ADVANCED_CONFIG.motor_pwm_rate);
        if (PID_ADVANCED_CONFIG.use_unsyncedPwm) {
            $('div.unsyncedpwmfreq').show();
        }
        else {
            $('div.unsyncedpwmfreq').hide();
        }
        
        // Gyro and PID update
        var gyroFreq = [
            "8KHz",
            "4KHz",
            "2.67KHz", 
            "2KHz",
            "1.6KHz",
            "1.33KHz",
            "1.14KHz",
            "1KHz"
        ];
        
        var gyro_select_e = $('select.gyroSyncDenom');
                
        for (var i = 0; i < gyroFreq.length; i++) {
            gyro_select_e.append('<option value="'+(i+1)+'">'+gyroFreq[i]+'</option>');
        }
        gyro_select_e.val(PID_ADVANCED_CONFIG.gyro_sync_denom);
 
        var gyroDenom = PID_ADVANCED_CONFIG.gyro_sync_denom;
        var pidFreq = [
            8 / (gyroDenom * 1),
            8 / (gyroDenom * 2),
            8 / (gyroDenom * 3),
            8 / (gyroDenom * 4),
            8 / (gyroDenom * 5),
            8 / (gyroDenom * 6),
            8 / (gyroDenom * 7),
            8 / (gyroDenom * 8)
        ];
 
        var pid_select_e = $('select.pidProcessDenom');
        for (var i = 0; i < pidFreq.length; i++) {
            var pidF = (1000 * pidFreq[i] / 10); // Could be done better
            pidF = pidF.toFixed(0);
            pid_select_e.append('<option value="'+(i+1)+'">'+(pidF / 100).toString()+'KHz</option>');
        }
        pid_select_e.val(PID_ADVANCED_CONFIG.pid_process_denom);
        
        $('select.gyroSyncDenom').change(function() {
           var gyroDenom = $('select.gyroSyncDenom').val();
           var newPidFreq = [
                8 / (gyroDenom * 1),
                8 / (gyroDenom * 2),
                8 / (gyroDenom * 3),
                8 / (gyroDenom * 4),
                8 / (gyroDenom * 5),
                8 / (gyroDenom * 6),
                8 / (gyroDenom * 7),
                8 / (gyroDenom * 8)
            ];
           for (var i=0; i<newPidFreq.length;i++) {
                var pidF = (1000 * newPidFreq[i] / 10); // Could be done better
                pidF = pidF.toFixed(0);
                $('select.pidProcessDenom option[value="'+(i+1)+'"]').text((pidF / 100).toString()+'KHz');}
           
        });

        $('input[id="accHardwareSwitch"]').prop('checked', SENSOR_CONFIG.acc_hardware !== 1);
        $('input[id="baroHardwareSwitch"]').prop('checked', SENSOR_CONFIG.baro_hardware !== 1);
        $('input[id="magHardwareSwitch"]').prop('checked', SENSOR_CONFIG.mag_hardware !== 1);


        // Only show these sections for supported FW
        if (CONFIG.flightControllerIdentifier == "BTFL" && semver.lt(CONFIG.flightControllerVersion, "2.8.1")) {
            $('.selectProtocol').hide();
            $('.checkboxPwm').hide();
            $('.selectPidProcessDenom').hide();
        }
        
        if (CONFIG.flightControllerIdentifier == "BTFL" && semver.lt(CONFIG.flightControllerVersion, "2.8.2")) {
            $('.hardwareSelection').hide();
        }

        $('input[name="vesselName"]').val(CONFIG.name);
        
        if (CONFIG.flightControllerIdentifier != "BTFL" || semver.lt(CONFIG.flightControllerVersion, "3.0.0")) {
            $('.miscSettings').hide();
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
    
        if (semver.lt(CONFIG.apiVersion, "1.6.0")) {
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
            gps_ubx_sbas_e.append('<option value="' + i + '">' + gpsSbas[i] + '</option>');
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

        if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
            serialRXtypes.push('IBUS');
        }

        if (CONFIG.flightControllerIdentifier == "BTFL" && semver.gte(CONFIG.flightControllerVersion, "2.6.0"))  {
            serialRXtypes.push('JETIEXBUS');
        }

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
        $('input[name="mag_declination"]').val(MISC.mag_declination.toFixed(2));

        //fill motor disarm params and FC loop time        
        if(semver.gte(CONFIG.apiVersion, "1.8.0")) {
            $('input[name="autodisarmdelay"]').val(ARMING_CONFIG.auto_disarm_delay);
            $('input[id="disarmkillswitch"]').prop('checked', ARMING_CONFIG.disarm_kill_switch !== 0);
            $('div.disarm').show();            
            if (BF_CONFIG.features.isEnabled('MOTOR_STOP')) {
                $('div.disarmdelay').show();
            } else {
                $('div.disarmdelay').hide();
            }

            $('div.cycles').show();
        }
        
        // fill throttle
        $('input[name="midrc"]').val(MISC.midrc);
        $('input[name="minthrottle"]').val(MISC.minthrottle);
        $('input[name="maxthrottle"]').val(MISC.maxthrottle);
        $('input[name="mincommand"]').val(MISC.mincommand);
        
        // fill battery
        $('input[name="mincellvoltage"]').val(MISC.vbatmincellvoltage);
        $('input[name="maxcellvoltage"]').val(MISC.vbatmaxcellvoltage);
        $('input[name="warningcellvoltage"]').val(MISC.vbatwarningcellvoltage);
        $('input[name="voltagescale"]').val(MISC.vbatscale);

        // fill current
        $('input[name="currentscale"]').val(BF_CONFIG.currentscale);
        $('input[name="currentoffset"]').val(BF_CONFIG.currentoffset);
        $('input[name="multiwiicurrentoutput"]').prop('checked', MISC.multiwiicurrentoutput !== 0);

        //fill 3D
        if (semver.lt(CONFIG.apiVersion, "1.14.0")) {
            $('.tab-configuration ._3d').hide();
        } else {
            $('input[name="3ddeadbandlow"]').val(_3D.deadband3d_low);
            $('input[name="3ddeadbandhigh"]').val(_3D.deadband3d_high);
            $('input[name="3dneutral"]').val(_3D.neutral3d);
            $('input[name="3ddeadbandthrottle"]').val(_3D.deadband3d_throttle);
        }

        // UI hooks
        $('input.feature', features_e).change(function () {
            var element = $(this);

            BF_CONFIG.features.updateData(element);
            updateTabList(BF_CONFIG.features);

            if (element.attr('name') === 'MOTOR_STOP') {
                if (BF_CONFIG.features.isEnabled('MOTOR_STOP')) {
                    $('div.disarmdelay').show();
                } else {
                    $('div.disarmdelay').hide();
                }
            }
        });

        $("input[id='unsyncedPWMSwitch']").change(function() {
            if ($(this).is(':checked')) {
                $('div.unsyncedpwmfreq').show();
            } else {
                $('div.unsyncedpwmfreq').hide();
            }
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
            if(semver.gte(CONFIG.apiVersion, "1.8.0")) {
                ARMING_CONFIG.auto_disarm_delay = parseInt($('input[name="autodisarmdelay"]').val());
                ARMING_CONFIG.disarm_kill_switch = $('input[id="disarmkillswitch"]').is(':checked') ? 1 : 0;
            }
            
            MISC.midrc = parseInt($('input[name="midrc"]').val());
            MISC.minthrottle = parseInt($('input[name="minthrottle"]').val());
            MISC.maxthrottle = parseInt($('input[name="maxthrottle"]').val());
            MISC.mincommand = parseInt($('input[name="mincommand"]').val());

            MISC.vbatmincellvoltage = parseFloat($('input[name="mincellvoltage"]').val());
            MISC.vbatmaxcellvoltage = parseFloat($('input[name="maxcellvoltage"]').val());
            MISC.vbatwarningcellvoltage = parseFloat($('input[name="warningcellvoltage"]').val());
            MISC.vbatscale = parseInt($('input[name="voltagescale"]').val());

            BF_CONFIG.currentscale = parseInt($('input[name="currentscale"]').val());
            BF_CONFIG.currentoffset = parseInt($('input[name="currentoffset"]').val());
            MISC.multiwiicurrentoutput = $('input[name="multiwiicurrentoutput"]').is(':checked') ? 1 : 0;

            if(semver.gte(CONFIG.apiVersion, "1.14.0")) {
                _3D.deadband3d_low = parseInt($('input[name="3ddeadbandlow"]').val());
                _3D.deadband3d_high = parseInt($('input[name="3ddeadbandhigh"]').val());
                _3D.neutral3d = parseInt($('input[name="3dneutral"]').val());
                _3D.deadband3d_throttle = ($('input[name="3ddeadbandthrottle"]').val());
            }

            SENSOR_ALIGNMENT.align_gyro = parseInt(orientation_gyro_e.val());
            SENSOR_ALIGNMENT.align_acc = parseInt(orientation_acc_e.val());
            SENSOR_ALIGNMENT.align_mag = parseInt(orientation_mag_e.val());
            
            PID_ADVANCED_CONFIG.fast_pwm_protocol = parseInt(esc_protocol_e.val()-1);
            PID_ADVANCED_CONFIG.use_unsyncedPwm = $('input[id="unsyncedPWMSwitch"]').is(':checked') ? 1 : 0;
            PID_ADVANCED_CONFIG.motor_pwm_rate = parseInt($('input[name="unsyncedpwmfreq"]').val());
            PID_ADVANCED_CONFIG.gyro_sync_denom = parseInt(gyro_select_e.val());
            PID_ADVANCED_CONFIG.pid_process_denom = parseInt(pid_select_e.val());

            function save_serial_config() {
                if (semver.lt(CONFIG.apiVersion, "1.6.0")) {
                    MSP.send_message(MSPCodes.MSP_SET_CF_SERIAL_CONFIG, mspHelper.crunch(MSPCodes.MSP_SET_CF_SERIAL_CONFIG), false, save_misc);
                } else {
                    save_misc();
                }
            }

            function save_misc() {
                MSP.send_message(MSPCodes.MSP_SET_MISC, mspHelper.crunch(MSPCodes.MSP_SET_MISC), false, save_3d);
            }

            function save_3d() {
                var next_callback = save_rc_deadband;
                if(semver.gte(CONFIG.apiVersion, "1.14.0")) {
                    MSP.send_message(MSPCodes.MSP_SET_3D, mspHelper.crunch(MSPCodes.MSP_SET_3D), false, next_callback);
                } else {
                    next_callback();
                }
            }

            function save_rc_deadband() {
                var next_callback = save_sensor_alignment;
                if(semver.gte(CONFIG.apiVersion, "1.17.0")) {
                    MSP.send_message(MSPCodes.MSP_SET_RC_DEADBAND, mspHelper.crunch(MSPCodes.MSP_SET_RC_DEADBAND), false, next_callback);
                } else {
                    next_callback();
                }
            }

            function save_sensor_alignment() {
                var next_callback = save_esc_protocol;
                if(semver.gte(CONFIG.apiVersion, "1.15.0")) {
                   MSP.send_message(MSPCodes.MSP_SET_SENSOR_ALIGNMENT, mspHelper.crunch(MSPCodes.MSP_SET_SENSOR_ALIGNMENT), false, next_callback);
                } else {
                   next_callback();
                }     
            }
            function save_esc_protocol() {
                var next_callback = save_acc_trim;
                if (CONFIG.flightControllerIdentifier == "BTFL" && semver.gte(CONFIG.flightControllerVersion, "2.8.1")) {
                    MSP.send_message(MSPCodes.MSP_SET_ADVANCED_CONFIG, mspHelper.crunch(MSPCodes.MSP_SET_ADVANCED_CONFIG), false, next_callback);
                } else {
                   next_callback();
                }
            }

            function save_acc_trim() {
                MSP.send_message(MSPCodes.MSP_SET_ACC_TRIM, mspHelper.crunch(MSPCodes.MSP_SET_ACC_TRIM), false
                                , semver.gte(CONFIG.apiVersion, "1.8.0") ? save_arming_config : save_to_eeprom);
            }

            function save_arming_config() {
                MSP.send_message(MSPCodes.MSP_SET_ARMING_CONFIG, mspHelper.crunch(MSPCodes.MSP_SET_ARMING_CONFIG), false, save_looptime_config);
            }

            function save_looptime_config() {
                var next_callback = save_sensor_config;
                if (CONFIG.flightControllerIdentifier == "BTFL" && semver.lt(CONFIG.flightControllerVersion, "2.8.1")) {
                    FC_CONFIG.loopTime = PID_ADVANCED_CONFIG.gyro_sync_denom * 125;
                    MSP.send_message(MSPCodes.MSP_SET_LOOP_TIME, mspHelper.crunch(MSPCodes.MSP_SET_LOOP_TIME), false, next_callback);
                } else {
                    next_callback();
                }
            }
            function save_sensor_config() {
                var next_callback = save_name;
                
                if (CONFIG.flightControllerIdentifier == "BTFL" && semver.gte(CONFIG.flightControllerVersion, "2.8.2")) {
                    SENSOR_CONFIG.acc_hardware = $('input[id="accHardwareSwitch"]').is(':checked') ? 0 : 1;
                    SENSOR_CONFIG.baro_hardware = $('input[id="baroHardwareSwitch"]').is(':checked') ? 0 : 1;
                    SENSOR_CONFIG.mag_hardware = $('input[id="magHardwareSwitch"]').is(':checked') ? 0 : 1;
                    MSP.send_message(MSPCodes.MSP_SET_SENSOR_CONFIG, mspHelper.crunch(MSPCodes.MSP_SET_SENSOR_CONFIG), false, next_callback);
                } else {
                    next_callback();
                }
            }

            function save_name() {
                var next_callback = save_to_eeprom;
                
                if (CONFIG.flightControllerIdentifier == "BTFL" && semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
                    CONFIG.name = $.trim($('input[name="vesselName"]').val());
                    MSP.send_message(MSPCodes.MSP_SET_NAME, mspHelper.crunch(MSPCodes.MSP_SET_NAME), false, next_callback);
                } else {
                    next_callback();
                }
            }

            function save_to_eeprom() {
                MSP.send_message(MSPCodes.MSP_EEPROM_WRITE, false, false, reboot);
            }

            function reboot() {
                GUI.log(chrome.i18n.getMessage('configurationEepromSaved'));

                GUI.tab_switch_cleanup(function() {
                    MSP.send_message(MSPCodes.MSP_SET_REBOOT, false, false, reinitialize);
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
                        MSP.send_message(MSPCodes.MSP_STATUS, false, false, function() {
                            GUI.log(chrome.i18n.getMessage('deviceReady'));
                            TABS.configuration.initialize(false, $('#content').scrollTop());
                        });
                    },1500); // 1500 ms seems to be just the right amount of delay to prevent data request timeouts
                }
            }

            MSP.send_message(MSPCodes.MSP_SET_BF_CONFIG, mspHelper.crunch(MSPCodes.MSP_SET_BF_CONFIG), false, save_serial_config);
            
        });

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSPCodes.MSP_STATUS);
        }, 250, true);
        GUI.interval_add('config_load_analog', load_analog, 250, true); // 4 fps
        GUI.content_ready(callback);
    }
};

TABS.configuration.cleanup = function (callback) {
    if (callback) callback();
};
