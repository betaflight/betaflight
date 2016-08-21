'use strict';


function MspHelper () {
}

MspHelper.prototype.process_data = function(dataHandler) {
    var self = this;
    var data = dataHandler.dataView; // DataView (allowing us to view arrayBuffer as struct/union)
    var code = dataHandler.code; 
    if (!dataHandler.unsupported) switch (code) {
        case MSPCodes.MSP_STATUS:
            CONFIG.cycleTime = data.readU16();
            CONFIG.i2cError = data.readU16();
            CONFIG.activeSensors = data.readU16();
            CONFIG.mode = data.readU32();
            CONFIG.profile = data.readU8();

            TABS.pid_tuning.checkUpdateProfile(false);

            sensor_status(CONFIG.activeSensors);
            $('span.i2c-error').text(CONFIG.i2cError);
            $('span.cycle-time').text(CONFIG.cycleTime);
            break;
        case MSPCodes.MSP_STATUS_EX:
            CONFIG.cycleTime = data.readU16();
            CONFIG.i2cError = data.readU16();
            CONFIG.activeSensors = data.readU16();
            CONFIG.mode = data.readU32();
            CONFIG.profile = data.readU8();
            CONFIG.cpuload = data.readU16();
            if (semver.gt(CONFIG.flightControllerVersion, "2.9.1")) {
                CONFIG.numProfiles = data.readU8();
                CONFIG.rateProfile = data.readU8();

                TABS.pid_tuning.checkUpdateProfile(true);
            }

            sensor_status(CONFIG.activeSensors);
            $('span.i2c-error').text(CONFIG.i2cError);
            $('span.cycle-time').text(CONFIG.cycleTime);
            $('span.cpu-load').text(chrome.i18n.getMessage('statusbar_cpu_load', [CONFIG.cpuload]));
            break;
        
        case MSPCodes.MSP_RAW_IMU:
            // 512 for mpu6050, 256 for mma
            // currently we are unable to differentiate between the sensor types, so we are goign with 512
            SENSOR_DATA.accelerometer[0] = data.read16() / 512;
            SENSOR_DATA.accelerometer[1] = data.read16() / 512;
            SENSOR_DATA.accelerometer[2] = data.read16() / 512;

            // properly scaled
            SENSOR_DATA.gyroscope[0] = data.read16() * (4 / 16.4);
            SENSOR_DATA.gyroscope[1] = data.read16() * (4 / 16.4);
            SENSOR_DATA.gyroscope[2] = data.read16() * (4 / 16.4);

            // no clue about scaling factor
            SENSOR_DATA.magnetometer[0] = data.read16() / 1090;
            SENSOR_DATA.magnetometer[1] = data.read16() / 1090;
            SENSOR_DATA.magnetometer[2] = data.read16() / 1090;
            break;
        case MSPCodes.MSP_SERVO:
            var servoCount = data.byteLength / 2;
            for (var i = 0; i < servoCount; i++) {
                SERVO_DATA[i] = data.readU16();
            }
            break;
        case MSPCodes.MSP_MOTOR:
            var motorCount = data.byteLength / 2;
            for (var i = 0; i < motorCount; i++) {
                MOTOR_DATA[i] = data.readU16();
            }
            break;
        case MSPCodes.MSP_RC:
            RC.active_channels = data.byteLength / 2;
            for (var i = 0; i < RC.active_channels; i++) {
                RC.channels[i] = data.readU16();
            }
            break;
        case MSPCodes.MSP_RAW_GPS:
            GPS_DATA.fix = data.readU8();
            GPS_DATA.numSat = data.readU8();
            GPS_DATA.lat = data.read32();
            GPS_DATA.lon = data.read32();
            GPS_DATA.alt = data.readU16();
            GPS_DATA.speed = data.readU16();
            GPS_DATA.ground_course = data.readU16();
            break;
        case MSPCodes.MSP_COMP_GPS:
            GPS_DATA.distanceToHome = data.readU16();
            GPS_DATA.directionToHome = data.readU16();
            GPS_DATA.update = data.readU8();
            break;
        case MSPCodes.MSP_ATTITUDE:
            SENSOR_DATA.kinematics[0] = data.read16() / 10.0; // x
            SENSOR_DATA.kinematics[1] = data.read16() / 10.0; // y
            SENSOR_DATA.kinematics[2] = data.read16(); // z
            break;
        case MSPCodes.MSP_ALTITUDE:
            SENSOR_DATA.altitude = parseFloat((data.read32() / 100.0).toFixed(2)); // correct scale factor
            break;
        case MSPCodes.MSP_SONAR:
            SENSOR_DATA.sonar = data.read32();
            break;
        case MSPCodes.MSP_ANALOG:
            ANALOG.voltage = data.readU8() / 10.0;
            ANALOG.mAhdrawn = data.readU16();
            ANALOG.rssi = data.readU16(); // 0-1023
            ANALOG.amperage = data.read16() / 100; // A
            ANALOG.last_received_timestamp = Date.now();
            break;
        case MSPCodes.MSP_RC_TUNING:
            RC_tuning.RC_RATE = parseFloat((data.readU8() / 100).toFixed(2));
            RC_tuning.RC_EXPO = parseFloat((data.readU8() / 100).toFixed(2));
            if (semver.lt(CONFIG.apiVersion, "1.7.0")) {
                RC_tuning.roll_pitch_rate = parseFloat((data.readU8() / 100).toFixed(2));
                RC_tuning.pitch_rate = 0;
                RC_tuning.roll_rate = 0;
            } else {
                RC_tuning.roll_pitch_rate = 0;
                RC_tuning.roll_rate = parseFloat((data.readU8() / 100).toFixed(2));
                RC_tuning.pitch_rate = parseFloat((data.readU8() / 100).toFixed(2));
            }
            RC_tuning.yaw_rate = parseFloat((data.readU8() / 100).toFixed(2));
            RC_tuning.dynamic_THR_PID = parseFloat((data.readU8() / 100).toFixed(2));
            RC_tuning.throttle_MID = parseFloat((data.readU8() / 100).toFixed(2));
            RC_tuning.throttle_EXPO = parseFloat((data.readU8() / 100).toFixed(2));
            if (semver.gte(CONFIG.apiVersion, "1.7.0")) {
                RC_tuning.dynamic_THR_breakpoint = data.readU16();
            } else {
                RC_tuning.dynamic_THR_breakpoint = 0;
            }
            if (semver.gte(CONFIG.apiVersion, "1.10.0")) {
                RC_tuning.RC_YAW_EXPO = parseFloat((data.readU8() / 100).toFixed(2));
                if (semver.gte(CONFIG.flightControllerVersion, "2.9.1")) {
                    RC_tuning.rcYawRate = parseFloat((data.readU8() / 100).toFixed(2));
                } else if (semver.lt(CONFIG.flightControllerVersion, "2.9.0")) {
                    RC_tuning.rcYawRate = 0;
                }
            } else {
                RC_tuning.RC_YAW_EXPO = 0;
            }
            break;
        case MSPCodes.MSP_PID:
            // PID data arrived, we need to scale it and save to appropriate bank / array
            for (var i = 0, needle = 0; i < (data.byteLength / 3); i++, needle += 3) {
                // main for loop selecting the pid section
                for (var j = 0; j < 3; j++) {
                    PIDs[i][j] = data.readU8();
                }
            }
            break;

        case MSPCodes.MSP_ARMING_CONFIG:
            if (semver.gte(CONFIG.apiVersion, "1.8.0")) {
                ARMING_CONFIG.auto_disarm_delay = data.readU8();
                ARMING_CONFIG.disarm_kill_switch = data.readU8();
            }
            break;
        case MSPCodes.MSP_LOOP_TIME:
            if (semver.gte(CONFIG.apiVersion, "1.8.0")) {
                FC_CONFIG.loopTime = data.readU16();
            }
            break;
        case MSPCodes.MSP_MISC: // 22 bytes
            MISC.midrc = data.readU16();
            MISC.minthrottle = data.readU16(); // 0-2000
            MISC.maxthrottle = data.readU16(); // 0-2000
            MISC.mincommand = data.readU16(); // 0-2000
            MISC.failsafe_throttle = data.readU16(); // 1000-2000
            MISC.gps_type = data.readU8();
            MISC.gps_baudrate = data.readU8();
            MISC.gps_ubx_sbas = data.readU8();
            MISC.multiwiicurrentoutput = data.readU8();
            MISC.rssi_channel = data.readU8();
            MISC.placeholder2 = data.readU8();                
            if (semver.lt(CONFIG.apiVersion, "1.18.0"))
                MISC.mag_declination = data.read16() / 10; // -1800-1800
            else
                MISC.mag_declination = data.read16() / 100; // -18000-18000                
            MISC.vbatscale = data.readU8(); // 10-200
            MISC.vbatmincellvoltage = data.readU8() / 10; // 10-50
            MISC.vbatmaxcellvoltage = data.readU8() / 10; // 10-50
            MISC.vbatwarningcellvoltage = data.readU8() / 10; // 10-50
            break;
        case MSPCodes.MSP_3D:
            _3D.deadband3d_low = data.readU16();
            _3D.deadband3d_high = data.readU16();
            _3D.neutral3d = data.readU16();
            if (semver.lt(CONFIG.apiVersion, "1.17.0")) {
                _3D.deadband3d_throttle = data.readU16();
            }
            break;
        case MSPCodes.MSP_BOXNAMES:
            AUX_CONFIG = []; // empty the array as new data is coming in

            var buff = [];
            for (var i = 0; i < data.byteLength; i++) {
                var char = data.readU8();
                if (char == 0x3B) { // ; (delimeter char)
                    AUX_CONFIG.push(String.fromCharCode.apply(null, buff)); // convert bytes into ASCII and save as strings

                    // empty buffer
                    buff = [];
                } else {
                    buff.push(char);
                }
            }
            break;
        case MSPCodes.MSP_PIDNAMES:
            PID_names = []; // empty the array as new data is coming in

            var buff = [];
            for (var i = 0; i < data.byteLength; i++) {
                var char = data.readU8();
                if (char == 0x3B) { // ; (delimeter char)
                    PID_names.push(String.fromCharCode.apply(null, buff)); // convert bytes into ASCII and save as strings

                    // empty buffer
                    buff = [];
                } else {
                    buff.push(char);
                }
            }
            break;
        case MSPCodes.MSP_BOXIDS:
            AUX_CONFIG_IDS = []; // empty the array as new data is coming in

            for (var i = 0; i < data.byteLength; i++) {
                AUX_CONFIG_IDS.push(data.readU8());
            }
            break;
        case MSPCodes.MSP_SERVO_MIX_RULES:
            break;

        case MSPCodes.MSP_SERVO_CONFIGURATIONS:
            SERVO_CONFIG = []; // empty the array as new data is coming in

            if (semver.gte(CONFIG.apiVersion, "1.12.0")) {
                if (data.byteLength % 14 == 0) {
                    for (var i = 0; i < data.byteLength; i += 14) {
                        var arr = {
                            'min':                      data.readU16(),
                            'max':                      data.readU16(),
                            'middle':                   data.readU16(),
                            'rate':                     data.read8(),
                            'angleAtMin':               data.readU8(),
                            'angleAtMax':               data.readU8(),
                            'indexOfChannelToForward':  data.readU8(),
                            'reversedInputSources':     data.readU32()
                        };

                        SERVO_CONFIG.push(arr);
                    }
                }
            } else {
                if (data.byteLength % 7 == 0) {
                    for (var i = 0; i < data.byteLength; i += 7) {
                        var arr = {
                            'min':                      data.readU16(),
                            'max':                      data.readU16(),
                            'middle':                   data.readU16(),
                            'rate':                     data.read8(),
                            'angleAtMin':               45,
                            'angleAtMax':               45,
                            'indexOfChannelToForward':  undefined,
                            'reversedInputSources':     0
                        };

                        SERVO_CONFIG.push(arr);
                    }
                }
                
                if (semver.eq(CONFIG.apiVersion, '1.10.0')) {
                    // drop two unused servo configurations due to MSP rx buffer to small)
                    while (SERVO_CONFIG.length > 8) {
                        SERVO_CONFIG.pop();
                    } 
                }
            }
            break;
        case MSPCodes.MSP_RC_DEADBAND:
            RC_deadband.deadband = data.readU8();
            RC_deadband.yaw_deadband = data.readU8();
            RC_deadband.alt_hold_deadband = data.readU8();
            
            if (semver.gte(CONFIG.apiVersion, "1.17.0")) {
                _3D.deadband3d_throttle = data.readU16();
            }
            break;
        case MSPCodes.MSP_SENSOR_ALIGNMENT:
            SENSOR_ALIGNMENT.align_gyro = data.readU8();
            SENSOR_ALIGNMENT.align_acc = data.readU8();
            SENSOR_ALIGNMENT.align_mag = data.readU8();
            break;
        case MSPCodes.MSP_SET_RAW_RC:
            break;
        case MSPCodes.MSP_SET_PID:
            console.log('PID settings saved');
            break;
        case MSPCodes.MSP_SET_RC_TUNING:
            console.log('RC Tuning saved');
            break;
        case MSPCodes.MSP_ACC_CALIBRATION:
            console.log('Accel calibration executed');
            break;
        case MSPCodes.MSP_MAG_CALIBRATION:
            console.log('Mag calibration executed');
            break;
        case MSPCodes.MSP_SET_MISC:
            console.log('MISC Configuration saved');
            break;
        case MSPCodes.MSP_RESET_CONF:
            console.log('Settings Reset');
            break;
        case MSPCodes.MSP_SELECT_SETTING:
            console.log('Profile selected');
            break;
        case MSPCodes.MSP_SET_SERVO_CONFIGURATION:
            console.log('Servo Configuration saved');
            break;
        case MSPCodes.MSP_EEPROM_WRITE:
            console.log('Settings Saved in EEPROM');
            break;
        case MSPCodes.MSP_DEBUG:
            for (var i = 0; i < 4; i++)
                SENSOR_DATA.debug[i] = data.read16();
            break;
        case MSPCodes.MSP_SET_MOTOR:
            console.log('Motor Speeds Updated');
            break;
        case MSPCodes.MSP_UID:
            CONFIG.uid[0] = data.readU32();
            CONFIG.uid[1] = data.readU32();
            CONFIG.uid[2] = data.readU32();
            break;
        case MSPCodes.MSP_ACC_TRIM:
            CONFIG.accelerometerTrims[0] = data.read16(); // pitch
            CONFIG.accelerometerTrims[1] = data.read16(); // roll
            break;
        case MSPCodes.MSP_SET_ACC_TRIM:
            console.log('Accelerometer trimms saved.');
            break;
        case MSPCodes.MSP_GPS_SV_INFO:
            if (data.byteLength > 0) {
                var numCh = data.readU8();

                for (var i = 0; i < numCh; i++) {
                    GPS_DATA.chn[i] = data.readU8();
                    GPS_DATA.svid[i] = data.readU8();
                    GPS_DATA.quality[i] = data.readU8();
                    GPS_DATA.cno[i] = data.readU8();
                }
            }
            break;

        case MSPCodes.MSP_RX_MAP:
            RC_MAP = []; // empty the array as new data is coming in

            for (var i = 0; i < data.byteLength; i++) {
                RC_MAP.push(data.readU8());
            }
            break;
        case MSPCodes.MSP_SET_RX_MAP:
            console.log('RCMAP saved');
            break;
        case MSPCodes.MSP_BF_CONFIG:
            BF_CONFIG.mixerConfiguration = data.readU8();
            BF_CONFIG.features.setMask(data.readU32());
            BF_CONFIG.serialrx_type = data.readU8();
            BF_CONFIG.board_align_roll = data.read16(); // -180 - 360
            BF_CONFIG.board_align_pitch = data.read16(); // -180 - 360
            BF_CONFIG.board_align_yaw = data.read16(); // -180 - 360
            BF_CONFIG.currentscale = data.read16();
            BF_CONFIG.currentoffset = data.readU16();

            updateTabList(BF_CONFIG.features);

            break;
        case MSPCodes.MSP_SET_BF_CONFIG:
            break;
        case MSPCodes.MSP_SET_REBOOT:
            console.log('Reboot request accepted');
            break;

        case MSPCodes.MSP_API_VERSION:
            CONFIG.mspProtocolVersion = data.readU8(); 
            CONFIG.apiVersion = data.readU8() + '.' + data.readU8() + '.0';
            break;

        case MSPCodes.MSP_FC_VARIANT:
            var identifier = '';
            for (var i = 0; i < 4; i++) {
                identifier += String.fromCharCode(data.readU8());
            }
            CONFIG.flightControllerIdentifier = identifier;
            break;

        case MSPCodes.MSP_FC_VERSION:
            CONFIG.flightControllerVersion = data.readU8() + '.' + data.readU8() + '.' + data.readU8();
            break;

        case MSPCodes.MSP_BUILD_INFO:
            var dateLength = 11;
            var buff = [];
            for (var i = 0; i < dateLength; i++) {
                buff.push(data.readU8());
            }
            buff.push(32); // ascii space
            
            var timeLength = 8;
            for (var i = 0; i < timeLength; i++) {
                buff.push(data.readU8());
            }
            CONFIG.buildInfo = String.fromCharCode.apply(null, buff);
            break;

        case MSPCodes.MSP_BOARD_INFO:
            var identifier = '';
            for (var i = 0; i < 4; i++) {
                identifier += String.fromCharCode(data.readU8());
            }
            CONFIG.boardIdentifier = identifier;
            CONFIG.boardVersion = data.readU16();
            break;

        case MSPCodes.MSP_NAME:
            CONFIG.name = '';
            var char;
            while ((char = data.readU8()) !== null) {
                CONFIG.name += String.fromCharCode(char);
            }
            break;

        case MSPCodes.MSP_SET_CHANNEL_FORWARDING:
            console.log('Channel forwarding saved');
            break;

        case MSPCodes.MSP_CF_SERIAL_CONFIG:
            var supportedBaudRates = [ // 0 based index.
                                       'AUTO',
                                       '9600',
                                       '19200',
                                       '38400',
                                       '57600',
                                       '115200',
                                       '230400',
                                       '250000',
                                   ];
            if (semver.lt(CONFIG.apiVersion, "1.6.0")) {
                SERIAL_CONFIG.ports = [];
                var serialPortCount = (data.byteLength - (4 * 4)) / 2;
                for (var i = 0; i < serialPortCount; i++) {
                    var serialPort = {
                        identifier: data.readU8(),
                        scenario: data.readU8()
                    }
                    SERIAL_CONFIG.ports.push(serialPort); 
                }
                SERIAL_CONFIG.mspBaudRate = data.readU32();
                SERIAL_CONFIG.cliBaudRate = data.readU32();
                SERIAL_CONFIG.gpsBaudRate = data.readU32();
                SERIAL_CONFIG.gpsPassthroughBaudRate = data.readU32();
            } else {
                SERIAL_CONFIG.ports = [];
                var bytesPerPort = 1 + 2 + (1 * 4);
                var serialPortCount = data.byteLength / bytesPerPort;
                
                for (var i = 0; i < serialPortCount; i++) {
                    var serialPort = {
                        identifier: data.readU8(),
                        functions: self.serialPortFunctionMaskToFunctions(data.readU16()),
                        msp_baudrate: supportedBaudRates[data.readU8()],
                        gps_baudrate: supportedBaudRates[data.readU8()],
                        telemetry_baudrate: supportedBaudRates[data.readU8()],
                        blackbox_baudrate: supportedBaudRates[data.readU8()]
                    }
                    
                    SERIAL_CONFIG.ports.push(serialPort);
                }
            }
            break;

        case MSPCodes.MSP_SET_CF_SERIAL_CONFIG:
            console.log('Serial config saved');
            break;

        case MSPCodes.MSP_MODE_RANGES:
            MODE_RANGES = []; // empty the array as new data is coming in

            var modeRangeCount = data.byteLength / 4; // 4 bytes per item.
            
            for (var i = 0; i < modeRangeCount; i++) {
                var modeRange = {
                    id: data.readU8(),
                    auxChannelIndex: data.readU8(),
                    range: {
                        start: 900 + (data.readU8() * 25),
                        end: 900 + (data.readU8() * 25)
                    }
                };
                MODE_RANGES.push(modeRange);
            }
            break;

        case MSPCodes.MSP_ADJUSTMENT_RANGES:
            ADJUSTMENT_RANGES = []; // empty the array as new data is coming in

            var adjustmentRangeCount = data.byteLength / 6; // 6 bytes per item.
            
            for (var i = 0; i < adjustmentRangeCount; i++) {
                var adjustmentRange = {
                    slotIndex: data.readU8(),
                    auxChannelIndex: data.readU8(),
                    range: {
                        start: 900 + (data.readU8() * 25),
                        end: 900 + (data.readU8() * 25)
                    },
                    adjustmentFunction: data.readU8(),
                    auxSwitchChannelIndex: data.readU8()
                };
                ADJUSTMENT_RANGES.push(adjustmentRange);
            }
            break;

        case MSPCodes.MSP_CHANNEL_FORWARDING:
            for (var i = 0; i < data.byteLength && i < SERVO_CONFIG.length; i ++) {
                var channelIndex = data.readU8();
                if (channelIndex < 255) {
                    SERVO_CONFIG[i].indexOfChannelToForward = channelIndex;
                } else {
                    SERVO_CONFIG[i].indexOfChannelToForward = undefined;
                }
            }
            break;

        case MSPCodes.MSP_RX_CONFIG:
            RX_CONFIG.serialrx_provider = data.readU8();
            RX_CONFIG.maxcheck = data.readU16();
            RX_CONFIG.midrc = data.readU16();
            RX_CONFIG.mincheck = data.readU16();
            RX_CONFIG.spektrum_sat_bind = data.readU8();
            RX_CONFIG.rx_min_usec = data.readU16();
            RX_CONFIG.rx_max_usec = data.readU16();
            if (semver.gte(CONFIG.apiVersion, "1.20.0")) {
                RX_CONFIG.rcInterpolation = data.readU8();
                RX_CONFIG.rcInterpolationInterval = data.readU8();
                RX_CONFIG.airModeActivateThreshold = data.readU16();
            } else {
                RX_CONFIG.rcInterpolation = 0;
                RX_CONFIG.rcInterpolationInterval = 0;
                RX_CONFIG.airModeActivateThreshold = 0;
            }
            break;

        case MSPCodes.MSP_FAILSAFE_CONFIG:
            FAILSAFE_CONFIG.failsafe_delay = data.readU8();
            FAILSAFE_CONFIG.failsafe_off_delay = data.readU8();
            FAILSAFE_CONFIG.failsafe_throttle = data.readU16();
            if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
                FAILSAFE_CONFIG.failsafe_kill_switch = data.readU8();
                FAILSAFE_CONFIG.failsafe_throttle_low_delay = data.readU16();
                FAILSAFE_CONFIG.failsafe_procedure = data.readU8();
            }
            break;

        case MSPCodes.MSP_RXFAIL_CONFIG:
            RXFAIL_CONFIG = []; // empty the array as new data is coming in

            var channelCount = data.byteLength / 3;
            for (var i = 0; i < channelCount; i++) {
                var rxfailChannel = {
                    mode:  data.readU8(),
                    value: data.readU16()
                };
                RXFAIL_CONFIG.push(rxfailChannel);
            }
            break;

        case MSPCodes.MSP_ADVANCED_CONFIG:
            PID_ADVANCED_CONFIG.gyro_sync_denom = data.readU8();
            PID_ADVANCED_CONFIG.pid_process_denom = data.readU8();
            PID_ADVANCED_CONFIG.use_unsyncedPwm = data.readU8();
            PID_ADVANCED_CONFIG.fast_pwm_protocol = data.readU8();
            PID_ADVANCED_CONFIG.motor_pwm_rate = data.readU16();
            break;
        case MSPCodes.MSP_FILTER_CONFIG:
            FILTER_CONFIG.gyro_soft_lpf_hz = data.readU8();
            FILTER_CONFIG.dterm_lpf_hz = data.readU16();
            FILTER_CONFIG.yaw_lpf_hz = data.readU16();
            if (semver.gte(CONFIG.apiVersion, "1.20.0")) {
                FILTER_CONFIG.gyro_soft_notch_hz = data.readU16();
                FILTER_CONFIG.gyro_soft_notch_cutoff = data.readU16();
                FILTER_CONFIG.dterm_notch_hz = data.readU16();
                FILTER_CONFIG.dterm_notch_cutoff = data.readU16();
            }
            break;

        case MSPCodes.MSP_SET_PID_ADVANCED:
            console.log("Advanced PID settings saved");
            break;
        case MSPCodes.MSP_PID_ADVANCED:
            ADVANCED_TUNING.rollPitchItermIgnoreRate = data.readU16();
            ADVANCED_TUNING.yawItermIgnoreRate = data.readU16();
            ADVANCED_TUNING.yaw_p_limit = data.readU16();
            ADVANCED_TUNING.deltaMethod = data.readU8();
            ADVANCED_TUNING.vbatPidCompensation = data.readU8();
            if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
                ADVANCED_TUNING.ptermSetpointWeight = data.readU8();
                ADVANCED_TUNING.dtermSetpointWeight = data.readU8();
                ADVANCED_TUNING.toleranceBand = data.readU8();
                ADVANCED_TUNING.toleranceBandReduction = data.readU8();
                ADVANCED_TUNING.itermThrottleGain = data.readU8();
                ADVANCED_TUNING.pidMaxVelocity = data.readU16();
                ADVANCED_TUNING.pidMaxVelocityYaw = data.readU16();
            }
            break;
        case MSPCodes.MSP_SPECIAL_PARAMETERS:
            if (semver.lt(CONFIG.flightControllerVersion, "2.9.1")) {
                if (semver.gte(CONFIG.flightControllerVersion, "2.8.0")) {
                    RC_tuning.rcYawRate = parseFloat((data.readU8() / 100).toFixed(2));
                    if (semver.gte(CONFIG.flightControllerVersion, "2.8.2")) {
                        RX_CONFIG.airModeActivateThreshold = data.readU16();
                        RX_CONFIG.rcSmoothInterval = data.readU8()
                        SPECIAL_PARAMETERS.escDesyncProtection = data.readU16();
                    }
                }
            }
            break;
        case MSPCodes.MSP_SENSOR_CONFIG:
            SENSOR_CONFIG.acc_hardware = data.readU8();
            SENSOR_CONFIG.baro_hardware = data.readU8();
            SENSOR_CONFIG.mag_hardware = data.readU8();
            break;
            
        case MSPCodes.MSP_LED_STRIP_CONFIG:
            LED_STRIP = [];
            
            var ledDirectionLetters =       ['n', 'e', 's', 'w', 'u', 'd'];      // in LSB bit order
            var ledFunctionLetters =        ['i', 'w', 'f', 'a', 't', 'r', 'c', 'g', 's', 'b', 'l']; // in LSB bit order
            var ledBaseFunctionLetters =    ['c', 'f', 'a', 'l', 's', 'g', 'r']; // in LSB bit 
            var ledOverlayLetters =         ['t', 'o', 'b', 'n', 'i', 'w']; // in LSB bit 

            
            var ledCount = data.byteLength / 7; // v1.4.0 and below incorrectly reported 4 bytes per led.
            if (semver.gte(CONFIG.apiVersion, "1.20.0"))
                ledCount = data.byteLength / 4;
                
            for (var i = 0; i < ledCount; i++) {

                if (semver.lt(CONFIG.apiVersion, "1.20.0")) {
                    var directionMask = data.readU16();
                    
                    var directions = [];
                    for (var directionLetterIndex = 0; directionLetterIndex < ledDirectionLetters.length; directionLetterIndex++) {
                        if (bit_check(directionMask, directionLetterIndex)) {
                            directions.push(ledDirectionLetters[directionLetterIndex]);
                        }
                    }

                    var functionMask = data.readU16();
                    
                    var functions = [];
                    for (var functionLetterIndex = 0; functionLetterIndex < ledFunctionLetters.length; functionLetterIndex++) {
                        if (bit_check(functionMask, functionLetterIndex)) {
                            functions.push(ledFunctionLetters[functionLetterIndex]);
                        }
                    }
                    
                    var led = {
                        directions: directions,
                        functions: functions,
                        x: data.readU8(),
                        y: data.readU8(),
                        color: data.readU8()
                    };
                    
                    LED_STRIP.push(led);
                } else {
                    var mask = data.readU32();
                    
                    var functionId = (mask >> 8) & 0xF;
                    var functions = [];
                    for (var baseFunctionLetterIndex = 0; baseFunctionLetterIndex < ledBaseFunctionLetters.length; baseFunctionLetterIndex++) {
                        if (functionId == baseFunctionLetterIndex) {
                            functions.push(ledBaseFunctionLetters[baseFunctionLetterIndex]);
                            break;
                        }
                    }
                    
                    var overlayMask = (mask >> 12) & 0x3F;
                    for (var overlayLetterIndex = 0; overlayLetterIndex < ledOverlayLetters.length; overlayLetterIndex++) {
                        if (bit_check(overlayMask, overlayLetterIndex)) {
                            functions.push(ledOverlayLetters[overlayLetterIndex]);
                        }
                    }
                    
                    var directionMask = (mask >> 22) & 0x3F;
                    var directions = [];
                    for (var directionLetterIndex = 0; directionLetterIndex < ledDirectionLetters.length; directionLetterIndex++) {
                        if (bit_check(directionMask, directionLetterIndex)) {
                            directions.push(ledDirectionLetters[directionLetterIndex]);
                        }
                    }
                    var led = {
                        y: (mask) & 0xF,
                        x: (mask >> 4) & 0xF,
                        functions: functions,
                        color: (mask >> 18) & 0xF,
                        directions: directions,
                        parameters: (mask >> 28) & 0xF
                    };
                    
                    LED_STRIP.push(led);
                }
            }
            break;
        case MSPCodes.MSP_SET_LED_STRIP_CONFIG:
            console.log('Led strip config saved');
            break;
        case MSPCodes.MSP_LED_COLORS:
            
            LED_COLORS = [];
            
            var colorCount = data.byteLength / 4;
            
            for (var i = 0; i < colorCount; i++) {

                var color = {
                    h: data.readU16(),
                    s: data.readU8(),
                    v: data.readU8()
                };
                LED_COLORS.push(color);
            }
            
            break;
        case MSPCodes.MSP_SET_LED_COLORS:
            console.log('Led strip colors saved');
            break;
        case MSPCodes.MSP_LED_STRIP_MODECOLOR:
            if (semver.gte(CONFIG.apiVersion, "1.19.0")) {

                LED_MODE_COLORS = [];
                
                var colorCount = data.byteLength / 3;
                
                for (var i = 0; i < colorCount; i++) {

                    var mode_color = {
                        mode: data.readU8(),
                        direction: data.readU8(),
                        color: data.readU8()
                    };
                    LED_MODE_COLORS.push(mode_color);
                }
            }
            break;
        case MSPCodes.MSP_SET_LED_STRIP_MODECOLOR:
            console.log('Led strip mode colors saved');
            break;

        case MSPCodes.MSP_DATAFLASH_SUMMARY:
            if (data.byteLength >= 13) {
                var flags = data.readU8();
                DATAFLASH.ready = (flags & 1) != 0;
                DATAFLASH.supported = (flags & 2) != 0 || DATAFLASH.ready;
                DATAFLASH.sectors = data.readU32();
                DATAFLASH.totalSize = data.readU32();
                DATAFLASH.usedSize = data.readU32();
            } else {
                // Firmware version too old to support MSP_DATAFLASH_SUMMARY
                DATAFLASH.ready = false;
                DATAFLASH.supported = false;
                DATAFLASH.sectors = 0;
                DATAFLASH.totalSize = 0;
                DATAFLASH.usedSize = 0;
            }
            update_dataflash_global();
            break;
        case MSPCodes.MSP_DATAFLASH_READ:
            // No-op, let callback handle it
            break;
        case MSPCodes.MSP_DATAFLASH_ERASE:
            console.log("Data flash erase begun...");
            break;
        case MSPCodes.MSP_SDCARD_SUMMARY:
            var flags = data.readU8(); 
            
            SDCARD.supported = (flags & 0x01) != 0;
            SDCARD.state = data.readU8();
            SDCARD.filesystemLastError = data.readU8();
            SDCARD.freeSizeKB = data.readU32();
            SDCARD.totalSizeKB = data.readU32();
            break;
        case MSPCodes.MSP_BLACKBOX_CONFIG:
            BLACKBOX.supported = (data.readU8() & 1) != 0;
            BLACKBOX.blackboxDevice = data.readU8();
            BLACKBOX.blackboxRateNum = data.readU8();
            BLACKBOX.blackboxRateDenom = data.readU8();
            break;
        case MSPCodes.MSP_SET_BLACKBOX_CONFIG:
            console.log("Blackbox config saved");
            break;
        case MSPCodes.MSP_TRANSPONDER_CONFIG:
            TRANSPONDER.supported = (data.readU8() & 1) != 0;
            TRANSPONDER.data = [];
            var bytesRemaining = data.byteLength - 1; 
            for (var i = 0; i < bytesRemaining; i++) {
                TRANSPONDER.data.push(data.readU8());
            }
            break;
        case MSPCodes.MSP_SET_TRANSPONDER_CONFIG:
            console.log("Transponder config saved");
            break;
        case MSPCodes.MSP_SET_MODE_RANGE:
            console.log('Mode range saved');
            break;
        case MSPCodes.MSP_SET_ADJUSTMENT_RANGE:
            console.log('Adjustment range saved');
            break;
            
        case MSPCodes.MSP_PID_CONTROLLER:
            PID.controller = data.readU8();
            break;
        case MSPCodes.MSP_SET_PID_CONTROLLER:
            console.log('PID controller changed');
            break;
        case MSPCodes.MSP_SET_LOOP_TIME:
            console.log('Looptime saved');
            break;
        case MSPCodes.MSP_SET_ARMING_CONFIG:
            console.log('Arming config saved');
            break;
        case MSPCodes.MSP_SET_RESET_CURR_PID:
            console.log('Current PID profile reset');
            break;
        case MSPCodes.MSP_SET_3D:
            console.log('3D settings saved');
            break;
        case MSPCodes.MSP_SET_RC_DEADBAND:
            console.log('Rc controls settings saved');
            break;
        case MSPCodes.MSP_SET_SENSOR_ALIGNMENT:
            console.log('Sensor alignment saved');
            break; 
        case MSPCodes.MSP_SET_RX_CONFIG:
            console.log('Rx config saved');
            break;
        case MSPCodes.MSP_SET_RXFAIL_CONFIG:
            console.log('Rxfail config saved');
            break;
        case MSPCodes.MSP_SET_FAILSAFE_CONFIG:
            console.log('Failsafe config saved');
            break;
        case MSPCodes.MSP_OSD_CONFIG:
            break;
        case MSPCodes.MSP_SET_OSD_CONFIG:
            console.log('OSD config set');
            break;
        case MSPCodes.MSP_OSD_CHAR_READ:
            break;
        case MSPCodes.MSP_OSD_CHAR_WRITE:
            console.log('OSD char uploaded');
            break;
        case MSPCodes.MSP_VTX_CONFIG:
            break;
        case MSPCodes.MSP_SET_VTX_CONFIG:
            break;
        case MSPCodes.MSP_SET_NAME:
            console.log('Name set');
            break;
        case MSPCodes.MSP_SET_FILTER_CONFIG:
            console.log('Filter config set');
            break;
        case MSPCodes.MSP_SET_ADVANCED_TUNING:
            console.log('Advanced tuning parameters set');
            break;
        case MSPCodes.MSP_SET_SPECIAL_PARAMETERS:
            console.log('Special parameters set');
            break;
        default:
            console.log('Unknown code detected: ' + code);
    } else {
        console.log('FC reports unsupported message error: ' + code);
    }

    // trigger callbacks, cleanup/remove callback after trigger
    for (var i = dataHandler.callbacks.length - 1; i >= 0; i--) { // itterating in reverse because we use .splice which modifies array length
        if (dataHandler.callbacks[i].code == code) {
            // save callback reference
            var callback = dataHandler.callbacks[i].callback;

            // remove timeout
            clearInterval(dataHandler.callbacks[i].timer);

            // remove object from array
            dataHandler.callbacks.splice(i, 1);

            // fire callback
            if (callback) callback({'command': code, 'data': data, 'length': data.byteLength});
        }
    }
}


/**
 * Encode the request body for the MSP request with the given code and return it as an array of bytes.
 */
MspHelper.prototype.crunch = function(code) {
    var buffer = [];
    var self = this;
    switch (code) {
        case MSPCodes.MSP_SET_BF_CONFIG:
            var featureMask = BF_CONFIG.features.getMask();
            buffer.push8(BF_CONFIG.mixerConfiguration)
                .push32(featureMask)
                .push8(BF_CONFIG.serialrx_type)
                .push16(BF_CONFIG.board_align_roll)
                .push16(BF_CONFIG.board_align_pitch)
                .push16(BF_CONFIG.board_align_yaw)
                .push16(BF_CONFIG.currentscale)
                .push16(BF_CONFIG.currentoffset);
            break;
        case MSPCodes.MSP_SET_PID_CONTROLLER:
            buffer.push8(PID.controller);
            break;
        case MSPCodes.MSP_SET_PID:
            for (var i = 0; i < PIDs.length; i++) {
                for (var j = 0; j < 3; j++) {
                    buffer.push8(parseInt(PIDs[i][j]));
                }
            }
            break;
        case MSPCodes.MSP_SET_RC_TUNING:
            buffer.push8(Math.round(RC_tuning.RC_RATE * 100))
                .push8(Math.round(RC_tuning.RC_EXPO * 100));
            if (semver.lt(CONFIG.apiVersion, "1.7.0")) {
                buffer.push8(Math.round(RC_tuning.roll_pitch_rate * 100));
            } else {
                buffer.push8(Math.round(RC_tuning.roll_rate * 100))
                    .push8(Math.round(RC_tuning.pitch_rate * 100));
            }
            buffer.push8(Math.round(RC_tuning.yaw_rate * 100))
                .push8(Math.round(RC_tuning.dynamic_THR_PID * 100))
                .push8(Math.round(RC_tuning.throttle_MID * 100))
                .push8(Math.round(RC_tuning.throttle_EXPO * 100));
            if (semver.gte(CONFIG.apiVersion, "1.7.0")) {
                buffer.push16(RC_tuning.dynamic_THR_breakpoint);
            }
            if (semver.gte(CONFIG.apiVersion, "1.10.0")) {
                buffer.push8(Math.round(RC_tuning.RC_YAW_EXPO * 100));
                if (semver.gte(CONFIG.flightControllerVersion, "2.9.1")) {
                    buffer.push8(Math.round(RC_tuning.rcYawRate * 100));
                }
            }
            break;
        case MSPCodes.MSP_SET_RX_MAP:
            for (var i = 0; i < RC_MAP.length; i++) {
                buffer.push8(RC_MAP[i]);
            }
            break;
        case MSPCodes.MSP_SET_ACC_TRIM:
            buffer.push16(CONFIG.accelerometerTrims[0])
                .push16(CONFIG.accelerometerTrims[1]);
            break;
        case MSPCodes.MSP_SET_ARMING_CONFIG:
            buffer.push8(ARMING_CONFIG.auto_disarm_delay)
                .push8(ARMING_CONFIG.disarm_kill_switch);
            break;
        case MSPCodes.MSP_SET_LOOP_TIME:
            buffer.push16(FC_CONFIG.loopTime);
            break;
        case MSPCodes.MSP_SET_MISC:
            buffer.push16(MISC.midrc)
                .push16(MISC.minthrottle)
                .push16(MISC.maxthrottle)
                .push16(MISC.mincommand)
                .push16(MISC.failsafe_throttle)
                .push8(MISC.gps_type)
                .push8(MISC.gps_baudrate)
                .push8(MISC.gps_ubx_sbas)
                .push8(MISC.multiwiicurrentoutput)
                .push8(MISC.rssi_channel)
                .push8(MISC.placeholder2);
            if (semver.lt(CONFIG.apiVersion, "1.18.0")) {
                buffer.push16(Math.round(MISC.mag_declination * 10));
            } else {            
                buffer.push16(Math.round(MISC.mag_declination * 100));
            }
            buffer.push8(MISC.vbatscale)
                .push8(Math.round(MISC.vbatmincellvoltage * 10))
                .push8(Math.round(MISC.vbatmaxcellvoltage * 10))
                .push8(Math.round(MISC.vbatwarningcellvoltage * 10));
            break;

        case MSPCodes.MSP_SET_RX_CONFIG:
            buffer.push8(RX_CONFIG.serialrx_provider)
                .push16(RX_CONFIG.maxcheck)
                .push16(RX_CONFIG.midrc)
                .push16(RX_CONFIG.mincheck)
                .push8(RX_CONFIG.spektrum_sat_bind)
                .push16(RX_CONFIG.rx_min_usec)
                .push16(RX_CONFIG.rx_max_usec);
            if (semver.gte(CONFIG.apiVersion, "1.20.0")) {
                buffer.push8(RX_CONFIG.rcInterpolation)
                    .push8(RX_CONFIG.rcInterpolationInterval)
                    .push16(RX_CONFIG.airModeActivateThreshold);
            }

            break;

        case MSPCodes.MSP_SET_FAILSAFE_CONFIG:
            buffer.push8(FAILSAFE_CONFIG.failsafe_delay)
                .push8(FAILSAFE_CONFIG.failsafe_off_delay)
                .push16(FAILSAFE_CONFIG.failsafe_throttle);
            if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
                buffer.push8(FAILSAFE_CONFIG.failsafe_kill_switch)
                    .push16(FAILSAFE_CONFIG.failsafe_throttle_low_delay)
                    .push8(FAILSAFE_CONFIG.failsafe_procedure);
            }
            break;

        case MSPCodes.MSP_SET_TRANSPONDER_CONFIG:
            for (var i = 0; i < TRANSPONDER.data.length; i++) {
                buffer.push8(TRANSPONDER.data[i]);
            }
            break;

        case MSPCodes.MSP_SET_CHANNEL_FORWARDING:
            for (var i = 0; i < SERVO_CONFIG.length; i++) {
                var out = SERVO_CONFIG[i].indexOfChannelToForward;
                if (out == undefined) {
                    out = 255; // Cleanflight defines "CHANNEL_FORWARDING_DISABLED" as "(uint8_t)0xFF"
                }
                buffer.push8(out);
            }
            break;
        case MSPCodes.MSP_SET_CF_SERIAL_CONFIG:
            var supportedBaudRates = [ // 0 based index.
                                       'AUTO',
                                       '9600',
                                       '19200',
                                       '38400',
                                       '57600',
                                       '115200',
                                       '230400',
                                       '250000',
                                   ]; //TODO, instead of lookuptable, this should be sent as uint32
            var serialPortFunctions = {
                'MSP': 0,
                'GPS': 1, 
                'TELEMETRY_FRSKY': 2, 
                'TELEMETRY_HOTT': 3,
                'TELEMETRY_MSP': 4, 
                'TELEMETRY_LTM': 4, // LTM replaced MSP 
                'TELEMETRY_SMARTPORT': 5,
                'RX_SERIAL': 6,
                'BLACKBOX': 7,
                'TELEMETRY_MAVLINK': 8,
            };

            if (semver.lt(CONFIG.apiVersion, "1.6.0")) {

                for (var i = 0; i < SERIAL_CONFIG.ports.length; i++) {
                    buffer.push8(SERIAL_CONFIG.ports[i].scenario);
                }
                buffer.push32(SERIAL_CONFIG.mspBaudRate)
                    .push32(SERIAL_CONFIG.cliBaudRate)
                    .push32(SERIAL_CONFIG.gpsBaudRate)
                    .push32(SERIAL_CONFIG.gpsPassthroughBaudRate);
            } else {
                for (var i = 0; i < SERIAL_CONFIG.ports.length; i++) {
                    var serialPort = SERIAL_CONFIG.ports[i];
                    
                    buffer.push8(serialPort.identifier);
                    
                    var functionMask = self.serialPortFunctionsToMask(serialPort.functions);
                    buffer.push16(functionMask)
                        .push8(supportedBaudRates.indexOf(serialPort.msp_baudrate))
                        .push8(supportedBaudRates.indexOf(serialPort.gps_baudrate))
                        .push8(supportedBaudRates.indexOf(serialPort.telemetry_baudrate))
                        .push8(supportedBaudRates.indexOf(serialPort.blackbox_baudrate));
                }
            }
            break;

        case MSPCodes.MSP_SET_3D:
            buffer.push16(_3D.deadband3d_low)
                .push16(_3D.deadband3d_high)
                .push16(_3D.neutral3d);
            if (semver.lt(CONFIG.apiVersion, "1.17.0")) {
                buffer.push16(_3D.deadband3d_throttle);
            }
            break;    

        case MSPCodes.MSP_SET_RC_DEADBAND:
            buffer.push8(RC_deadband.deadband)
                .push8(RC_deadband.yaw_deadband) 
                .push8(RC_deadband.alt_hold_deadband);
            if (semver.gte(CONFIG.apiVersion, "1.17.0")) {
                buffer.push16(_3D.deadband3d_throttle);
            }
            break;

        case MSPCodes.MSP_SET_SENSOR_ALIGNMENT:
            buffer.push8(SENSOR_ALIGNMENT.align_gyro)
                .push8(SENSOR_ALIGNMENT.align_acc)
                .push8(SENSOR_ALIGNMENT.align_mag);
            break
        case MSPCodes.MSP_SET_ADVANCED_CONFIG:
            buffer.push8(PID_ADVANCED_CONFIG.gyro_sync_denom)
                .push8(PID_ADVANCED_CONFIG.pid_process_denom)
                .push8(PID_ADVANCED_CONFIG.use_unsyncedPwm)
                .push8(PID_ADVANCED_CONFIG.fast_pwm_protocol)
                .push16(PID_ADVANCED_CONFIG.motor_pwm_rate);
            break;
        case MSPCodes.MSP_SET_FILTER_CONFIG:
            buffer.push8(FILTER_CONFIG.gyro_soft_lpf_hz)
                .push16(FILTER_CONFIG.dterm_lpf_hz)
                .push16(FILTER_CONFIG.yaw_lpf_hz);
            if (semver.gte(CONFIG.apiVersion, "1.20.0")) {
                buffer.push16(FILTER_CONFIG.gyro_soft_notch_hz)
                    .push16(FILTER_CONFIG.gyro_soft_notch_cutoff)
                    .push16(FILTER_CONFIG.dterm_notch_hz)
                    .push16(FILTER_CONFIG.dterm_notch_cutoff);
            }
            break;
        case MSPCodes.MSP_SET_PID_ADVANCED:
            if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
                buffer.push16(ADVANCED_TUNING.rollPitchItermIgnoreRate)
                    .push16(ADVANCED_TUNING.yawItermIgnoreRate)
                    .push16(ADVANCED_TUNING.yaw_p_limit)
                    .push8(ADVANCED_TUNING.deltaMethod)
                    .push8(ADVANCED_TUNING.vbatPidCompensation)
                    .push8(ADVANCED_TUNING.ptermSetpointWeight)
                    .push8(ADVANCED_TUNING.dtermSetpointWeight)
                    .push8(ADVANCED_TUNING.toleranceBand)
                    .push8(ADVANCED_TUNING.toleranceBandReduction)
                    .push8(ADVANCED_TUNING.itermThrottleGain)
                    .push16(ADVANCED_TUNING.pidMaxVelocity)
                    .push16(ADVANCED_TUNING.pidMaxVelocityYaw);
            }
            // only supports 1 version pre bf 3.0
            else {
                buffer.push16(ADVANCED_TUNING.rollPitchItermIgnoreRate)
                   .push16(ADVANCED_TUNING.yawItermIgnoreRate)
                   .push16(ADVANCED_TUNING.yaw_p_limit)
                   .push8(ADVANCED_TUNING.deltaMethod)
                   .push8(ADVANCED_TUNING.vbatPidCompensation);
            }
            break;
        case MSPCodes.MSP_SET_SPECIAL_PARAMETERS:
            if (semver.lt(CONFIG.flightControllerVersion, "2.9.1")) {
                buffer.push8(Math.round(RC_tuning.rcYawRate * 100));
                if (semver.gte(CONFIG.flightControllerVersion, "2.8.2")) {
                    buffer.push16(RX_CONFIG.airModeActivateThreshold)
                        .push8(RX_CONFIG.rcSmoothInterval)
                        .push16(SPECIAL_PARAMETERS.escDesyncProtection);
                }
            }
            break;
        case MSPCodes.MSP_SET_SENSOR_CONFIG:
            buffer.push8(SENSOR_CONFIG.acc_hardware)
                .push8(SENSOR_CONFIG.baro_hardware)
                .push8(SENSOR_CONFIG.mag_hardware);
            break;
            
        case MSPCodes.MSP_SET_NAME:
            var MSP_BUFFER_SIZE = 64;
            for (var i = 0; i<CONFIG.name.length && i<MSP_BUFFER_SIZE; i++) {
                buffer.push8(CONFIG.name.charCodeAt(i));
            }
            break;
        
        case MSPCodes.MSP_SET_BLACKBOX_CONFIG:
            buffer.push8(BLACKBOX.blackboxDevice) 
                .push8(BLACKBOX.blackboxRateNum)
                .push8(BLACKBOX.blackboxRateDenom);
            break;     
        
        default:
            return false;
    }

    return buffer;
};

/**
 * Set raw Rx values over MSP protocol.
 * 
 * Channels is an array of 16-bit unsigned integer channel values to be sent. 8 channels is probably the maximum.
 */
MspHelper.prototype.setRawRx = function(channels) {
    var buffer = [];
    
    for (var i = 0; i < channels.length; i++) {
        buffer.push16(channels[i]);
    }
    
    MSP.send_message(MSPCodes.MSP_SET_RAW_RC, buffer, false);
}

/**
 * Send a request to read a block of data from the dataflash at the given address and pass that address and a dataview
 * of the returned data to the given callback (or null for the data if an error occured).
 */
MspHelper.prototype.dataflashRead = function(address, onDataCallback) {
    MSP.send_message(MSPCodes.MSP_DATAFLASH_READ, [address & 0xFF, (address >> 8) & 0xFF, (address >> 16) & 0xFF, (address >> 24) & 0xFF], 
            false, function(response) {
        var chunkAddress = response.data.readU32();
        
        // Verify that the address of the memory returned matches what the caller asked for
        if (chunkAddress == address) {
            /* Strip that address off the front of the reply and deliver it separately so the caller doesn't have to
             * figure out the reply format:
             */
            onDataCallback(address, new DataView(response.data.buffer, response.data.byteOffset + 4, response.data.buffer.byteLength - 4));
        } else {
            // Report error
            onDataCallback(address, null);
        }
    });
};

MspHelper.prototype.sendServoConfigurations = function(onCompleteCallback) {
    var nextFunction = send_next_servo_configuration; 
    
    var servoIndex = 0;

    if (SERVO_CONFIG.length == 0) {
        onCompleteCallback();
    } else {
        nextFunction();
    }


    function send_next_servo_configuration() {
        
        var buffer = [];

        if (semver.lt(CONFIG.apiVersion, "1.12.0")) {
            // send all in one go
            // 1.9.0 had a bug where the MSP input buffer was too small, limit to 8.
            for (var i = 0; i < SERVO_CONFIG.length && i < 8; i++) {
                buffer.push16(SERVO_CONFIG[i].min)
                    .push16(SERVO_CONFIG[i].max)
                    .push16(SERVO_CONFIG[i].middle)
                    .push8(SERVO_CONFIG[i].rate);
            }
            nextFunction = send_channel_forwarding;
        } else {
            // send one at a time, with index
            
            var servoConfiguration = SERVO_CONFIG[servoIndex];
            
            buffer.push8(servoIndex)
                .push16(servoConfiguration.min)
                .push16(servoConfiguration.max)
                .push16(servoConfiguration.middle)
                .push8(servoConfiguration.rate)
                .push8(servoConfiguration.angleAtMin)
                .push8(servoConfiguration.angleAtMax);

            var out = servoConfiguration.indexOfChannelToForward;
            if (out == undefined) {
                out = 255; // Cleanflight defines "CHANNEL_FORWARDING_DISABLED" as "(uint8_t)0xFF"
            }
            buffer.push8(out)
                .push32(servoConfiguration.reversedInputSources);
            
            // prepare for next iteration
            servoIndex++;
            if (servoIndex == SERVO_CONFIG.length) {
                nextFunction = onCompleteCallback;
            }
        }
        MSP.send_message(MSPCodes.MSP_SET_SERVO_CONFIGURATION, buffer, false, nextFunction);
    }
    
    function send_channel_forwarding() {
        var buffer = [];

        for (var i = 0; i < SERVO_CONFIG.length; i++) {
            var out = SERVO_CONFIG[i].indexOfChannelToForward;
            if (out == undefined) {
                out = 255; // Cleanflight defines "CHANNEL_FORWARDING_DISABLED" as "(uint8_t)0xFF"
            }
            buffer.push8(out);
        }

        nextFunction = onCompleteCallback;

        MSP.send_message(MSPCodes.MSP_SET_CHANNEL_FORWARDING, buffer, false, nextFunction);
    }
};

MspHelper.prototype.sendModeRanges = function(onCompleteCallback) {
    var nextFunction = send_next_mode_range; 
    
    var modeRangeIndex = 0;

    if (MODE_RANGES.length == 0) {
        onCompleteCallback();
    } else {
        send_next_mode_range();
    }

    function send_next_mode_range() {
        
        var modeRange = MODE_RANGES[modeRangeIndex];
                        
        var buffer = [];
        buffer.push8(modeRangeIndex)
            .push8(modeRange.id)
            .push8(modeRange.auxChannelIndex)
            .push8((modeRange.range.start - 900) / 25)
            .push8((modeRange.range.end - 900) / 25);
        
        // prepare for next iteration
        modeRangeIndex++;
        if (modeRangeIndex == MODE_RANGES.length) {
            nextFunction = onCompleteCallback;
        
        }
        MSP.send_message(MSPCodes.MSP_SET_MODE_RANGE, buffer, false, nextFunction);
    }
};

MspHelper.prototype.sendAdjustmentRanges = function(onCompleteCallback) {
    var nextFunction = send_next_adjustment_range; 
        
    var adjustmentRangeIndex = 0;

    if (ADJUSTMENT_RANGES.length == 0) {
        onCompleteCallback();
    } else {
        send_next_adjustment_range();
    }


    function send_next_adjustment_range() {
        
        var adjustmentRange = ADJUSTMENT_RANGES[adjustmentRangeIndex];
                        
        var buffer = [];
        buffer.push8(adjustmentRangeIndex)
            .push8(adjustmentRange.slotIndex)
            .push8(adjustmentRange.auxChannelIndex)
            .push8((adjustmentRange.range.start - 900) / 25)
            .push8((adjustmentRange.range.end - 900) / 25)
            .push8(adjustmentRange.adjustmentFunction)
            .push8(adjustmentRange.auxSwitchChannelIndex);
        
        // prepare for next iteration
        adjustmentRangeIndex++;
        if (adjustmentRangeIndex == ADJUSTMENT_RANGES.length) {
            nextFunction = onCompleteCallback;
        
        }
        MSP.send_message(MSPCodes.MSP_SET_ADJUSTMENT_RANGE, buffer, false, nextFunction);
    }
};

MspHelper.prototype.sendLedStripConfig = function(onCompleteCallback) {
    
    var nextFunction = send_next_led_strip_config; 
    
    var ledIndex = 0;

    if (LED_STRIP.length == 0) {
        onCompleteCallback();
    } else {
        send_next_led_strip_config();
    }

    function send_next_led_strip_config() {
        
        var led = LED_STRIP[ledIndex];
        var ledDirectionLetters =        ['n', 'e', 's', 'w', 'u', 'd'];      // in LSB bit order
        var ledFunctionLetters =         ['i', 'w', 'f', 'a', 't', 'r', 'c', 'g', 's', 'b', 'l']; // in LSB bit order
        var ledBaseFunctionLetters =     ['c', 'f', 'a', 'l', 's', 'g', 'r']; // in LSB bit 
        var ledOverlayLetters =         ['t', 'o', 'b', 'n', 'i', 'w']; // in LSB bit 

        var buffer = [];
        
        buffer.push(ledIndex);

        if (semver.lt(CONFIG.apiVersion, "1.20.0")) {
            var directionMask = 0;
            for (var directionLetterIndex = 0; directionLetterIndex < led.directions.length; directionLetterIndex++) {
                var bitIndex = ledDirectionLetters.indexOf(led.directions[directionLetterIndex]);
                if (bitIndex >= 0) {
                    directionMask = bit_set(directionMask, bitIndex);
                }
            }
            buffer.push16(directionMask);
    
            var functionMask = 0;
            for (var functionLetterIndex = 0; functionLetterIndex < led.functions.length; functionLetterIndex++) {
                var bitIndex = ledFunctionLetters.indexOf(led.functions[functionLetterIndex]);
                if (bitIndex >= 0) {
                    functionMask = bit_set(functionMask, bitIndex);
                }
            }
            buffer.push16(functionMask)
            
                .push8(led.x)
                .push8(led.y)
    
                .push8(led.color);
        } else {
            var mask = 0;

            mask |= (led.y << 0);
            mask |= (led.x << 4);

            for (var functionLetterIndex = 0; functionLetterIndex < led.functions.length; functionLetterIndex++) {
                var fnIndex = ledBaseFunctionLetters.indexOf(led.functions[functionLetterIndex]);
                if (fnIndex >= 0) {
                    mask |= (fnIndex << 8);
                    break;
                }
            }
            
            for (var overlayLetterIndex = 0; overlayLetterIndex < led.functions.length; overlayLetterIndex++) {
                var bitIndex = ledOverlayLetters.indexOf(led.functions[overlayLetterIndex]);
                if (bitIndex >= 0) {
                    mask |= bit_set(mask, bitIndex + 12);
                }
            }

            mask |= (led.color << 18);

            for (var directionLetterIndex = 0; directionLetterIndex < led.directions.length; directionLetterIndex++) {
                var bitIndex = ledDirectionLetters.indexOf(led.directions[directionLetterIndex]);
                if (bitIndex >= 0) {
                    mask |= bit_set(mask, bitIndex + 22);
                }
            }
            
            mask |= (0 << 28); // parameters
            
            
            buffer.push32(mask);
        }
        
        // prepare for next iteration
        ledIndex++;
        if (ledIndex == LED_STRIP.length) {
            nextFunction = onCompleteCallback;
        }
        
        MSP.send_message(MSPCodes.MSP_SET_LED_STRIP_CONFIG, buffer, false, nextFunction);
    }
}

MspHelper.prototype.sendLedStripColors = function(onCompleteCallback) {
    if (LED_COLORS.length == 0) {
        onCompleteCallback();
    } else {
        var buffer = [];
        
        for (var colorIndex = 0; colorIndex < LED_COLORS.length; colorIndex++) {
            var color = LED_COLORS[colorIndex];
            
            buffer.push16(color.h)
                .push8(color.s)
                .push8(color.v);
        }
        MSP.send_message(MSPCodes.MSP_SET_LED_COLORS, buffer, false, onCompleteCallback);
    }
}

MspHelper.prototype.sendLedStripModeColors = function(onCompleteCallback) {
    
    var nextFunction = send_next_led_strip_mode_color; 
    var index = 0;
    
    if (LED_MODE_COLORS.length == 0) {
        onCompleteCallback();
    } else {
        send_next_led_strip_mode_color();
    }
    
    function send_next_led_strip_mode_color() {
        var buffer = [];
        
        var mode_color = LED_MODE_COLORS[index];
        
        buffer.push8(mode_color.mode)
            .push8(mode_color.direction)
            .push8(mode_color.color);

        // prepare for next iteration
        index++;
        if (index == LED_MODE_COLORS.length) {
            nextFunction = onCompleteCallback;
        }

        MSP.send_message(MSPCodes.MSP_SET_LED_STRIP_MODECOLOR, buffer, false, nextFunction);
    }
}

MspHelper.prototype.serialPortFunctionMaskToFunctions = function(functionMask) {
    var functions = [];
    var serialPortFunctions = {
        'MSP': 0,
        'GPS': 1, 
        'TELEMETRY_FRSKY': 2, 
        'TELEMETRY_HOTT': 3,
        'TELEMETRY_MSP': 4, 
        'TELEMETRY_LTM': 4, // LTM replaced MSP 
        'TELEMETRY_SMARTPORT': 5,
        'RX_SERIAL': 6,
        'BLACKBOX': 7,
        'TELEMETRY_MAVLINK': 8,
    };

    var keys = Object.keys(serialPortFunctions);
    for (var index = 0; index < keys.length; index++) {
        var key = keys[index];
        var bit = serialPortFunctions[key];
        if (bit_check(functionMask, bit)) {
            functions.push(key);
        }
    }
    return functions;
}

MspHelper.prototype.serialPortFunctionsToMask = function(functions) {
    var mask = 0;
    var serialPortFunctions = {
            'MSP': 0,
            'GPS': 1, 
            'TELEMETRY_FRSKY': 2, 
            'TELEMETRY_HOTT': 3,
            'TELEMETRY_MSP': 4, 
            'TELEMETRY_LTM': 4, // LTM replaced MSP 
            'TELEMETRY_SMARTPORT': 5,
            'RX_SERIAL': 6,
            'BLACKBOX': 7,
            'TELEMETRY_MAVLINK': 8,
        };
    
    var keys = Object.keys(serialPortFunctions);
    for (var index = 0; index < functions.length; index++) {
        var key = functions[index];
        var bitIndex = serialPortFunctions[key];
        if (bitIndex >= 0) {
            mask = bit_set(mask, bitIndex);
        }
    }
    return mask;
}

MspHelper.prototype.sendRxFailConfig = function(onCompleteCallback) {
    var nextFunction = send_next_rxfail_config;

    var rxFailIndex = 0;

    if (RXFAIL_CONFIG.length == 0) {
        onCompleteCallback();
    } else {
        send_next_rxfail_config();
    }

    function send_next_rxfail_config() {

        var rxFail = RXFAIL_CONFIG[rxFailIndex];

        var buffer = [];
        buffer.push8(rxFailIndex)
            .push8(rxFail.mode)
            .push16(rxFail.value);
        

        // prepare for next iteration
        rxFailIndex++;
        if (rxFailIndex == RXFAIL_CONFIG.length) {
            nextFunction = onCompleteCallback;

        }
        MSP.send_message(MSPCodes.MSP_SET_RXFAIL_CONFIG, buffer, false, nextFunction);
    }
}

MSP.SDCARD_STATE_NOT_PRESENT = 0; //TODO, move these to better place
MSP.SDCARD_STATE_FATAL       = 1;
MSP.SDCARD_STATE_CARD_INIT   = 2;
MSP.SDCARD_STATE_FS_INIT     = 3;
MSP.SDCARD_STATE_READY       = 4;
