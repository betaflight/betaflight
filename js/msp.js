'use strict';

// MSP_codes needs to be re-integrated inside MSP object
var MSP_codes = {
    MSP_IDENT:              100,
    MSP_STATUS:             101,
    MSP_RAW_IMU:            102,
    MSP_SERVO:              103,
    MSP_MOTOR:              104,
    MSP_RC:                 105,
    MSP_RAW_GPS:            106,
    MSP_COMP_GPS:           107,
    MSP_ATTITUDE:           108,
    MSP_ALTITUDE:           109,
    MSP_ANALOG:             110,
    MSP_RC_TUNING:          111,
    MSP_PID:                112,
    MSP_BOX:                113,
    MSP_MISC:               114,
    MSP_MOTOR_PINS:         115,
    MSP_BOXNAMES:           116,
    MSP_PIDNAMES:           117,
    MSP_WP:                 118,
    MSP_BOXIDS:             119,
    MSP_SERVO_CONF:         120,

    MSP_SET_RAW_RC:         200,
    MSP_SET_RAW_GPS:        201,
    MSP_SET_PID:            202,
    MSP_SET_BOX:            203,
    MSP_SET_RC_TUNING:      204,
    MSP_ACC_CALIBRATION:    205,
    MSP_MAG_CALIBRATION:    206,
    MSP_SET_MISC:           207,
    MSP_RESET_CONF:         208,
    MSP_SET_WP:             209,
    MSP_SELECT_SETTING:     210,
    MSP_SET_HEAD:           211,
    MSP_SET_SERVO_CONF:     212,
    MSP_SET_MOTOR:          214,

    // MSP_BIND:               240,

    MSP_EEPROM_WRITE:       250,

    MSP_DEBUGMSG:           253,
    MSP_DEBUG:              254,

    // Additional baseflight commands that are not compatible with MultiWii
    MSP_UID:                160, // Unique device ID
    MSP_ACC_TRIM:           240, // get acc angle trim values
    MSP_SET_ACC_TRIM:       239, // set acc angle trim values
    MSP_GPSSVINFO:          164, // get Signal Strength (only U-Blox)

    // Additional private MSP for baseflight configurator (yes thats us \o/)
    MSP_RCMAP:              64, // get channel map (also returns number of channels total)
    MSP_SET_RCMAP:          65, // set rc map, numchannels to set comes from MSP_RCMAP
    MSP_CONFIG:             66, // baseflight-specific settings that aren't covered elsewhere
    MSP_SET_CONFIG:         67, // baseflight-specific settings save
    MSP_SET_REBOOT:         68, // reboot settings
    MSP_BUILDINFO:          69  // build date as well as some space for future expansion
};

var MSP = {
    state:                      0,
    message_direction:          1,
    code:                       0,
    message_length_expected:    0,
    message_length_received:    0,
    message_buffer:             null,
    message_buffer_uint8_view:  null,
    message_checksum:           0,

    callbacks:                  [],
    packet_error:               0,

    read: function (readInfo) {
        var data = new Uint8Array(readInfo.data);

        for (var i = 0; i < data.length; i++) {
            switch (this.state) {
                case 0: // sync char 1
                    if (data[i] == 36) { // $
                        this.state++;
                    }
                    break;
                case 1: // sync char 2
                    if (data[i] == 77) { // M
                        this.state++;
                    } else { // restart and try again
                        this.state = 0;
                    }
                    break;
                case 2: // direction (should be >)
                    if (data[i] == 62) { // >
                        this.message_direction = 1;
                    } else { // <
                        this.message_direction = 0;
                    }

                    this.state++;
                    break;
                case 3:
                    this.message_length_expected = data[i];

                    this.message_checksum = data[i];

                    // setup arraybuffer
                    this.message_buffer = new ArrayBuffer(this.message_length_expected);
                    this.message_buffer_uint8_view = new Uint8Array(this.message_buffer);

                    this.state++;
                    break;
                case 4:
                    this.code = data[i];
                    this.message_checksum ^= data[i];

                    if (this.message_length_expected > 0) {
                        // process payload
                        this.state++;
                    } else {
                        // no payload
                        this.state += 2;
                    }
                    break;
                case 5: // payload
                    this.message_buffer_uint8_view[this.message_length_received] = data[i];
                    this.message_checksum ^= data[i];
                    this.message_length_received++;

                    if (this.message_length_received >= this.message_length_expected) {
                        this.state++;
                    }
                    break;
                case 6:
                    if (this.message_checksum == data[i]) {
                        // message received, process
                        this.process_data(this.code, this.message_buffer, this.message_length_expected);
                    } else {
                        console.log('code: ' + this.code + ' - crc failed');

                        this.packet_error++;
                        $('span.packet-error').html(this.packet_error);
                    }

                    // Reset variables
                    this.message_length_received = 0;
                    this.state = 0;
                    break;

                default:
                    console.log('Unknown state detected: ' + this.state);
            }
        }
    },
    process_data: function (code, message_buffer, message_length) {
        var data = new DataView(message_buffer, 0); // DataView (allowing us to view arrayBuffer as struct/union)

        switch (code) {
            case MSP_codes.MSP_IDENT:
                CONFIG.version = parseFloat((data.getUint8(0) / 100).toFixed(2));
                CONFIG.multiType = data.getUint8(1);
                CONFIG.msp_version = data.getUint8(2);
                CONFIG.capability = data.getUint32(3, 1);
                break;
            case MSP_codes.MSP_STATUS:
                CONFIG.cycleTime = data.getUint16(0, 1);
                CONFIG.i2cError = data.getUint16(2, 1);
                CONFIG.activeSensors = data.getUint16(4, 1);
                CONFIG.mode = data.getUint32(6, 1);
                CONFIG.profile = data.getUint8(10);

                sensor_status(CONFIG.activeSensors);
                $('span.i2c-error').text(CONFIG.i2cError);
                $('span.cycle-time').text(CONFIG.cycleTime);
                break;
            case MSP_codes.MSP_RAW_IMU:
                // 512 for mpu6050, 256 for mma
                // currently we are unable to differentiate between the sensor types, so we are goign with 512
                SENSOR_DATA.accelerometer[0] = data.getInt16(0, 1) / 512;
                SENSOR_DATA.accelerometer[1] = data.getInt16(2, 1) / 512;
                SENSOR_DATA.accelerometer[2] = data.getInt16(4, 1) / 512;

                // properly scaled
                SENSOR_DATA.gyroscope[0] = data.getInt16(6, 1) * (4 / 16.4);
                SENSOR_DATA.gyroscope[1] = data.getInt16(8, 1) * (4 / 16.4);
                SENSOR_DATA.gyroscope[2] = data.getInt16(10, 1) * (4 / 16.4);

                // no clue about scaling factor
                SENSOR_DATA.magnetometer[0] = data.getInt16(12, 1) / 1090;
                SENSOR_DATA.magnetometer[1] = data.getInt16(14, 1) / 1090;
                SENSOR_DATA.magnetometer[2] = data.getInt16(16, 1) / 1090;
                break;
            case MSP_codes.MSP_SERVO:
                var needle = 0;
                for (var i = 0; i < 8; i++) {
                    SERVO_DATA[i] = data.getUint16(needle, 1);

                    needle += 2;
                }
                break;
            case MSP_codes.MSP_MOTOR:
                var needle = 0;
                for (var i = 0; i < 8; i++) {
                    MOTOR_DATA[i] = data.getUint16(needle, 1);

                    needle += 2;
                }
                break;
            case MSP_codes.MSP_RC:
                RC.active_channels = message_length / 2;

                for (var i = 0; i < RC.active_channels; i++) {
                    RC.channels[i] = data.getUint16((i * 2), 1);
                }
                break;
            case MSP_codes.MSP_RAW_GPS:
                GPS_DATA.fix = data.getUint8(0);
                GPS_DATA.numSat = data.getUint8(1);
                GPS_DATA.lat = data.getInt32(2, 1);
                GPS_DATA.lon = data.getInt32(6, 1);
                GPS_DATA.alt = data.getUint16(10, 1);
                GPS_DATA.speed = data.getUint16(12, 1);
                GPS_DATA.ground_course = data.getUint16(14, 1);
                break;
            case MSP_codes.MSP_COMP_GPS:
                GPS_DATA.distanceToHome = data.getUint16(0, 1);
                GPS_DATA.directionToHome = data.getUint16(2, 1);
                GPS_DATA.update = data.getUint8(4);
                break;
            case MSP_codes.MSP_ATTITUDE:
                SENSOR_DATA.kinematics[0] = data.getInt16(0, 1) / 10.0; // x
                SENSOR_DATA.kinematics[1] = data.getInt16(2, 1) / 10.0; // y
                SENSOR_DATA.kinematics[2] = data.getInt16(4, 1); // z
                break;
            case MSP_codes.MSP_ALTITUDE:
                SENSOR_DATA.altitude = parseFloat((data.getInt32(0, 1) / 100.0).toFixed(2)); // correct scale factor
                break;
            case MSP_codes.MSP_ANALOG:
                ANALOG.voltage = data.getUint8(0) / 10.0;
                ANALOG.mAhdrawn = data.getUint16(1, 1);
                ANALOG.rssi = data.getUint16(3, 1); // 0-1023
                ANALOG.amperage = data.getUint16(5, 1) / 100; // A
                break;
            case MSP_codes.MSP_RC_TUNING:
                RC_tuning.RC_RATE = parseFloat((data.getUint8(0) / 100).toFixed(2));
                RC_tuning.RC_EXPO = parseFloat((data.getUint8(1) / 100).toFixed(2));
                RC_tuning.roll_pitch_rate = parseFloat((data.getUint8(2) / 100).toFixed(2));
                RC_tuning.yaw_rate = parseFloat((data.getUint8(3) / 100).toFixed(2));
                RC_tuning.dynamic_THR_PID = parseFloat((data.getUint8(4) / 100).toFixed(2));
                RC_tuning.throttle_MID = parseFloat((data.getUint8(5) / 100).toFixed(2));
                RC_tuning.throttle_EXPO = parseFloat((data.getUint8(6) / 100).toFixed(2));
                break;
            case MSP_codes.MSP_PID:
                // PID data arrived, we need to scale it and save to appropriate bank / array
                for (var i = 0, needle = 0; i < (message_length / 3); i++, needle += 3) {
                    // main for loop selecting the pid section
                    switch (i) {
                        case 0:
                        case 1:
                        case 2:
                        case 3:
                        case 7:
                        case 8:
                        case 9:
                            PIDs[i][0] = data.getUint8(needle) / 10;
                            PIDs[i][1] = data.getUint8(needle + 1) / 1000;
                            PIDs[i][2] = data.getUint8(needle + 2);
                            break;
                        case 4:
                            PIDs[i][0] = data.getUint8(needle) / 100;
                            PIDs[i][1] = data.getUint8(needle + 1) / 100;
                            PIDs[i][2] = data.getUint8(needle + 2) / 1000;
                            break;
                        case 5:
                        case 6:
                            PIDs[i][0] = data.getUint8(needle) / 10;
                            PIDs[i][1] = data.getUint8(needle + 1) / 100;
                            PIDs[i][2] = data.getUint8(needle + 2) / 1000;
                            break;
                    }
                }
                break;
            case MSP_codes.MSP_BOX:
                AUX_CONFIG_values = []; // empty the array as new data is coming in

                // fill in current data
                for (var i = 0; i < data.byteLength; i += 2) { // + 2 because uint16_t = 2 bytes
                    AUX_CONFIG_values.push(data.getUint16(i, 1));
                }
                break;
            case MSP_codes.MSP_MISC: // 22 bytes
                MISC.midrc = data.getInt16(0, 1);
                MISC.minthrottle = data.getUint16(2, 1); // 0-2000
                MISC.maxthrottle = data.getUint16(4, 1); // 0-2000
                MISC.mincommand = data.getUint16(6, 1); // 0-2000
                MISC.failsafe_throttle = data.getUint16(8, 1); // 1000-2000
                MISC.gps_type = data.getUint8(10);
                MISC.gps_baudrate = data.getUint8(11);
                MISC.gps_ubx_sbas = data.getInt8(12);
                MISC.multiwiicurrentoutput = data.getUint8(13);
                MISC.rssi_aux_channel = data.getUint8(14);
                MISC.placeholder2 = data.getUint8(15);
                MISC.mag_declination = data.getInt16(16, 1) / 10; // -18000-18000
                MISC.vbatscale = data.getUint8(18, 1); // 10-200
                MISC.vbatmincellvoltage = data.getUint8(19, 1) / 10; // 10-50
                MISC.vbatmaxcellvoltage = data.getUint8(20, 1) / 10; // 10-50
                MISC.vbatwarningcellvoltage = data.getUint8(21, 1) / 10; // 10-50
                break;
            case MSP_codes.MSP_MOTOR_PINS:
                console.log(data);
                break;
            case MSP_codes.MSP_BOXNAMES:
                AUX_CONFIG = []; // empty the array as new data is coming in

                var buff = [];
                for (var i = 0; i < data.byteLength; i++) {
                    if (data.getUint8(i) == 0x3B) { // ; (delimeter char)
                        AUX_CONFIG.push(String.fromCharCode.apply(null, buff)); // convert bytes into ASCII and save as strings

                        // empty buffer
                        buff = [];
                    } else {
                        buff.push(data.getUint8(i));
                    }
                }
                break;
            case MSP_codes.MSP_PIDNAMES:
                PID_names = []; // empty the array as new data is coming in

                var buff = [];
                for (var i = 0; i < data.byteLength; i++) {
                    if (data.getUint8(i) == 0x3B) { // ; (delimeter char)
                        PID_names.push(String.fromCharCode.apply(null, buff)); // convert bytes into ASCII and save as strings

                        // empty buffer
                        buff = [];
                    } else {
                        buff.push(data.getUint8(i));
                    }
                }
                break;
            case MSP_codes.MSP_WP:
                console.log(data);
                break;
            case MSP_codes.MSP_BOXIDS:
                AUX_CONFIG_IDS = []; // empty the array as new data is coming in

                for (var i = 0; i < data.byteLength; i++) {
                    AUX_CONFIG_IDS.push(data.getUint8(i));
                }
                break;
            case MSP_codes.MSP_SERVO_CONF:
                SERVO_CONFIG = []; // empty the array as new data is coming in

                for (var i = 0; i < 56; i += 7) {
                    var arr = {
                        'min': data.getInt16(i, 1),
                        'max': data.getInt16(i + 2, 1),
                        'middle': data.getInt16(i + 4, 1),
                        'rate': data.getInt8(i + 6)
                    };

                    SERVO_CONFIG.push(arr);
                }
                break;
            case MSP_codes.MSP_SET_RAW_RC:
                break;
            case MSP_codes.MSP_SET_RAW_GPS:
                break;
            case MSP_codes.MSP_SET_PID:
                console.log('PID settings saved');
                break;
            case MSP_codes.MSP_SET_BOX:
                console.log('AUX Configuration saved');
                break;
            case MSP_codes.MSP_SET_RC_TUNING:
                console.log('RC Tuning saved');
                break;
            case MSP_codes.MSP_ACC_CALIBRATION:
                console.log('Accel calibration executed');
                break;
            case MSP_codes.MSP_MAG_CALIBRATION:
                console.log('Mag calibration executed');
                break;
            case MSP_codes.MSP_SET_MISC:
                console.log('MISC Configuration saved');
                break;
            case MSP_codes.MSP_RESET_CONF:
                console.log('Settings Reset');
                break;
            case MSP_codes.MSP_SELECT_SETTING:
                console.log('Profile selected');
                break;
            case MSP_codes.MSP_SET_SERVO_CONF:
                console.log('Servo Configuration saved');
                break;
            case MSP_codes.MSP_EEPROM_WRITE:
                console.log('Settings Saved in EEPROM');
                break;
            case MSP_codes.MSP_DEBUGMSG:
                break;
            case MSP_codes.MSP_DEBUG:
                for (var i = 0; i < 4; i++)
                    SENSOR_DATA.debug[i] = data.getInt16((2 * i), 1);
                break;
            case MSP_codes.MSP_SET_MOTOR:
                console.log('Motor Speeds Updated');
                break;
            // Additional baseflight commands that are not compatible with MultiWii
            case MSP_codes.MSP_UID:
                CONFIG.uid[0] = data.getUint32(0, 1);
                CONFIG.uid[1] = data.getUint32(4, 1);
                CONFIG.uid[2] = data.getUint32(8, 1);
                break;
            case MSP_codes.MSP_ACC_TRIM:
                CONFIG.accelerometerTrims[0] = data.getInt16(0, 1); // pitch
                CONFIG.accelerometerTrims[1] = data.getInt16(2, 1); // roll
                break;
            case MSP_codes.MSP_SET_ACC_TRIM:
                console.log('Accelerometer trimms saved.');
                break;
            case MSP_codes.MSP_GPSSVINFO:
                if (data.byteLength > 0) {
                    var numCh = data.getUint8(0);

                    var needle = 1;
                    for (var i = 0; i < numCh; i++) {
                        GPS_DATA.chn[i] = data.getUint8(needle);
                        GPS_DATA.svid[i] = data.getUint8(needle + 1);
                        GPS_DATA.quality[i] = data.getUint8(needle + 2);
                        GPS_DATA.cno[i] = data.getUint8(needle + 3);

                        needle += 4;
                    }
                }
                break;
            // Additional private MSP for baseflight configurator
            case MSP_codes.MSP_RCMAP:
                RC_MAP = []; // empty the array as new data is coming in

                for (var i = 0; i < data.byteLength; i++) {
                    RC_MAP.push(data.getUint8(i));
                }
                break;
            case MSP_codes.MSP_SET_RCMAP:
                console.log('RCMAP saved');
                break;
            case MSP_codes.MSP_CONFIG:
                BF_CONFIG.mixerConfiguration = data.getUint8(0);
                BF_CONFIG.features = data.getUint32(1, 1);
                BF_CONFIG.serialrx_type = data.getUint8(5);
                BF_CONFIG.board_align_roll = data.getInt16(6, 1);
                BF_CONFIG.board_align_pitch = data.getInt16(8, 1);
                BF_CONFIG.board_align_yaw = data.getInt16(10, 1);
                BF_CONFIG.currentscale = data.getUint16(12, 1);
                BF_CONFIG.currentoffset = data.getUint16(14, 1);
                break;
            case MSP_codes.MSP_SET_CONFIG:
                break;
            case MSP_codes.MSP_SET_REBOOT:
                console.log('Reboot request accepted');
                break;
            case MSP_codes.MSP_BUILDINFO:
                var buff = [];

                for (var i = 0; i < data.byteLength; i++) {
                    buff.push(data.getUint8(i));
                }

                CONFIG.buildInfo = String.fromCharCode.apply(null, buff);
                break;

            default:
                console.log('Unknown code detected: ' + code);
        }

        // trigger callbacks, cleanup/remove callback after trigger
        for (var i = this.callbacks.length - 1; i >= 0; i--) { // itterating in reverse because we use .splice which modifies array length
            if (this.callbacks[i].code == code) {
                // save callback reference
                var callback = this.callbacks[i].callback;

                // remove timeout
                clearInterval(this.callbacks[i].timer);

                // remove object from array
                this.callbacks.splice(i, 1);

                // fire callback
                if (callback) callback({'command': code, 'data': data, 'length': message_length});
            }
        }
    },
    send_message: function (code, data, callback_sent, callback_msp) {
        var bufferOut,
            bufView;

        // always reserve 6 bytes for protocol overhead !
        if (data) {
            var size = data.length + 6,
                checksum = 0;

            bufferOut = new ArrayBuffer(size);
            bufView = new Uint8Array(bufferOut);

            bufView[0] = 36; // $
            bufView[1] = 77; // M
            bufView[2] = 60; // <
            bufView[3] = data.length;
            bufView[4] = code;

            checksum = bufView[3] ^ bufView[4];

            for (var i = 0; i < data.length; i++) {
                bufView[i + 5] = data[i];

                checksum ^= bufView[i + 5];
            }

            bufView[5 + data.length] = checksum;
        } else {
            bufferOut = new ArrayBuffer(6);
            bufView = new Uint8Array(bufferOut);

            bufView[0] = 36; // $
            bufView[1] = 77; // M
            bufView[2] = 60; // <
            bufView[3] = 0; // data length
            bufView[4] = code; // code
            bufView[5] = bufView[3] ^ bufView[4]; // checksum
        }

        // dev version 0.57 code below got recently changed due to the fact that queueing same MSP codes was unsupported
        // and was causing trouble while backup/restoring configurations
        // watch out if the recent change create any inconsistencies and then adjust accordingly
        var obj = {'code': code, 'requestBuffer': bufferOut, 'callback': (callback_msp) ? callback_msp : false, 'timer': false};

        var requestExists = false;
        for (var i = 0; i < MSP.callbacks.length; i++) {
            if (MSP.callbacks[i].code == code) {
                // request already exist, we will just attach
                requestExists = true;
                break;
            }
        }

        if (!requestExists) {
            obj.timer = setInterval(function () {
                console.log('MSP data request timed-out: ' + code);

                serial.send(bufferOut, false);
            }, 1000); // we should be able to define timeout in the future
        }

        MSP.callbacks.push(obj);

        // always send messages with data payload (even when there is a message already in the queue)
        if (data || !requestExists) {
            serial.send(bufferOut, function (sendInfo) {
                if (sendInfo.bytesSent == bufferOut.length) {
                    if (callback_sent) callback_sent();
                }
            });
        }

        return true;
    },
    callbacks_cleanup: function () {
        for (var i = 0; i < this.callbacks.length; i++) {
            clearInterval(this.callbacks[i].timer);
        }

        this.callbacks = [];
    },
    disconnect_cleanup: function () {
        this.state = 0; // reset packet state for "clean" initial entry (this is only required if user hot-disconnects)
        this.packet_error = 0; // reset CRC packet error counter for next session

        this.callbacks_cleanup();
    }
};

MSP.crunch = function (code) {
    var buffer = [];

    switch (code) {
        case MSP_codes.MSP_SET_CONFIG:
            buffer.push(BF_CONFIG.mixerConfiguration);
            buffer.push(specificByte(BF_CONFIG.features, 0));
            buffer.push(specificByte(BF_CONFIG.features, 1));
            buffer.push(specificByte(BF_CONFIG.features, 2));
            buffer.push(specificByte(BF_CONFIG.features, 3));
            buffer.push(BF_CONFIG.serialrx_type);
            buffer.push(specificByte(BF_CONFIG.board_align_roll, 0));
            buffer.push(specificByte(BF_CONFIG.board_align_roll, 1));
            buffer.push(specificByte(BF_CONFIG.board_align_pitch, 0));
            buffer.push(specificByte(BF_CONFIG.board_align_pitch, 1));
            buffer.push(specificByte(BF_CONFIG.board_align_yaw, 0));
            buffer.push(specificByte(BF_CONFIG.board_align_yaw, 1));
            buffer.push(lowByte(BF_CONFIG.currentscale));
            buffer.push(highByte(BF_CONFIG.currentscale));
            buffer.push(lowByte(BF_CONFIG.currentoffset));
            buffer.push(highByte(BF_CONFIG.currentoffset));
            break;
        case MSP_codes.MSP_SET_PID:
            for (var i = 0; i < PIDs.length; i++) {
                switch (i) {
                    case 0:
                    case 1:
                    case 2:
                    case 3:
                    case 7:
                    case 8:
                    case 9:
                        buffer.push(parseInt(PIDs[i][0] * 10));
                        buffer.push(parseInt(PIDs[i][1] * 1000));
                        buffer.push(parseInt(PIDs[i][2]));
                        break;
                    case 4:
                        buffer.push(parseInt(PIDs[i][0] * 100));
                        buffer.push(parseInt(PIDs[i][1] * 100));
                        buffer.push(parseInt(PIDs[i][2]));
                        break;
                    case 5:
                    case 6:
                        buffer.push(parseInt(PIDs[i][0] * 10));
                        buffer.push(parseInt(PIDs[i][1] * 100));
                        buffer.push(parseInt(PIDs[i][2] * 1000));
                        break;
                }
            }
            break;
        case MSP_codes.MSP_SET_RC_TUNING:
            buffer.push(parseInt(RC_tuning.RC_RATE * 100));
            buffer.push(parseInt(RC_tuning.RC_EXPO * 100));
            buffer.push(parseInt(RC_tuning.roll_pitch_rate * 100));
            buffer.push(parseInt(RC_tuning.yaw_rate * 100));
            buffer.push(parseInt(RC_tuning.dynamic_THR_PID * 100));
            buffer.push(parseInt(RC_tuning.throttle_MID * 100));
            buffer.push(parseInt(RC_tuning.throttle_EXPO * 100));
            break;
        case MSP_codes.MSP_SET_BOX:
            for (var i = 0; i < AUX_CONFIG_values.length; i++) {
                buffer.push(lowByte(AUX_CONFIG_values[i]));
                buffer.push(highByte(AUX_CONFIG_values[i]));
            }
            break;
        case MSP_codes.MSP_SET_RCMAP:
            for (var i = 0; i < RC_MAP.length; i++) {
                buffer.push(RC_MAP[i]);
            }
            break;
        case MSP_codes.MSP_SET_ACC_TRIM:
            buffer.push(lowByte(CONFIG.accelerometerTrims[0]));
            buffer.push(highByte(CONFIG.accelerometerTrims[0]));
            buffer.push(lowByte(CONFIG.accelerometerTrims[1]));
            buffer.push(highByte(CONFIG.accelerometerTrims[1]));
            break;
        case MSP_codes.MSP_SET_MISC:
            buffer.push(lowByte(MISC.midrc));
            buffer.push(highByte(MISC.midrc));
            buffer.push(lowByte(MISC.minthrottle));
            buffer.push(highByte(MISC.minthrottle));
            buffer.push(lowByte(MISC.maxthrottle));
            buffer.push(highByte(MISC.maxthrottle));
            buffer.push(lowByte(MISC.mincommand));
            buffer.push(highByte(MISC.mincommand));
            buffer.push(lowByte(MISC.failsafe_throttle));
            buffer.push(highByte(MISC.failsafe_throttle));
            buffer.push(MISC.gps_type);
            buffer.push(MISC.gps_baudrate);
            buffer.push(MISC.gps_ubx_sbas);
            buffer.push(MISC.multiwiicurrentoutput);
            buffer.push(MISC.rssi_aux_channel);
            buffer.push(MISC.placeholder2);
            buffer.push(lowByte(MISC.mag_declination * 10));
            buffer.push(highByte(MISC.mag_declination * 10));
            buffer.push(MISC.vbatscale);
            buffer.push(MISC.vbatmincellvoltage * 10);
            buffer.push(MISC.vbatmaxcellvoltage * 10);
            buffer.push(MISC.vbatwarningcellvoltage * 10);
            break;
        case MSP_codes.MSP_SET_SERVO_CONF:
            for (var i = 0; i < SERVO_CONFIG.length; i++) {
                buffer.push(lowByte(SERVO_CONFIG[i].min));
                buffer.push(highByte(SERVO_CONFIG[i].min));

                buffer.push(lowByte(SERVO_CONFIG[i].max));
                buffer.push(highByte(SERVO_CONFIG[i].max));

                buffer.push(lowByte(SERVO_CONFIG[i].middle));
                buffer.push(highByte(SERVO_CONFIG[i].middle));

                buffer.push(lowByte(SERVO_CONFIG[i].rate));
            }
            break;

        default:
            return false;
    }

    return buffer;
};