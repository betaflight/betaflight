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
    MSP_GPSSVINFO:          164  // get Signal Strength (only U-Blox)
};

var MSP = {
    state:                      0,
    message_status:             1,
    code:                       0,
    message_length_expected:    0,
    message_length_received:    0,
    message_buffer:             undefined,
    message_buffer_uint8_view:  undefined,
    message_checksum:           0,

    callbacks:                  [],
    packet_error:               0,

    callbacks_cleanup: function() {
        for (var i = 0; i < this.callbacks.length; i++) {
            clearInterval(this.callbacks[i].timer);
        }

        this.callbacks = [];
    },

    disconnect_cleanup: function() {
        this.state = 0; // reset packet state for "clean" initial entry (this is only required if user hot-disconnects)
        this.packet_error = 0; // reset CRC packet error counter for next session

        this.callbacks_cleanup();
    }
};

MSP.read = function(readInfo) {
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
                    message_status = 1;
                } else { // unknown
                    message_status = 0;
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

                if (this.message_length_expected != 0) { // standard message
                    this.state++;
                } else { // MSP_ACC_CALIBRATION, etc...
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
        }
    }
};

MSP.process_data = function(code, message_buffer, message_length) {
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
            RC.channelCount = message_length / 2;
            
            switch(RC.channelCount) {
                case 12:
                    RC.AUX8 = data.getUint16(22, 1);
                    RC.AUX7 = data.getUint16(20, 1);
                    RC.AUX6 = data.getUint16(18, 1);
                    RC.AUX5 = data.getUint16(16, 1);
                case 8:
                    RC.AUX4 = data.getUint16(14, 1);
                    RC.AUX3 = data.getUint16(12, 1);
                    RC.AUX2 = data.getUint16(10, 1);
                    RC.AUX1 = data.getUint16(8, 1);
                    RC.throttle = data.getUint16(6, 1);
                    RC.yaw = data.getUint16(4, 1);
                    RC.pitch = data.getUint16(2, 1);
                    RC.roll = data.getUint16(0, 1);
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
            SENSOR_DATA.kinematicsX = data.getInt16(0, 1) / 10.0;
            SENSOR_DATA.kinematicsY = data.getInt16(2, 1) / 10.0;
            SENSOR_DATA.kinematicsZ = data.getInt16(4, 1);
            break;
        case MSP_codes.MSP_ALTITUDE:
            SENSOR_DATA.altitude = parseFloat((data.getInt32(0, 1) / 100.0).toFixed(2)); // correct scale factor
            break;
        case MSP_codes.MSP_ANALOG:
            ANALOG.voltage = data.getUint8(0) / 10.0;
            ANALOG.power = data.getUint16(1, 1);
            ANALOG.rssi = data.getUint16(3, 1); // 0-1023
            ANALOG.amperage = data.getUint16(5, 1);
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
            // dump previous data (if there was any)
            AUX_CONFIG_values = new Array();

            // fill in current data
            for (var i = 0; i < data.byteLength; i += 2) { // + 2 because uint16_t = 2 bytes
                AUX_CONFIG_values.push(data.getUint16(i, 1));
            }
            break;
        case MSP_codes.MSP_MISC: // 22 bytes
            MISC.PowerTrigger1 = data.getInt16(0, 1);
            MISC.minthrottle = data.getUint16(2, 1); // 0-2000
            MISC.maxthrottle = data.getUint16(4, 1); // 0-2000
            MISC.mincommand = data.getUint16(6, 1); // 0-2000
            MISC.failsafe_throttle = data.getUint16(8, 1); // 1000-2000
            MISC.plog0 = data.getUint16(10, 1);
            MISC.plog1 = data.getUint32(12, 1);
            MISC.mag_declination = data.getInt16(16, 1); // -18000-18000
            MISC.vbatscale = data.getUint8(18, 1); // 10-200
            MISC.vbatmincellvoltage = data.getUint8(19, 1) / 10; // 10-50
            MISC.vbatmaxcellvoltage = data.getUint8(20, 1) / 10; // 10-50
            MISC.empty = data.getUint8(21, 1);
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
            console.log(data);
            break;
        case MSP_codes.MSP_SERVO_CONF:
            // drop previous data
            SERVO_CONFIG = [];

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

            // With new flight software settings in place, we have to re-pull
            // latest values
            send_message(MSP_codes.MSP_IDENT);
            send_message(MSP_codes.MSP_STATUS);
            send_message(MSP_codes.MSP_PID);
            send_message(MSP_codes.MSP_RC_TUNING);
            send_message(MSP_codes.MSP_BOXNAMES);
            send_message(MSP_codes.MSP_BOX);

            // baseflight specific
            send_message(MSP_codes.MSP_UID);
            send_message(MSP_codes.MSP_ACC_TRIM);
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
};

function send_message(code, data, callback_sent, callback_msp) {
    var bufferOut;
    var bufView;

    // always reserve 6 bytes for protocol overhead !
    if (data) {
        var size = data.length + 6;
        var checksum = 0;

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

    // utilizing callback/timeout system for all commands
    for (var i = 0; i < MSP.callbacks.length; i++) {
        if (MSP.callbacks[i].code == code) {
            // request already exist
            return false; // skips the code below
        }
    }

    var obj = {'code': code, 'callback': (callback_msp) ? callback_msp : false};
    obj.timer = setInterval(function() {
        console.log('MSP data request timed-out: ' + code);

        serial.send(bufferOut, function(sendInfo) {});
    }, 1000); // we should be able to define timeout in the future

    MSP.callbacks.push(obj);

    serial.send(bufferOut, function(sendInfo) {
        if (sendInfo.bytesSent > 0) {
            if (callback_sent) callback_sent();
        }
    });

    return true;
}