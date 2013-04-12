var connectionId = -1;
var connection_delay = 0; // delay which defines "when" will the configurator request configurator data after connection was established

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
    MSP_BAT:                110,
    MSP_RC_TUNING:          111,
    MSP_PID:                112,
    MSP_BOX:                113,
    MSP_MISC:               114,
    MSP_MOTOR_PINS:         115,
    MSP_BOXNAMES:           116,
    MSP_PIDNAMES:           117,
    
    MSP_SET_RAW_RC:         200,
    MSP_SET_RAW_GPS:        201,
    MSP_SET_PID:            202,
    MSP_SET_BOX:            203,
    MSP_SET_RC_TUNING:      204,
    MSP_ACC_CALIBRATION:    205,
    MSP_MAG_CALIBRATION:    206,
    MSP_SET_MISC:           207,
    MSP_RESET_CONF:         208,
    MSP_SELECT_SETTING:     210,
    
    MSP_BIND:               240,
    
    MSP_EEPROM_WRITE:       250,
    
    MSP_DEBUGMSG:           253,
    MSP_DEBUG:              254
};

var CONFIG = {
    version:       0,
    multiType:     0,
    cycleTime:     0,
    i2cError:      0,
    activeSensors: 0,
    mode:          0
};

var PIDs = new Array(10);
for (var i = 0; i < 10; i++) {
    PIDs[i] = new Array(3);
}

var RC = {
    roll:     0,
    pitch:    0,
    yaw:      0,
    throttle: 0,
    AUX1:     0,
    AUX2:     0,
    AUX3:     0,
    AUX4:     0
};

var RC_tuning = {
    RC_RATE:         0,
    RC_EXPO:         0,
    roll_pitch_rate: 0,
    yaw_rate:        0,
    dynamic_THR_PID: 0,
    throttle_MID:    0,
    throttle_EXPO:   0,
};

var AUX_CONFIG = new Array();
var AUX_CONFIG_values = new Array();

var SENSOR_DATA = {
    gyroscope:     [0, 0, 0],
    accelerometer: [0, 0, 0],
    magnetometer:  [0, 0, 0],
    altitude:      0,
    kinematicsX:   0.0,
    kinematicsY:   0.0,
    kinematicsZ:   0.0
};

var MOTOR_DATA = new Array(8);
var SERVO_DATA = new Array(8);

var GPS_DATA = {
    fix:             0,
    numSat:          0,
    lat:             0,
    lon:             0,
    alt:             0,
    speed:           0,
    distanceToHome:  0,
    ditectionToHome: 0,
    update:          0
};

var CLI_active = false;

$(document).ready(function() { 
    port_picker = $('div#port-picker .port select');
    baud_picker = $('div#port-picker #baud');
    delay_picker = $('div#port-picker #delay');
    
    $('div#port-picker a.refresh').click(function() {
        console.log("Available port list requested.");
        port_picker.html('');

        chrome.serial.getPorts(function(ports) {
            if (ports.length > 0) {
                // Port list received
                
                ports.forEach(function(port) {
                    $(port_picker).append($("<option/>", {
                        value: port,
                        text: port
                    }));        
                });
            } else {
                $(port_picker).append($("<option/>", {
                    value: 0,
                    text: 'NOT FOUND'
                }));
                
                console.log("No serial ports detected");
            }
        });
    });
    
    // software click to refresh port picker select (during initial load)
    $('div#port-picker a.refresh').click();
    
    $('div#port-picker a.connect').click(function() {
        var clicks = $(this).data('clicks');
        
        selected_port = String($(port_picker).val());
        selected_baud = parseInt(baud_picker.val());
        connection_delay = parseInt(delay_picker.val());
        
        if (selected_port != '0') {
            if (clicks) { // odd number of clicks
                // Disable any active "data pulling" timer
                disable_timers();

                // Disable CLI (there is no "nicer way of doing so right now)
                if (CLI_active == true) {
                    leave_CLI(function() {
                        chrome.serial.close(connectionId, onClosed);
                        
                        clearTimeout(connection_delay);
                        clearInterval(serial_poll);
                        clearInterval(port_usage_poll);
                    });
                } else {               
                    chrome.serial.close(connectionId, onClosed);
                    
                    clearTimeout(connection_delay);
                    clearInterval(serial_poll);
                    clearInterval(port_usage_poll);
                }
                
                // Change port utilization to 0
                $('span.port-usage').html('0%');

                $(this).text('Connect');
                $(this).removeClass('active');                
            } else { // even number of clicks        
                console.log('Connecting to: ' + selected_port);
                
                chrome.serial.open(selected_port, {
                    bitrate: selected_baud
                }, onOpen);
                
                $(this).text('Disconnect');  
                $(this).addClass('active');
            }
            
            $(this).data("clicks", !clicks);
        }
    });     
}); 

function onOpen(openInfo) {
    connectionId = openInfo.connectionId;
    
    if (connectionId != -1) {
        console.log('Connection was opened with ID: ' + connectionId);
        
        connection_delay = setTimeout(function() {
            // start polling
            serial_poll = setInterval(readPoll, 10);
            port_usage_poll = setInterval(port_usage, 1000);
            
            // request configuration data
            send_message(MSP_codes.MSP_IDENT, MSP_codes.MSP_IDENT);
            send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
            send_message(MSP_codes.MSP_PID, MSP_codes.MSP_PID);
            send_message(MSP_codes.MSP_RC_TUNING, MSP_codes.MSP_RC_TUNING);
            send_message(MSP_codes.MSP_BOXNAMES, MSP_codes.MSP_BOXNAMES);
            send_message(MSP_codes.MSP_BOX, MSP_codes.MSP_BOX);
            
        }, connection_delay * 1000);
    }
}

function onClosed(result) {
    if (result) { // All went as expected
        connectionId = -1; // reset connection id
        
        sensor_status(sensors_detected = 0); // reset active sensor indicators
        $('#content').empty(); // empty content
        $('#tabs > ul li').removeClass('active'); // de-select any selected tabs
        
        console.log('Connection closed successfully.');
    } else { // Something went wrong
        if (connectionId > 0) {
            console.log('There was an error that happened during "connection-close" procedure.');
        }
    } 
}

function readPoll() {
    chrome.serial.read(connectionId, 128, onCharRead);
}   

var message_state = 0;
var message_status = 1;
var message_code = 0;
var message_length_expected = 0;
var message_length_received = 0;
var message_buffer;
var message_buffer_uint8_view;
var message_checksum = 0;
var char_counter = 0;

function onCharRead(readInfo) {
    if (readInfo && readInfo.bytesRead > 0 && readInfo.data) {
        var data = new Uint8Array(readInfo.data);
        
        for (var i = 0; i < data.length; i++) {
            if (CLI_active != true) {
                // Standard "GUI" MSP handling
                switch (message_state) {
                    case 0: // sync char 1
                        if (data[i] == 36) { // $
                            message_state++;
                        }
                        break;
                    case 1: // sync char 2
                        if (data[i] == 77) { // M
                            message_state++;
                        } else { // restart and try again
                            message_state = 0;
                        }
                        break;
                    case 2: // direction (should be >)
                        if (data[i] == 62) { // >
                            message_status = 1;
                        } else { // unknown
                            message_status = 0;
                        }
                        
                        message_state++;
                        break;
                    case 3:
                        message_length_expected = data[i]; // data length
                       
                        message_checksum = data[i];
                       
                        // setup arraybuffer
                        message_buffer = new ArrayBuffer(message_length_expected);
                        message_buffer_uint8_view = new Uint8Array(message_buffer);
                        
                        message_state++;
                        break;
                    case 4:
                        message_code = data[i]; // code
                        message_checksum ^= data[i];
                        
                        if (message_length_expected != 0) { // standard message
                            message_state++;
                        } else { // MSP_ACC_CALIBRATION, etc...
                            message_state += 2;
                        }
                        break;
                    case 5: // data / payload
                        message_buffer_uint8_view[message_length_received] = data[i];
                        message_checksum ^= data[i];
                        message_length_received++;
                        
                        if (message_length_received >= message_length_expected) {
                            message_state++;
                        }
                        break;
                    case 6: // CRC
                        if (message_checksum == data[i]) {
                            // process data
                            process_message(message_code, message_buffer);
                        }
                        
                        // Reset variables
                        message_length_received = 0;
                        message_state = 0;                   
                        break;
                }
            } else {
                // CLI Enabled (Terminal "style" handling)
                handle_CLI(data[i]);
            }
            
            char_counter++;
        }
    }
}

function send_message(code, data) {        
    if (typeof data === 'object') {
        var size = 6 + data.length; // 6 bytes for protocol overhead
        var checksum = 0;
        
        var bufferOut = new ArrayBuffer(size);
        var bufView = new Uint8Array(bufferOut);        
        
        bufView[0] = 36; // $
        bufView[1] = 77; // M
        bufView[2] = 60; // <
        bufView[3] = data.length; // data length
        bufView[4] = code; // code
        
        checksum = bufView[3] ^ bufView[4];
        
        for (var i = 0; i < data.length; i++) {
            bufView[i + 5] = data[i];
            
            checksum ^= bufView[i + 5];
        }

        bufView[5 + data.length] = checksum;
    } else {
        var bufferOut = new ArrayBuffer(7);
        var bufView = new Uint8Array(bufferOut);
        
        bufView[0] = 36; // $
        bufView[1] = 77; // M
        bufView[2] = 60; // <
        bufView[3] = 0; // data length
        bufView[4] = code; // code
        bufView[5] = data; // data
        bufView[6] = bufView[3] ^ bufView[4] ^ bufView[5]; // checksum
    }

    chrome.serial.write(connectionId, bufferOut, function(writeInfo) {
        // used for debugging purposes (should be disabled in "stable" builds
        //console.log("Wrote: " + writeInfo.bytesWritten + " bytes");
    });    
}

function process_message(code, data) {
    var view = new DataView(data, 0);
    
    switch (code) {
        case MSP_codes.MSP_IDENT:
            CONFIG.version = parseFloat((view.getUint8(0) / 100).toFixed(2));
            CONFIG.multiType = view.getUint8(1);
            
            $('.software-version').html(CONFIG.version);
            
            // TODO: utilize this info
            switch (CONFIG.multiType) {
                case 1: // TRI
                    break;
                case 2: // QUAD+
                    break;
                case 3: // QUAD X
                    break;
                case 4: // BI
                    break;
                case 5: // GIMBAL
                    break;
                case 6: // Y6
                    break;
                case 7: // HEX 6
                    break;
                case 8: // FLYING_WING
                    break;
                case 9: // Y4
                    break;
                case 10: // HEX6 X
                    break;
                case 11: // OCTO X8
                case 12:
                case 13:
                    break;
                case 14: // AIRPLANE
                    break;
                case 15: // Heli 120 
                    break;
                case 16: // Heli 90 
                    break;
                case 17: // Vtail 
                    break;
                case 18: // HEX6 H
                    break;
            }
            
            // IDENT received, show the tab content
            $('#tabs li a:first').click();
            break;
        case MSP_codes.MSP_STATUS:
            CONFIG.cycleTime = view.getUint16(0, 1);
            CONFIG.i2cError = view.getUint16(2, 1);
            CONFIG.activeSensors = view.getUint16(4, 1);
            CONFIG.mode = view.getUint32(6, 1);
            
            sensor_status(CONFIG.activeSensors);
            break;
        case MSP_codes.MSP_RAW_IMU:
            SENSOR_DATA.accelerometer[0] = view.getInt16(0, 1) / 1000; // properly scaled
            SENSOR_DATA.accelerometer[1] = view.getInt16(2, 1) / 1000;
            SENSOR_DATA.accelerometer[2] = view.getInt16(4, 1) / 1000;
            
            SENSOR_DATA.gyroscope[0] = view.getInt16(6, 1) / 8; // no clue about scaling factor
            SENSOR_DATA.gyroscope[1] = view.getInt16(8, 1) / 8;
            SENSOR_DATA.gyroscope[2] = view.getInt16(10, 1) / 8;

            SENSOR_DATA.magnetometer[0] = view.getInt16(12, 1) / 3; // no clue about scaling factor
            SENSOR_DATA.magnetometer[1] = view.getInt16(14, 1) / 3;
            SENSOR_DATA.magnetometer[2] = view.getInt16(16, 1) / 3;
            break;
        case MSP_codes.MSP_SERVO:
            var needle = 0;
            for (var i = 0; i < SERVO_DATA.length; i++) {
                SERVO_DATA[i] = view.getUint16(needle, 1);
                
                needle += 2;
            } 
            break; 
        case MSP_codes.MSP_MOTOR:
            var needle = 0;
            for (var i = 0; i < MOTOR_DATA.length; i++) {
                MOTOR_DATA[i] = view.getUint16(needle, 1);
                
                needle += 2;
            }            
            break; 
        case MSP_codes.MSP_RC:
            RC.roll = view.getUint16(0, 1);
            RC.pitch = view.getUint16(2, 1);
            RC.yaw = view.getUint16(4, 1);
            RC.throttle = view.getUint16(6, 1);
            
            RC.AUX1 = view.getUint16(8, 1);
            RC.AUX2 = view.getUint16(10, 1);
            RC.AUX3 = view.getUint16(12, 1);
            RC.AUX4 = view.getUint16(14, 1);
            break; 
        case MSP_codes.MSP_RAW_GPS:
            GPS_DATA.fix = view.getUint8(0);
            GPS_DATA.numSat = view.getUint8(1);
            GPS_DATA.lat = view.getUint32(2, 1);
            GPS_DATA.lon = view.getUint32(6, 1);
            GPS_DATA.alt = view.getUint16(10, 1);
            GPS_DATA.speed = view.getUint16(12, 1);
            break; 
        case MSP_codes.MSP_COMP_GPS:
            GPS_DATA.distanceToHome = view.getUint16(0, 1);
            GPS_DATA.directionToHome = view.getUint16(2, 1);
            GPS_DATA.update = view.getUint8(4);
            break; 
        case MSP_codes.MSP_ATTITUDE:
            SENSOR_DATA.kinematicsX = view.getInt16(0, 1) / 10.0;
            SENSOR_DATA.kinematicsY = view.getInt16(2, 1) / 10.0;
            SENSOR_DATA.kinematicsZ = view.getInt16(4, 1);
            break; 
        case MSP_codes.MSP_ALTITUDE:
            SENSOR_DATA.altitude = view.getUint32(0, 1) / 100.0; // correct scale factor
            break; 
        case MSP_codes.MSP_BAT:
            console.log(data);
            break; 
        case MSP_codes.MSP_RC_TUNING:
            RC_tuning.RC_RATE = parseFloat((view.getUint8(0) / 100).toFixed(2));
            RC_tuning.RC_EXPO = parseFloat((view.getUint8(1) / 100).toFixed(2));
            RC_tuning.roll_pitch_rate = parseFloat((view.getUint8(2) / 100).toFixed(2));
            RC_tuning.yaw_rate = parseFloat((view.getUint8(3) / 100).toFixed(2));
            RC_tuning.dynamic_THR_PID = parseFloat((view.getUint8(4) / 100).toFixed(2));
            RC_tuning.throttle_MID = parseFloat((view.getUint8(5) / 100).toFixed(2));
            RC_tuning.throttle_EXPO = parseFloat((view.getUint8(6) / 100).toFixed(2));
            break; 
        case MSP_codes.MSP_PID:
            // PID data arrived, we need to scale it and save to appropriate bank / array
            var needle = 0;
            for (var i = 0; i < 10; i++) {
                // main for loop selecting the pid section 
                switch (i) {
                    case 0: 
                    case 1: 
                    case 2: 
                    case 3: 
                    case 7: 
                    case 8:
                    case 9:
                        PIDs[i][0] = view.getUint8(needle) / 10;
                        PIDs[i][1] = view.getUint8(needle + 1) / 1000;
                        PIDs[i][2] = view.getUint8(needle + 2);
                        break;
                    case 4:
                        PIDs[i][0] = view.getUint8(needle) / 100;
                        PIDs[i][1] = view.getUint8(needle + 1) / 100;
                        PIDs[i][2] = view.getUint8(needle + 2) / 1000;
                        break;
                    case 5: 
                    case 6:
                        PIDs[i][0] = view.getUint8(needle) / 10;
                        PIDs[i][1] = view.getUint8(needle + 1) / 100;
                        PIDs[i][2] = view.getUint8(needle + 2) / 1000;
                        break;                     
                }
                needle += 3;
            }
            break; 
        case MSP_codes.MSP_BOX:
            for (var i = 0; i < data.byteLength; i += 2) { // + 2 because uint16_t = 2 bytes
                AUX_CONFIG_values.push(view.getUint16(i, 1));
            }
            break; 
        case MSP_codes.MSP_MISC:
            console.log(data);
            break; 
        case MSP_codes.MSP_MOTOR_PINS:
            console.log(data);
            break; 
        case MSP_codes.MSP_BOXNAMES:
            AUX_CONFIG = []; // empty the array as new data is coming in
            
            var buff = new Array();
            for (var i = 0; i < data.byteLength; i++) {
                if (view.getUint8(i) == 0x3B) { // ; (delimeter char)
                    AUX_CONFIG.push(String.fromCharCode.apply(null, buff)); // convert bytes into ASCII and save as strings
                    
                    // empty buffer
                    buff = [];
                } else {
                    buff.push(view.getUint8(i));
                }
                
            }
            break; 
        case MSP_codes.MSP_PIDNAMES:
            console.log(data);
            break; 
        case MSP_codes.MSP_SET_RAW_RC:
            console.log(data);
            break; 
        case MSP_codes.MSP_SET_RAW_GPS:
            console.log(data);
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
            console.log(data);
            break;  
        case MSP_codes.MSP_RESET_CONF:
            console.log('Settings Reset');
            
            // With new flight software settings in place, we have to re-pull
            // latest values
            send_message(MSP_codes.MSP_IDENT, MSP_codes.MSP_IDENT);
            send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
            send_message(MSP_codes.MSP_PID, MSP_codes.MSP_PID);
            send_message(MSP_codes.MSP_RC_TUNING, MSP_codes.MSP_RC_TUNING);
            send_message(MSP_codes.MSP_BOXNAMES, MSP_codes.MSP_BOXNAMES);
            send_message(MSP_codes.MSP_BOX, MSP_codes.MSP_BOX);
            break;  
        case MSP_codes.MSP_SELECT_SETTING:
            console.log(data);
            break;  
        case MSP_codes.MSP_BIND:
            console.log(data);
            break;  
        case MSP_codes.MSP_EEPROM_WRITE:
            console.log('Settings Saved in EEPROM');
            break;  
        case MSP_codes.MSP_DEBUGMSG:
            console.log(data);
            break;  
        case MSP_codes.MSP_DEBUG:
            console.log(data);
            break;               
    }
}

function port_usage() {
    var port_usage = (char_counter * 10 / selected_baud) * 100;    
    $('span.port-usage').html(parseInt(port_usage) + '%');

    // reset counter
    char_counter = 0;
}

function sensor_status(sensors_detected) {
    var e_sensor_status = $('div#sensor-status');
    
    if (bit_check(sensors_detected, 0)) { // Gyroscope & accel detected
        $('.gyro', e_sensor_status).addClass('on');
        $('.accel', e_sensor_status).addClass('on');
    } else {
        $('.gyro', e_sensor_status).removeClass('on');
        $('.accel', e_sensor_status).removeClass('on');
    }

    if (bit_check(sensors_detected, 1)) { // Barometer detected
        $('.baro', e_sensor_status).addClass('on');
    } else {
        $('.baro', e_sensor_status).removeClass('on');
    } 

    if (bit_check(sensors_detected, 2)) { // Magnetometer detected
        $('.mag', e_sensor_status).addClass('on');
    } else {
        $('.mag', e_sensor_status).removeClass('on');
    } 

    if (bit_check(sensors_detected, 3)) { // GPS detected
        $('.gps', e_sensor_status).addClass('on');
    } else {
        $('.gps', e_sensor_status).removeClass('on');
    } 

    if (bit_check(sensors_detected, 4)) { // Sonar detected
        $('.sonar', e_sensor_status).addClass('on');
    } else {
        $('.sonar', e_sensor_status).removeClass('on');
    }     
}

function highByte(num) {
    return num >> 8;
}

function lowByte(num) {
    return 0x00FF & num;
}

function bit_check(num, bit) {
    return ((num >> bit) % 2 != 0)
}

function bit_set(num, bit) {
    return num | 1 << bit;
}

function bit_clear(num, bit) {
    return num & ~(1 << bit);
}