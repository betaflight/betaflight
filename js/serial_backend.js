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
    mode:          0,
    gyroscope:     [0, 0, 0],
    accelerometer: [0, 0, 0],
    magnetometer:  [0, 0, 0],
    altitude:      0
};

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
                chrome.serial.close(connectionId, onClosed);
                
                clearTimeout(connection_delay);
                clearInterval(serial_poll);
                
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
            
            // should request some sort of configuration data
            //send_message(MSP_codes.MSP_ACC_CALIBRATION, MSP_codes.MSP_ACC_CALIBRATION);
            send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
            send_message(MSP_codes.MSP_IDENT, MSP_codes.MSP_IDENT);
            //send_message(MSP_codes.MSP_BOXNAMES, MSP_codes.MSP_BOXNAMES);
            
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

function onCharRead(readInfo) {
    if (readInfo && readInfo.bytesRead > 0 && readInfo.data) {
        var data = new Uint8Array(readInfo.data);
        
        for (var i = 0; i < data.length; i++) {
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
        }
    }
}

function send_message(code, data, bytes_n) {    
    if (typeof data === 'array') { // array portion of this code is untested
        var size = 6 + data.length;
        var checksum = 0;
        
        var bufferOut = new ArrayBuffer(size);
        var bufView = new Uint8Array(bufferOut);        
        
        bufView[0] = 36; // $
        bufView[1] = 77; // M
        bufView[2] = 60; // <
        bufView[3] = data.length; // data length
        bufView[4] = code; // code
        
        checksum = bufView[3] ^ bufView[4];
        for (var i = 5; i < data.length; i++) {
            bufView[i] = data[i - 5];
            
            checksum ^= bufView[i];
        }

        bufView[5 + data.length] = checksum;
    } else {
        var bufferOut = new ArrayBuffer(7);
        var bufView = new Uint8Array(bufferOut);
        
        bufView[0] = 36; // $
        bufView[1] = 77; // M
        bufView[2] = 60; // <
        bufView[3] = bytes_n; // data length
        bufView[4] = code; // code
        bufView[5] = data; // data
        bufView[6] = bufView[3] ^ bufView[4] ^ bufView[5]; // checksum
    }

    chrome.serial.write(connectionId, bufferOut, function(writeInfo) {
        console.log("Wrote: " + writeInfo.bytesWritten + " bytes");
    });    
}

function process_message(code, data) {
    var view = new DataView(data, 0);
    
    switch (code) {
        case MSP_codes.MSP_IDENT:
            CONFIG.version = view.getUint8(0);
            CONFIG.multiType = view.getUint8(1);
            
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
            CONFIG.accelerometer[0] = view.getInt16(0, 1);
            CONFIG.accelerometer[1] = view.getInt16(2, 1);
            CONFIG.accelerometer[2] = view.getInt16(4, 1);
            
            CONFIG.gyroscope[0] = view.getInt16(6, 1) / 8;
            CONFIG.gyroscope[1] = view.getInt16(8, 1) / 8;
            CONFIG.gyroscope[2] = view.getInt16(10, 1) / 8;

            CONFIG.magnetometer[0] = view.getInt16(12, 1) / 3;
            CONFIG.magnetometer[1] = view.getInt16(14, 1) / 3;
            CONFIG.magnetometer[2] = view.getInt16(16, 1) / 3;
            break;
        case MSP_codes.MSP_SERVO:
            console.log(data);
            break; 
        case MSP_codes.MSP_MOTOR:
            console.log(data);        
            break; 
        case MSP_codes.MSP_RC:
            console.log(data);
            break; 
        case MSP_codes.MSP_RAW_GPS:
            console.log(data);
            break; 
        case MSP_codes.MSP_COMP_GPS:
            console.log(data);
            break; 
        case MSP_codes.MSP_ATTITUDE:
            console.log(data);
            break; 
        case MSP_codes.MSP_ALTITUDE:
            CONFIG.altitude = view.getUint32(0, 1);
            break; 
        case MSP_codes.MSP_BAT:
            console.log(data);
            break; 
        case MSP_codes.MSP_RC_TUNING:
            console.log(data);
            break; 
        case MSP_codes.MSP_PID:
            console.log(data);
            break; 
        case MSP_codes.MSP_BOX:
            console.log(data);
            break; 
        case MSP_codes.MSP_MISC:
            console.log(data);
            break; 
        case MSP_codes.MSP_MOTOR_PINS:
            console.log(data);
            break; 
        case MSP_codes.MSP_BOXNAMES:
            console.log(data);
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
            console.log(data);
            break; 
        case MSP_codes.MSP_SET_BOX:
            console.log(data);
            break; 
        case MSP_codes.MSP_SET_RC_TUNING:
            console.log(data);
            break;  
        case MSP_codes.MSP_ACC_CALIBRATION:
            console.log('Accel calibration finished');
            break;  
        case MSP_codes.MSP_MAG_CALIBRATION:
            console.log('Mag calibration finished');
            break;  
        case MSP_codes.MSP_SET_MISC:
            console.log(data);
            break;  
        case MSP_codes.MSP_RESET_CONF:
            console.log(data);
            break;  
        case MSP_codes.MSP_SELECT_SETTING:
            console.log(data);
            break;  
        case MSP_codes.MSP_BIND:
            console.log(data);
            break;  
        case MSP_codes.MSP_EEPROM_WRITE:
            console.log(data);
            break;  
        case MSP_codes.MSP_DEBUGMSG:
            console.log(data);
            break;  
        case MSP_codes.MSP_DEBUG:
            console.log(data);
            break;               
    }
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