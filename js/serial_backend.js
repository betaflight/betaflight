var connectionId = -1;
var connection_delay = 0; // delay which defines "when" will the configurator request configurator data after connection was established

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
            send_message(MSP_codes.MSP_ACC_CALIBRATION, 205);
            
        }, connection_delay * 1000);
    }
}

function onClosed(result) {
    if (result) { // All went as expected
        connectionId = -1; // reset connection id
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
                   
                    // setup arraybuffer
                    message_buffer = new ArrayBuffer(message_length_expected);
                    message_buffer_uint8_view = new Uint8Array(message_buffer);
                    
                    message_state++;
                    break;
                case 4:
                    message_code = data[i]; // code
                    
                    message_state++;
                    break;
                case 5: // data / payload
                    message_buffer_uint8_view[message_length_received] = data[i];
                    message_length_received++;
                    
                    if (message_length_received >= message_length_expected) {
                        // message received, process
                        console.log(message_code);
                        console.log(message_buffer_uint8_view);
                        
                        // Reset variables
                        message_length_received = 0;
                        message_state = 0;                        
                    }
                    break;
            }
        }
    }        
}

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
        
        checksum = bufView[3] | bufView[4];
        for (var i = 5; i < data.length; i++) {
            bufView[i] = data[i - 5];
            
            checksum |= bufView[i];
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
        bufView[6] = bufView[3] | bufView[4] | bufView[5]; // checksum
    }

    chrome.serial.write(connectionId, bufferOut, function(writeInfo) {
        console.log("Wrote: " + writeInfo.bytesWritten + " bytes");
    });    
}