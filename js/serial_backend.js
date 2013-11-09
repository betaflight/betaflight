var connectionId = -1;
var connection_delay = 0; // delay which defines "when" will the configurator request configurator data after connection was established
var configuration_received = false;

var CONFIG = {
    version:       0,
    multiType:     0,
    cycleTime:     0,
    i2cError:      0,
    activeSensors: 0,
    mode:          0,
    profile:       0,
    
    uid:           [0, 0, 0],
    accelerometerTrims: [0, 0]
}

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
}

var RC_tuning = {
    RC_RATE:         0,
    RC_EXPO:         0,
    roll_pitch_rate: 0,
    yaw_rate:        0,
    dynamic_THR_PID: 0,
    throttle_MID:    0,
    throttle_EXPO:   0,
}

var AUX_CONFIG = new Array();
var AUX_CONFIG_values = new Array();

var SERVO_CONFIG = new Array();

var SENSOR_DATA = {
    gyroscope:     [0, 0, 0],
    accelerometer: [0, 0, 0],
    magnetometer:  [0, 0, 0],
    altitude:      0,
    kinematicsX:   0.0,
    kinematicsY:   0.0,
    kinematicsZ:   0.0,
    debug:        [0, 0, 0, 0]
}

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
    update:          0,
    
    // baseflight specific gps stuff
    chn:     new Array(),
    svid:    new Array(),
    quality: new Array(),
    cno:     new Array()
}

var BATTERY = {
    voltage:   0,
    pMeterSum: 0,
}

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
                
                chrome.storage.local.get('last_used_port', function(result) {
                    // if last_used_port was set, we try to select it
                    if (typeof result.last_used_port != 'undefined') {
                        // check if same port exists, if it does, select it
                        ports.forEach(function(port) {
                            if (port == result.last_used_port) {
                                $(port_picker).val(result.last_used_port);
                            }
                        });
                    }
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

                // reset valid config received variable (used to block tabs while not connected properly)
                configuration_received = false;
                
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
    backgroundPage.connectionId = openInfo.connectionId; // also pass connectionId to the background page
    
    if (connectionId != -1) {
        console.log('Connection was opened with ID: ' + connectionId);
        
        // save selected port with chrome.storage if the port differs
        chrome.storage.local.get('last_used_port', function(result) {
            if (typeof result.last_used_port != 'undefined') {
                if (result.last_used_port != selected_port) {
                    // last used port doesn't match the one found in local db, we will store the new one
                    chrome.storage.local.set({'last_used_port': selected_port}, function() {
                        // Debug message is currently disabled (we dont need to spam the console log with that)
                        // console.log('Last selected port was saved in chrome.storage.');
                    });
                }
            } else {
                // variable isn't stored yet, saving
                chrome.storage.local.set({'last_used_port': selected_port}, function() {
                    // Debug message is currently disabled (we dont need to spam the console log with that)
                    // console.log('Last selected port was saved in chrome.storage.');
                });
            }
        });

        connection_delay = setTimeout(function() {
            // start polling
            serial_poll = setInterval(readPoll, 10);
            port_usage_poll = setInterval(port_usage, 1000);
 
            // baseflight specific
            send_message(MSP_codes.MSP_UID, MSP_codes.MSP_UID);
            send_message(MSP_codes.MSP_ACC_TRIM, MSP_codes.MSP_ACC_TRIM);
 
            // request configuration data
            send_message(MSP_codes.MSP_IDENT, MSP_codes.MSP_IDENT);
            send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
            send_message(MSP_codes.MSP_PID, MSP_codes.MSP_PID);
            send_message(MSP_codes.MSP_RC_TUNING, MSP_codes.MSP_RC_TUNING);
            send_message(MSP_codes.MSP_BOXNAMES, MSP_codes.MSP_BOXNAMES);
            send_message(MSP_codes.MSP_BOX, MSP_codes.MSP_BOX);
            
        }, connection_delay * 1000);
    } else {
        console.log('Failed to open serial port');
        notify('Failed to open serial port', 'red');
        
        $('div#port-picker a.connect').text('Connect');
        $('div#port-picker a.connect').removeClass('active'); 
        
        // reset data
        $('div#port-picker a.connect').data("clicks", false);
    }
}

function onClosed(result) {
    if (result) { // All went as expected
        connectionId = -1; // reset connection id
        backgroundPage.connectionId = connectionId; // also pass latest connectionId to the background page
        
        sensor_status(sensors_detected = 0); // reset active sensor indicators
        $('#tabs > ul li').removeClass('active'); // de-select any selected tabs
        tab_initialize_default();
        
        console.log('Connection closed successfully.');
    } else { // Something went wrong
        if (connectionId > 0) {
            console.log('There was an error that happened during "connection-close" procedure.');
            notify('Failed to close serial port', 'red');
        }
    } 
}

function readPoll() {
    chrome.serial.read(connectionId, 128, MSP_char_read);
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