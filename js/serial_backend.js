var configuration_received = false;
var CLI_active = false;
var CLI_valid = false;

var CONFIG = {
    version:       0,
    multiType:     0,
    msp_version:   0,
    capability:    0,
    cycleTime:     0,
    i2cError:      0,
    activeSensors: 0,
    mode:          0,
    profile:       0,
    
    uid:           [0, 0, 0],
    accelerometerTrims: [0, 0]
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
    ground_course:   0,
    distanceToHome:  0,
    ditectionToHome: 0,
    update:          0,
    
    // baseflight specific gps stuff
    chn:     new Array(),
    svid:    new Array(),
    quality: new Array(),
    cno:     new Array()
};

var BATTERY = {
    voltage:   0,
    pMeterSum: 0,
};

var MISC = {
    PowerTrigger1:      0, // intPowerTrigger1 (aka useless trash)
    minthrottle:        0,
    maxthrottle:        0,
    mincommand:         0,
    failsafe_throttle:  0,
    plog0:              0, // plog useless shit
    plog1:              0, // plog useless shit
    mag_declination:    0, // not checked
    vbatscale:          0,
    vbatmincellvoltage: 0,
    vbatmaxcellvoltage: 0,
    empty:              0 // unknown
};

$(document).ready(function() {    
    console.log('Scanning for new ports...');
    update_ports();
    
    $('div#port-picker a.connect').click(function() {
        if (GUI.connect_lock != true) { // GUI control overrides the user control
            var clicks = $(this).data('clicks');
            
            var selected_port = String($('div#port-picker .port select').val());
            var selected_baud = parseInt($('div#port-picker #baud').val());
            
            if (selected_port != '0') {
                if (!clicks) {
                    console.log('Connecting to: ' + selected_port);
                    GUI.connecting_to = selected_port;
                    
                    // lock port select & baud while we are connecting / connected
                    $('div#port-picker #port, div#port-picker #baud, div#port-picker #delay').prop('disabled', true);
                    $('div#port-picker a.connect').text('Connecting'); 
                    
                    serial.connect(selected_port, {bitrate: selected_baud}, onOpen);
                } else {
                    // Disable any active "data pulling" timer
                    GUI.interval_kill_all(['port-update']);
                    
                    GUI.tab_switch_cleanup();
                    GUI.timeout_remove('connecting');
                    
                    // remove listeners
                    serial.onReceive.listeners_.forEach(function(listener) {
                        serial.onReceive.removeListener(listener.callback);
                    });
                    
                    serial.disconnect(onClosed);
                    
                    GUI.connected_to = false;
                    
                    // Reset various UI elements
                    $('span.port-usage').html('0%');
                    $('.software-version').html('0.0');
                    $('span.cycle-time').html('0');
                    
                    MSP.state = 0; // reset packet state for "clean" initial entry (this is only required if user hot-disconnects)
                    MSP.packet_error = 0; // reset CRC packet error counter for next session
                    configuration_received = false; // reset valid config received variable (used to block tabs while not connected properly)
                    
                    // unlock port select & baud
                    $('div#port-picker #port').prop('disabled', false);
                    if (!GUI.auto_connect) $('div#port-picker #baud').prop('disabled', false);
                    
                    $(this).text('Connect');
                    $(this).removeClass('active');                 
                }
                
                $(this).data("clicks", !clicks);
            }
        }
    });

    // auto-connect
    chrome.storage.local.get('auto_connect', function(result) {
        if (typeof result.auto_connect === 'undefined') {
            // auto_connect wasn't saved yet, save and push true to the GUI
            chrome.storage.local.set({'auto_connect': true});
            
            GUI.auto_connect = true;
            $('select#baud').val(115200).prop('disabled', true);
        } else {
            if (result.auto_connect) { 
                // enabled by user
                GUI.auto_connect = true;
                
                $('input.auto_connect').prop('checked', true);
                $('input.auto_connect, span.auto_connect').prop('title', 'Auto-Connect: Enabled - Configurator automatically tries to connect when new serial port is detected');
                
                $('select#baud').val(115200).prop('disabled', true);
            } else { 
                // disabled by user
                GUI.auto_connect = false;
                
                $('input.auto_connect').prop('checked', false);
                $('input.auto_connect, span.auto_connect').prop('title', 'Auto-Connect: Disabled - User needs to select the correct serial port and click "Connect" button on its own');
            }
        }
        
        // bind UI hook to auto-connect checkbos
        $('input.auto_connect').change(function() {
            GUI.auto_connect = $(this).is(':checked');
            
            // update title/tooltip
            if (GUI.auto_connect) {
                $('input.auto_connect, span.auto_connect').prop('title', 'Auto-Connect: Enabled - Configurator automatically tries to connect when new port is detected');
                
                $('select#baud').val(115200).prop('disabled', true);
            } else {
                $('input.auto_connect, span.auto_connect').prop('title', 'Auto-Connect: Disabled - User needs to select the correct serial port and click "Connect" button on its own');
                
                if (!GUI.connected_to && !GUI.connecting_to) $('select#baud').prop('disabled', false);
            }
            
            chrome.storage.local.set({'auto_connect': GUI.auto_connect}, function() {});
        });
    });    
}); 

function onOpen(openInfo) {    
    if (openInfo.connectionId > 0) {
        // update connected_to
        GUI.connected_to = GUI.connecting_to;
        
        // reset connecting_to
        GUI.connecting_to = false;
        
        console.log('Connection was opened with ID: ' + openInfo.connectionId + ', Baud: ' + openInfo.bitrate);
        
        // save selected port with chrome.storage if the port differs
        chrome.storage.local.get('last_used_port', function(result) {
            if (typeof result.last_used_port != 'undefined') {
                if (result.last_used_port != GUI.connected_to) {
                    // last used port doesn't match the one found in local db, we will store the new one
                    chrome.storage.local.set({'last_used_port': GUI.connected_to}, function() {
                        // Debug message is currently disabled (we dont need to spam the console log with that)
                        // console.log('Last selected port was saved in chrome.storage.');
                    });
                }
            } else {
                // variable isn't stored yet, saving
                chrome.storage.local.set({'last_used_port': GUI.connected_to}, function() {
                    // Debug message is currently disabled (we dont need to spam the console log with that)
                    // console.log('Last selected port was saved in chrome.storage.');
                });
            }
        });

        serial.onReceive.addListener(read_serial);
        GUI.interval_add('port_usage', port_usage, 1000, true);
        
        // disconnect after 10 seconds with error if we don't get IDENT data
        GUI.timeout_add('connecting', function() {
            if (!configuration_received) {
                notify('Did not received configuration within <span style="color: red">10 seconds</span>, communication <span style="color: red">failed</span> - Disconnecting');
                
                $('div#port-picker a.connect').click(); // disconnect
            }
        }, 10000);

        // request configuration data
        send_message(MSP_codes.MSP_UID, MSP_codes.MSP_UID, false, function() {
            send_message(MSP_codes.MSP_IDENT, MSP_codes.MSP_IDENT, false, function() {
                GUI.timeout_remove('connecting'); // kill connecting timer
                
                // Update UI elements that doesn't need consistent refreshing
                $('.software-version').html(CONFIG.version);
                
                configuration_received = true;
                $('div#port-picker a.connect').text('Disconnect').addClass('active');
                $('#tabs li a:first').click();
            });
        });
    } else {
        console.log('Failed to open serial port');
        notify('Failed to open serial port', 'red');
        
        $('div#port-picker a.connect').text('Connect');
        $('div#port-picker a.connect').removeClass('active'); 
        
        // unlock port select & baud
        $('div#port-picker #port, div#port-picker #baud, div#port-picker #delay').prop('disabled', false);
        
        // reset data
        $('div#port-picker a.connect').data("clicks", false);
    }
}

function onClosed(result) {
    if (result) { // All went as expected        
        sensor_status(sensors_detected = 0); // reset active sensor indicators
        $('#tabs > ul li').removeClass('active'); // de-select any selected tabs
        tab_initialize_default();
        
        console.log('Connection closed successfully.');
    } else { // Something went wrong
        console.log('There was an error that happened during "connection-close" procedure.');
        notify('Failed to close serial port', 'red');
    } 
}

function read_serial(info) {
    if (!CLI_active) {
        MSP_char_read(info);
    } else {
        handle_CLI(info);
    }
}

function port_usage() {
    var port_usage = (char_counter * 10 / parseInt($('div#port-picker #baud').val())) * 100;    
    $('span.port-usage').html(parseInt(port_usage) + '%');

    // reset counter
    char_counter = 0;
}

function update_ports() {
    var initial_ports = false;
    
    GUI.interval_add('port-update', function() {
        serial.getDevices(function(current_ports) {
            if (initial_ports.length > current_ports.length || !initial_ports) {
                // port got removed or initial_ports wasn't initialized yet
                var removed_ports = array_difference(initial_ports, current_ports);
                
                if (initial_ports != false) console.log('Port removed: ' + removed_ports);
                
                // disconnect "UI" if necessary
                if (GUI.connected_to != false && removed_ports[0] == GUI.connected_to) {
                    $('div#port-picker a.connect').click();
                }
                
                // update COM port list
                update_port_select_menu(current_ports);
                
                // auto-select last used port (only during initialization)
                if (!initial_ports) {
                    chrome.storage.local.get('last_used_port', function(result) {
                        // if last_used_port was set, we try to select it
                        if (result.last_used_port) {                            
                            current_ports.forEach(function(port) {
                                if (port == result.last_used_port) {
                                    console.log('Selecting last used port: ' + result.last_used_port);
                                    
                                    $('div#port-picker .port select').val(result.last_used_port);
                                }
                            });
                        } else {
                            console.log('Last used port wasn\'t saved "yet", auto-select disabled.');
                        }
                    });
                }
                
                // reset initial_ports
                initial_ports = current_ports;
            }
            
            var new_ports = array_difference(current_ports, initial_ports);
            
            if (new_ports.length > 0) {
                console.log('New port found: ' + new_ports[0]);
                
                // generate new COM port list
                update_port_select_menu(current_ports);
                
                // select / highlight new port, if connected -> select connected port
                if (!GUI.connected_to) {
                    $('div#port-picker .port select').val(new_ports[0]);
                } else {   
                    $('div#port-picker .port select').val(GUI.connected_to);
                }
                
                // start connect procedure (if statement is valid)
                if (GUI.auto_connect && !GUI.connecting_to && !GUI.connected_to) {
                    // we need firmware flasher protection over here
                    if (GUI.active_tab != 'firmware_flasher') {
                        GUI.timeout_add('auto-connect_timeout', function() {
                            $('div#port-picker a.connect').click();
                        }, 50); // small timeout so we won't get any nasty connect errors due to system initializing the bus
                    }
                }
                
                // reset initial_ports
                initial_ports = current_ports;
            }
        });
    }, 250, true);
}

function update_port_select_menu(ports) {
    $('div#port-picker .port select').html(''); // drop previous one
    
    if (ports.length > 0) {
        for (var i = 0; i < ports.length; i++) {
            $('div#port-picker .port select').append($("<option/>", {value: ports[i], text: ports[i]}));
        }
    } else {
        $('div#port-picker .port select').append($("<option/>", {value: 0, text: 'NOT FOUND'}));
    }    
}

function sensor_status(sensors_detected) {
    // initialize variable (if it wasn't)
    if (typeof sensor_status.previous_sensors_detected == 'undefined') {
        sensor_status.previous_sensors_detected = 0;
    }
    
    // update UI (if necessary)
    if (sensor_status.previous_sensors_detected != sensors_detected) {
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
        
        // set current value
        sensor_status.previous_sensors_detected = sensors_detected;
    }
}

function highByte(num) {
    return num >> 8;
}

function lowByte(num) {
    return 0x00FF & num;
}

function bit_check(num, bit) {
    return ((num >> bit) % 2 != 0);
}

function bit_set(num, bit) {
    return num | 1 << bit;
}

function bit_clear(num, bit) {
    return num & ~(1 << bit);
}