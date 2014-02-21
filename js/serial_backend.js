var configuration_received = false;
var CLI_active = false;
var CLI_valid = false;

$(document).ready(function() {    
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
                    GUI.interval_kill_all();
                    
                    GUI.tab_switch_cleanup();
                    GUI.timeout_remove('connecting');
                    
                    serial.disconnect(onClosed);
                    
                    GUI.connected_to = false;
                    
                    // Reset various UI elements
                    $('span.port-usage').html('0%');
                    $('.software-version').html('0.0');
                    $('span.cycle-time').html('0');
                    
                    MSP.disconnect_cleanup();
                    configuration_received = false; // reset valid config received variable (used to block tabs while not connected properly)
                    
                    // unlock port select & baud
                    $('div#port-picker #port').prop('disabled', false);
                    if (!GUI.auto_connect) $('div#port-picker #baud').prop('disabled', false);
                    
                    $(this).text('Connect');
                    $(this).removeClass('active');
                    
                    sensor_status(sensors_detected = 0); // reset active sensor indicators
                    $('#tabs > ul li').removeClass('active'); // de-select any selected tabs
                    tab_initialize_default();
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
    PortHandler.initialize();
}); 

function onOpen(openInfo) {    
    if (openInfo) {
        // update connected_to
        GUI.connected_to = GUI.connecting_to;
        
        // reset connecting_to
        GUI.connecting_to = false;
        
        GUI.log('Serial port <span style="color: green">successfully</span> opened with ID: ' + openInfo.connectionId);
        
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
                GUI.log('No configuration received within <span style="color: red">10 seconds</span>, communication <span style="color: red">failed</span>');
                
                $('div#port-picker a.connect').click(); // disconnect
            }
        }, 10000);

        // request configuration data
        send_message(MSP_codes.MSP_UID, MSP_codes.MSP_UID, false, function() {
            GUI.log('Unique device ID <span style="color: green">received</span> - <strong>0x' + CONFIG.uid[0].toString(16) + CONFIG.uid[1].toString(16) + CONFIG.uid[2].toString(16) + '</strong>');
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
        GUI.log('Failed to open serial port', 'red');
        
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
        GUI.log('Serial port <span style="color: green">successfully</span> closed');
    } else { // Something went wrong
        GUI.log('<span style="color: red">Failed</span> to close serial port');
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