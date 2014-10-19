'use strict';

$(document).ready(function () {
    $('div#port-picker a.connect').click(function () {
        if (GUI.connect_lock != true) { // GUI control overrides the user control
            var clicks = $(this).data('clicks'),
                selected_port = String($('div#port-picker #port').val()),
                selected_baud = parseInt($('div#port-picker #baud').val());

            if (selected_port != '0' && selected_port != 'DFU') {
                if (!clicks) {
                    console.log('Connecting to: ' + selected_port);
                    GUI.connecting_to = selected_port;

                    // lock port select & baud while we are connecting / connected
                    $('div#port-picker #port, div#port-picker #baud, div#port-picker #delay').prop('disabled', true);
                    $('div#port-picker a.connect').text(chrome.i18n.getMessage('connecting'));

                    serial.connect(selected_port, {bitrate: selected_baud}, onOpen);
                } else {
                    GUI.timeout_kill_all();
                    GUI.interval_kill_all();
                    GUI.tab_switch_cleanup();
                    GUI.tab_switch_in_progress = false;

                    serial.disconnect(onClosed);

                    GUI.connected_to = false;
                    CONFIGURATOR.connectionValid = false;
                    MSP.disconnect_cleanup();
                    PortUsage.reset();

                    // Reset various UI elements
                    $('span.i2c-error').text(0);
                    $('span.cycle-time').text(0);

                    // unlock port select & baud
                    $('div#port-picker #port').prop('disabled', false);
                    if (!GUI.auto_connect) $('div#port-picker #baud').prop('disabled', false);

                    // reset connect / disconnect button
                    $(this).text(chrome.i18n.getMessage('connect'));
                    $(this).removeClass('active');

                    // reset active sensor indicators
                    sensor_status(0);

                    // de-select any selected tabs
                    $('#tabs > ul li').removeClass('active');

                    // detach listeners and remove element data
                    $('#content').empty();

                    // load default html
                    TABS.landing.initialize();
                }

                $(this).data("clicks", !clicks);
            }
        }
    });

    // auto-connect
    chrome.storage.local.get('auto_connect', function (result) {
        if (result.auto_connect === 'undefined' || result.auto_connect) {
            // default or enabled by user
            GUI.auto_connect = true;

            $('input.auto_connect').prop('checked', true);
            $('input.auto_connect, span.auto_connect').prop('title', chrome.i18n.getMessage('autoConnectEnabled'));

            $('select#baud').val(115200).prop('disabled', true);
        } else {
            // disabled by user
            GUI.auto_connect = false;

            $('input.auto_connect').prop('checked', false);
            $('input.auto_connect, span.auto_connect').prop('title', chrome.i18n.getMessage('autoConnectDisabled'));
        }

        // bind UI hook to auto-connect checkbos
        $('input.auto_connect').change(function () {
            GUI.auto_connect = $(this).is(':checked');

            // update title/tooltip
            if (GUI.auto_connect) {
                $('input.auto_connect, span.auto_connect').prop('title', chrome.i18n.getMessage('autoConnectEnabled'));

                $('select#baud').val(115200).prop('disabled', true);
            } else {
                $('input.auto_connect, span.auto_connect').prop('title', chrome.i18n.getMessage('autoConnectDisabled'));

                if (!GUI.connected_to && !GUI.connecting_to) $('select#baud').prop('disabled', false);
            }

            chrome.storage.local.set({'auto_connect': GUI.auto_connect});
        });
    });

    PortHandler.initialize();
    PortUsage.initialize();
});

function onOpen(openInfo) {
    if (openInfo) {
        // update connected_to
        GUI.connected_to = GUI.connecting_to;

        // reset connecting_to
        GUI.connecting_to = false;

        GUI.log(chrome.i18n.getMessage('serialPortOpened', [openInfo.connectionId]));

        // save selected port with chrome.storage if the port differs
        chrome.storage.local.get('last_used_port', function (result) {
            if (result.last_used_port) {
                if (result.last_used_port != GUI.connected_to) {
                    // last used port doesn't match the one found in local db, we will store the new one
                    chrome.storage.local.set({'last_used_port': GUI.connected_to});
                }
            } else {
                // variable isn't stored yet, saving
                chrome.storage.local.set({'last_used_port': GUI.connected_to});
            }
        });

        serial.onReceive.addListener(read_serial);

        // disconnect after 10 seconds with error if we don't get IDENT data
        GUI.timeout_add('connecting', function () {
            if (!CONFIGURATOR.connectionValid) {
                GUI.log(chrome.i18n.getMessage('noConfigurationReceived'));

                $('div#port-picker a.connect').click(); // disconnect
            }
        }, 10000);

        // request configuration data
        MSP.send_message(MSP_codes.MSP_UID, false, false, function () {
            GUI.timeout_remove('connecting'); // kill connecting timer

            GUI.log(chrome.i18n.getMessage('uniqueDeviceIdReceived', [CONFIG.uid[0].toString(16) + CONFIG.uid[1].toString(16) + CONFIG.uid[2].toString(16)]));
            MSP.send_message(MSP_codes.MSP_IDENT, false, false, function () {

                if (CONFIG.version >= CONFIGURATOR.firmwareVersionAccepted) {
                    MSP.send_message(MSP_codes.MSP_BUILDINFO, false, false, function () {
                        googleAnalytics.sendEvent('Firmware', 'Using', CONFIG.buildInfo);
                        GUI.log('Running firmware released on: <strong>' + CONFIG.buildInfo + '</strong>');

                        // continue as usually
                        CONFIGURATOR.connectionValid = true;

                        $('div#port-picker a.connect').text(chrome.i18n.getMessage('disconnect')).addClass('active');
                        $('#tabs li a:first').click();
                    });
                } else {
                    GUI.log(chrome.i18n.getMessage('firmwareVersionNotSupported', [CONFIGURATOR.firmwareVersionAccepted]));
                    $('div#port-picker a.connect').click(); // disconnect
                }
            });
        });
    } else {
        console.log('Failed to open serial port');
        GUI.log(chrome.i18n.getMessage('serialPortOpenFail'));

        $('div#port-picker a.connect').text(chrome.i18n.getMessage('connect'));
        $('div#port-picker a.connect').removeClass('active');

        // unlock port select & baud
        $('div#port-picker #port, div#port-picker #baud, div#port-picker #delay').prop('disabled', false);

        // reset data
        $('div#port-picker a.connect').data("clicks", false);
    }
}

function onClosed(result) {
    if (result) { // All went as expected
        GUI.log(chrome.i18n.getMessage('serialPortClosedOk'));
    } else { // Something went wrong
        GUI.log(chrome.i18n.getMessage('serialPortClosedFail'));
    }
}

function read_serial(info) {
    if (!CONFIGURATOR.cliActive) {
        MSP.read(info);
    } else if (CONFIGURATOR.cliActive) {
        TABS.cli.read(info);
    }
}

function sensor_status(sensors_detected) {
    // initialize variable (if it wasn't)
    if (!sensor_status.previous_sensors_detected) {
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

function specificByte(num, pos) {
    return 0x000000FF & (num >> (8 * pos));
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