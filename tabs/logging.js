var MSP_pass_through = false;

function tab_initialize_logging() {
    ga_tracker.sendAppView('Logging');
    GUI.active_tab = 'logging';

    var requested_properties = [];

    if (configuration_received) {
        MSP.send_message(MSP_codes.MSP_RC, false, false, get_motor_data);

        function get_motor_data() {
            MSP.send_message(MSP_codes.MSP_MOTOR, false, false, load_html);
        }

        function load_html() {
            $('#content').load("./tabs/logging.html", process_html);
        }
    } else {
        MSP_pass_through = true;

        // we will initialize RC.channels array and MOTOR_DATA array manually
        RC.active_channels = 8;
        for (var i = 0; i < RC.active_channels; i++) {
            RC.channels[i] = 0;
        }

        for (var i = 0; i < 8; i++) {
            MOTOR_DATA[i] = 0;
        }

        $('#content').load("./tabs/logging.html", process_html);
    }

    function process_html() {
        // translate to user-selected language
        localize();

        // UI hooks
        $('a.log_file').click(prepare_file);

        $('a.logging').click(function() {
            if (GUI.connected_to) {
                if (fileEntry != null) {
                    var clicks = $(this).data('clicks');

                    if (!clicks) {
                        // reset some variables before start
                        samples = 0;
                        requests = 0;
                        log_buffer = [];
                        requested_properties = [];

                        $('.properties input:checked').each(function() {
                            requested_properties.push($(this).prop('name'));
                        });

                        if (requested_properties.length) {
                            // print header for the csv file
                            print_head();

                            function poll_data() {
                                if (requests) {
                                    // save current data (only after everything is initialized)
                                    crunch_data();
                                }

                                // request new
                                if (!MSP_pass_through) {
                                    for (var i = 0; i < requested_properties.length; i++, requests++) {
                                        MSP.send_message(MSP_codes[requested_properties[i]]);
                                    }
                                }
                            }

                            GUI.interval_add('log_data_pull', poll_data, parseInt($('select.speed').val()), true); // refresh rate goes here
                            GUI.interval_add('flush_data', function() {
                                if (log_buffer.length) { // only execute when there is actual data to write
                                    if (fileWriter.readyState == 0 || fileWriter.readyState == 2) {
                                        append_to_file(log_buffer.join('\n'));

                                        $('.samples').text(samples += log_buffer.length);

                                        log_buffer = [];
                                    } else {
                                        console.log('IO having trouble keeping up with the data flow');
                                    }
                                }
                            }, 1000);

                            $('.speed').prop('disabled', true);
                            $(this).text(chrome.i18n.getMessage('loggingStop'));
                            $(this).data("clicks", !clicks);
                        } else {
                            GUI.log(chrome.i18n.getMessage('loggingErrorOneProperty'));
                        }
                    } else {
                        GUI.interval_remove('log_data_pull');
                        GUI.interval_remove('flush_data');

                        $('.speed').prop('disabled', false);
                        $(this).text(chrome.i18n.getMessage('loggingStart'));
                        $(this).data("clicks", !clicks);
                    }
                } else {
                    GUI.log(chrome.i18n.getMessage('loggingErrorLogFile'));
                }
            } else {
                GUI.log(chrome.i18n.getMessage('loggingErrorNotConnected'));
            }
        });

        if (MSP_pass_through) {
            $('a.back').show();

            $('a.back').click(function() {
                if (GUI.connected_to) {
                    $('a.connect').click();
                } else {
                    GUI.tab_switch_cleanup(function() {
                        MSP_pass_through = false;
                        $('#tabs > ul li').removeClass('active');
                        tab_initialize_default();
                    });
                }
            });
        }

        chrome.storage.local.get('logging_file_entry', function(result) {
            if (result.logging_file_entry) {
                chrome.fileSystem.restoreEntry(result.logging_file_entry, function(entry) {
                    fileEntry = entry;
                    prepare_writer(true);
                });
            }
        });
    }

    function print_head() {
        var head = "timestamp";

        for (var i = 0; i < requested_properties.length; i++) {
            switch (requested_properties[i]) {
                case 'MSP_RAW_IMU':
                    head += ',' + 'gyroscopeX';
                    head += ',' + 'gyroscopeY';
                    head += ',' + 'gyroscopeZ';

                    head += ',' + 'accelerometerX';
                    head += ',' + 'accelerometerY';
                    head += ',' + 'accelerometerZ';

                    head += ',' + 'magnetometerX';
                    head += ',' + 'magnetometerY';
                    head += ',' + 'magnetometerZ';
                    break;
                case 'MSP_ATTITUDE':
                    head += ',' + 'kinematicsX';
                    head += ',' + 'kinematicsY';
                    head += ',' + 'kinematicsZ';
                    break;
                case 'MSP_ALTITUDE':
                    head += ',' + 'altitude';
                    break;
                case 'MSP_RAW_GPS':
                    head += ',' + 'gpsFix';
                    head += ',' + 'gpsNumSat';
                    head += ',' + 'gpsLat';
                    head += ',' + 'gpsLon';
                    head += ',' + 'gpsAlt';
                    head += ',' + 'gpsSpeed';
                    head += ',' + 'gpsGroundCourse';
                    break;
                case 'MSP_ANALOG':
                    head += ',' + 'voltage';
                    head += ',' + 'amperage';
                    head += ',' + 'mAhdrawn';
                    head += ',' + 'rssi';
                    break;
                case 'MSP_RC':
                    for (var chan = 0; chan < RC.active_channels; chan++) {
                        head += ',' + 'RC' + chan;
                    }
                    break;
                case 'MSP_MOTOR':
                    for (var motor = 0; motor < MOTOR_DATA.length; motor++) {
                        head += ',' + 'Motor' + motor;
                    }
                    break;
                case 'MSP_DEBUG':
                    for (var debug = 0; debug < SENSOR_DATA.debug.length; debug++) {
                        head += ',' + 'Debug' + debug;
                    }
                    break;
            }
        }

        append_to_file(head);
    }

    var samples = 0;
    var requests = 0;
    var log_buffer = [];
    function crunch_data() {
        var sample = millitime();

        for (var i = 0; i < requested_properties.length; i++) {
            switch (requested_properties[i]) {
                case 'MSP_RAW_IMU':
                    sample += ',' + SENSOR_DATA.gyroscope;
                    sample += ',' + SENSOR_DATA.accelerometer;
                    sample += ',' + SENSOR_DATA.magnetometer;
                    break;
                case 'MSP_ATTITUDE':
                    sample += ',' + SENSOR_DATA.kinematics[0];
                    sample += ',' + SENSOR_DATA.kinematics[1];
                    sample += ',' + SENSOR_DATA.kinematics[2];
                    break;
                case 'MSP_ALTITUDE':
                    sample += ',' + SENSOR_DATA.altitude;
                    break;
                case 'MSP_RAW_GPS':
                    sample += ',' + GPS_DATA.fix;
                    sample += ',' + GPS_DATA.numSat;
                    sample += ',' + (GPS_DATA.lat / 10000000);
                    sample += ',' + (GPS_DATA.lon / 10000000);
                    sample += ',' + GPS_DATA.alt;
                    sample += ',' + GPS_DATA.speed;
                    sample += ',' + GPS_DATA.ground_course;
                    break;
                case 'MSP_ANALOG':
                    sample += ',' + ANALOG.voltage;
                    sample += ',' + ANALOG.amperage;
                    sample += ',' + ANALOG.mAhdrawn;
                    sample += ',' + ANALOG.rssi;
                    break;
                case 'MSP_RC':
                    for (var chan = 0; chan < RC.active_channels; chan++) {
                        sample += ',' + RC.channels[chan];
                    }
                    break;
                case 'MSP_MOTOR':
                    sample += ',' + MOTOR_DATA;
                    break;
                case 'MSP_DEBUG':
                    sample += ',' + SENSOR_DATA.debug;
                    break;
            }
        }

        log_buffer.push(sample);
    }

    // IO related methods
    var fileEntry = null;
    var fileWriter = null;

    function prepare_file() {
        // create or load the file
        chrome.fileSystem.chooseEntry({type: 'saveFile', suggestedName: 'bf_data_log', accepts: [{extensions: ['csv']}]}, function(entry) {
            if (!entry) {
                console.log('No file selected');
                return;
            }

            fileEntry = entry;

            // echo/console log path specified
            chrome.fileSystem.getDisplayPath(fileEntry, function(path) {
                console.log('Log file path: ' + path);
            });

            // change file entry from read only to read/write
            chrome.fileSystem.getWritableEntry(fileEntry, function(fileEntryWritable) {
                // check if file is writable
                chrome.fileSystem.isWritableEntry(fileEntryWritable, function(isWritable) {
                    if (isWritable) {
                        fileEntry = fileEntryWritable;

                        // save entry for next use
                        chrome.storage.local.set({'logging_file_entry': chrome.fileSystem.retainEntry(fileEntry)});

                        // reset sample counter in UI
                        $('.samples').text(0);

                        prepare_writer();
                    } else {
                        console.log('File appears to be read only, sorry.');
                    }
                });
            });
        });
    }

    function prepare_writer(retaining) {
        fileEntry.createWriter(function(writer) {
            fileWriter = writer;

            fileWriter.onerror = function(e) {
                console.error(e);

                // stop logging if the procedure was/is still running
                if ($('a.logging').data('clicks')) $('a.logging').click();
            };

            fileWriter.onwriteend = function() {
                $('.size').text(bytesToSize(fileWriter.length));
            };

            if (retaining) {
                chrome.fileSystem.getDisplayPath(fileEntry, function(path) {
                    GUI.log(chrome.i18n.getMessage('loggingAutomaticallyRetained', [path]));
                });
            }

            // update log size in UI on fileWriter creation
            $('.size').text(bytesToSize(fileWriter.length));
        }, function(e) {
            // File is not readable or does not exist!
            console.error(e);

            if (retaining) {
                fileEntry = null;
            }
        });
    }

    function append_to_file(data) {
        if (fileWriter.position < fileWriter.length) {
            fileWriter.seek(fileWriter.length);
        }

        fileWriter.write(new Blob([data + '\n'], {type: 'text/plain'}));
    }
}