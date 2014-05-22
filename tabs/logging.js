function tab_initialize_logging() {
    ga_tracker.sendAppView('Logging');
    GUI.active_tab = 'logging';

    var requested_properties = [];

    send_message(MSP_codes.MSP_RC, false, false, get_motor_data);

    function get_motor_data() {
        send_message(MSP_codes.MSP_MOTOR, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/logging.html", process_html);
    }

    function process_html() {
        // translate to user-selected language
        localize();

        // UI hooks
        $('a.log_file').click(prepare_file);

        $('a.logging').click(function() {
            if (fileEntry != null) {
                var clicks = $(this).data('clicks');

                if (!clicks) {
                    // reset some variables before start
                    samples = 0;
                    log_buffer = [];
                    requested_properties = [];

                    $('.properties input:checked').each(function() {
                        requested_properties.push($(this).prop('name'));
                    });

                    if (requested_properties.length) {
                        // print header for the
                        print_head();

                        function poll_data() {
                            // save current
                            crunch_data();

                            // request new
                            for (var i = 0; i < requested_properties.length; i++) {
                                send_message(MSP_codes[requested_properties[i]]);

                                /* this approach could be used if we want to utilize request time compensation
                                if (i < requested_properties.length -1) {
                                    send_message(requested_properties[i]);
                                } else {
                                    send_message(requested_properties[i], false, false, poll_data);
                                }
                                */
                            }
                        }

                        GUI.interval_add('log_data_pull', poll_data, parseInt($('select.speed').val()), true); // refresh rate goes here
                        GUI.interval_add('flush_data', function() {
                            if (log_buffer.length) { // only execute when there is actual data to write
                                if (fileWriter.readyState == 0 || fileWriter.readyState == 2) {
                                    append_to_file(log_buffer.join('\n'));

                                    $('.samples').text(samples += log_buffer.length);
                                    $('.size').text(chrome.i18n.getMessage('loggingKB', [(fileWriter.length / 1024).toFixed(2)]));

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
        });

        chrome.storage.local.get('logging_file_entry', function(result) {
            if (result.logging_file_entry) {
                chrome.fileSystem.restoreEntry(result.logging_file_entry, function(entry) {
                    fileEntry = entry;
                    prepare_writer(true);
                });
            }
        });
    }

    var samples = 0;
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
                    sample += ',' + SENSOR_DATA.kinematicsX;
                    sample += ',' + SENSOR_DATA.kinematicsY;
                    sample += ',' + SENSOR_DATA.kinematicsZ;
                    break;
                case 'MSP_ALTITUDE':
                    sample += ',' + SENSOR_DATA.altitude;
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

        log_buffer.push(head);
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
                // console.log('Data written');
            };

            if (retaining) {
                chrome.fileSystem.getDisplayPath(fileEntry, function(path) {
                    GUI.log(chrome.i18n.getMessage('loggingAutomaticallyRetained', [path]));
                });
            }
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