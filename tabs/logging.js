function tab_initialize_logging() {
    ga_tracker.sendAppView('Logging');
    GUI.active_tab = 'logging';

    var requested_properties = [];

    $('#content').load("./tabs/logging.html", process_html);

    function process_html() {
        // translate to user-selected language
        localize();

        // UI hooks
        $('a.log_file').click(prepare_file);

        $('a.logging').click(function() {
            if (fileEntry.isFile) {
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

                        GUI.interval_add('log_data_pull', poll_data, 1000, true); // refresh rate goes here
                        GUI.interval_add('flush_data', function() {
                            if (fileWriter.readyState == 0 || fileWriter.readyState == 2) {
                                append_to_file(log_buffer.join('\n'));

                                $('.samples').text(samples += log_buffer.length);
                                $('.size').text((fileWriter.length / 1024).toFixed(2) + ' kB');

                                log_buffer = [];
                            } else {
                                console.log('IO having trouble keeping up with the data flow');
                            }
                        }, 1000);

                        $(this).text('Stop Logging');
                        $(this).data("clicks", !clicks);
                    } else {
                        GUI.log('Please select at least one property to log');
                    }
                } else {
                    GUI.interval_remove('log_data_pull');
                    GUI.interval_remove('flush_data');

                    $(this).text('Start Logging');
                    $(this).data("clicks", !clicks);
                }
            } else {
                GUI.log('Please select log file');
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

                        prepare_writer();
                    } else {
                        console.log('File appears to be read only, sorry.');
                    }
                });
            });
        });
    }

    function prepare_writer() {
        fileEntry.createWriter(function(writer) {
            fileWriter = writer;

            fileWriter.onerror = function(e) {
                console.error(e);
            };

            fileWriter.onwriteend = function() {
                // console.log('Data written');
            };
        }, function(e) {
            console.error(e);
        });
    }

    function append_to_file(data) {
        if (fileWriter.position < fileWriter.length) {
            fileWriter.seek(fileWriter.length);
        }

        fileWriter.write(new Blob([data + '\n'], {type: 'text/plain'}));
    }
}