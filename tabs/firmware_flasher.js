function tab_initialize_firmware_flasher() {
    ga_tracker.sendAppView('Firmware Flasher');
    GUI.active_tab = 'firmware_flasher';

    var intel_hex = false; // standard intel hex in string format
    var parsed_hex = false; // parsed raw hex in array format

    $('#content').load("./tabs/firmware_flasher.html", function() {
        // translate to user-selected language
        localize();

        // UI Hooks
        $('a.load_file').click(function() {
            chrome.fileSystem.chooseEntry({type: 'openFile', accepts: [{extensions: ['hex']}]}, function(fileEntry) {
                if (!fileEntry) {
                    // no "valid" file selected/created, aborting
                    console.log('No valid file selected, aborting');
                    return;
                }

                // hide github info (if it exists)
                $('div.git_info').slideUp();

                chrome.fileSystem.getDisplayPath(fileEntry, function(path) {
                    console.log('Loading file from: ' + path);
                    $('span.path').html(path);

                    fileEntry.file(function(file) {
                        var reader = new FileReader();

                        reader.onprogress = function(e) {
                            if (e.total > 1048576) { // 1 MB
                                // dont allow reading files bigger then 1 MB
                                console.log('File limit (1 MB) exceeded, aborting');
                                reader.abort();
                            }
                        };

                        reader.onloadend = function(e) {
                            if (e.total != 0 && e.total == e.loaded) {
                                console.log('File loaded');

                                intel_hex = e.target.result;

                                parse_hex(intel_hex, function(data) {
                                    parsed_hex = data;

                                    if (parsed_hex) {
                                        GUI.log(chrome.i18n.getMessage('firmwareFlasherLocalFirmwareLoaded'));
                                        ga_tracker.sendEvent('Flashing', 'Firmware', 'local');
                                        $('a.flash_firmware').removeClass('locked');

                                        $('span.size').html(parsed_hex.bytes_total + ' bytes');
                                    } else {
                                        GUI.log(chrome.i18n.getMessage('firmwareFlasherHexCorrupted'));
                                    }
                                });
                            }
                        };

                        reader.readAsText(file);
                    });
                });
            });
        });

        $('a.load_remote_file').click(function() {
            $.get('https://raw.githubusercontent.com/hydra/cleanflight/master/obj/cleanflight_NAZE.hex', function(data) {
                intel_hex = data;

                parse_hex(intel_hex, function(data) {
                    parsed_hex = data;

                    if (parsed_hex) {
                        GUI.log(chrome.i18n.getMessage('firmwareFlasherRemoteFirmwareLoaded'));
                        ga_tracker.sendEvent('Flashing', 'Firmware', 'online');
                        $('a.flash_firmware').removeClass('locked');

                        $('span.path').text('Using remote Firmware');
                        $('span.size').text(parsed_hex.bytes_total + ' bytes');
                    } else {
                        GUI.log(chrome.i18n.getMessage('firmwareFlasherHexCorrupted'));
                    }
                });
            }).fail(function() {
                GUI.log(chrome.i18n.getMessage('firmwareFlasherFailedToLoadOnlineFirmware'));
                $('a.flash_firmware').addClass('locked');
            });

            $.get('https://api.github.com/repos/hydra/cleanflight/commits?page=1&per_page=1&path=obj/cleanflight_NAZE.hex', function(data) {
                var data = data[0];
                var d = new Date(data.commit.author.date);
                var date = ('0' + (d.getMonth() + 1)).slice(-2) + '.' + ('0' + (d.getDate() + 1)).slice(-2) + '.' + d.getFullYear();
                date += ' @ ' + ('0' + d.getHours()).slice(-2) + ':' + ('0' + d.getMinutes()).slice(-2);

                $('div.git_info .committer').text(data.commit.author.name);
                $('div.git_info .date').text(date);
                $('div.git_info .message').text(data.commit.message);

                $('div.git_info').slideDown();
            });
        });

        $('a.flash_firmware').click(function() {
            if (!$(this).hasClass('locked')) {
                if (!GUI.connect_lock) { // button disabled while flashing is in progress
                    if (parsed_hex != false) {
                        if (String($('div#port-picker #port').val()) != 'DFU') {
                            STM32.connect(parsed_hex);
                        } else {
                            STM32DFU.connect(parsed_hex);
                        }
                    } else {
                        GUI.log(chrome.i18n.getMessage('firmwareFlasherFirmwareNotLoaded'));
                    }
                }
            }
        });

        chrome.storage.local.get('no_reboot_sequence', function(result) {
            if (result.no_reboot_sequence) {
                $('input.updating').prop('checked', true);
                $('label.flash_on_connect_wrapper').show();
            } else {
                $('input.updating').prop('checked', false);
            }

            // bind UI hook so the status is saved on change
            $('input.updating').change(function() {
                var status = $(this).is(':checked');

                if (status) {
                    $('label.flash_on_connect_wrapper').show();
                } else {
                    $('input.flash_on_connect').prop('checked', false).change();
                    $('label.flash_on_connect_wrapper').hide();
                }

                chrome.storage.local.set({'no_reboot_sequence': status});
            });
        });

        chrome.storage.local.get('flash_on_connect', function(result) {
            if (result.flash_on_connect) {
                $('input.flash_on_connect').prop('checked', true);
            } else {
                $('input.flash_on_connect').prop('checked', false);
            }

            $('input.flash_on_connect').change(function() {
                var status = $(this).is(':checked');

                if (status) {
                    var flashing_port;

                    function start() {
                        PortHandler.port_detected('flash_next_device', function(result) {
                            flashing_port = result[0];
                            GUI.log('Detected: <strong>' + flashing_port + '</strong> - triggering flash on connect');
                            console.log('Detected: ' + flashing_port + ' - triggering flash on connect');

                            // Trigger regular Flashing sequence
                            $('a.flash_firmware').click();

                            // Detect port removal to create a new callback
                            end();
                        }, false, true);
                    }

                    function end() {
                        PortHandler.port_removed('flashed_device_removed', function(result) {
                            for (var i = 0; i < result.length; i++) {
                                if (result[i] == flashing_port) {
                                    // flashed device removed
                                    GUI.log('Removed: <strong>' + flashing_port + '</strong> - ready for next device');
                                    console.log('Removed: ' + flashing_port + ' - ready for next device');

                                    flashing_port = false;
                                    start();

                                    return;
                                }
                            }

                            // different device removed, we need to retry
                            end();
                        }, false, true);
                    }

                    start();
                } else {
                    PortHandler.flush_callbacks();
                }

                chrome.storage.local.set({'flash_on_connect': status});
            }).change();
        });

        chrome.storage.local.get('erase_chip', function(result) {
            if (result.erase_chip) {
                $('input.erase_chip').prop('checked', true);
            } else {
                $('input.erase_chip').prop('checked', false);
            }

            // bind UI hook so the status is saved on change
            $('input.erase_chip').change(function() {
                var status = $(this).is(':checked');

                chrome.storage.local.set({'erase_chip': status});
            });
        });

        /*
        chrome.storage.local.get('dev_mode', function(result) {
            if (typeof result.dev_mode !== 'undefined') {
                if (result.dev_mode) {
                    GUI.log('Dev mode: <strong>Enabled</strong>');
                    bind_enter_handler();
                }
            }
        });

        var keys_down = {};
        $(document).keydown(function(e) {
            keys_down[e.which] = true;

            // idkfa
            if (keys_down[65] && keys_down[68] && keys_down[70] && keys_down[73] && keys_down[75]) {
                chrome.storage.local.get('dev_mode', function(result) {
                    if (typeof result.dev_mode === 'undefined') {
                        GUI.log('Dev mode: <strong>Enabled</strong>');
                        bind_enter_handler();

                        chrome.storage.local.set({'dev_mode': true});
                    }
                });
            }
        });

        $(document).keyup(function(e) {
            delete keys_down[e.which];
        });

        function bind_enter_handler() {
            // unbind first (in case there is anything bound here)
            $(document).unbind('keypress');

            $(document).keypress(function(e) {
                if (e.which == 13) { // enter
                    // Trigger regular Flashing sequence
                    $('a.flash_firmware').click();
                }
            });
        }
        */

        /*
        GUI.interval_add('factory_mode', function factory_mode() {
            serial.getControlSignals(function(result) {
                if (result.cts == true) {
                    // Trigger regular Flashing sequence
                    $('a.flash_firmware').click();
                }
            });
        }, 500);
        */

        $(document).keypress(function(e) {
            if (e.which == 13) { // enter
                // Trigger regular Flashing sequence
                $('a.flash_firmware').click();
            }
        });

        // back button
        $('a.back').click(function() {
            if (!GUI.connect_lock) { // button disabled while flashing is in progress
                GUI.tab_switch_cleanup(function() {
                    tab_initialize_default();
                });
            } else {
                GUI.log(chrome.i18n.getMessage('firmwareFlasherWaitForFinish'));
            }
        });
    });
}

function parse_hex(str, callback) {
    // parsing hex in different thread
    var worker = new Worker('./js/workers/hex_parser.js');

    // "callback"
    worker.onmessage = function (event) {
        callback(event.data);
    };

    // send data/string over for processing
    worker.postMessage(str);
}