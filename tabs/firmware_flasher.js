'use strict';

TABS.firmware_flasher = {};
TABS.firmware_flasher.initialize = function (callback) {
    GUI.active_tab_ref = this;
    GUI.active_tab = 'firmware_flasher';
    googleAnalytics.sendAppView('Firmware Flasher');

    var intel_hex = false; // standard intel hex in string format
    var parsed_hex = false; // parsed raw hex in array format

    $('#content').load("./tabs/firmware_flasher.html", function () {
        // translate to user-selected language
        localize();

        // UI Hooks
        $('a.load_file').click(function () {
            chrome.fileSystem.chooseEntry({type: 'openFile', accepts: [{extensions: ['hex']}]}, function (fileEntry) {
                if (!fileEntry) {
                    // no "valid" file selected/created, aborting
                    console.log('No valid file selected, aborting');
                    return;
                }

                // hide github info (if it exists)
                $('div.git_info').slideUp();

                chrome.fileSystem.getDisplayPath(fileEntry, function (path) {
                    console.log('Loading file from: ' + path);

                    fileEntry.file(function (file) {
                        var reader = new FileReader();

                        reader.onprogress = function (e) {
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

                                parse_hex(intel_hex, function (data) {
                                    parsed_hex = data;

                                    if (parsed_hex) {
                                        googleAnalytics.sendEvent('Flashing', 'Firmware', 'local');
                                        $('a.flash_firmware').removeClass('locked');

                                        $('span.progressLabel').text('Loaded Local Firmware: (' + parsed_hex.bytes_total + ' bytes)');
                                    } else {
                                        $('span.progressLabel').text(chrome.i18n.getMessage('firmwareFlasherHexCorrupted'));
                                    }
                                });
                            }
                        };

                        reader.readAsText(file);
                    });
                });
            });
        });

        $('a.load_remote_file').click(function () {
            $.get('https://raw.githubusercontent.com/multiwii/baseflight/master/obj/baseflight.hex', function (data) {
                intel_hex = data;

                parse_hex(intel_hex, function (data) {
                    parsed_hex = data;

                    if (parsed_hex) {
                        googleAnalytics.sendEvent('Flashing', 'Firmware', 'online');
                        $('a.flash_firmware').removeClass('locked');

                        $('span.progressLabel').text('Loaded Online Firmware: (' + parsed_hex.bytes_total + ' bytes)');
                    } else {
                        $('span.progressLabel').text(chrome.i18n.getMessage('firmwareFlasherHexCorrupted'));
                    }
                });
            }).fail(function () {
                $('span.progressLabel').text(chrome.i18n.getMessage('firmwareFlasherFailedToLoadOnlineFirmware'));
                $('a.flash_firmware').addClass('locked');
            });

            $.get('https://api.github.com/repos/multiwii/baseflight/commits?page=1&per_page=1&path=obj/baseflight.hex', function (data) {
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

        $('a.flash_firmware').click(function () {
            if (!$(this).hasClass('locked')) {
                if (!GUI.connect_lock) { // button disabled while flashing is in progress
                    if (parsed_hex != false) {
                        if (String($('div#port-picker #port').val()) != 'DFU') {
                            if (String($('div#port-picker #port').val()) != '0') {
                                var options = {},
                                    port = String($('div#port-picker #port').val()),
                                    baud;

                                switch (GUI.operating_system) {
                                    case 'Windows':
                                    case 'MacOS':
                                    case 'ChromeOS':
                                    case 'Linux':
                                    case 'UNIX':
                                        baud = 921600;
                                        break;

                                    default:
                                        baud = 115200;
                                }

                                if ($('input.updating').is(':checked')) {
                                    options.no_reboot = true;
                                } else {
                                    options.reboot_baud = parseInt($('div#port-picker #baud').val());
                                }

                                if ($('input.erase_chip').is(':checked')) {
                                    options.erase_chip = true;
                                }

                                if ($('input.flash_slowly').is(':checked')) {
                                    options.flash_slowly = true;
                                }

                                STM32.connect(port, baud, parsed_hex, options);
                            } else {
                                console.log('Please select valid serial port');
                                GUI.log('<span style="color: red">Please select valid serial port</span>');
                            }
                        } else {
                            STM32DFU.connect(usbDevices.STM32DFU, parsed_hex);
                        }
                    } else {
                        $('span.progressLabel').text(chrome.i18n.getMessage('firmwareFlasherFirmwareNotLoaded'));
                    }
                }
            }
        });

        chrome.storage.local.get('no_reboot_sequence', function (result) {
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

        chrome.storage.local.get('flash_on_connect', function (result) {
            if (result.flash_on_connect) {
                $('input.flash_on_connect').prop('checked', true);
            } else {
                $('input.flash_on_connect').prop('checked', false);
            }

            $('input.flash_on_connect').change(function () {
                var status = $(this).is(':checked');

                if (status) {
                    var flashing_port;

                    var start = function () {
                        PortHandler.port_detected('flash_next_device', function (result) {
                            flashing_port = result[0];
                            GUI.log('Detected: <strong>' + flashing_port + '</strong> - triggering flash on connect');
                            console.log('Detected: ' + flashing_port + ' - triggering flash on connect');

                            // Trigger regular Flashing sequence
                            $('a.flash_firmware').click();

                            // Detect port removal to create a new callback
                            end();
                        }, false, true);
                    }

                    var end = function () {
                        PortHandler.port_removed('flashed_device_removed', function (result) {
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

        chrome.storage.local.get('erase_chip', function (result) {
            if (result.erase_chip) {
                $('input.erase_chip').prop('checked', true);
            } else {
                $('input.erase_chip').prop('checked', false);
            }

            // bind UI hook so the status is saved on change
            $('input.erase_chip').change(function () {
                chrome.storage.local.set({'erase_chip': $(this).is(':checked')});
            });
        });

        chrome.storage.local.get('flash_slowly', function (result) {
            if (result.flash_slowly) {
                $('input.flash_slowly').prop('checked', true);
            } else {
                $('input.flash_slowly').prop('checked', false);
            }

            // bind UI hook so the status is saved on change
            $('input.flash_slowly').change(function () {
                chrome.storage.local.set({'flash_slowly': $(this).is(':checked')});
            });
        });

        $(document).keypress(function (e) {
            if (e.which == 13) { // enter
                // Trigger regular Flashing sequence
                $('a.flash_firmware').click();
            }
        });

        // back button
        $('a.back').click(function () {
            if (!GUI.connect_lock) { // button disabled while flashing is in progress
                GUI.tab_switch_cleanup(function () {
                    TABS.default.initialize();
                });
            } else {
                GUI.log(chrome.i18n.getMessage('firmwareFlasherWaitForFinish'));
            }
        });

        if (callback) callback();
    });
};

TABS.firmware_flasher.cleanup = function (callback) {
    PortHandler.flush_callbacks();

    // unbind "global" events
    $(document).unbind('keypress');

    if (callback) callback();
};

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