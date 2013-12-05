function tab_initialize_firmware_flasher() {
    ga_tracker.sendAppView('Firmware Flasher');
    GUI.active_tab = 'firmware_flasher';
    
    var intel_hex = false; // standard intel hex in string format
    var parsed_hex = false; // parsed raw hex in array format
    
    $('#content').load("./tabs/firmware_flasher.html", function() {
        // UI Hooks
        $('a.load_file').click(function() {
            chrome.fileSystem.chooseEntry({type: 'openFile', accepts: [{extensions: ['hex']}]}, function(fileEntry) {
                if (!fileEntry) {
                    // no "valid" file selected/created, aborting
                    console.log('No valid file selected, aborting');
                    return;
                }
                
                chrome.fileSystem.getDisplayPath(fileEntry, function(path) {
                    console.log('Loading file from: ' + path);
                    $('span.path').html(path);
                    
                    fileEntry.file(function(file) {
                        var reader = new FileReader();

                        reader.onerror = function (e) {
                            console.error(e);
                        };
                        
                        reader.onloadend = function(e) {
                            console.log('File loaded');
                            
                            intel_hex = e.target.result;
                            
                            parse_hex(intel_hex, function(data) {
                                parsed_hex = data;
                                
                                if (parsed_hex) {
                                    STM32.GUI_status('<span style="color: green">Firmware loaded, ready for flashing</span>');
                                    $('a.flash_firmware').removeClass('locked');
                                    
                                    $('span.size').html(parsed_hex.bytes + ' bytes');
                                } else {
                                    STM32.GUI_status('<span style="color: red">HEX file appears to be corrupted</span>');
                                }
                            });
                        };

                        reader.readAsText(file);
                    });
                });
            });
        });
        
        $('a.load_remote_file').click(function() {
            /*  for future use
                $.get('https://api.github.com/repos/multiwii/baseflight/tags', function(data) {
                    console.log(data)
                });
            */
            
            $.get('https://raw.github.com/multiwii/baseflight/master/obj/baseflight.hex', function(data) {
                intel_hex = data;
                
                parse_hex(intel_hex, function(data) {
                    parsed_hex = data;
                    
                    if (parsed_hex) {
                        STM32.GUI_status('<span style="color: green">Remote Firmware loaded, ready for flashing</span>');
                        $('a.flash_firmware').removeClass('locked');
                        
                        $('span.path').html('Using remote Firmware');
                        $('span.size').html(parsed_hex.bytes + ' bytes');
                    } else {
                        STM32.GUI_status('<span style="color: red">HEX file appears to be corrupted</span>');
                    }
                });
            }).fail(function() {
                STM32.GUI_status('<span style="color: red">Failed to load remote firmware</span>');
                $('a.flash_firmware').addClass('locked');
            });
        });
        
        $('a.flash_firmware').click(function() {
            if (!$(this).hasClass('locked')) {
                if (!GUI.connect_lock) { // button disabled while flashing is in progress
                    if (parsed_hex != false) {
                        STM32.hex = parsed_hex;
                        
                        STM32.connect();
                    } else {
                        STM32.GUI_status('<span style="color: red">Firmware not loaded</span>');
                    }
                }
            }
        });
        
        $('a.back').click(function() {
            if (!GUI.connect_lock) { // button disabled while flashing is in progress                
                tab_initialize_default();
            } else {
                notify('You <span style="color: red">can\'t</span> do this right now, please wait for current operation to finish ...');
            }
        });
    });
}

function parse_hex(str, callback) {
    // parsing hex in different thread
    var worker = new Worker('./workers/hex_parser.js');
    
    // "callback"
    worker.onmessage = function (event) {
        callback(event.data);
    };
    
    // send data/string over for processing
    worker.postMessage(str);
}